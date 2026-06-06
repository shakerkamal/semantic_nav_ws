"""BM25-based caption ranker for object-centric semantic navigation.

Provides:
  - RankedObject  — frozen dataclass (10 fields) returned by all ranker implementations
  - CaptionRanker — structural Protocol that all ranker implementations must satisfy
  - BM25CaptionRanker — concrete BM25 implementation

Future ranker variants (LLMCaptionRanker, HybridCaptionRanker) must satisfy the
same CaptionRanker Protocol without subclassing this module.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import List, Optional, Protocol, Sequence, Tuple, runtime_checkable

from semantic_nav_semantics.semantic_store import ObjectRow


# ---------------------------------------------------------------------------
# RankedObject — shared output type for all ranker implementations
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class RankedObject:
    """Scored object instance produced by a CaptionRanker."""

    row: ObjectRow
    score: float
    lexical_score: float
    affordance_score: float
    caption_tag_bonus: float
    caption_boost_bonus: float
    volume_bonus: float
    distance_bonus: float
    conflict_penalty: float
    reasons: Tuple[str, ...]


# ---------------------------------------------------------------------------
# CaptionRanker — structural Protocol
# ---------------------------------------------------------------------------

@runtime_checkable
class CaptionRanker(Protocol):
    """Uniform ranker interface. Implementations must be deterministic."""

    def rank(
        self,
        candidates: Sequence[ObjectRow],
        intent_hint: str,
        robot_xy: Optional[Tuple[float, float]] = None,
        user_command: str = "",
    ) -> List[RankedObject]:
        ...


# ---------------------------------------------------------------------------
# BM25CaptionRanker — concrete implementation
# ---------------------------------------------------------------------------

class BM25CaptionRanker:
    """Tag-gated BM25 caption ranker. Caller must hydrate candidate ObjectRows.

    Scoring components (all additive):
      lexical_score      — BM25 score of caption against intent_hint tokens
      caption_tag_bonus  — bonus if the object's own tag appears in its caption
      caption_boost_bonus — bonus for each sidecar caption_boost_term found in caption
      affordance_score   — bonus for each sidecar query_hint matched in intent_hint
      volume_bonus       — gentle log-volume bonus (larger objects slightly preferred)
      distance_bonus     — closer-to-robot bonus (0 when robot_xy unknown)
      conflict_penalty   — negative when caption lacks own tag but mentions
                           a *different* navigable tag (likely mis-tagged object)

    Sort order: score descending, then object_key ascending (deterministic tie-break).
    """

    def __init__(
        self,
        affordances=None,           # Optional[ObjectIntentAffordances]
        navigable_tags=None,        # Optional[frozenset[str]] — for conflict penalty
        conflict_penalty: float = -0.75,
        caption_tag_bonus: float = 0.5,
        caption_boost_bonus: float = 0.3,
        affordance_bonus: float = 0.4,
        volume_weight: float = 0.05,
        distance_weight: float = 0.05,
    ) -> None:
        self._aff = affordances
        self._navigable_tags = navigable_tags  # frozenset of navigable tag names
        self._conflict_penalty = float(conflict_penalty)
        self._caption_tag_bonus = float(caption_tag_bonus)
        self._caption_boost_bonus = float(caption_boost_bonus)
        self._affordance_bonus = float(affordance_bonus)
        self._volume_weight = float(volume_weight)
        self._distance_weight = float(distance_weight)

    def rank(
        self,
        candidates: Sequence[ObjectRow],
        intent_hint: str,
        robot_xy: Optional[Tuple[float, float]] = None,
        user_command: str = "",
    ) -> List[RankedObject]:
        """Rank candidates by relevance to *intent_hint*.

        Parameters
        ----------
        candidates:
            ObjectRow instances — all expected to share the same normalized_tag
            (tag-gating is the caller's responsibility).
        intent_hint:
            Free-text hint extracted from the user command (e.g. "dining seating").
            Empty string disables lexical scoring and returns stable key-order.
        robot_xy:
            Optional (x, y) robot position in map frame for proximity scoring.
        user_command:
            Full user command string; accepted for Protocol compatibility but not
            currently used beyond what intent_hint already captures.

        Returns
        -------
        List of RankedObject, sorted by score descending, object_key ascending.
        """
        if not candidates:
            return []

        from semantic_nav_semantics.bm25 import BM25

        captions = [c.object_caption for c in candidates]
        bm25 = BM25(captions)
        lexical = bm25.scores(intent_hint or "")

        # Build navigable tag set for conflict detection.
        # Explicit navigable_tags kwarg takes priority over sidecar.
        navigable: set = set()
        if self._navigable_tags is not None:
            navigable = set(self._navigable_tags)
        elif self._aff is not None:
            navigable = {t for t, m in self._aff.by_tag.items() if m.navigable}

        ranked: List[RankedObject] = []
        for cand, lex in zip(candidates, lexical):
            caption_low = (cand.object_caption or "").lower()
            tag = cand.normalized_tag

            # --- caption-mentions-tag bonus -----------------------------------
            caption_has_tag = tag in caption_low
            ctb = self._caption_tag_bonus if caption_has_tag else 0.0

            # --- affordance / caption-boost bonuses (sidecar driven) ----------
            boost_terms: Tuple[str, ...] = ()
            hints: Tuple[str, ...] = ()
            if self._aff is not None:
                meta = self._aff.metadata_for_tag(tag)
                boost_terms = meta.caption_boost_terms
                hints = meta.query_hints

            cbb = sum(
                self._caption_boost_bonus
                for t in boost_terms
                if t.lower() in caption_low
            )
            # Token-level match (not substring) to avoid "sit" firing on "exquisite"
            hint_tokens = set((intent_hint or "").lower().split())
            ab = sum(
                self._affordance_bonus
                for h in hints
                if h.lower() in hint_tokens
            )

            # --- volume bonus (gentle): log of volume, clipped ---------------
            vol = max(cand.bbox_volume, 0.0)
            vb = self._volume_weight * math.log1p(vol)

            # --- distance bonus (closer = larger), 0 if robot_xy unknown -----
            db = 0.0
            if robot_xy is not None:
                dx = cand.bbox_center[0] - robot_xy[0]
                dy = cand.bbox_center[1] - robot_xy[1]
                d = math.sqrt(dx * dx + dy * dy)
                db = self._distance_weight * (1.0 / (1.0 + d))

            # --- tag/caption conflict penalty ---------------------------------
            # Fires when the caption lacks the object's own tag AND explicitly
            # mentions a *different* known navigable tag.  This catches objects
            # whose captions describe a nearby / enclosing object (e.g. a small
            # box described as "on the cabinet") so they rank below cleaner hits.
            penalty = 0.0
            if not caption_has_tag and navigable:
                for other in navigable:
                    if other == tag:
                        continue
                    if other and other in caption_low:
                        penalty = self._conflict_penalty
                        break

            score = lex + ctb + cbb + ab + vb + db + penalty
            reasons = self._build_reasons(lex, ctb, cbb, ab, vb, db, penalty)
            ranked.append(
                RankedObject(
                    row=cand,
                    score=score,
                    lexical_score=lex,
                    affordance_score=ab,
                    caption_tag_bonus=ctb,
                    caption_boost_bonus=cbb,
                    volume_bonus=vb,
                    distance_bonus=db,
                    conflict_penalty=penalty,
                    reasons=reasons,
                )
            )

        # Stable, deterministic sort: score desc, then object_key asc
        ranked.sort(key=lambda r: (-r.score, r.row.object_key))
        return ranked

    @staticmethod
    def _build_reasons(
        lex: float,
        ctb: float,
        cbb: float,
        ab: float,
        vb: float,
        db: float,
        pen: float,
    ) -> Tuple[str, ...]:
        out: List[str] = []
        if lex > 0:
            out.append(f"bm25={lex:.2f}")
        if ctb > 0:
            out.append("caption_contains_tag")
        if cbb > 0:
            out.append(f"caption_boost={cbb:.2f}")
        if ab > 0:
            out.append(f"affordance={ab:.2f}")
        if vb > 0:
            out.append(f"volume={vb:.2f}")
        if db > 0:
            out.append(f"distance={db:.2f}")
        if pen < 0:
            out.append(f"conflict={pen:.2f}")
        return tuple(out)
