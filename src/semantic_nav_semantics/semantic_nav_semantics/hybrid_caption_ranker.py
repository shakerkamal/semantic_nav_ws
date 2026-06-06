from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

from semantic_nav_semantics.caption_ranker import RankedObject
from semantic_nav_semantics.semantic_store import ObjectRow


@dataclass
class HybridCaptionRanker:
    bm25: object    # BM25CaptionRanker or duck-type
    llm: object     # LLMCaptionRanker or duck-type
    delta: float = 0.5
    top_k: int = 4

    def rank(
        self,
        candidates: Sequence[ObjectRow],
        intent_hint: str,
        robot_xy: Optional[Tuple[float, float]] = None,
        user_command: str = "",
    ) -> List[RankedObject]:
        bm_ranked = self.bm25.rank(candidates, intent_hint, robot_xy=robot_xy)
        if len(bm_ranked) <= 1:
            return bm_ranked

        margin = bm_ranked[0].score - bm_ranked[1].score
        if margin >= self.delta:
            # Clear winner: tag and return BM25 result.
            head = bm_ranked[0]
            new_head = RankedObject(
                row=head.row, score=head.score,
                lexical_score=head.lexical_score, affordance_score=head.affordance_score,
                caption_tag_bonus=head.caption_tag_bonus,
                caption_boost_bonus=head.caption_boost_bonus,
                volume_bonus=head.volume_bonus, distance_bonus=head.distance_bonus,
                conflict_penalty=head.conflict_penalty,
                reasons=(*head.reasons, f"hybrid_clear_margin={margin:.2f}"),
            )
            return [new_head] + bm_ranked[1:]

        # Close call: hand top-K candidates to LLM.
        top = [r.row for r in bm_ranked[: self.top_k]]
        llm_ranked = self.llm.rank(
            top, intent_hint=intent_hint, robot_xy=robot_xy,
            user_command=user_command,
        )
        if not llm_ranked:
            return bm_ranked
        chosen = llm_ranked[0]
        annotated = RankedObject(
            row=chosen.row, score=chosen.score,
            lexical_score=chosen.lexical_score, affordance_score=chosen.affordance_score,
            caption_tag_bonus=chosen.caption_tag_bonus,
            caption_boost_bonus=chosen.caption_boost_bonus,
            volume_bonus=chosen.volume_bonus, distance_bonus=chosen.distance_bonus,
            conflict_penalty=chosen.conflict_penalty,
            reasons=(*chosen.reasons, f"hybrid_close_margin={margin:.2f}"),
        )
        rest = [r for r in bm_ranked if r.row.object_key != chosen.row.object_key]
        return [annotated] + rest
