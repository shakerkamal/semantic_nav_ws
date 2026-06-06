"""Unit tests for semantic_nav_semantics.caption_ranker.

Run from the package directory with ROS 2 sourced:

    source /opt/ros/humble/setup.bash
    cd src/semantic_nav_semantics
    python3 -m pytest test/test_caption_ranker.py -v
"""

import json
import math
import pytest
from semantic_nav_semantics.semantic_store import ObjectRow, load_object_intent_affordances
from semantic_nav_semantics.caption_ranker import BM25CaptionRanker, CaptionRanker, RankedObject


def row(key, tag, caption, vol=0.5, center=(0.0, 0.0, 0.0)):
    return ObjectRow(
        source_key=f"object_{key.split(':')[1]}",
        object_key=key, object_id=int(key.split(':')[1]),
        object_tag=tag, normalized_tag=tag,
        object_caption=caption, object_state="movable",
        bbox_center=center, bbox_extent=(0.5, 0.5, 0.5), bbox_volume=vol,
    )


@pytest.fixture
def sidecar(tmp_path):
    p = tmp_path / "s.json"
    p.write_text(json.dumps({
        "defaults": {"navigable": True},
        "by_tag": {
            "chair":   {"navigable": True, "query_hints": ["sit", "dining"]},
            "cabinet": {"navigable": True, "query_hints": ["storage"]},
        },
    }))
    return load_object_intent_affordances(str(p))


def test_ranked_object_has_10_fields(sidecar):
    rows = [row("chair:2", "chair", "padded wooden chair suitable for dining")]
    ranker = BM25CaptionRanker(affordances=sidecar)
    ranked = ranker.rank(rows, intent_hint="dining")
    r = ranked[0]
    assert isinstance(r, RankedObject)
    # 10 required fields
    _ = (r.row, r.score, r.lexical_score, r.affordance_score,
         r.caption_tag_bonus, r.caption_boost_bonus,
         r.volume_bonus, r.distance_bonus, r.conflict_penalty, r.reasons)


def test_ranks_dining_chair_above_office_chair(sidecar):
    rows = [
        row("chair:2",  "chair", "padded wooden chair suitable for dining or kitchen use"),
        row("chair:4",  "chair", "dark upholstered office chair with wheels"),
        row("chair:39", "chair", "blue patterned folding chair against the wall"),
    ]
    ranker = BM25CaptionRanker(affordances=sidecar)
    ranked = ranker.rank(rows, intent_hint="dining or kitchen seating")
    assert isinstance(ranked[0], RankedObject)
    assert ranked[0].row.object_key == "chair:2"
    assert ranked[0].score >= ranked[1].score >= ranked[2].score


def test_empty_hint_returns_stable_order(sidecar):
    rows = [row("chair:2", "chair", "x"), row("chair:1", "chair", "y")]
    ranker = BM25CaptionRanker(affordances=sidecar)
    ranked = ranker.rank(rows, intent_hint="")
    keys = [r.row.object_key for r in ranked]
    assert keys == sorted(keys)   # deterministic fallback: sort by object_key asc
    # Lexical component is zero; all non-lexical bonuses are equal across rows,
    # so scores are identical and tie-break is purely by object_key.
    assert ranked[0].score == ranked[1].score
    assert ranked[0].lexical_score == 0.0
    assert ranked[1].lexical_score == 0.0


def test_noisy_chair_with_cigar_box_caption_ranks_below_dining_chair(sidecar):
    # chair:292 mentions "cabinet" (different navigable tag) -> conflict penalty
    rows = [
        row("chair:2", "chair", "padded wooden chair suitable for dining or kitchen use"),
        row("chair:292", "chair",
            "small rectangular cigar box on the side cabinet near a coffeepot"),
    ]
    ranker = BM25CaptionRanker(
        affordances=sidecar,
        navigable_tags=frozenset({"chair", "cabinet"}),  # cabinet is a known navigable tag
    )
    ranked = ranker.rank(rows, intent_hint="dining seating")
    assert ranked[0].row.object_key == "chair:2"
    assert ranked[1].row.object_key == "chair:292"
    assert ranked[1].conflict_penalty < 0    # explicitly penalised


def test_bm25_caption_ranker_satisfies_protocol():
    assert isinstance(BM25CaptionRanker(), CaptionRanker)


def test_empty_caption_does_not_crash(sidecar):
    rows = [row("chair:1", "chair", "")]
    ranker = BM25CaptionRanker(affordances=sidecar)
    ranked = ranker.rank(rows, intent_hint="dining")
    assert len(ranked) == 1
    assert math.isfinite(ranked[0].score)
