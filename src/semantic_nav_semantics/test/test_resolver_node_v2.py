import os

import pytest

from semantic_nav_semantics.caption_ranker import BM25CaptionRanker
from semantic_nav_semantics.semantic_store import load_semantic_store
from semantic_nav_semantics.standoff_planner import StandoffPlanner

_DIR = os.path.dirname(__file__)
REAL_MAP = os.path.join(_DIR, "..", "config", "map_v001.json")
REAL_SIDECAR = os.path.join(_DIR, "..", "config", "object_intent_affordances.json")

pytestmark = pytest.mark.skipif(
    not os.path.exists(REAL_MAP), reason="map_v001 missing"
)


@pytest.fixture
def stack():
    store = load_semantic_store(REAL_MAP, affordances_path=REAL_SIDECAR)
    ranker = BM25CaptionRanker(affordances=store.affordances)
    planner = StandoffPlanner()
    return store, ranker, planner


def test_resolves_refrigerator_to_fridge_row(stack):
    store, ranker, planner = stack
    rows = store.rows_for_tag("refrigerator")
    ranked = ranker.rank(rows, "food storage and eating", robot_xy=(0.0, 0.0))
    assert ranked[0].row.object_key == "refrigerator:6"
    pose = planner.plan(ranked[0].row, robot_xy=(0.0, 0.0))
    assert -10.0 < pose.position_xy[0] < 10.0
    assert -10.0 < pose.position_xy[1] < 10.0


def test_resolves_chair_prefers_dining_caption(stack):
    store, ranker, _ = stack
    rows = store.rows_for_tag("chair")
    ranked = ranker.rank(rows, "dining or kitchen seating", robot_xy=(0.0, 0.0))
    top_caption = ranked[0].row.object_caption.lower()
    assert any(w in top_caption for w in ("dining", "kitchen", "wooden"))


def test_picture_tag_is_unreachable_via_navigable_vocabulary(stack):
    store, _, _ = stack
    assert "picture" in store.tag_vocabulary
    assert "picture" not in store.navigable_tag_vocabulary
