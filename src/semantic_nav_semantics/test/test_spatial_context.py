import os

import pytest

from semantic_nav_semantics.semantic_store import ObjectRow
from semantic_nav_semantics.spatial_context import SpatialContextBuilder


def row(key, tag, center):
    return ObjectRow(
        source_key=f"object_{key.split(':')[1]}",
        object_key=key, object_id=int(key.split(':')[1]),
        object_tag=tag, normalized_tag=tag,
        object_caption="x", object_state="movable",
        bbox_center=center, bbox_extent=(0.5, 0.5, 0.5), bbox_volume=0.125,
    )


def navigable_set():
    return {"chair", "refrigerator", "desk", "cabinet"}


def test_summarises_neighbours_within_radius():
    target = row("chair:2", "chair", (5.0, 0.0, 0.0))
    others = [
        row("refrigerator:9", "refrigerator", (5.5, 0.0, 0.0)),   # 0.5 m away
        row("desk:12",        "desk",         (6.0, 0.0, 0.0)),   # 1.0 m away
        row("cabinet:24",     "cabinet",      (50.0, 0.0, 0.0)),  # far
    ]
    sb = SpatialContextBuilder(neighbour_radius_m=2.0, max_neighbours=3)
    summary = sb.build(target, others, robot_xy=(0.0, 0.0), navigable_tags=navigable_set())

    assert "refrigerator:9" in summary
    assert "desk:12" in summary
    assert "cabinet:24" not in summary
    assert "Robot distance" in summary
    assert summary.index("refrigerator:9") < summary.index("desk:12")


def test_no_neighbours_within_radius():
    target = row("chair:2", "chair", (0.0, 0.0, 0.0))
    others = [row("desk:12", "desk", (10.0, 0.0, 0.0))]
    sb = SpatialContextBuilder(neighbour_radius_m=2.0)
    summary = sb.build(target, others, robot_xy=(0.0, 0.0), navigable_tags=navigable_set())
    assert "(none within 2.0 m)" in summary
    assert "Robot distance: 0.0 m" in summary


def test_skips_non_navigable_neighbours():
    target = row("chair:2", "chair", (0.0, 0.0, 0.0))
    others = [
        row("picture:7", "picture", (0.1, 0.0, 0.0)),
        row("desk:12",   "desk",    (1.5, 0.0, 0.0)),
    ]
    sb = SpatialContextBuilder()
    summary = sb.build(target, others, robot_xy=(0.0, 0.0), navigable_tags=navigable_set())
    assert "picture:7" not in summary
    assert "desk:12" in summary


def test_excludes_target_from_its_own_neighbours():
    target = row("chair:2", "chair", (0.0, 0.0, 0.0))
    others = [target]
    sb = SpatialContextBuilder()
    summary = sb.build(target, others, robot_xy=(0.0, 0.0), navigable_tags=navigable_set())
    assert "chair:2" not in summary or "Near: (none within" in summary


def test_caps_neighbours_at_max():
    target = row("chair:2", "chair", (0.0, 0.0, 0.0))
    others = [row(f"desk:{i}", "desk", (float(i) * 0.1, 0.0, 0.0)) for i in range(1, 8)]
    sb = SpatialContextBuilder(neighbour_radius_m=2.0, max_neighbours=3)
    summary = sb.build(target, others, robot_xy=(0.0, 0.0), navigable_tags=navigable_set())
    listed = sum(summary.count(f"desk:{i}") for i in range(1, 8))
    assert listed == 3


# --- real-map smoke (Task LR.2) ---

_DIR = os.path.dirname(__file__)
REAL_MAP = os.path.join(_DIR, "..", "config", "map_v001.json")
REAL_SIDECAR = os.path.join(_DIR, "..", "config", "object_intent_affordances.json")


@pytest.mark.skipif(not os.path.exists(REAL_MAP), reason="map_v001.json not present")
def test_real_map_refrigerator_summary():
    from semantic_nav_semantics.semantic_store import load_semantic_store
    store = load_semantic_store(REAL_MAP, affordances_path=REAL_SIDECAR)
    all_rows = list(store.by_object_key.values())
    fridge = store.by_object_key["refrigerator:6"]

    sb = SpatialContextBuilder(neighbour_radius_m=2.5, max_neighbours=4)
    summary = sb.build(
        fridge, all_rows, robot_xy=(0.0, 0.0),
        navigable_tags=set(store.navigable_tag_vocabulary),
    )

    assert ":" in summary
    assert "Robot distance" in summary
