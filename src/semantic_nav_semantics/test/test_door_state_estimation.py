# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Unit tests for door-state estimation — no ROS runtime required."""

from semantic_nav_semantics.door_state_estimation import (
    DoorFootprint,
    GridView,
    classify_door_state,
    load_door_footprints,
    occupied_fraction,
)


def _grid(cells):
    # 4x4 grid, 1.0 m resolution, origin at (0, 0). `cells` is row-major length 16.
    return GridView(
        resolution=1.0, width=4, height=4,
        origin_x=0.0, origin_y=0.0, data=cells,
    )


def _footprint():
    # Centered at (1.5, 1.5), extent (1.0, 1.0) -> samples cells i in {1,2}, j in {1,2}
    # i.e. flat indices 5, 6, 9, 10.
    return DoorFootprint("door:119", 1.5, 1.5, 1.0, 1.0)


def _cells_with(indices_to_value):
    cells = [0] * 16
    for idx, val in indices_to_value.items():
        cells[idx] = val
    return cells


def test_occupied_fraction_all_occupied():
    cells = _cells_with({5: 100, 6: 100, 9: 100, 10: 100})
    frac, observed = occupied_fraction(_grid(cells), _footprint(), lethal_threshold=90)
    assert observed == 4
    assert frac == 1.0


def test_occupied_fraction_all_free():
    frac, observed = occupied_fraction(_grid([0] * 16), _footprint(), lethal_threshold=90)
    assert observed == 4
    assert frac == 0.0


def test_occupied_fraction_half():
    cells = _cells_with({5: 100, 6: 100})  # 9, 10 stay free
    frac, observed = occupied_fraction(_grid(cells), _footprint(), lethal_threshold=90)
    assert observed == 4
    assert frac == 0.5


def test_occupied_fraction_unknown_cells_skipped():
    cells = _cells_with({5: -1, 6: -1, 9: -1, 10: -1})
    frac, observed = occupied_fraction(_grid(cells), _footprint(), lethal_threshold=90)
    assert observed == 0
    assert frac == 0.0


def test_occupied_fraction_zero_resolution_is_safe():
    grid = GridView(0.0, 4, 4, 0.0, 0.0, [0] * 16)
    frac, observed = occupied_fraction(grid, _footprint(), lethal_threshold=90)
    assert (frac, observed) == (0.0, 0)


def test_classify_closed():
    est = classify_door_state(
        1.0, 4, blocked_fraction=0.30, open_fraction=0.10,
        min_observed_cells=3, object_key="door:119",
    )
    assert est.door_state == "closed"
    assert est.traversability == "blocked"
    assert est.confidence > 0.0


def test_classify_open():
    est = classify_door_state(
        0.0, 4, blocked_fraction=0.30, open_fraction=0.10,
        min_observed_cells=3, object_key="door:119",
    )
    assert est.door_state == "open"
    assert est.traversability == "passable"


def test_classify_ambiguous_band_is_unknown():
    est = classify_door_state(
        0.20, 4, blocked_fraction=0.30, open_fraction=0.10,
        min_observed_cells=3, object_key="door:119",
    )
    assert est.door_state == "unknown"
    assert est.traversability == "unknown"


def test_classify_insufficient_evidence_is_unknown():
    est = classify_door_state(
        1.0, 2, blocked_fraction=0.30, open_fraction=0.10,
        min_observed_cells=3, object_key="door:119",
    )
    assert est.door_state == "unknown"
    assert est.confidence == 0.0


def test_load_door_footprints_selects_doors_only():
    map_objects = {
        "object_1": {
            "id": 1, "object_tag": "lamp",
            "bbox_center": [0.0, 0.0, 0.0], "bbox_extent": [1.0, 1.0, 1.0],
        },
        "object_119": {
            "id": 119, "object_tag": "door",
            "bbox_center": [4.12, -1.83, 1.0], "bbox_extent": [0.85, 0.12, 2.0],
        },
    }
    footprints = load_door_footprints(map_objects)
    assert len(footprints) == 1
    fp = footprints[0]
    assert fp.object_key == "door:119"
    assert fp.center_x == 4.12
    assert fp.center_y == -1.83
    assert fp.extent_x == 0.85
    assert fp.extent_y == 0.12


def test_load_door_footprints_matches_compound_door_tags():
    map_objects = {
        "object_5": {
            "id": 5, "object_tag": "closet door",
            "bbox_center": [1.0, 2.0, 1.0], "bbox_extent": [0.6, 0.1, 2.0],
        },
    }
    footprints = load_door_footprints(map_objects)
    assert len(footprints) == 1
    assert footprints[0].object_key.endswith(":5")
