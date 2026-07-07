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
    # occupied_fraction returns (blocked_width_fraction, observed_LINES). The
    # footprint spans width-columns i in {1,2}; every line has a lethal cell.
    cells = _cells_with({5: 100, 6: 100, 9: 100, 10: 100})
    frac, observed = occupied_fraction(_grid(cells), _footprint(), lethal_threshold=90)
    assert observed == 2  # two width-lines
    assert frac == 1.0


def test_occupied_fraction_all_free():
    frac, observed = occupied_fraction(_grid([0] * 16), _footprint(), lethal_threshold=90)
    assert observed == 2  # two width-lines, both observed, none blocked
    assert frac == 0.0


def test_occupied_fraction_half():
    # Block one full width-column (i=1: flat indices 5 and 9); the other (i=2)
    # stays free -> one of two width-lines blocked -> 0.5.
    cells = _cells_with({5: 100, 9: 100})
    frac, observed = occupied_fraction(_grid(cells), _footprint(), lethal_threshold=90)
    assert observed == 2
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


def _doorway_grid():
    # 3 wide x 9 tall, 0.1 m res, origin (0,0). Door-post rows at j=0 and j=8
    # are occupied (like the frame posts that clip door:119); the middle rows
    # (the clear opening) are free.
    cells = [0] * 27
    for i in range(3):        # bottom post row j=0
        cells[0 * 3 + i] = 100
    for i in range(3):        # top post row j=8
        cells[8 * 3 + i] = 100
    return GridView(resolution=0.1, width=3, height=9,
                    origin_x=0.0, origin_y=0.0, data=cells)


def _doorway_footprint():
    # Full door bbox: center (0.15, 0.45), extent (0.3, 0.9) -> spans all 27 cells,
    # so it clips both post rows (mirrors the real 0.9 m box over a 0.7 m gap).
    return DoorFootprint("door:119", 0.15, 0.45, 0.3, 0.9)


def test_occupied_fraction_no_margin_clips_posts():
    # Without a margin the full bbox catches both post rows (width-lines j=0 and
    # j=8) -> 2 of 9 width-lines blocked -> falsely elevated occupancy.
    frac, observed = occupied_fraction(
        _doorway_grid(), _doorway_footprint(), lethal_threshold=100
    )
    assert observed == 9  # nine width-lines (Y is the wider axis)
    assert frac > 0.20  # 2 post lines / 9 == 0.22


def test_occupied_fraction_margin_excludes_posts():
    # A 0.25 fractional inset samples only the clear opening -> no false occupancy.
    frac, observed = occupied_fraction(
        _doorway_grid(), _doorway_footprint(), lethal_threshold=100, margin_frac=0.25
    )
    assert frac == 0.0
    assert observed > 0  # still sampling the free center rows


def test_occupied_fraction_margin_never_inverts_footprint():
    # A fraction >= 1 must not produce an empty/expanded box (clamped to >=1 cell).
    frac, observed = occupied_fraction(
        _doorway_grid(), _doorway_footprint(), lethal_threshold=100, margin_frac=1.5
    )
    assert observed > 0


def _one_sided_closed_grid():
    # Models a CLOSED thin door observed from ONE side: the laser marks only the
    # near FACE -> a single lethal column across the door's THICKNESS (X); the
    # slab interior/far side is never seen, so it is not lethal. res 0.1,
    # 5 cols (X thickness) x 15 rows (Y width), origin (0,0). Column i=0 is
    # lethal for every row (the observed face).
    cells = [0] * (5 * 15)
    for j in range(15):
        cells[j * 5 + 0] = 100
    return GridView(resolution=0.1, width=5, height=15,
                    origin_x=0.0, origin_y=0.0, data=cells)


def _thin_door_footprint():
    # Thin in X (thickness 0.5), wide in Y (width 1.5) -> like door:119's shape.
    return DoorFootprint("door:119", 0.25, 0.75, 0.5, 1.5)


def test_one_sided_closed_face_reads_closed():
    # A fully-closed thin door seen from one side marks only its near face (one
    # column across the thickness). Measuring 2-D AREA dilutes that to
    # ~1/thickness (here 15/75 == 0.20) and misreads the closed door as unknown;
    # measuring the blocked fraction of the WIDTH (collapsing thickness) gives
    # 1.0 -> unambiguously closed.
    frac, observed = occupied_fraction(
        _one_sided_closed_grid(), _thin_door_footprint(), lethal_threshold=100
    )
    assert frac >= 0.30
    assert observed >= 3


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
