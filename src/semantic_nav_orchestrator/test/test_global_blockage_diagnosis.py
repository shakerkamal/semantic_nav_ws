# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Unit tests for global blockage diagnosis — synthetic grids, no ROS."""

from semantic_nav_orchestrator.global_blockage_diagnosis import (
    CostGrid,
    cell_to_world,
    flood_fill_free,
    is_free,
    world_to_cell,
)


def _uniform_grid(width, height, value, resolution=1.0, ox=0.0, oy=0.0):
    return CostGrid(resolution, width, height, ox, oy, [value] * (width * height))


def test_is_free_bands():
    assert is_free(0, 90) is True
    assert is_free(89, 90) is True
    assert is_free(90, 90) is False
    assert is_free(-1, 90) is False  # unknown is not free


def test_world_to_cell_and_back_roundtrip_center():
    grid = _uniform_grid(4, 4, 0, resolution=0.5, ox=-1.0, oy=-1.0)
    # world (0.0, 0.0) -> cell, then back to that cell's center
    cell = world_to_cell(grid, 0.0, 0.0)
    assert cell == (2, 2)
    cx, cy = cell_to_world(grid, *cell)
    assert abs(cx - 0.25) < 1e-9 and abs(cy - 0.25) < 1e-9


def test_flood_fill_all_free():
    grid = _uniform_grid(3, 3, 0)
    region = flood_fill_free(grid, (0, 0), lethal_threshold=90)
    assert len(region) == 9


def test_flood_fill_stops_at_wall():
    # 3-wide: column 1 is a lethal wall -> flood from (0,0) sees only column 0.
    data = [
        0, 100, 0,
        0, 100, 0,
        0, 100, 0,
    ]
    grid = CostGrid(1.0, 3, 3, 0.0, 0.0, data)
    region = flood_fill_free(grid, (0, 0), lethal_threshold=90)
    assert region == {(0, 0), (0, 1), (0, 2)}


def test_flood_fill_start_not_free_returns_empty():
    grid = _uniform_grid(3, 3, 100)
    assert flood_fill_free(grid, (0, 0), lethal_threshold=90) == set()


# ---------------------------------------------------------------------------
# Task 2: barrier localization + geometry helpers
# ---------------------------------------------------------------------------

from semantic_nav_orchestrator.global_blockage_diagnosis import (  # noqa: E402
    barrier_cells,
    barrier_centroid_world,
    barrier_extent_m,
    nearest_free_cell,
    unknown_fraction,
)


def _two_rooms_closed(wall_value):
    # 5 wide x 3 tall. Column 2 is the wall (wall_value). Rooms: cols 0-1, cols 3-4.
    row = [0, 0, wall_value, 0, 0]
    data = row * 3
    return CostGrid(1.0, 5, 3, 0.0, 0.0, data)


def test_nearest_free_cell_returns_self_when_free():
    grid = _uniform_grid(3, 3, 0)
    assert nearest_free_cell(grid, (1, 1), 2, 90) == (1, 1)


def test_nearest_free_cell_finds_neighbor():
    # center lethal, ring free
    data = [0, 0, 0, 0, 100, 0, 0, 0, 0]
    grid = CostGrid(1.0, 3, 3, 0.0, 0.0, data)
    result = nearest_free_cell(grid, (1, 1), 1, 90)
    assert result is not None and result != (1, 1)


def test_nearest_free_cell_none_when_all_blocked_in_range():
    grid = _uniform_grid(3, 3, 100)
    assert nearest_free_cell(grid, (1, 1), 1, 90) is None


def test_barrier_cells_thin_wall_touches_both_regions():
    grid = _two_rooms_closed(100)
    r = flood_fill_free(grid, (0, 0), 90)   # left room
    g = flood_fill_free(grid, (4, 0), 90)   # right room
    barrier = barrier_cells(grid, r, g, 90)
    # The whole column 2 (3 cells) separates the rooms and touches both.
    assert barrier == {(2, 0), (2, 1), (2, 2)}


def test_barrier_cells_thick_wall_is_empty():
    # 6 wide: columns 2,3 both lethal (2-cell-thick wall). Interior cells
    # do not touch both rooms -> no thin barrier.
    row = [0, 0, 100, 100, 0, 0]
    grid = CostGrid(1.0, 6, 3, 0.0, 0.0, row * 3)
    r = flood_fill_free(grid, (0, 0), 90)
    g = flood_fill_free(grid, (5, 0), 90)
    assert barrier_cells(grid, r, g, 90) == set()


def test_barrier_centroid_and_extent():
    grid = _two_rooms_closed(100)
    cells = {(2, 0), (2, 1), (2, 2)}
    cx, cy = barrier_centroid_world(grid, cells)
    assert abs(cx - 2.5) < 1e-9   # column 2 center x = 2.5
    assert abs(cy - 1.5) < 1e-9   # rows 0..2 center y = 1.5
    # bbox is 1 cell wide (di=1) x 3 cells tall (dj=3) at res 1.0
    ext = barrier_extent_m(grid, cells)
    assert abs(ext - (1.0 ** 2 + 3.0 ** 2) ** 0.5) < 1e-9


def test_unknown_fraction():
    grid = _two_rooms_closed(-1)   # wall cells are unknown
    cells = {(2, 0), (2, 1), (2, 2)}
    assert unknown_fraction(grid, cells) == 1.0
    grid2 = _two_rooms_closed(100)
    assert unknown_fraction(grid2, cells) == 0.0


# ---------------------------------------------------------------------------
# Task 3: standoff computation + reachable-side selection
# ---------------------------------------------------------------------------

import math  # noqa: E402

from semantic_nav_orchestrator.global_blockage_diagnosis import (  # noqa: E402
    compute_standoff,
    select_reachable_standoff_side,
)


def test_compute_standoff_backs_off_toward_robot_and_faces_barrier():
    grid = _two_rooms_closed(100)          # wall at column 2 (x center 2.5)
    r = flood_fill_free(grid, (0, 0), 90)  # left room (cols 0,1)
    # barrier centroid ~ (2.5, 1.5); robot at (0.5, 1.5) -> standoff to the left.
    pose = compute_standoff(
        grid, barrier_xy=(2.5, 1.5), robot_xy=(0.5, 1.5),
        region_r=r, standoff_distance_m=1.0, lethal_threshold=90,
    )
    assert pose is not None
    x, y, yaw = pose
    assert x < 2.5                          # backed off toward the robot side
    assert world_to_cell(grid, x, y) in r   # snapped into the reachable region
    # faces the barrier (+x direction): yaw ~ 0
    assert abs(math.atan2(math.sin(yaw), math.cos(yaw))) < 1e-6


def test_compute_standoff_none_when_no_free_cell_in_r_near_point():
    grid = _uniform_grid(3, 3, 100)         # everything lethal
    pose = compute_standoff(
        grid, barrier_xy=(1.5, 1.5), robot_xy=(0.5, 0.5),
        region_r=set(), standoff_distance_m=1.0, lethal_threshold=90,
    )
    assert pose is None


def test_select_reachable_standoff_side_picks_side_in_r():
    grid = _two_rooms_closed(100)
    r = flood_fill_free(grid, (0, 0), 90)   # left room only
    side_a = (3.5, 1.5)   # right room (not in R)
    side_b = (0.5, 1.5)   # left room (in R)
    chosen = select_reachable_standoff_side(grid, r, side_a, side_b, 90)
    assert chosen == side_b


def test_select_reachable_standoff_side_none_when_neither_in_r():
    grid = _two_rooms_closed(100)
    r = flood_fill_free(grid, (0, 0), 90)
    assert select_reachable_standoff_side(
        grid, r, (3.5, 1.5), (4.5, 1.5), 90
    ) is None


# ---------------------------------------------------------------------------
# Task 4: end-to-end diagnose_global_blockage
# ---------------------------------------------------------------------------

from semantic_nav_orchestrator.global_blockage_diagnosis import (  # noqa: E402
    DIAG_BLOCKED,
    DIAG_GOAL_UNMAPPED,
    DIAG_NO_THIN_BARRIER,
    DIAG_REACHABLE,
    DIAG_UNKNOWN_FRONTIER,
    diagnose_global_blockage,
)


def _two_rooms_with_door(door_value):
    # 5 wide x 3 tall. Column 2 is a wall EXCEPT the middle cell (2,1) which
    # takes door_value. door_value=0 -> open; 100 -> closed; -1 -> unknown.
    data = [
        0, 0, 100, 0, 0,
        0, 0, door_value, 0, 0,
        0, 0, 100, 0, 0,
    ]
    return CostGrid(1.0, 5, 3, 0.0, 0.0, data)


def test_diagnose_reachable_when_door_open():
    grid = _two_rooms_with_door(0)                 # open doorway
    d = diagnose_global_blockage(grid, (0.5, 1.5), (4.5, 1.5))
    assert d.diagnosis == DIAG_REACHABLE


def test_diagnose_blocked_when_door_closed():
    grid = _two_rooms_with_door(100)               # closed door (lethal)
    d = diagnose_global_blockage(grid, (0.5, 1.5), (4.5, 1.5))
    assert d.diagnosis == DIAG_BLOCKED
    assert d.barrier_centroid is not None
    cx, _ = d.barrier_centroid
    assert abs(cx - 2.5) < 0.6                      # barrier is at column 2
    assert d.standoff_pose is not None
    sx, _, _ = d.standoff_pose
    assert sx < 2.5                                 # standoff on robot side


def test_diagnose_unknown_frontier_when_barrier_unknown():
    # The whole separating column is unknown, so the barrier is unknown-dominated.
    data = [
        0, 0, -1, 0, 0,
        0, 0, -1, 0, 0,
        0, 0, -1, 0, 0,
    ]
    grid = CostGrid(1.0, 5, 3, 0.0, 0.0, data)
    d = diagnose_global_blockage(grid, (0.5, 1.5), (4.5, 1.5))
    assert d.diagnosis == DIAG_UNKNOWN_FRONTIER
    assert d.unknown_cell_fraction >= 0.5


def test_diagnose_no_thin_barrier_for_thick_wall():
    # 6 wide: columns 2,3 lethal (2-cell-thick wall).
    row = [0, 0, 100, 100, 0, 0]
    grid = CostGrid(1.0, 6, 3, 0.0, 0.0, row * 3)
    d = diagnose_global_blockage(grid, (0.5, 1.5), (5.5, 1.5))
    assert d.diagnosis == DIAG_NO_THIN_BARRIER
    assert d.approach_frontier is not None


def test_diagnose_goal_unmapped_when_goal_in_lethal():
    grid = _two_rooms_with_door(100)
    # Goal placed inside the wall column, no free cell within tolerance.
    d = diagnose_global_blockage(
        grid, (0.5, 1.5), (2.5, 1.5), goal_tolerance_cells=0
    )
    assert d.diagnosis == DIAG_GOAL_UNMAPPED
