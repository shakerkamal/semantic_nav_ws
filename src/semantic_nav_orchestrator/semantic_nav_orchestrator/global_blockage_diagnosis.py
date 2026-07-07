# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Pure, ROS-free diagnosis of up-front (global) navigation blockages.

Given a global costmap (as a plain CostGrid), the robot pose, and the goal
pose, decide whether the goal is reachable and, if not, locate the thin
barrier separating the robot's free-space region from the goal's region and
classify it. No rclpy / nav_msgs imports — unit-testable against synthetic
grids.
"""

from __future__ import annotations

import math
from collections import deque
from dataclasses import dataclass
from typing import Sequence, Set, Tuple

Cell = Tuple[int, int]

_NEIGHBORS4 = ((1, 0), (-1, 0), (0, 1), (0, -1))
_NEIGHBORS8 = (
    (-1, -1), (-1, 0), (-1, 1),
    (0, -1), (0, 1),
    (1, -1), (1, 0), (1, 1),
)


@dataclass(frozen=True)
class CostGrid:
    """A plain occupancy grid: row-major data, -1 unknown, 0..100 cost."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    data: Sequence[int]

    def in_bounds(self, i: int, j: int) -> bool:
        """Return True if cell (i, j) is inside the grid."""
        return 0 <= i < self.width and 0 <= j < self.height

    def value(self, i: int, j: int) -> int:
        """Return the raw cost at cell (i, j) (caller ensures in-bounds)."""
        return self.data[j * self.width + i]


def is_free(value: int, lethal_threshold: int) -> bool:
    """Return True if a cost value is traversable free space.

    Unknown cells (value < 0) are NOT free.
    """
    return 0 <= value < lethal_threshold


def world_to_cell(grid: CostGrid, x: float, y: float) -> Cell:
    """Convert a world (x, y) in the grid frame to integer cell indices."""
    i = int((x - grid.origin_x) / grid.resolution)
    j = int((y - grid.origin_y) / grid.resolution)
    return i, j


def barrier_lethal_fraction(
    grid: CostGrid,
    centroid_xy: Tuple[float, float],
    radius_m: float,
    lethal_threshold: int,
) -> Tuple[float, int]:
    """Fraction of KNOWN cells within radius_m of a point that are >= lethal.

    Object-agnostic footprint check: measures whether the responsible barrier is
    still physically present at its centroid, regardless of what it is. Unknown
    cells (< 0) are skipped and not counted. Returns (lethal_fraction,
    observed_cells); (0.0, 0) if resolution is non-positive or nothing known was
    sampled. Use a lethal_threshold that counts only TRUE obstacles (e.g. 100),
    so inflation from adjacent structure does not read as "still blocked".
    """
    if grid.resolution <= 0.0:
        return 0.0, 0
    ci, cj = world_to_cell(grid, centroid_xy[0], centroid_xy[1])
    rc = max(1, int(radius_m / grid.resolution))
    lethal = 0
    observed = 0
    for dj in range(-rc, rc + 1):
        for di in range(-rc, rc + 1):
            i, j = ci + di, cj + dj
            if not grid.in_bounds(i, j):
                continue
            v = grid.value(i, j)
            if v < 0:
                continue
            observed += 1
            if v >= lethal_threshold:
                lethal += 1
    if observed == 0:
        return 0.0, 0
    return lethal / observed, observed


def cell_to_world(grid: CostGrid, i: float, j: float) -> Tuple[float, float]:
    """Convert cell indices to the world coordinate of the cell center."""
    x = grid.origin_x + (i + 0.5) * grid.resolution
    y = grid.origin_y + (j + 0.5) * grid.resolution
    return x, y


def flood_fill_free(
    grid: CostGrid, start: Cell, lethal_threshold: int
) -> Set[Cell]:
    """Return the 4-connected free-cell region reachable from start.

    Returns an empty set if the start cell is out of bounds or not free.
    """
    si, sj = start
    if not grid.in_bounds(si, sj) or not is_free(
        grid.value(si, sj), lethal_threshold
    ):
        return set()
    seen: Set[Cell] = {start}
    queue = deque([start])
    while queue:
        i, j = queue.popleft()
        for di, dj in _NEIGHBORS4:
            ni, nj = i + di, j + dj
            if (
                grid.in_bounds(ni, nj)
                and (ni, nj) not in seen
                and is_free(grid.value(ni, nj), lethal_threshold)
            ):
                seen.add((ni, nj))
                queue.append((ni, nj))
    return seen


def _is_blocked_or_unknown(grid: CostGrid, i: int, j: int, lethal: int) -> bool:
    """Return True if cell (i, j) is lethal or unknown (i.e. not free)."""
    v = grid.value(i, j)
    return v < 0 or v >= lethal


def _adjacent_to_region(i: int, j: int, region: Set[Cell]) -> bool:
    """Return True if any 8-neighbor of (i, j) is in region."""
    for di, dj in _NEIGHBORS8:
        if (i + di, j + dj) in region:
            return True
    return False


def nearest_free_cell(
    grid: CostGrid, cell: Cell, radius_cells: int, lethal_threshold: int
):
    """Return the nearest free cell within Chebyshev radius_cells, or None.

    Returns the cell itself if it is already free. Searches rings outward.
    """
    ci, cj = cell
    if grid.in_bounds(ci, cj) and is_free(grid.value(ci, cj), lethal_threshold):
        return cell
    for r in range(1, radius_cells + 1):
        best = None
        for dj in range(-r, r + 1):
            for di in range(-r, r + 1):
                if max(abs(di), abs(dj)) != r:
                    continue  # only the ring at Chebyshev distance r
                ni, nj = ci + di, cj + dj
                if grid.in_bounds(ni, nj) and is_free(
                    grid.value(ni, nj), lethal_threshold
                ):
                    if best is None:
                        best = (ni, nj)
        if best is not None:
            return best
    return None


def barrier_cells(
    grid: CostGrid,
    region_r: Set[Cell],
    region_g: Set[Cell],
    lethal_threshold: int,
) -> Set[Cell]:
    """Return blocked/unknown cells 8-adjacent to BOTH regions (the thin cut)."""
    result: Set[Cell] = set()
    for j in range(grid.height):
        for i in range(grid.width):
            if not _is_blocked_or_unknown(grid, i, j, lethal_threshold):
                continue
            if _adjacent_to_region(i, j, region_r) and _adjacent_to_region(
                i, j, region_g
            ):
                result.add((i, j))
    return result


def barrier_centroid_world(
    grid: CostGrid, cells: Set[Cell]
) -> Tuple[float, float]:
    """Return the mean world coordinate of the given cells' centers."""
    n = len(cells)
    sx = 0.0
    sy = 0.0
    for i, j in cells:
        x, y = cell_to_world(grid, i, j)
        sx += x
        sy += y
    return sx / n, sy / n


def barrier_extent_m(grid: CostGrid, cells: Set[Cell]) -> float:
    """Return the bounding-box diagonal length of the cells, in meters."""
    xs = [i for i, _ in cells]
    ys = [j for _, j in cells]
    di = (max(xs) - min(xs) + 1) * grid.resolution
    dj = (max(ys) - min(ys) + 1) * grid.resolution
    return (di * di + dj * dj) ** 0.5


def unknown_fraction(grid: CostGrid, cells: Set[Cell]) -> float:
    """Return the fraction of the given cells whose value is unknown (< 0)."""
    if not cells:
        return 0.0
    unknown = sum(1 for i, j in cells if grid.value(i, j) < 0)
    return unknown / len(cells)


def _free_frontier(
    grid: CostGrid, region: Set[Cell], lethal_threshold: int
) -> Set[Cell]:
    """Return free cells in region that are 8-adjacent to a blocked/unknown cell."""
    frontier: Set[Cell] = set()
    for i, j in region:
        for di, dj in _NEIGHBORS8:
            ni, nj = i + di, j + dj
            if grid.in_bounds(ni, nj) and _is_blocked_or_unknown(
                grid, ni, nj, lethal_threshold
            ):
                frontier.add((i, j))
                break
    return frontier


def locate_barrier(
    grid: CostGrid,
    region_r: Set[Cell],
    region_g: Set[Cell],
    lethal_threshold: int,
    gap_tolerance_cells: float = 2.0,
    cluster_window_cells: float = 10.0,
):
    """Localize the narrowest gap between R and G (the doorway).

    Finds the single closest free-frontier pair (the narrowest crossing), then
    averages the midpoints of near-minimum pairs that lie within
    cluster_window_cells of that crossing — centering the centroid WITHIN the
    doorway without letting a long wall (whose band thickness ~ the doorway's
    under inflation) drag the centroid toward the wall's center of mass. Robust
    to costmap inflation. Returns (centroid_xy, gap_m), or None if either region
    has no blocked boundary. All-pairs over frontier cells; runs once/recovery.
    """
    rf = _free_frontier(grid, region_r, lethal_threshold)
    gf = _free_frontier(grid, region_g, lethal_threshold)
    if not rf or not gf:
        return None

    min_d2 = None
    anchor = None
    for ri, rj in rf:
        for gi, gj in gf:
            d2 = (ri - gi) ** 2 + (rj - gj) ** 2
            if min_d2 is None or d2 < min_d2:
                min_d2 = d2
                anchor = ((ri, rj), (gi, gj))

    (ar, ag) = anchor
    arx, ary = cell_to_world(grid, *ar)
    agx, agy = cell_to_world(grid, *ag)
    anchor_x = (arx + agx) / 2.0
    anchor_y = (ary + agy) / 2.0

    min_d = min_d2 ** 0.5
    thresh_d2 = (min_d + gap_tolerance_cells) ** 2
    window_m2 = (cluster_window_cells * grid.resolution) ** 2
    sum_x = 0.0
    sum_y = 0.0
    count = 0
    for ri, rj in rf:
        for gi, gj in gf:
            if (ri - gi) ** 2 + (rj - gj) ** 2 > thresh_d2:
                continue
            rx, ry = cell_to_world(grid, ri, rj)
            gx, gy = cell_to_world(grid, gi, gj)
            mx = (rx + gx) / 2.0
            my = (ry + gy) / 2.0
            if (mx - anchor_x) ** 2 + (my - anchor_y) ** 2 <= window_m2:
                sum_x += mx
                sum_y += my
                count += 1

    centroid = (sum_x / count, sum_y / count)
    gap_m = min_d * grid.resolution
    return centroid, gap_m


def _blocked_cells_near(
    grid: CostGrid,
    centroid_xy: Tuple[float, float],
    radius_cells: int,
    lethal_threshold: int,
) -> Set[Cell]:
    """Return blocked/unknown cells within Chebyshev radius of a world point."""
    ci, cj = world_to_cell(grid, centroid_xy[0], centroid_xy[1])
    cells: Set[Cell] = set()
    for dj in range(-radius_cells, radius_cells + 1):
        for di in range(-radius_cells, radius_cells + 1):
            ni, nj = ci + di, cj + dj
            if grid.in_bounds(ni, nj) and _is_blocked_or_unknown(
                grid, ni, nj, lethal_threshold
            ):
                cells.add((ni, nj))
    return cells


def compute_standoff(
    grid: CostGrid,
    barrier_xy: Tuple[float, float],
    robot_xy: Tuple[float, float],
    region_r: Set[Cell],
    standoff_distance_m: float,
    lethal_threshold: int,
):
    """Compute a reachable standoff pose in front of the barrier.

    Backs off standoff_distance_m from the barrier centroid toward the robot,
    snaps the point to the nearest free cell inside region_r, and orients the
    pose to face the barrier. Returns (x, y, yaw) or None if no reachable cell
    is found near the backed-off point.
    """
    bx, by = barrier_xy
    rx, ry = robot_xy
    dx, dy = rx - bx, ry - by
    dist = math.hypot(dx, dy)
    if dist < 1e-9:
        ux, uy = 1.0, 0.0
    else:
        ux, uy = dx / dist, dy / dist
    px, py = bx + ux * standoff_distance_m, by + uy * standoff_distance_m

    cell = world_to_cell(grid, px, py)
    search_radius = max(1, int(standoff_distance_m / grid.resolution) + 1)
    snapped = nearest_free_cell(grid, cell, search_radius, lethal_threshold)
    if snapped is None or snapped not in region_r:
        # widen the search once, but only accept cells in R
        snapped = _nearest_cell_in_region(grid, cell, region_r, search_radius)
        if snapped is None:
            return None

    sx, sy = cell_to_world(grid, *snapped)
    yaw = math.atan2(by - sy, bx - sx)   # face the barrier
    return sx, sy, yaw


def _nearest_cell_in_region(
    grid: CostGrid, cell: Cell, region: Set[Cell], radius_cells: int
):
    """Return the nearest cell to `cell` that is a member of region, or None."""
    ci, cj = cell
    if cell in region:
        return cell
    for r in range(1, radius_cells + 1):
        for dj in range(-r, r + 1):
            for di in range(-r, r + 1):
                if max(abs(di), abs(dj)) != r:
                    continue
                candidate = (ci + di, cj + dj)
                if candidate in region:
                    return candidate
    return None


def select_reachable_standoff_side(
    grid: CostGrid,
    region_r: Set[Cell],
    side_a_xy: Tuple[float, float],
    side_b_xy: Tuple[float, float],
    lethal_threshold: int,
):
    """Return whichever candidate standoff (side_a first) lies in region_r.

    Returns None if neither candidate's cell is in the robot-reachable region.
    """
    for xy in (side_a_xy, side_b_xy):
        cell = world_to_cell(grid, xy[0], xy[1])
        if cell in region_r:
            return xy
    return None


DIAG_REACHABLE = "reachable_but_planner_failed"
DIAG_BLOCKED = "blocked"
DIAG_UNKNOWN_FRONTIER = "unknown_frontier"
DIAG_NO_THIN_BARRIER = "no_thin_barrier"
DIAG_GOAL_UNMAPPED = "goal_unmapped"


@dataclass(frozen=True)
class GlobalBlockageDiagnosis:
    """Result of a global reachability diagnosis. Poses are plain tuples."""

    diagnosis: str
    barrier_centroid: Tuple[float, float] = None
    barrier_extent_m: float = 0.0
    standoff_pose: Tuple[float, float, float] = None
    approach_frontier: Tuple[float, float] = None
    blocked_cell_fraction: float = 0.0
    unknown_cell_fraction: float = 0.0
    confidence: float = 0.0


def _nearest_region_cell_to_point(
    grid: CostGrid, region: Set[Cell], point_cell: Cell
):
    """Return the region cell closest (Euclidean) to point_cell, or None."""
    if not region:
        return None
    pi, pj = point_cell
    best = None
    best_d2 = None
    for i, j in region:
        d2 = (i - pi) ** 2 + (j - pj) ** 2
        if best_d2 is None or d2 < best_d2:
            best_d2 = d2
            best = (i, j)
    return best


def diagnose_global_blockage(
    grid: CostGrid,
    robot_xy: Tuple[float, float],
    goal_xy: Tuple[float, float],
    *,
    lethal_threshold: int = 90,
    goal_tolerance_cells: int = 3,
    standoff_distance_m: float = 1.0,
    unknown_fraction_threshold: float = 0.5,
    max_gap_m: float = 2.0,
    barrier_sample_radius_cells: int = 4,
) -> GlobalBlockageDiagnosis:
    """Diagnose why the goal is unreachable from the robot on the grid.

    Returns one of the DIAG_* labels with supporting geometry. See module
    docstring and design spec sections 5.3, 5.5, 5.6.
    """
    robot_cell = world_to_cell(grid, robot_xy[0], robot_xy[1])
    goal_cell = world_to_cell(grid, goal_xy[0], goal_xy[1])

    region_r = flood_fill_free(grid, robot_cell, lethal_threshold)
    if not region_r:
        return GlobalBlockageDiagnosis(diagnosis=DIAG_NO_THIN_BARRIER)

    goal_free = nearest_free_cell(
        grid, goal_cell, goal_tolerance_cells, lethal_threshold
    )
    if goal_free is None:
        return GlobalBlockageDiagnosis(diagnosis=DIAG_GOAL_UNMAPPED)
    if goal_free in region_r:
        return GlobalBlockageDiagnosis(diagnosis=DIAG_REACHABLE)

    region_g = flood_fill_free(grid, goal_free, lethal_threshold)
    located = locate_barrier(grid, region_r, region_g, lethal_threshold)

    if located is None or located[1] > max_gap_m:
        frontier_cell = _nearest_region_cell_to_point(
            grid, region_r, goal_cell
        )
        frontier = (
            cell_to_world(grid, *frontier_cell)
            if frontier_cell is not None
            else None
        )
        return GlobalBlockageDiagnosis(
            diagnosis=DIAG_NO_THIN_BARRIER, approach_frontier=frontier
        )

    centroid, _gap_m = located
    near = _blocked_cells_near(
        grid, centroid, barrier_sample_radius_cells, lethal_threshold
    )
    extent = barrier_extent_m(grid, near) if near else _gap_m
    unk_frac = unknown_fraction(grid, near)
    blocked_frac = 1.0 - unk_frac
    standoff = compute_standoff(
        grid, centroid, robot_xy, region_r,
        standoff_distance_m, lethal_threshold,
    )

    if unk_frac >= unknown_fraction_threshold:
        diagnosis = DIAG_UNKNOWN_FRONTIER
        confidence = 0.4
    else:
        diagnosis = DIAG_BLOCKED
        confidence = min(1.0, 0.6 + 0.4 * blocked_frac)

    return GlobalBlockageDiagnosis(
        diagnosis=diagnosis,
        barrier_centroid=centroid,
        barrier_extent_m=extent,
        standoff_pose=standoff,
        approach_frontier=None,
        blocked_cell_fraction=blocked_frac,
        unknown_cell_fraction=unk_frac,
        confidence=confidence,
    )
