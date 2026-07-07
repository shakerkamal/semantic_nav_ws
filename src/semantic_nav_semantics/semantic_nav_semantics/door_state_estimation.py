# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Pure door-state estimation from an occupancy grid.

No ROS imports — testable without a running node. Given a door's bbox footprint
and an occupancy grid, compute the occupied fraction over the footprint and
classify the door as open / closed / unknown.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Mapping, Sequence, Tuple

from semantic_nav_semantics.semantic_store import make_object_key


@dataclass(frozen=True)
class DoorFootprint:
    object_key: str
    center_x: float
    center_y: float
    extent_x: float
    extent_y: float


@dataclass(frozen=True)
class GridView:
    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    data: Sequence[int]  # row-major, length width*height; <0 == unknown


@dataclass(frozen=True)
class DoorStateEstimate:
    object_key: str
    door_state: str        # open | closed | unknown
    traversability: str    # passable | blocked | unknown
    confidence: float      # 0.0 - 1.0
    occupied_fraction: float
    observed_cells: int


def occupied_fraction(
    grid: GridView,
    footprint: DoorFootprint,
    lethal_threshold: int,
    margin_frac: float = 0.0,
) -> Tuple[float, int]:
    """Fraction of the door's WIDTH that is blocked by a lethal obstacle.

    Returns (blocked_width_fraction, observed_lines). A door is a thin barrier
    across an opening, so its closure is "how much of the WIDTH (long axis) is
    blocked", NOT how much of the 2-D bbox AREA is occupied. A laser sees only
    the near FACE, so a fully-closed thin slab marks just one lethal cell across
    its THICKNESS (short axis); measuring 2-D area would dilute that to
    ~1/thickness and never reach the "blocked" band. So we collapse the
    thickness: we scan each line across the width and count it *blocked* if ANY
    cell along the thickness is lethal (>= lethal_threshold), *observed* if it
    has any KNOWN cell (value >= 0). The result is blocked_lines / observed_lines
    — 0.0 for a clear opening, ~1.0 for a closed door, regardless of which side
    the laser observed or how thick the slab is.

    ``margin_frac`` insets the WIDTH axis by this fraction of its half-extent so
    sampling targets the *clear opening* the slab fills, skipping the frame posts
    that overlap the bbox ends. The thickness axis is always sampled in full (its
    single observed face is the closure signal — never shrink it). A fraction,
    not absolute metres, scales to any door. The width half-extent is clamped to
    ``[min(half, res), half]`` so the inset never inverts or expands the box.

    Returns (0.0, 0) if the grid has no positive resolution or no known cells
    fall inside the footprint.
    """
    res = grid.resolution
    if res <= 0.0:
        return 0.0, 0

    half_x = footprint.extent_x / 2.0
    half_y = footprint.extent_y / 2.0
    # The door's WIDTH is its longer horizontal extent (frame posts sit at its
    # ends); the THICKNESS is the shorter one (only its near face is observed).
    width_is_x = footprint.extent_x >= footprint.extent_y
    if width_is_x:
        half_w, center_w, origin_w, n_w = half_x, footprint.center_x, grid.origin_x, grid.width
        half_t, center_t, origin_t, n_t = half_y, footprint.center_y, grid.origin_y, grid.height
    else:
        half_w, center_w, origin_w, n_w = half_y, footprint.center_y, grid.origin_y, grid.height
        half_t, center_t, origin_t, n_t = half_x, footprint.center_x, grid.origin_x, grid.width

    if margin_frac > 0.0:
        half_w = max(half_w * (1.0 - margin_frac), min(half_w, res))

    min_w = int((center_w - half_w - origin_w) / res)
    max_w = int((center_w + half_w - origin_w) / res)
    min_t = int((center_t - half_t - origin_t) / res)
    max_t = int((center_t + half_t - origin_t) / res)

    blocked_lines = 0
    observed_lines = 0
    for w in range(max(0, min_w), min(n_w, max_w + 1)):
        line_observed = False
        line_blocked = False
        for t in range(max(0, min_t), min(n_t, max_t + 1)):
            i, j = (w, t) if width_is_x else (t, w)
            v = grid.data[j * grid.width + i]
            if v < 0:
                continue
            line_observed = True
            if v >= lethal_threshold:
                line_blocked = True
                break
        if line_observed:
            observed_lines += 1
            if line_blocked:
                blocked_lines += 1

    if observed_lines == 0:
        return 0.0, 0
    return blocked_lines / observed_lines, observed_lines


def classify_door_state(
    occ_frac: float,
    observed_cells: int,
    *,
    blocked_fraction: float,
    open_fraction: float,
    min_observed_cells: int,
    object_key: str,
) -> DoorStateEstimate:
    """Classify a door as closed / open / unknown from its occupied fraction.

    - observed_cells < min_observed_cells   -> unknown (no fresh evidence)
    - occ_frac >= blocked_fraction          -> closed / blocked
    - occ_frac <  open_fraction             -> open / passable
    - otherwise (ambiguous band)            -> unknown
    """
    if observed_cells < min_observed_cells:
        return DoorStateEstimate(
            object_key, "unknown", "unknown", 0.0, occ_frac, observed_cells
        )
    if occ_frac >= blocked_fraction:
        return DoorStateEstimate(
            object_key, "closed", "blocked",
            min(1.0, occ_frac), occ_frac, observed_cells,
        )
    if occ_frac < open_fraction:
        return DoorStateEstimate(
            object_key, "open", "passable",
            min(1.0, 1.0 - occ_frac), occ_frac, observed_cells,
        )
    return DoorStateEstimate(
        object_key, "unknown", "unknown", 0.0, occ_frac, observed_cells
    )


def load_door_footprints(
    map_objects: Mapping[str, Mapping[str, object]],
) -> List[DoorFootprint]:
    """Extract door footprints from a parsed map_v001.json dict.

    A door is any object whose object_tag contains 'door' (case-insensitive),
    e.g. 'door', 'closet door'. The object_key is built with make_object_key so
    it matches the keys local_object_query_node serves.
    """
    doors: List[DoorFootprint] = []
    for _source_key, obj in map_objects.items():
        tag = str(obj.get("object_tag", "")).lower()
        if "door" not in tag:
            continue
        center = obj["bbox_center"]
        extent = obj["bbox_extent"]
        key = make_object_key(str(obj["object_tag"]), int(obj["id"]))
        doors.append(
            DoorFootprint(
                object_key=key,
                center_x=float(center[0]),
                center_y=float(center[1]),
                extent_x=float(extent[0]),
                extent_y=float(extent[1]),
            )
        )
    return doors
