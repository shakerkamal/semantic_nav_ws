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
) -> Tuple[float, int]:
    """Fraction of KNOWN cells inside the door footprint that are >= lethal.

    Returns (occupied_fraction, observed_cells). Unknown cells (value < 0) are
    skipped and not counted in observed_cells. Returns (0.0, 0) if the grid has
    no positive resolution or no known cells fall inside the footprint.
    """
    res = grid.resolution
    if res <= 0.0:
        return 0.0, 0

    half_x = footprint.extent_x / 2.0
    half_y = footprint.extent_y / 2.0
    min_i = int((footprint.center_x - half_x - grid.origin_x) / res)
    max_i = int((footprint.center_x + half_x - grid.origin_x) / res)
    min_j = int((footprint.center_y - half_y - grid.origin_y) / res)
    max_j = int((footprint.center_y + half_y - grid.origin_y) / res)

    occ = 0
    observed = 0
    for j in range(max(0, min_j), min(grid.height, max_j + 1)):
        row = j * grid.width
        for i in range(max(0, min_i), min(grid.width, max_i + 1)):
            v = grid.data[row + i]
            if v < 0:
                continue
            observed += 1
            if v >= lethal_threshold:
                occ += 1

    if observed == 0:
        return 0.0, 0
    return occ / observed, observed


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
