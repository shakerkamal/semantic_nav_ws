import math
from dataclasses import dataclass
from typing import Tuple

from semantic_nav_semantics.semantic_store import ObjectRow

_EPS = 1e-6


@dataclass(frozen=True)
class StandoffPose:
    position_xy: Tuple[float, float]
    yaw: float
    standoff_distance: float


class StandoffPlanner:
    """Computes a robot-reachable standoff pose in front of a detected object.

    The goal position is placed at (half max bbox extent + footprint + margin)
    along the robot→object vector, facing back toward the object.
    """

    def __init__(
        self,
        robot_footprint_radius: float = 0.22,
        clearance_margin: float = 0.20,
    ) -> None:
        self._radius = float(robot_footprint_radius)
        self._margin = float(clearance_margin)

    def plan(
        self,
        row: ObjectRow,
        robot_xy: Tuple[float, float],
    ) -> StandoffPose:
        cx, cy, _ = row.bbox_center
        rx, ry = robot_xy
        vx = cx - rx
        vy = cy - ry
        norm = math.hypot(vx, vy)
        if norm < _EPS:
            vx, vy, norm = 1.0, 0.0, 1.0

        half_extent_xy = 0.5 * max(row.bbox_extent[0], row.bbox_extent[1])
        d = half_extent_xy + self._radius + self._margin

        ux, uy = vx / norm, vy / norm
        gx = cx - d * ux
        gy = cy - d * uy

        yaw = math.atan2(cy - gy, cx - gx)
        return StandoffPose(position_xy=(gx, gy), yaw=yaw, standoff_distance=d)
