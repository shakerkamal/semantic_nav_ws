import math
from dataclasses import dataclass
from typing import Sequence, Set, Tuple

from semantic_nav_semantics.semantic_store import ObjectRow


@dataclass(frozen=True)
class SpatialContextBuilder:
    """Converts object geometry into a short textual neighbour summary."""

    neighbour_radius_m: float = 2.0
    max_neighbours: int = 3

    def build(
        self,
        target: ObjectRow,
        all_rows: Sequence[ObjectRow],
        robot_xy: Tuple[float, float],
        navigable_tags: Set[str],
    ) -> str:
        cx, cy, _ = target.bbox_center
        rx, ry = robot_xy
        robot_d = math.hypot(cx - rx, cy - ry)

        neighbours = []
        for row in all_rows:
            if row.object_key == target.object_key:
                continue
            if row.normalized_tag not in navigable_tags:
                continue
            d = math.hypot(row.bbox_center[0] - cx, row.bbox_center[1] - cy)
            if d > self.neighbour_radius_m:
                continue
            neighbours.append((d, row.object_key))

        neighbours.sort(key=lambda p: p[0])
        neighbours = neighbours[: self.max_neighbours]

        if neighbours:
            near = ", ".join(f"{k} ({d:.1f} m)" for d, k in neighbours)
            near_phrase = f"Near: {near}."
        else:
            near_phrase = f"Near: (none within {self.neighbour_radius_m:.1f} m)."

        return f"{near_phrase} Robot distance: {robot_d:.1f} m."
