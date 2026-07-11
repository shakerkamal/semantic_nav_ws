# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""FailureDiagnosis (spec 10): explicit diagnosis object for logging, eval,
and LLM prompting. ROS-free; poses are (frame, x, y, yaw) tuples or None."""

from __future__ import annotations

from dataclasses import asdict, dataclass, field
from typing import List, Optional, Tuple

Pose = Tuple[str, float, float, float]
Point2 = Tuple[float, float]


@dataclass(frozen=True)
class FailureDiagnosis:
    event_id: str
    failure_stage: str
    nav2_error_code: str
    original_query: str
    intent_hint: str
    resolved_target_object_key: str
    resolved_target_tag: str
    original_goal_pose: Optional[Pose]
    diagnosis: str
    costmap_source: str
    robot_region_id: int
    target_region_id: int
    barrier_centroid: Optional[Point2]
    barrier_extent_m: float
    blocked_cell_fraction: float
    unknown_cell_fraction: float
    responsible_object_key: str
    responsible_object_tag: str
    responsible_state_detail: str
    responsible_traversability: str
    responsible_openable: bool
    responsible_clearable: bool
    responsible_robot_openable: bool
    responsible_safety_class: str
    responsible_match_type: str
    responsible_match_confidence: float
    standoff_pose: Optional[Pose]
    standoff_validated: bool
    allowed_actions: List[str] = field(default_factory=list)
    deterministic_override: bool = False
    local_db_version: int = 0


def to_log_dict(fd: FailureDiagnosis) -> dict:
    """JSON-serializable dict for the recovery ledger."""
    return asdict(fd)
