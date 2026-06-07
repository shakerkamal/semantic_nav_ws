"""Pure-Python responsible-object matcher.

Used by the orchestrator's /match_responsible_object service handler and
internally by recovery-trigger ingestion. No rclpy or ROS msg imports.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Sequence, Tuple


INFERRED_FALLBACK_RADIUS_M = 0.75


@dataclass(frozen=True)
class ObjectCandidate:
    """Generic candidate shape; ROS handlers convert ObjectInstance.msg to this."""
    object_key: str
    object_tag: str
    object_state: str
    safety_class: str
    openable: bool
    clearable: bool
    bbox_center: Tuple[float, float, float]
    bbox_extent: Tuple[float, float, float]


@dataclass(frozen=True)
class MatchResult:
    success: bool
    match_type: str               # "verified" | "inferred" | "unknown"
    responsible_object_key: str
    responsible_object_tag: str
    responsible_object_state: str
    safety_class: str
    openable: bool
    clearable: bool
    bbox_center: Tuple[float, float, float]
    bbox_extent: Tuple[float, float, float]
    message: str

    @classmethod
    def unknown(cls, message: str = "no candidate matched") -> "MatchResult":
        return cls(
            success=False,
            match_type="unknown",
            responsible_object_key="",
            responsible_object_tag="",
            responsible_object_state="",
            safety_class="",
            openable=False,
            clearable=False,
            bbox_center=(0.0, 0.0, 0.0),
            bbox_extent=(0.0, 0.0, 0.0),
            message=message,
        )


def _inflated_contains(
    centroid: Tuple[float, float, float],
    inflate: float,
    candidate: ObjectCandidate,
) -> bool:
    cx, cy, _ = centroid
    bx, by, _ = candidate.bbox_center
    ex, ey, _ = candidate.bbox_extent

    half_x = max(0.0, ex) / 2.0 + inflate
    half_y = max(0.0, ey) / 2.0 + inflate

    return abs(cx - bx) <= half_x and abs(cy - by) <= half_y


def _planar_distance_to_bbox_center(
    centroid: Tuple[float, float, float],
    candidate: ObjectCandidate,
) -> float:
    cx, cy, _ = centroid
    bx, by, _ = candidate.bbox_center

    dx = cx - bx
    dy = cy - by

    return math.sqrt(dx * dx + dy * dy)


def _result_from_candidate(
    candidate: ObjectCandidate,
    match_type: str,
    message: str,
) -> MatchResult:
    return MatchResult(
        success=True,
        match_type=match_type,
        responsible_object_key=candidate.object_key,
        responsible_object_tag=candidate.object_tag,
        responsible_object_state=candidate.object_state,
        safety_class=candidate.safety_class,
        openable=bool(candidate.openable),
        clearable=bool(candidate.clearable),
        bbox_center=candidate.bbox_center,
        bbox_extent=candidate.bbox_extent,
        message=message,
    )


def match_responsible_object(
    blockage_centroid: Tuple[float, float, float],
    blockage_extent_m: float,
    candidates: Sequence[ObjectCandidate],
    inferred_fallback_radius_m: float = INFERRED_FALLBACK_RADIUS_M,
) -> MatchResult:
    """Match blockage geometry against object candidates.

    Verified:
        The blockage centroid lies inside an inflated object bbox.
        If multiple inflated bboxes contain the centroid, the nearest bbox
        center is selected and the match remains verified.

    Inferred:
        No inflated bbox contains the centroid, but the nearest bbox center is
        within inferred_fallback_radius_m.

    Unknown:
        No candidate satisfies either rule.
    """
    if not candidates:
        return MatchResult.unknown("no candidates supplied")

    inflate = max(0.0, float(blockage_extent_m) / 2.0)

    verified = [
        candidate
        for candidate in candidates
        if _inflated_contains(blockage_centroid, inflate, candidate)
    ]

    if verified:
        nearest_verified = min(
            verified,
            key=lambda candidate: _planar_distance_to_bbox_center(
                blockage_centroid,
                candidate,
            ),
        )

        if len(verified) == 1:
            message = "verified inflated-bbox containment"
        else:
            message = (
                f"verified nearest of {len(verified)} inflated-bbox matches"
            )

        return _result_from_candidate(
            nearest_verified,
            match_type="verified",
            message=message,
        )

    nearest = min(
        candidates,
        key=lambda candidate: _planar_distance_to_bbox_center(
            blockage_centroid,
            candidate,
        ),
    )

    distance = _planar_distance_to_bbox_center(blockage_centroid, nearest)

    if distance <= float(inferred_fallback_radius_m):
        return _result_from_candidate(
            nearest,
            match_type="inferred",
            message=f"inferred nearest-fallback distance={distance:.3f} m",
        )

    return MatchResult.unknown(
        f"nearest candidate at {distance:.3f} m exceeds fallback"
    )