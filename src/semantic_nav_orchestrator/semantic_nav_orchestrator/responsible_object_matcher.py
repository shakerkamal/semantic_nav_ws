"""Pure-Python responsible-object matcher.

Used by the orchestrator's /match_responsible_object service handler and
internally by recovery-trigger ingestion. No rclpy or ROS msg imports.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Optional, Sequence, Tuple


INFERRED_FALLBACK_RADIUS_M = 0.75


def should_trust_supplied_match(match_type: Optional[str]) -> bool:
    """True if a match_type already supplied by an upstream matcher (e.g.
    /match_responsible_object, which sees BOTH static and dynamic candidates)
    should be trusted as-is, rather than re-derived from a source that can
    only see static/persistent-map objects (e.g. a static-only catalog that
    has no knowledge of live-perceived detections at all). Only "unknown"
    (or empty/missing) means "nothing to trust" and callers should fall back
    to their own re-match.
    """
    return (match_type or "").strip().lower() in {"verified", "inferred"}


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
    state_detail: str = ""
    traversability: str = ""
    source: str = ""               # "persistent_map" | "dynamic_overlay" | ""


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
    state_detail: str = ""
    traversability: str = ""
    source: str = ""              # winning candidate's provenance

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
            state_detail="",
            traversability="",
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


_COLOCATION_EPSILON_M = 1e-6


def _aabb_intersects_2d(a: ObjectCandidate, b: ObjectCandidate) -> bool:
    ax, ay, _ = a.bbox_center
    bx, by, _ = b.bbox_center
    aex, aey, _ = a.bbox_extent
    bex, bey, _ = b.bbox_extent

    return (
        abs(ax - bx) <= (max(0.0, aex) + max(0.0, bex)) / 2.0 and
        abs(ay - by) <= (max(0.0, aey) + max(0.0, bey)) / 2.0
    )


def _qualifying_live_overrides(
    centroid: Tuple[float, float, float],
    candidates: Sequence[ObjectCandidate],
    static_winner: ObjectCandidate,
    fallback_radius_m: float,
) -> "list[Tuple[float, ObjectCandidate]]":
    """Live-perceived candidates that plausibly explain a static-only match.

    Depth sensing marks an object's NEAR FACE, so a measured centroid is
    systematically displaced from the object center toward the robot -- a
    small live object can miss its own inflated-bbox containment while a
    co-located static record's long bbox still contains the centroid
    (S3 2026-07-17: chair vs room partition). A live observation qualifies
    to take precedence over the static-only verified winner when it is
    within the fallback radius, its bbox intersects the static winner's
    (same physical spot, not an unrelated bystander), and it is at least
    as close to the blockage as the static winner's own center.

    Freshness is owned by the semantics layer: DynamicObjectCache.snapshot()
    purges expired entries before /refresh_local_objects responds, so every
    dynamic_overlay candidate seen here is a live observation by contract.
    """
    static_distance = _planar_distance_to_bbox_center(centroid, static_winner)

    qualifying = []
    for candidate in candidates:
        if candidate.source != "dynamic_overlay":
            continue
        distance = _planar_distance_to_bbox_center(centroid, candidate)
        if (
            distance <= float(fallback_radius_m) and
            _aabb_intersects_2d(candidate, static_winner) and
            distance <= static_distance + _COLOCATION_EPSILON_M
        ):
            qualifying.append((distance, candidate))
    return qualifying


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
        state_detail=candidate.state_detail,
        traversability=candidate.traversability,
        source=candidate.source,
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
        # A static persistent-map record can share coordinates with a live
        # -detected dynamic object placed on top of it (e.g. a chair spawned
        # on a room partition's own bbox) -- both then satisfy inflated-bbox
        # containment. The static record only reflects the map; the dynamic
        # observation is what perception actually found there right now, so
        # prefer it even when it is not the nearest bbox center -- "nearest"
        # alone has no principled way to break this specific tie.
        dynamic_verified = [c for c in verified if c.source == "dynamic_overlay"]
        pool = dynamic_verified if dynamic_verified else verified

        nearest_verified = min(
            pool,
            key=lambda candidate: _planar_distance_to_bbox_center(
                blockage_centroid,
                candidate,
            ),
        )

        if not dynamic_verified:
            qualifying = _qualifying_live_overrides(
                blockage_centroid,
                candidates,
                nearest_verified,
                inferred_fallback_radius_m,
            )
            if len(qualifying) == 1:
                distance, live = qualifying[0]
                return _result_from_candidate(
                    live,
                    match_type="inferred",
                    message=(
                        "live_static_colocation_precedence "
                        f"distance={distance:.3f} m over static "
                        f"'{nearest_verified.object_key}'"
                    ),
                )
            if len(qualifying) > 1:
                return MatchResult.unknown(
                    "ambiguous_live_static_overlap: "
                    f"{len(qualifying)} live-perceived candidates plausibly "
                    "explain the blockage; refusing to pick arbitrarily"
                )

        if len(verified) == 1:
            message = "verified inflated-bbox containment"
        elif dynamic_verified:
            message = (
                f"verified dynamic-preferred nearest of {len(dynamic_verified)} "
                f"live-perceived match(es) (of {len(verified)} total)"
            )
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