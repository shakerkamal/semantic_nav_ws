"""Unit tests for the responsible-object matcher pure function."""

from typing import Tuple

from semantic_nav_orchestrator.responsible_object_matcher import (
    ObjectCandidate,
    match_responsible_object,
    should_trust_supplied_match,
)


def test_should_trust_supplied_match_verified_and_inferred():
    # A match_type already determined by an upstream matcher that DOES see
    # dynamic candidates (e.g. /match_responsible_object) must be trusted,
    # not re-derived from a static-only catalog that cannot see them at all
    # (found 2026-07-15, S2: a dynamically-perceived 'door:903' was silently
    # swapped for the co-located static 'door:119' because the orchestrator's
    # internal static-only re-match couldn't find the dynamic key).
    assert should_trust_supplied_match("verified") is True
    assert should_trust_supplied_match("inferred") is True


def test_should_trust_supplied_match_rejects_unknown_and_empty():
    assert should_trust_supplied_match("unknown") is False
    assert should_trust_supplied_match("") is False
    assert should_trust_supplied_match(None) is False


def _cand(
    key: str,
    tag: str,
    state: str,
    safety_class: str,
    openable: bool,
    clearable: bool,
    center: Tuple[float, float, float],
    extent: Tuple[float, float, float],
) -> ObjectCandidate:
    return ObjectCandidate(
        object_key=key,
        object_tag=tag,
        object_state=state,
        safety_class=safety_class,
        openable=openable,
        clearable=clearable,
        bbox_center=center,
        bbox_extent=extent,
    )


def test_centroid_inside_bbox_yields_verified_match():
    candidate = _cand(
        "closet door:8",
        "closet door",
        "semi-static",
        "none",
        openable=True,
        clearable=False,
        center=(0.0, 0.0, 0.0),
        extent=(2.0, 1.0, 0.02),
    )

    result = match_responsible_object(
        blockage_centroid=(0.1, 0.0, 0.0),
        blockage_extent_m=0.2,
        candidates=[candidate],
    )

    assert result.success is True
    assert result.match_type == "verified"
    assert result.responsible_object_key == "closet door:8"
    assert result.openable is True


def test_centroid_outside_bbox_but_near_yields_inferred():
    candidate = _cand(
        "chair:1",
        "chair",
        "movable",
        "none",
        openable=False,
        clearable=True,
        center=(0.0, 0.0, 0.0),
        extent=(0.5, 0.5, 0.5),
    )

    result = match_responsible_object(
        blockage_centroid=(0.6, 0.0, 0.0),
        blockage_extent_m=0.1,
        candidates=[candidate],
    )

    assert result.success is True
    assert result.match_type == "inferred"
    assert result.responsible_object_key == "chair:1"


def test_centroid_far_yields_unknown():
    candidate = _cand(
        "chair:1",
        "chair",
        "movable",
        "none",
        openable=False,
        clearable=True,
        center=(0.0, 0.0, 0.0),
        extent=(0.5, 0.5, 0.5),
    )

    result = match_responsible_object(
        blockage_centroid=(5.0, 5.0, 0.0),
        blockage_extent_m=0.1,
        candidates=[candidate],
    )

    assert result.success is False
    assert result.match_type == "unknown"
    assert result.responsible_object_key == ""


def test_empty_candidates_yields_unknown():
    result = match_responsible_object(
        blockage_centroid=(0.0, 0.0, 0.0),
        blockage_extent_m=0.1,
        candidates=[],
    )

    assert result.success is False
    assert result.match_type == "unknown"


def test_verified_match_preserves_safety_class_for_human():
    candidate = _cand(
        "person:3",
        "person",
        "movable",
        "human",
        openable=False,
        clearable=False,
        center=(0.0, 0.0, 0.0),
        extent=(0.6, 0.6, 1.8),
    )

    result = match_responsible_object(
        blockage_centroid=(0.0, 0.1, 0.0),
        blockage_extent_m=0.2,
        candidates=[candidate],
    )

    assert result.success is True
    assert result.match_type == "verified"
    assert result.safety_class == "human"
    assert result.clearable is False


def test_multiple_verified_matches_remain_verified_and_choose_nearest():
    farther = _cand(
        "cabinet:1",
        "cabinet",
        "static",
        "none",
        openable=False,
        clearable=False,
        center=(0.3, 0.0, 0.0),
        extent=(1.0, 1.0, 1.0),
    )
    nearer = _cand(
        "chair:2",
        "chair",
        "movable",
        "none",
        openable=False,
        clearable=True,
        center=(0.05, 0.0, 0.0),
        extent=(1.0, 1.0, 1.0),
    )

    result = match_responsible_object(
        blockage_centroid=(0.0, 0.0, 0.0),
        blockage_extent_m=0.2,
        candidates=[farther, nearer],
    )

    assert result.success is True
    assert result.match_type == "verified"
    assert result.responsible_object_key == "chair:2"
    assert "nearest of 2" in result.message


def test_colocated_verified_matches_prefer_dynamic_over_static_even_if_farther():
    # A static persistent-map object (e.g. a room partition) can share
    # coordinates with a live-detected dynamic object placed on top of it
    # (e.g. a chair or person spawned at the same gap) -- both then satisfy
    # inflated-bbox containment. The static record reflects the map, not what
    # is physically there right now; the dynamic observation is what a live
    # detector actually perceived. Prefer it even when it is NOT the nearest
    # bbox center, since "nearest" alone has no way to break this tie
    # correctly (found 2026-07-15, S3/S4/S5 co-location with object_121).
    static_partition = ObjectCandidate(
        object_key="partition:121",
        object_tag="room partition",
        object_state="semi-static",
        safety_class="none",
        openable=True,
        clearable=False,
        bbox_center=(0.0, 0.0, 0.0),
        bbox_extent=(1.0, 1.0, 1.0),
        source="persistent_map",
    )
    dynamic_chair = ObjectCandidate(
        object_key="chair:901",
        object_tag="chair",
        object_state="movable",
        safety_class="none",
        openable=False,
        clearable=True,
        bbox_center=(0.05, 0.0, 0.0),
        bbox_extent=(1.0, 1.0, 1.0),
        source="dynamic_overlay",
    )

    result = match_responsible_object(
        blockage_centroid=(0.0, 0.0, 0.0),
        blockage_extent_m=0.2,
        candidates=[static_partition, dynamic_chair],
    )

    assert result.success is True
    assert result.match_type == "verified"
    assert result.responsible_object_key == "chair:901"


def test_colocated_static_only_still_chooses_nearest():
    # No dynamic candidate at all -- preference has nothing to prefer, so the
    # existing nearest-bbox-center tie-break is unchanged (no regression).
    farther_static = ObjectCandidate(
        object_key="wall:1",
        object_tag="wall",
        object_state="static",
        safety_class="none",
        openable=False,
        clearable=False,
        bbox_center=(0.1, 0.0, 0.0),
        bbox_extent=(1.0, 1.0, 1.0),
        source="persistent_map",
    )
    nearer_static = ObjectCandidate(
        object_key="partition:121",
        object_tag="room partition",
        object_state="semi-static",
        safety_class="none",
        openable=True,
        clearable=False,
        bbox_center=(0.0, 0.0, 0.0),
        bbox_extent=(1.0, 1.0, 1.0),
        source="persistent_map",
    )

    result = match_responsible_object(
        blockage_centroid=(0.0, 0.0, 0.0),
        blockage_extent_m=0.2,
        candidates=[farther_static, nearer_static],
    )

    assert result.success is True
    assert result.responsible_object_key == "partition:121"

def _s3_partition() -> ObjectCandidate:
    return ObjectCandidate(
        object_key="room partition:121",
        object_tag="room partition",
        object_state="semi-static",
        safety_class="none",
        openable=False,
        clearable=False,
        bbox_center=(-2.507, -1.350, 0.0),
        bbox_extent=(0.200, 0.900, 2.0),
        source="persistent_map",
    )


def _s3_chair(center=(-2.507, -1.350, 0.0)) -> ObjectCandidate:
    return ObjectCandidate(
        object_key="chair:901",
        object_tag="chair",
        object_state="movable",
        safety_class="none",
        openable=False,
        clearable=True,
        bbox_center=center,
        bbox_extent=(0.500, 0.500, 0.9),
        source="dynamic_overlay",
    )


def test_s3_geometry_live_chair_overrides_static_only_containment():
    # EXACT S3 r1 attempt-1 numbers (2026-07-17): the measured centroid is
    # the chair's NEAR FACE (depth marks are surface marks), which misses the
    # chair's small bbox but lands inside the co-located partition's long
    # thin bbox. The partition then verified ALONE and the dynamic-preference
    # tie-break never engaged. A fresh live observation that intersects the
    # static winner's bbox, sits within the fallback radius, and is at least
    # as close to the blockage must take precedence: the detector asserted
    # identity directly, the static record only reflects the map.
    result = match_responsible_object(
        blockage_centroid=(-2.425, -0.925, 0.0),
        blockage_extent_m=0.150,
        candidates=[_s3_partition(), _s3_chair()],
    )

    assert result.success is True
    assert result.responsible_object_key == "chair:901"
    assert result.match_type == "inferred"
    assert "live_static_colocation_precedence" in result.message


def test_unrelated_live_object_does_not_override_verified_static():
    # A sealed door verified at ~0.2m with a person detected 0.7m to the
    # side and NO bbox intersection: the static match must be retained.
    door = ObjectCandidate(
        object_key="door:119",
        object_tag="door",
        object_state="semi-static",
        safety_class="none",
        openable=True,
        clearable=False,
        bbox_center=(0.2, 0.0, 0.0),
        bbox_extent=(0.2, 0.9, 2.0),
        source="persistent_map",
    )
    person = ObjectCandidate(
        object_key="person:902",
        object_tag="person",
        object_state="movable",
        safety_class="human",
        openable=False,
        clearable=False,
        bbox_center=(0.7, 0.7, 0.0),
        bbox_extent=(0.5, 0.5, 1.7),
        source="dynamic_overlay",
    )

    result = match_responsible_object(
        blockage_centroid=(0.0, 0.0, 0.0),
        blockage_extent_m=0.6,
        candidates=[door, person],
    )

    assert result.responsible_object_key == "door:119"
    assert result.match_type == "verified"


def test_farther_live_object_does_not_override_nearer_static():
    # Distance dominance: the live candidate must be at least as close to
    # the blockage as the static winner's center, else static is retained
    # even when the bboxes overlap.
    result = match_responsible_object(
        blockage_centroid=(-2.507, -1.350, 0.0),
        blockage_extent_m=0.2,
        candidates=[_s3_partition(), _s3_chair(center=(-2.507, -2.2, 0.0))],
    )

    assert result.responsible_object_key == "room partition:121"
    assert result.match_type == "verified"


def test_multiple_qualifying_live_objects_is_ambiguous():
    # Two live objects both plausibly explaining the same blockage: refuse
    # to pick arbitrarily.
    chair_a = _s3_chair()
    chair_b = ObjectCandidate(
        object_key="chair:905",
        object_tag="chair",
        object_state="movable",
        safety_class="none",
        openable=False,
        clearable=True,
        # Slightly NEARER the centroid than the static winner's center, so
        # it passes the dominance filter like chair_a and forces a genuine
        # two-way ambiguity.
        bbox_center=(-2.500, -1.340, 0.0),
        bbox_extent=(0.500, 0.500, 0.9),
        source="dynamic_overlay",
    )

    result = match_responsible_object(
        blockage_centroid=(-2.425, -0.925, 0.0),
        blockage_extent_m=0.150,
        candidates=[_s3_partition(), chair_a, chair_b],
    )

    assert result.success is False
    assert result.match_type == "unknown"
    assert "ambiguous" in result.message
