"""Unit tests for the responsible-object matcher pure function."""

from typing import Tuple

from semantic_nav_orchestrator.responsible_object_matcher import (
    ObjectCandidate,
    match_responsible_object,
)


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