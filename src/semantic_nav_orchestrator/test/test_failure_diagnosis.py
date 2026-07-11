# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Unit tests for the FailureDiagnosis schema (spec 10)."""

import json

from semantic_nav_orchestrator.failure_diagnosis import FailureDiagnosis, to_log_dict


def _fd(**kw):
    base = dict(
        event_id="e1", failure_stage="validation", nav2_error_code="6",
        original_query="fridge", intent_hint="cold drinks",
        resolved_target_object_key="refrigerator:6", resolved_target_tag="refrigerator",
        original_goal_pose=("map", 7.17, -1.35, 0.0), diagnosis="blocked",
        costmap_source="/global_costmap/costmap", robot_region_id=1, target_region_id=2,
        barrier_centroid=(4.86, -0.68), barrier_extent_m=0.9,
        blocked_cell_fraction=0.2, unknown_cell_fraction=0.0,
        responsible_object_key="door:119", responsible_object_tag="door",
        responsible_state_detail="closed", responsible_traversability="blocked",
        responsible_openable=True, responsible_clearable=False,
        responsible_robot_openable=False, responsible_safety_class="none",
        responsible_match_type="verified", responsible_match_confidence=0.9,
        standoff_pose=("map", 3.9, -1.3, 0.0), standoff_validated=True,
        allowed_actions=["approach_and_recheck", "open_door_then_replan", "give_up"],
        deterministic_override=False, local_db_version=763157730,
    )
    base.update(kw)
    return FailureDiagnosis(**base)


def test_failure_diagnosis_log_dict_roundtrip():
    fd = _fd()
    d = to_log_dict(fd)
    assert d["event_id"] == "e1"
    assert d["allowed_actions"] == [
        "approach_and_recheck", "open_door_then_replan", "give_up"
    ]
    assert d["responsible_openable"] is True
    assert d["deterministic_override"] is False
    # must be JSON-serializable
    json.dumps(d)


def test_failure_diagnosis_allows_none_poses():
    fd = _fd(original_goal_pose=None, standoff_pose=None, barrier_centroid=None)
    d = to_log_dict(fd)
    assert d["standoff_pose"] is None
    json.dumps(d)
