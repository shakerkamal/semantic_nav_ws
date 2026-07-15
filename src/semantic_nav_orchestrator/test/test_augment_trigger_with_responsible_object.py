# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Unit tests for the dynamic-object trigger-augmentation bug (2026-07-15).

Uses NavigationOrchestrator.__new__ to exercise the method without the full
node __init__ (which needs rclpy + a live ROS graph), mirroring the pattern
already used in semantic_nav_llm/test/test_navigator_recovery_parse.py.
"""
from geometry_msgs.msg import Point

from semantic_nav_orchestrator.navigation_orchestrator import (
    NavigationOrchestrator,
    TriggerInfo,
)


class _DummyLogger:
    def info(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def error(self, *a, **k):
        pass


def _orchestrator(semantic_objects=()):
    node = NavigationOrchestrator.__new__(NavigationOrchestrator)
    node.get_logger = lambda: _DummyLogger()
    node._semantic_objects = list(semantic_objects)
    return node


def test_dynamic_only_key_keeps_supplied_match_not_treated_as_unknown():
    # 2026-07-15: door:903 (a live-perceived, dynamic-only key -- never added
    # to the orchestrator's static catalog by design) was silently swapped
    # for the co-located static door:119 because the old code treated "not
    # in the static catalog" as "unknown" unconditionally, discarding the
    # already-correct match_type/affordances supplied by the caller (which
    # came from /match_responsible_object, which DOES see dynamic
    # candidates). The catalog is empty here to simulate exactly that case.
    node = _orchestrator(semantic_objects=[])
    trigger = TriggerInfo(
        trigger_source="bt_recovery_plugin",
        failure_stage="execution",
        responsible_object_key="door:903",
        responsible_object_tag="door",
        responsible_object_state="semi-static",
        responsible_safety_class="none",
        responsible_openable=True,
        responsible_clearable=False,
        match_type="inferred",
        blockage_centroid=Point(x=4.461, y=-0.595, z=0.0),
        blockage_extent_m=0.6,
    )

    node._augment_trigger_with_responsible_object(trigger)

    assert trigger.responsible_object_key == "door:903"
    assert trigger.match_type == "inferred"
    assert trigger.responsible_openable is True
    assert trigger.responsible_object_tag == "door"


def test_key_with_no_supplied_match_type_still_falls_back_to_unknown():
    # No match_type at all (the TriggerInfo default is "unknown") -- there is
    # nothing to trust, so the existing "not found -> unknown" behavior must
    # be unchanged (no regression for callers that never supply match quality).
    node = _orchestrator(semantic_objects=[])
    trigger = TriggerInfo(
        trigger_source="bt_recovery_plugin",
        failure_stage="execution",
        responsible_object_key="ghost:1",
        match_type="unknown",
        blockage_centroid=Point(x=0.0, y=0.0, z=0.0),
        blockage_extent_m=0.0,
    )

    node._augment_trigger_with_responsible_object(trigger)

    assert trigger.responsible_object_key == ""


def test_key_found_in_static_catalog_still_uses_catalog_geometry():
    # Regression: a genuinely static object must still be looked up and
    # enriched from the catalog as before (this path is unchanged).
    from semantic_nav_orchestrator.navigation_orchestrator import SemanticObject

    obj = SemanticObject(
        key="door:119", object_id=119, tag="door",
        caption="The doorway connecting the hallway to the kitchen.",
        state="semi-static",
        x=4.8622, y=-0.6772, z=1.0,
        extent_x=0.2, extent_y=0.9, extent_z=2.0,
        volume=0.36,
        openable=True, clearable=False, safety_class="none",
    )
    node = _orchestrator(semantic_objects=[obj])
    trigger = TriggerInfo(
        trigger_source="bt_recovery_plugin",
        failure_stage="execution",
        responsible_object_key="door:119",
        match_type="verified",
        blockage_centroid=Point(x=4.86, y=-0.68, z=0.0),
        blockage_extent_m=0.2,
    )

    node._augment_trigger_with_responsible_object(trigger)

    assert trigger.responsible_object_key == "door:119"
    assert trigger.responsible_bbox_center.x == 4.8622
    assert trigger.responsible_openable is True
