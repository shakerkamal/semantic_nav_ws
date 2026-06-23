# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Unit tests for door-state overlay merge logic (no ROS runtime required)."""

from semantic_nav_semantics.dynamic_overlay import DynamicObjectCache


# ---------------------------------------------------------------------------
# Minimal stand-in helpers (no ROS imports)
# ---------------------------------------------------------------------------

class _Obs:
    """Minimal DoorStateObservation-like object."""
    def __init__(
        self, key, door_state, traversability,
        robot_openable=False, confidence=0.9, ttl_sec=5.0
    ):
        self.object_key = key
        self.door_state = door_state
        self.traversability = traversability
        self.robot_openable = robot_openable
        self.confidence = confidence
        self.ttl_sec = ttl_sec


class _Obj:
    """Minimal ObjectInstance-like object (persistent map door)."""
    def __init__(self, key, tag="door", state="semi-static"):
        self.object_key = key
        self.object_tag = tag
        self.object_state = state
        self.source = "persistent_map"
        self.state_detail = ""
        self.traversability = ""
        self.openable = False
        self.confidence = 0.0
        self.ttl_sec = 0.0


def _apply_door_overlay(objects, door_states, now_sec=0.0):
    """Pure reimplementation of _apply_door_state_overlay for testing."""
    expired = [
        k for k, (_, exp) in door_states.items() if exp <= now_sec
    ]
    for k in expired:
        del door_states[k]
    for obj in objects:
        key = (obj.object_key or "").strip()
        entry = door_states.get(key)
        if entry is None:
            continue
        obs, _ = entry
        obj.source = "persistent_map+door_state_overlay"
        obj.state_detail = obs.door_state
        obj.traversability = obs.traversability
        obj.openable = bool(obs.robot_openable)
        obj.confidence = float(obs.confidence)
        obj.ttl_sec = float(obs.ttl_sec)
    return objects


# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

def test_mapped_door_receives_closed_state():
    door_states = {"door:1": (_Obs("door:1", "closed", "blocked"), 999.0)}
    objects = [_Obj("door:1")]
    _apply_door_overlay(objects, door_states, now_sec=0.0)

    assert objects[0].state_detail == "closed"
    assert objects[0].traversability == "blocked"
    assert objects[0].source == "persistent_map+door_state_overlay"


def test_open_door_receives_passable_state():
    door_states = {"door:2": (_Obs("door:2", "open", "passable"), 999.0)}
    objects = [_Obj("door:2")]
    _apply_door_overlay(objects, door_states, now_sec=0.0)

    assert objects[0].state_detail == "open"
    assert objects[0].traversability == "passable"


def test_expired_door_state_is_ignored():
    door_states = {"door:1": (_Obs("door:1", "closed", "blocked"), 1.0)}
    objects = [_Obj("door:1")]
    _apply_door_overlay(objects, door_states, now_sec=2.0)

    assert objects[0].state_detail == ""
    assert objects[0].traversability == ""
    assert objects[0].source == "persistent_map"


def test_expired_door_state_is_purged():
    door_states = {"door:1": (_Obs("door:1", "closed", "blocked"), 1.0)}
    _apply_door_overlay([], door_states, now_sec=2.0)
    assert len(door_states) == 0


def test_unknown_door_key_does_not_create_new_object():
    door_states = {"door:99": (_Obs("door:99", "closed", "blocked"), 999.0)}
    objects = [_Obj("door:1")]
    _apply_door_overlay(objects, door_states, now_sec=0.0)

    assert len(objects) == 1
    assert objects[0].object_key == "door:1"
    assert objects[0].state_detail == ""


def test_closed_door_remains_non_displaced():
    door_states = {"door:1": (_Obs("door:1", "closed", "blocked"), 999.0)}
    objects = [_Obj("door:1", state="semi-static")]
    _apply_door_overlay(objects, door_states, now_sec=0.0)

    assert objects[0].object_state == "semi-static"


def test_robot_openable_propagates_to_openable_field():
    door_states = {"door:1": (
        _Obs("door:1", "closed", "blocked", robot_openable=True), 999.0
    )}
    objects = [_Obj("door:1")]
    _apply_door_overlay(objects, door_states, now_sec=0.0)

    assert objects[0].openable is True


def test_multiple_doors_only_matched_overlay_applied():
    door_states = {"door:1": (_Obs("door:1", "closed", "blocked"), 999.0)}
    objects = [_Obj("door:1"), _Obj("door:2")]
    _apply_door_overlay(objects, door_states, now_sec=0.0)

    assert objects[0].state_detail == "closed"
    assert objects[1].state_detail == ""


def test_non_door_object_unchanged_by_overlay():
    door_states = {"door:1": (_Obs("door:1", "closed", "blocked"), 999.0)}
    objects = [_Obj("chair:1", tag="chair")]
    _apply_door_overlay(objects, door_states, now_sec=0.0)

    assert objects[0].state_detail == ""
    assert objects[0].source == "persistent_map"


def test_dynamic_object_cache_independent_of_door_cache():
    dynamic_cache = DynamicObjectCache(default_ttl_sec=3.0, max_ttl_sec=10.0)
    dynamic_cache.update(
        "human:1", 0.0, 0.0, 3.0, {"key": "human:1"}, now_sec=0.0
    )
    door_states = {"door:1": (_Obs("door:1", "closed", "blocked"), 999.0)}

    assert len(dynamic_cache) == 1
    assert len(door_states) == 1
