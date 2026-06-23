# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Unit tests for maybe_build_closed_door_directive."""

from semantic_nav_orchestrator.recovery_directives import (
    maybe_build_closed_door_directive,
)


class _Trigger:
    """Minimal TriggerInfo-like object for testing."""
    def __init__(
        self,
        tag="door",
        state_detail="",
        traversability="",
        openable=False,
    ):
        self.responsible_object_tag = tag
        self.responsible_state_detail = state_detail
        self.responsible_traversability = traversability
        self.responsible_openable = openable


# ---------------------------------------------------------------------------
# Closed + blocked → passive wait
# ---------------------------------------------------------------------------

def test_closed_blocked_non_openable_returns_wait_then_replan():
    t = _Trigger(
        tag="door",
        state_detail="closed",
        traversability="blocked",
        openable=False,
    )
    d = maybe_build_closed_door_directive(t)
    assert d is not None
    assert d.action == "wait_then_replan"


def test_closed_blocked_non_openable_no_signal():
    t = _Trigger(tag="door", state_detail="closed",
                 traversability="blocked", openable=False)
    d = maybe_build_closed_door_directive(t)
    assert d.emit_signal_during_wait is False
    assert d.signal_attempts == 0


def test_wait_seconds_is_positive():
    t = _Trigger(tag="door", state_detail="closed",
                 traversability="blocked", openable=False)
    d = maybe_build_closed_door_directive(t)
    assert d.wait_seconds > 0


# ---------------------------------------------------------------------------
# Closed + blocked → give_up when robot_openable
# ---------------------------------------------------------------------------

def test_closed_blocked_openable_gives_give_up():
    t = _Trigger(
        tag="door",
        state_detail="closed",
        traversability="blocked",
        openable=True,
    )
    d = maybe_build_closed_door_directive(t)
    assert d is not None
    assert d.action == "give_up"
    assert d.escalate_to_operator is True


# ---------------------------------------------------------------------------
# Non-matching cases — returns None
# ---------------------------------------------------------------------------

def test_non_door_tag_returns_none():
    t = _Trigger(tag="chair", state_detail="closed",
                 traversability="blocked", openable=False)
    assert maybe_build_closed_door_directive(t) is None


def test_open_passable_door_returns_none():
    t = _Trigger(tag="door", state_detail="open",
                 traversability="passable", openable=False)
    assert maybe_build_closed_door_directive(t) is None


def test_unknown_state_returns_none():
    t = _Trigger(tag="door", state_detail="unknown",
                 traversability="unknown", openable=False)
    assert maybe_build_closed_door_directive(t) is None


def test_empty_state_returns_none():
    t = _Trigger(tag="door", state_detail="", traversability="", openable=False)
    assert maybe_build_closed_door_directive(t) is None


# ---------------------------------------------------------------------------
# Partial matches — either field sufficient to trigger
# ---------------------------------------------------------------------------

def test_closed_only_state_detail_triggers():
    t = _Trigger(tag="door", state_detail="closed",
                 traversability="", openable=False)
    d = maybe_build_closed_door_directive(t)
    assert d is not None
    assert d.action == "wait_then_replan"


def test_blocked_only_traversability_triggers():
    t = _Trigger(tag="door", state_detail="",
                 traversability="blocked", openable=False)
    d = maybe_build_closed_door_directive(t)
    assert d is not None
    assert d.action == "wait_then_replan"


# ---------------------------------------------------------------------------
# Compound tag with "door" in it
# ---------------------------------------------------------------------------

def test_compound_door_tag_matches():
    t = _Trigger(tag="closet door", state_detail="closed",
                 traversability="blocked", openable=False)
    d = maybe_build_closed_door_directive(t)
    assert d is not None


# ---------------------------------------------------------------------------
# No versioned map update implied (policy only returns Directive, not a path)
# ---------------------------------------------------------------------------

def test_directive_does_not_contain_map_path():
    t = _Trigger(tag="door", state_detail="closed",
                 traversability="blocked", openable=False)
    d = maybe_build_closed_door_directive(t)
    assert not hasattr(d, "new_map_path")
