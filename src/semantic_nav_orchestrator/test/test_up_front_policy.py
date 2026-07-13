# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Unit tests for the deterministic up-front affordance policy."""

from semantic_nav_orchestrator.up_front_policy import (
    STANDOFF_OBJECT_KEY,
    DirectiveSelection,
    ResponsibleAffordances,
    barrier_cleared_status,
    behavior_tree_for_target,
    choose_directive,
    eligible_directives,
    select_and_override_directive,
)


def _aff(tag="door", openable=False, clearable=False, safety="none", match="verified"):
    return ResponsibleAffordances(tag, openable, clearable, safety, match)


def test_door_eligible_set_and_pick():
    aff = _aff(tag="door", openable=True, match="verified")
    elig = eligible_directives("blocked", aff, has_reachable_standoff=True)
    assert "approach_and_recheck" in elig
    assert "open_door_then_replan" in elig
    assert "clear_object_then_replan" not in elig      # not clearable -> forbidden
    assert choose_directive("blocked", aff, True) == "approach_and_recheck"


def test_door_without_standoff_falls_back_to_operator():
    aff = _aff(tag="door", openable=True)
    assert "approach_and_recheck" not in eligible_directives("blocked", aff, False)
    assert choose_directive("blocked", aff, False) == "open_door_then_replan"


def test_operator_actions_gated_on_verify_range():
    # Far from the barrier with a reachable standoff: the robot could never
    # verify an operator action, so open/clear are NOT eligible -- it must
    # approach first. They become eligible once within verify range.
    aff = _aff(tag="door", openable=True, clearable=True, match="verified")
    far = eligible_directives(
        "blocked", aff, has_reachable_standoff=True, within_verify_range=False
    )
    assert "open_door_then_replan" not in far
    assert "clear_object_then_replan" not in far
    assert "approach_and_recheck" in far

    near = eligible_directives(
        "blocked", aff, has_reachable_standoff=True, within_verify_range=True
    )
    assert "open_door_then_replan" in near
    assert "clear_object_then_replan" in near


def test_operator_actions_kept_when_no_standoff_even_if_far():
    # No reachable standoff: the robot can never get close, so excluding the
    # operator actions would delete the only remedial option. They stay
    # eligible (executor falls back to a best-effort in-place rescan).
    aff = _aff(tag="door", openable=True)
    elig = eligible_directives(
        "blocked", aff, has_reachable_standoff=False, within_verify_range=False
    )
    assert "open_door_then_replan" in elig


def test_clearable_box_pick():
    aff = _aff(tag="box", clearable=True, match="verified")
    assert choose_directive("blocked", aff, True) == "approach_and_recheck"
    assert "clear_object_then_replan" in eligible_directives("blocked", aff, True)


def test_animate_never_clears():
    aff = _aff(tag="person", clearable=False, safety="human")
    elig = eligible_directives("blocked", aff, True)
    assert "clear_object_then_replan" not in elig
    assert choose_directive("blocked", aff, True) == "wait_then_replan"


def test_structural_wall_gives_up():
    aff = _aff(tag="wall", openable=False, clearable=False, match="verified")
    assert choose_directive("blocked", aff, True) == "give_up"


def test_unknown_barrier_approaches_then_gives_up():
    aff = _aff(tag="", match="none")
    assert choose_directive("blocked", aff, True) == "approach_and_recheck"
    assert choose_directive("blocked", aff, False) == "give_up"


def test_unknown_frontier_prefers_approach():
    aff = _aff(tag="", match="none")
    assert choose_directive("unknown_frontier", aff, True) == "approach_and_recheck"


# --- generic barrier-cleared gate (object-agnostic footprint check) ---

def test_barrier_cleared_below_threshold_is_cleared():
    # Footprint mostly free after re-observe -> the obstacle is gone.
    assert barrier_cleared_status(
        0.0, observed_cells=80, clear_max_lethal_fraction=0.15, min_observed_cells=8
    ) == "cleared"
    assert barrier_cleared_status(
        0.10, observed_cells=80, clear_max_lethal_fraction=0.15, min_observed_cells=8
    ) == "cleared"


def test_barrier_still_occupied_is_still_blocked():
    # Footprint still holds a real obstacle -> do not proceed.
    assert barrier_cleared_status(
        0.60, observed_cells=80, clear_max_lethal_fraction=0.15, min_observed_cells=8
    ) == "still_blocked"


def test_barrier_too_few_cells_is_unconfirmed():
    # Couldn't observe enough of the footprint (e.g. out of costmap bounds).
    assert barrier_cleared_status(
        0.0, observed_cells=2, clear_max_lethal_fraction=0.15, min_observed_cells=8
    ) == "unconfirmed"
    assert barrier_cleared_status(
        None, observed_cells=0, clear_max_lethal_fraction=0.15, min_observed_cells=8
    ) == "unconfirmed"


# --- behavior-tree selection: keep the standoff approach LLM-free ---

def test_standoff_target_gets_plain_bt():
    # The deterministic standoff maneuver must NOT carry the semantic (LLM) BT.
    bt = behavior_tree_for_target(
        STANDOFF_OBJECT_KEY, semantic_bt="/sem.xml", standoff_bt="",
    )
    assert bt == ""  # empty -> Nav2 default (geometric recovery, no LLM)


def test_real_target_keeps_semantic_bt():
    # The real goal (incl. re-dispatch after the barrier clears) keeps the
    # semantic recovery BT so its Tier-3 LLM escalation is still available.
    bt = behavior_tree_for_target(
        "refrigerator:6", semantic_bt="/sem.xml", standoff_bt="",
    )
    assert bt == "/sem.xml"


def test_standoff_plain_bt_can_be_an_explicit_path():
    bt = behavior_tree_for_target(
        STANDOFF_OBJECT_KEY, semantic_bt="/sem.xml", standoff_bt="/plain.xml",
    )
    assert bt == "/plain.xml"


# --- approach_and_recheck eligibility (spec 8.4) ---

def test_structural_barrier_no_standoff_excludes_approach():
    aff = _aff(tag="", openable=False, clearable=False, match="none")
    elig = eligible_directives("blocked", aff, has_reachable_standoff=False)
    assert "approach_and_recheck" not in elig
    assert "give_up" in elig


def test_reachable_nonstructural_includes_approach():
    aff = _aff(tag="door", openable=True, match="verified")
    elig = eligible_directives("blocked", aff, has_reachable_standoff=True)
    assert "approach_and_recheck" in elig
    assert "open_door_then_replan" in elig
    assert len(elig) >= 2  # LLM is load-bearing here


# --- select_and_override_directive (filter-not-policy, spec 21.3 / 11.3) ---

def test_llm_pick_honored_when_eligible():
    elig = ["approach_and_recheck", "retry_target", "give_up"]
    aff = _aff(tag="door", openable=True)
    sel = select_and_override_directive(elig, "approach_and_recheck", aff, True)
    assert sel == DirectiveSelection("approach_and_recheck", False, "llm_selected")


def test_ineligible_llm_pick_overridden():
    elig = ["wait_then_replan", "give_up"]
    aff = _aff(tag="person", safety="human")
    sel = select_and_override_directive(elig, "clear_object_then_replan", aff, False)
    assert sel.action == "wait_then_replan"
    assert sel.overridden is True
    assert sel.reason.startswith("override_ineligible")


def test_single_eligible_needs_no_llm():
    sel = select_and_override_directive(["give_up"], None, _aff(tag=""), False)
    assert sel.action == "give_up"
    assert sel.reason == "single_eligible"


def test_llm_unavailable_uses_priority_default():
    elig = ["approach_and_recheck", "retry_target", "give_up"]
    sel = select_and_override_directive(elig, None, _aff(tag="door", openable=True), True)
    assert sel.action == "approach_and_recheck"  # highest priority in eligible
    assert sel.overridden is True
    assert sel.reason == "llm_unavailable"


def test_empty_eligible_gives_up():
    sel = select_and_override_directive([], "retry_target", _aff(tag=""), False)
    assert sel == DirectiveSelection("give_up", True, "no_eligible_actions")


# --- operator prompt text (up-front open-door/clear loop) ---

def test_operator_prompt_for_open_door():
    from semantic_nav_orchestrator.up_front_policy import operator_prompt_for
    msg = operator_prompt_for("open_door_then_replan", "door:119")
    assert "open" in msg.lower()
    assert "door:119" in msg


def test_operator_prompt_for_clear_object():
    from semantic_nav_orchestrator.up_front_policy import operator_prompt_for
    msg = operator_prompt_for("clear_object_then_replan", "box:7")
    assert "clear" in msg.lower()
    assert "box:7" in msg


# --- exhaustion narrowing: drop already-tried actions (except give_up) ---

def test_eligible_after_attempts_drops_tried_action():
    from semantic_nav_orchestrator.up_front_policy import eligible_after_attempts
    elig = ["approach_and_recheck", "open_door_then_replan", "retry_target", "give_up"]
    out = eligible_after_attempts(elig, {"approach_and_recheck"})
    assert "approach_and_recheck" not in out
    assert out == ["open_door_then_replan", "retry_target", "give_up"]


def test_eligible_after_attempts_keeps_give_up_always():
    from semantic_nav_orchestrator.up_front_policy import eligible_after_attempts
    elig = ["approach_and_recheck", "open_door_then_replan", "retry_target", "give_up"]
    tried = {"approach_and_recheck", "open_door_then_replan", "retry_target", "give_up"}
    assert eligible_after_attempts(elig, tried) == ["give_up"]


def test_eligible_after_attempts_no_tried_is_unchanged():
    from semantic_nav_orchestrator.up_front_policy import eligible_after_attempts
    elig = ["approach_and_recheck", "give_up"]
    assert eligible_after_attempts(elig, set()) == elig
