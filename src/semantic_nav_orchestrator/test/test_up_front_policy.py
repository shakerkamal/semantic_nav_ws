# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Unit tests for the deterministic up-front affordance policy."""

from semantic_nav_orchestrator.up_front_policy import (
    ResponsibleAffordances,
    barrier_cleared_status,
    choose_directive,
    eligible_directives,
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
