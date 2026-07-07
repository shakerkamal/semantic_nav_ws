# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Deterministic up-front recovery policy (filter-not-decider, spec 21.3).

eligible_directives() answers "which actions are safe/valid here?" (the filter,
which M4's LLM keeps). choose_directive() makes a deterministic pick from that
set (which M4's LLM replaces). Keeping them separate is what lets the LLM add
value without loosening safety.
"""

from dataclasses import dataclass
from typing import List, Optional

_ANIMATE = {"human", "animal"}

# object_key stamped on the internal standoff sub-goal of the up-front layer.
STANDOFF_OBJECT_KEY = "__standoff__"


def behavior_tree_for_target(
    object_key: str,
    semantic_bt: str,
    standoff_bt: str,
    standoff_object_key: str = STANDOFF_OBJECT_KEY,
) -> str:
    """Pick the Nav2 behavior_tree for a goal, keeping the up-front recovery
    DETERMINISTIC (no LLM).

    The standoff approach is an internal maneuver of the deterministic up-front
    layer, so it must NOT run the semantic recovery BT -- whose Tier-3
    EscalateToLLMRecovery would pull the LLM into a path documented "no LLM"
    and, when the LLM is down, block ~30 s then abort the approach. The standoff
    uses ``standoff_bt`` (a plain BT: geometric recovery only; empty string ->
    Nav2's configured default). Every other goal -- including the real target
    re-dispatched once the barrier clears -- keeps ``semantic_bt``.
    """
    if object_key == standoff_object_key:
        return standoff_bt
    return semantic_bt


def barrier_cleared_status(
    lethal_fraction: Optional[float],
    observed_cells: int,
    clear_max_lethal_fraction: float,
    min_observed_cells: int,
) -> str:
    """Object-agnostic confirmation that the responsible barrier has cleared.

    Used at the standoff, after re-observing, before committing to the original
    goal. Instead of asking "is this DOOR open?", it asks the generic question
    "is the barrier's footprint now free of a real obstacle?" -- true for any
    dynamic obstacle that opened, moved, or was removed, not just doors. This is
    what keeps the recovery a dynamic-obstacle system rather than a door system;
    a door-state sensor, if present, is an optional richer signal layered on top.

    ``lethal_fraction`` is the fraction of KNOWN cells inside the barrier
    footprint that are still true obstacles (see barrier_lethal_fraction in
    global_blockage_diagnosis). Returns one of:
      - "cleared"       footprint lethal fraction is at/below the threshold.
      - "still_blocked" footprint is still occupied above the threshold.
      - "unconfirmed"   too few known cells sampled (couldn't observe it).

    Only "cleared" is positive confirmation; the caller still requires a valid
    plan as well.
    """
    if lethal_fraction is None or observed_cells < min_observed_cells:
        return "unconfirmed"
    if lethal_fraction <= clear_max_lethal_fraction:
        return "cleared"
    return "still_blocked"


@dataclass(frozen=True)
class ResponsibleAffordances:
    """Affordances of the barrier's responsible object (from M1 overlay/match)."""

    tag: str
    openable: bool
    clearable: bool
    safety_class: str   # human | animal | none
    match_type: str     # verified | inferred | none


def eligible_directives(
    diagnosis: str,
    aff: ResponsibleAffordances,
    has_reachable_standoff: bool,
) -> List[str]:
    """Return the safe/valid directive set for this diagnosis + affordances."""
    animate = aff.safety_class.strip().lower() in _ANIMATE
    if animate:
        # Living obstacle: wait it out; never clear, never drive up aggressively.
        return ["wait_then_replan", "give_up"]

    elig: List[str] = []
    if has_reachable_standoff:
        elig.append("approach_and_recheck")
    if aff.openable:
        elig.append("open_door_then_replan")
    if aff.clearable:
        elig.append("clear_object_then_replan")
    # retry_target is always safe (a different reachable instance), if one exists.
    elig.append("retry_target")
    elig.append("give_up")
    return elig


def choose_directive(
    diagnosis: str,
    aff: ResponsibleAffordances,
    has_reachable_standoff: bool,
) -> str:
    """Deterministically pick one directive from the eligible set."""
    elig = eligible_directives(diagnosis, aff, has_reachable_standoff)

    if "wait_then_replan" in elig and aff.safety_class.strip().lower() in _ANIMATE:
        return "wait_then_replan"

    openable_or_clearable = aff.openable or aff.clearable
    unknown = aff.match_type.strip().lower() == "none"

    # A barrier that can change (door/box/unknown) is worth approaching if we can.
    if "approach_and_recheck" in elig and (openable_or_clearable or unknown):
        return "approach_and_recheck"

    # Openable/clearable but no reachable standoff -> hand to the operator.
    if aff.openable and "open_door_then_replan" in elig:
        return "open_door_then_replan"
    if aff.clearable and "clear_object_then_replan" in elig:
        return "clear_object_then_replan"

    # Structural / immovable / unknown-without-standoff -> concede (retry_target
    # needs a known reachable alternative, which M3 does not yet enumerate).
    return "give_up"
