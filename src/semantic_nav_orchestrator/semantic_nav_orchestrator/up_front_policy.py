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


def operator_prompt_for(action: str, object_key: str) -> str:
    """Operator prompt text for an up-front operator directive.

    Deterministic (no LLM). Used when the up-front loop executes an
    ``open_door_then_replan`` / ``clear_object_then_replan`` directive: prompt
    the operator, then re-scan and re-validate after they confirm.
    """
    key = (object_key or "").strip() or "the responsible object"
    if action == "open_door_then_replan":
        return f"Please open the door blocking the path (object: {key}), then confirm."
    if action == "clear_object_then_replan":
        return f"Please clear the object blocking the path (object: {key}), then confirm."
    return f"Operator action required for '{action}' (object: {key}), then confirm."


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
    within_verify_range: bool = True,
) -> List[str]:
    """Return the safe/valid directive set for this diagnosis + affordances.

    ``within_verify_range``: operator directives (open/clear) are only *valid*
    when the robot is close enough to the barrier to verify the state change
    afterwards -- costmaps update with line of sight, so a rescan from across
    the house can never confirm a door opened. Far away with a reachable
    standoff, the robot must approach first; the operator actions become
    eligible on the next attempt, once it is near. This is a physical
    constraint, so it lives in the deterministic filter, not in the prompt
    (small LLMs read "the robot is 8.8 m away" and pick open_door anyway).
    Defaults to True: the en-route BT path triggers at the blockage itself.
    """
    animate = aff.safety_class.strip().lower() in _ANIMATE
    if animate:
        # Living obstacle: wait it out; never clear, never drive up aggressively.
        return ["wait_then_replan", "give_up"]

    # Operator actions require post-hoc verification, which requires proximity.
    # Exception: no reachable standoff means the robot can never get close, so
    # excluding them would delete the only remedial action; keep them and let
    # the executor fall back to a best-effort in-place rescan.
    operator_ok = within_verify_range or not has_reachable_standoff

    elig: List[str] = []
    if has_reachable_standoff:
        elig.append("approach_and_recheck")
    if aff.openable and operator_ok:
        elig.append("open_door_then_replan")
    if aff.clearable and operator_ok:
        elig.append("clear_object_then_replan")
    # retry_target is always safe (a different reachable instance), if one exists.
    elig.append("retry_target")
    elig.append("give_up")
    return elig


_DIRECTIVE_PRIORITY = [
    "approach_and_recheck",
    "open_door_then_replan",
    "clear_object_then_replan",
    "wait_then_replan",
    "retry_target",
    "give_up",
]


@dataclass(frozen=True)
class DirectiveSelection:
    action: str
    overridden: bool
    reason: str


def _deterministic_default(eligible: List[str]) -> str:
    for action in _DIRECTIVE_PRIORITY:
        if action in eligible:
            return action
    return "give_up"


def eligible_after_attempts(eligible: List[str], tried_actions) -> List[str]:
    """Drop actions already attempted-and-failed this recovery (except give_up).

    Filter-not-policy: an action that was tried and left the goal blocked is no
    longer *valid* to re-select, so the deterministic layer removes it. This is
    the reliable guard the prompt hint ("do not repeat") cannot provide with a
    small local LLM -- once approach_and_recheck is exhausted the LLM is forced
    to choose among the remaining options (e.g. open_door_then_replan). give_up
    is always kept as the safe terminal; never returns an empty set.
    """
    tried = set(tried_actions or ())
    narrowed = [d for d in eligible if d not in tried or d == "give_up"]
    return narrowed or ["give_up"]


def select_and_override_directive(
    eligible: List[str],
    llm_action: Optional[str],
    aff: ResponsibleAffordances,
    has_reachable_standoff: bool,
) -> DirectiveSelection:
    """Filter-not-policy selection (spec 21.3 + 11.3).

    The deterministic layer supplies ``eligible``; the LLM's ``llm_action`` is
    honored only when it is in the eligible set. Single-eligible needs no LLM;
    an unavailable or ineligible pick falls back to the deterministic priority
    default. Returns the final action plus whether/why it was overridden.
    """
    if not eligible:
        return DirectiveSelection("give_up", True, "no_eligible_actions")
    if len(eligible) == 1:
        return DirectiveSelection(
            eligible[0], llm_action != eligible[0], "single_eligible"
        )
    action = (llm_action or "").strip()
    if not action:
        return DirectiveSelection(
            _deterministic_default(eligible), True, "llm_unavailable"
        )
    if action in eligible:
        return DirectiveSelection(action, False, "llm_selected")
    return DirectiveSelection(
        _deterministic_default(eligible), True, f"override_ineligible:{action}"
    )


def choose_directive(
    diagnosis: str,
    aff: ResponsibleAffordances,
    has_reachable_standoff: bool,
    within_verify_range: bool = True,
) -> str:
    """Deterministically pick one directive from the eligible set."""
    elig = eligible_directives(
        diagnosis, aff, has_reachable_standoff, within_verify_range
    )

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
