"""Pure directive builders for the /request_recovery handler.

Each builder takes an LLM proposal and orchestrator context and returns a
Directive describing what the BT should do next.

No rclpy or ROS msg imports. The orchestrator handler converts Directive into
RequestRecovery.Response at the ROS boundary.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Callable, Optional, Tuple


# Pure-test representation:
# (frame_id, x, y, yaw_rad)
ResolvedPose = Tuple[str, float, float, float]

# resolver(object_tag, intent_hint) -> (pose_tuple_or_none, object_key)
TargetResolver = Callable[[str, str], Tuple[Optional[ResolvedPose], str]]


@dataclass(frozen=True)
class LLMProposal:
    action: str
    rationale: str = ""
    confidence_percent: int = 0

    # Object-centric retry_target fields.
    target_object_tag: str = ""
    target_intent_hint: str = ""

    # Future / other action fields.
    wait_seconds: int = 0
    operator_message: str = ""
    responsible_object_key: str = ""


@dataclass(frozen=True)
class ProposalContext:
    attempts_used: int
    retry_cap: int
    responsible_safety_class: str
    responsible_object_state: str
    recovery_event_id: str


@dataclass(frozen=True)
class Directive:
    action: str

    # retry_target
    target_pose: Optional[ResolvedPose] = None
    target_object_key: str = ""
    target_object_tag: str = ""
    target_intent_hint: str = ""

    # wait_then_replan
    wait_seconds: int = 0
    emit_signal_during_wait: bool = False
    signal_attempts: int = 0

    # open_door / clear_object
    responsible_object_key: str = ""
    operator_message: str = ""

    # common
    rationale: str = ""
    confidence_percent: int = 0
    escalate_to_operator: bool = False
    recovery_event_id: str = ""


@dataclass(frozen=True)
class OverrideConfig:
    signal_attempts_default: int
    short_signal_wait_seconds: int
    passive_wait_seconds_default: int


def build_approach_and_recheck_directive(
    proposal: LLMProposal,
    context: ProposalContext,
    target_pose: Optional[ResolvedPose],
    responsible_object_key: str,
) -> Directive:
    """Move to a reachable standoff, then retry the original goal (spec 8.3).

    The standoff pose is computed by the orchestrator (not the LLM). If no
    reachable standoff exists, degrade to terminal give_up with escalation.
    """
    if target_pose is None:
        return Directive(
            action="give_up",
            rationale="approach_and_recheck unresolved: no reachable standoff",
            confidence_percent=int(proposal.confidence_percent),
            escalate_to_operator=True,
            recovery_event_id=context.recovery_event_id,
        )
    return Directive(
        action="approach_and_recheck",
        target_pose=target_pose,
        responsible_object_key=(responsible_object_key or "").strip(),
        rationale=proposal.rationale,
        confidence_percent=int(proposal.confidence_percent),
        escalate_to_operator=False,
        recovery_event_id=context.recovery_event_id,
    )


def build_retry_target_directive(
    proposal: LLMProposal,
    context: ProposalContext,
    resolver: TargetResolver,
) -> Directive:
    """Resolve target_object_tag + target_intent_hint to a standoff pose.

    The LLM does not emit an object instance key in object-centric v1.
    It emits a target object tag and intent hint. The orchestrator resolves
    that symbolic target deterministically.

    If resolution fails, return a terminal give_up directive with operator
    escalation instead of making another LLM call.
    """
    target_tag = (proposal.target_object_tag or "").strip()
    target_hint = (proposal.target_intent_hint or "").strip()

    if not target_tag:
        return Directive(
            action="give_up",
            rationale="retry_target unresolved: empty target_object_tag",
            confidence_percent=int(proposal.confidence_percent),
            escalate_to_operator=True,
            recovery_event_id=context.recovery_event_id,
        )

    pose, object_key = resolver(target_tag, target_hint)

    if pose is None or not object_key:
        return Directive(
            action="give_up",
            rationale=(
                f"retry_target unresolved: "
                f"tag='{target_tag}' hint='{target_hint}'"
            ),
            confidence_percent=int(proposal.confidence_percent),
            escalate_to_operator=True,
            recovery_event_id=context.recovery_event_id,
        )

    return Directive(
        action="retry_target",
        target_pose=pose,
        target_object_key=object_key,
        target_object_tag=target_tag,
        target_intent_hint=target_hint,
        rationale=proposal.rationale,
        confidence_percent=int(proposal.confidence_percent),
        escalate_to_operator=False,
        recovery_event_id=context.recovery_event_id,
    )


def build_wait_then_replan_directive(
    proposal: LLMProposal,
    context: ProposalContext,
    signal_attempts_default: int,
    max_wait_seconds: int,
) -> Directive:
    """Build a bounded wait_then_replan directive.

    Human/animal safety classes set emit_signal_during_wait=True so the BT can
    choose the signal-wait-recheck branch. Static/semi-static/non-living objects
    use passive waiting without a signal.
    """
    wait_seconds = max(
        0,
        min(
            int(proposal.wait_seconds),
            int(max_wait_seconds),
        ),
    )

    safety_class = (context.responsible_safety_class or "").strip().lower()
    is_living_obstacle = safety_class in {"human", "animal"}

    signal_attempts = (
        max(0, int(signal_attempts_default))
        if is_living_obstacle
        else 0
    )

    return Directive(
        action="wait_then_replan",
        wait_seconds=wait_seconds,
        emit_signal_during_wait=is_living_obstacle,
        signal_attempts=signal_attempts,
        responsible_object_key="",
        rationale=proposal.rationale,
        confidence_percent=int(proposal.confidence_percent),
        escalate_to_operator=False,
        recovery_event_id=context.recovery_event_id,
    )


def build_give_up_directive(
    proposal: LLMProposal,
    context: ProposalContext,
    overrides: OverrideConfig,
) -> Directive:
    """Build terminal give_up or a bounded deterministic wait override.

    Deterministic override rule:
      attempts_used < 1
      AND (
        responsible_safety_class in {"human", "animal"}
        OR responsible_object_state == "semi-static"
      )

    No internal LLM re-prompt is performed.
    """
    safety_class = (context.responsible_safety_class or "").strip().lower()
    object_state = (context.responsible_object_state or "").strip().lower()

    is_living_obstacle = safety_class in {"human", "animal"}
    is_semistatic = object_state == "semi-static"
    first_attempt = int(context.attempts_used) < 1

    if first_attempt and (is_living_obstacle or is_semistatic):
        if is_living_obstacle:
            return Directive(
                action="wait_then_replan",
                wait_seconds=max(0, int(overrides.short_signal_wait_seconds)),
                emit_signal_during_wait=True,
                signal_attempts=max(0, int(overrides.signal_attempts_default)),
                rationale=(
                    "deterministic_override=true "
                    "original_llm_action=give_up "
                    "override_action=wait_then_replan "
                    f"reason=safety_class={safety_class}"
                ),
                confidence_percent=int(proposal.confidence_percent),
                escalate_to_operator=False,
                recovery_event_id=context.recovery_event_id,
            )

        return Directive(
            action="wait_then_replan",
            wait_seconds=max(0, int(overrides.passive_wait_seconds_default)),
            emit_signal_during_wait=False,
            signal_attempts=0,
            rationale=(
                "deterministic_override=true "
                "original_llm_action=give_up "
                "override_action=wait_then_replan "
                "reason=object_state=semi-static"
            ),
            confidence_percent=int(proposal.confidence_percent),
            escalate_to_operator=False,
            recovery_event_id=context.recovery_event_id,
        )

    return Directive(
        action="give_up",
        rationale=proposal.rationale or "LLM returned terminal give_up",
        confidence_percent=int(proposal.confidence_percent),
        escalate_to_operator=True,
        recovery_event_id=context.recovery_event_id,
    )


def build_open_door_directive(
    proposal: LLMProposal,
    context: ProposalContext,
    responsible_object_key: str,
) -> Directive:
    """Build an open_door_then_replan directive for openable door obstacles.

    Only reached when responsible_openable=True and object tag is 'door'.
    Overrides the LLM's wait_then_replan with operator-escalated door action.
    """
    key = (responsible_object_key or "").strip()
    return Directive(
        action="open_door_then_replan",
        responsible_object_key=key,
        operator_message=(
            f"Please open the door blocking the path (object: {key}), then confirm."
        ),
        rationale=(
            "deterministic_override=true "
            "original_llm_action=wait_then_replan "
            "override_action=open_door_then_replan "
            f"reason=responsible_openable=true key={key}"
        ),
        confidence_percent=int(proposal.confidence_percent),
        escalate_to_operator=True,
        recovery_event_id=context.recovery_event_id,
    )


def build_clear_object_directive(
    proposal: LLMProposal,
    context: ProposalContext,
    responsible_object_key: str,
) -> Directive:
    """Build a clear_object_then_replan directive for clearable non-animate obstacles.

    Only reached when responsible_clearable=True, safety_class not in
    {'human', 'animal'}, and tag not in the animate-object set.
    Overrides the LLM's wait_then_replan with operator-escalated clear action.
    """
    key = (responsible_object_key or "").strip()
    return Directive(
        action="clear_object_then_replan",
        responsible_object_key=key,
        operator_message=(
            f"Please clear the object blocking the path (object: {key}), then confirm."
        ),
        rationale=(
            "deterministic_override=true "
            "original_llm_action=wait_then_replan "
            "override_action=clear_object_then_replan "
            f"reason=responsible_clearable=true key={key}"
        ),
        confidence_percent=int(proposal.confidence_percent),
        escalate_to_operator=True,
        recovery_event_id=context.recovery_event_id,
    )


# M4 (spec 21.3): the closed-door deterministic decider
# (maybe_build_closed_door_directive + build_direct_* helpers) was removed. A
# closed door now yields >=2 eligible actions and the LLM selects among them;
# the deterministic layer filters + overrides invalid picks, it does not decide.
