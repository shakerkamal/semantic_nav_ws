"""Unit tests for recovery directive builders."""

from semantic_nav_orchestrator.recovery_directives import (
    LLMProposal,
    ProposalContext,
    build_retry_target_directive,
    build_wait_then_replan_directive,
    OverrideConfig,
    build_give_up_directive,
)


def _ctx(
    safety_class="none",
    object_state="static",
    attempts_used=0,
):
    return ProposalContext(
        attempts_used=attempts_used,
        retry_cap=3,
        responsible_safety_class=safety_class,
        responsible_object_state=object_state,
        recovery_event_id="event-1",
    )

def _overrides():
    return OverrideConfig(
        signal_attempts_default=3,
        short_signal_wait_seconds=2,
        passive_wait_seconds_default=5,
    )



def test_retry_target_uses_resolved_pose():
    proposal = LLMProposal(
        action="retry_target",
        target_object_tag="cabinet",
        target_intent_hint="kitchen food storage alternative",
        rationale="The refrigerator path is blocked.",
        confidence_percent=78,
    )

    resolved_pose = ("map", 1.5, 2.5, 0.0)

    def fake_resolver(tag: str, hint: str):
        assert tag == "cabinet"
        assert hint == "kitchen food storage alternative"
        return resolved_pose, "cabinet:4"

    directive = build_retry_target_directive(
        proposal,
        _ctx(),
        fake_resolver,
    )

    assert directive.action == "retry_target"
    assert directive.target_pose == resolved_pose
    assert directive.target_object_key == "cabinet:4"
    assert directive.target_object_tag == "cabinet"
    assert directive.target_intent_hint == "kitchen food storage alternative"
    assert directive.rationale == "The refrigerator path is blocked."
    assert directive.confidence_percent == 78
    assert directive.escalate_to_operator is False
    assert directive.recovery_event_id == "event-1"


def test_retry_target_falls_back_when_resolver_returns_none():
    proposal = LLMProposal(
        action="retry_target",
        target_object_tag="cabinet",
        target_intent_hint="kitchen food storage alternative",
        rationale="Try cabinet.",
        confidence_percent=70,
    )

    def fake_resolver(tag, hint):
        return None, ""

    directive = build_retry_target_directive(
        proposal,
        _ctx(),
        fake_resolver,
    )

    assert directive.action == "give_up"
    assert directive.escalate_to_operator is True
    assert "unresolved" in directive.rationale.lower()
    assert "cabinet" in directive.rationale
    assert directive.confidence_percent == 70


def test_retry_target_falls_back_when_object_key_empty():
    proposal = LLMProposal(
        action="retry_target",
        target_object_tag="cabinet",
        target_intent_hint="kitchen food storage alternative",
        confidence_percent=65,
    )

    def fake_resolver(tag, hint):
        return ("map", 1.0, 2.0, 0.0), ""

    directive = build_retry_target_directive(
        proposal,
        _ctx(),
        fake_resolver,
    )

    assert directive.action == "give_up"
    assert directive.escalate_to_operator is True
    assert "unresolved" in directive.rationale.lower()


def test_retry_target_falls_back_when_target_tag_empty():
    proposal = LLMProposal(
        action="retry_target",
        target_object_tag="",
        target_intent_hint="food storage",
        confidence_percent=60,
    )

    def fake_resolver(tag, hint):
        raise AssertionError("resolver should not be called for empty tag")

    directive = build_retry_target_directive(
        proposal,
        _ctx(),
        fake_resolver,
    )

    assert directive.action == "give_up"
    assert directive.escalate_to_operator is True
    assert "empty target_object_tag" in directive.rationale

def test_wait_then_replan_for_static_obstacle_no_signal():
    proposal = LLMProposal(
        action="wait_then_replan",
        wait_seconds=4,
        rationale="Wait for clearance.",
        confidence_percent=70,
    )

    directive = build_wait_then_replan_directive(
        proposal,
        _ctx(safety_class="none"),
        signal_attempts_default=3,
        max_wait_seconds=30,
    )

    assert directive.action == "wait_then_replan"
    assert directive.wait_seconds == 4
    assert directive.emit_signal_during_wait is False
    assert directive.signal_attempts == 0
    assert directive.rationale == "Wait for clearance."
    assert directive.confidence_percent == 70
    assert directive.escalate_to_operator is False


def test_wait_then_replan_for_human_sets_signal_flag():
    proposal = LLMProposal(
        action="wait_then_replan",
        wait_seconds=2,
        rationale="Wait for person.",
        confidence_percent=80,
    )

    directive = build_wait_then_replan_directive(
        proposal,
        _ctx(safety_class="human"),
        signal_attempts_default=3,
        max_wait_seconds=30,
    )

    assert directive.action == "wait_then_replan"
    assert directive.wait_seconds == 2
    assert directive.emit_signal_during_wait is True
    assert directive.signal_attempts == 3
    assert directive.escalate_to_operator is False


def test_wait_then_replan_for_animal_sets_signal_flag():
    proposal = LLMProposal(
        action="wait_then_replan",
        wait_seconds=3,
        rationale="Wait for animal.",
        confidence_percent=75,
    )

    directive = build_wait_then_replan_directive(
        proposal,
        _ctx(safety_class="animal"),
        signal_attempts_default=2,
        max_wait_seconds=30,
    )

    assert directive.action == "wait_then_replan"
    assert directive.wait_seconds == 3
    assert directive.emit_signal_during_wait is True
    assert directive.signal_attempts == 2


def test_wait_then_replan_clamps_to_max():
    proposal = LLMProposal(
        action="wait_then_replan",
        wait_seconds=120,
        rationale="Wait too long.",
        confidence_percent=50,
    )

    directive = build_wait_then_replan_directive(
        proposal,
        _ctx(),
        signal_attempts_default=3,
        max_wait_seconds=30,
    )

    assert directive.action == "wait_then_replan"
    assert directive.wait_seconds == 30


def test_wait_then_replan_clamps_negative_to_zero():
    proposal = LLMProposal(
        action="wait_then_replan",
        wait_seconds=-5,
        rationale="Negative wait.",
        confidence_percent=50,
    )

    directive = build_wait_then_replan_directive(
        proposal,
        _ctx(),
        signal_attempts_default=3,
        max_wait_seconds=30,
    )

    assert directive.action == "wait_then_replan"
    assert directive.wait_seconds == 0


def test_wait_then_replan_handles_case_and_whitespace_safety_class():
    proposal = LLMProposal(
        action="wait_then_replan",
        wait_seconds=2,
        rationale="Wait for person.",
        confidence_percent=80,
    )

    directive = build_wait_then_replan_directive(
        proposal,
        _ctx(safety_class=" Human "),
        signal_attempts_default=4,
        max_wait_seconds=30,
    )

    assert directive.emit_signal_during_wait is True
    assert directive.signal_attempts == 4

def test_give_up_terminal_when_attempts_already_used():
    proposal = LLMProposal(
        action="give_up",
        rationale="Recovery budget already used.",
        confidence_percent=80,
    )

    directive = build_give_up_directive(
        proposal,
        _ctx(safety_class="human", attempts_used=2),
        overrides=_overrides(),
    )

    assert directive.action == "give_up"
    assert directive.escalate_to_operator is True
    assert directive.rationale == "Recovery budget already used."
    assert directive.confidence_percent == 80


def test_give_up_terminal_when_no_safe_class_or_state():
    proposal = LLMProposal(
        action="give_up",
        rationale="No safe semantic fallback.",
        confidence_percent=75,
    )

    directive = build_give_up_directive(
        proposal,
        _ctx(safety_class="none", object_state="static", attempts_used=0),
        overrides=_overrides(),
    )

    assert directive.action == "give_up"
    assert directive.escalate_to_operator is True
    assert directive.rationale == "No safe semantic fallback."


def test_give_up_terminal_when_safety_class_has_whitespace_but_attempt_used():
    proposal = LLMProposal(
        action="give_up",
        rationale="Already tried once.",
        confidence_percent=80,
    )

    directive = build_give_up_directive(
        proposal,
        _ctx(safety_class=" Human ", attempts_used=1),
        overrides=_overrides(),
    )

    assert directive.action == "give_up"
    assert directive.escalate_to_operator is True


def test_give_up_converted_to_wait_for_human_on_first_attempt():
    proposal = LLMProposal(
        action="give_up",
        rationale="LLM gave up early.",
        confidence_percent=70,
    )

    directive = build_give_up_directive(
        proposal,
        _ctx(safety_class="human", attempts_used=0),
        overrides=_overrides(),
    )

    assert directive.action == "wait_then_replan"
    assert directive.emit_signal_during_wait is True
    assert directive.signal_attempts == 3
    assert directive.wait_seconds == 2
    assert directive.escalate_to_operator is False
    assert "deterministic_override=true" in directive.rationale
    assert "safety_class=human" in directive.rationale


def test_give_up_converted_to_wait_for_animal_on_first_attempt():
    proposal = LLMProposal(
        action="give_up",
        rationale="LLM gave up early.",
        confidence_percent=70,
    )

    directive = build_give_up_directive(
        proposal,
        _ctx(safety_class=" Animal ", attempts_used=0),
        overrides=_overrides(),
    )

    assert directive.action == "wait_then_replan"
    assert directive.emit_signal_during_wait is True
    assert directive.signal_attempts == 3
    assert directive.wait_seconds == 2
    assert "safety_class=animal" in directive.rationale


def test_give_up_converted_to_passive_wait_for_semi_static():
    proposal = LLMProposal(
        action="give_up",
        rationale="LLM gave up early.",
        confidence_percent=70,
    )

    directive = build_give_up_directive(
        proposal,
        _ctx(safety_class="none", object_state="semi-static", attempts_used=0),
        overrides=_overrides(),
    )

    assert directive.action == "wait_then_replan"
    assert directive.emit_signal_during_wait is False
    assert directive.signal_attempts == 0
    assert directive.wait_seconds == 5
    assert directive.escalate_to_operator is False
    assert "deterministic_override=true" in directive.rationale
    assert "object_state=semi-static" in directive.rationale


def test_give_up_converted_to_passive_wait_for_semistatic_with_whitespace():
    proposal = LLMProposal(
        action="give_up",
        rationale="LLM gave up early.",
        confidence_percent=70,
    )

    directive = build_give_up_directive(
        proposal,
        _ctx(safety_class="none", object_state=" Semi-Static ", attempts_used=0),
        overrides=_overrides(),
    )

    assert directive.action == "wait_then_replan"
    assert directive.emit_signal_during_wait is False
    assert directive.wait_seconds == 5


def test_give_up_uses_default_rationale_when_empty():
    proposal = LLMProposal(
        action="give_up",
        rationale="",
        confidence_percent=55,
    )

    directive = build_give_up_directive(
        proposal,
        _ctx(safety_class="none", object_state="static", attempts_used=0),
        overrides=_overrides(),
    )

    assert directive.action == "give_up"
    assert directive.escalate_to_operator is True
    assert directive.rationale == "LLM returned terminal give_up"
    assert directive.confidence_percent == 55