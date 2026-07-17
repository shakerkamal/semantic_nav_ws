"""Unit tests for recovery directive builders."""

from semantic_nav_orchestrator.recovery_directives import (
    LLMProposal,
    ProposalContext,
    build_approach_and_recheck_directive,
    build_retry_target_directive,
    build_wait_then_replan_directive,
    OverrideConfig,
    build_give_up_directive,
    build_open_door_directive,
    build_clear_object_directive,
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
        eligible_actions=("wait_then_replan", "give_up"),
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
        eligible_actions=("wait_then_replan", "give_up"),
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
        eligible_actions=("wait_then_replan", "give_up"),
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
        eligible_actions=("wait_then_replan", "give_up"),
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
        eligible_actions=("wait_then_replan", "give_up"),
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
        eligible_actions=("wait_then_replan", "give_up"),
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
        eligible_actions=("wait_then_replan", "give_up"),
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
        eligible_actions=("wait_then_replan", "give_up"),
    )

    assert directive.action == "give_up"
    assert directive.escalate_to_operator is True
    assert directive.rationale == "LLM returned terminal give_up"
    assert directive.confidence_percent == 55


# ---- build_open_door_directive / build_clear_object_directive ---------------

def test_open_door_directive_sets_action_and_key():
    proposal = LLMProposal(
        action="wait_then_replan",
        rationale="LLM wants to wait.",
        confidence_percent=72,
    )
    directive = build_open_door_directive(proposal, _ctx(), responsible_object_key="door:3")
    assert directive.action == "open_door_then_replan"
    assert directive.responsible_object_key == "door:3"
    assert directive.escalate_to_operator is True
    assert "door:3" in directive.operator_message
    assert "deterministic_override=true" in directive.rationale
    assert "open_door_then_replan" in directive.rationale
    assert directive.confidence_percent == 72
    assert directive.recovery_event_id == "event-1"


def test_open_door_directive_strips_whitespace_from_key():
    proposal = LLMProposal(action="wait_then_replan", confidence_percent=60)
    directive = build_open_door_directive(proposal, _ctx(), responsible_object_key="  door:1  ")
    assert directive.responsible_object_key == "door:1"


def test_clear_object_directive_sets_action_and_key():
    proposal = LLMProposal(
        action="wait_then_replan",
        rationale="LLM wants to wait.",
        confidence_percent=65,
    )
    directive = build_clear_object_directive(proposal, _ctx(), responsible_object_key="box:7")
    assert directive.action == "clear_object_then_replan"
    assert directive.responsible_object_key == "box:7"
    assert directive.escalate_to_operator is True
    assert "box:7" in directive.operator_message
    assert "deterministic_override=true" in directive.rationale
    assert "clear_object_then_replan" in directive.rationale
    assert directive.confidence_percent == 65
    assert directive.recovery_event_id == "event-1"


def test_clear_object_directive_strips_whitespace_from_key():
    proposal = LLMProposal(action="wait_then_replan", confidence_percent=55)
    directive = build_clear_object_directive(proposal, _ctx(), responsible_object_key=" box:2 ")
    assert directive.responsible_object_key == "box:2"

# --- approach_and_recheck directive builder (spec 8.3) ---

def test_approach_directive_carries_standoff():
    proposal = LLMProposal(
        action="approach_and_recheck", rationale="r", confidence_percent=60
    )
    ctx = ProposalContext(0, 3, "none", "semi-static", "evt-1")
    d = build_approach_and_recheck_directive(
        proposal, ctx,
        target_pose=("map", 3.9, -1.3, 0.0),
        responsible_object_key="door:119",
    )
    assert d.action == "approach_and_recheck"
    assert d.target_pose == ("map", 3.9, -1.3, 0.0)
    assert d.responsible_object_key == "door:119"
    assert d.escalate_to_operator is False


def test_approach_directive_without_standoff_gives_up():
    proposal = LLMProposal(action="approach_and_recheck", confidence_percent=60)
    ctx = ProposalContext(0, 3, "none", "semi-static", "evt-1")
    d = build_approach_and_recheck_directive(
        proposal, ctx, target_pose=None, responsible_object_key=""
    )
    assert d.action == "give_up"
    assert d.escalate_to_operator is True


def test_give_up_only_eligible_set_never_substitutes():
    # S3 r1 (2026-07-17): eligible=['give_up'] (single_eligible), the LLM
    # answered give_up @95 -- and the deterministic first-attempt semi-static
    # override STILL issued wait_then_replan for an immovable partition,
    # burning a full recovery round. When the policy already reduced the set
    # to give_up only, the builder must not resurrect an ineligible action.
    proposal = LLMProposal(
        action="give_up",
        rationale="Blocked by a room partition; no safe recovery.",
        confidence_percent=95,
    )

    directive = build_give_up_directive(
        proposal,
        _ctx(safety_class="none", object_state="semi-static", attempts_used=0),
        overrides=_overrides(),
        eligible_actions=("give_up",),
    )

    assert directive.action == "give_up"
    assert directive.escalate_to_operator is True


def test_substitution_requires_wait_in_eligible_set():
    proposal = LLMProposal(
        action="give_up",
        rationale="LLM gave up early.",
        confidence_percent=70,
    )

    directive = build_give_up_directive(
        proposal,
        _ctx(safety_class="human", attempts_used=0),
        overrides=_overrides(),
        eligible_actions=("clear_object_then_replan", "give_up"),
    )

    assert directive.action == "give_up"


def test_substitution_still_permitted_when_wait_is_eligible():
    proposal = LLMProposal(
        action="give_up",
        rationale="LLM gave up early.",
        confidence_percent=70,
    )

    directive = build_give_up_directive(
        proposal,
        _ctx(safety_class="human", attempts_used=0),
        overrides=_overrides(),
        eligible_actions=("wait_then_replan", "give_up"),
    )

    assert directive.action == "wait_then_replan"
    assert directive.emit_signal_during_wait is True


def test_enforce_directive_eligibility_passes_eligible_directive():
    from semantic_nav_orchestrator.recovery_directives import (
        enforce_directive_eligibility,
    )

    proposal = LLMProposal(
        action="give_up", rationale="x", confidence_percent=50
    )
    directive = build_give_up_directive(
        proposal,
        _ctx(safety_class="human", attempts_used=0),
        overrides=_overrides(),
        eligible_actions=("wait_then_replan", "give_up"),
    )

    unchanged = enforce_directive_eligibility(
        directive, ("wait_then_replan", "give_up")
    )
    assert unchanged is directive


def test_enforce_directive_eligibility_converts_ineligible_to_terminal():
    from semantic_nav_orchestrator.recovery_directives import (
        enforce_directive_eligibility,
    )

    proposal = LLMProposal(
        action="give_up", rationale="x", confidence_percent=50
    )
    directive = build_give_up_directive(
        proposal,
        _ctx(safety_class="human", attempts_used=0),
        overrides=_overrides(),
        eligible_actions=("wait_then_replan", "give_up"),
    )
    assert directive.action == "wait_then_replan"

    fixed = enforce_directive_eligibility(directive, ("give_up",))
    assert fixed.action == "give_up"
    assert fixed.escalate_to_operator is True
    assert "directive_not_eligible" in fixed.rationale
