# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Unit tests for navigator recovery-output parsing (no ROS runtime).

Uses NavigatorNode.__new__ to exercise the pure parse method without the full
node __init__ (which needs rclpy + llama_ros).
"""

from semantic_nav_llm.navigator_node import NavigatorNode


class _DummyLogger:
    def error(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def info(self, *a, **k):
        pass


def _parser():
    node = NavigatorNode.__new__(NavigatorNode)
    node._allow_json_extraction_fallback = False
    node.get_logger = lambda: _DummyLogger()
    return node


def test_parse_approach_and_recheck():
    node = _parser()
    parsed = node._parse_recovery_output(
        '{"action":"approach_and_recheck",'
        '"rationale":"move to a standoff and recheck","confidence":70}'
    )
    assert parsed is not None
    assert parsed.action == "approach_and_recheck"
    assert parsed.confidence == 70


def test_parse_give_up_still_maps_to_give_up():
    # The fallthrough must not turn every non-branch action into give_up; but a
    # genuine give_up still parses as give_up.
    node = _parser()
    parsed = node._parse_recovery_output(
        '{"action":"give_up","rationale":"no safe recovery","confidence":40}'
    )
    assert parsed is not None
    assert parsed.action == "give_up"
    assert parsed.confidence == 40


def test_prompt_eligibility_uses_allowed_actions():
    from types import SimpleNamespace
    node = NavigatorNode.__new__(NavigatorNode)
    req = SimpleNamespace(
        allowed_actions=["approach_and_recheck", "retry_target", "give_up"]
    )
    text = node._render_action_eligibility(req)
    assert "approach_and_recheck: ELIGIBLE" in text
    assert "retry_target: ELIGIBLE" in text
    assert "open_door_then_replan: INELIGIBLE" in text  # not in allowed_actions
    assert "give_up: ELIGIBLE" in text


def test_validate_fills_approach_and_recheck_response():
    # Regression: approach_and_recheck must have a response validator, else the
    # /propose_recovery handler KeyErrors and never replies (240s orchestrator
    # timeout). Mirrors give_up (no geometry; orchestrator computes standoff).
    from types import SimpleNamespace
    from semantic_nav_llm.navigator_node import ParsedRecoveryAction
    node = NavigatorNode.__new__(NavigatorNode)
    node.get_logger = lambda: _DummyLogger()
    node._min_confidence_percent = 60
    parsed = ParsedRecoveryAction(
        action="approach_and_recheck", target_object_tag="", target_intent_hint="",
        waypoints=[], wait_seconds=0, responsible_object_key="", operator_message="",
        rationale="approach the door and recheck", confidence=80,
    )
    response = SimpleNamespace()
    result = node._validate_recovery_and_fill_response(
        response=response, parsed=parsed, request=SimpleNamespace(), raw_output="{}"
    )
    assert result.success is True
    assert result.action == "approach_and_recheck"
