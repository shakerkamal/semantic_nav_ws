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


def _parsed_action(key="door:119", message="Open the door"):
    from semantic_nav_llm.navigator_node import ParsedRecoveryAction
    return ParsedRecoveryAction(
        action="open_door_then_replan",
        target_object_tag="",
        target_intent_hint="",
        waypoints=[],
        wait_seconds=0,
        responsible_object_key=key,
        operator_message=message,
        rationale="",
        confidence=90,
    )


def _request(match_type, key="door:119", safety_class="none"):
    from types import SimpleNamespace
    return SimpleNamespace(
        match_type=match_type,
        responsible_object_key=key,
        responsible_safety_class=safety_class,
    )


def test_operator_action_rejects_unknown_match_type():
    # 2026-07-15: real-world detection/matching noise means a strict
    # "verified only" gate would reject correct picks far more often than in
    # simulation -- but "unknown" (no plausible match at all) must still be
    # refused; there is nothing to safely act on.
    node = NavigatorNode.__new__(NavigatorNode)
    error = node._validate_operator_object_action_common(
        parsed=_parsed_action(), request=_request("unknown"),
        action_name="open_door_then_replan",
    )
    assert error is not None
    assert "not verified" in error or "unknown" in error.lower()


def test_operator_action_accepts_verified_match_type():
    node = NavigatorNode.__new__(NavigatorNode)
    error = node._validate_operator_object_action_common(
        parsed=_parsed_action(), request=_request("verified"),
        action_name="open_door_then_replan",
    )
    assert error is None


def test_operator_action_accepts_inferred_match_type():
    # THE fix: "inferred" already means "within a bounded proximity radius of
    # the blockage centroid" (responsible_object_matcher's
    # inferred_fallback_radius_m), not "anywhere in the map" -- it is a real,
    # existing confidence tier, not a free-for-all. Requiring exact inflated
    # -bbox containment (verified) before ANY operator action is eligible is
    # an unrealistic bar once real sensor/geometry noise is in play (a few cm
    # of centroid error against e.g. a 0.2m-thick door is enough to miss
    # "verified" every time) -- confirmed live 2026-07-15, S2 (a match at
    # 0.41m, a few mm outside the inflated bbox, was rejected outright).
    node = NavigatorNode.__new__(NavigatorNode)
    error = node._validate_operator_object_action_common(
        parsed=_parsed_action(), request=_request("inferred"),
        action_name="open_door_then_replan",
    )
    assert error is None


def test_operator_action_still_rejects_mismatched_object_key():
    # Loosening the match_type gate must not loosen the OTHER checks.
    node = NavigatorNode.__new__(NavigatorNode)
    error = node._validate_operator_object_action_common(
        parsed=_parsed_action(key="door:119"),
        request=_request("inferred", key="door:999"),
        action_name="open_door_then_replan",
    )
    assert error is not None
    assert "does not match" in error


def test_operator_action_still_rejects_unsafe_safety_class():
    node = NavigatorNode.__new__(NavigatorNode)
    error = node._validate_operator_object_action_common(
        parsed=_parsed_action(),
        request=_request("verified", safety_class="human"),
        action_name="open_door_then_replan",
    )
    assert error is not None
    assert "safety_class" in error


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
