# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Unit tests for the navigation terminal operator escalation menu."""

import queue

from semantic_nav_interfaces.srv import OperatorDecision

from semantic_nav_orchestrator.navigation_terminal import (
    NavigationTerminal,
    _next_command,
    _operator_escalation,
)


def _run(choices):
    q = queue.Queue()
    for c in choices:
        q.put(c)
    return _operator_escalation(None, q, "refrigerator:6", "recovery exhausted")


def test_escalation_option_1_retries_original_goal():
    assert _run(["1"]) == ("retry", None)
    assert _run(["retry"]) == ("retry", None)


def test_escalation_option_4_aborts():
    assert _run(["4"]) == ("abort", None)
    assert _run([""]) == ("abort", None)  # empty -> abort


def test_escalation_option_2_navigates_to_new_destination():
    assert _run(["2", "chair:2"]) == ("navigate", "chair:2")


def test_escalation_option_3_teleop_returns_abort():
    assert _run(["3"]) == ("abort", None)


def test_escalation_eof_exits():
    assert _run([None]) == ("exit", None)


def _make_idle_terminal():
    """Terminal with only the queue plumbing set up (no ROS node)."""
    node = NavigationTerminal.__new__(NavigationTerminal)
    node._op_req_q = queue.Queue()
    node._op_resp_q = queue.Queue()
    return node


def test_idle_operator_prompt_is_answered_before_next_command():
    # An en-route trial dispatched by the eval harness (not typed into the
    # terminal) raises /operator_decision while the controller sits idle in
    # its command wait — the prompt must still be served from there.
    node = _make_idle_terminal()
    req = OperatorDecision.Request()
    req.prompt_text = "Open the door"
    req.responsible_object_key = "door:903"
    req.directive_action = "open_door_then_replan"
    node._op_req_q.put(req)

    cmd_q = queue.Queue()
    cmd_q.put("y")        # answers the operator prompt
    cmd_q.put("chair:2")  # the next real command

    assert _next_command(node, cmd_q) == "chair:2"
    assert node._op_resp_q.get_nowait() == (True, "operator_confirmed")


def test_next_command_passes_through_plain_commands():
    node = _make_idle_terminal()
    cmd_q = queue.Queue()
    cmd_q.put("bed:120")
    assert _next_command(node, cmd_q) == "bed:120"
    assert node._op_resp_q.empty()
