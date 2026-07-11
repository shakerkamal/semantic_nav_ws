# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Unit tests for the navigation terminal operator escalation menu."""

import queue

from semantic_nav_orchestrator.navigation_terminal import _operator_escalation


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
