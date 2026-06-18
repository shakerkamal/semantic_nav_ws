"""Unit tests for OperatorIONode service logic (no ROS spin required)."""

import io

from semantic_nav_operator_io.operator_io_node import _decide


def test_auto_ack_returns_true():
    result = _decide(
        prompt_text="Please open the door.",
        responsible_object_key="door:1",
        failure_stage="execution",
        auto_ack=True,
        timeout_sec=0.0,
    )
    assert result == (True, "auto_ack_for_dev")


def test_eof_treated_as_timeout():
    # io.StringIO("") → readline() returns "" (EOF) → timeout path.
    result = _decide(
        prompt_text="Please open the door.",
        responsible_object_key="door:1",
        failure_stage="execution",
        auto_ack=False,
        timeout_sec=5.0,
        _stdin=io.StringIO(""),
    )
    assert result == (False, "timeout")


def test_y_input_acknowledged():
    result = _decide(
        prompt_text="Please open the door.",
        responsible_object_key="door:1",
        failure_stage="execution",
        auto_ack=False,
        timeout_sec=5.0,
        _stdin=io.StringIO("y\n"),
    )
    ack, note = result
    assert ack is True
    assert "y" in note


def test_n_input_rejected():
    result = _decide(
        prompt_text="Clear the box.",
        responsible_object_key="box:3",
        failure_stage="execution",
        auto_ack=False,
        timeout_sec=5.0,
        _stdin=io.StringIO("n\n"),
    )
    ack, note = result
    assert ack is False
    assert "n" in note


def test_q_input_rejected():
    result = _decide(
        prompt_text="Clear the box.",
        responsible_object_key="box:3",
        failure_stage="execution",
        auto_ack=False,
        timeout_sec=5.0,
        _stdin=io.StringIO("q\n"),
    )
    ack, note = result
    assert ack is False


def test_empty_line_rejected_not_timeout():
    # "\n" (just Enter with no text) is a valid non-empty line → rejected, not timeout.
    result = _decide(
        prompt_text="Open the door.",
        responsible_object_key="door:2",
        failure_stage="execution",
        auto_ack=False,
        timeout_sec=5.0,
        _stdin=io.StringIO("\n"),
    )
    ack, note = result
    assert ack is False
    assert note != "timeout"
