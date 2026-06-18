"""Operator I/O service node for BT-LR M5.

Serves /operator_decision. Prints a prompt to stdout and waits for y/n/q.
Uses select() for non-blocking stdin so the ROS spin thread is not starved
when auto_ack_for_dev=True.

Stdin blocking note: the service handler blocks the callback thread until the
operator responds or the timeout fires. Because MultiThreadedExecutor is used,
the node heartbeat and other topics continue running on separate threads.
For unattended testing, always set auto_ack_for_dev:=True.
"""

from __future__ import annotations

import select
import sys
from typing import IO, Optional, Tuple

import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from semantic_nav_interfaces.srv import OperatorDecision


def _decide(
    prompt_text: str,
    responsible_object_key: str,
    failure_stage: str,
    auto_ack: bool,
    timeout_sec: float,
    _stdin: Optional[IO[str]] = None,
) -> Tuple[bool, str]:
    """Return (acknowledged, operator_note).

    Extracted for unit-testability — no Node dependency.

    EOF on stdin (readline returns empty string) is treated as timeout.
    This allows io.StringIO("") in tests to exercise the timeout path.

    _stdin: injectable stdin; defaults to sys.stdin.
    """
    if auto_ack:
        return True, "auto_ack_for_dev"

    stdin = _stdin if _stdin is not None else sys.stdin

    print(f"\n[OPERATOR PROMPT] {prompt_text}", flush=True)
    print(
        f"[OPERATOR PROMPT] Object: {responsible_object_key}  Stage: {failure_stage}",
        flush=True,
    )
    print("[OPERATOR PROMPT] Press y to confirm, n/q to reject: ", end="", flush=True)

    timeout = timeout_sec if timeout_sec > 0.0 else None

    try:
        ready, _, _ = select.select([stdin], [], [], timeout)
    except (ValueError, OSError):
        # select() does not work on StringIO or non-fd streams (e.g. in tests).
        # Fall through to readline() and let EOF handling decide the outcome.
        ready = [stdin]

    if not ready:
        print("\n[OPERATOR PROMPT] Timeout — rejecting.", flush=True)
        return False, "timeout"

    raw_line = stdin.readline()

    if raw_line == "":
        # EOF: stream closed or StringIO exhausted — treat as timeout.
        print("\n[OPERATOR PROMPT] EOF/no input — rejecting as timeout.", flush=True)
        return False, "timeout"

    line = raw_line.strip().lower()

    if line == "y":
        return True, f"operator_confirmed: '{line}'"

    return False, f"operator_rejected: '{line}'"


class OperatorIONode(Node):
    """Serve /operator_decision from stdin."""

    def __init__(self) -> None:
        super().__init__("operator_io_node")

        self.declare_parameter("prompt_timeout_sec", 0.0)
        self.declare_parameter("auto_ack_for_dev", False)

        self._timeout = float(
            self.get_parameter("prompt_timeout_sec").get_parameter_value().double_value
        )
        self._auto_ack = bool(
            self.get_parameter("auto_ack_for_dev").get_parameter_value().bool_value
        )

        self._srv = self.create_service(
            OperatorDecision,
            "/operator_decision",
            self._handle_operator_decision,
        )

        self.get_logger().info(
            f"OperatorIONode ready: "
            f"auto_ack_for_dev={self._auto_ack}, "
            f"prompt_timeout_sec={self._timeout:.1f}"
        )

    def _handle_operator_decision(
        self,
        request: OperatorDecision.Request,
        response: OperatorDecision.Response,
    ) -> OperatorDecision.Response:
        ack, note = _decide(
            prompt_text=str(request.prompt_text),
            responsible_object_key=str(request.responsible_object_key),
            failure_stage=str(request.failure_stage),
            auto_ack=self._auto_ack,
            timeout_sec=self._timeout,
        )
        response.acknowledged = ack
        response.operator_note = note

        self.get_logger().info(
            f"[OPERATOR_IO] acknowledged={ack} "
            f"note='{note}' "
            f"object='{request.responsible_object_key}' "
            f"directive='{request.directive_action}' "
            f"event='{request.recovery_event_id}'"
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = OperatorIONode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt — shutting down.")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
