#!/usr/bin/env python3
"""Fake ProposeRecovery service for BT-LR M1D validation.

Runs a deterministic /fake_propose_recovery service so /request_recovery can be
tested without llama_ros.
"""

import rclpy
from rclpy.node import Node

from semantic_nav_interfaces.srv import ProposeRecovery


class FakeProposeRecoveryNode(Node):
    def __init__(self):
        super().__init__("fake_propose_recovery_node")

        self.declare_parameter("service_name", "/fake_propose_recovery")
        self.declare_parameter("proposal_action", "wait_then_replan")
        self.declare_parameter("success", True)
        self.declare_parameter("target_object_tag", "cabinet")
        self.declare_parameter("target_intent_hint", "kitchen food storage alternative")
        self.declare_parameter("target", "")
        self.declare_parameter("wait_seconds", 4)
        self.declare_parameter("confidence_percent", 80)
        self.declare_parameter("rationale", "fake deterministic recovery proposal")

        service_name = (
            self.get_parameter("service_name")
            .get_parameter_value()
            .string_value
        )

        self._srv = self.create_service(
            ProposeRecovery,
            service_name,
            self._handle,
        )

        self.get_logger().info(
            f"FakeProposeRecoveryNode serving '{service_name}'."
        )

    def _handle(self, request, response):
        action = (
            self.get_parameter("proposal_action")
            .get_parameter_value()
            .string_value
            .strip()
        )

        success = (
            self.get_parameter("success")
            .get_parameter_value()
            .bool_value
        )

        target_object_tag = (
            self.get_parameter("target_object_tag")
            .get_parameter_value()
            .string_value
            .strip()
        )

        target_intent_hint = (
            self.get_parameter("target_intent_hint")
            .get_parameter_value()
            .string_value
            .strip()
        )

        target = (
            self.get_parameter("target")
            .get_parameter_value()
            .string_value
            .strip()
        )

        wait_seconds = (
            self.get_parameter("wait_seconds")
            .get_parameter_value()
            .integer_value
        )

        confidence_percent = (
            self.get_parameter("confidence_percent")
            .get_parameter_value()
            .integer_value
        )

        rationale = (
            self.get_parameter("rationale")
            .get_parameter_value()
            .string_value
        )

        self.get_logger().info(
            "Fake proposal requested: "
            f"failure_stage='{request.failure_stage}', "
            f"trigger_source='{getattr(request, 'trigger_source', '')}', "
            f"original_object_tag='{getattr(request, 'original_object_tag', '')}', "
            f"original_intent_hint='{getattr(request, 'original_intent_hint', '')}', "
            f"current_target_object_key='{getattr(request, 'current_target_object_key', '')}', "
            f"remaining_retry_budget={request.remaining_retry_budget}"
        )

        response.success = bool(success)
        response.action = action
        response.target = target
        response.waypoints = []
        response.rationale = rationale
        response.confidence_percent = int(confidence_percent)
        response.raw_output = (
            "{"
            f"\"fake\":true,"
            f"\"action\":\"{action}\","
            f"\"target_object_tag\":\"{target_object_tag}\","
            f"\"target_intent_hint\":\"{target_intent_hint}\""
            "}"
        )
        response.message = f"fake proposal action={action}"

        # Extended recovery fields.
        response.responsible_object_key = getattr(
            request,
            "responsible_object_key",
            "",
        )
        response.operator_message = "fake operator message"
        response.wait_seconds = int(wait_seconds)

        response.target_object_tag = target_object_tag
        response.target_intent_hint = target_intent_hint
        response.target_object_key = ""

        self.get_logger().info(
            "Fake response: "
            f"success={response.success}, "
            f"action='{response.action}', "
            f"target_object_tag='{response.target_object_tag}', "
            f"target_intent_hint='{response.target_intent_hint}', "
            f"wait_seconds={response.wait_seconds}"
        )

        return response


def main(args=None):
    rclpy.init(args=args)
    node = FakeProposeRecoveryNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()