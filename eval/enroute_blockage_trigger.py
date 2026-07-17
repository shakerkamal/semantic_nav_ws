#!/usr/bin/env python3
"""En-route blockage trigger (eval-only): changes the world, nothing else.

Watches the robot pose; when it crosses the scenario trigger line, spawns the
blocker at its fixed pose. Optionally deletes it after ``delete_after_sec``.
For operator-action scenarios, deletion is requested only after a keyed action
request and completion is published only after Gazebo confirms deletion.

Usage:
    python3 eval/enroute_blockage_trigger.py --scenario S2
"""

import argparse
import math
import os
import sys
from typing import Optional, Tuple

import rclpy
from ament_index_python.packages import get_package_share_directory
from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from enroute_common import load_scenarios, robot_map_pose  # noqa: E402


def crossed(
    axis: str,
    threshold: float,
    direction: str,
    xy: Tuple[float, float],
) -> bool:
    value = xy[0] if axis == "x" else xy[1]
    if direction == "increasing":
        return value > threshold
    return value < threshold


def parse_action_token(token: str) -> Tuple[str, str, str]:
    """Parse ``event_id|object_key|directive_action`` used by M2."""
    parts = token.split("|")
    if len(parts) != 3 or any(not part.strip() for part in parts):
        raise ValueError(
            "action token must be event_id|object_key|directive_action"
        )
    return parts[0], parts[1], parts[2]


def database_include_xml(model_name: str, entity_name: str) -> str:
    """Return an SDF payload for a Gazebo model-database spawn.

    Mirrors gazebo_ros's own ``spawn_entity.py`` model-database template, with
    no pose in the XML. Placement comes from ``SpawnEntity.initial_pose``.
    The explicit ``<include><name>`` override makes ``DeleteEntity`` address
    the inserted model by the requested scenario entity name.
    """
    return (
        "<sdf version='1.6'><world name='default'><include>"
        "<name>{}</name><uri>model://{}</uri></include></world></sdf>"
    ).format(entity_name, model_name)


class BlockageTrigger(Node):
    """Spawn and remove deterministic evaluation blockers."""

    def __init__(self, scenario_name: str, config: dict):
        super().__init__("enroute_blockage_trigger")

        common = config["common"]
        scenario = config["scenarios"][scenario_name]

        self._trigger = scenario["trigger"]
        self._blocker = scenario["blocker"]
        self._detector = scenario.get("detector")
        self._expected_directive = str(
            scenario.get("expected_directive") or ""
        )
        self._delete_after = float(
            scenario.get("delete_after_sec") or 0.0
        )
        self._delete_on_obstacle_signal = bool(
            scenario.get("delete_on_obstacle_signal", False)
        )
        self._obstacle_signal_topic = str(
            scenario.get("obstacle_signal_topic")
            or "/robot_obstacle_signal"
        )
        self._obstacle_signal_value = str(
            scenario.get("obstacle_signal_value") or ""
        )
        self._signal_reaction_delay_sec = max(
            0.0,
            float(scenario.get("signal_reaction_delay_sec") or 0.0),
        )
        self._action_settle_sec = max(
            0.0,
            float(common.get("operator_action_settle_sec", 0.0)),
        )

        self._fired = False
        self._deleted = False
        self._delete_in_flight = False
        self._spawn_time = None
        self._pending_action_token: Optional[str] = None
        self._completion_timer = None
        self._signal_delete_timer = None
        self._signal_received = False
        self._obstacle_signal_sub = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        latched = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self._state_pub = self.create_publisher(
            String,
            str(common["blocker_state_topic"]),
            latched,
        )
        self._action_completion_pub = self.create_publisher(
            String,
            str(common["operator_action_completion_topic"]),
            latched,
        )

        self._spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self._delete_cli = self.create_client(DeleteEntity, "/delete_entity")

        # Only operator-action scenarios consume the request/completion seam.
        if self._expected_directive in (
            "open_door_then_replan",
            "clear_object_then_replan",
        ):
            self.create_subscription(
                String,
                str(common["operator_action_request_topic"]),
                self._on_operator_action_requested,
                latched,
            )

        if self._delete_on_obstacle_signal:
            self._obstacle_signal_sub = self.create_subscription(
                String,
                self._obstacle_signal_topic,
                self._on_obstacle_signal,
                10,
            )
            self.get_logger().info(
                "[TRIGGER] signal-driven dynamic departure armed: "
                f"topic='{self._obstacle_signal_topic}' "
                f"value='{self._obstacle_signal_value}' "
                f"reaction_delay={self._signal_reaction_delay_sec:.2f}s"
            )
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"[TRIGGER] armed for {scenario_name}: "
            f"{self._trigger['axis']} {self._trigger['direction']} "
            f"{self._trigger['threshold']}"
        )

    def _on_operator_action_requested(self, msg: String) -> None:
        if not msg.data:
            return

        try:
            event_id, object_key, directive_action = parse_action_token(
                msg.data
            )
        except ValueError as exc:
            self.get_logger().warning(
                "[TRIGGER] ignoring malformed operator action token: "
                f"{exc}"
            )
            return

        if not self._fired or self._deleted or self._delete_in_flight:
            return

        expected_object_key = (
            str(self._detector.get("object_key"))
            if self._detector is not None
            else ""
        )
        if expected_object_key and object_key != expected_object_key:
            self.get_logger().warning(
                "[TRIGGER] ignoring operator action for unrelated object "
                f"'{object_key}' (expected '{expected_object_key}')"
            )
            return

        if directive_action != self._expected_directive:
            self.get_logger().warning(
                "[TRIGGER] ignoring operator action "
                f"'{directive_action}' "
                f"(expected '{self._expected_directive}')"
            )
            return

        self._pending_action_token = msg.data
        self.get_logger().info(
            "[TRIGGER] operator action requested "
            f"event_id='{event_id}' object='{object_key}' "
            f"action='{directive_action}' -- deleting blocker"
        )
        self._delete()

    def _on_obstacle_signal(self, msg: String) -> None:
        if not self._delete_on_obstacle_signal:
            return
        if not self._fired or self._deleted or self._delete_in_flight:
            return
        if self._signal_received:
            return
        if self._obstacle_signal_value and msg.data != self._obstacle_signal_value:
            self.get_logger().warning(
                "[TRIGGER] ignoring unrelated obstacle signal "
                f"'{msg.data}' (expected '{self._obstacle_signal_value}')"
            )
            return

        self._signal_received = True
        self.get_logger().info(
            "[TRIGGER] received obstacle signal "
            f"payload='{msg.data}'; dynamic-obstacle reaction delay "
            f"{self._signal_reaction_delay_sec:.2f}s started"
        )
        if self._signal_reaction_delay_sec <= 0.0:
            self._delete()
            return
        self._signal_delete_timer = self.create_timer(
            self._signal_reaction_delay_sec,
            self._delete_after_signal_delay,
        )

    def _delete_after_signal_delay(self) -> None:
        timer = self._signal_delete_timer
        self._signal_delete_timer = None
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
        self.get_logger().info(
            "[TRIGGER] dynamic-obstacle reaction delay completed; "
            "deleting blocker"
        )
        self._delete()

    def _blocker_xml(self) -> str:
        blocker = self._blocker
        if blocker["kind"] == "database":
            return database_include_xml(
                blocker["model"], blocker["entity"]
            )

        share = get_package_share_directory("semantic_nav_bringup")
        for subdirectory in (
            "door_scenario",
            "person_scenario",
            "obstacle_scenario",
        ):
            path = os.path.join(
                share,
                "models",
                subdirectory,
                blocker["model"],
            )
            if os.path.exists(path):
                with open(path, encoding="utf-8") as stream:
                    return stream.read()

        raise FileNotFoundError(blocker["model"])

    def _spawn(self) -> None:
        # Remove a leftover entity from an interrupted previous repetition.
        request = DeleteEntity.Request()
        request.name = self._blocker["entity"]
        future = self._delete_cli.call_async(request)
        future.add_done_callback(self._on_precleanup_response)

    def _on_precleanup_response(self, future) -> None:
        try:
            response = future.result()
            self.get_logger().info(
                "[TRIGGER] pre-spawn cleanup: "
                f"success={response.success} "
                f"message='{response.status_message}'"
            )
        except Exception as exc:  # noqa: BLE001
            self.get_logger().info(
                "[TRIGGER] pre-spawn cleanup call failed "
                f"(ignored): {exc}"
            )

        self._do_spawn()

    def _do_spawn(self) -> None:
        blocker = self._blocker
        x, y, yaw = (float(value) for value in blocker["pose"])

        request = SpawnEntity.Request()
        request.name = blocker["entity"]
        request.xml = self._blocker_xml()
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.orientation.z = math.sin(yaw / 2.0)
        request.initial_pose.orientation.w = math.cos(yaw / 2.0)
        request.reference_frame = "world"

        self._spawn_time = self.get_clock().now()
        self.get_logger().info(
            f"[TRIGGER] spawn requested '{blocker['entity']}' "
            f"at ({x:.3f}, {y:.3f})"
        )

        future = self._spawn_cli.call_async(request)
        future.add_done_callback(self._on_spawn_response)

    def _on_spawn_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self.get_logger().error(
                f"[TRIGGER] spawn service call failed: {exc}"
            )
            return

        self.get_logger().info(
            f"[TRIGGER] spawn result: success={response.success} "
            f"message='{response.status_message}'"
        )
        if response.success:
            self._state_pub.publish(String(data="spawned"))

    def _delete(self) -> None:
        if self._deleted or self._delete_in_flight:
            return

        self._delete_in_flight = True
        request = DeleteEntity.Request()
        request.name = self._blocker["entity"]

        future = self._delete_cli.call_async(request)
        future.add_done_callback(self._on_delete_response)
        self.get_logger().info(
            f"[TRIGGER] delete requested '{self._blocker['entity']}'"
        )

    def _on_delete_response(self, future) -> None:
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            self._delete_in_flight = False
            self.get_logger().error(
                f"[TRIGGER] delete service call failed: {exc}"
            )
            return

        self._delete_in_flight = False
        self.get_logger().info(
            f"[TRIGGER] delete result: success={response.success} "
            f"message='{response.status_message}'"
        )

        if not response.success:
            self.get_logger().error(
                "[TRIGGER] blocker deletion was rejected; action "
                "completion will not be published"
            )
            return

        # Gazebo success is the authoritative transition to deleted.
        self._deleted = True
        self._state_pub.publish(String(data="deleted"))

        if self._pending_action_token is None:
            return

        if self._action_settle_sec <= 0.0:
            self._publish_action_completion()
            return

        self.get_logger().info(
            "[TRIGGER] deletion confirmed; waiting "
            f"{self._action_settle_sec:.2f}s for fresh sensor frames"
        )
        self._completion_timer = self.create_timer(
            self._action_settle_sec,
            self._publish_action_completion,
        )

    def _publish_action_completion(self) -> None:
        timer = self._completion_timer
        self._completion_timer = None
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)

        token = self._pending_action_token
        if token is None:
            return

        self._pending_action_token = None
        self._action_completion_pub.publish(String(data=token))
        self.get_logger().info(
            f"[TRIGGER] operator action completed token='{token}'"
        )

    def _tick(self) -> None:
        if self._fired:
            if (
                self._delete_after > 0.0
                and not self._deleted
                and not self._delete_in_flight
                and self._spawn_time is not None
            ):
                elapsed = (
                    self.get_clock().now() - self._spawn_time
                ).nanoseconds / 1e9
                if elapsed >= self._delete_after:
                    self._delete()
            return

        robot = robot_map_pose(self._tf_buffer)
        if robot is None:
            return

        if crossed(
            self._trigger["axis"],
            float(self._trigger["threshold"]),
            self._trigger["direction"],
            robot,
        ):
            self._fired = True
            self._spawn()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--scenario",
        required=True,
        choices=["S1", "S2", "S3", "S4", "S5"],
    )
    parser.add_argument(
        "--config",
        default=os.path.join(
            os.path.dirname(os.path.abspath(__file__)),
            "enroute_scenarios.yaml",
        ),
    )
    args = parser.parse_args()

    rclpy.init()
    node = BlockageTrigger(args.scenario, load_scenarios(args.config))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
