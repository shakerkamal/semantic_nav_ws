#!/usr/bin/env python3
"""En-route blockage trigger (eval-only): changes the WORLD, nothing else.

Watches the robot pose; when it crosses the scenario's trigger line, spawns
the blocker at its fixed pose (reproducible mid-route injection). Optionally
deletes it after delete_after_sec (S1 transient, S4 person-leaves). Announces
spawned/deleted on a latched topic so the mock detector knows the object
exists to be perceived.

Usage:
  python3 eval/enroute_blockage_trigger.py --scenario S2
"""
import argparse
import math
import os
import sys
from typing import Tuple

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from ament_index_python.packages import get_package_share_directory

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from enroute_common import load_scenarios, robot_map_pose  # noqa: E402


def crossed(axis: str, threshold: float, direction: str,
            xy: Tuple[float, float]) -> bool:
    value = xy[0] if axis == "x" else xy[1]
    if direction == "increasing":
        return value > threshold
    return value < threshold


def _pose_xml(x: float, y: float, yaw: float) -> str:
    return f"<pose>{x} {y} 0 0 0 {yaw}</pose>"


class BlockageTrigger(Node):
    def __init__(self, scenario_name: str, config: dict):
        super().__init__("enroute_blockage_trigger")
        sc = config["scenarios"][scenario_name]
        self._trigger = sc["trigger"]
        self._blocker = sc["blocker"]
        self._delete_after = float(sc.get("delete_after_sec") or 0.0)
        self._fired = False
        self._deleted = False
        self._spawn_time = None

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self._state_pub = self.create_publisher(
            String, str(config["common"]["blocker_state_topic"]), latched)

        self._spawn_cli = self.create_client(SpawnEntity, "/spawn_entity")
        self._delete_cli = self.create_client(DeleteEntity, "/delete_entity")
        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f"[TRIGGER] armed for {scenario_name}: "
            f"{self._trigger['axis']} {self._trigger['direction']} "
            f"{self._trigger['threshold']}")

    def _blocker_xml(self) -> str:
        b = self._blocker
        x, y, yaw = (float(v) for v in b["pose"])
        if b["kind"] == "database":
            return (
                "<sdf version='1.6'><model name='{n}'>{p}<include>"
                "<uri>model://{m}</uri></include></model></sdf>".format(
                    n=b["entity"], p=_pose_xml(x, y, yaw), m=b["model"]))
        share = get_package_share_directory("semantic_nav_bringup")
        for sub in ("door_scenario", "person_scenario"):
            path = os.path.join(share, "models", sub, b["model"])
            if os.path.exists(path):
                return open(path).read()
        raise FileNotFoundError(b["model"])

    def _spawn(self) -> None:
        b = self._blocker
        x, y, yaw = (float(v) for v in b["pose"])
        req = SpawnEntity.Request()
        req.name = b["entity"]
        req.xml = self._blocker_xml()
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.orientation.z = math.sin(yaw / 2.0)
        req.initial_pose.orientation.w = math.cos(yaw / 2.0)
        self._spawn_cli.call_async(req)
        self._spawn_time = self.get_clock().now()
        self._state_pub.publish(String(data="spawned"))
        self.get_logger().info(
            f"[TRIGGER] spawned '{b['entity']}' at ({x:.3f}, {y:.3f})")

    def _delete(self) -> None:
        req = DeleteEntity.Request()
        req.name = self._blocker["entity"]
        self._delete_cli.call_async(req)
        self._state_pub.publish(String(data="deleted"))
        self.get_logger().info(
            f"[TRIGGER] deleted '{self._blocker['entity']}'")

    def _tick(self) -> None:
        if self._fired:
            if (self._delete_after > 0.0 and not self._deleted
                    and self._spawn_time is not None):
                elapsed = (self.get_clock().now()
                           - self._spawn_time).nanoseconds / 1e9
                if elapsed >= self._delete_after:
                    self._deleted = True
                    self._delete()
            return
        robot = robot_map_pose(self._tf_buffer)
        if robot is None:
            return
        if crossed(self._trigger["axis"], float(self._trigger["threshold"]),
                   self._trigger["direction"], robot):
            self._fired = True
            self._spawn()


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True,
                        choices=["S1", "S2", "S3", "S4", "S5"])
    parser.add_argument("--config", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "enroute_scenarios.yaml"))
    args = parser.parse_args()
    rclpy.init()
    node = BlockageTrigger(args.scenario, load_scenarios(args.config))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
