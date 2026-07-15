#!/usr/bin/env python3
"""Mock ConceptGraph-style detector for the en-route ablation (eval-only).

Models PERCEIVING the scenario blocker: while the blocker exists (trigger
publishes its state) AND the robot is within perception range, publishes the
object as a DynamicObjectArray observation on /semantic_dynamic_objects.

PERCEPTION FIELDS ONLY: tag, caption, state, bbox, confidence, ttl.
Affordances (openable/clearable/safety_class) stay at message defaults —
local_object_query_node classifies them at ingestion from the affordance
table, exactly like persistent-map objects (up-front parity).

Usage:
  python3 eval/mock_concept_graph_detector.py --scenario S4
"""
import argparse
import os
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from geometry_msgs.msg import Point, Vector3
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener

from semantic_nav_interfaces.msg import (
    DynamicObjectArray,
    DynamicObjectObservation,
    ObjectInstance,
)

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from enroute_common import load_scenarios, planar_dist, robot_map_pose  # noqa: E402


class MockConceptGraphDetector(Node):
    def __init__(self, scenario_name: str, config: dict):
        super().__init__("mock_concept_graph_detector")
        common = config["common"]
        sc = config["scenarios"][scenario_name]
        det = sc["detector"]
        if det is None:
            raise SystemExit(
                f"{scenario_name} has no detector block (by design); "
                "do not run the detector for this scenario.")

        self._range_m = float(common["perception_range_m"])
        self._ttl = float(common["ttl_sec"])
        self._det = det
        self._blocker_xy = (float(sc["blocker"]["pose"][0]),
                            float(sc["blocker"]["pose"][1]))
        self._blocker_present = False

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub = self.create_publisher(
            DynamicObjectArray, "/semantic_dynamic_objects", 10)
        latched = QoSProfile(
            depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(
            String, str(common["blocker_state_topic"]),
            self._on_blocker_state, latched)

        period = 1.0 / float(common["detect_rate_hz"])
        self.create_timer(period, self._tick)
        self.get_logger().info(
            f"[MOCK_DETECTOR] up for {scenario_name}: tag='{det['tag']}' "
            f"range={self._range_m}m ttl={self._ttl}s")

    def _on_blocker_state(self, msg: String) -> None:
        self._blocker_present = (msg.data.strip() == "spawned")
        self.get_logger().info(
            f"[MOCK_DETECTOR] blocker_state={msg.data.strip()}")

    def _tick(self) -> None:
        robot = robot_map_pose(self._tf_buffer)
        if robot is None:
            return
        dist = planar_dist(robot, self._blocker_xy)
        publishing = self._blocker_present and dist <= self._range_m
        # Task 6 parses these lines for S4 min_standoff / reapproach metrics.
        self.get_logger().info(
            f"[MOCK_DETECTOR] dist={dist:.2f} publishing={publishing}")
        if not publishing:
            return

        obj = ObjectInstance()
        obj.object_key = self._det["object_key"]
        obj.object_tag = self._det["tag"]
        obj.object_caption = " ".join(str(self._det["caption"]).split())
        obj.object_state = self._det["state"]
        obj.bbox_center = Point(
            x=self._blocker_xy[0], y=self._blocker_xy[1],
            z=float(self._det["extent"][2]) / 2.0)
        obj.bbox_extent = Vector3(
            x=float(self._det["extent"][0]),
            y=float(self._det["extent"][1]),
            z=float(self._det["extent"][2]))
        obj.bbox_volume = (float(self._det["extent"][0])
                           * float(self._det["extent"][1])
                           * float(self._det["extent"][2]))
        obj.confidence = 0.9
        obj.ttl_sec = self._ttl
        obj.observation_stamp = self.get_clock().now().to_msg()
        # openable/clearable/safety_class stay at defaults: perception only.

        obs = DynamicObjectObservation()
        obs.header.stamp = obj.observation_stamp
        obs.header.frame_id = "map"
        obs.object = obj
        arr = DynamicObjectArray()
        arr.observations = [obs]
        self._pub.publish(arr)


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--scenario", required=True,
                        choices=["S2", "S3", "S4"])
    parser.add_argument("--config", default=os.path.join(
        os.path.dirname(os.path.abspath(__file__)),
        "enroute_scenarios.yaml"))
    args = parser.parse_args()
    rclpy.init()
    node = MockConceptGraphDetector(args.scenario, load_scenarios(args.config))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
