# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Publishes live door open/closed state derived from the global costmap.

Samples the global costmap cells inside each mapped door's bbox footprint and
publishes DoorStateArray on /semantic_door_states. This is the missing producer
for the door-state overlay lane consumed by local_object_query_node.

The node is intentionally "dumb": it reports observations only. It never decides
recovery, never clears costmaps, and never calls a recovery service.
"""

from __future__ import annotations

import json
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from semantic_nav_interfaces.msg import DoorStateArray, DoorStateObservation
from semantic_nav_semantics.door_state_estimation import (
    GridView,
    classify_door_state,
    load_door_footprints,
    occupied_fraction,
)


def _default_map_path() -> str:
    share = get_package_share_directory("semantic_nav_semantics")
    return os.path.join(share, "config", "map_v001.json")


class DoorStateMonitorNode(Node):
    def __init__(self) -> None:
        super().__init__("door_state_monitor_node")

        self.declare_parameter("map_path", _default_map_path())
        self.declare_parameter("costmap_topic", "/global_costmap/costmap")
        self.declare_parameter("door_states_topic", "/semantic_door_states")
        self.declare_parameter("publish_hz", 2.0)
        self.declare_parameter("ttl_sec", 3.0)
        # 100 (not 90): count only true obstacles (cost 100), never the
        # inflation halo (<=99). A door slab marks as a genuine obstacle; the
        # frame's inflation must not read as "closed".
        self.declare_parameter("lethal_threshold", 100)
        self.declare_parameter("blocked_fraction", 0.30)
        self.declare_parameter("open_fraction", 0.10)
        self.declare_parameter("min_observed_cells", 3)
        # Inset each side of a door's sampled footprint by this FRACTION of its
        # half-extent, so we read the CLEAR OPENING (which the moving slab fills)
        # rather than the door object's bbox, whose ends overlap the frame posts.
        # A fraction (not absolute metres) is one rule that scales to any door:
        # 0.25 samples the central half, where the slab always is and frames
        # never are.
        self.declare_parameter("footprint_margin_frac", 0.25)
        self.declare_parameter("robot_openable", False)

        p = self.get_parameter
        map_path = p("map_path").get_parameter_value().string_value
        costmap_topic = p("costmap_topic").get_parameter_value().string_value
        door_states_topic = p("door_states_topic").get_parameter_value().string_value
        self._publish_hz = float(p("publish_hz").value)
        self._ttl = float(p("ttl_sec").value)
        self._lethal = int(p("lethal_threshold").value)
        self._blocked_fraction = float(p("blocked_fraction").value)
        self._open_fraction = float(p("open_fraction").value)
        self._min_observed_cells = int(p("min_observed_cells").value)
        self._footprint_margin_frac = float(p("footprint_margin_frac").value)
        self._robot_openable = bool(p("robot_openable").value)

        with open(map_path, "r", encoding="utf-8") as f:
            self._doors = load_door_footprints(json.load(f))

        self._grid = None

        # Nav2 costmaps are latched TRANSIENT_LOCAL.
        qos = QoSProfile(depth=1)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.create_subscription(OccupancyGrid, costmap_topic, self._on_grid, qos)

        self._pub = self.create_publisher(DoorStateArray, door_states_topic, 10)
        self.create_timer(1.0 / max(0.1, self._publish_hz), self._tick)

        self.get_logger().info(
            f"door_state_monitor_node: monitoring {len(self._doors)} door(s); "
            f"costmap='{costmap_topic}', publishing '{door_states_topic}' "
            f"at {self._publish_hz:.1f} Hz."
        )

    def _on_grid(self, msg: OccupancyGrid) -> None:
        self._grid = msg

    def _tick(self) -> None:
        if self._grid is None or not self._doors:
            return
        g = self._grid
        view = GridView(
            resolution=g.info.resolution,
            width=g.info.width,
            height=g.info.height,
            origin_x=g.info.origin.position.x,
            origin_y=g.info.origin.position.y,
            data=g.data,
        )

        arr = DoorStateArray()
        arr.header.stamp = self.get_clock().now().to_msg()
        arr.header.frame_id = "map"
        for fp in self._doors:
            frac, observed = occupied_fraction(
                view, fp, self._lethal, margin_frac=self._footprint_margin_frac
            )
            est = classify_door_state(
                frac, observed,
                blocked_fraction=self._blocked_fraction,
                open_fraction=self._open_fraction,
                min_observed_cells=self._min_observed_cells,
                object_key=fp.object_key,
            )
            obs = DoorStateObservation()
            obs.header = arr.header
            obs.object_key = est.object_key
            obs.door_state = est.door_state
            obs.traversability = est.traversability
            obs.robot_openable = self._robot_openable
            obs.confidence = float(est.confidence)
            obs.ttl_sec = float(self._ttl)
            obs.source = "costmap_door_monitor"
            arr.observations.append(obs)
        self._pub.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = DoorStateMonitorNode()
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
