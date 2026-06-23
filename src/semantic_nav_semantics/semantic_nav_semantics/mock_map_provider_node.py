"""Mock semantic map provider for e2e testing.

Loads a map_v00x.json file at startup and publishes it on /semantic_map/updates
with TRANSIENT_LOCAL QoS so late-joining subscribers receive the map immediately.

Subscribes to /semantic_map/corrections. On a suspected_displacement correction
with match_type "inferred" or "verified", marks the object displaced in the
internal JSON, increments the revision counter, and re-publishes the updated map.

Serves /semantic_map/query_region (QuerySemanticRegion). Filters the current
in-memory map by the requested region and returns matching records as JSON.

Parameters
----------
map_path : str
    Absolute path to the JSON map file. Defaults to the installed map_v001.json.
map_topic : str
    Topic to publish on. Default: /semantic_map/updates
corrections_topic : str
    Topic to subscribe for corrections. Default: /semantic_map/corrections
query_region_service : str
    Service name for regional queries. Default: /semantic_map/query_region
"""

from __future__ import annotations

import json
import math
import os
import threading

import rclpy
from ament_index_python.packages import get_package_share_directory
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from semantic_nav_interfaces.msg import SemanticCorrectionReport, SemanticMapUpdate
from semantic_nav_interfaces.srv import QuerySemanticRegion


class MockMapProviderNode(Node):

    def __init__(self):
        super().__init__("mock_map_provider")

        self.declare_parameter("map_path", "")
        self.declare_parameter("map_topic", "/semantic_map/updates")
        self.declare_parameter("corrections_topic", "/semantic_map/corrections")
        self.declare_parameter("query_region_service", "/semantic_map/query_region")
        self.declare_parameter("provider_radius_m", 4.0)

        map_path = (
            self.get_parameter("map_path").get_parameter_value().string_value.strip()
        )
        map_topic = (
            self.get_parameter("map_topic").get_parameter_value().string_value.strip()
            or "/semantic_map/updates"
        )
        corrections_topic = (
            self.get_parameter("corrections_topic")
            .get_parameter_value()
            .string_value.strip()
            or "/semantic_map/corrections"
        )
        query_region_service = (
            self.get_parameter("query_region_service")
            .get_parameter_value()
            .string_value.strip()
            or "/semantic_map/query_region"
        )
        self._provider_radius_m: float = float(
            self.get_parameter("provider_radius_m").get_parameter_value().double_value
        )

        if not map_path:
            map_path = os.path.join(
                get_package_share_directory("semantic_nav_semantics"),
                "config",
                "map_v001.json",
            )

        self._lock = threading.Lock()
        self._revision: int = 0
        self._map_data: dict = {}
        self._base_version: str = "map_v001"

        try:
            with open(map_path, "r", encoding="utf-8") as f:
                self._map_data = json.load(f)
            self._base_version = os.path.splitext(os.path.basename(map_path))[0]
            self.get_logger().info(
                f"[MOCK_PROVIDER] Loaded '{map_path}': "
                f"{len(self._map_data)} objects, base_version='{self._base_version}'"
            )
        except Exception as exc:
            self.get_logger().error(f"[MOCK_PROVIDER] Failed to load map: {exc}")
            return

        pub_qos = QoSProfile(depth=1)
        pub_qos.reliability = ReliabilityPolicy.RELIABLE
        pub_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._map_pub = self.create_publisher(SemanticMapUpdate, map_topic, pub_qos)

        self._corrections_sub = self.create_subscription(
            SemanticCorrectionReport,
            corrections_topic,
            self._handle_correction,
            10,
        )

        self._query_region_srv = self.create_service(
            QuerySemanticRegion,
            query_region_service,
            self._handle_query_region,
        )

        self._publish_map()
        self.get_logger().info(
            f"[MOCK_PROVIDER] Published initial map on '{map_topic}' (TRANSIENT_LOCAL). "
            f"Listening for corrections on '{corrections_topic}'. "
            f"Serving regional queries on '{query_region_service}'."
        )

    # ------------------------------------------------------------------

    def _current_version(self) -> str:
        if self._revision == 0:
            return self._base_version
        return f"{self._base_version}_r{self._revision}"

    def _publish_map(self) -> None:
        msg = SemanticMapUpdate()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.semantic_map_version = self._current_version()
        msg.json_payload = json.dumps(self._map_data)
        self._map_pub.publish(msg)

    def _handle_query_region(
        self,
        request: QuerySemanticRegion.Request,
        response: QuerySemanticRegion.Response,
    ) -> QuerySemanticRegion.Response:
        cx = float(request.query_center.x)
        cy = float(request.query_center.y)
        radius = self._provider_radius_m
        include_displaced = bool(request.include_displaced)

        with self._lock:
            version = self._current_version()
            now_stamp = self.get_clock().now().to_msg()

            filtered = {}
            for key, record in self._map_data.items():
                if not isinstance(record, dict):
                    continue
                state = str(record.get("object_state", "")).strip()
                if state == "displaced" and not include_displaced:
                    continue
                center = record.get("bbox_center")
                if not isinstance(center, list) or len(center) < 2:
                    continue
                try:
                    dx = float(center[0]) - cx
                    dy = float(center[1]) - cy
                except (TypeError, ValueError):
                    continue
                if math.sqrt(dx * dx + dy * dy) <= radius:
                    filtered[key] = record
            json_payload = json.dumps(filtered)
            matched = len(filtered)

        response.success = True
        response.semantic_map_version = version
        response.db_stamp = now_stamp
        response.json_payload = json_payload
        response.source = "live_map"
        response.message = (
            f"returned {matched} local objects "
            f"near ({cx:.3f}, {cy:.3f}), version='{version}'"
        )

        self.get_logger().info(
            f"[MOCK_PROVIDER] query_region: center=({cx:.3f}, {cy:.3f}), "
            f"include_displaced={include_displaced}, "
            f"matched={matched}, version='{version}', "
            f"recovery_event_id='{request.recovery_event_id}'"
        )
        return response

    def _handle_correction(self, msg: SemanticCorrectionReport) -> None:
        if msg.correction_type != "suspected_displacement":
            self.get_logger().info(
                f"[MOCK_PROVIDER] Ignoring correction_type='{msg.correction_type}'"
            )
            return

        if msg.responsible_match_type not in {"inferred", "verified"}:
            self.get_logger().info(
                f"[MOCK_PROVIDER] Ignoring match_type='{msg.responsible_match_type}'"
            )
            return

        object_key = (msg.object_key or "").strip()
        if not object_key:
            return

        with self._lock:
            found = self._mark_displaced(
                object_key=object_key,
                reason=msg.reason or "suspected_displacement",
                recovery_event_id=msg.recovery_event_id or "",
            )
            if not found:
                self.get_logger().warn(
                    f"[MOCK_PROVIDER] object_key='{object_key}' not found; "
                    "correction ignored."
                )
                return

            self._revision += 1
            new_version = self._current_version()
            self._publish_map()

        self.get_logger().info(
            f"[MOCK_PROVIDER] Accepted suspected_displacement for '{object_key}': "
            f"new_version='{new_version}', "
            f"recovery_event_id='{msg.recovery_event_id}'"
        )

    def _mark_displaced(
        self,
        *,
        object_key: str,
        reason: str,
        recovery_event_id: str,
    ) -> bool:
        """Set object_state='displaced' on the matching record. Returns True if found."""
        sep = object_key.rfind(":")
        if sep < 1:
            return False
        target_tag = object_key[:sep].strip()
        try:
            target_id = int(object_key[sep + 1:].strip())
        except ValueError:
            return False

        for record in self._map_data.values():
            if not isinstance(record, dict):
                continue
            if str(record.get("object_tag", "")).strip() != target_tag:
                continue
            try:
                if int(record.get("id", -1)) != target_id:
                    continue
            except (TypeError, ValueError):
                continue
            record["object_state"] = "displaced"
            record["displaced_reason"] = reason
            record["displaced_by_recovery_event"] = recovery_event_id
            return True
        return False


def main(args=None):
    rclpy.init(args=args)
    node = MockMapProviderNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
