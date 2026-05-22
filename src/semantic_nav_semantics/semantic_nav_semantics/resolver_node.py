import math
import os
from typing import Optional

import rclpy
from ament_index_python.packages import get_package_share_directory
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from semantic_nav_interfaces.srv import ResolveLocation
from semantic_nav_semantics.semantic_store import ResolvedLocation, SemanticStore


def yaw_to_quaternion(yaw: float):
    """
    Convert planar yaw in radians to a quaternion for a PoseStamped in map frame.
    """
    half_yaw = yaw * 0.5
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)
    return qz, qw


class ResolverNode(Node):
    def __init__(self):
        super().__init__("resolver_node")

        default_db_path = os.path.join(
            get_package_share_directory("semantic_nav_semantics"),
            "config",
            "semantic_db.json",
        )

        self.declare_parameter("semantic_db_path", default_db_path)
        self.declare_parameter("resolve_service", "/resolve_location")
        self.declare_parameter("initial_db_version", 1)

        store_path = (
            self.get_parameter("semantic_db_path")
            .get_parameter_value()
            .string_value
            .strip()
        )

        if not store_path:
            self.get_logger().warn(
                f"No semantic_db_path provided. Using default path: {default_db_path}"
            )
            store_path = default_db_path

        resolve_service = (
            self.get_parameter("resolve_service")
            .get_parameter_value()
            .string_value
        )

        initial_db_version = (
            self.get_parameter("initial_db_version")
            .get_parameter_value()
            .integer_value
        )

        if initial_db_version <= 0:
            raise ValueError("Parameter 'initial_db_version' must be >= 1")

        self.get_logger().info(f"Using semantic database path: {store_path}")
        self.get_logger().info(f"Initial semantic DB version: {initial_db_version}")

        self.semantic_store = SemanticStore(
            store_path=store_path,
            initial_db_version=initial_db_version,
        )

        self.srv = self.create_service(
            ResolveLocation,
            resolve_service,
            self.resolve_location_callback,
        )

        self.get_logger().info(
            f"ResolverNode initialized: service='{resolve_service}', "
            f"db_version={self.semantic_store.db_version}, "
            f"db_stamp={self._stamp_to_string(self._current_db_stamp())}, "
            f"locations={self.semantic_store.location_count}, "
            f"aliases={self.semantic_store.alias_count}"
        )

    def resolve_location_callback(self, request, response):
        query = request.query.strip()

        self.get_logger().info(
            f"[RESOLUTION] Received resolve_location request: query='{query}'"
        )

        if not query:
            self._fill_failure_response(
                response=response,
                message="Query cannot be empty.",
            )
            return response

        resolved = self.semantic_store.resolve_location(query)

        if resolved is None:
            self.get_logger().warn(
                f"[RESOLUTION] Location query='{query}' not found "
                f"under db_version={self.semantic_store.db_version}"
            )

            self._fill_failure_response(
                response=response,
                message=(
                    f"Location '{query}' not found "
                    f"under db_version={self.semantic_store.db_version}."
                ),
            )
            return response

        if resolved.frame_id != "map":
            # This should not happen because SemanticStore validates the DB at load time.
            self.get_logger().error(
                f"[RESOLUTION] Location '{resolved.location_id}' has unsupported "
                f"frame_id='{resolved.frame_id}'. Expected 'map'."
            )

            self._fill_failure_response(
                response=response,
                message=(
                    f"Unsupported frame_id '{resolved.frame_id}' "
                    f"for location '{resolved.location_id}'."
                ),
                location_id=resolved.location_id,
                db_version=resolved.db_version,
                db_stamp=self._stamp_from_resolved(resolved),
            )
            return response

        pose = self._resolved_to_pose(resolved)

        response.success = True
        response.location_id = resolved.location_id
        response.pose = pose
        response.db_version = resolved.db_version
        response.db_stamp = self._stamp_from_resolved(resolved)
        response.message = (
            f"Location '{query}' resolved successfully to "
            f"location_id='{resolved.location_id}'."
        )

        self.get_logger().info(
            f"[RESOLUTION] Resolved query='{query}' -> "
            f"location_id='{resolved.location_id}', "
            f"db_version={resolved.db_version}, "
            f"db_stamp={self._stamp_to_string(response.db_stamp)}, "
            f"frame='{pose.header.frame_id}', "
            f"x={pose.pose.position.x:.3f}, "
            f"y={pose.pose.position.y:.3f}, "
            f"yaw={resolved.yaw:.3f}"
        )

        return response

    def _resolved_to_pose(self, resolved: ResolvedLocation) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = resolved.frame_id
        pose.header.stamp = self.get_clock().now().to_msg()

        pose.pose.position.x = resolved.x
        pose.pose.position.y = resolved.y
        pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(resolved.yaw)

        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        return pose

    def _fill_failure_response(
        self,
        response,
        message: str,
        location_id: str = "",
        db_version: Optional[int] = None,
        db_stamp: Optional[Time] = None,
    ):
        response.success = False
        response.location_id = location_id
        response.pose = PoseStamped()
        response.db_version = (
            int(db_version)
            if db_version is not None
            else int(self.semantic_store.db_version)
        )
        response.db_stamp = db_stamp if db_stamp is not None else self._current_db_stamp()
        response.message = message

    def _current_db_stamp(self) -> Time:
        stamp = Time()
        stamp.sec = int(self.semantic_store.db_stamp_sec)
        stamp.nanosec = int(self.semantic_store.db_stamp_nanosec)
        return stamp

    @staticmethod
    def _stamp_from_resolved(resolved: ResolvedLocation) -> Time:
        stamp = Time()
        stamp.sec = int(resolved.db_stamp_sec)
        stamp.nanosec = int(resolved.db_stamp_nanosec)
        return stamp

    @staticmethod
    def _stamp_to_string(stamp: Time) -> str:
        return f"{stamp.sec}.{stamp.nanosec:09d}"


def main(args=None):
    rclpy.init(args=args)
    resolver_node = ResolverNode()

    try:
        rclpy.spin(resolver_node)
    except KeyboardInterrupt:
        resolver_node.get_logger().info(
            "Keyboard interrupt received. Shutting down resolver_node."
        )
    finally:
        resolver_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()