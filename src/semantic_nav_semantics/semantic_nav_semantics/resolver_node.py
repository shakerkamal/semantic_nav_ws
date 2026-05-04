import math
import os

import rclpy
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node

from semantic_nav_interfaces.srv import ResolveLocation
from semantic_nav_semantics.semantic_store import SemanticStore

def yaw_to_quaternion(yaw: float):
    """Convert a yaw angle (in radians) to a quaternion."""
    half_yaw = yaw / 2.0
    qz = math.sin(half_yaw)
    qw = math.cos(half_yaw)
    return qz, qw

class ResolverNode(Node):
    def __init__(self):
        super().__init__('resolver_node')

        default_locations_path = os.path.join(
            get_package_share_directory('semantic_nav_semantics'),
            'config',
            'semantic_db.json'
        )

        self.declare_parameter('semantic_db_path', default_locations_path)
        store_path = self.get_parameter('semantic_db_path').get_parameter_value().string_value
        if not store_path:
            self.get_logger().warn("No semantic_db_path provided, using default path: " + default_locations_path)
            store_path = default_locations_path
        self.get_logger().info(f"Using semantic database path: {store_path}")

        self.semantic_store = SemanticStore(store_path)

        self.srv = self.create_service(ResolveLocation, 'resolve_location', self.resolve_location_callback)
        self.get_logger().info("ResolverNode initialized and ready to resolve locations")

        self.get_logger().info(f"ResolverNode initialized with semantic store at {store_path}")

    def resolve_location_callback(self, request, response):
        query = request.query.strip()
        self.get_logger().info(f"Received resolve_location request: '{query}'")

        if not query:
            response.success = False
            response.location_id = ''
            response.message = "Query cannot be empty"
            return response
        
        resolved = self.semantic_store.resolve_location(query)
        if resolved is None:
            self.get_logger().warn(f"Location '{query}' not found in semantic store")
            response.success = False
            response.location_id = ''
            response.message = f"Location '{query}' not found"
            return response
        
        frame_id = resolved.get("frame_id", "map")
        if frame_id != "map":
            self.get_logger().warn(f"Location '{query}' has unsupported frame_id '{frame_id}'")
            response.success = False
            response.location_id = resolved["location_id"]
            response.message = f"Unsupported frame_id '{frame_id}' for location '{query}'"
            return response
        
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = float(resolved["x"])
        pose.pose.position.y = float(resolved["y"])
        pose.pose.position.z = 0.0

        qz, qw = yaw_to_quaternion(float(resolved["yaw"]))
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = qz
        pose.pose.orientation.w = qw

        response.success = True
        response.location_id = resolved["location_id"]
        response.pose = pose
        response.message = f"Location '{query}' resolved successfully"
        return response

def main(args=None):
    rclpy.init(args=args)
    resolver_node = ResolverNode()
    rclpy.spin(resolver_node)
    resolver_node.destroy_node()
    rclpy.shutdown()