import math
import rclpy

from rclpy.node import Node
from rclpy.action import ActionClient

from nav2_msgs.action import ComputePathToPose
from semantic_nav_interfaces.srv import ValidatePose

def path_length(path_msg):
    poses = path_msg.poses
    if len(poses) < 2:
        return 0.0
    total_length = 0.0
    for i in range(1, len(poses)):
        p0 = poses[i-1].pose.position
        p1 = poses[i].pose.position
        total_length += math.hypot(p1.x - p0.x, p1.y - p0.y)
    return total_length

class SemanticNavValidator(Node):
    def __init__(self):
        super().__init__('semantic_nav_validator')

        self.declare_parameter('compute_path_action', '/compute_path_to_pose')
        self.declare_parameter('default_planner_id', '')

        action_name = self.get_parameter('compute_path_action').get_parameter_value().string_value
        self._default_planner_id = self.get_parameter('default_planner_id').get_parameter_value().string_value

        self._planner_client = ActionClient(self, ComputePathToPose, action_name)

        self._srv = self.create_service(
            ValidatePose,
            'validate_pose_goal',
            self.validate_pose_callback
        )

        self.get_logger().info(f'Validator initialized, waiting for {action_name} action server...')

    def validate_pose_callback(self, request, response):
        goal_pose = request.goal

        if goal_pose.header.frame_id != 'map':
            response.valid = False
            response.message = f'Goal frame_id must be "map", received "{goal_pose.header.frame_id}"'
            response.path_length = 0.0
            response.pose_count = 0
            return response
        
        if not self._planner_client.wait_for_server(timeout_sec=5.0):
            response.valid = False
            response.message = 'ComputePathToPose action server not available'
            response.path_length = 0.0
            response.pose_count = 0
            return response
        
        action_goal = ComputePathToPose.Goal()
        action_goal.goal = goal_pose
        action_goal.planner_id = request.planner_id if request.planner_id else self._default_planner_id
        action_goal.use_start = bool(request.use_start)
        if action_goal.use_start:
            action_goal.start = request.start

        send_goal_future = self._planner_client.send_goal_async(action_goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if goal_handle is None or not goal_handle.accepted:
            response.valid = False
            response.message = 'Planner rejected validation request'
            response.path_length = 0.0
            response.pose_count = 0
            return response
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result_wrap = result_future.result()
        if result_wrap is None:
            response.valid = False
            response.message = 'Planner failed to return a result'
            response.path_length = 0.0
            response.pose_count = 0
            return response
        
        status = result_wrap.status
        result = result_wrap.result

        if status != 4:  # 4 == SUCCEEDED
            response.valid = False
            response.message = f'Planner failed with status code {status}'
            response.path_length = 0.0
            response.pose_count = 0
            return response
        
        if len(result.path.poses) == 0:
            response.valid = False
            response.message = 'Planner returned an empty path'
            response.path_length = 0.0
            response.pose_count = 0
            return response
        
        response.valid = True
        response.message = 'Pose is valid and reachable'
        response.path_length = float(path_length(result.path))
        response.pose_count = len(result.path.poses)
        return response
    
def main(args=None):
    rclpy.init(args=args)
    validator = SemanticNavValidator()
    rclpy.spin(validator)
    validator.destroy_node()
    rclpy.shutdown()
        