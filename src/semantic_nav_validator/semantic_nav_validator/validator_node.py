import math
import rclpy
import threading

from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from action_msgs.msg import GoalStatus
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

class ValidatorNode(Node):
    def __init__(self):
        super().__init__('semantic_nav_validator')

        self.declare_parameter('compute_path_action', '/compute_path_to_pose')
        self.declare_parameter('default_planner_id', '')
        self.declare_parameter('validation_timeout_sec', 10.0)

        action_name = self.get_parameter('compute_path_action').get_parameter_value().string_value
        self._default_planner_id = self.get_parameter('default_planner_id').get_parameter_value().string_value
        self._timeout = float(self.get_parameter('validation_timeout_sec').value)
        self._cb_group = ReentrantCallbackGroup()

        self._planner_client = ActionClient(self, ComputePathToPose, action_name, callback_group=self._cb_group)

        self._srv = self.create_service(
            ValidatePose,
            'validate_pose_goal',
            self.validate_pose_callback,
            callback_group=self._cb_group
        )

        self.get_logger().info(f'Validator initialized, waiting for {action_name} action server...')

    def validate_pose_callback(self, request, response):
        self.get_logger().info('Received validation request')
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
        
        done_event = threading.Event()
        result_box = {
            'error': None,
            'status': None,
            'result': None,
        }

        self.get_logger().info('Sending ComputePathToPose goal')

        send_goal_future = self._planner_client.send_goal_async(action_goal, feedback_callback=None)

        def on_goal_response(future):
            try:
                goal_handle = future.result()
            except Exception as e:
                result_box['error'] = f'Planner action call failed: {str(e)}'
                done_event.set()
                return
            
            if goal_handle is None or not goal_handle.accepted:
                result_box['error'] = 'Planner rejected validation request'
                done_event.set()
                return
            
            self.get_logger().info('Goal accepted by planner, waiting for result...')

            result_future = goal_handle.get_result_async()

            def on_result(future):
                try:
                    result_wrap = future.result()
                except Exception as e:
                    result_box['error'] = f'Failed to get result from planner: {str(e)}'
                    done_event.set()
                    return
                
                if result_wrap is None:
                        result_box['error'] = 'Planner failed to return a result'
                        done_event.set()
                        return
                
                result_box['status'] = result_wrap.status
                result_box['result'] = result_wrap.result
                done_event.set()
            
            result_future.add_done_callback(on_result)
        
        send_goal_future.add_done_callback(on_goal_response)

        if not done_event.wait(timeout=self._timeout):
            response.valid = False
            response.message = 'Planner validation timed out'
            response.path_length = 0.0
            response.pose_count = 0
            return response
        
        if result_box['error'] is not None:
            response.valid = False
            response.message = result_box['error']
            response.path_length = 0.0
            response.pose_count = 0
            return response
        
        status = result_box['status']
        result = result_box['result']

        if status != GoalStatus.STATUS_SUCCEEDED:
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
    validator = ValidatorNode()

    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(validator)
    try:
        executor.spin()
    finally:
        rclpy.spin(validator)
        validator.destroy_node()
        rclpy.shutdown()
        