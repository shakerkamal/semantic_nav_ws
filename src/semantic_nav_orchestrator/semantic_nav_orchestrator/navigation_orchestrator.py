import sys
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from semantic_nav_interfaces.action import ExecutePose
from semantic_nav_interfaces.srv import ResolveLocation, ValidatePose

class NavigationOrchestrator(Node):
    def __init__(self):
        super().__init__('navigation_orchestrator')
        self.declare_parameter('query', '')
        self.declare_parameter('resolve_service', '/resolve_location')
        self.declare_parameter('validate_service', '/validate_pose_goal')
        self.declare_parameter('execute_action', '/execute_pose')
        self.declare_parameter('planner_id', '')  # Optional: specify a planner if needed
        self.declare_parameter('enable_validation', True) 

        self._query = self.get_parameter('query').get_parameter_value().string_value.strip()
        self._resolve_service_name = self.get_parameter('resolve_service').get_parameter_value().string_value
        self._validate_service_name = self.get_parameter('validate_service').get_parameter_value().string_value
        self._execute_action_name = self.get_parameter('execute_action').get_parameter_value().string_value
        self._planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
        self._enable_validation = self.get_parameter('enable_validation').get_parameter_value().bool_value

        self._resolve_location_client = self.create_client(ResolveLocation, self._resolve_service_name)
        self._validate_pose_client = self.create_client(ValidatePose, self._validate_service_name)
        self._execute_pose_client = ActionClient(self, ExecutePose, self._execute_action_name)

        self.get_logger().info(f'Navigation Orchestrator initialized with query: "{self._query}"')

        self._done = False
        self._final_success = False
        self._goal_handle = None
        self._result_future = None
        
        # # Create service client for resolving location
        # self._resolve_location_client = self.create_client(ResolveLocation, 'resolve_location')
        # while not self._resolve_location_client.wait_for_service(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for resolve_location service...')
        
        # # Create action client for executing pose
        # self._execute_pose_client = ActionClient(self, ExecutePose, 'execute_pose')
        # while not self._execute_pose_client.wait_for_server(timeout_sec=1.0):
        #     self.get_logger().info('Waiting for execute_pose action server...')

    def run(self) -> bool:
        if not self._query:
            self.get_logger().error('No query provided. Please set the "query" parameter.')
            return False
        
        pose = self._resolve_query()
        if pose is None:
            self.get_logger().error('Failed to resolve query to a pose.')
            return False
        
        if self._enable_validation:
            self.get_logger().info('Validating resolved pose with planner...')
            if not self._validate_pose(pose):
                self.get_logger().error('Pose validation failed. Aborting navigation.')
                return False
            self.get_logger().info('Pose validation succeeded.')
        
        return self._execute_pose(pose)
    
    def _resolve_query(self):
        self.get_logger().info(f'Resolving location for query: "{self._query}"')

        if not self._resolve_location_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'Resolve location service "{self._resolve_service_name}" not available')
            return False
        
        req = ResolveLocation.Request()
        req.query = self._query

        future = self._resolve_location_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(f'Failed to call resolve_location service: {future.exception()}')
            return False
        
        response = future.result()

        if not response.success:
            self.get_logger().error(f'Location resolution failed: {response.message}')
            return False
        
        self.get_logger().info(f'Location resolved: {response.location_name}, executing pose...')

        pose = response.pose
        self.get_logger().info(
            f"Resolved '{self._query}' -> location_id='{response.place_id}', "
            f"frame='{pose.header.frame_id}', "
            f"x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}"
        )

        return pose
    
    def _validate_pose(self, pose) -> bool:
        self.get_logger().info('Validating goal with ComputePathToPose...')

        if not self._validate_pose_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f'Validate pose service "{self._validate_service_name}" not available')
            return False
        
        req = ValidatePose.Request()
        req.goal = pose
        req.planner_id = self._planner_id
        req.use_start = False 

        future = self._validate_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is None:
            self.get_logger().error(f'Validation service call failed: {future.exception()}')
            return False
        
        response = future.result()

        if not response.valid:
            self.get_logger().error(f'Goal validation failed: {response.message}')
            return False
        
        self.get_logger().info(
            f"Validation succeeded: {response.message}, "
            f"path_length={response.path_length:.3f}, pose_count={response.pose_count}"
        )
        return True
    
    def _execute_pose(self, pose) -> bool:
        if not self._execute_pose_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f'Execute pose action server "{self._execute_action_name}" not available')
            return False
        
        goal_msg = ExecutePose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = '' # Optional: specify a behavior tree if needed

        self.get_logger().info(f'Sending goal to execute_pose action server: frame={pose.header.frame_id}, x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}')

        send_goal_future = self._execute_pose_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        rclpy.spin_until_future_complete(self, send_goal_future)

        self._goal_handle = send_goal_future.result()
        if self._goal_handle is None:
            self.get_logger().error("Failed to get goal handle from executor.")
            return False
        
        if not self._goal_handle.accepted:
            self.get_logger().error('Goal rejected by action server')
            return False
        
        self.get_logger().info('Goal accepted, waiting for result...')
        self._result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._result_future)

        result_wrap = self._result_future.result()
        if result_wrap is None:
            self.get_logger().error(f'Failed to get result from execute_pose action: {self._result_future.exception()}')
            return False
        
        result = result_wrap.result
        status = result_wrap.status

        self.get_logger().info(
            f"Executor finished with status={status}, success={result.success}, message='{result.message}'"
        )

        self._final_success = bool(result.success)
        return self._final_success

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self.get_logger().info(
            "Feedback: "
            f"distance_remaining={fb.distance_remaining:.3f}, "
            f"recoveries={fb.number_of_recoveries}, "
            f"current_x={fb.current_pose.pose.position.x:.3f}, "
            f"current_y={fb.current_pose.pose.position.y:.3f}"
        )

    def cancel_goal(self):
        if self._goal_handle is not None:
            self.get_logger().info('Cancel request received. Cancelling goal...')
            cancel_future = self._goal_handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel_future)

def extract_query_from_argv() -> Optional[str]:
    """
    Extract the query string from the command line arguments.
    Supports:
      - ros2 run ... navigation_orchestrator --ros-args -p query:=kitchen
      - ros2 run ... navigation_orchestrator kitchen
    """
    argv = sys.argv[1:]
    positional = [arg for arg in argv if not arg.startswith('-') and ':=' not in arg]
    if positional:
        return " ".join(positional).strip()
    return None

def main(args=None):
    rclpy.init(args=args)
    node = NavigationOrchestrator()

    cli_query = extract_query_from_argv()
    if cli_query:
        node.get_logger().info(f'Overriding query parameter with command line argument: "{cli_query}"')
        node.set_parameters([
            rclpy.parameter.Parameter(
                'query', 
                rclpy.parameter.Parameter.Type.STRING, 
                cli_query
            )
        ])
        node._query = cli_query
    
    try:
        success = node.run()
        if success:
            node.get_logger().info('Navigation task completed successfully!')
        else:
            node.get_logger().error('Navigation task failed.')
        raise SystemExit(0 if success else 1)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, cancelling goal...')
        node.cancel_goal()
        raise SystemExit(130)
    finally:
        node.destroy_node()
        rclpy.shutdown()
