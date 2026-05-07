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

        self._goal_handle = None
        self._result_future = None
        self._final_success = False
        self._db_version: int = 0

        self.get_logger().info(f'Navigation Orchestrator initialized with query: "{self._query}"')

    def _log_stage_info(self, stage: str, message: str):
        self.get_logger().info(f'[{stage}] {message}')

    def _log_stage_error(self, stage: str, message: str):
        self.get_logger().error(f'[{stage}] {message}')             

    def run(self) -> bool:
        if not self._query:
            self._log_stage_error('RESOLUTION', 'No query provided. Please set the "query" parameter.')
            return False
        
        pose = self._resolve_query()
        if pose is None:
            self._log_stage_error('RESOLUTION', 'Failed to resolve query to a pose.')
            return False
        
        if self._enable_validation:
            self._log_stage_info('VALIDATION', 'Validating resolved pose with planner...')
            if not self._validate_pose(pose):
                self._log_stage_error('VALIDATION', 'Pose validation failed. Aborting navigation.')
                return False
            self._log_stage_info('VALIDATION', 'Pose validation succeeded.')
        
        if not self._execute_pose(pose):
            self._log_stage_error('EXECUTION', 'Failed to execute pose. Navigation unsuccessful.')
            return False
        
        return True
    
    def _resolve_query(self):
        self._log_stage_info('RESOLUTION', f'Resolving location for query: "{self._query}"')

        if not self._resolve_location_client.wait_for_service(timeout_sec=10.0):
            self._log_stage_error('RESOLUTION', f'Resolve location service "{self._resolve_service_name}" not available')
            return None
        
        req = ResolveLocation.Request()
        req.query = self._query

        future = self._resolve_location_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.done():
            self._log_stage_error('RESOLUTION', 'Service call to resolve location did not complete')
            return None
        
        if future.exception() is not None:
            self._log_stage_error('RESOLUTION', f'Failed to call resolve_location service: {future.exception()}')
            return None
        
        response = future.result()
        if response is None:
            self._log_stage_error('RESOLUTION', 'Resolve location service returned no response.')
            return None

        if not response.success:
            self._log_stage_error('RESOLUTION', f'Location resolution failed: {response.message}')
            return None

        self._db_version = response.db_version
        self._log_stage_info('RESOLUTION', f'Location resolved: {response.location_id} (db_version={self._db_version})')

        pose = response.pose
        if pose is None or pose.header.frame_id == '':
            self._log_stage_error('RESOLUTION', 'Resolution succeeded but returned an invalid pose.')
            return None

        self._log_stage_info('RESOLUTION',
            f"Resolved '{self._query}' -> location_id='{response.location_id}', "
            f"db_version={self._db_version}, "
            f"frame='{pose.header.frame_id}', "
            f"x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}"
        )

        return pose
    
    def _validate_pose(self, pose) -> bool:
        self._log_stage_info('VALIDATION', f'Validating goal with ComputePathToPose (db_version={self._db_version})...')

        if pose is None:
            self._log_stage_error('VALIDATION', 'No pose provided for validation.')
            return False

        if not self._validate_pose_client.wait_for_service(timeout_sec=10.0):
            self._log_stage_error('VALIDATION', f'Validate pose service "{self._validate_service_name}" not available')
            return False
        
        req = ValidatePose.Request()
        req.goal = pose
        req.planner_id = self._planner_id
        req.use_start = False 

        future = self._validate_pose_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if not future.done():
            self._log_stage_error('VALIDATION', 'Service call to validate pose did not complete')
            return False

        if future.exception() is not None:
            self._log_stage_error('VALIDATION', f'Validation service call failed: {future.exception()}')
            return False

        response = future.result()
        if response is None:
            self._log_stage_error('VALIDATION', 'Validate pose service returned no response.')
            return False
        
        if not response.valid:
            self._log_stage_error('VALIDATION', f'Goal validation failed: {response.message}')
            return False
        
        self._log_stage_info('VALIDATION',
            f"Validation succeeded: {response.message}, "
            f"path_length={response.path_length:.3f}, pose_count={response.pose_count}"
        )
        return True
    
    def _execute_pose(self, pose) -> bool:
        if pose is None:
            self._log_stage_error('EXECUTION', 'No pose provided for execution.')
            return False
        
        if not self._execute_pose_client.wait_for_server(timeout_sec=10.0):
            self._log_stage_error('EXECUTION', f'Execute pose action server "{self._execute_action_name}" not available')
            return False
        
        goal_msg = ExecutePose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = '' # Optional: specify a behavior tree if needed

        self._log_stage_info('EXECUTION',
            f"Sending goal to execute_pose action server (db_version={self._db_version}): "
            f"frame={pose.header.frame_id}, "
            f"x={pose.pose.position.x:.3f}, y={pose.pose.position.y:.3f}"
        )

        send_goal_future = self._execute_pose_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        rclpy.spin_until_future_complete(self, send_goal_future)

        if not send_goal_future.done():
            self._log_stage_error('EXECUTION', 'Send goal to execute_pose action server did not complete')
            return False
        
        if send_goal_future.exception() is not None:
            self._log_stage_error('EXECUTION', f'Failed to send goal to execute_pose action server: {send_goal_future.exception()}')
            return False

        self._goal_handle = send_goal_future.result()
        if self._goal_handle is None:
            self._log_stage_error('EXECUTION', "Failed to get goal handle from executor.")
            return False
        
        if not self._goal_handle.accepted:
            self._log_stage_error('EXECUTION', 'Goal rejected by action server')
            return False
        
        self._log_stage_info('EXECUTION', 'Goal accepted, waiting for result...')
        self._result_future = self._goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, self._result_future)

        if not self._result_future.done():
            self._log_stage_error('EXECUTION', 'ExecutePose result did not complete.')
            return False

        if self._result_future.exception() is not None:
            self._log_stage_error('EXECUTION', f'Failed to get result from execute_pose action: {self._result_future.exception()}')
            return False

        result_wrap = self._result_future.result()
        if result_wrap is None:
            self._log_stage_error('EXECUTION', f'Failed to get result from execute_pose action: {self._result_future.exception()}')
            return False
        
        result = result_wrap.result
        status = result_wrap.status

        self._log_stage_info('EXECUTION',
            f"Executor finished with status={status}, success={result.success}, "
            f"db_version={self._db_version}, message='{result.message}'"
        )

        return bool(result.success)

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback
        self._log_stage_info('EXECUTION',
            "Feedback: "
            f"distance_remaining={fb.distance_remaining:.3f}, "
            f"recoveries={fb.number_of_recoveries}, "
            f"current_x={fb.current_pose.pose.position.x:.3f}, "
            f"current_y={fb.current_pose.pose.position.y:.3f}"
        )

    def cancel_goal(self):
        if self._goal_handle is not None:
            self._log_stage_info('EXECUTION', 'Cancel request received. Cancelling goal...')
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
