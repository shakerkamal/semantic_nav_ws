import math
import sys
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter

from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped

from semantic_nav_interfaces.action import ExecutePose
from semantic_nav_interfaces.srv import ResolveLocation, ValidatePose, ParseSemanticCommand


@dataclass(frozen=True)
class ResolvedTarget:
    query: str
    location_id: str
    pose: PoseStamped
    db_version: int
    db_stamp: Time

@dataclass(frozen=True)
class ParsedCommand:
    original_command: str
    intent: str
    location_query: str
    canonical_location_id: str
    confidence_percent: int
    raw_output: str

class NavigationOrchestrator(Node):
    def __init__(self):
        super().__init__('navigation_orchestrator')

        self.declare_parameter('query', '')

        self.declare_parameter("command", "")
        self.declare_parameter("parse_service", "/parse_semantic_command")

        self.declare_parameter('resolve_service', '/resolve_location')
        self.declare_parameter('validate_service', '/validate_pose_goal')
        self.declare_parameter('execute_action', '/execute_pose')

        self.declare_parameter('planner_id', '')
        self.declare_parameter('behavior_tree', '')
        self.declare_parameter('enable_validation', True)

        self.declare_parameter('service_wait_timeout_sec', 30.0)
        self.declare_parameter('service_call_timeout_sec', 30.0)
        self.declare_parameter('action_server_wait_timeout_sec', 10.0)
        self.declare_parameter('action_send_goal_timeout_sec', 10.0)

        # Set <= 0.0 for no execution timeout.
        self.declare_parameter('execution_timeout_sec', 300.0)

        self._query = self.get_parameter('query').get_parameter_value().string_value.strip()
        self._command = self.get_parameter('command').get_parameter_value().string_value.strip()
        self._parse_service_name = self.get_parameter('parse_service').get_parameter_value().string_value
        self._resolve_service_name = self.get_parameter('resolve_service').get_parameter_value().string_value
        self._validate_service_name = self.get_parameter('validate_service').get_parameter_value().string_value
        self._execute_action_name = self.get_parameter('execute_action').get_parameter_value().string_value
        self._planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
        self._behavior_tree = self.get_parameter('behavior_tree').get_parameter_value().string_value
        self._enable_validation = self.get_parameter('enable_validation').get_parameter_value().bool_value

        self._service_wait_timeout_sec = (
            self.get_parameter('service_wait_timeout_sec').get_parameter_value().double_value
        )
        self._service_call_timeout_sec = (
            self.get_parameter('service_call_timeout_sec').get_parameter_value().double_value
        )
        self._action_server_wait_timeout_sec = (
            self.get_parameter('action_server_wait_timeout_sec').get_parameter_value().double_value
        )
        self._action_send_goal_timeout_sec = (
            self.get_parameter('action_send_goal_timeout_sec').get_parameter_value().double_value
        )
        self._execution_timeout_sec = (
            self.get_parameter('execution_timeout_sec').get_parameter_value().double_value
        )

        self._resolve_location_client = self.create_client(ResolveLocation, self._resolve_service_name)
        self._validate_pose_client = self.create_client(ValidatePose, self._validate_service_name)
        self._execute_pose_client = ActionClient(self, ExecutePose, self._execute_action_name)
        self._parse_command_client = self.create_client(ParseSemanticCommand, self._parse_service_name)

        self._goal_handle = None
        self._result_future = None
        self._final_success = False

        self._resolved_target: Optional[ResolvedTarget] = None
        self._parsed_command: Optional[ParsedCommand] = None

        self._db_version: int = 0
        self._db_stamp: Optional[Time] = None

        self.get_logger().info(f'Navigation Orchestrator initialized with query: "{self._query}"')

    def _log_stage_info(self, stage: str, message: str):
        self.get_logger().info(f'[{stage}] {message}')

    def _log_stage_warn(self, stage: str, message: str):
        self.get_logger().warn(f'[{stage}] {message}')

    def _log_stage_error(self, stage: str, message: str):
        self.get_logger().error(f'[{stage}] {message}')      

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        if timeout_sec is None or timeout_sec <= 0.0:
            rclpy.spin_until_future_complete(self, future)
        else:
            rclpy.spin_until_future_complete(self, future, timeout_sec=timeout_sec)

        return future.done()

    @staticmethod
    def _stamp_to_string(stamp: Optional[Time]) -> str:
        if stamp is None:
            return 'unset'
        return f'{stamp.sec}.{stamp.nanosec:09d}'    

    @staticmethod
    def _goal_status_to_string(status: int) -> str:
        names = {
            GoalStatus.STATUS_UNKNOWN: 'UNKNOWN',
            GoalStatus.STATUS_ACCEPTED: 'ACCEPTED',
            GoalStatus.STATUS_EXECUTING: 'EXECUTING',
            GoalStatus.STATUS_CANCELING: 'CANCELING',
            GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED',
            GoalStatus.STATUS_CANCELED: 'CANCELED',
            GoalStatus.STATUS_ABORTED: 'ABORTED',
        }
        return names.get(status, f'UNRECOGNIZED({status})')   
    
    def _pose_is_valid_for_navigation(self, pose: PoseStamped) -> bool:
        if pose is None:
            self._log_stage_error('RESOLUTION', 'Resolution succeeded but returned pose=None.')
            return False

        if pose.header.frame_id == '':
            self._log_stage_error('RESOLUTION', 'Resolution succeeded but returned an empty frame_id.')
            return False

        if pose.header.frame_id != 'map':
            self._log_stage_error(
                'RESOLUTION',
                f"Resolution returned frame='{pose.header.frame_id}', expected 'map'.",
            )
            return False

        p = pose.pose.position
        q = pose.pose.orientation

        values = [
            p.x, p.y, p.z,
            q.x, q.y, q.z, q.w,
        ]

        if not all(math.isfinite(v) for v in values):
            self._log_stage_error(
                'RESOLUTION',
                'Resolution returned non-finite pose values.',
            )
            return False

        q_norm = math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w)
        if q_norm < 1e-6:
            self._log_stage_error(
                'RESOLUTION',
                'Resolution returned an invalid near-zero quaternion.',
            )
            return False

        return True

    def run(self) -> bool:
        semantic_query = self._get_semantic_query()
        if semantic_query is None:
            return False

        target = self._resolve_query(semantic_query)
        if target is None:
            self._log_stage_error(
                "RESOLUTION",
                "Failed to resolve query to a valid navigation target.",
            )
            return False

        if self._enable_validation:
            self._log_stage_info(
                "VALIDATION",
                (
                    f"Validating resolved pose with planner "
                    f"(location_id='{target.location_id}', "
                    f"db_version={target.db_version}, "
                    f"db_stamp={self._stamp_to_string(target.db_stamp)})..."
                ),
            )

            if not self._validate_pose(target):
                self._log_stage_error(
                    "VALIDATION",
                    "Pose validation failed. Aborting navigation.",
                )
                return False

            self._log_stage_info(
                "VALIDATION",
                (
                    f"Pose validation succeeded "
                    f"(location_id='{target.location_id}', "
                    f"db_version={target.db_version})."
                ),
            )
        else:
            self._log_stage_warn(
                "VALIDATION",
                (
                    f"Validation disabled. Proceeding directly to execution "
                    f"(location_id='{target.location_id}', "
                    f"db_version={target.db_version})."
                ),
            )

        if not self._execute_pose(target):
            self._log_stage_error(
                "EXECUTION",
                "Failed to execute pose. Navigation unsuccessful.",
            )
            return False

        return True

    def _get_semantic_query(self) -> Optional[str]:
        """
        Select the semantic query source.

        Priority:
          1. direct query parameter / positional CLI query
          2. NL command parsed through /parse_semantic_command

        This preserves the existing deterministic path:
          navigation_orchestrator kitchen

        and adds:
          navigation_orchestrator --ros-args -p command:="I am hungry"
        """
        if self._query:
            if self._command:
                self._log_stage_warn(
                    "INTENT",
                    (
                        "Both 'query' and 'command' were provided. "
                        "Using direct semantic query and bypassing LLM parsing."
                    ),
                )

            self._log_stage_info(
                "INTENT",
                f"Using direct semantic query: '{self._query}'",
            )
            return self._query

        if not self._command:
            self._log_stage_error(
                "INTENT",
                (
                    "No navigation input provided. Set either 'query' for a direct "
                    "semantic target or 'command' for natural-language parsing."
                ),
            )
            return None

        parsed = self._parse_command(self._command)
        if parsed is None:
            return None

        self._parsed_command = parsed

        # Prefer canonical_location_id for deterministic downstream resolution.
        # Fall back to location_query if the parser did not provide canonical ID.
        semantic_query = parsed.canonical_location_id or parsed.location_query

        self._log_stage_info(
            "INTENT",
            (
                f"Natural-language command parsed: "
                f"command='{parsed.original_command}', "
                f"intent='{parsed.intent}', "
                f"location_query='{parsed.location_query}', "
                f"canonical_location_id='{parsed.canonical_location_id}', "
                f"semantic_query='{semantic_query}', "
                f"confidence={parsed.confidence_percent}"
            ),
        )

        return semantic_query

    def _parse_command(self, command: str) -> Optional[ParsedCommand]:
        self._log_stage_info(
            "INTENT",
            f"Parsing natural-language command: '{command}'",
        )

        if not self._parse_command_client.wait_for_service(
            timeout_sec=self._service_wait_timeout_sec
        ):
            self._log_stage_error(
                "INTENT",
                (
                    f"Parse semantic command service "
                    f"'{self._parse_service_name}' not available."
                ),
            )
            return None

        req = ParseSemanticCommand.Request()
        req.command = command

        future = self._parse_command_client.call_async(req)

        if not self._wait_for_future(future, self._service_call_timeout_sec):
            self._log_stage_error(
                "INTENT",
                (
                    f"Service call to parse semantic command timed out after "
                    f"{self._service_call_timeout_sec:.1f}s."
                ),
            )
            return None

        if future.exception() is not None:
            self._log_stage_error(
                "INTENT",
                f"Parse semantic command service call failed: {future.exception()}",
            )
            return None

        response = future.result()
        if response is None:
            self._log_stage_error(
                "INTENT",
                "Parse semantic command service returned no response.",
            )
            return None

        self._log_stage_info(
            "INTENT",
            (
                f"Parser response: success={response.success}, "
                f"intent='{response.intent}', "
                f"location_query='{response.location_query}', "
                f"canonical_location_id='{response.canonical_location_id}', "
                f"confidence={response.confidence_percent}, "
                f"location_known={response.location_known}, "
                f"message='{response.message}'"
            ),
        )

        if not response.success:
            self._log_stage_error(
                "INTENT",
                f"Command parsing failed: {response.message}",
            )
            return None

        if response.intent != "navigate_to_location":
            self._log_stage_error(
                "INTENT",
                (
                    f"Parsed command is not executable navigation: "
                    f"intent='{response.intent}', message='{response.message}'"
                ),
            )
            return None

        if not response.location_known:
            self._log_stage_error(
                "INTENT",
                (
                    f"Parsed location is not known: "
                    f"location_query='{response.location_query}', "
                    f"message='{response.message}'"
                ),
            )
            return None

        if not response.location_query and not response.canonical_location_id:
            self._log_stage_error(
                "INTENT",
                "Parser returned navigate_to_location but no usable location query.",
            )
            return None

        return ParsedCommand(
            original_command=command,
            intent=response.intent,
            location_query=response.location_query,
            canonical_location_id=response.canonical_location_id,
            confidence_percent=int(response.confidence_percent),
            raw_output=response.raw_output,
        )
    
    def _resolve_query(self, query: str) -> Optional[ResolvedTarget]:
        self._log_stage_info(
            "RESOLUTION",
            f'Resolving location for query: "{query}"',
        )

        if not self._resolve_location_client.wait_for_service(
            timeout_sec=self._service_wait_timeout_sec
        ):
            self._log_stage_error(
                "RESOLUTION",
                (
                    f'Resolve location service "{self._resolve_service_name}" '
                    "not available."
                ),
            )
            return None

        req = ResolveLocation.Request()
        req.query = query

        future = self._resolve_location_client.call_async(req)

        if not self._wait_for_future(future, self._service_call_timeout_sec):
            self._log_stage_error(
                "RESOLUTION",
                (
                    f"Service call to resolve location timed out after "
                    f"{self._service_call_timeout_sec:.1f}s."
                ),
            )
            return None

        if future.exception() is not None:
            self._log_stage_error(
                "RESOLUTION",
                f"Failed to call resolve_location service: {future.exception()}",
            )
            return None

        response = future.result()
        if response is None:
            self._log_stage_error(
                "RESOLUTION",
                "Resolve location service returned no response.",
            )
            return None

        if not response.success:
            self._log_stage_error(
                "RESOLUTION",
                f"Location resolution failed: {response.message}",
            )
            return None

        pose = response.pose
        if not self._pose_is_valid_for_navigation(pose):
            return None

        target = ResolvedTarget(
            query=query,
            location_id=response.location_id,
            pose=pose,
            db_version=int(response.db_version),
            db_stamp=response.db_stamp,
        )

        self._resolved_target = target

        self._log_stage_info(
            "RESOLUTION",
            (
                f"Resolved '{target.query}' -> "
                f"location_id='{target.location_id}', "
                f"db_version={target.db_version}, "
                f"db_stamp={self._stamp_to_string(target.db_stamp)}, "
                f"frame='{target.pose.header.frame_id}', "
                f"x={target.pose.pose.position.x:.3f}, "
                f"y={target.pose.pose.position.y:.3f}"
            ),
        )

        return target
    
    def _validate_pose(self, target: ResolvedTarget) -> bool:
        if target is None or target.pose is None:
            self._log_stage_error(
                'VALIDATION',
                'No resolved target provided for validation.',
            )
            return False

        self._log_stage_info(
            'VALIDATION',
            (
                f"Validating goal with ComputePathToPose "
                f"(location_id='{target.location_id}', "
                f"db_version={target.db_version}, "
                f"db_stamp={self._stamp_to_string(target.db_stamp)})..."
            ),
        )

        if not self._validate_pose_client.wait_for_service(
            timeout_sec=self._service_wait_timeout_sec
        ):
            self._log_stage_error(
                'VALIDATION',
                f'Validate pose service "{self._validate_service_name}" not available.',
            )
            return False

        req = ValidatePose.Request()
        req.goal = target.pose
        req.planner_id = self._planner_id
        req.use_start = False

        future = self._validate_pose_client.call_async(req)

        if not self._wait_for_future(future, self._service_call_timeout_sec):
            self._log_stage_error(
                'VALIDATION',
                (
                    f'Service call to validate pose timed out after '
                    f'{self._service_call_timeout_sec:.1f}s.'
                ),
            )
            return False

        if future.exception() is not None:
            self._log_stage_error(
                'VALIDATION',
                f'Validation service call failed: {future.exception()}',
            )
            return False

        response = future.result()
        if response is None:
            self._log_stage_error(
                'VALIDATION',
                'Validate pose service returned no response.',
            )
            return False

        if not response.valid:
            self._log_stage_error(
                'VALIDATION',
                (
                    f"Goal validation failed "
                    f"(location_id='{target.location_id}', "
                    f"db_version={target.db_version}): {response.message}"
                ),
            )
            return False

        self._log_stage_info(
            'VALIDATION',
            (
                f"Validation succeeded "
                f"(location_id='{target.location_id}', "
                f"db_version={target.db_version}): "
                f"{response.message}, "
                f"path_length={response.path_length:.3f}, "
                f"pose_count={response.pose_count}"
            ),
        )

        return True
    
    def _execute_pose(self, target: ResolvedTarget) -> bool:
        if target is None or target.pose is None:
            self._log_stage_error(
                'EXECUTION',
                'No resolved target provided for execution.',
            )
            return False

        pose = target.pose

        if not self._execute_pose_client.wait_for_server(
            timeout_sec=self._action_server_wait_timeout_sec
        ):
            self._log_stage_error(
                'EXECUTION',
                f'Execute pose action server "{self._execute_action_name}" not available.',
            )
            return False

        goal_msg = ExecutePose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = self._behavior_tree

        self._log_stage_info(
            'EXECUTION',
            (
                f"Sending goal to execute_pose action server "
                f"(location_id='{target.location_id}', "
                f"db_version={target.db_version}, "
                f"db_stamp={self._stamp_to_string(target.db_stamp)}): "
                f"frame='{pose.header.frame_id}', "
                f"x={pose.pose.position.x:.3f}, "
                f"y={pose.pose.position.y:.3f}"
            ),
        )

        send_goal_future = self._execute_pose_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )

        if not self._wait_for_future(send_goal_future, self._action_send_goal_timeout_sec):
            self._log_stage_error(
                'EXECUTION',
                (
                    f'Send goal to execute_pose action server timed out after '
                    f'{self._action_send_goal_timeout_sec:.1f}s.'
                ),
            )
            return False

        if send_goal_future.exception() is not None:
            self._log_stage_error(
                'EXECUTION',
                (
                    f'Failed to send goal to execute_pose action server: '
                    f'{send_goal_future.exception()}'
                ),
            )
            return False

        self._goal_handle = send_goal_future.result()
        if self._goal_handle is None:
            self._log_stage_error(
                'EXECUTION',
                'Failed to get goal handle from executor.',
            )
            return False

        if not self._goal_handle.accepted:
            self._log_stage_error(
                'EXECUTION',
                (
                    f"Goal rejected by action server "
                    f"(location_id='{target.location_id}', "
                    f"db_version={target.db_version})."
                ),
            )
            return False

        self._log_stage_info(
            'EXECUTION',
            (
                f"Goal accepted, waiting for result "
                f"(location_id='{target.location_id}', "
                f"db_version={target.db_version})."
            ),
        )

        self._result_future = self._goal_handle.get_result_async()

        if not self._wait_for_future(self._result_future, self._execution_timeout_sec):
            self._log_stage_error(
                'EXECUTION',
                (
                    f'ExecutePose result timed out after '
                    f'{self._execution_timeout_sec:.1f}s. Cancelling goal.'
                ),
            )
            self.cancel_goal()
            return False

        if self._result_future.exception() is not None:
            self._log_stage_error(
                'EXECUTION',
                f'Failed to get result from execute_pose action: {self._result_future.exception()}',
            )
            return False

        result_wrap = self._result_future.result()
        if result_wrap is None:
            self._log_stage_error(
                'EXECUTION',
                'ExecutePose action returned no result wrapper.',
            )
            return False

        result = result_wrap.result
        status = result_wrap.status
        status_name = self._goal_status_to_string(status)

        self._log_stage_info(
            'EXECUTION',
            (
                f"Executor finished with status={status_name}({status}), "
                f"success={result.success}, "
                f"location_id='{target.location_id}', "
                f"db_version={target.db_version}, "
                f"db_stamp={self._stamp_to_string(target.db_stamp)}, "
                f"message='{result.message}'"
            ),
        )

        succeeded = (
            status == GoalStatus.STATUS_SUCCEEDED
            and bool(result.success)
        )

        if not succeeded:
            self._log_stage_error(
                'EXECUTION',
                (
                    f"Execution failed or ended with non-success status: "
                    f"status={status_name}({status}), "
                    f"success={result.success}, "
                    f"location_id='{target.location_id}', "
                    f"db_version={target.db_version}, "
                    f"message='{result.message}'"
                ),
            )

        return succeeded

    def feedback_callback(self, feedback_msg):
        fb = feedback_msg.feedback

        if self._resolved_target is not None:
            target_context = (
                f"location_id='{self._resolved_target.location_id}', "
                f"db_version={self._resolved_target.db_version}, "
            )
        else:
            target_context = ''

        self._log_stage_info(
            'EXECUTION',
            (
                f"Feedback: "
                f"{target_context}"
                f"distance_remaining={fb.distance_remaining:.3f}, "
                f"recoveries={fb.number_of_recoveries}, "
                f"current_x={fb.current_pose.pose.position.x:.3f}, "
                f"current_y={fb.current_pose.pose.position.y:.3f}"
            ),
        )

    def cancel_goal(self):
        if self._goal_handle is None:
            return

        self._log_stage_info(
            'EXECUTION',
            'Cancel request received. Cancelling goal...',
        )

        cancel_future = self._goal_handle.cancel_goal_async()
        completed = self._wait_for_future(cancel_future, timeout_sec=5.0)

        if not completed:
            self._log_stage_error(
                'EXECUTION',
                'Cancel request did not complete within timeout.',
            )
            return

        if cancel_future.exception() is not None:
            self._log_stage_error(
                'EXECUTION',
                f'Cancel request failed: {cancel_future.exception()}',
            )

def extract_query_from_argv() -> Optional[str]:
    """
    Positional CLI remains direct semantic query only.

    Supported:
      ros2 run semantic_nav_orchestrator navigation_orchestrator kitchen
      ros2 run semantic_nav_orchestrator navigation_orchestrator living room

    Natural language should use:
      --ros-args -p command:='I am hungry'
    """
    argv = sys.argv[1:]

    positional = [
        arg
        for arg in argv
        if not arg.startswith('-') and ':=' not in arg
    ]

    if positional:
        return ' '.join(positional).strip()

    return None

def main(args=None):
    rclpy.init(args=args)
    node = NavigationOrchestrator()

    cli_query = extract_query_from_argv()
    if cli_query:
        node.get_logger().info(
            f'Overriding query parameter with command line argument: "{cli_query}"'
        )
        node.set_parameters([
            Parameter(
                'query',
                Parameter.Type.STRING,
                cli_query,
            )
        ])
        node._query = cli_query.strip()

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
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
