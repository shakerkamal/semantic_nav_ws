import os
import math
import sys
import json
import uuid
from datetime import datetime, timezone
from dataclasses import dataclass, asdict
from typing import Optional

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from tf2_ros import TransformException, Buffer, TransformListener

from semantic_nav_interfaces.action import ExecutePose
from semantic_nav_interfaces.srv import ResolveLocation, ValidatePose, ParseSemanticCommand, ProposeRecovery


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

@dataclass
class PipelineOutcome:
    success: bool
    stage: str
    message: str
    target: Optional[ResolvedTarget] = None

@dataclass
class AttemptRecord:
    action: str
    value: str
    outcome: str
    rationale: str
    failure_stage: str
    message: str

@dataclass
class RecoveryProposal:
    success: bool
    action: str
    target: str
    waypoints: list
    rationale: str
    confidence_percent: int
    raw_output: str
    message: str

class NavigationOrchestrator(Node):
    def __init__(self):
        super().__init__('navigation_orchestrator')

        self.declare_parameter('query', '')

        self.declare_parameter("command", "")
        self.declare_parameter("parse_service", "/parse_semantic_command")

        self.declare_parameter('resolve_service', '/resolve_location')
        self.declare_parameter('validate_service', '/validate_pose_goal')
        self.declare_parameter('execute_action', '/execute_pose')

        default_semantic_db_path = os.path.join(
            get_package_share_directory("semantic_nav_semantics"),
            "config",
            "semantic_db.json",
        )

        self.declare_parameter("semantic_db_path", default_semantic_db_path)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("nearest_location_count", 5)

        # Recovery parameters
        self.declare_parameter("recovery_cap", 3)
        self.declare_parameter("propose_recovery_service", "/propose_recovery")
        self.declare_parameter("recovery_log_path", "")
        self.declare_parameter("require_recovery_approval", False)
        self.declare_parameter("allow_stdin_intervention", True)

        self.declare_parameter('planner_id', '')
        self.declare_parameter('behavior_tree', '')
        self.declare_parameter('enable_validation', True)

        self.declare_parameter('service_wait_timeout_sec', 30.0)
        self.declare_parameter('service_call_timeout_sec', 240.0)
        self.declare_parameter('action_server_wait_timeout_sec', 10.0)
        self.declare_parameter('action_send_goal_timeout_sec', 10.0)


        # Set <= 0.0 for no execution timeout.
        self.declare_parameter('execution_timeout_sec', 300.0)

        self._query = self.get_parameter('query').get_parameter_value().string_value.strip()
        self._command = self.get_parameter('command').get_parameter_value().string_value.strip()
        self._parse_service_name = self.get_parameter('parse_service').get_parameter_value().string_value
        self._propose_recovery_service_name =  self.get_parameter("propose_recovery_service").get_parameter_value().string_value
        self._resolve_service_name = self.get_parameter('resolve_service').get_parameter_value().string_value
        self._validate_service_name = self.get_parameter('validate_service').get_parameter_value().string_value
        self._execute_action_name = self.get_parameter('execute_action').get_parameter_value().string_value

        self._semantic_db_path = self.get_parameter('semantic_db_path').get_parameter_value().string_value.strip()
        self._global_frame = self.get_parameter('global_frame').get_parameter_value().string_value.strip()
        self._robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value.strip()
        self._nearest_location_count = self.get_parameter('nearest_location_count').get_parameter_value().integer_value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._recovery_locations = self._load_recovery_locations(self._semantic_db_path)

        self._planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
        self._behavior_tree = self.get_parameter('behavior_tree').get_parameter_value().string_value
        self._enable_validation = self.get_parameter('enable_validation').get_parameter_value().bool_value

        self._recovery_cap = self.get_parameter('recovery_cap').get_parameter_value().integer_value
        self._recovery_log_path = self.get_parameter("recovery_log_path").get_parameter_value().string_value.strip()
        self._require_recovery_approval = self.get_parameter("require_recovery_approval").get_parameter_value().bool_value
        self._allow_stdin_intervention = self.get_parameter("allow_stdin_intervention").get_parameter_value().bool_value
        self._service_wait_timeout_sec = self.get_parameter('service_wait_timeout_sec').get_parameter_value().double_value
        self._service_call_timeout_sec = self.get_parameter('service_call_timeout_sec').get_parameter_value().double_value
        self._action_server_wait_timeout_sec = self.get_parameter('action_server_wait_timeout_sec').get_parameter_value().double_value
        self._action_send_goal_timeout_sec = self.get_parameter('action_send_goal_timeout_sec').get_parameter_value().double_value
        self._execution_timeout_sec = self.get_parameter('execution_timeout_sec').get_parameter_value().double_value

        self._parse_command_client = self.create_client(ParseSemanticCommand, self._parse_service_name)
        self._propose_recovery_client = self.create_client(ProposeRecovery, self._propose_recovery_service_name)
        self._resolve_location_client = self.create_client(ResolveLocation, self._resolve_service_name)
        self._validate_pose_client = self.create_client(ValidatePose, self._validate_service_name)
        self._execute_pose_client = ActionClient(self, ExecutePose, self._execute_action_name)

        self._goal_handle = None
        self._result_future = None
        self._final_success = False

        self._resolved_target: Optional[ResolvedTarget] = None
        self._parsed_command: Optional[ParsedCommand] = None

        self._db_version: int = 0
        self._db_stamp: Optional[Time] = None

        self._session_id = str(uuid.uuid4())
        self._last_validation_message = ""
        self._last_execution_message = ""
        self._last_feedback_distance_remaining = 0.0
        self._last_feedback_recoveries = 0
        self._last_feedback_pose = None

        self.get_logger().info(
            "Navigation Orchestrator initialized: "
            f"query='{self._query}', "
            f"command='{self._command}', "
            f"recovery_cap={self._recovery_cap}, "
            f"propose_recovery_service='{self._propose_recovery_service_name}', "
            f"require_recovery_approval={self._require_recovery_approval}, "
            f"allow_stdin_intervention={self._allow_stdin_intervention}"
        )

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

        original_nl_command = self._command if self._command else ""

        return self._run_with_recovery(
            initial_query=semantic_query,
            original_nl_command=original_nl_command,
        )

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
    
    def _run_pipeline_once(self, query: str) -> PipelineOutcome:
        target = self._resolve_query(query)
        if target is None:
            return PipelineOutcome(
                success=False,
                stage="resolution",
                message="Failed to resolve query to a valid navigation target.",
                target=None,
            )

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
                return PipelineOutcome(
                    success=False,
                    stage="validation",
                    message=self._last_validation_message or "Pose validation failed.",
                    target=target,
                )

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
            return PipelineOutcome(
                success=False,
                stage="execution",
                message=self._last_execution_message or "ExecutePose failed.",
                target=target,
            )

        return PipelineOutcome(
            success=True,
            stage="done",
            message="Navigation succeeded.",
            target=target,
        )
    
    def _run_with_recovery(
        self,
        initial_query: str,
        original_nl_command: str = "",
    ) -> bool:
        attempts = []
        recovery_count = 0
        current_query = initial_query
        chain_queue = []
        original_target_id = None

        self._log_stage_info(
            "RECOVERY",
            (
                f"Recovery loop enabled: recovery_cap={self._recovery_cap}, "
                f"require_recovery_approval={self._require_recovery_approval}, "
                f"allow_stdin_intervention={self._allow_stdin_intervention}"
            ),
        )

        while True:
            outcome = self._run_pipeline_once(current_query)

            if outcome.target is not None and original_target_id is None:
                original_target_id = outcome.target.location_id

            if outcome.success:
                if chain_queue:
                    next_query = chain_queue.pop(0)
                    self._log_stage_info(
                        "RECOVERY",
                        (
                            f"Waypoint leg succeeded. Continuing chain with "
                            f"next target='{next_query}'. Remaining legs={chain_queue}"
                        ),
                    )
                    current_query = next_query
                    continue

                return True

            if outcome.stage == "resolution":
                return self._escalate_intervention(
                    reason="resolution_failed",
                    original_nl_command=original_nl_command,
                    original_target=original_target_id or current_query,
                    attempts=attempts,
                    last_outcome=outcome,
                )

            if outcome.stage not in {"validation", "execution"}:
                return False

            failed_target_id = (
                outcome.target.location_id if outcome.target else current_query
            )

            stable_original_target = original_target_id or failed_target_id

            if recovery_count >= self._recovery_cap:
                return self._escalate_intervention(
                    reason="cap_reached",
                    original_nl_command=original_nl_command,
                    original_target=stable_original_target,
                    attempts=attempts,
                    last_outcome=outcome,
                )

            proposal = self._call_propose_recovery(
                original_nl_command=original_nl_command,
                original_target=stable_original_target,
                failure_stage=outcome.stage,
                nav2_message=outcome.message,
                attempts=attempts,
                target=outcome.target,
                remaining_retry_budget=self._recovery_cap - recovery_count,
            )

            recovery_count += 1

            if proposal is None:
                attempts.append(
                    AttemptRecord(
                        action="unusable_proposal",
                        value="",
                        outcome="proposal_call_failed",
                        rationale="",
                        failure_stage=outcome.stage,
                        message="ProposeRecovery service call failed.",
                    )
                )
                return self._escalate_intervention(
                    reason="unusable_proposal",
                    original_nl_command=original_nl_command,
                    original_target=stable_original_target,
                    attempts=attempts,
                    last_outcome=outcome,
                )

            self._write_recovery_log(
                original_nl_command=original_nl_command,
                original_target=stable_original_target,
                failure_stage=outcome.stage,
                nav2_message=outcome.message,
                attempts=attempts,
                proposal=proposal,
                outcome="proposal_received",
            )

            if not proposal.success:
                attempts.append(
                    AttemptRecord(
                        action=proposal.action or "unusable_proposal",
                        value=proposal.target or ",".join(proposal.waypoints),
                        outcome="proposal_rejected",
                        rationale=proposal.rationale,
                        failure_stage=outcome.stage,
                        message=proposal.message,
                    )
                )
                return self._escalate_intervention(
                    reason="unusable_proposal",
                    original_nl_command=original_nl_command,
                    original_target=stable_original_target,
                    attempts=attempts,
                    last_outcome=outcome,
                )

            if proposal.action == "give_up":
                attempts.append(
                    AttemptRecord(
                        action="give_up",
                        value="",
                        outcome="llm_give_up",
                        rationale=proposal.rationale,
                        failure_stage=outcome.stage,
                        message=proposal.message,
                    )
                )

                return self._escalate_intervention(
                    reason="give_up",
                    original_nl_command=original_nl_command,
                    original_target=stable_original_target,
                    attempts=attempts,
                    last_outcome=outcome,
                )

            if self._require_recovery_approval:
                if not self._approve_recovery_proposal(proposal):
                    attempts.append(
                        AttemptRecord(
                            action=proposal.action,
                            value=proposal.target or ",".join(proposal.waypoints),
                            outcome="operator_rejected_proposal",
                            rationale=proposal.rationale,
                            failure_stage=outcome.stage,
                            message=proposal.message,
                        )
                    )
                    return self._escalate_intervention(
                        reason="operator_rejected_proposal",
                        original_nl_command=original_nl_command,
                        original_target=stable_original_target,
                        attempts=attempts,
                        last_outcome=outcome,
                    )

            if proposal.action == "retry_target":
                attempts.append(
                    AttemptRecord(
                        action="retry_target",
                        value=proposal.target,
                        outcome="dispatching_retry_target",
                        rationale=proposal.rationale,
                        failure_stage=outcome.stage,
                        message=proposal.message,
                    )
                )
                chain_queue = []
                current_query = proposal.target
                continue

            if proposal.action == "via_waypoints":
                if not proposal.waypoints:
                    attempts.append(
                        AttemptRecord(
                            action="via_waypoints",
                            value="",
                            outcome="empty_waypoint_chain",
                            rationale=proposal.rationale,
                            failure_stage=outcome.stage,
                            message=proposal.message,
                        )
                    )
                    return self._escalate_intervention(
                        reason="empty_waypoint_chain",
                        original_nl_command=original_nl_command,
                        original_target=stable_original_target,
                        attempts=attempts,
                        last_outcome=outcome,
                    )

                attempts.append(
                    AttemptRecord(
                        action="via_waypoints",
                        value=",".join(proposal.waypoints),
                        outcome="dispatching_waypoint_chain",
                        rationale=proposal.rationale,
                        failure_stage=outcome.stage,
                        message=proposal.message,
                    )
                )

                current_query = proposal.waypoints[0]
                chain_queue = list(proposal.waypoints[1:])

                self._log_stage_info(
                    "RECOVERY",
                    (
                        f"Dispatching waypoint chain. "
                        f"current_target='{current_query}', "
                        f"remaining_chain={chain_queue}"
                    ),
                )

                continue

            attempts.append(
                AttemptRecord(
                    action=proposal.action,
                    value=proposal.target or ",".join(proposal.waypoints),
                    outcome="unknown_recovery_action",
                    rationale=proposal.rationale,
                    failure_stage=outcome.stage,
                    message=proposal.message,
                )
            )

            return self._escalate_intervention(
                reason="unknown_recovery_action",
                original_nl_command=original_nl_command,
                original_target=stable_original_target,
                attempts=attempts,
                last_outcome=outcome,
            )
        
    def _call_propose_recovery(
        self,
        original_nl_command: str,
        original_target: str,
        failure_stage: str,
        nav2_message: str,
        attempts: list,
        target: Optional[ResolvedTarget],
        remaining_retry_budget: int,
    ) -> Optional[RecoveryProposal]:
        self._log_stage_info(
            "RECOVERY",
            (
                f"Calling propose recovery: original_target='{original_target}', "
                f"failure_stage='{failure_stage}', "
                f"remaining_retry_budget={remaining_retry_budget}"
            ),
        )

        if not self._propose_recovery_client.wait_for_service(
            timeout_sec=self._service_wait_timeout_sec
        ):
            self._log_stage_error(
                "RECOVERY",
                (
                    f"Propose recovery service "
                    f"'{self._propose_recovery_service_name}' not available."
                ),
            )
            return None

        req = ProposeRecovery.Request()
        req.original_nl_command = original_nl_command
        req.original_target = original_target
        req.failure_stage = failure_stage
        req.nav2_message = nav2_message

        req.attempted_actions = [a.action for a in attempts]
        req.attempted_values = [a.value for a in attempts]
        req.attempt_outcomes = [a.outcome for a in attempts]
        req.attempt_rationales = [a.rationale for a in attempts]

        recovery_pose = self._make_recovery_pose(target)
        req.robot_pose_at_failure = recovery_pose
        req.nearest_locations_summary = self._build_nearest_locations_summary(
            robot_pose=recovery_pose,
            original_target=target,
        )

        if failure_stage == "execution":
            req.distance_remaining_at_abort = float(
                self._last_feedback_distance_remaining
            )
            req.nav2_recoveries_attempted = int(self._last_feedback_recoveries)
        else:
            req.distance_remaining_at_abort = 0.0
            req.nav2_recoveries_attempted = 0

        req.remaining_retry_budget = int(remaining_retry_budget)

        future = self._propose_recovery_client.call_async(req)

        if not self._wait_for_future(future, self._service_call_timeout_sec):
            self._log_stage_error(
                "RECOVERY",
                (
                    f"Service call to propose recovery timed out after "
                    f"{self._service_call_timeout_sec:.1f}s."
                ),
            )
            return None

        if future.exception() is not None:
            self._log_stage_error(
                "RECOVERY",
                f"Propose recovery service call failed: {future.exception()}",
            )
            return None

        response = future.result()
        if response is None:
            self._log_stage_error(
                "RECOVERY",
                "Propose recovery service returned no response.",
            )
            return None

        self._log_stage_info(
            "RECOVERY",
            (
                f"Proposal response: success={response.success}, "
                f"action='{response.action}', "
                f"target='{response.target}', "
                f"waypoints={list(response.waypoints)}, "
                f"confidence={response.confidence_percent}, "
                f"message='{response.message}'"
            ),
        )

        return RecoveryProposal(
            success=bool(response.success),
            action=response.action,
            target=response.target,
            waypoints=list(response.waypoints),
            rationale=response.rationale,
            confidence_percent=int(response.confidence_percent),
            raw_output=response.raw_output,
            message=response.message,
        )
    
    def _make_recovery_pose(self, target: Optional[ResolvedTarget]) -> PoseStamped:
        pose = self._lookup_robot_pose()

        if pose is not None:
            return pose

        if self._last_feedback_pose is not None:
            return self._last_feedback_pose

        if target is not None and target.pose is not None:
            # Fallback only. Not semantically ideal, but keeps recovery service populated.
            return target.pose

        pose = PoseStamped()
        pose.header.frame_id = self._global_frame or "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.orientation.w = 1.0
        return pose
    
    def _lookup_robot_pose(self) -> Optional[PoseStamped]:
        base_frames = [
            self._robot_base_frame,
            "base_link",
            "base_footprint"
        ]

        #Preserve order but remove duplicates while ensuring robot_base_frame is first if set
        seen = set()
        ordered_frames = []
        for frame in base_frames:
            if frame and frame not in seen:
                seen.add(frame)
                ordered_frames.append(frame)
        
        for base_frame in ordered_frames:
            try:
                transform = self._tf_buffer.lookup_transform(
                    self._global_frame,
                    base_frame,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.5),
                )
            except TransformException:
                continue
            except Exception:
                continue

            pose = PoseStamped()
            pose.header = transform.header
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation

            return pose
        
        self._log_stage_warn(
            "RECOVERY",
            (
                f"Could not look up robot pose using global_frame='{self._global_frame}' "
                f"and base frames={ordered_frames}."
            ),
        )

        return None
        
    
    def _load_recovery_locations(self, db_path: str):
        if not db_path:
            self._log_stage_warn(
                "RECOVERY",
                "semantic_db_path is empty. Nearest-location summaries disabled.",
            )
            return []

        if not os.path.exists(db_path):
            self._log_stage_warn(
                "RECOVERY",
                f"semantic_db_path does not exist: '{db_path}'. "
                "Nearest-location summaries disabled.",
            )
            return []

        try:
            with open(db_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            self._log_stage_warn(
                "RECOVERY",
                f"Failed to read semantic DB for recovery summaries: {exc}",
            )
            return []

        locations = data.get("locations", {})
        if not isinstance(locations, dict):
            self._log_stage_warn(
                "RECOVERY",
                "semantic DB has no valid 'locations' object. "
                "Nearest-location summaries disabled.",
            )
            return []

        parsed = []

        for location_id, record in locations.items():
            if not isinstance(record, dict):
                continue

            frame_id = str(record.get("frame_id", "map"))
            if frame_id != "map":
                continue

            try:
                x = float(record["x"])
                y = float(record["y"])
            except Exception:
                continue

            if not math.isfinite(x) or not math.isfinite(y):
                continue

            parsed.append({
                "id": str(location_id),
                "x": x,
                "y": y,
            })

        self._log_stage_info(
            "RECOVERY",
            f"Loaded {len(parsed)} semantic locations for nearest-location summaries.",
        )

        return parsed

    def _build_nearest_locations_summary(
        self,
        robot_pose: PoseStamped,
        original_target: Optional[ResolvedTarget],
    ) -> str:
        if robot_pose is None:
            return "robot pose unavailable"

        if not self._recovery_locations:
            return "semantic location catalog unavailable"

        rx = float(robot_pose.pose.position.x)
        ry = float(robot_pose.pose.position.y)

        ranked = []

        for loc in self._recovery_locations:
            dx = rx - float(loc["x"])
            dy = ry - float(loc["y"])
            d = math.sqrt(dx * dx + dy * dy)
            ranked.append((d, loc["id"]))

        ranked.sort(key=lambda item: item[0])

        limit = max(1, int(self._nearest_location_count))
        nearest = ranked[:limit]

        nearest_text = ", ".join(
            f"{name} ({distance:.2f} m)"
            for distance, name in nearest
        )

        suffix = ""

        if original_target is not None:
            tx = float(original_target.pose.pose.position.x)
            ty = float(original_target.pose.pose.position.y)
            d_target = math.sqrt((rx - tx) ** 2 + (ry - ty) ** 2)
            suffix = (
                f"; distance to original target "
                f"{original_target.location_id}: {d_target:.2f} m"
            )

        return f"nearest semantic locations: {nearest_text}{suffix}"

    def _approve_recovery_proposal(self, proposal: RecoveryProposal) -> bool:
        if not self._allow_stdin_intervention:
            self._log_stage_warn(
                "RECOVERY",
                "Recovery approval required but stdin intervention is disabled.",
            )
            return False

        print()
        # print("============================================================")
        print("  LLM RECOVERY PROPOSAL — approval required")
        # print("============================================================")
        print(f"action:     {proposal.action}")
        print(f"target:     {proposal.target}")
        print(f"waypoints:  {proposal.waypoints}")
        print(f"confidence: {proposal.confidence_percent}")
        print(f"rationale:  {proposal.rationale}")
        print("Approve this proposal? [y/N]")
        choice = input("> ").strip().lower()

        return choice in {"y", "yes"}

    def _escalate_intervention(
        self,
        reason: str,
        original_nl_command: str,
        original_target: str,
        attempts: list,
        last_outcome: Optional[PipelineOutcome],
    ) -> bool:
        self._log_stage_warn(
            "RECOVERY",
            f"Escalating to operator intervention: reason='{reason}'",
        )

        if not self._allow_stdin_intervention:
            self._log_stage_error(
                "RECOVERY",
                "stdin intervention disabled. Aborting orchestrator.",
            )
            return False

        print()
        print("============================================================")
        print("  NAVIGATION BLOCKED — operator input required")
        print("============================================================")
        print(f"reason: {reason}")
        print()
        print("Original goal:")
        print(f"  user command:  \"{original_nl_command or '(none)'}\"")
        print(f"  canonical id:  {original_target}")
        print()

        print(f"LLM recovery attempts ({len(attempts)}/{self._recovery_cap} used):")
        if not attempts:
            print("  (none)")
        else:
            for i, attempt in enumerate(attempts, start=1):
                print(f"  {i}. action:    {attempt.action}")
                print(f"     value:     {attempt.value}")
                print(f"     outcome:   {attempt.outcome}")
                print(f"     rationale: {attempt.rationale}")
                print(f"     message:   {attempt.message}")

        print()

        if last_outcome is not None:
            print("Last failure:")
            print(f"  stage:   {last_outcome.stage}")
            print(f"  message: {last_outcome.message}")

        print()
        print("Choose:")
        print("  [t] provide a new semantic target manually")
        print("  [a] abort orchestrator so you can teleop and re-run")
        print("  [g] give up entirely")

        while True:
            choice = input("> ").strip().lower()

            if choice == "t":
                new_target = input("New semantic target: ").strip()

                if not new_target:
                    print("Target cannot be empty.")
                    continue

                self._log_stage_info(
                    "RECOVERY",
                    f"Operator provided new semantic target: '{new_target}'",
                )

                return self._run_with_recovery(
                    initial_query=new_target,
                    original_nl_command="",
                )

            if choice == "a":
                self._log_stage_warn(
                    "RECOVERY",
                    "Operator aborted orchestrator for teleoperation.",
                )
                return False

            if choice == "g":
                self._log_stage_warn(
                    "RECOVERY",
                    "Operator selected give up.",
                )
                return False

            print("Invalid choice. Use [t], [a], or [g].")

    def _write_recovery_log(
        self,
        original_nl_command: str,
        original_target: str,
        failure_stage: str,
        nav2_message: str,
        attempts: list,
        proposal: RecoveryProposal,
        outcome: str,
    ):
        if not self._recovery_log_path:
            return

        record = {
            "session_id": self._session_id,
            "ts": datetime.now(timezone.utc).isoformat(),
            "original_nl_command": original_nl_command,
            "original_target": original_target,
            "failure_stage": failure_stage,
            "nav2_message": nav2_message,
            "attempts_so_far": [asdict(a) for a in attempts],
            "nearest_locations": [],
            "remaining_retry_budget": None,
            "raw_llm_output": proposal.raw_output,
            "llm_confidence": proposal.confidence_percent,
            "llm_rationale": proposal.rationale,
            "decision": proposal.action,
            "decision_payload": {
                "target": proposal.target,
                "waypoints": proposal.waypoints,
            },
            "outcome": outcome,
        }

        try:
            with open(self._recovery_log_path, "a", encoding="utf-8") as f:
                f.write(json.dumps(record) + "\n")
        except Exception as exc:
            self._log_stage_error(
                "RECOVERY",
                f"Failed to write recovery log '{self._recovery_log_path}': {exc}",
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
        self._last_validation_message = ""

        if target is None or target.pose is None:
            self._last_validation_message = "No resolved target provided for validation."
            self._log_stage_error(
                'VALIDATION',
                self._last_validation_message,
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
            self._last_validation_message = f'Validate pose service "{self._validate_service_name}" not available.'
            self._log_stage_error(
                'VALIDATION',
                self._last_validation_message,
            )
            return False

        req = ValidatePose.Request()
        req.goal = target.pose
        req.planner_id = self._planner_id
        req.use_start = False

        future = self._validate_pose_client.call_async(req)

        if not self._wait_for_future(future, self._service_call_timeout_sec):
            self._last_validation_message = (f'Service call to validate pose timed out after '
                                             f'{self._service_call_timeout_sec:.1f}s.')
            self._log_stage_error(
                'VALIDATION',
                self._last_validation_message,
            )
            return False

        if future.exception() is not None:
            self._last_validation_message = f'Failed to call validate pose service: {future.exception()}'
            self._log_stage_error(
                'VALIDATION',
                self._last_validation_message,
            )
            return False

        response = future.result()
        if response is None:
            self._last_validation_message = 'Validate pose service returned no response.'
            self._log_stage_error(
                'VALIDATION',
                self._last_validation_message,
            )
            return False
        
        self._last_validation_message = response.message

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
        self._last_execution_message = ""
        self._last_feedback_distance_remaining = 0.0
        self._last_feedback_recoveries = 0
        self._last_feedback_pose = None

        if target is None or target.pose is None:
            self._last_execution_message = "No resolved target provided for execution."
            self._log_stage_error(
                "EXECUTION",
                self._last_execution_message,
            )
            return False

        pose = target.pose

        if not self._execute_pose_client.wait_for_server(
            timeout_sec=self._action_server_wait_timeout_sec
        ):
            self._last_execution_message = (
                f'Execute pose action server "{self._execute_action_name}" not available.'
            )
            self._log_stage_error(
                "EXECUTION",
                self._last_execution_message,
            )
            return False

        goal_msg = ExecutePose.Goal()
        goal_msg.pose = pose
        goal_msg.behavior_tree = self._behavior_tree

        self._log_stage_info(
            "EXECUTION",
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

        if not self._wait_for_future(
            send_goal_future,
            self._action_send_goal_timeout_sec,
        ):
            self._last_execution_message = (
                f"Send goal to execute_pose action server timed out after "
                f"{self._action_send_goal_timeout_sec:.1f}s."
            )
            self._log_stage_error(
                "EXECUTION",
                self._last_execution_message,
            )
            return False

        if send_goal_future.exception() is not None:
            self._last_execution_message = (
                f"Failed to send goal to execute_pose action server: "
                f"{send_goal_future.exception()}"
            )
            self._log_stage_error(
                "EXECUTION",
                self._last_execution_message,
            )
            return False

        self._goal_handle = send_goal_future.result()

        if self._goal_handle is None:
            self._last_execution_message = "Failed to get goal handle from executor."
            self._log_stage_error(
                "EXECUTION",
                self._last_execution_message,
            )
            return False

        if not self._goal_handle.accepted:
            self._last_execution_message = (
                f"Goal rejected by action server "
                f"(location_id='{target.location_id}', db_version={target.db_version})."
            )
            self._log_stage_error(
                "EXECUTION",
                self._last_execution_message,
            )
            return False

        self._log_stage_info(
            "EXECUTION",
            (
                f"Goal accepted, waiting for result "
                f"(location_id='{target.location_id}', "
                f"db_version={target.db_version})."
            ),
        )

        self._result_future = self._goal_handle.get_result_async()

        if not self._wait_for_future(
            self._result_future,
            self._execution_timeout_sec,
        ):
            self._last_execution_message = (
                f"ExecutePose result timed out after "
                f"{self._execution_timeout_sec:.1f}s."
            )
            self._log_stage_error(
                "EXECUTION",
                self._last_execution_message + " Cancelling goal.",
            )
            self.cancel_goal()
            return False

        if self._result_future.exception() is not None:
            self._last_execution_message = (
                f"Failed to get result from execute_pose action: "
                f"{self._result_future.exception()}"
            )
            self._log_stage_error(
                "EXECUTION",
                self._last_execution_message,
            )
            return False

        result_wrap = self._result_future.result()
        if result_wrap is None:
            self._last_execution_message = "ExecutePose action returned no result wrapper."
            self._log_stage_error(
                "EXECUTION",
                self._last_execution_message,
            )
            return False

        result = result_wrap.result
        status = result_wrap.status
        status_name = self._goal_status_to_string(status)

        self._last_execution_message = result.message

        self._log_stage_info(
            "EXECUTION",
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
                "EXECUTION",
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

        self._last_feedback_pose = fb.current_pose
        self._last_feedback_distance_remaining = float(fb.distance_remaining)
        self._last_feedback_recoveries = int(fb.number_of_recoveries)

        if self._resolved_target is not None:
            target_context = (
                f"location_id='{self._resolved_target.location_id}', "
                f"db_version={self._resolved_target.db_version}, "
            )
        else:
            target_context = ""

        self._log_stage_info(
            "EXECUTION",
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
            "EXECUTION",
            "Cancel request received. Cancelling goal...",
        )

        cancel_future = self._goal_handle.cancel_goal_async()
        completed = self._wait_for_future(cancel_future, timeout_sec=5.0)

        if not completed:
            self._log_stage_error(
                "EXECUTION",
                "Cancel request did not complete within timeout.",
            )
            return

        if cancel_future.exception() is not None:
            self._log_stage_error(
                "EXECUTION",
                f"Cancel request failed: {cancel_future.exception()}",
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
