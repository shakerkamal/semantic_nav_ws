import os
import math
import re
import sys
import json
import uuid
import time
import threading
from datetime import datetime, timezone
from dataclasses import dataclass, asdict, field
from typing import Optional, List, Tuple
from enum import Enum

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped, Vector3
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from tf2_ros import TransformException, Buffer, TransformListener
from nav2_msgs.srv import ClearEntireCostmap

from semantic_nav_interfaces.action import ExecutePose
from semantic_nav_interfaces.srv import ResolveLocation, ValidatePose, ParseSemanticCommand, ProposeRecovery, RequestRecovery
from semantic_nav_interfaces.msg import RecoveryTrigger


_OBJECT_KEY_RE = re.compile(r"[a-z][a-z0-9 _]*:\d+")


def _looks_like_object_key(s: str) -> bool:
    return bool(_OBJECT_KEY_RE.fullmatch((s or "").strip().lower()))


@dataclass(frozen=True)
class ResolvedTarget:
    query: str
    location_id: str
    pose: PoseStamped
    db_version: int
    db_stamp: Time

@dataclass(frozen=True)
class ObjectActionAttributes:
    openable: bool
    clearable: bool
    safety_class: str

@dataclass(frozen=True)
class SemanticObject:
    key: str
    object_id: int
    tag: str
    caption: str
    state: str
    x: float
    y: float
    z: float
    extent_x: float
    extent_y: float
    extent_z: float
    volume: float
    openable: bool
    clearable: bool
    safety_class: str

@dataclass(frozen=True)
class ResponsibleObjectMatch:
    match_type: str
    object: Optional[SemanticObject]
    distance_m: float
    summary: str

@dataclass(frozen=True)
class ObjectRecoveryContext:
    summary: str
    policy: str
    primary_tag: str
    primary_state: str
    primary_distance: float

@dataclass(frozen=True)
class ParsedCommand:
    original_command: str
    intent: str
    location_query: str
    canonical_location_id: str
    # New object-centric fields (OC-M7)
    object_tag: str
    intent_hint: str
    target_object_key: str
    confidence_percent: int
    raw_output: str

@dataclass
class PipelineOutcome:
    success: bool
    stage: str
    message: str
    target: Optional[ResolvedTarget] = None

class RecoveryFSMState(str, Enum):
    IDLE = "IDLE"
    EXECUTING = "EXECUTING"
    DETERMINISTIC_WAIT = "DETERMINISTIC_WAIT"
    RECOVERY_IN_PROGRESS = "RECOVERY_IN_PROGRESS"
    LLM_WAIT = "LLM_WAIT"
    AWAITING_OPERATOR = "AWAITING_OPERATOR"
    OPERATOR_RECHECK = "OPERATOR_RECHECK"
    ESCALATE_OPERATOR = "ESCALATE_OPERATOR"
    TERMINAL_SUCCESS = "TERMINAL_SUCCESS"
    TERMINAL_FAIL = "TERMINAL_FAIL"

@dataclass
class TriggerInfo:
    trigger_source: str
    failure_stage: str
    nav2_message: str = ""

    robot_pose: Optional[PoseStamped] = None

    responsible_object_key: str = ""
    responsible_object_tag: str = ""
    responsible_object_state: str = ""
    responsible_bbox_center: Point = field(default_factory=Point)
    responsible_bbox_extent: Vector3 = field(default_factory=Vector3)
    responsible_safety_class: str = "none"
    responsible_openable: bool = False
    responsible_clearable: bool = False
    match_type: str = "unknown"

    blockage_centroid: Point = field(default_factory=Point)
    blockage_extent_m: float = 0.0

    blocked_plan_index_lo: int = 0
    blocked_plan_index_hi: int = 0

    debounce_key: str = ""
    stamp_sec: float = 0.0

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
    responsible_object_key: str = ""
    operator_message: str = ""
    wait_seconds: int = 0
    target_object_tag: str = ""
    target_intent_hint: str = ""

class NavigationOrchestrator(Node):
    def __init__(self):
        super().__init__('navigation_orchestrator')

        self.declare_parameter('query', '')

        self.declare_parameter("command", "")
        self.declare_parameter("parse_service", "/parse_semantic_command")

        self.declare_parameter('resolve_service', '/resolve_location')
        self.declare_parameter('validate_service', '/validate_pose_goal')
        self.declare_parameter('execute_action', '/execute_pose')

        default_semantic_map_path = os.path.join(
            get_package_share_directory("semantic_nav_semantics"),
            "config",
            "map_v001.json",
        )
        default_legacy_semantic_db_path = os.path.join(
            get_package_share_directory("semantic_nav_semantics"),
            "config",
            "semantic_db.json",
        )

        # map_v001.json is the primary semantic source going forward.
        # semantic_db.json is kept only as a temporary fallback for older room-level catalogs.
        self.declare_parameter("semantic_map_path", default_semantic_map_path)
        self.declare_parameter("semantic_db_path", default_legacy_semantic_db_path)
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

        # BT parameters for recovery triggering and logging
        self.declare_parameter("recovery_status_topic", "/recovery_status")
        self.declare_parameter("request_recovery_service", "/request_recovery")
        self.declare_parameter("recovery_trigger_topic", "/recovery_trigger")
        self.declare_parameter("enable_bt_recovery_trigger", True)
        self.declare_parameter("enable_plan_intersection_trigger", True)
        self.declare_parameter("enable_stall_watchdog", True)
        self.declare_parameter("stall_distance_epsilon_m", 0.05)
        self.declare_parameter("stall_window_sec", 4.0)
        self.declare_parameter("stall_nav2_recoveries_cap", 2)
        self.declare_parameter("responsible_object_debounce_sec", 2.0)
        self.declare_parameter("unknown_blockage_debounce_sec", 1.0)
        self.declare_parameter("bbox_inflation_m", 0.20)
        self.declare_parameter("nearest_fallback_radius_m", 0.90)
        self.declare_parameter("start_idle", False)

        default_semantic_object_db_path = default_semantic_map_path
        default_object_action_attributes_path = os.path.join(
            get_package_share_directory("semantic_nav_semantics"),
            "config",
            "object_action_attributes.json",
        )
        self.declare_parameter("semantic_object_db_path", default_semantic_object_db_path)
        self.declare_parameter("object_action_attributes_path", default_object_action_attributes_path)

        self._query = self.get_parameter('query').get_parameter_value().string_value.strip()
        self._command = self.get_parameter('command').get_parameter_value().string_value.strip()
        self._parse_service_name = self.get_parameter('parse_service').get_parameter_value().string_value
        self._propose_recovery_service_name =  self.get_parameter("propose_recovery_service").get_parameter_value().string_value
        self._resolve_service_name = self.get_parameter('resolve_service').get_parameter_value().string_value
        self._validate_service_name = self.get_parameter('validate_service').get_parameter_value().string_value
        self._execute_action_name = self.get_parameter('execute_action').get_parameter_value().string_value

        self._semantic_map_path = self.get_parameter("semantic_map_path").get_parameter_value().string_value.strip()
        self._semantic_db_path = self.get_parameter('semantic_db_path').get_parameter_value().string_value.strip()
        self._global_frame = self.get_parameter('global_frame').get_parameter_value().string_value.strip()
        self._robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value.strip()
        self._nearest_location_count = self.get_parameter('nearest_location_count').get_parameter_value().integer_value
        self._semantic_object_db_path = self.get_parameter("semantic_object_db_path").get_parameter_value().string_value.strip()
        self._object_action_attributes_path = self.get_parameter("object_action_attributes_path").get_parameter_value().string_value.strip()
        self._bbox_inflation_m = self.get_parameter("bbox_inflation_m").get_parameter_value().double_value
        self._nearest_fallback_radius_m = self.get_parameter("nearest_fallback_radius_m").get_parameter_value().double_value
        self._start_idle = self.get_parameter("start_idle").get_parameter_value().bool_value
        self._recovery_trigger_topic = self.get_parameter("recovery_trigger_topic").get_parameter_value().string_value
        self._enable_plan_intersection_trigger = self.get_parameter("enable_plan_intersection_trigger").get_parameter_value().bool_value
        self._enable_stall_watchdog = self.get_parameter("enable_stall_watchdog").get_parameter_value().bool_value
        self._stall_distance_epsilon_m = self.get_parameter("stall_distance_epsilon_m").get_parameter_value().double_value
        self._stall_window_sec = self.get_parameter("stall_window_sec").get_parameter_value().double_value
        self._stall_nav2_recoveries_cap = self.get_parameter("stall_nav2_recoveries_cap").get_parameter_value().integer_value

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._log_stage_info(
            "RECOVERY",
            (
                f"Using semantic_map_path='{self._semantic_map_path}' as the primary semantic map; "
                f"legacy semantic_db_path='{self._semantic_db_path}' remains a fallback."
            ),
        )
        self._recovery_locations = self._load_recovery_locations_from_sources(
            [self._semantic_map_path, self._semantic_db_path]
        )
        self._object_action_defaults, self._object_action_by_tag = self._load_object_action_attributes(
            self._object_action_attributes_path,
        )
        self._semantic_objects = self._load_semantic_objects(self._semantic_object_db_path)

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

        self._fsm_state = RecoveryFSMState.IDLE
        self._active_recovery = False
        self._last_trigger: Optional[TriggerInfo] = None
        self._last_trigger_by_key = {}

        self._attempt_records: List[AttemptRecord] = []

        self._parse_command_client = self.create_client(ParseSemanticCommand, self._parse_service_name)
        self._propose_recovery_client = self.create_client(ProposeRecovery, self._propose_recovery_service_name)
        self._resolve_location_client = self.create_client(ResolveLocation, self._resolve_service_name)
        self._validate_pose_client = self.create_client(ValidatePose, self._validate_service_name)
        self._execute_pose_client = ActionClient(self, ExecutePose, self._execute_action_name)

        self._recovery_status_pub = self.create_publisher(
            String,
            self.get_parameter("recovery_status_topic").get_parameter_value().string_value,
            10,
        )
        self._publish_recovery_status("RECOVERY_IDLE")

        if self.get_parameter("enable_bt_recovery_trigger").value:
            self._request_recovery_srv = self.create_service(
                RequestRecovery,
                self.get_parameter("request_recovery_service").get_parameter_value().string_value,
                self._handle_request_recovery,
            )

        self._recovery_trigger_sub = None
        if self._enable_plan_intersection_trigger:
            self._recovery_trigger_sub = self.create_subscription(
                RecoveryTrigger,
                self._recovery_trigger_topic,
                self._handle_recovery_trigger_msg,
                10,
            )
            self._log_stage_info(
                "RECOVERY/MONITOR",
                f"Subscribed to recovery trigger topic '{self._recovery_trigger_topic}'.",
            )

        self._goal_handle = None
        self._result_future = None
        self._navigation_goal_active = False
        self._stall_watchdog_triggered = False
        self._stall_baseline_distance_remaining: Optional[float] = None
        self._stall_baseline_stamp_sec: Optional[float] = None
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
            f"allow_stdin_intervention={self._allow_stdin_intervention}, "
            f"recovery_trigger_topic='{self._recovery_trigger_topic}', "
            f"enable_plan_intersection_trigger={self._enable_plan_intersection_trigger}, "
            f"enable_stall_watchdog={self._enable_stall_watchdog}"
        )

    def _log_stage_info(self, stage: str, message: str):
        self.get_logger().info(f'[{stage}] {message}')

    def _log_stage_warn(self, stage: str, message: str):
        self.get_logger().warn(f'[{stage}] {message}')

    def _log_stage_error(self, stage: str, message: str):
        self.get_logger().error(f'[{stage}] {message}')      

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        # The node is spun by MultiThreadedExecutor in main(). Poll here instead
        # of calling spin_until_future_complete(), which can block service callbacks
        # or conflict with the executor that already owns this node.
        if timeout_sec is None or timeout_sec <= 0.0:
            while rclpy.ok() and not future.done():
                time.sleep(0.01)
            return future.done()

        deadline = time.monotonic() + float(timeout_sec)
        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.01)

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
        if self._start_idle:
            self._log_stage_info(
                "RECOVERY",
                "start_idle=true. Orchestrator initialized without starting navigation.",
            )
            self._transition_recovery_fsm(
                RecoveryFSMState.IDLE,
                reason="start_idle",
            )
            return True

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

            if _looks_like_object_key(self._query):
                self._log_stage_info(
                    "INTENT",
                    f"[LLM_INTENT] Skipped: CLI query '{self._query}' looks like an object key.",
                )
            else:
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

        if parsed.intent == "navigate_to_object":
            semantic_query = parsed.object_tag
        else:
            semantic_query = parsed.canonical_location_id or parsed.location_query

        self._log_stage_info(
            "INTENT",
            (
                f"Natural-language command parsed: "
                f"command='{parsed.original_command}', "
                f"intent='{parsed.intent}', "
                f"location_query='{parsed.location_query}', "
                f"canonical_location_id='{parsed.canonical_location_id}', "
                f"object_tag='{parsed.object_tag}', "
                f"intent_hint='{parsed.intent_hint}', "
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

        valid_nav_intents = {"navigate_to_location", "navigate_to_object"}
        if response.intent not in valid_nav_intents:
            self._log_stage_error(
                "INTENT",
                (
                    f"Parsed command is not executable navigation: "
                    f"intent='{response.intent}', message='{response.message}'"
                ),
            )
            return None

        if not getattr(response, "target_known", response.location_known):
            self._log_stage_error(
                "INTENT",
                (
                    f"Parsed target is not known: "
                    f"intent='{response.intent}', "
                    f"object_tag='{getattr(response, 'object_tag', '')}', "
                    f"location_query='{response.location_query}', "
                    f"message='{response.message}'"
                ),
            )
            return None

        # For navigate_to_location, require a usable query.
        if (response.intent == "navigate_to_location"
                and not response.location_query
                and not response.canonical_location_id):
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
            object_tag=getattr(response, "object_tag", "") or "",
            intent_hint=getattr(response, "intent_hint", "") or "",
            target_object_key=getattr(response, "target_object_key", "") or "",
            confidence_percent=int(response.confidence_percent),
            raw_output=response.raw_output,
        )
    
    def _run_pipeline_once(self, query: str) -> PipelineOutcome:
        recovery_ctx = getattr(self, "_recovery_resolve_context", {})
        self._recovery_resolve_context = {}
        target = self._resolve_query(query, recovery_context=recovery_ctx)
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
        self._attempt_records = attempts

        recovery_count = 0
        current_query = initial_query
        chain_queue = []
        original_target_id = None

        self._active_recovery = False
        self._last_trigger = None
        self._transition_recovery_fsm(
            RecoveryFSMState.EXECUTING,
            reason="starting_navigation_pipeline",
        )

        self._log_stage_info(
            "RECOVERY",
            (
                f"Recovery loop enabled: recovery_cap={self._recovery_cap}, "
                f"require_recovery_approval={self._require_recovery_approval}, "
                f"allow_stdin_intervention={self._allow_stdin_intervention}, "
                f"recovery_trigger_topic='{self._recovery_trigger_topic}', "
                f"enable_plan_intersection_trigger={self._enable_plan_intersection_trigger}, "
                f"enable_stall_watchdog={self._enable_stall_watchdog}"
            ),
        )

        while True:
            outcome = self._run_pipeline_once(current_query)

            if outcome.target is not None and original_target_id is None:
                original_target_id = outcome.target.location_id

            if outcome.success:
                self._active_recovery = False

                if chain_queue:
                    next_query = chain_queue.pop(0)
                    self._transition_recovery_fsm(
                        RecoveryFSMState.EXECUTING,
                        reason="continuing_waypoint_chain",
                    )
                    self._log_stage_info(
                        "RECOVERY",
                        (
                            f"Waypoint leg succeeded. Continuing chain with "
                            f"next target='{next_query}'. Remaining legs={chain_queue}"
                        ),
                    )
                    current_query = next_query
                    continue

                self._transition_recovery_fsm(
                    RecoveryFSMState.TERMINAL_SUCCESS,
                    reason="goal_reached",
                )
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
                self._active_recovery = False
                self._transition_recovery_fsm(
                    RecoveryFSMState.TERMINAL_FAIL,
                    reason=f"unsupported_failure_stage:{outcome.stage}",
                )
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

            trigger_status = self._action_backstop_trigger(
                failure_stage=outcome.stage,
                nav2_message=outcome.message,
                robot_pose=self._make_recovery_pose(outcome.target),
                distance_remaining=self._last_feedback_distance_remaining,
                nav2_recoveries=self._last_feedback_recoveries,
                failed_target_id=failed_target_id,
                recovery_count=recovery_count,
            )

            if trigger_status not in {"accepted", "already_in_recovery"}:
                return self._escalate_intervention(
                    reason=f"recovery_trigger_{trigger_status}",
                    original_nl_command=original_nl_command,
                    original_target=stable_original_target,
                    attempts=attempts,
                    last_outcome=outcome,
                )

            self._transition_recovery_fsm(
                RecoveryFSMState.LLM_WAIT,
                reason="calling_propose_recovery",
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

            self._transition_recovery_fsm(
                RecoveryFSMState.RECOVERY_IN_PROGRESS,
                reason="proposal_received",
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
                self._transition_recovery_fsm(
                    RecoveryFSMState.AWAITING_OPERATOR,
                    reason="proposal_approval_required",
                )

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

                self._transition_recovery_fsm(
                    RecoveryFSMState.RECOVERY_IN_PROGRESS,
                    reason="proposal_approved",
                )

            if proposal.action == "retry_target":
                attempts.append(
                    AttemptRecord(
                        action="retry_target",
                        value=proposal.target_object_tag,
                        outcome="dispatching_retry_target",
                        rationale=proposal.rationale,
                        failure_stage=outcome.stage,
                        message=proposal.message,
                    )
                )
                self._active_recovery = False
                self._transition_recovery_fsm(
                    RecoveryFSMState.EXECUTING,
                    reason="dispatching_retry_target",
                )
                chain_queue = []
                current_query = proposal.target_object_tag
                # Store object-centric context consumed by the next _resolve_query call.
                self._recovery_resolve_context = {
                    "object_tag": proposal.target_object_tag,
                    "intent_hint": proposal.target_intent_hint,
                }
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

                self._active_recovery = False
                self._transition_recovery_fsm(
                    RecoveryFSMState.EXECUTING,
                    reason="dispatching_waypoint_chain",
                )

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

        parsed = getattr(self, "_parsed_command", None)
        req.original_object_tag = getattr(parsed, "object_tag", "") or "" if parsed else ""
        req.original_intent_hint = getattr(parsed, "intent_hint", "") or "" if parsed else ""
        req.current_target_object_key = target.location_id if target else ""

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

        self._populate_bt_recovery_request_defaults(req, target)

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
                f"target_object_tag='{getattr(response, 'target_object_tag', '')}', "
                f"target_intent_hint='{getattr(response, 'target_intent_hint', '')}', "
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
            responsible_object_key=getattr(response, "responsible_object_key", ""),
            operator_message=getattr(response, "operator_message", ""),
            wait_seconds=int(getattr(response, "wait_seconds", 0)),
            target_object_tag=getattr(response, "target_object_tag", "") or "",
            target_intent_hint=getattr(response, "target_intent_hint", "") or "",
        )

    def _populate_bt_recovery_request_defaults(
        self,
        req: ProposeRecovery.Request,
        target: Optional[ResolvedTarget],
    ) -> None:
        trigger = self._last_trigger

        req.trigger_source = (
            trigger.trigger_source
            if trigger is not None and trigger.trigger_source
            else "action_backstop"
        )

        req.responsible_object_key = (
            trigger.responsible_object_key
            if trigger is not None
            else ""
        )
        req.match_type = (
            trigger.match_type
            if trigger is not None and trigger.match_type
            else "unknown"
        )

        req.responsible_object_tag = (
            trigger.responsible_object_tag
            if trigger is not None
            else ""
        )
        req.responsible_object_state = (
            trigger.responsible_object_state
            if trigger is not None
            else ""
        )

        if trigger is not None:
            req.responsible_bbox_center = trigger.responsible_bbox_center
            req.responsible_bbox_extent = trigger.responsible_bbox_extent
            req.responsible_safety_class = trigger.responsible_safety_class or "none"
            req.responsible_openable = bool(trigger.responsible_openable)
            req.responsible_clearable = bool(trigger.responsible_clearable)
            req.blockage_centroid = trigger.blockage_centroid
            req.blockage_extent_m = float(trigger.blockage_extent_m)
        else:
            req.blockage_centroid = Point()
            req.blockage_extent_m = 0.0

        req.deterministic_waits_used = 0
        req.deterministic_wait_cap = 0
        req.total_seconds_blocked = 0.0

        if target is not None:
            req.db_version = int(target.db_version)
            req.db_stamp = target.db_stamp
        else:
            req.db_version = int(self._db_version)
            if self._db_stamp is not None:
                req.db_stamp = self._db_stamp

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
        
    
    def _load_recovery_locations_from_sources(self, db_paths: List[str]):
        for db_path in db_paths:
            locations = self._load_recovery_locations(db_path)
            if locations:
                self._log_stage_info(
                    "RECOVERY",
                    (
                        f"Using '{db_path}' for nearest-location recovery summaries "
                        f"({len(locations)} locations)."
                    ),
                )
                return locations

        self._log_stage_warn(
            "RECOVERY",
            "No usable semantic location catalog found in semantic_map_path or legacy semantic_db_path.",
        )
        return []

    def _iter_recovery_location_records(self, data):
        if not isinstance(data, dict):
            return

        # map_v001.json is expected to become the single semantic source, but during
        # the transition we accept several common room/location containers. The old
        # semantic_db.json format is still covered by the 'locations' case.
        candidate_keys = [
            "locations",
            "rooms",
            "places",
            "semantic_locations",
            "waypoints",
            "nodes",
        ]

        for container_key in candidate_keys:
            container = data.get(container_key)

            if isinstance(container, dict):
                for key, record in container.items():
                    yield key, record
                return

            if isinstance(container, list):
                for index, record in enumerate(container):
                    yield f"{container_key}_{index}", record
                return

        # Conservative fallback for transitional map files with top-level room-like
        # records. Deliberately skip object records so object bbox centers do not
        # pollute nearest-room summaries.
        for key, record in data.items():
            if not isinstance(record, dict):
                continue
            if str(key).startswith("object_"):
                continue
            if "object_state" in record or "object-state" in record or "object_tag" in record:
                continue
            yield key, record

    @staticmethod
    def _extract_xy_from_mapping(value):
        if not isinstance(value, dict):
            return None

        try:
            x = float(value["x"])
            y = float(value["y"])
            return x, y
        except Exception:
            return None

    @staticmethod
    def _extract_xy_from_sequence(value):
        if not isinstance(value, (list, tuple)) or len(value) < 2:
            return None

        try:
            x = float(value[0])
            y = float(value[1])
            return x, y
        except Exception:
            return None

    def _extract_location_xy(self, record: dict):
        # Old semantic_db.json: {"x": ..., "y": ...}
        xy = self._extract_xy_from_mapping(record)
        if xy is not None:
            return xy

        # Common Pose-like maps: {"pose": {"position": {"x": ..., "y": ...}}}
        pose = record.get("pose")
        if isinstance(pose, dict):
            position = pose.get("position", pose)
            xy = self._extract_xy_from_mapping(position)
            if xy is not None:
                return xy
            xy = self._extract_xy_from_sequence(position)
            if xy is not None:
                return xy

        for key in ["position", "center", "centroid", "bbox_center"]:
            value = record.get(key)
            xy = self._extract_xy_from_mapping(value)
            if xy is not None:
                return xy
            xy = self._extract_xy_from_sequence(value)
            if xy is not None:
                return xy

        return None

    def _location_id_from_record(self, fallback_key: str, record: dict) -> str:
        for key in [
            "location_id",
            "canonical_location_id",
            "room_id",
            "place_id",
            "id",
            "name",
            "room_name",
            "label",
            "tag",
        ]:
            value = record.get(key)
            if value is not None and str(value).strip():
                return str(value).strip()

        return str(fallback_key)

    def _load_recovery_locations(self, db_path: str):
        if not db_path:
            return []

        if not os.path.exists(db_path):
            self._log_stage_warn(
                "RECOVERY",
                f"Semantic map candidate does not exist: '{db_path}'.",
            )
            return []

        try:
            with open(db_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            self._log_stage_warn(
                "RECOVERY",
                f"Failed to read semantic map candidate '{db_path}' for recovery summaries: {exc}",
            )
            return []

        parsed = []
        skipped_invalid_geometry = 0

        for fallback_key, record in self._iter_recovery_location_records(data):
            if not isinstance(record, dict):
                continue

            frame_id = str(record.get("frame_id", record.get("frame", "map")))
            if frame_id and frame_id != "map":
                continue

            xy = self._extract_location_xy(record)
            if xy is None:
                skipped_invalid_geometry += 1
                continue

            x, y = xy
            if not math.isfinite(x) or not math.isfinite(y):
                skipped_invalid_geometry += 1
                continue

            parsed.append({
                "id": self._location_id_from_record(fallback_key, record),
                "x": x,
                "y": y,
                "source": db_path,
            })

        self._log_stage_info(
            "RECOVERY",
            (
                f"Parsed {len(parsed)} semantic recovery locations from '{db_path}' "
                f"(skipped_invalid_geometry={skipped_invalid_geometry})."
            ),
        )

        return parsed

    @staticmethod
    def _normalize_object_tag(tag: str) -> str:
        return " ".join(str(tag or "").strip().lower().split())

    @staticmethod
    def _safe_object_state(value: str) -> str:
        state = str(value or "").strip().lower()
        if state in {"static", "semi-static", "movable"}:
            return state
        return ""

    @staticmethod
    def _safe_safety_class(value: str) -> str:
        safety_class = str(value or "none").strip().lower()
        if safety_class in {"none", "human", "animal"}:
            return safety_class
        return "none"

    @staticmethod
    def _make_responsible_object_key(tag: str, object_id: int) -> str:
        normalized_tag = NavigationOrchestrator._normalize_object_tag(tag)
        if not normalized_tag:
            normalized_tag = "object"
        return f"{normalized_tag}:{int(object_id)}"

    def _load_object_action_attributes(self, path: str):
        defaults = ObjectActionAttributes(
            openable=False,
            clearable=False,
            safety_class="none",
        )
        by_tag = {}

        if not path:
            self._log_stage_warn(
                "RECOVERY",
                "object_action_attributes_path is empty. Object action attributes use safe defaults.",
            )
            return defaults, by_tag

        if not os.path.exists(path):
            self._log_stage_warn(
                "RECOVERY",
                f"object_action_attributes_path does not exist: '{path}'. "
                "Object action attributes use safe defaults.",
            )
            return defaults, by_tag

        try:
            with open(path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            self._log_stage_warn(
                "RECOVERY",
                f"Failed to read object action attributes '{path}': {exc}. "
                "Object action attributes use safe defaults.",
            )
            return defaults, by_tag

        raw_defaults = data.get("defaults", {})
        if isinstance(raw_defaults, dict):
            defaults = ObjectActionAttributes(
                openable=bool(raw_defaults.get("openable", False)),
                clearable=bool(raw_defaults.get("clearable", False)),
                safety_class=self._safe_safety_class(raw_defaults.get("safety_class", "none")),
            )

        raw_by_tag = data.get("by_tag", {})
        if isinstance(raw_by_tag, dict):
            for tag, attrs in raw_by_tag.items():
                if not isinstance(attrs, dict):
                    continue

                normalized_tag = self._normalize_object_tag(tag)
                if not normalized_tag:
                    continue

                by_tag[normalized_tag] = ObjectActionAttributes(
                    openable=bool(attrs.get("openable", defaults.openable)),
                    clearable=bool(attrs.get("clearable", defaults.clearable)),
                    safety_class=self._safe_safety_class(
                        attrs.get("safety_class", defaults.safety_class)
                    ),
                )

        self._log_stage_info(
            "RECOVERY",
            (
                f"Loaded object action attributes: "
                f"defaults(openable={defaults.openable}, "
                f"clearable={defaults.clearable}, "
                f"safety_class='{defaults.safety_class}'), "
                f"tag_entries={len(by_tag)}."
            ),
        )

        return defaults, by_tag

    def _object_attributes_for_tag(self, tag: str) -> ObjectActionAttributes:
        normalized_tag = self._normalize_object_tag(tag)
        return self._object_action_by_tag.get(
            normalized_tag,
            self._object_action_defaults,
        )

    def _iter_semantic_object_records(self, data):
        if isinstance(data, dict):
            objects = data.get("objects", None)
            if isinstance(objects, dict):
                for key, record in objects.items():
                    yield key, record
                return

            if isinstance(objects, list):
                for index, record in enumerate(objects):
                    yield f"object_{index}", record
                return

            for key, record in data.items():
                if str(key).startswith("object_") and isinstance(record, dict):
                    yield key, record

    def _load_semantic_objects(self, db_path: str) -> List[SemanticObject]:
        if not db_path:
            self._log_stage_warn(
                "RECOVERY",
                "semantic_object_db_path is empty. Responsible-object matching disabled.",
            )
            return []

        if not os.path.exists(db_path):
            self._log_stage_warn(
                "RECOVERY",
                f"semantic_object_db_path does not exist: '{db_path}'. "
                "Responsible-object matching disabled.",
            )
            return []

        try:
            with open(db_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as exc:
            self._log_stage_warn(
                "RECOVERY",
                f"Failed to read semantic object DB '{db_path}': {exc}. "
                "Responsible-object matching disabled.",
            )
            return []

        objects: List[SemanticObject] = []
        skipped_missing_state = 0
        skipped_invalid_geometry = 0

        for fallback_key, record in self._iter_semantic_object_records(data):
            if not isinstance(record, dict):
                continue

            state = self._safe_object_state(
                record.get("object_state", record.get("object-state", ""))
            )
            if not state:
                skipped_missing_state += 1
                continue

            tag = str(
                record.get(
                    "object_tag",
                    record.get("tag", record.get("name", fallback_key)),
                )
            ).strip()
            caption = str(record.get("object_caption", record.get("caption", "")))

            try:
                object_id = int(record.get("id", len(objects)))
            except Exception:
                object_id = len(objects)

            center = record.get("bbox_center", record.get("center", None))
            extent = record.get("bbox_extent", record.get("extent", None))

            try:
                cx = float(center[0])
                cy = float(center[1])
                cz = float(center[2]) if len(center) > 2 else 0.0
                ex = abs(float(extent[0]))
                ey = abs(float(extent[1]))
                ez = abs(float(extent[2])) if len(extent) > 2 else 0.0
            except Exception:
                skipped_invalid_geometry += 1
                continue

            values = [cx, cy, cz, ex, ey, ez]
            if not all(math.isfinite(v) for v in values):
                skipped_invalid_geometry += 1
                continue

            try:
                volume = float(record.get("bbox_volume", record.get("volume", 0.0)))
            except Exception:
                volume = 0.0

            attrs = self._object_attributes_for_tag(tag)
            key = self._make_responsible_object_key(tag, object_id)

            objects.append(
                SemanticObject(
                    key=key,
                    object_id=object_id,
                    tag=tag,
                    caption=caption,
                    state=state,
                    x=cx,
                    y=cy,
                    z=cz,
                    extent_x=ex,
                    extent_y=ey,
                    extent_z=ez,
                    volume=volume,
                    openable=attrs.openable,
                    clearable=attrs.clearable,
                    safety_class=attrs.safety_class,
                )
            )

        self._log_stage_info(
            "RECOVERY",
            (
                f"Loaded {len(objects)} semantic objects for responsible-object matching "
                f"from '{db_path}'. "
                f"skipped_missing_state={skipped_missing_state}, "
                f"skipped_invalid_geometry={skipped_invalid_geometry}."
            ),
        )

        return objects

    def _find_semantic_object_by_key(self, key: str) -> Optional[SemanticObject]:
        if not key:
            return None

        for obj in self._semantic_objects:
            if obj.key == key:
                return obj

        return None

    def _bbox_contains_point_2d(
        self,
        obj: SemanticObject,
        point: Point,
        inflation_m: float,
    ) -> bool:
        half_x = (float(obj.extent_x) * 0.5) + float(inflation_m)
        half_y = (float(obj.extent_y) * 0.5) + float(inflation_m)

        return (
            abs(float(point.x) - float(obj.x)) <= half_x
            and abs(float(point.y) - float(obj.y)) <= half_y
        )

    @staticmethod
    def _distance_2d_to_object_center(obj: SemanticObject, point: Point) -> float:
        dx = float(point.x) - float(obj.x)
        dy = float(point.y) - float(obj.y)
        return math.sqrt(dx * dx + dy * dy)

    def _match_responsible_object(self, point: Point) -> ResponsibleObjectMatch:
        if not self._semantic_objects:
            return ResponsibleObjectMatch(
                match_type="unknown",
                object=None,
                distance_m=float("inf"),
                summary="semantic object catalog unavailable",
            )

        verified = []
        for obj in self._semantic_objects:
            if self._bbox_contains_point_2d(obj, point, self._bbox_inflation_m):
                verified.append((self._distance_2d_to_object_center(obj, point), obj))

        if verified:
            verified.sort(key=lambda item: item[0])
            distance, obj = verified[0]
            return ResponsibleObjectMatch(
                match_type="verified",
                object=obj,
                distance_m=distance,
                summary=(
                    f"verified object match: key='{obj.key}', tag='{obj.tag}', "
                    f"object_state='{obj.state}', distance={distance:.2f} m"
                ),
            )

        nearest = []
        for obj in self._semantic_objects:
            distance = self._distance_2d_to_object_center(obj, point)
            if distance <= float(self._nearest_fallback_radius_m):
                nearest.append((distance, obj))

        if nearest:
            nearest.sort(key=lambda item: item[0])
            distance, obj = nearest[0]
            return ResponsibleObjectMatch(
                match_type="inferred",
                object=obj,
                distance_m=distance,
                summary=(
                    f"inferred nearest object: key='{obj.key}', tag='{obj.tag}', "
                    f"object_state='{obj.state}', distance={distance:.2f} m"
                ),
            )

        return ResponsibleObjectMatch(
            match_type="unknown",
            object=None,
            distance_m=float("inf"),
            summary="no responsible object match",
        )

    def _copy_object_geometry_to_trigger(
        self,
        trigger: TriggerInfo,
        obj: SemanticObject,
        match_type: str,
    ) -> None:
        trigger.responsible_object_key = obj.key
        trigger.responsible_object_tag = obj.tag
        trigger.responsible_object_state = obj.state
        trigger.match_type = match_type

        trigger.responsible_bbox_center.x = float(obj.x)
        trigger.responsible_bbox_center.y = float(obj.y)
        trigger.responsible_bbox_center.z = float(obj.z)

        trigger.responsible_bbox_extent.x = float(obj.extent_x)
        trigger.responsible_bbox_extent.y = float(obj.extent_y)
        trigger.responsible_bbox_extent.z = float(obj.extent_z)

        trigger.responsible_safety_class = obj.safety_class
        trigger.responsible_openable = bool(obj.openable)
        trigger.responsible_clearable = bool(obj.clearable)

    def _augment_trigger_with_responsible_object(self, trigger: TriggerInfo) -> None:
        if trigger.responsible_object_key:
            obj = self._find_semantic_object_by_key(trigger.responsible_object_key)
            if obj is not None:
                match_type = trigger.match_type if trigger.match_type else "inferred"
                if match_type not in {"verified", "inferred", "unknown"}:
                    match_type = "inferred"
                self._copy_object_geometry_to_trigger(trigger, obj, match_type)
                self._log_stage_info(
                    "RECOVERY/OBJECT",
                    (
                        f"Using supplied responsible object: key='{obj.key}', "
                        f"match_type='{trigger.match_type}', "
                        f"object_state='{obj.state}', safety_class='{obj.safety_class}', "
                        f"openable={obj.openable}, clearable={obj.clearable}."
                    ),
                )
                return

            self._log_stage_warn(
                "RECOVERY/OBJECT",
                (
                    f"Trigger supplied responsible_object_key='{trigger.responsible_object_key}', "
                    "but it was not found in the semantic object DB. Treating as unknown."
                ),
            )
            trigger.responsible_object_key = ""

        match = self._match_responsible_object(trigger.blockage_centroid)
        if match.object is None:
            trigger.match_type = "unknown"
            trigger.responsible_object_tag = ""
            trigger.responsible_object_state = ""
            trigger.responsible_safety_class = "none"
            trigger.responsible_openable = False
            trigger.responsible_clearable = False
            self._log_stage_info("RECOVERY/OBJECT", match.summary)
            return

        self._copy_object_geometry_to_trigger(
            trigger=trigger,
            obj=match.object,
            match_type=match.match_type,
        )

        self._log_stage_info(
            "RECOVERY/OBJECT",
            (
                f"{match.summary}; safety_class='{match.object.safety_class}', "
                f"openable={match.object.openable}, clearable={match.object.clearable}."
            ),
        )

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
        self._active_recovery = False
        self._transition_recovery_fsm(
            RecoveryFSMState.ESCALATE_OPERATOR,
            reason=reason,
        )

        self._log_stage_warn(
            "RECOVERY",
            f"Escalating to operator intervention: reason='{reason}'",
        )

        if not self._allow_stdin_intervention:
            self._log_stage_error(
                "RECOVERY",
                "stdin intervention disabled. Aborting orchestrator.",
            )
            self._transition_recovery_fsm(
                RecoveryFSMState.TERMINAL_FAIL,
                reason="stdin_intervention_disabled",
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

                self._transition_recovery_fsm(
                    RecoveryFSMState.OPERATOR_RECHECK,
                    reason="operator_new_target",
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
                self._transition_recovery_fsm(
                    RecoveryFSMState.TERMINAL_FAIL,
                    reason="operator_abort",
                )
                return False

            if choice == "g":
                self._log_stage_warn(
                    "RECOVERY",
                    "Operator selected give up.",
                )
                self._transition_recovery_fsm(
                    RecoveryFSMState.TERMINAL_FAIL,
                    reason="operator_give_up",
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
            "fsm_state": self._fsm_state.value,
            "trigger_source": (
                self._last_trigger.trigger_source
                if self._last_trigger is not None
                else ""
            ),
            "trigger_debounce_key": (
                self._last_trigger.debounce_key
                if self._last_trigger is not None
                else ""
            ),
            "responsible_object": {
                "key": (
                    self._last_trigger.responsible_object_key
                    if self._last_trigger is not None
                    else ""
                ),
                "match_type": (
                    self._last_trigger.match_type
                    if self._last_trigger is not None
                    else "unknown"
                ),
                "tag": (
                    self._last_trigger.responsible_object_tag
                    if self._last_trigger is not None
                    else ""
                ),
                "object_state": (
                    self._last_trigger.responsible_object_state
                    if self._last_trigger is not None
                    else ""
                ),
                "safety_class": (
                    self._last_trigger.responsible_safety_class
                    if self._last_trigger is not None
                    else "none"
                ),
                "openable": (
                    bool(self._last_trigger.responsible_openable)
                    if self._last_trigger is not None
                    else False
                ),
                "clearable": (
                    bool(self._last_trigger.responsible_clearable)
                    if self._last_trigger is not None
                    else False
                ),
            },
            "responsible_object_key": (
                self._last_trigger.responsible_object_key
                if self._last_trigger is not None
                else ""
            ),
            "blockage_geometry": {
                "centroid": {
                    "x": (
                        float(self._last_trigger.blockage_centroid.x)
                        if self._last_trigger is not None
                        else 0.0
                    ),
                    "y": (
                        float(self._last_trigger.blockage_centroid.y)
                        if self._last_trigger is not None
                        else 0.0
                    ),
                    "z": (
                        float(self._last_trigger.blockage_centroid.z)
                        if self._last_trigger is not None
                        else 0.0
                    ),
                },
                "extent_m": (
                    float(self._last_trigger.blockage_extent_m)
                    if self._last_trigger is not None
                    else 0.0
                ),
            },
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

    def _resolve_query(
        self,
        query: str,
        recovery_context: dict = None,
    ) -> Optional[ResolvedTarget]:
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

        # Priority 1: object-centric recovery context (retry_target dispatch).
        if recovery_context and recovery_context.get("object_tag"):
            req.object_tag = recovery_context["object_tag"]
            req.intent_hint = recovery_context.get("intent_hint", "")
        # Priority 2: initial navigate_to_object intent from LLM parser.
        elif (parsed := getattr(self, "_parsed_command", None)) is not None \
                and getattr(parsed, "intent", "") == "navigate_to_object":
            req.object_tag = getattr(parsed, "object_tag", "") or ""
            req.intent_hint = getattr(parsed, "intent_hint", "") or ""
            req.target_object_key = getattr(parsed, "target_object_key", "") or ""
        else:
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

        if getattr(response, "object_key", ""):
            self.get_logger().info(
                f"[RETRIEVAL] object_tag='{response.object_tag}' "
                f"candidates={response.candidates_considered} "
                f"selected_object_key='{response.object_key}' "
                f"top_score={response.top_score:.3f} "
                f"db_version={response.db_version}"
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
        self._navigation_goal_active = False
        self._last_execution_message = ""
        self._last_feedback_distance_remaining = 0.0
        self._last_feedback_recoveries = 0
        self._last_feedback_pose = None
        self._reset_stall_watchdog()

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

        self._navigation_goal_active = True

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
            self._navigation_goal_active = False
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
            self._navigation_goal_active = False
            return False

        result_wrap = self._result_future.result()
        if result_wrap is None:
            self._last_execution_message = "ExecutePose action returned no result wrapper."
            self._log_stage_error(
                "EXECUTION",
                self._last_execution_message,
            )
            self._navigation_goal_active = False
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

        self._navigation_goal_active = False
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

        self._maybe_fire_stall_watchdog(fb)

    def _reset_stall_watchdog(self) -> None:
        self._stall_watchdog_triggered = False
        self._stall_baseline_distance_remaining = None
        self._stall_baseline_stamp_sec = None

    def _maybe_fire_stall_watchdog(self, fb) -> None:
        if not self._enable_stall_watchdog:
            return

        if self._stall_watchdog_triggered:
            return

        if self._active_recovery or not self._navigation_goal_active:
            return

        if self._fsm_state != RecoveryFSMState.EXECUTING:
            return

        try:
            distance_remaining = float(fb.distance_remaining)
            nav2_recoveries = int(fb.number_of_recoveries)
        except Exception:
            return

        if not math.isfinite(distance_remaining):
            return

        now = time.monotonic()

        if self._stall_baseline_distance_remaining is None:
            self._stall_baseline_distance_remaining = distance_remaining
            self._stall_baseline_stamp_sec = now
            return

        trigger_reason = ""

        if (
            self._stall_nav2_recoveries_cap > 0
            and nav2_recoveries >= self._stall_nav2_recoveries_cap
        ):
            trigger_reason = (
                f"nav2_recoveries_cap_reached:{nav2_recoveries}"
            )
        else:
            distance_delta = abs(
                distance_remaining - float(self._stall_baseline_distance_remaining)
            )

            if distance_delta > self._stall_distance_epsilon_m:
                self._stall_baseline_distance_remaining = distance_remaining
                self._stall_baseline_stamp_sec = now
                return

            elapsed = now - float(self._stall_baseline_stamp_sec or now)
            if elapsed >= self._stall_window_sec:
                trigger_reason = (
                    f"no_progress_for_{elapsed:.2f}s:"
                    f"distance_delta={distance_delta:.3f}"
                )

        if not trigger_reason:
            return

        self._stall_watchdog_triggered = True
        self._raise_stall_watchdog_trigger(
            reason=trigger_reason,
            current_pose=fb.current_pose,
            distance_remaining=distance_remaining,
            nav2_recoveries=nav2_recoveries,
        )

    def _raise_stall_watchdog_trigger(
        self,
        reason: str,
        current_pose: PoseStamped,
        distance_remaining: float,
        nav2_recoveries: int,
    ) -> None:
        blockage_centroid = Point()
        if current_pose is not None:
            blockage_centroid.x = float(current_pose.pose.position.x)
            blockage_centroid.y = float(current_pose.pose.position.y)
            blockage_centroid.z = float(current_pose.pose.position.z)

        trigger = TriggerInfo(
            trigger_source="stall_watchdog",
            failure_stage="execution",
            nav2_message=(
                f"Controller stall watchdog fired: {reason}; "
                f"distance_remaining={distance_remaining:.3f}; "
                f"nav2_recoveries={nav2_recoveries}"
            ),
            robot_pose=current_pose,
            match_type="unknown",
            blockage_centroid=blockage_centroid,
            blockage_extent_m=0.0,
            debounce_key=f"stall_watchdog:{reason}",
            stamp_sec=self.get_clock().now().nanoseconds * 1e-9,
        )

        status = self._on_trigger(trigger)

        self._log_stage_warn(
            "RECOVERY/STALL",
            (
                f"Stall watchdog trigger processed: status={status}, "
                f"reason='{reason}', distance_remaining={distance_remaining:.3f}, "
                f"nav2_recoveries={nav2_recoveries}"
            ),
        )

        if status == "accepted":
            self._cancel_active_goal_for_recovery(trigger)

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

    def _action_backstop_trigger(
        self,
        failure_stage: str,
        nav2_message: str,
        robot_pose: Optional[PoseStamped],
        distance_remaining: float = 0.0,
        nav2_recoveries: int = 0,
        failed_target_id: str = "",
        recovery_count: int = 0,
    ) -> str:
        blockage_centroid = Point()
        if robot_pose is not None:
            blockage_centroid.x = float(robot_pose.pose.position.x)
            blockage_centroid.y = float(robot_pose.pose.position.y)
            blockage_centroid.z = float(robot_pose.pose.position.z)

        trigger = TriggerInfo(
            trigger_source="action_backstop",
            failure_stage=failure_stage,
            nav2_message=nav2_message,
            robot_pose=robot_pose,
            match_type="unknown",
            blockage_centroid=blockage_centroid,
            blockage_extent_m=0.0,
            debounce_key=(
                f"action_backstop:{failure_stage}:"
                f"{failed_target_id or 'unknown'}:{recovery_count}"
            ),
            stamp_sec=self.get_clock().now().nanoseconds * 1e-9,
        )

        status = self._on_trigger(trigger)

        self.get_logger().info(
            f"[RECOVERY/BACKSTOP] status={status} "
            f"stage={failure_stage} distance_remaining={distance_remaining:.3f} "
            f"nav2_recoveries={nav2_recoveries}"
        )

        return status

    def _transition_recovery_fsm(self, new_state: RecoveryFSMState, reason: str = "") -> None:
        old_state = self._fsm_state
        self._fsm_state = new_state

        self.get_logger().info(
            f"[RECOVERY/FSM] {old_state.value} -> {new_state.value}"
            + (f" reason={reason}" if reason else "")
        )

        self._publish_recovery_status(new_state.value, reason=reason)


    def _publish_recovery_status(self, status: str, reason: str = "") -> None:
        msg = String()
        if reason:
            msg.data = f"{status}|reason={reason}"
        else:
            msg.data = status
        self._recovery_status_pub.publish(msg)
    
    def _trigger_bucket_key(self, trigger: TriggerInfo) -> str:
        if trigger.responsible_object_key:
            return f"object:{trigger.responsible_object_key}"

        if trigger.debounce_key:
            return f"debounce:{trigger.debounce_key}"

        x = round(float(trigger.blockage_centroid.x), 1)
        y = round(float(trigger.blockage_centroid.y), 1)
        return f"centroid:{x:.1f},{y:.1f}"


    def _is_duplicate_trigger(self, trigger: TriggerInfo) -> bool:
        now = time.monotonic()
        key = self._trigger_bucket_key(trigger)

        debounce_sec = (
            float(self.get_parameter("responsible_object_debounce_sec").value)
            if trigger.responsible_object_key
            else float(self.get_parameter("unknown_blockage_debounce_sec").value)
        )

        last = self._last_trigger_by_key.get(key)
        if last is not None and (now - last) < debounce_sec:
            return True

        self._last_trigger_by_key[key] = now
        return False
    
    def _finite_point(self, point: Point) -> bool:
        values = [point.x, point.y, point.z]
        return all(math.isfinite(float(v)) for v in values)

    def _trigger_is_navigation_source(self, trigger: TriggerInfo) -> bool:
        return trigger.trigger_source in {
            "action_backstop",
            "plan_intersection_monitor",
            "stall_watchdog",
        }

    def _validate_trigger(self, trigger: TriggerInfo) -> bool:
        if not trigger.trigger_source:
            self.get_logger().warn("[RECOVERY/TRIGGER] rejected trigger with empty source")
            return False

        if not self._finite_point(trigger.blockage_centroid):
            self._log_stage_warn(
                "RECOVERY/MONITOR",
                f"Rejected trigger from '{trigger.trigger_source}' with non-finite blockage centroid.",
            )
            return False

        if trigger.blocked_plan_index_hi < trigger.blocked_plan_index_lo:
            self._log_stage_warn(
                "RECOVERY/MONITOR",
                (
                    f"Rejected trigger from '{trigger.trigger_source}' with invalid plan indices: "
                    f"lo={trigger.blocked_plan_index_lo}, hi={trigger.blocked_plan_index_hi}."
                ),
            )
            return False

        if not math.isfinite(float(trigger.blockage_extent_m)) or float(trigger.blockage_extent_m) < 0.0:
            self._log_stage_warn(
                "RECOVERY/MONITOR",
                f"Rejected trigger from '{trigger.trigger_source}' with invalid blockage_extent_m.",
            )
            return False

        return True

    def _accept_trigger(self, trigger: TriggerInfo) -> str:
        self._last_trigger = trigger
        self._active_recovery = True
        self._transition_recovery_fsm(
            RecoveryFSMState.RECOVERY_IN_PROGRESS,
            reason=trigger.trigger_source,
        )
        return "accepted"

    def _cancel_active_goal_for_recovery(self, trigger: TriggerInfo) -> None:
        if self._goal_handle is None:
            self._log_stage_info(
                "RECOVERY/TRIGGER",
                f"Accepted trigger source='{trigger.trigger_source}' but no active goal handle is available to cancel.",
            )
            return

        self._log_stage_warn(
            "RECOVERY/TRIGGER",
            (
                f"Accepted trigger source='{trigger.trigger_source}'. "
                "Cancelling active ExecutePose goal so recovery can run through the orchestrator."
            ),
        )

        # Do not block inside a subscriber/feedback callback while waiting for the
        # cancel service response. The main MultiThreadedExecutor keeps spinning,
        # and the navigation worker observes the resulting ExecutePose terminal state.
        threading.Thread(target=self.cancel_goal, daemon=True).start()

    def _handle_recovery_trigger_msg(self, msg: RecoveryTrigger) -> None:
        if not self._enable_plan_intersection_trigger:
            return

        trigger = TriggerInfo(
            trigger_source=msg.trigger_source or "plan_intersection_monitor",
            failure_stage="execution",
            nav2_message=msg.note,
            robot_pose=self._make_recovery_pose(self._resolved_target),
            responsible_object_key=msg.responsible_object_key,
            match_type=msg.match_type or "unknown",
            blockage_centroid=msg.blockage_centroid,
            blockage_extent_m=float(msg.blockage_extent_m),
            blocked_plan_index_lo=int(msg.blocked_plan_index_lo),
            blocked_plan_index_hi=int(msg.blocked_plan_index_hi),
            debounce_key=msg.debounce_key,
            stamp_sec=self.get_clock().now().nanoseconds * 1e-9,
        )

        status = self._on_trigger(trigger)

        self._log_stage_info(
            "RECOVERY/MONITOR",
            (
                f"RecoveryTrigger processed: status={status}, "
                f"source='{trigger.trigger_source}', match_type='{trigger.match_type}', "
                f"key='{self._trigger_bucket_key(trigger)}', "
                f"blocked_indices=[{trigger.blocked_plan_index_lo}, {trigger.blocked_plan_index_hi}], "
                f"extent={trigger.blockage_extent_m:.3f}"
            ),
        )

        if status == "accepted":
            self._cancel_active_goal_for_recovery(trigger)

    def _on_trigger(self, trigger: TriggerInfo) -> str:
        if not self._validate_trigger(trigger):
            return "rejected"

        self._augment_trigger_with_responsible_object(trigger)

        if self._active_recovery:
            self.get_logger().info(
                f"[RECOVERY/TRIGGER] already_in_recovery source={trigger.trigger_source}"
            )
            return "already_in_recovery"

        if trigger.trigger_source != "action_backstop" and self._is_duplicate_trigger(trigger):
            self.get_logger().info(
                f"[RECOVERY/TRIGGER] duplicate source={trigger.trigger_source} "
                f"key={self._trigger_bucket_key(trigger)}"
            )
            return "duplicate"

        if not self._trigger_is_navigation_source(trigger):
            self.get_logger().info(
                f"[RECOVERY/TRIGGER] rejected non-wired trigger source={trigger.trigger_source}"
            )
            return "rejected"

        if trigger.trigger_source == "plan_intersection_monitor":
            if not self._navigation_goal_active:
                self.get_logger().info(
                    "[RECOVERY/TRIGGER] rejected monitor trigger because no active ExecutePose goal is running"
                )
                return "rejected"

            if self._fsm_state not in {
                RecoveryFSMState.EXECUTING,
                RecoveryFSMState.RECOVERY_IN_PROGRESS,
            }:
                self.get_logger().info(
                    f"[RECOVERY/TRIGGER] rejected monitor trigger while fsm_state={self._fsm_state.value}"
                )
                return "rejected"

        if trigger.trigger_source == "stall_watchdog":
            if not self._navigation_goal_active:
                self.get_logger().info(
                    "[RECOVERY/TRIGGER] rejected stall watchdog trigger because no active ExecutePose goal is running"
                )
                return "rejected"

            if self._fsm_state != RecoveryFSMState.EXECUTING:
                self.get_logger().info(
                    f"[RECOVERY/TRIGGER] rejected stall watchdog trigger while fsm_state={self._fsm_state.value}"
                )
                return "rejected"

        return self._accept_trigger(trigger)

    def _handle_request_recovery(
        self,
        request: RequestRecovery.Request,
        response: RequestRecovery.Response,
    ) -> RequestRecovery.Response:
        trigger = TriggerInfo(
            trigger_source=request.trigger_source or "bt_recovery_plugin",
            failure_stage=request.failure_stage or "execution",
            nav2_message=request.nav2_message,
            robot_pose=request.robot_pose,
            responsible_object_key=request.responsible_object_key,
            responsible_object_tag=request.responsible_object_tag,
            responsible_object_state=request.responsible_object_state,
            blockage_centroid=request.blockage_centroid,
            blockage_extent_m=float(request.blockage_extent_m),
            debounce_key=request.debounce_key,
            stamp_sec=self.get_clock().now().nanoseconds * 1e-9,
        )

        status = self._on_trigger(trigger)
        response.status = status

        if status == "accepted":
            response.message = "Recovery trigger accepted by orchestrator."
        elif status == "duplicate":
            response.message = "Duplicate recovery trigger absorbed."
        elif status == "already_in_recovery":
            response.message = "Recovery already in progress."
        else:
            response.message = "Recovery trigger rejected in current orchestrator mode."

        return response

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

    executor = MultiThreadedExecutor()
    executor.add_node(node)

    exit_code = 1
    navigation_done = threading.Event()

    def navigation_worker():
        nonlocal exit_code

        try:
            success = node.run()

            if success:
                node.get_logger().info('Navigation task completed successfully!')
                exit_code = 0
            else:
                node.get_logger().error('Navigation task failed.')
                exit_code = 1

        except Exception as exc:
            node.get_logger().error(f'Navigation worker failed: {exc}')
            exit_code = 1

        finally:
            navigation_done.set()

    worker = threading.Thread(target=navigation_worker, daemon=True)
    worker.start()

    try:
        while rclpy.ok():
            executor.spin_once(timeout_sec=0.1)

            if not node._start_idle and navigation_done.is_set():
                break

        raise SystemExit(exit_code)

    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received, cancelling goal...')
        node.cancel_goal()
        raise SystemExit(130)

    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()