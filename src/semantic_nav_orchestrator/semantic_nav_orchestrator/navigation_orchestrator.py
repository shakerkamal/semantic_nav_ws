import os
import math
import re
import sys
import json
import uuid
import time
import threading
from datetime import datetime, timezone
from dataclasses import dataclass, asdict, field, replace
from typing import Optional, List, Tuple
from enum import Enum

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from action_msgs.msg import GoalStatus
from builtin_interfaces.msg import Time
from geometry_msgs.msg import Point, PoseStamped, Vector3
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from ament_index_python.packages import get_package_share_directory
from rclpy.duration import Duration
from tf2_ros import TransformException, Buffer, TransformListener
from nav2_msgs.srv import ClearEntireCostmap

from std_srvs.srv import Trigger

from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from nav2_msgs.action import Spin
from semantic_nav_interfaces.action import ExecutePose
from semantic_nav_interfaces.msg import SemanticCorrectionReport, SemanticStoreUpdated
from semantic_nav_interfaces.srv import (
    InferAffordance,
    MatchResponsibleObject,
    NavigateToQuery,
    OperatorDecision,
    ParseSemanticCommand,
    ProposeRecovery,
    RequestRecovery,
    ResolveLocation,
    ValidatePose,
)
from semantic_nav_orchestrator.responsible_object_matcher import (
    ObjectCandidate,
    match_responsible_object,
)
from semantic_nav_orchestrator.costmap_adapter import occupancygrid_to_costgrid
from semantic_nav_orchestrator.affordance_classification import (
    InferredAffordance,
    accept_inference,
    tag_is_classifiable,
)
from semantic_nav_orchestrator.up_front_policy import (
    STANDOFF_OBJECT_KEY,
    ResponsibleAffordances,
    barrier_cleared_status,
    behavior_tree_for_target,
    eligible_after_attempts,
    eligible_directives,
    operator_prompt_for,
    select_and_override_directive,
)
from semantic_nav_orchestrator.global_blockage_diagnosis import (
    DIAG_REACHABLE,
    barrier_lethal_fraction,
    diagnose_global_blockage,
)
from semantic_nav_orchestrator.recovery_directives import (
    LLMProposal as DirectiveLLMProposal,
    OverrideConfig,
    ProposalContext,
    build_give_up_directive,
    build_open_door_directive,
    build_clear_object_directive,
    build_retry_target_directive,
    build_wait_then_replan_directive,
)
from semantic_nav_orchestrator.semantic_map_updates import (
    write_displaced_semistatic_map,
)


_OBJECT_KEY_RE = re.compile(r"[a-z][a-z0-9 _]*:\d+")


def _looks_like_object_key(s: str) -> bool:
    return bool(_OBJECT_KEY_RE.fullmatch((s or "").strip().lower()))


@dataclass(frozen=True)
class ResolvedTarget:
    query: str
    pose: PoseStamped
    db_version: int
    db_stamp: Time
    object_key: str = ""
    object_tag: str = ""
    intent_hint: str = ""

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
    responsible_state_detail: str = ""
    responsible_traversability: str = ""

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
        
        self._callback_group = ReentrantCallbackGroup()

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

        self.declare_parameter("semantic_map_path", default_semantic_map_path)
        self.declare_parameter("global_frame", "map")
        self.declare_parameter("robot_base_frame", "base_link")
        self.declare_parameter("nearest_location_count", 5)

        # Recovery parameters
        self.declare_parameter("recovery_cap", 3)
        self.declare_parameter("propose_recovery_service", "/propose_recovery")
        self.declare_parameter("recovery_log_path", "")
        self.declare_parameter("require_recovery_approval", False)
        self.declare_parameter("allow_stdin_intervention", True)

        default_bt_xml_path = os.path.join(
            get_package_share_directory("semantic_nav_nav2_plugins"),
            "config",
            "semantic_recovery_bt.xml",
        )

        self.declare_parameter('planner_id', '')
        self.declare_parameter('behavior_tree', default_bt_xml_path)
        # BT for the deterministic up-front standoff approach. Empty -> Nav2's
        # configured default (stock geometric recovery); it must NOT be the
        # semantic recovery BT, so the "no LLM" up-front layer stays LLM-free.
        self.declare_parameter('standoff_behavior_tree', '')
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
        self.declare_parameter("responsible_object_debounce_sec", 2.0)
        self.declare_parameter("unknown_blockage_debounce_sec", 1.0)
        self.declare_parameter("bbox_inflation_m", 0.20)
        self.declare_parameter("nearest_fallback_radius_m", 0.90)
        self.declare_parameter("start_idle", False)

        self.declare_parameter("orchestration_mode", "bt_led")  # bt_led is the only active mode
        self.declare_parameter("environment_id", "")
        self.declare_parameter("semantic_map_id", "")
        self.declare_parameter("signal_attempts_default", 3)
        self.declare_parameter("short_signal_wait_seconds", 2)
        self.declare_parameter("passive_wait_seconds_default", 5)
        self.declare_parameter("max_wait_seconds", 30)

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
        self._global_frame = self.get_parameter('global_frame').get_parameter_value().string_value.strip()
        self._robot_base_frame = self.get_parameter('robot_base_frame').get_parameter_value().string_value.strip()
        self._nearest_location_count = self.get_parameter('nearest_location_count').get_parameter_value().integer_value
        self._semantic_object_db_path = self.get_parameter("semantic_object_db_path").get_parameter_value().string_value.strip()
        self._object_action_attributes_path = self.get_parameter("object_action_attributes_path").get_parameter_value().string_value.strip()
        self._bbox_inflation_m = self.get_parameter("bbox_inflation_m").get_parameter_value().double_value
        self._nearest_fallback_radius_m = self.get_parameter("nearest_fallback_radius_m").get_parameter_value().double_value
        self._start_idle = self.get_parameter("start_idle").get_parameter_value().bool_value

        self._orchestration_mode = (
            self.get_parameter("orchestration_mode")
            .get_parameter_value()
            .string_value
            .strip()
            .lower()
        )

        if self._orchestration_mode not in {"bt_led"}:
            self._log_stage_warn(
                "RECOVERY",
                (
                    f"Invalid orchestration_mode='{self._orchestration_mode}'. "
                    "Falling back to 'bt_led'."
                ),
            )
            self._orchestration_mode = "bt_led"

        self._environment_id = (
            self.get_parameter("environment_id").get_parameter_value().string_value.strip()
        )
        self._semantic_map_id = (
            self.get_parameter("semantic_map_id").get_parameter_value().string_value.strip()
        )
        self._semantic_map_version: str = ""  # updated when provider publishes

        self._signal_attempts_default = (
            self.get_parameter("signal_attempts_default")
            .get_parameter_value()
            .integer_value
        )
        self._short_signal_wait_seconds = (
            self.get_parameter("short_signal_wait_seconds")
            .get_parameter_value()
            .integer_value
        )
        self._passive_wait_seconds_default = (
            self.get_parameter("passive_wait_seconds_default")
            .get_parameter_value()
            .integer_value
        )
        self._max_wait_seconds = (
            self.get_parameter("max_wait_seconds")
            .get_parameter_value()
            .integer_value
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._log_stage_info(
            "RECOVERY",
            f"Loading semantic map from '{self._semantic_map_path}'.",
        )
        self._recovery_locations = self._load_recovery_locations_from_sources(
            [self._semantic_map_path]
        )
        self._object_action_defaults, self._object_action_by_tag = self._load_object_action_attributes(
            self._object_action_attributes_path,
        )
        self._semantic_objects = self._load_semantic_objects(self._semantic_object_db_path)

        self._planner_id = self.get_parameter('planner_id').get_parameter_value().string_value
        self._behavior_tree = self.get_parameter('behavior_tree').get_parameter_value().string_value
        self._standoff_behavior_tree = self.get_parameter(
            'standoff_behavior_tree').get_parameter_value().string_value
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
        self._bt_directive_in_progress = False
        self._attempt_records: List[AttemptRecord] = []

        self._parse_command_client = self.create_client(
            ParseSemanticCommand,
            self._parse_service_name,
            callback_group=self._callback_group,
        )
        self._propose_recovery_client = self.create_client(
            ProposeRecovery,
            self._propose_recovery_service_name,
            callback_group=self._callback_group,
        )
        self._resolve_location_client = self.create_client(
            ResolveLocation,
            self._resolve_service_name,
            callback_group=self._callback_group,
        )
        self._validate_pose_client = self.create_client(
            ValidatePose,
            self._validate_service_name,
            callback_group=self._callback_group,
        )
        self._execute_pose_client = ActionClient(
            self,
            ExecutePose,
            self._execute_action_name,
            callback_group=self._callback_group,
        )

        self._recovery_status_pub = self.create_publisher(
            String,
            self.get_parameter("recovery_status_topic").get_parameter_value().string_value,
            10,
        )
        self._publish_recovery_status("RECOVERY_IDLE")

        self._request_recovery_srv = self.create_service(
            RequestRecovery,
            self.get_parameter("request_recovery_service").get_parameter_value().string_value,
            self._handle_request_recovery,
            callback_group=self._callback_group,
        )

        self._match_responsible_object_service = self.create_service(
            MatchResponsibleObject,
            "/match_responsible_object",
            self._handle_match_responsible_object,
            callback_group=self._callback_group,
        )

        store_update_qos = QoSProfile(depth=1)
        store_update_qos.reliability = ReliabilityPolicy.RELIABLE
        store_update_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._semantic_store_updated_pub = self.create_publisher(
            SemanticStoreUpdated,
            "/semantic_store_updated",
            store_update_qos,
        )
        self._correction_report_pub = self.create_publisher(
            SemanticCorrectionReport,
            "/semantic_map/corrections",
            10,
        )

        # --- M3: global costmap subscription for up-front blockage diagnosis ---
        self.declare_parameter('global_costmap_topic', '/global_costmap/costmap')
        self.declare_parameter('up_front_recovery_enabled', True)
        self.declare_parameter('up_front_cap', 2)
        self.declare_parameter('up_front_standoff_distance_m', 1.0)
        self.declare_parameter('up_front_recheck_polls', 6)
        self.declare_parameter('up_front_recheck_interval_s', 2.0)
        # Max robot-to-barrier distance from which the rescan gate can actually
        # observe a state change. Farther than this after an operator action,
        # the robot first drives to the standoff to verify (costmaps only
        # update with line of sight; rescanning from across the house can
        # never confirm a door opened).
        self.declare_parameter('up_front_verify_range_m', 2.5)
        # Sticky attribution: on re-diagnosis within one recovery episode, an
        # unmatched centroid within this radius of the previously VERIFIED
        # responsible object reuses that attribution (clustering noise is far
        # more likely than a new anonymous obstacle at the same spot).
        self.declare_parameter('up_front_sticky_match_radius_m', 2.0)
        self._up_front_recovery_enabled = bool(
            self.get_parameter('up_front_recovery_enabled')
            .get_parameter_value().bool_value)
        self._up_front_cap = int(
            self.get_parameter('up_front_cap')
            .get_parameter_value().integer_value)
        self._up_front_standoff_distance_m = float(
            self.get_parameter('up_front_standoff_distance_m')
            .get_parameter_value().double_value)
        self._up_front_recheck_polls = int(
            self.get_parameter('up_front_recheck_polls')
            .get_parameter_value().integer_value)
        self._up_front_recheck_interval_s = float(
            self.get_parameter('up_front_recheck_interval_s')
            .get_parameter_value().double_value)
        self._up_front_verify_range_m = float(
            self.get_parameter('up_front_verify_range_m')
            .get_parameter_value().double_value)
        self._up_front_sticky_match_radius_m = float(
            self.get_parameter('up_front_sticky_match_radius_m')
            .get_parameter_value().double_value)
        # Generic (object-agnostic) confirmation gate: at the standoff, before
        # committing to the original goal, re-observe (spin) so SLAM + costmap
        # re-sense the passage, then require BOTH a valid plan AND the
        # responsible barrier's footprint to read clear on the costmap. Works
        # for any dynamic obstacle that opened/moved/was removed -- not just
        # doors. Deterministic; no LLM. Disable to fall back to plan-only.
        self.declare_parameter('up_front_require_barrier_clear', True)
        self.declare_parameter('barrier_clear_radius_m', 0.30)
        self.declare_parameter('barrier_clear_max_lethal_fraction', 0.15)
        self.declare_parameter('barrier_clear_lethal_threshold', 100)
        self.declare_parameter('barrier_clear_min_observed_cells', 8)
        self.declare_parameter('up_front_reobserve_enabled', True)
        self.declare_parameter('up_front_reobserve_yaw_rad', 0.7)
        self.declare_parameter('up_front_reobserve_time_allowance_s', 10.0)
        self._up_front_require_barrier_clear = bool(
            self.get_parameter('up_front_require_barrier_clear')
            .get_parameter_value().bool_value)
        self._barrier_clear_radius_m = float(
            self.get_parameter('barrier_clear_radius_m')
            .get_parameter_value().double_value)
        self._barrier_clear_max_lethal_fraction = float(
            self.get_parameter('barrier_clear_max_lethal_fraction')
            .get_parameter_value().double_value)
        self._barrier_clear_lethal_threshold = int(
            self.get_parameter('barrier_clear_lethal_threshold')
            .get_parameter_value().integer_value)
        self._barrier_clear_min_observed_cells = int(
            self.get_parameter('barrier_clear_min_observed_cells')
            .get_parameter_value().integer_value)
        self._up_front_reobserve_enabled = bool(
            self.get_parameter('up_front_reobserve_enabled')
            .get_parameter_value().bool_value)
        self._up_front_reobserve_yaw_rad = float(
            self.get_parameter('up_front_reobserve_yaw_rad')
            .get_parameter_value().double_value)
        self._up_front_reobserve_time_allowance_s = float(
            self.get_parameter('up_front_reobserve_time_allowance_s')
            .get_parameter_value().double_value)
        # Up-front operator directives (open_door/clear_object): prompt the
        # operator via /operator_decision, wait for confirmation, then re-scan
        # + re-validate. Disable to escalate (NEEDS_OPERATOR) instead.
        self.declare_parameter('up_front_operator_enabled', True)
        self.declare_parameter('operator_decision_service', '/operator_decision')
        self.declare_parameter('operator_prompt_timeout_sec', 120.0)
        self._up_front_operator_enabled = bool(
            self.get_parameter('up_front_operator_enabled')
            .get_parameter_value().bool_value)
        self._operator_prompt_timeout_sec = float(
            self.get_parameter('operator_prompt_timeout_sec')
            .get_parameter_value().double_value)
        # Dedicated (short) timeout for the up-front LLM strategy call, so a
        # slow/hung LLM fails fast to the deterministic default instead of
        # freezing the robot for the whole service_call_timeout_sec (240s).
        # 45s: headroom above observed inference (~9-32s), far below 240s.
        self.declare_parameter('up_front_llm_timeout_sec', 45.0)
        self._up_front_llm_timeout_sec = float(
            self.get_parameter('up_front_llm_timeout_sec')
            .get_parameter_value().double_value)
        # Ablation switch (A1 vs A2): when False the up-front loop skips the LLM
        # strategy call and uses the deterministic default from
        # select_and_override_directive -- i.e. the deterministic-only baseline.
        # True = LLM selects among the eligible set (the M4 contribution).
        # Read live at use-time (see _up_front_llm_enabled) so `ros2 param set`
        # flips A1<->A2 on the same SLAM map without a relaunch.
        self.declare_parameter('up_front_llm_enabled', True)
        # Open-set affordance inference (spec 21.4): for a blocker whose tag the
        # affordance table cannot classify, ask the LLM to infer its affordances
        # from the caption. Ablation switch (A1 vs A2 for the open-set case):
        # False = table-only, unclassifiable tags keep the restrictive default.
        # Also read live at use-time (see _open_set_inference_enabled).
        self.declare_parameter('open_set_inference_enabled', True)
        # Fixed-goal constraint: en-route (BT-led) recovery must never change
        # the goal autonomously -- there is no operator gate on that path, so
        # retry_target is dropped from the en-route eligible set. Up-front
        # retry_target is unaffected (it escalates to the operator). True only
        # for the explicit retry-as-alternative ablation leg (en-route S5).
        # Read live at use-time (see _enroute_retry_target_enabled) so the S5
        # ablation arm flips via `ros2 param set` without a relaunch, like the
        # sibling up_front_llm_enabled / open_set_inference_enabled switches.
        self.declare_parameter('enroute_retry_target_enabled', False)
        self.declare_parameter('affordance_confidence_floor', 60)
        self._affordance_confidence_floor = int(
            self.get_parameter('affordance_confidence_floor')
            .get_parameter_value().integer_value)
        self.declare_parameter('infer_affordance_service', '/infer_affordance')
        self._infer_affordance_service_name = (
            self.get_parameter('infer_affordance_service')
            .get_parameter_value().string_value)
        # The set of tags the affordance table already covers -- inference runs
        # only for tags absent from this set (and not matched by the door rule).
        self._affordance_table_tags = {
            str(k).strip().lower() for k in self._object_action_by_tag
        }
        self._infer_affordance_client = self.create_client(
            InferAffordance,
            self._infer_affordance_service_name,
            callback_group=self._callback_group,
        )
        self._latest_global_costmap = None
        self.create_subscription(
            OccupancyGrid,
            self.get_parameter('global_costmap_topic')
            .get_parameter_value().string_value,
            self._on_global_costmap,
            store_update_qos,
            callback_group=self._callback_group,
        )
        self._clear_global_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/global_costmap/clear_entirely_global_costmap',
            callback_group=self._callback_group,
        )
        self._clear_local_costmap_client = self.create_client(
            ClearEntireCostmap,
            '/local_costmap/clear_entirely_local_costmap',
            callback_group=self._callback_group,
        )
        # /spin behavior for the standoff re-observation maneuver.
        self._spin_client = ActionClient(
            self, Spin, '/spin', callback_group=self._callback_group,
        )
        # /operator_decision (semantic_nav_operator_io) for up-front operator
        # directives (open the door / clear the object, then confirm).
        self._operator_decision_client = self.create_client(
            OperatorDecision,
            self.get_parameter('operator_decision_service')
            .get_parameter_value().string_value,
            callback_group=self._callback_group,
        )

        # /navigate_to_query: one navigation at a time; terminal is the caller.
        self._nav_to_query_lock = threading.Lock()
        self._nav_to_query_srv = self.create_service(
            NavigateToQuery,
            "/navigate_to_query",
            self._handle_navigate_to_query,
            callback_group=self._callback_group,
        )
        self._cancel_navigation_srv = self.create_service(
            Trigger,
            "/cancel_navigation",
            self._handle_cancel_navigation,
            callback_group=self._callback_group,
        )

        self._goal_handle = None
        self._result_future = None
        self._navigation_goal_active = False
        self._final_success = False

        self._resolved_target: Optional[ResolvedTarget] = None
        self._parsed_command: Optional[ParsedCommand] = None
        
        # Active semantic target context used as a fallback for BT-led
        # /request_recovery calls. ExecutePose/NavigateToPose carries only
        # pose + behavior_tree, so these object-centric fields are not naturally
        # available on the Nav2 BT blackboard in M2.
        self._active_original_object_tag = ""
        self._active_original_intent_hint = ""
        self._active_current_target_object_key = ""
        self._active_nl_command = ""

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
            f"orchestration_mode='{self._orchestration_mode}', "
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
        # bt_led daemon launched from bringup with no query: stay alive and serve
        # BT service calls instead of immediately failing on an empty query.
        if self._orchestration_mode == "bt_led" and not self._query and not self._command:
            self._start_idle = True

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

        # -----------------------------------------------------------------------
        # BT-LED MODE 
        # Recovery ownership has been transferred to the Nav2 behavior tree
        # defined in semantic_recovery_bt.xml. The BT owns:
        #   - ValidateSemantic (geometric veto before motion starts)
        #   - PathClearCondition (corridor monitor during motion)
        #   - QuerySemanticContext (responsible-object identification)
        #   - EscalateToLLMRecovery (LLM directive via /request_recovery)
        #   - RetryTargetBranch / WaitThenReplanBranch / GiveUpTerminal
        # The orchestrator's role in bt_led mode is limited to:
        #   - Semantic resolution (query → pose)
        #   - Single ExecutePose dispatch (with BT XML path)
        #   - Serving /request_recovery and /propose_recovery to BT plugins
        # -----------------------------------------------------------------------
        if self._orchestration_mode == "bt_led":
            return self._run_bt_led_once(
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

        semantic_query = parsed.object_tag

        self._log_stage_info(
            "INTENT",
            (
                f"Natural-language command parsed: "
                f"command='{parsed.original_command}', "
                f"intent='{parsed.intent}', "
                f"object_tag='{parsed.object_tag}', "
                f"intent_hint='{parsed.intent_hint}', "
                f"target_object_key='{parsed.target_object_key}', "
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
                f"object_tag='{response.object_tag}', "
                f"intent_hint='{response.intent_hint}', "
                f"target_object_key='{response.target_object_key}', "
                f"confidence={response.confidence_percent}, "
                f"target_known={response.target_known}, "
                f"message='{response.message}'"
            ),
        )

        if not response.success:
            self._log_stage_error(
                "INTENT",
                f"Command parsing failed: {response.message}",
            )
            return None

        if response.intent != "navigate_to_object":
            self._log_stage_error(
                "INTENT",
                (
                    f"Parsed command is not executable navigation: "
                    f"intent='{response.intent}', message='{response.message}'"
                ),
            )
            return None

        if not response.target_known:
            self._log_stage_error(
                "INTENT",
                (
                    f"Parsed target is not known: "
                    f"intent='{response.intent}', "
                    f"object_tag='{response.object_tag}', "
                    f"message='{response.message}'"
                ),
            )
            return None

        return ParsedCommand(
            original_command=command,
            intent=response.intent,
            object_tag=response.object_tag,
            intent_hint=response.intent_hint,
            target_object_key=response.target_object_key,
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
                    f"(object_key='{target.object_key}', "
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
                    f"(object_key='{target.object_key}', "
                    f"db_version={target.db_version})."
                ),
            )
        else:
            self._log_stage_warn(
                "VALIDATION",
                (
                    f"Validation disabled. Proceeding directly to execution "
                    f"(object_key='{target.object_key}', "
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
    
    def _record_active_bt_target_context(
    self,
    target: ResolvedTarget,
    semantic_query: str,
    ) -> None:
        parsed = getattr(self, "_parsed_command", None)

        original_object_tag = ""
        original_intent_hint = ""

        if parsed is not None and getattr(parsed, "intent", "") == "navigate_to_object":
            original_object_tag = getattr(parsed, "object_tag", "") or ""
            original_intent_hint = getattr(parsed, "intent_hint", "") or ""

        if not original_object_tag:
            original_object_tag = (
                getattr(target, "object_tag", "")
                or (semantic_query if not _looks_like_object_key(semantic_query) else "")
                or ""
            )

        if not original_intent_hint:
            original_intent_hint = getattr(target, "intent_hint", "") or ""
        if not original_intent_hint:
            original_intent_hint = self._active_original_intent_hint

        current_target_object_key = getattr(target, "object_key", "") or ""

        self._active_original_object_tag = original_object_tag
        self._active_original_intent_hint = original_intent_hint
        self._active_current_target_object_key = current_target_object_key

        self._log_stage_info(
            "BT_LED",
            (
                "Active target context recorded: "
                f"original_object_tag='{self._active_original_object_tag}', "
                f"original_intent_hint='{self._active_original_intent_hint}', "
                f"current_target_object_key='{self._active_current_target_object_key}'."
            ),
        )

    def _on_global_costmap(self, msg):
        """Cache the latest global costmap for up-front blockage diagnosis."""
        self._latest_global_costmap = msg

    def _standoff_tuple_to_pose(self, standoff) -> PoseStamped:
        """Convert an (x, y, yaw) standoff tuple into a map-frame PoseStamped."""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = float(standoff[0])
        pose.pose.position.y = float(standoff[1])
        yaw = float(standoff[2])
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose

    def _clear_costmaps(self) -> None:
        """Fire-and-forget clear of local + global costmaps (best-effort).

        Wipes stale obstacle marks so a barrier that has physically cleared
        (e.g. a door opening) gets re-observed on the next costmap update.
        """
        for client in (
            self._clear_local_costmap_client,
            self._clear_global_costmap_client,
        ):
            if client.service_is_ready():
                client.call_async(ClearEntireCostmap.Request())

    def _barrier_confirmation(self, barrier_xy, radius_m: float) -> str:
        """Generic costmap footprint check for the standoff recheck.

        Samples the freshest global costmap around the barrier centroid and
        delegates to up_front_policy.barrier_cleared_status. Object-agnostic:
        returns "cleared" / "still_blocked" / "unconfirmed".
        """
        grid_msg = self._latest_global_costmap
        if grid_msg is None or barrier_xy is None:
            return "unconfirmed"
        grid = occupancygrid_to_costgrid(grid_msg)
        frac, observed = barrier_lethal_fraction(
            grid, barrier_xy, radius_m, self._barrier_clear_lethal_threshold
        )
        return barrier_cleared_status(
            frac, observed,
            self._barrier_clear_max_lethal_fraction,
            self._barrier_clear_min_observed_cells,
        )

    def _rescan_confirm_and_validate(self, target, barrier_xy, clear_radius) -> bool:
        """Re-scan at the standoff and confirm the goal is reachable.

        Each poll: re-observe (spin) so SLAM + costmap re-sense the passage,
        clear stale marks, then GATE on grounded perception -- a valid plan AND
        the responsible barrier's footprint reading CLEAR on the costmap.
        Object-agnostic; shared by the approach_and_recheck and the operator
        (open_door/clear_object) branches. Returns True if the goal became
        reachable within up_front_recheck_polls.
        """
        for poll in range(int(self._up_front_recheck_polls)):
            self._reobserve()
            self._clear_costmaps()
            time.sleep(float(self._up_front_recheck_interval_s))
            barrier = self._barrier_confirmation(barrier_xy, clear_radius)
            plan_ok = self._validate_pose(target)
            barrier_ok = (
                barrier == "cleared"
                or not self._up_front_require_barrier_clear
            )
            self._log_stage_info(
                "UP_FRONT",
                f"Recheck poll={poll}: barrier={barrier} plan_ok={plan_ok} "
                f"barrier_ok={barrier_ok}",
            )
            if plan_ok and barrier_ok:
                return True
        return False

    def _prompt_operator(self, action: str, obj) -> bool:
        """Prompt the operator via /operator_decision; return True if acknowledged.

        Deterministic; no LLM. The operator confirms they performed the physical
        action (opened the door / cleared the object) -- the robot then re-scans
        and re-validates rather than trusting the confirmation.
        """
        key = obj.key if obj is not None else ""
        if not self._operator_decision_client.wait_for_service(
            timeout_sec=self._service_wait_timeout_sec
        ):
            self._log_stage_warn(
                "UP_FRONT", "Operator decision service unavailable; escalating."
            )
            return False
        req = OperatorDecision.Request()
        req.prompt_text = operator_prompt_for(action, key)
        req.responsible_object_key = key
        req.failure_stage = "validation"
        req.directive_action = action
        req.recovery_event_id = self._session_id
        future = self._operator_decision_client.call_async(req)
        if not self._wait_for_future(future, self._operator_prompt_timeout_sec):
            self._log_stage_warn(
                "UP_FRONT", "Operator decision timed out; escalating."
            )
            return False
        resp = future.result() if future.exception() is None else None
        if resp is None:
            return False
        self._log_stage_info(
            "UP_FRONT",
            f"Operator decision for '{action}': acknowledged={resp.acknowledged} "
            f"note='{resp.operator_note}'",
        )
        return bool(resp.acknowledged)

    def _reobserve(self) -> None:
        """Rotate left-right in place so SLAM + costmap re-sense a freshly-opened
        doorway before the confirmation gate reads it. Perception grounding, not
        a decision; no LLM."""
        if not self._up_front_reobserve_enabled:
            return
        yaw = self._up_front_reobserve_yaw_rad
        for target_yaw in (yaw, -2.0 * yaw, yaw):  # left, right, recenter
            self._send_spin(target_yaw)

    def _send_spin(self, target_yaw: float) -> None:
        """Fire one Nav2 /spin of target_yaw radians and wait for it (best-effort)."""
        if not self._spin_client.wait_for_server(
            timeout_sec=self._action_server_wait_timeout_sec
        ):
            self._log_stage_warn(
                "UP_FRONT", "Spin behavior server unavailable; skipping re-observe."
            )
            return
        goal = Spin.Goal()
        goal.target_yaw = float(target_yaw)
        goal.time_allowance = Duration(
            seconds=self._up_front_reobserve_time_allowance_s
        ).to_msg()
        send_future = self._spin_client.send_goal_async(goal)
        if not self._wait_for_future(send_future, self._action_send_goal_timeout_sec):
            return
        handle = send_future.result()
        if handle is None or not handle.accepted:
            return
        result_future = handle.get_result_async()
        self._wait_for_future(
            result_future, self._up_front_reobserve_time_allowance_s + 5.0
        )

    def _run_up_front_recovery(self, target, initial_query: str) -> bool:
        """Diagnose an up-front blockage and run an approach-and-recheck loop.

        Deterministic (no LLM): directives come from up_front_policy (a pure
        function), and the standoff approach is dispatched with a plain BT (see
        behavior_tree_for_target) so its Nav2 recovery never escalates to the
        LLM. Bounded by up_front_cap. Returns True only if the original goal is
        ultimately reached; False escalates via the NavigateToQuery
        NEEDS_OPERATOR outcome (the orchestrator has no operator client of its
        own). The real goal, re-dispatched once the barrier clears, keeps the
        semantic recovery BT.
        """
        self._last_failure_kind = "execution"
        goal_ps = target.pose
        goal_xy = (goal_ps.pose.position.x, goal_ps.pose.position.y)

        # (action, outcome) of prior up-front attempts, fed to the LLM so it can
        # escalate (e.g. approach_and_recheck exhausted -> open_door_then_replan)
        # instead of repeating the same choice.
        up_front_attempts = []

        # Sticky attribution (this episode only): last VERIFIED responsible
        # object + affordances, reused when a later re-diagnosis drifts the
        # centroid off the object and matches nothing.
        sticky_obj = None
        sticky_aff = None

        for attempt in range(int(self._up_front_cap)):
            grid_msg = self._latest_global_costmap
            robot = self._lookup_robot_pose()
            if grid_msg is None or robot is None:
                self._log_stage_error(
                    "UP_FRONT",
                    "No global costmap or robot pose available; escalating.",
                )
                return False

            grid = occupancygrid_to_costgrid(grid_msg)
            robot_xy = (robot.pose.position.x, robot.pose.position.y)
            diag = diagnose_global_blockage(
                grid, robot_xy, goal_xy,
                standoff_distance_m=float(self._up_front_standoff_distance_m),
            )
            self._log_stage_info(
                "UP_FRONT",
                f"attempt={attempt} diagnosis={diag.diagnosis} "
                f"centroid={diag.barrier_centroid}",
            )
            if diag.diagnosis == DIAG_REACHABLE:
                self._log_stage_warn(
                    "UP_FRONT",
                    "Goal region is reachable but the planner still failed; "
                    "not a topological blockage. Escalating.",
                )
                return False

            center = diag.barrier_centroid or diag.approach_frontier
            if center is None:
                return False

            centroid_point = Point()
            centroid_point.x = float(center[0])
            centroid_point.y = float(center[1])
            match = self._match_responsible_object(centroid_point)
            obj = match.object

            # Sticky attribution: the barrier clusterer can drift the centroid
            # off a previously-verified object (e.g. merging door cells with
            # the adjacent wall after a close-range re-observation), which
            # would silently reset the affordances to the restrictive default
            # mid-recovery. Reuse the previous attribution when the unmatched
            # centroid is still near it; a fresh verified match always wins,
            # and the operator-confirm + rescan gates still stand downstream.
            if obj is None and sticky_obj is not None:
                sticky_dist = math.hypot(
                    center[0] - sticky_obj.x, center[1] - sticky_obj.y
                )
                if sticky_dist <= self._up_front_sticky_match_radius_m:
                    obj = sticky_obj
                    self._log_stage_info(
                        "UP_FRONT",
                        f"Centroid matched no object but lies "
                        f"{sticky_dist:.1f}m from previously verified "
                        f"'{sticky_obj.key}' (sticky radius "
                        f"{self._up_front_sticky_match_radius_m:.1f}m); "
                        "reusing that attribution.",
                    )

            if obj is None:
                aff = ResponsibleAffordances(
                    tag="", openable=False, clearable=False,
                    safety_class="none", match_type="none",
                )
            elif sticky_aff is not None and obj is sticky_obj:
                # Reused attribution keeps its affordances (including any
                # open-set inference already paid for at a prior attempt).
                aff = sticky_aff
            else:
                # Open-set inference (spec 21.4): if the affordance table can't
                # classify this tag, ask the LLM to infer the affordances from
                # the caption; otherwise use the table-driven object attributes.
                inferred = None
                if (self._open_set_inference_enabled()
                        and not tag_is_classifiable(obj.tag, self._affordance_table_tags)):
                    inferred = self._infer_affordance(obj.tag, obj.caption)
                aff = ResponsibleAffordances(
                    tag=obj.tag,
                    openable=inferred.openable if inferred else bool(obj.openable),
                    clearable=inferred.clearable if inferred else bool(obj.clearable),
                    safety_class=(inferred.safety_class if inferred
                                  else (obj.safety_class or "none")),
                    match_type=match.match_type or "none",
                )

            if obj is not None and aff.match_type == "verified":
                sticky_obj, sticky_aff = obj, aff

            standoff_ps = None
            has_standoff = False
            if diag.standoff_pose is not None:
                standoff_ps = self._standoff_tuple_to_pose(diag.standoff_pose)
                has_standoff = self._pose_is_reachable(standoff_ps)

            # Operator directives are only eligible once the robot is close
            # enough to VERIFY the state change afterwards (costmaps update
            # with line of sight). Far away, the filter forces an approach
            # first; open/clear become eligible on the next attempt.
            dist_to_barrier = math.hypot(
                robot_xy[0] - center[0], robot_xy[1] - center[1]
            )
            within_verify_range = (
                dist_to_barrier <= self._up_front_verify_range_m
            )

            # M4 filter-not-policy (spec 21.3/9): the deterministic layer filters
            # to the eligible set; the LLM selects among it when >=2 remain; the
            # deterministic override coerces invalid/unavailable picks. Actions
            # already tried this recovery are dropped (exhaustion) so the LLM
            # can't keep re-picking approach_and_recheck and is forced to escalate.
            tried_actions = {a for (a, _o) in up_front_attempts}
            eligible = eligible_after_attempts(
                eligible_directives(
                    diag.diagnosis, aff, has_standoff,
                    within_verify_range=within_verify_range,
                ),
                tried_actions,
            )
            llm_action = ""
            if len(eligible) >= 2 and self._up_front_llm_enabled():
                llm_action = self._request_up_front_llm_choice(
                    diag, aff, obj, eligible, target, initial_query,
                    up_front_attempts,
                )
            selection = select_and_override_directive(
                eligible, llm_action, aff, has_standoff
            )
            action = selection.action
            self._log_stage_info(
                "UP_FRONT",
                f"eligible={eligible} llm='{llm_action}' -> directive={action} "
                f"(overridden={selection.overridden} reason={selection.reason}) "
                f"responsible_tag='{aff.tag}' match_type={aff.match_type} "
                f"has_standoff={has_standoff} "
                f"dist_to_barrier={dist_to_barrier:.1f}m "
                f"within_verify_range={within_verify_range}",
            )

            if action == "approach_and_recheck":
                standoff_target = ResolvedTarget(
                    query=STANDOFF_OBJECT_KEY,
                    pose=standoff_ps,
                    db_version=target.db_version,
                    db_stamp=target.db_stamp,
                    object_key=STANDOFF_OBJECT_KEY,
                )
                if not self._execute_pose(standoff_target):
                    self._log_stage_warn(
                        "UP_FRONT",
                        "Failed to reach the standoff pose; escalating.",
                    )
                    return False
                # Re-scan and re-validate at the standoff (spin -> clear ->
                # barrier-clear confirm -> validate; see the helper).
                barrier_xy = diag.barrier_centroid or center
                clear_radius = max(
                    self._barrier_clear_radius_m, diag.barrier_extent_m / 2.0
                )
                if self._rescan_confirm_and_validate(
                    target, barrier_xy, clear_radius
                ):
                    self._log_stage_info(
                        "UP_FRONT",
                        "Goal reachable and barrier clear after approach; "
                        "dispatching.",
                    )
                    return self._execute_pose(target)
                self._log_stage_info(
                    "UP_FRONT",
                    "Still blocked (barrier not confirmed clear) after approach "
                    "+ wait; re-diagnosing.",
                )
                up_front_attempts.append(
                    ("approach_and_recheck", "approached_still_blocked")
                )
                continue

            if action == "wait_then_replan":
                time.sleep(5.0)
                if self._validate_pose(target):
                    return self._execute_pose(target)
                up_front_attempts.append(("wait_then_replan", "waited_still_blocked"))
                continue

            if action in ("open_door_then_replan", "clear_object_then_replan"):
                # Execute the operator directive up-front: prompt the operator,
                # wait for confirmation, then re-scan + re-validate (same gate as
                # approach). No operator client / declined -> escalate.
                if not self._up_front_operator_enabled:
                    self._log_stage_info(
                        "UP_FRONT",
                        f"Directive '{action}' needs operator action "
                        "(up_front_operator_enabled=false); escalating.",
                    )
                    return False
                if not self._prompt_operator(action, obj):
                    self._log_stage_info(
                        "UP_FRONT",
                        f"Operator declined/unavailable for '{action}'; "
                        "escalating.",
                    )
                    return False
                barrier_xy = diag.barrier_centroid or center
                # The rescan gate verifies by OBSERVING the barrier: costmaps
                # only update with line of sight, so rescanning from far away
                # is guaranteed to stay 'still_blocked' even after the operator
                # really opened/cleared it. Drive to the standoff first when
                # out of verify range (best effort — fall back to an in-place
                # rescan if the approach fails).
                robot_now = self._lookup_robot_pose()
                if robot_now is not None and has_standoff:
                    dist_to_barrier = math.hypot(
                        robot_now.pose.position.x - barrier_xy[0],
                        robot_now.pose.position.y - barrier_xy[1],
                    )
                    if dist_to_barrier > self._up_front_verify_range_m:
                        self._log_stage_info(
                            "UP_FRONT",
                            f"Robot is {dist_to_barrier:.1f}m from the barrier "
                            f"(verify range "
                            f"{self._up_front_verify_range_m:.1f}m); moving to "
                            "the standoff to verify the operator action.",
                        )
                        standoff_target = ResolvedTarget(
                            query=STANDOFF_OBJECT_KEY,
                            pose=standoff_ps,
                            db_version=target.db_version,
                            db_stamp=target.db_stamp,
                            object_key=STANDOFF_OBJECT_KEY,
                        )
                        if not self._execute_pose(standoff_target):
                            self._log_stage_warn(
                                "UP_FRONT",
                                "Failed to reach the standoff for "
                                "verification; rescanning in place.",
                            )
                clear_radius = max(
                    self._barrier_clear_radius_m, diag.barrier_extent_m / 2.0
                )
                if self._rescan_confirm_and_validate(
                    target, barrier_xy, clear_radius
                ):
                    self._log_stage_info(
                        "UP_FRONT",
                        f"Goal reachable after operator '{action}'; dispatching.",
                    )
                    return self._execute_pose(target)
                self._log_stage_info(
                    "UP_FRONT",
                    f"Still blocked after operator '{action}' + rescan; "
                    "re-diagnosing.",
                )
                up_front_attempts.append((action, "operator_confirmed_still_blocked"))
                continue

            # give_up / retry_target-without-alternative -> escalate.
            self._log_stage_info(
                "UP_FRONT",
                f"Directive '{action}' needs operator action; escalating.",
            )
            return False

        self._log_stage_info(
            "UP_FRONT", "Up-front recovery cap reached; escalating to operator.")
        return False

    def _run_bt_led_once(
        self,
        initial_query: str,
        original_nl_command: str = "",
        original_intent_hint: str = "",
    ) -> bool:
        self._attempt_records = []
        self._active_recovery = False
        self._bt_directive_in_progress = False
        self._active_nl_command = original_nl_command
        self._active_original_intent_hint = original_intent_hint
        # Distinguishes why a run failed so the terminal can react:
        #   "resolution" — query could not be resolved to a pose
        #   "execution"  — dispatched but Nav2 + BT recovery exhausted / gave up
        self._last_failure_kind: Optional[str] = None

        self._transition_recovery_fsm(
            RecoveryFSMState.EXECUTING,
            reason="bt_led_initial_dispatch",
        )

        self._log_stage_info(
            "BT_LED",
            (
                "BT-led mode enabled. The orchestrator will resolve and dispatch "
                "one ExecutePose goal; validation, planning/control failure, "
                "costmap clearing, and recovery retries are owned by the Nav2 BT."
            ),
        )

        target = self._resolve_query(initial_query)
        if target is None:
            self._last_failure_kind = "resolution"
            self._transition_recovery_fsm(
                RecoveryFSMState.TERMINAL_FAIL,
                reason="bt_led_resolution_failed",
            )
            return False

        self._record_active_bt_target_context(
            target=target,
            semantic_query=initial_query,
        )

        if not self._behavior_tree:
            self._log_stage_warn(
                "BT_LED",
                (
                    "behavior_tree parameter is empty. Nav2 will use its default "
                    "BT XML; semantic BT-led recovery will not run."
                ),
            )
        else:
            self._log_stage_info(
                "BT_LED",
                f"Dispatching ExecutePose with behavior_tree='{self._behavior_tree}'.",
            )

        if self._up_front_recovery_enabled and not self._validate_pose(target):
            self._log_stage_info(
                "UP_FRONT",
                "Pre-flight validation failed; entering up-front blockage recovery.",
            )
            return self._run_up_front_recovery(target, initial_query)

        self._log_stage_info(
            "BT_LED",
            (
                "Pre-flight validation passed (or up-front recovery disabled); "
                "dispatching ExecutePose. The Nav2 BT owns further recovery."
            ),
        )

        if not self._execute_pose(target):
            self._last_failure_kind = "execution"
            self._transition_recovery_fsm(
                RecoveryFSMState.TERMINAL_FAIL,
                reason="bt_led_execute_pose_failed",
            )
            return False

        self._transition_recovery_fsm(
            RecoveryFSMState.TERMINAL_SUCCESS,
            reason="bt_led_goal_reached",
        )
        return True
    
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
            "No usable semantic object catalog found in semantic_map_path.",
        )
        return []

    def _iter_recovery_location_records(self, data):
        if not isinstance(data, dict):
            return

        for key, record in data.items():
            if isinstance(record, dict) and "object_tag" in record:
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
        for key in ["object_tag", "id", "name"]:
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
        if state in {"static", "semi-static", "movable", "displaced", "dynamic"}:
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
                f"{original_target.object_key}: {d_target:.2f} m"
            )

        return f"nearest semantic locations: {nearest_text}{suffix}"

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

        resolved_object_key = (
            getattr(response, "object_key", "")
            or ""
        )
        resolved_object_tag = (
            getattr(response, "object_tag", "")
            or ""
        )

        resolved_intent_hint = ""
        if recovery_context and recovery_context.get("intent_hint"):
            resolved_intent_hint = recovery_context.get("intent_hint", "") or ""
        elif (parsed := getattr(self, "_parsed_command", None)) is not None:
            resolved_intent_hint = getattr(parsed, "intent_hint", "") or ""

        target = ResolvedTarget(
            query=query,
            pose=pose,
            db_version=int(response.db_version),
            db_stamp=response.db_stamp,
            object_key=resolved_object_key,
            object_tag=resolved_object_tag,
            intent_hint=resolved_intent_hint,
        )

        self._resolved_target = target

        self._log_stage_info(
            "RESOLUTION",
            (
                f"Resolved '{target.query}' -> "
                f"object_key='{target.object_key}', "
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
                f"(object_key='{target.object_key}', "
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
                    f"(object_key='{target.object_key}', "
                    f"db_version={target.db_version}): {response.message}"
                ),
            )
            return False

        self._log_stage_info(
            'VALIDATION',
            (
                f"Validation succeeded "
                f"(object_key='{target.object_key}', "
                f"db_version={target.db_version}): "
                f"{response.message}, "
                f"path_length={response.path_length:.3f}, "
                f"pose_count={response.pose_count}"
            ),
        )

        return True

    def _pose_is_reachable(self, pose: PoseStamped) -> bool:
        """Ask Nav2's planner whether a path to `pose` exists against the current
        costmap. Used during recovery to pick a *reachable* retry alternative
        instead of handing the BT a pose the planner provably cannot reach.

        Unlike _validate_pose (which logs against a ResolvedTarget), this is a
        bare ComputePathToPose probe returning a bool.
        """
        if pose is None:
            return False

        if not self._validate_pose_client.wait_for_service(
            timeout_sec=self._service_wait_timeout_sec
        ):
            self._log_stage_warn(
                "RECOVERY/BT",
                (
                    f"Validate pose service '{self._validate_service_name}' "
                    "unavailable; treating candidate as unreachable."
                ),
            )
            return False

        req = ValidatePose.Request()
        req.goal = pose
        req.planner_id = self._planner_id
        req.use_start = False

        fut = self._validate_pose_client.call_async(req)
        if not self._wait_for_future(fut, self._service_call_timeout_sec):
            return False
        if fut.exception() is not None:
            return False
        resp = fut.result()
        return bool(resp is not None and resp.valid)

    def _execute_pose(self, target: ResolvedTarget) -> bool:
        self._navigation_goal_active = False
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
        # Keep the deterministic standoff maneuver LLM-free: it gets a plain BT,
        # the real goal keeps the semantic recovery BT (see behavior_tree_for_target).
        goal_msg.behavior_tree = behavior_tree_for_target(
            target.object_key, self._behavior_tree, self._standoff_behavior_tree
        )

        self._log_stage_info(
            "EXECUTION",
            (
                f"Sending goal to execute_pose action server "
                f"(object_key='{target.object_key}', "
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
                f"(object_key='{target.object_key}', db_version={target.db_version})."
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
                f"(object_key='{target.object_key}', "
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
                f"object_key='{target.object_key}', "
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
                    f"object_key='{target.object_key}', "
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

    def _handle_navigate_to_query(
        self,
        request: NavigateToQuery.Request,
        response: NavigateToQuery.Response,
    ) -> NavigateToQuery.Response:
        if not self._nav_to_query_lock.acquire(blocking=False):
            response.success = False
            response.outcome = "BUSY"
            response.failure_reason = "Navigation already in progress; cancel first"
            return response
        try:
            query = request.query.strip()
            nl_command = request.nl_command.strip()
            if not query:
                response.success = False
                response.outcome = "INVALID"
                response.failure_reason = "Empty query"
                return response
            self._parsed_command = None
            self._last_failure_kind = None
            success = self._run_bt_led_once(
                initial_query=query,
                original_nl_command=nl_command,
                original_intent_hint=request.intent_hint.strip(),
            )
            response.success = success
            if success:
                response.outcome = "REACHED"
                response.failure_reason = ""
                # Surface the actual target reached: when BT recovery rerouted to
                # a reachable alternative, report that key instead of the original
                # query so the terminal does not claim the original was reached.
                redirected = self._last_redirected_target_key()
                response.reached_target = redirected if redirected else query
            elif self._last_failure_kind == "resolution":
                response.outcome = "RESOLUTION_FAILED"
                response.failure_reason = (
                    f"Could not resolve '{query}' to a known object in the "
                    "semantic map. Check the object key or try a different target."
                )
            else:
                # Dispatched, but Nav2 + every BT recovery tier was exhausted or the
                # recovery policy returned give_up. This is the operator-handoff case.
                response.outcome = "NEEDS_OPERATOR"
                response.failure_reason = (
                    f"Could not reach '{query}'. Geometric and semantic recovery "
                    "were exhausted and no reachable alternative was found. "
                    "Operator input required."
                )
        finally:
            self._nav_to_query_lock.release()
        return response

    def _handle_cancel_navigation(
        self,
        request: Trigger.Request,
        response: Trigger.Response,
    ) -> Trigger.Response:
        self.cancel_goal()
        response.success = True
        response.message = "Cancel sent"
        return response

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


    def _finite_point(self, point: Point) -> bool:
        values = [point.x, point.y, point.z]
        return all(math.isfinite(float(v)) for v in values)

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

    def _build_trigger_from_request(
        self,
        request: RequestRecovery.Request,
    ) -> TriggerInfo:
        trigger = TriggerInfo(
            trigger_source=request.trigger_source or "bt_recovery_plugin",
            failure_stage=request.failure_stage or "execution",
            nav2_message=request.nav2_message,
            robot_pose=request.robot_pose,
            responsible_object_key=request.responsible_object_key,
            responsible_object_tag=request.responsible_object_tag,
            responsible_object_state=request.responsible_object_state,
            responsible_safety_class=request.responsible_safety_class or "none",
            responsible_openable=bool(request.responsible_openable),
            responsible_clearable=bool(request.responsible_clearable),
            blockage_centroid=request.blockage_centroid,
            blockage_extent_m=float(request.blockage_extent_m),
            debounce_key=request.debounce_key,
            stamp_sec=self.get_clock().now().nanoseconds * 1e-9,
        )

        _srv_match = (request.responsible_match_type or "").strip().lower()
        trigger.match_type = (
            _srv_match
            if _srv_match in {"verified", "inferred"}
            else ("inferred" if trigger.responsible_object_key else "unknown")
        )

        trigger.responsible_state_detail = (
            request.responsible_state_detail or ""
        ).strip()
        trigger.responsible_traversability = (
            request.responsible_traversability or ""
        ).strip()

        return trigger

    def _arbitrate_bt_recovery_request(
        self,
        trigger: TriggerInfo,
    ) -> str:
        """BT-led request arbitration only.

        This path must not:
          - cancel Nav2
          - enter the legacy RecoveryFSM
          - call _accept_trigger()
          - call _on_trigger()
        """
        if not self._validate_trigger(trigger):
            return "rejected"

        self._augment_trigger_with_responsible_object(trigger)

        if self._bt_directive_in_progress:
            return "already_in_recovery"

        return "accepted"

    def _remaining_bt_retry_budget(self) -> int:
        return max(
            0,
            int(self._recovery_cap) - len(self._attempt_records),
        )

    def _bt_request_object_context(
    self,
    request: RequestRecovery.Request,
    ) -> Tuple[str, str, str]:
        original_object_tag = (
            request.original_object_tag
            or self._active_original_object_tag
            or ""
        )
        original_intent_hint = (
            request.original_intent_hint
            or self._active_original_intent_hint
            or ""
        )
        current_target_object_key = (
            request.current_target_object_key
            or self._active_current_target_object_key
            or ""
        )

        if not request.original_object_tag and original_object_tag:
            self._log_stage_info(
                "RECOVERY/BT",
                (
                    "Filled original_object_tag from active target context: "
                    f"'{original_object_tag}'."
                ),
            )

        if not request.original_intent_hint and original_intent_hint:
            self._log_stage_info(
                "RECOVERY/BT",
                (
                    "Filled original_intent_hint from active target context: "
                    f"'{original_intent_hint}'."
                ),
            )

        if not request.current_target_object_key and current_target_object_key:
            self._log_stage_info(
                "RECOVERY/BT",
                (
                    "Filled current_target_object_key from active target context: "
                    f"'{current_target_object_key}'."
                ),
            )

        return (
            original_object_tag,
            original_intent_hint,
            current_target_object_key,
        )

    def _trigger_to_affordances(self, trigger: TriggerInfo) -> ResponsibleAffordances:
        """Build the deterministic affordance view from an en-route BT trigger."""
        return ResponsibleAffordances(
            tag=trigger.responsible_object_tag or "",
            openable=bool(trigger.responsible_openable),
            clearable=bool(trigger.responsible_clearable),
            safety_class=trigger.responsible_safety_class or "none",
            match_type=trigger.match_type or "none",
        )

    def _eligible_for_trigger(self, trigger: TriggerInfo) -> list:
        """Eligible directive set for the en-route BT path (spec 21.3).

        No standoff is computed en-route (the geometric tier only backs up), so
        has_reachable_standoff=False -> approach_and_recheck is naturally
        excluded; it belongs to the up-front loop.

        Fixed-goal constraint: unlike the up-front loop (where retry_target
        escalates to the operator), the en-route path executes directives
        autonomously -- so retry_target would silently redirect the goal with
        no human in the loop. It is filtered out unless the explicit ablation
        switch enroute_retry_target_enabled is set.
        """
        elig = eligible_directives(
            "blocked",
            self._trigger_to_affordances(trigger),
            has_reachable_standoff=False,
        )
        if not self._enroute_retry_target_enabled():
            elig = [a for a in elig if a != "retry_target"]
        return elig

    def _enroute_retry_target_enabled(self) -> bool:
        """Read the S5 fixed-goal ablation switch live so `ros2 param set`
        flips arms without a relaunch."""
        return bool(self.get_parameter('enroute_retry_target_enabled')
                    .get_parameter_value().bool_value)

    def _up_front_llm_enabled(self) -> bool:
        """Read the M4 ablation switch live so `ros2 param set` flips A1<->A2
        without a relaunch."""
        return bool(self.get_parameter('up_front_llm_enabled')
                    .get_parameter_value().bool_value)

    def _open_set_inference_enabled(self) -> bool:
        """Read the open-set ablation switch live (spec 21.4)."""
        return bool(self.get_parameter('open_set_inference_enabled')
                    .get_parameter_value().bool_value)

    def _infer_affordance(self, tag, caption):
        """Return an InferredAffordance from the LLM, or None (fall back to
        table default). Runs only for unclassifiable tags (spec 21.4)."""
        if not self._infer_affordance_client.wait_for_service(
            timeout_sec=self._service_wait_timeout_sec
        ):
            self._log_stage_warn("UP_FRONT", "infer_affordance unavailable; table default.")
            return None
        req = InferAffordance.Request()
        req.object_tag = tag or ""
        req.object_caption = caption or ""
        future = self._infer_affordance_client.call_async(req)
        if not self._wait_for_future(future, self._up_front_llm_timeout_sec):
            self._log_stage_warn("UP_FRONT", "infer_affordance timed out; table default.")
            return None
        resp = future.result() if future.exception() is None else None
        if resp is None or not bool(resp.success):
            return None
        inf = InferredAffordance(
            bool(resp.openable), bool(resp.clearable),
            str(resp.safety_class or "none"), int(resp.confidence_percent),
        )
        self._log_stage_info(
            "UP_FRONT",
            f"open-set affordance inferred for tag='{tag}': openable={inf.openable} "
            f"clearable={inf.clearable} safety={inf.safety_class} conf={inf.confidence}",
        )
        if not accept_inference(inf, self._affordance_confidence_floor):
            self._log_stage_info("UP_FRONT", "inference below floor; table default.")
            return None
        return inf

    def _request_up_front_llm_choice(
        self, diag, aff, obj, eligible, target, initial_query: str, attempts=None
    ) -> str:
        """Ask the LLM to pick one action from the eligible set (spec 21.3).

        Returns the chosen action string, or "" on failure/timeout so the caller
        falls back to the deterministic default. The standoff pose is never
        requested from the LLM -- the orchestrator computes it. ``attempts`` is a
        list of (action, outcome) from prior up-front attempts so the LLM can
        escalate (e.g. approach exhausted -> ask the operator) instead of
        repeating the same choice.
        """
        if not self._propose_recovery_client.wait_for_service(
            timeout_sec=self._service_wait_timeout_sec
        ):
            self._log_stage_warn(
                "UP_FRONT",
                "Propose recovery service unavailable; deterministic fallback.",
            )
            return ""

        req = ProposeRecovery.Request()
        req.failure_stage = "validation"
        req.nav2_message = "up-front global blockage"
        req.original_nl_command = self._active_nl_command or (initial_query or "")
        req.original_object_tag = self._active_original_object_tag or aff.tag
        req.original_intent_hint = self._active_original_intent_hint
        req.current_target_object_key = getattr(target, "object_key", "") or ""
        req.original_target = (
            req.current_target_object_key or req.original_object_tag
        )

        req.responsible_object_key = obj.key if obj is not None else ""
        req.match_type = aff.match_type or "unknown"
        req.responsible_object_tag = aff.tag
        req.responsible_object_state = obj.state if obj is not None else ""
        req.responsible_safety_class = aff.safety_class or "none"
        req.responsible_openable = bool(aff.openable)
        req.responsible_clearable = bool(aff.clearable)

        if diag.barrier_centroid is not None:
            req.blockage_centroid = Point(
                x=float(diag.barrier_centroid[0]),
                y=float(diag.barrier_centroid[1]),
                z=0.0,
            )
        req.blockage_extent_m = float(diag.barrier_extent_m)

        # Robot pose + nearest-locations context so the LLM knows how far the
        # robot is from the blockage (far away -> approach_and_recheck is the
        # only way to verify a state change; see the recovery prompt).
        robot = self._lookup_robot_pose()
        if robot is not None:
            req.robot_pose_at_failure = robot
            req.nearest_locations_summary = self._build_nearest_locations_summary(
                robot_pose=robot,
                original_target=None,
            )

        req.allowed_actions = list(eligible)
        # Prior up-front attempts -> the LLM's "Already tried" context, so it
        # escalates instead of repeating an exhausted action.
        history = list(attempts or [])
        req.attempted_actions = [a for (a, _o) in history]
        req.attempt_outcomes = [o for (_a, o) in history]
        req.attempted_values = ["" for _ in history]
        req.attempt_rationales = ["" for _ in history]
        req.remaining_retry_budget = int(self._up_front_cap)
        req.db_version = int(self._db_version)
        if self._db_stamp is not None:
            req.db_stamp = self._db_stamp

        self._log_stage_info(
            "UP_FRONT",
            f"Requesting LLM recovery choice via /propose_recovery "
            f"(allowed={list(eligible)}, "
            f"timeout={self._up_front_llm_timeout_sec:.0f}s).",
        )
        t0 = time.monotonic()
        future = self._propose_recovery_client.call_async(req)
        if not self._wait_for_future(future, self._up_front_llm_timeout_sec):
            self._log_stage_warn(
                "UP_FRONT",
                f"Propose recovery did not return within "
                f"{self._up_front_llm_timeout_sec:.0f}s "
                f"(waited {time.monotonic() - t0:.1f}s); deterministic fallback.",
            )
            return ""
        elapsed = time.monotonic() - t0
        if future.exception() is not None:
            self._log_stage_warn(
                "UP_FRONT",
                f"Propose recovery raised after {elapsed:.1f}s: "
                f"{future.exception()}; deterministic fallback.",
            )
            return ""
        response = future.result()
        if response is None:
            self._log_stage_warn(
                "UP_FRONT", "Propose recovery returned no response object."
            )
            return ""
        self._log_stage_info(
            "UP_FRONT",
            f"LLM recovery response in {elapsed:.1f}s: success={response.success} "
            f"action='{response.action}' rationale='{response.rationale}' "
            f"message='{response.message}'",
        )
        if not bool(response.success):
            return ""
        return str(response.action or "")

    def _call_propose_recovery_for_bt_request(
        self,
        request: RequestRecovery.Request,
        trigger: TriggerInfo,
    ) -> Optional[RecoveryProposal]:
        if not self._propose_recovery_client.wait_for_service(
            timeout_sec=self._service_wait_timeout_sec
        ):
            self._log_stage_error(
                "RECOVERY/BT",
                (
                    f"Propose recovery service "
                    f"'{self._propose_recovery_service_name}' not available."
                ),
            )
            return None

        req = ProposeRecovery.Request()

        (
            original_object_tag,
            original_intent_hint,
            current_target_object_key,
        ) = self._bt_request_object_context(request)

        req.original_nl_command = (
            self._active_nl_command
            or (self._command if self._command else "")
        )
        req.original_target = (
            current_target_object_key
            or original_object_tag
            or ""
        )
        req.failure_stage = trigger.failure_stage
        req.nav2_message = trigger.nav2_message

        req.original_object_tag = original_object_tag
        req.original_intent_hint = original_intent_hint
        req.current_target_object_key = current_target_object_key

        req.attempted_actions = [a.action for a in self._attempt_records]
        req.attempted_values = [a.value for a in self._attempt_records]
        req.attempt_outcomes = [a.outcome for a in self._attempt_records]
        req.attempt_rationales = [a.rationale for a in self._attempt_records]

        req.robot_pose_at_failure = request.robot_pose
        req.nearest_locations_summary = self._build_nearest_locations_summary(
            robot_pose=request.robot_pose,
            original_target=None,
        )

        req.distance_remaining_at_abort = 0.0
        req.nav2_recoveries_attempted = 0
        req.remaining_retry_budget = int(self._remaining_bt_retry_budget())

        req.trigger_source = trigger.trigger_source
        req.responsible_object_key = trigger.responsible_object_key
        req.match_type = trigger.match_type or "unknown"
        req.responsible_object_tag = trigger.responsible_object_tag
        req.responsible_object_state = trigger.responsible_object_state
        req.responsible_bbox_center = trigger.responsible_bbox_center
        req.responsible_bbox_extent = trigger.responsible_bbox_extent
        req.responsible_safety_class = trigger.responsible_safety_class or "none"
        req.responsible_openable = bool(trigger.responsible_openable)
        req.responsible_clearable = bool(trigger.responsible_clearable)

        # M4: filter-not-policy. Hand the LLM exactly the eligible actions.
        req.allowed_actions = self._eligible_for_trigger(trigger)

        req.blockage_centroid = trigger.blockage_centroid
        req.blockage_extent_m = float(trigger.blockage_extent_m)

        req.deterministic_waits_used = 0
        req.deterministic_wait_cap = 0
        req.total_seconds_blocked = 0.0

        req.db_version = int(request.local_db_version or self._db_version)

        if request.local_db_stamp.sec != 0 or request.local_db_stamp.nanosec != 0:
            req.db_stamp = request.local_db_stamp
        elif self._db_stamp is not None:
            req.db_stamp = self._db_stamp

        future = self._propose_recovery_client.call_async(req)

        if not self._wait_for_future(future, self._service_call_timeout_sec):
            self._log_stage_error(
                "RECOVERY/BT",
                (
                    f"Service call to propose recovery timed out after "
                    f"{self._service_call_timeout_sec:.1f}s."
                ),
            )
            return None

        if future.exception() is not None:
            self._log_stage_error(
                "RECOVERY/BT",
                f"Propose recovery service call failed: {future.exception()}",
            )
            return None

        response = future.result()
        if response is None:
            self._log_stage_error(
                "RECOVERY/BT",
                "Propose recovery service returned no response.",
            )
            return None

        self._log_stage_info(
            "RECOVERY/BT",
            (
                f"BT proposal response: success={response.success}, "
                f"action='{response.action}', "
                f"target_object_tag='{getattr(response, 'target_object_tag', '')}', "
                f"target_intent_hint='{getattr(response, 'target_intent_hint', '')}', "
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

    def _proposal_to_directive_llm_proposal(
        self,
        proposal: RecoveryProposal,
        request: RequestRecovery.Request,
    ) -> DirectiveLLMProposal:
        return DirectiveLLMProposal(
            action=proposal.action,
            rationale=proposal.rationale,
            confidence_percent=int(proposal.confidence_percent),
            target_object_tag=(
                proposal.target_object_tag
                or proposal.target
                or ""
            ),
            target_intent_hint=(
                proposal.target_intent_hint
                or request.original_intent_hint
                or self._active_original_intent_hint
                or ""
            ),
            wait_seconds=int(proposal.wait_seconds),
            operator_message=proposal.operator_message,
            responsible_object_key=proposal.responsible_object_key,
        )

    def _build_bt_proposal_context(
        self,
        trigger: TriggerInfo,
        recovery_event_id: str,
    ) -> ProposalContext:
        return ProposalContext(
            attempts_used=len(self._attempt_records),
            retry_cap=int(self._recovery_cap),
            responsible_safety_class=trigger.responsible_safety_class or "none",
            responsible_object_state=trigger.responsible_object_state or "",
            recovery_event_id=recovery_event_id,
        )

    def _resolve_target_for_directive(
        self,
        object_tag: str,
        intent_hint: str,
        exclude_object_key: str = "",
    ):
        """Resolve object_tag to the nearest *reachable* instance pose + key.

        Every instance of object_tag is a candidate, including the previously
        blocked key (exclude_object_key) which is re-validated in case a
        transient block has cleared. Candidates are ordered nearest-first from
        the robot's current pose and each is checked with Nav2's planner
        (ComputePathToPose via /validate_pose_goal). The first instance the
        planner can actually reach *right now* (against the current costmap,
        which still contains the blockage) is returned.

        Returns (None, "") when no instance is reachable, in which case the
        caller degrades retry_target to give_up rather than handing the BT a
        pose the planner cannot reach (which would waste a recovery retry).

        intent_hint is retained for signature/Protocol compatibility; nearest-
        first reachability is the dominant selection criterion for recovery.

        Returns:
          (PoseStamped or None, object_key)
        """
        if not object_tag:
            return None, ""

        if not self._resolve_location_client.wait_for_service(
            timeout_sec=self._service_wait_timeout_sec
        ):
            self._log_stage_error(
                "RECOVERY/BT",
                (
                    f"Resolve location service '{self._resolve_service_name}' "
                    "not available while building retry_target directive."
                ),
            )
            return None, ""

        tag_norm = object_tag.strip().lower()

        # Candidate instances of this tag, nearest-first from the robot pose so
        # the planner check returns on the first reachable (and usually closest)
        # alternative — keeping recovery responsive.
        candidates = [obj for obj in self._semantic_objects if obj.tag == tag_norm]
        if not candidates:
            self._log_stage_error(
                "RECOVERY/BT",
                f"No instances of tag '{object_tag}' in the semantic catalog.",
            )
            return None, ""

        robot_pose = self._lookup_robot_pose()
        if robot_pose is not None:
            rx = robot_pose.pose.position.x
            ry = robot_pose.pose.position.y
            candidates.sort(
                key=lambda o: (o.x - rx) ** 2 + (o.y - ry) ** 2
            )

        def _resolve_key(key):
            req = ResolveLocation.Request()
            req.query = ""
            req.object_tag = ""
            req.intent_hint = ""
            req.target_object_key = key
            fut = self._resolve_location_client.call_async(req)
            if not self._wait_for_future(fut, self._service_call_timeout_sec):
                return None
            if fut.exception() is not None:
                return None
            r = fut.result()
            if r is None or not r.success:
                return None
            if not self._pose_is_valid_for_navigation(r.pose):
                return None
            return r

        resolved_count = 0
        for obj in candidates:
            resolved = _resolve_key(obj.key)
            if resolved is None:
                continue
            resolved_count += 1
            if not self._pose_is_reachable(resolved.pose):
                self._log_stage_info(
                    "RECOVERY/BT",
                    (
                        f"Candidate '{obj.key}' (tag='{object_tag}') resolved but "
                        "the planner cannot reach it right now; skipping."
                    ),
                )
                continue

            if exclude_object_key and obj.key == exclude_object_key:
                note = (
                    f"original target '{obj.key}' is reachable again "
                    "after the block cleared"
                )
            else:
                note = (
                    f"redirected from blocked "
                    f"'{exclude_object_key or 'n/a'}' to reachable "
                    f"alternative '{obj.key}'"
                )
            self._log_stage_info(
                "RECOVERY/BT",
                f"Retry target {note} (tag='{object_tag}').",
            )
            return resolved.pose, resolved.object_key or obj.key

        self._log_stage_warn(
            "RECOVERY/BT",
            (
                f"No reachable instance of tag '{object_tag}' among "
                f"{len(candidates)} candidate(s) ({resolved_count} resolved). "
                "Degrading retry_target to give_up."
            ),
        )
        return None, ""

    def _build_directive_from_bt_proposal(
        self,
        proposal: RecoveryProposal,
        request: RequestRecovery.Request,
        trigger: TriggerInfo,
        recovery_event_id: str,
    ):
        directive_proposal = self._proposal_to_directive_llm_proposal(
            proposal,
            request,
        )
        context = self._build_bt_proposal_context(
            trigger=trigger,
            recovery_event_id=recovery_event_id,
        )

        if not proposal.success:
            return build_give_up_directive(
                directive_proposal,
                context,
                overrides=OverrideConfig(
                    signal_attempts_default=self._signal_attempts_default,
                    short_signal_wait_seconds=self._short_signal_wait_seconds,
                    passive_wait_seconds_default=self._passive_wait_seconds_default,
                ),
            )

        # M4 filter-not-policy (spec 21.3): the LLM selects among the eligible
        # set; the deterministic layer overrides only invalid/ineligible picks.
        # No forced open_door/clear_object overrides -- those are eligible
        # actions the LLM selects (or the priority default falls back to).
        aff = self._trigger_to_affordances(trigger)
        eligible = self._eligible_for_trigger(trigger)
        selection = select_and_override_directive(
            eligible, proposal.action, aff, has_reachable_standoff=False
        )
        self._log_stage_info(
            "RECOVERY/BT",
            (
                f"eligible={eligible} llm='{proposal.action}' -> "
                f"action={selection.action} (overridden={selection.overridden} "
                f"reason={selection.reason})"
            ),
        )

        overrides = OverrideConfig(
            signal_attempts_default=self._signal_attempts_default,
            short_signal_wait_seconds=self._short_signal_wait_seconds,
            passive_wait_seconds_default=self._passive_wait_seconds_default,
        )

        if selection.action == "retry_target":
            _, _, blocked_key = self._bt_request_object_context(request)
            return build_retry_target_directive(
                directive_proposal,
                context,
                resolver=lambda tag, hint: self._resolve_target_for_directive(
                    tag, hint, exclude_object_key=blocked_key
                ),
            )

        if selection.action == "open_door_then_replan":
            return build_open_door_directive(
                directive_proposal,
                context,
                responsible_object_key=trigger.responsible_object_key or "",
            )

        if selection.action == "clear_object_then_replan":
            return build_clear_object_directive(
                directive_proposal,
                context,
                responsible_object_key=trigger.responsible_object_key or "",
            )

        if selection.action == "wait_then_replan":
            # Floor the wait when the action was overridden in (the LLM may not
            # have supplied wait_seconds for a different action).
            wait_proposal = directive_proposal
            if int(getattr(directive_proposal, "wait_seconds", 0) or 0) <= 0:
                wait_proposal = replace(
                    directive_proposal,
                    wait_seconds=int(self._passive_wait_seconds_default),
                )
            return build_wait_then_replan_directive(
                wait_proposal,
                context,
                signal_attempts_default=self._signal_attempts_default,
                max_wait_seconds=self._max_wait_seconds,
            )

        # give_up, plus any residual (e.g. approach_and_recheck, which is
        # up-front-only and never eligible on the en-route BT path).
        return build_give_up_directive(directive_proposal, context, overrides=overrides)

    def _record_bt_directive_attempt(
        self,
        directive,
        proposal: Optional[RecoveryProposal],
        trigger: TriggerInfo,
    ) -> None:
        if directive.action == "retry_target":
            value = directive.target_object_key or directive.target_object_tag
            outcome = "bt_directive_retry_target"
        elif directive.action == "wait_then_replan":
            value = str(int(directive.wait_seconds))
            outcome = "bt_directive_wait_then_replan"
        elif directive.action == "open_door_then_replan":
            value = directive.responsible_object_key
            outcome = "bt_directive_open_door_then_replan"
        elif directive.action == "clear_object_then_replan":
            value = directive.responsible_object_key
            outcome = "bt_directive_clear_object_then_replan"
        elif directive.action == "give_up":
            value = ""
            outcome = "bt_directive_give_up"
        else:
            value = ""
            outcome = "bt_directive_unsupported"

        self._attempt_records.append(
            AttemptRecord(
                action=directive.action,
                value=value,
                outcome=outcome,
                rationale=directive.rationale,
                failure_stage=trigger.failure_stage,
                message=proposal.message if proposal is not None else "",
            )
        )

    def _last_redirected_target_key(self) -> str:
        """Return the alternative object key the run was rerouted to, or "".

        Scans this run's attempt records for the most recent ``retry_target``
        directive and returns its target key when it differs from the originally
        resolved target. Returns "" when no reroute happened (no retry_target, or
        the retry pointed back at the original target after a transient block
        cleared) so callers can fall back to the original query string.
        """
        original_key = getattr(self, "_active_current_target_object_key", "") or ""
        for record in reversed(self._attempt_records):
            if record.action != "retry_target":
                continue
            value = (record.value or "").strip()
            if value and value != original_key:
                return value
            return ""
        return ""

    def _fill_request_recovery_response_from_directive(
        self,
        response: RequestRecovery.Response,
        status: str,
        message: str,
        directive,
    ) -> RequestRecovery.Response:
        response.status = status
        response.message = message
        response.action = directive.action or ""

        pose = directive.target_pose
        if isinstance(pose, PoseStamped):
            response.target_pose = pose
        elif pose is not None:
            # Backward-compatible pure-test tuple shape:
            # (frame_id, x, y, yaw). Orientation is identity here because
            # production path returns a full PoseStamped from ResolveLocation.
            frame_id, x, y, _yaw = pose
            response.target_pose.header.frame_id = str(frame_id)
            response.target_pose.header.stamp = self.get_clock().now().to_msg()
            response.target_pose.pose.position.x = float(x)
            response.target_pose.pose.position.y = float(y)
            response.target_pose.pose.orientation.w = 1.0

        response.target_object_key = directive.target_object_key
        response.target_object_tag = directive.target_object_tag
        response.target_intent_hint = directive.target_intent_hint

        response.wait_seconds = max(
            0,
            min(255, int(directive.wait_seconds)),
        )
        response.emit_signal_during_wait = bool(
            directive.emit_signal_during_wait
        )
        response.signal_attempts = max(
            0,
            min(255, int(directive.signal_attempts)),
        )

        response.responsible_object_key = directive.responsible_object_key
        response.operator_message = directive.operator_message

        response.rationale = directive.rationale
        response.confidence_percent = max(
            0,
            min(100, int(directive.confidence_percent)),
        )

        response.attempts_used = max(
            0,
            min(65535, len(self._attempt_records)),
        )
        response.retry_cap = max(
            0,
            min(65535, int(self._recovery_cap)),
        )
        response.escalate_to_operator = bool(directive.escalate_to_operator)
        response.recovery_event_id = directive.recovery_event_id

        return response

    def _fill_empty_request_recovery_response(
        self,
        response: RequestRecovery.Response,
        status: str,
        message: str,
        recovery_event_id: str = "",
    ) -> RequestRecovery.Response:
        response.status = status
        response.message = message
        response.action = ""
        response.attempts_used = max(
            0,
            min(65535, len(self._attempt_records)),
        )
        response.retry_cap = max(
            0,
            min(65535, int(self._recovery_cap)),
        )
        response.recovery_event_id = recovery_event_id
        response.escalate_to_operator = False
        return response
    
    def _maybe_update_semantic_map(
        self,
        *,
        object_key: str,
        object_tag: str,
        object_state: str,
        responsible_match_type: str,
        responsible_openable: bool,
        responsible_clearable: bool,
        directive_action: str,
        blockage_centroid: Point,
        blockage_extent_m: float,
        robot_pose: Optional[PoseStamped] = None,
        recovery_event_id: str = "",
    ) -> None:
        """Write a versioned map update for an inferred semi-static blockage.

        Guards (all must pass):
          - match_type == "inferred"  (blockage near but outside mapped bbox)
          - object_state == "semi-static"
          - object is not openable (door / gate handled by open_door directive)
          - object is not clearable (movable handled by clear_object directive)
          - directive is not give_up (no reason to update map on give_up)
          - object_key is parseable as tag:id
          - semantic_map_path is configured and exists
        """
        if responsible_match_type != "inferred":
            return
        if not object_key:
            return
        if object_state != "semi-static":
            return
        if responsible_openable:
            return
        if responsible_clearable:
            return
        if directive_action == "give_up":
            return

        map_path = self._semantic_map_path
        if not map_path or not os.path.exists(map_path):
            self._log_stage_warn(
                "RECOVERY/MAP",
                f"semantic_map_path='{map_path}' not found; "
                "skipping map update.",
            )
            return

        try:
            result = write_displaced_semistatic_map(
                map_path=map_path,
                object_key=object_key,
                object_state=object_state,
            )
        except Exception as exc:
            self._log_stage_error(
                "RECOVERY/MAP",
                f"write_displaced_semistatic_map raised: {exc}",
            )
            return

        if result is None:
            self._log_stage_info(
                "RECOVERY/MAP",
                f"No map update written for object_key='{object_key}' "
                f"(object not found or state mismatch).",
            )
            return

        version_str = os.path.splitext(os.path.basename(result.new_map_path))[0]
        self._log_stage_info(
            "RECOVERY/MAP",
            f"Wrote displaced map: {result.new_map_path} "
            f"(displaced_object_key='{result.displaced_object_key}', "
            f"previous='{result.previous_map_path}')",
        )

        extent_m = float(blockage_extent_m) if blockage_extent_m > 0 else 1.0

        store_msg = SemanticStoreUpdated()
        store_msg.semantic_map_uri = result.new_map_path
        store_msg.semantic_map_version = version_str
        store_msg.displaced_object_key = result.displaced_object_key
        store_msg.reason = (
            "suspected_semi_static_displacement_from_inferred_blockage"
        )
        store_msg.update_center.x = float(blockage_centroid.x)
        store_msg.update_center.y = float(blockage_centroid.y)
        store_msg.update_center.z = float(blockage_centroid.z)
        store_msg.update_radius_m = extent_m
        self._semantic_store_updated_pub.publish(store_msg)

        self._log_stage_info(
            "RECOVERY/MAP",
            f"Published /semantic_store_updated "
            f"(semantic_map_version='{version_str}').",
        )

        now_stamp = self.get_clock().now().to_msg()
        report = SemanticCorrectionReport()
        report.header.stamp = now_stamp
        report.header.frame_id = "map"
        report.environment_id = self._environment_id
        report.semantic_map_id = self._semantic_map_id
        report.base_map_version = self._semantic_map_version
        report.object_key = result.displaced_object_key
        report.object_tag = object_tag
        report.previous_object_state = object_state
        report.correction_type = "suspected_displacement"
        report.responsible_match_type = responsible_match_type
        report.confidence = 0.7
        report.evidence_center.x = float(blockage_centroid.x)
        report.evidence_center.y = float(blockage_centroid.y)
        report.evidence_center.z = float(blockage_centroid.z)
        report.evidence_radius_m = extent_m
        if robot_pose is not None:
            report.robot_pose = robot_pose
        report.frame_id = "map"
        report.recovery_event_id = recovery_event_id
        report.directive_action = directive_action
        report.reason = "suspected_semi_static_displacement_from_inferred_blockage"
        report.observed_at = now_stamp
        report.evidence_source = "bt_recovery"
        self._correction_report_pub.publish(report)

        self._log_stage_info(
            "RECOVERY/MAP",
            f"Published /semantic_map/corrections "
            f"(object_key='{result.displaced_object_key}', "
            f"recovery_event_id='{recovery_event_id}').",
        )

    def _handle_request_recovery(
        self,
        request: RequestRecovery.Request,
        response: RequestRecovery.Response,
    ) -> RequestRecovery.Response:
        trigger = self._build_trigger_from_request(request)

        recovery_event_id = str(uuid.uuid4())

        if len(self._attempt_records) >= int(self._recovery_cap):
            terminal = build_give_up_directive(
                DirectiveLLMProposal(
                    action="give_up",
                    rationale="BT-led recovery retry cap reached.",
                    confidence_percent=100,
                ),
                ProposalContext(
                    attempts_used=len(self._attempt_records),
                    retry_cap=int(self._recovery_cap),
                    responsible_safety_class=(
                        trigger.responsible_safety_class or "none"
                    ),
                    responsible_object_state=(
                        trigger.responsible_object_state or ""
                    ),
                    recovery_event_id=recovery_event_id,
                ),
                overrides=OverrideConfig(
                    signal_attempts_default=self._signal_attempts_default,
                    short_signal_wait_seconds=self._short_signal_wait_seconds,
                    passive_wait_seconds_default=self._passive_wait_seconds_default,
                ),
            )

            self._record_bt_directive_attempt(
                directive=terminal,
                proposal=None,
                trigger=trigger,
            )

            return self._fill_request_recovery_response_from_directive(
                response=response,
                status="terminal_fail",
                message="BT-led recovery retry cap reached.",
                directive=terminal,
            )

        status = self._arbitrate_bt_recovery_request(trigger)

        if status != "accepted":
            if status == "duplicate":
                message = "Duplicate BT-led recovery request absorbed."
            elif status == "already_in_recovery":
                message = "BT-led recovery directive already in progress."
            elif status == "rejected":
                message = "BT-led recovery request rejected."
            else:
                message = f"BT-led recovery request status='{status}'."

            return self._fill_empty_request_recovery_response(
                response=response,
                status=status,
                message=message,
                recovery_event_id=recovery_event_id,
            )

        self._bt_directive_in_progress = True
        self._publish_recovery_status(
            "BT_RECOVERY_DIRECTIVE_IN_PROGRESS",
            reason=trigger.trigger_source,
        )

        try:
            # M4 (spec 21.3): no closed-door deterministic short-circuit. A
            # closed door yields >=2 eligible actions and the LLM selects; the
            # deterministic layer only filters + overrides invalid picks.
            proposal = self._call_propose_recovery_for_bt_request(
                request=request,
                trigger=trigger,
            )

            if proposal is None:
                fallback = build_give_up_directive(
                    DirectiveLLMProposal(
                        action="give_up",
                        rationale="ProposeRecovery service call failed.",
                        confidence_percent=0,
                    ),
                    self._build_bt_proposal_context(
                        trigger=trigger,
                        recovery_event_id=recovery_event_id,
                    ),
                    overrides=OverrideConfig(
                        signal_attempts_default=self._signal_attempts_default,
                        short_signal_wait_seconds=self._short_signal_wait_seconds,
                        passive_wait_seconds_default=self._passive_wait_seconds_default,
                    ),
                )

                self._record_bt_directive_attempt(
                    directive=fallback,
                    proposal=None,
                    trigger=trigger,
                )

                return self._fill_request_recovery_response_from_directive(
                    response=response,
                    status="terminal_fail",
                    message="ProposeRecovery service call failed.",
                    directive=fallback,
                )

            directive = self._build_directive_from_bt_proposal(
                proposal=proposal,
                request=request,
                trigger=trigger,
                recovery_event_id=recovery_event_id,
            )

            self._record_bt_directive_attempt(
                directive=directive,
                proposal=proposal,
                trigger=trigger,
            )

            self._maybe_update_semantic_map(
                object_key=trigger.responsible_object_key,
                object_tag=trigger.responsible_object_tag,
                object_state=trigger.responsible_object_state,
                responsible_match_type=trigger.match_type,
                responsible_openable=bool(trigger.responsible_openable),
                responsible_clearable=bool(trigger.responsible_clearable),
                directive_action=directive.action,
                blockage_centroid=trigger.blockage_centroid,
                blockage_extent_m=trigger.blockage_extent_m,
                robot_pose=trigger.robot_pose,
                recovery_event_id=recovery_event_id,
            )

             # An accepted give_up is still an accepted directive. The BT plugin
            # returns SUCCESS for accepted give_up so the XML Switch3 reaches the
            # visible ForceFailure branch. terminal_fail is reserved for service
            # failure/cap exhaustion paths above.
            response_status = "accepted"

            message = (
                f"BT-led recovery directive issued: action='{directive.action}'."
            )

            self._log_stage_info(
                "RECOVERY/BT",
                (
                    f"{message} "
                    f"attempts_used={len(self._attempt_records)}, "
                    f"retry_cap={self._recovery_cap}, "
                    f"event_id='{recovery_event_id}'"
                ),
            )

            return self._fill_request_recovery_response_from_directive(
                response=response,
                status=response_status,
                message=message,
                directive=directive,
            )

        finally:
            self._bt_directive_in_progress = False
            self._publish_recovery_status(
                "BT_RECOVERY_DIRECTIVE_READY",
                reason=recovery_event_id,
            )

    @staticmethod
    def _object_instance_to_candidate(msg) -> ObjectCandidate:
        return ObjectCandidate(
            object_key=str(msg.object_key or ""),
            object_tag=str(msg.object_tag or ""),
            object_state=str(msg.object_state or ""),
            safety_class=str(msg.safety_class or "none"),
            openable=bool(msg.openable),
            clearable=bool(msg.clearable),
            bbox_center=(
                float(msg.bbox_center.x),
                float(msg.bbox_center.y),
                float(msg.bbox_center.z),
            ),
            bbox_extent=(
                float(msg.bbox_extent.x),
                float(msg.bbox_extent.y),
                float(msg.bbox_extent.z),
            ),
            state_detail=str(
                getattr(msg, "state_detail", "") or ""
            ),
            traversability=str(
                getattr(msg, "traversability", "") or ""
            ),
        )

    @staticmethod
    def _point_tuple_from_msg(point: Point) -> Tuple[float, float, float]:
        return (
            float(point.x),
            float(point.y),
            float(point.z),
        )

    def _handle_match_responsible_object(
        self,
        request: MatchResponsibleObject.Request,
        response: MatchResponsibleObject.Response,
    ) -> MatchResponsibleObject.Response:
        candidates = []

        for obj_msg in request.objects:
            try:
                candidate = self._object_instance_to_candidate(obj_msg)
            except Exception as exc:
                self._log_stage_warn(
                    "RECOVERY/OBJECT",
                    (
                        "Skipping malformed ObjectInstance in "
                        f"/match_responsible_object request: {exc}"
                    ),
                )
                continue

            if not candidate.object_key:
                self._log_stage_warn(
                    "RECOVERY/OBJECT",
                    (
                        "Skipping ObjectInstance with empty object_key in "
                        "/match_responsible_object request."
                    ),
                )
                continue

            candidates.append(candidate)

        result = match_responsible_object(
            blockage_centroid=self._point_tuple_from_msg(
                request.blockage_centroid
            ),
            blockage_extent_m=float(request.blockage_extent_m),
            candidates=candidates,
            inferred_fallback_radius_m=float(self._nearest_fallback_radius_m),
        )

        response.success = bool(result.success)
        response.match_type = result.match_type
        response.responsible_object_key = result.responsible_object_key
        response.responsible_object_tag = result.responsible_object_tag
        response.responsible_object_state = result.responsible_object_state
        response.safety_class = result.safety_class
        response.openable = bool(result.openable)
        response.clearable = bool(result.clearable)

        response.bbox_center.x = float(result.bbox_center[0])
        response.bbox_center.y = float(result.bbox_center[1])
        response.bbox_center.z = float(result.bbox_center[2])

        response.bbox_extent.x = float(result.bbox_extent[0])
        response.bbox_extent.y = float(result.bbox_extent[1])
        response.bbox_extent.z = float(result.bbox_extent[2])

        response.state_detail = result.state_detail
        response.traversability = result.traversability
        response.message = result.message

        self._log_stage_info(
            "RECOVERY/OBJECT",
            (
                f"/match_responsible_object: "
                f"candidates={len(candidates)}, "
                f"success={response.success}, "
                f"match_type='{response.match_type}', "
                f"responsible_object_key='{response.responsible_object_key}', "
                f"tag='{response.responsible_object_tag}', "
                f"state='{response.responsible_object_state}', "
                f"safety_class='{response.safety_class}', "
                f"openable={response.openable}, "
                f"clearable={response.clearable}, "
                f"message='{response.message}'"
            ),
        )

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

    # Everything at and after --ros-args is ROS infrastructure (remaps, params-file
    # paths, etc.).  Truncate before collecting positionals so that values like
    # /tmp/launch_params_xyz are never mistaken for a navigation query.
    if '--ros-args' in argv:
        argv = argv[:argv.index('--ros-args')]

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

    executor = MultiThreadedExecutor(num_threads=4)
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