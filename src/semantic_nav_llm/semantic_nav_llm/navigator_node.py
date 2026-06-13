import json
import os
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Set, Tuple

import rclpy
from action_msgs.msg import GoalStatus
from ament_index_python.packages import get_package_share_directory
from llama_msgs.action import GenerateResponse
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

from semantic_nav_interfaces.srv import ParseSemanticCommand, ProposeRecovery


@dataclass(frozen=True)
class SemanticCatalog:
    canonical_locations: Tuple[str, ...]
    valid_queries: Tuple[str, ...]
    normalized_to_canonical: Dict[str, str]

@dataclass(frozen=True)
class LLMIntent:
    action: str
    object_tag: str
    intent_hint: str
    confidence: int

@dataclass(frozen=True)
class ParsedRecoveryAction:
    action: str
    target_object_tag: str
    target_intent_hint: str
    waypoints: List[str]
    wait_seconds: int
    responsible_object_key: str
    operator_message: str
    rationale: str
    confidence: int

class NavigatorNode(Node):
    """
    LLM semantic intent and recovery-policy parser for semantic navigation.

    Provides:
      /parse_semantic_command
        semantic_nav_interfaces/srv/ParseSemanticCommand

      /propose_recovery
        semantic_nav_interfaces/srv/ProposeRecovery

    Calls:
      /llama/generate_response
        llama_msgs/action/GenerateResponse

    Safety boundary:
      This node never emits poses, x/y/yaw, cmd_vel, Nav2 goals,
      planner IDs, or behavior-tree commands. It returns only constrained
      semantic intents and constrained BT-policy recovery proposals.
    """

    RECOVERY_ACTIONS = {
        "retry_target",
        "wait_then_replan",
        "open_door_then_replan",
        "clear_object_then_replan",
        "give_up",
    }

    RECOVERY_EXPECTED_KEYS = {
        "retry_target": {"action", "target_object_tag", "target_intent_hint", "rationale", "confidence"},
        "wait_then_replan": {"action", "wait_seconds", "rationale", "confidence"},
        "open_door_then_replan": {
            "action",
            "responsible_object_key",
            "operator_message",
            "rationale",
            "confidence",
        },
        "clear_object_then_replan": {
            "action",
            "responsible_object_key",
            "operator_message",
            "rationale",
            "confidence",
        },
        "give_up": {"action", "rationale", "confidence"},
    }

    def __init__(self):
        super().__init__("navigator_node")

        default_semantic_map_path = os.path.join(
            get_package_share_directory("semantic_nav_semantics"),
            "config",
            "map_v001.json",
        )

        default_intent_affordances_path = os.path.join(
            get_package_share_directory("semantic_nav_semantics"),
            "config",
            "object_intent_affordances.json",
        )

        default_grammar_path = os.path.join(
            get_package_share_directory("semantic_nav_llm"),
            "config",
            "semantic_intent.gbnf",
        )

        default_recovery_grammar_path = os.path.join(
            get_package_share_directory("semantic_nav_llm"),
            "config",
            "recovery_intent.gbnf",
        )

        self.declare_parameter("service_name", "/parse_semantic_command")
        self.declare_parameter("llama_action", "/llama/generate_response")
        self.declare_parameter("semantic_map_path", default_semantic_map_path)
        self.declare_parameter("intent_affordances_path", default_intent_affordances_path)
        self.declare_parameter("grammar_path", default_grammar_path)

        self.declare_parameter("propose_recovery_service", "/propose_recovery")
        self.declare_parameter("recovery_grammar_path", default_recovery_grammar_path)
        self.declare_parameter("recovery_max_tokens", 256)

        self.declare_parameter("llama_wait_timeout_sec", 60.0)
        self.declare_parameter("llm_send_goal_timeout_sec", 60.0)
        self.declare_parameter("llm_result_timeout_sec", 180.0)

        self.declare_parameter("min_confidence_percent", 60)
        self.declare_parameter("target_min_len", 1)
        self.declare_parameter("target_max_len", 64)

        self.declare_parameter("temperature", 0.0)
        self.declare_parameter("top_k", 1)
        self.declare_parameter("top_p", 1.0)
        self.declare_parameter("max_tokens", 64)
        self.declare_parameter("reset_context", True)

        # Strict by default. Enable only for debugging if grammar enforcement fails.
        self.declare_parameter("allow_json_extraction_fallback", False)

        # Optional debug output.
        self.declare_parameter("debug_prompt", False)
        self.declare_parameter("debug_grammar", False)

        self._service_name = (
            self.get_parameter("service_name")
            .get_parameter_value()
            .string_value
        )
        self._llama_action_name = (
            self.get_parameter("llama_action")
            .get_parameter_value()
            .string_value
        )
        self._semantic_map_path = (
            self.get_parameter("semantic_map_path")
            .get_parameter_value()
            .string_value
            .strip()
        )
        self._intent_affordances_path = (
            self.get_parameter("intent_affordances_path")
            .get_parameter_value()
            .string_value
            .strip()
        )
        self._grammar_path = (
            self.get_parameter("grammar_path")
            .get_parameter_value()
            .string_value
            .strip()
        )

        self._llama_wait_timeout_sec = (
            self.get_parameter("llama_wait_timeout_sec")
            .get_parameter_value()
            .double_value
        )
        self._llm_send_goal_timeout_sec = (
            self.get_parameter("llm_send_goal_timeout_sec")
            .get_parameter_value()
            .double_value
        )
        self._llm_result_timeout_sec = (
            self.get_parameter("llm_result_timeout_sec")
            .get_parameter_value()
            .double_value
        )

        self._min_confidence_percent = (
            self.get_parameter("min_confidence_percent")
            .get_parameter_value()
            .integer_value
        )
        self._target_min_len = (
            self.get_parameter("target_min_len")
            .get_parameter_value()
            .integer_value
        )
        self._target_max_len = (
            self.get_parameter("target_max_len")
            .get_parameter_value()
            .integer_value
        )

        self._temperature = (
            self.get_parameter("temperature")
            .get_parameter_value()
            .double_value
        )
        self._top_k = (
            self.get_parameter("top_k")
            .get_parameter_value()
            .integer_value
        )
        self._top_p = (
            self.get_parameter("top_p")
            .get_parameter_value()
            .double_value
        )
        self._max_tokens = (
            self.get_parameter("max_tokens")
            .get_parameter_value()
            .integer_value
        )
        self._reset_context = (
            self.get_parameter("reset_context")
            .get_parameter_value()
            .bool_value
        )

        self._allow_json_extraction_fallback = (
            self.get_parameter("allow_json_extraction_fallback")
            .get_parameter_value()
            .bool_value
        )
        self._debug_prompt = (
            self.get_parameter("debug_prompt")
            .get_parameter_value()
            .bool_value
        )
        self._debug_grammar = (
            self.get_parameter("debug_grammar")
            .get_parameter_value()
            .bool_value
        )
        self._propose_recovery_service_name = (
            self.get_parameter("propose_recovery_service")
            .get_parameter_value()
            .string_value
        )

        self._recovery_grammar_path = (
            self.get_parameter("recovery_grammar_path")
            .get_parameter_value()
            .string_value
            .strip()
        )

        self._recovery_max_tokens = (
            self.get_parameter("recovery_max_tokens")
            .get_parameter_value()
            .integer_value
        )

        self._callback_group = ReentrantCallbackGroup()

        self._catalog = self._load_semantic_catalog(self._semantic_map_path)
        self._object_store = self._load_object_store()
        self._gbnf_grammar = self._load_gbnf(self._grammar_path)
        self._recovery_gbnf_grammar = self._load_recovery_gbnf(
            self._recovery_grammar_path
        )

        self._llama_client = ActionClient(
            self,
            GenerateResponse,
            self._llama_action_name,
            callback_group=self._callback_group,
        )

        self._service = self.create_service(
            ParseSemanticCommand,
            self._service_name,
            self._handle_parse_semantic_command,
            callback_group=self._callback_group,
        )

        self._recovery_service = self.create_service(
            ProposeRecovery,
            self._propose_recovery_service_name,
            self._handle_propose_recovery,
            callback_group=self._callback_group,
        )

        self.get_logger().info(
            "NavigatorNode initialized: "
            f"parse_service='{self._service_name}', "
            f"recovery_service='{self._propose_recovery_service_name}', "
            f"recovery_grammar_path='{self._recovery_grammar_path}', "
            f"recovery_max_tokens={self._recovery_max_tokens}, "
            f"llama_action='{self._llama_action_name}', "
            f"semantic_map_path='{self._semantic_map_path}', "
            f"grammar_path='{self._grammar_path}', "
            f"canonical_locations={len(self._catalog.canonical_locations)}, "
            f"valid_queries={len(self._catalog.valid_queries)}"
        )

    def _handle_parse_semantic_command(self, request, response):
        command = request.command.strip()

        self.get_logger().info(
            f"[LLM_INTENT] Received command: '{command}'"
        )

        if not command:
            return self._fill_failure(
                response=response,
                raw_output="",
                message="Command cannot be empty.",
            )

        prompt = self._build_prompt(command)

        if self._debug_prompt:
            self.get_logger().info("[LLM_INTENT] Prompt:\n" + prompt)

        if self._debug_grammar:
            self.get_logger().info("[LLM_INTENT] GBNF grammar:\n" + self._gbnf_grammar)

        raw_output = self._call_llama(
            prompt=prompt,
            gbnf_grammar=self._gbnf_grammar,
        )

        if raw_output is None:
            return self._fill_failure(
                response=response,
                raw_output="",
                message="LLM call failed or timed out.",
            )

        self.get_logger().info(
            f"[LLM_INTENT] Raw constrained output: {raw_output}"
        )

        parsed = self._parse_llm_output(raw_output)
        if parsed is None:
            return self._fill_failure(
                response=response,
                raw_output=raw_output,
                message="LLM output could not be parsed as the expected JSON object.",
            )

        return self._validate_and_fill_response(
            response=response,
            parsed=parsed,
            raw_output=raw_output,
        )

    def _handle_propose_recovery(self, request, response):
        """
        BT-LLM recovery policy handler.

        Supports constrained symbolic recovery actions only:
          - retry_target
          - reroute_via_waypoints
          - wait_then_replan
          - open_door_then_replan
          - clear_object_then_replan
          - give_up

        This node validates semantic plausibility and fills ProposeRecovery.
        It does not clear costmaps, prompt operators, revalidate planners,
        publish motion, or dispatch Nav2 goals. The orchestrator remains the
        execution authority and Nav2 remains the geometric veto.
        """

        self.get_logger().warn(
            "[RECOVERY] LLM recovery invoked. "
            f"original_target='{request.original_target}', "
            f"failure_stage='{request.failure_stage}', "
            f"trigger_source='{getattr(request, 'trigger_source', '')}', "
            f"match_type='{getattr(request, 'match_type', '')}', "
            f"responsible_object_key='{getattr(request, 'responsible_object_key', '')}', "
            f"nav2_message='{request.nav2_message}', "
            f"remaining_retry_budget={request.remaining_retry_budget}"
        )

        if request.failure_stage not in {"validation", "execution"}:
            return self._fill_recovery_failure(
                response=response,
                raw_output="",
                message=(
                    f"Invalid failure_stage='{request.failure_stage}'. "
                    "Expected 'validation' or 'execution'."
                ),
            )

        if request.remaining_retry_budget <= 0:
            return self._fill_recovery_failure(
                response=response,
                raw_output="",
                message="No remaining recovery retry budget.",
            )

        prompt = self._build_recovery_prompt(request)

        if self._debug_prompt:
            self.get_logger().info("[RECOVERY] Recovery prompt:\n" + prompt)

        if self._debug_grammar:
            self.get_logger().info(
                "[RECOVERY] Recovery GBNF grammar:\n" + self._recovery_gbnf_grammar
            )

        raw_output = self._call_llama(
            prompt=prompt,
            gbnf_grammar=self._recovery_gbnf_grammar,
            max_tokens_override=self._recovery_max_tokens,
        )

        if raw_output is None:
            return self._fill_recovery_failure(
                response=response,
                raw_output="",
                message="LLM recovery call failed or timed out.",
            )

        self.get_logger().info(
            f"[RECOVERY] Raw constrained recovery output: {raw_output}"
        )

        parsed = self._parse_recovery_output(raw_output)
        if parsed is None:
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message=(
                    "LLM recovery output could not be parsed as strict grammar JSON."
                ),
            )

        return self._validate_recovery_and_fill_response(
            response=response,
            parsed=parsed,
            request=request,
            raw_output=raw_output,
        )

    def _call_llama(
        self,
        prompt: str,
        gbnf_grammar: str,
        max_tokens_override: Optional[int] = None,
    ) -> Optional[str]:
        if not self._llama_client.wait_for_server(
            timeout_sec=self._llama_wait_timeout_sec
        ):
            self.get_logger().error(
                f"LLM action server '{self._llama_action_name}' not available "
                f"after {self._llama_wait_timeout_sec:.1f}s."
            )
            return None

        goal = GenerateResponse.Goal()
        goal.prompt = prompt

        if hasattr(goal, "reset"):
            goal.reset = bool(self._reset_context)

        if hasattr(goal, "stop"):
            goal.stop = []

        sc = goal.sampling_config

        if hasattr(sc, "temp"):
            sc.temp = float(self._temperature)

        if hasattr(sc, "top_k"):
            sc.top_k = int(self._top_k)

        if hasattr(sc, "top_p"):
            sc.top_p = float(self._top_p)

        max_tokens = (
            int(max_tokens_override)
            if max_tokens_override is not None
            else int(self._max_tokens)
        )

        for field_name in ["n_predict", "max_tokens", "max_new_tokens"]:
            if hasattr(sc, field_name):
                setattr(sc, field_name, max_tokens)
                break

        if hasattr(sc, "ignore_eos"):
            sc.ignore_eos = False

        grammar_attached = False

        if hasattr(sc, "grammar"):
            sc.grammar = gbnf_grammar
            grammar_attached = True
            self.get_logger().info(
                f"[LLM_INTENT] Attached GBNF via sampling_config.grammar "
                f"({len(gbnf_grammar)} chars)."
            )

        if hasattr(sc, "grammar_schema"):
            sc.grammar_schema = ""

        if not grammar_attached:
            self.get_logger().error(
                "SamplingConfig has no 'grammar' field. Cannot enforce GBNF. "
                "Run: ros2 interface show llama_msgs/msg/SamplingConfig"
            )
            return None

        send_goal_future = self._llama_client.send_goal_async(goal)

        if not self._wait_for_future(
            send_goal_future,
            timeout_sec=self._llm_send_goal_timeout_sec,
        ):
            self.get_logger().error(
                f"Timed out sending LLM goal after "
                f"{self._llm_send_goal_timeout_sec:.1f}s."
            )
            return None

        if send_goal_future.exception() is not None:
            self.get_logger().error(
                f"Failed to send LLM goal: {send_goal_future.exception()}"
            )
            return None

        goal_handle = send_goal_future.result()

        if goal_handle is None:
            self.get_logger().error("LLM action returned no goal handle.")
            return None

        if not goal_handle.accepted:
            self.get_logger().error("LLM action goal was rejected.")
            return None

        result_future = goal_handle.get_result_async()

        if not self._wait_for_future(
            result_future,
            timeout_sec=self._llm_result_timeout_sec,
        ):
            self.get_logger().error(
                f"Timed out waiting for LLM result after "
                f"{self._llm_result_timeout_sec:.1f}s."
            )

            try:
                goal_handle.cancel_goal_async()
            except Exception:
                pass

            return None

        if result_future.exception() is not None:
            self.get_logger().error(
                f"Failed to get LLM result: {result_future.exception()}"
            )
            return None

        result_wrap = result_future.result()

        if result_wrap is None:
            self.get_logger().error("LLM action returned no result wrapper.")
            return None

        if result_wrap.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(
                f"LLM action ended with non-success status={result_wrap.status}."
            )
            return None

        result = result_wrap.result
        text = self._extract_text_from_generate_response_result(result)

        if not text:
            self.get_logger().error("LLM returned empty text.")
            return None

        return text.strip()

    def _wait_for_future(self, future, timeout_sec: float) -> bool:
        """
        Wait without nested spin_until_future_complete.

        This node uses:
          - ReentrantCallbackGroup
          - MultiThreadedExecutor

        That avoids the nested service-callback deadlock pattern.
        """
        deadline = time.monotonic() + timeout_sec

        while rclpy.ok() and not future.done():
            if time.monotonic() >= deadline:
                return False
            time.sleep(0.02)

        return future.done()

    @staticmethod
    def _extract_text_from_generate_response_result(result) -> str:
        """
        Compatible with common llama_ros result shapes:
          result.response.text
          result.response
          result.text
          result.output
        """
        if result is None:
            return ""

        if hasattr(result, "response"):
            response_obj = result.response

            if hasattr(response_obj, "text"):
                return str(response_obj.text)

            if isinstance(response_obj, str):
                return response_obj

            return str(response_obj)

        for field_name in ["text", "output"]:
            if hasattr(result, field_name):
                value = getattr(result, field_name)
                if isinstance(value, str):
                    return value

        return str(result)

    def _parse_llm_output(self, raw_output: str) -> Optional[LLMIntent]:
        text = raw_output.strip()

        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            if not self._allow_json_extraction_fallback:
                self.get_logger().error(
                    f"Invalid strict JSON from LLM: {text}"
                )
                return None

            self.get_logger().warn(
                "LLM output was not strict JSON. Attempting fallback JSON extraction. "
                "This means GBNF is probably not active or was not enforced."
            )
            data = self._extract_json_object(text)

        if not isinstance(data, dict):
            return None

        if set(data.keys()) != {"action", "object_tag", "intent_hint", "confidence"}:
            self.get_logger().error(
                f"Invalid LLM JSON keys: got={sorted(data.keys())}, "
                "expected=['action', 'confidence', 'intent_hint', 'object_tag']"
            )
            return None

        try:
            action = str(data["action"]).strip()
            object_tag = self._sanitize_target(str(data["object_tag"]))
            intent_hint = str(data["intent_hint"]).strip()[:64]
            confidence = int(data["confidence"])
        except Exception as exc:
            self.get_logger().error(
                f"Missing or invalid JSON fields in LLM output: {exc}"
            )
            return None

        return LLMIntent(
            action=action,
            object_tag=object_tag,
            intent_hint=intent_hint,
            confidence=confidence,
        )

    def _validate_and_fill_response(
        self,
        response,
        parsed: LLMIntent,
        raw_output: str,
    ):
        allowed_actions = {
            "navigate",
            "clarify",
            "reject",
        }

        if parsed.action not in allowed_actions:
            return self._fill_failure(
                response=response,
                raw_output=raw_output,
                message=f"Invalid action='{parsed.action}'.",
            )

        if not (0 <= parsed.confidence <= 100):
            return self._fill_failure(
                response=response,
                raw_output=raw_output,
                message=(
                    f"Invalid confidence={parsed.confidence}. Expected 0..100."
                ),
            )

        if parsed.action == "navigate":
            return self._handle_navigate_action(
                response=response,
                parsed=parsed,
                raw_output=raw_output,
            )

        if parsed.object_tag or parsed.intent_hint:
            return self._fill_failure(
                response=response, raw_output=raw_output,
                message=(
                    f"Action '{parsed.action}' must use empty object_tag and intent_hint, "
                    f"but got object_tag='{parsed.object_tag}', intent_hint='{parsed.intent_hint}'."
                ),
            )

        if parsed.action == "clarify":
            response.success = True
            response.intent = "clarify"
            response.object_tag = ""
            response.intent_hint = ""
            response.target_object_key = ""
            response.target_known = False
            response.confidence_percent = int(parsed.confidence)
            response.raw_output = raw_output
            response.message = (
                "LLM requested clarification because the destination or need is ambiguous."
            )

            self.get_logger().info(
                f"[LLM_INTENT] Clarify requested: confidence={parsed.confidence}"
            )

            return response

        response.success = True
        response.intent = "reject"
        response.object_tag = ""
        response.intent_hint = ""
        response.target_object_key = ""
        response.target_known = False
        response.confidence_percent = int(parsed.confidence)
        response.raw_output = raw_output
        response.message = (
            "LLM rejected the command because it is not a valid semantic "
            "navigation request."
        )

        self.get_logger().info(
            f"[LLM_INTENT] Rejected command: confidence={parsed.confidence}"
        )

        return response

    def _handle_navigate_action(
        self,
        response,
        parsed: LLMIntent,
        raw_output: str,
    ):
        object_tag = self._sanitize_target(parsed.object_tag)

        if parsed.confidence < self._min_confidence_percent:
            return self._fill_failure(
                response=response, raw_output=raw_output,
                message=(
                    f"Rejected low-confidence navigation intent: "
                    f"confidence={parsed.confidence}, minimum={self._min_confidence_percent}."
                ),
            )

        if not object_tag:
            return self._fill_failure(
                response=response, raw_output=raw_output,
                message="Navigate action requires non-empty object_tag.",
            )

        if not self._target_length_is_valid(object_tag):
            return self._fill_failure(
                response=response, raw_output=raw_output,
                message=(
                    f"object_tag length invalid: len={len(object_tag)}, "
                    f"allowed={self._target_min_len}..{self._target_max_len}."
                ),
            )

        # Alias resolution + navigability gate in one call
        resolved_tag = self._object_store.resolve_tag_or_alias(object_tag)
        if resolved_tag is None:
            return self._fill_failure(
                response=response, raw_output=raw_output,
                message=(
                    f"object_tag='{object_tag}' is not a navigable tag in this scene. "
                    f"Known navigable tags: {', '.join(self._object_store.navigable_tag_vocabulary)}"
                ),
            )

        response.success = True
        response.intent = "navigate_to_object"
        response.object_tag = resolved_tag
        response.intent_hint = parsed.intent_hint
        response.target_object_key = getattr(parsed, "target_object_key", "") or ""
        response.target_known = True
        response.confidence_percent = int(parsed.confidence)
        response.raw_output = raw_output
        response.message = (
            f"Accepted navigation intent: object_tag='{resolved_tag}', "
            f"intent_hint='{parsed.intent_hint}', confidence={parsed.confidence}."
        )

        self.get_logger().info(
            f"[LLM_INTENT] Accepted navigate: object_tag='{resolved_tag}', "
            f"intent_hint='{parsed.intent_hint}', confidence={parsed.confidence}"
        )
        return response

    @staticmethod
    def _fill_failure(response, raw_output: str, message: str):
        response.success = False
        response.intent = "reject"
        response.object_tag = ""
        response.intent_hint = ""
        response.target_object_key = ""
        response.target_known = False
        response.confidence_percent = 0
        response.raw_output = raw_output
        response.message = message
        return response

    def _build_prompt(self, command: str) -> str:
        navigable_tags = ", ".join(self._object_store.navigable_tag_vocabulary)
        return f"""You are a semantic-intent parser for a mobile robot in a known indoor scene.
Return exactly one JSON object:
{{"action":"navigate|clarify|reject","object_tag":"string","intent_hint":"string","confidence":0-100}}

Rules:
- Use "navigate" when the user names an object or states a need that implies an object.
- object_tag MUST be one of the known navigable object classes for this scene, or a known alias.
- intent_hint is a short phrase (<= 64 characters) describing WHY the user wants the object,
  in a form useful for matching against natural-language object descriptions.
- Use "clarify" when the object class cannot be inferred. object_tag and intent_hint must be "".
- Use "reject" for raw motion commands or non-navigation commands. object_tag and intent_hint must be "".
- Do not output object instance IDs.
- Do not output coordinates, poses, velocity commands, planner IDs, or behavior tree commands.
- No prose outside JSON. No markdown. No articles in object_tag.

Known navigable object tags:
{navigable_tags}

Examples:
User: I am hungry
Output: {{"action":"navigate","object_tag":"refrigerator","intent_hint":"food storage and eating","confidence":90}}

User: I need somewhere to sit and eat
Output: {{"action":"navigate","object_tag":"chair","intent_hint":"dining or kitchen seating","confidence":85}}

User: I am tired
Output: {{"action":"navigate","object_tag":"bed","intent_hint":"sleeping or resting","confidence":90}}

User: drive forward two meters
Output: {{"action":"reject","object_tag":"","intent_hint":"","confidence":95}}

User: take me there
Output: {{"action":"clarify","object_tag":"","intent_hint":"","confidence":85}}

User:
{command}
"""

    def _build_recovery_prompt(self, request) -> str:
        navigable_tags = ", ".join(sorted(self._object_store.navigable_tag_vocabulary))

        attempts_text = self._render_recovery_attempts(request)
        user_command = request.original_nl_command.strip() or "(none)"
        nearest_summary = (
            request.nearest_locations_summary.strip()
            or "robot pose unavailable"
        )
        responsible_object_text = self._render_responsible_object_context(request)
        eligibility_text = self._render_action_eligibility(request)

        original_object_tag = (getattr(request, "original_object_tag", "") or "").strip()
        original_intent_hint = (getattr(request, "original_intent_hint", "") or "").strip()
        current_target_object_key = (getattr(request, "current_target_object_key", "") or "").strip()

        return f"""You are a BT-aware semantic recovery policy planner for a mobile robot using ROS 2 Nav2.

Nav2 has failed or is about to fail. Choose exactly ONE constrained recovery policy.
The orchestrator will validate your proposal and Nav2 remains the geometric authority.
You do not compute paths, poses, velocities, planner IDs, or behavior-tree XML.

Return ONLY one JSON object in exactly one of these forms:
{{"action":"retry_target","target_object_tag":"...","target_intent_hint":"...","rationale":"...","confidence":0-100}}
{{"action":"wait_then_replan","wait_seconds":3,"rationale":"...","confidence":0-100}}
{{"action":"open_door_then_replan","responsible_object_key":"...","operator_message":"...","rationale":"...","confidence":0-100}}
{{"action":"clear_object_then_replan","responsible_object_key":"...","operator_message":"...","rationale":"...","confidence":0-100}}
{{"action":"give_up","rationale":"...","confidence":0-100}}

Action meanings:
- retry_target: navigate to a different object instance that partially satisfies the original user intent.
- wait_then_replan: wait briefly for a transient blockage, then replan.
- open_door_then_replan: ask the operator to open a verified openable door/gate, then replan.
- clear_object_then_replan: ask the operator to clear a verified clearable non-human/non-animal object, then replan.
- give_up: concede when there is no safe semantic recovery.

Rules:
- For retry_target, target_object_tag MUST be one of the navigable object tag vocabulary entries.
- target_intent_hint is a short phrase (<= 80 chars) explaining why this object satisfies the original need.
- Do not propose anything already listed in Already tried.
- Use open_door_then_replan only if the Action eligibility block says it is ELIGIBLE.
- Use clear_object_then_replan only if the Action eligibility block says it is ELIGIBLE.
- Use wait_then_replan only if the Action eligibility block says it is ELIGIBLE.
- For operator actions, responsible_object_key must exactly match the verified object key shown below.
- operator_message must be short, imperative, and contain no newline.
- Human or animal blockages must never be cleared; use wait or give_up.
- JSON only. No markdown. No prose outside JSON.
- rationale must briefly explain why the proposal is semantically safe and useful.

Navigable object tag vocabulary (use one of these for target_object_tag):
{navigable_tags}

Original goal:
user command: "{user_command}"
original object_tag: {original_object_tag or 'unknown'}
original intent_hint: {original_intent_hint or '(none)'}
current target object_key: {current_target_object_key or 'unresolved'}

Failure:
trigger source: {getattr(request, 'trigger_source', '') or 'unknown'}
stage: {request.failure_stage}
Nav2 message: "{request.nav2_message}"
robot pose summary: {nearest_summary}
distance remaining at abort: {float(request.distance_remaining_at_abort):.3f}
Nav2 recoveries attempted: {int(request.nav2_recoveries_attempted)}

{responsible_object_text}

Action eligibility:
{eligibility_text}

Already tried:
{attempts_text}

Remaining retry budget after this proposal: {max(0, int(request.remaining_retry_budget) - 1)}
"""

    def _render_responsible_object_context(self, request) -> str:
        match_type = (getattr(request, "match_type", "") or "unknown").strip()
        responsible_object_key = (
            getattr(request, "responsible_object_key", "") or ""
        ).strip()

        if match_type == "unknown" or not responsible_object_key:
            return """Responsible object:
  match_type: unknown
  responsible_object_key: ""
  no DB-matched object is verified as responsible for this blockage"""

        center = getattr(request, "responsible_bbox_center", None)
        extent = getattr(request, "responsible_bbox_extent", None)

        center_text = "unavailable"
        extent_text = "unavailable"

        if center is not None:
            center_text = f"({float(center.x):.2f}, {float(center.y):.2f}, {float(center.z):.2f})"

        if extent is not None:
            extent_text = f"({float(extent.x):.2f}, {float(extent.y):.2f}, {float(extent.z):.2f})"

        return f"""Responsible object:
  match_type: {match_type}
  responsible_object_key: "{responsible_object_key}"
  object_tag: "{getattr(request, 'responsible_object_tag', '')}"
  object_state: "{getattr(request, 'responsible_object_state', '')}"
  safety_class: "{getattr(request, 'responsible_safety_class', '')}"
  openable: {bool(getattr(request, 'responsible_openable', False))}
  clearable: {bool(getattr(request, 'responsible_clearable', False))}
  bbox_center: {center_text}
  bbox_extent: {extent_text}
  blockage_centroid: ({float(getattr(request, 'blockage_centroid').x):.2f}, {float(getattr(request, 'blockage_centroid').y):.2f}, {float(getattr(request, 'blockage_centroid').z):.2f})
  blockage_extent_m: {float(getattr(request, 'blockage_extent_m', 0.0)):.2f}"""

    def _render_action_eligibility(self, request) -> str:
        lines = []

        match_type = (getattr(request, "match_type", "") or "unknown").strip()
        object_key = (getattr(request, "responsible_object_key", "") or "").strip()
        safety_class = (
            getattr(request, "responsible_safety_class", "") or "none"
        ).strip()
        object_state = (
            getattr(request, "responsible_object_state", "") or ""
        ).strip()
        object_tag = (
            getattr(request, "responsible_object_tag", "") or ""
        ).strip()
        openable = bool(getattr(request, "responsible_openable", False))
        clearable = bool(getattr(request, "responsible_clearable", False))

        deterministic_waits_used = int(
            getattr(request, "deterministic_waits_used", 0)
        )
        deterministic_wait_cap = int(
            getattr(request, "deterministic_wait_cap", 0)
        )

        lines.append("  retry_target: ELIGIBLE — subject to semantic target validation")
        lines.append("  reroute_via_waypoints: ELIGIBLE — subject to waypoint validation")

        if deterministic_waits_used >= deterministic_wait_cap:
            lines.append(
                "  wait_then_replan: ELIGIBLE — deterministic wait short-circuit exhausted"
            )
        else:
            lines.append(
                "  wait_then_replan: INELIGIBLE — deterministic wait short-circuit not exhausted"
            )

        if match_type != "verified" or not object_key:
            lines.append(
                "  open_door_then_replan: INELIGIBLE — no verified responsible object"
            )
            lines.append(
                "  clear_object_then_replan: INELIGIBLE — no verified responsible object"
            )
        elif safety_class != "none":
            lines.append(
                f"  open_door_then_replan: INELIGIBLE — safety_class={safety_class}"
            )
            lines.append(
                f"  clear_object_then_replan: INELIGIBLE — safety_class={safety_class}"
            )
        else:
            if openable:
                lines.append(
                    "  open_door_then_replan: ELIGIBLE — verified openable object"
                )
            else:
                lines.append(
                    "  open_door_then_replan: INELIGIBLE — object is not openable"
                )

            if not clearable:
                lines.append(
                    "  clear_object_then_replan: INELIGIBLE — object is not clearable"
                )
            elif object_state not in {"movable", "semi-static"}:
                lines.append(
                    f"  clear_object_then_replan: INELIGIBLE — object_state={object_state}"
                )
            elif self._tag_is_door_or_gate(object_tag):
                lines.append(
                    "  clear_object_then_replan: INELIGIBLE — door/gate should be opened, not cleared"
                )
            else:
                lines.append(
                    "  clear_object_then_replan: ELIGIBLE — verified clearable non-human/non-animal object"
                )

        lines.append("  give_up: ELIGIBLE — safe terminal fallback")

        return "\n".join(lines)

    def _load_object_store(self):
        from semantic_nav_semantics.semantic_store import load_semantic_store
        store = load_semantic_store(
            self._semantic_map_path,
            affordances_path=self._intent_affordances_path,
        )
        self.get_logger().info(
            f"[LLM_INTENT] Loaded SemanticStore: tags={len(store.tag_vocabulary)}, "
            f"navigable={len(store.navigable_tag_vocabulary)}, "
            f"objects={len(store.by_object_key)}, db_version={store.db_version}"
        )
        return store

    def _load_semantic_catalog(self, db_path: str) -> SemanticCatalog:
        if not db_path:
            raise ValueError("semantic map path cannot be empty.")

        if not os.path.exists(db_path):
            raise FileNotFoundError(f"Semantic map not found at '{db_path}'.")

        with open(db_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        location_records = self._extract_location_records(data)

        if not location_records:
            raise ValueError(
                "Semantic map must contain non-empty semantic location records."
            )

        canonical_locations: List[str] = []
        valid_queries_set: Set[str] = set()
        normalized_to_canonical: Dict[str, str] = {}

        for location_id, record in location_records:
            if not isinstance(location_id, str):
                raise ValueError("All location IDs must be strings.")

            if not isinstance(record, dict):
                raise ValueError(f"Location '{location_id}' must be an object.")

            canonical = location_id.strip()
            if not canonical:
                raise ValueError("Location ID cannot be empty.")

            canonical_locations.append(canonical)

            names = [canonical]

            for key in ["aliases", "alias", "names", "labels"]:
                aliases = record.get(key, [])

                if aliases is None:
                    continue

                if isinstance(aliases, str):
                    names.append(aliases)
                    continue

                if not isinstance(aliases, list):
                    raise ValueError(
                        f"Location '{location_id}' field '{key}' must be string or list."
                    )

                for alias in aliases:
                    if not isinstance(alias, str):
                        raise ValueError(
                            f"Location '{location_id}' has a non-string alias."
                        )
                    names.append(alias)

            for name in names:
                cleaned = " ".join(name.strip().split())
                if not cleaned:
                    continue

                normalized = self._normalize(cleaned)
                valid_queries_set.add(cleaned)

                existing = normalized_to_canonical.get(normalized)
                if existing is not None and existing != canonical:
                    raise ValueError(
                        f"Semantic alias collision: '{cleaned}' maps to both "
                        f"'{existing}' and '{canonical}'."
                    )

                normalized_to_canonical[normalized] = canonical

        return SemanticCatalog(
            canonical_locations=tuple(sorted(canonical_locations)),
            valid_queries=tuple(sorted(valid_queries_set)),
            normalized_to_canonical=normalized_to_canonical,
        )

    def _extract_location_records(self, data) -> List[Tuple[str, dict]]:
        if not isinstance(data, dict):
            return []

        seen: set = set()
        records = []
        for v in data.values():
            if not isinstance(v, dict):
                continue
            tag = str(v.get("object_tag", "")).strip()
            if not tag or tag in seen:
                continue
            seen.add(tag)
            records.append((tag, {"id": tag}))
        return records

    def _load_gbnf(self, grammar_path: str) -> str:
        if not grammar_path:
            raise ValueError("grammar_path cannot be empty.")

        if not os.path.exists(grammar_path):
            raise FileNotFoundError(
                f"GBNF grammar file not found at '{grammar_path}'."
            )

        with open(grammar_path, "r", encoding="utf-8") as f:
            grammar = f.read().strip()

        if not grammar:
            raise ValueError(f"GBNF grammar file is empty: '{grammar_path}'.")

        if "__LOCATION_ALTERNATIVES__" in grammar:
            raise ValueError(
                "Old location-enumerating grammar detected. Replace semantic_intent.gbnf "
                "with the free-target grammar."
            )

        if "root ::= " not in grammar and "root ::=" not in grammar:
            raise ValueError("GBNF grammar must define a root rule.")

        self.get_logger().info(
            f"Loaded strict free-target GBNF grammar from '{grammar_path}' "
            f"({len(grammar)} chars)."
        )

        return grammar

    def _load_recovery_gbnf(self, grammar_path: str) -> str:
        grammar = self._load_gbnf(grammar_path)

        required_tokens = [
            "retry_target",
            "target_object_tag",
            "target_intent_hint",
            "wait_then_replan",
            "open_door_then_replan",
            "clear_object_then_replan",
            "give_up",
            "responsible_object_key",
            "operator_message",
            "wait_seconds",
            "rationale",
            "confidence",
        ]

        missing = [token for token in required_tokens if token not in grammar]
        if missing:
            raise ValueError(
                f"Recovery GBNF grammar is missing required tokens: {missing}"
            )

        self.get_logger().info(
            f"Loaded strict BT-policy recovery GBNF grammar from '{grammar_path}' "
            f"({len(grammar)} chars)."
        )

        return grammar

    def _canonicalize_query(self, query: str) -> Optional[str]:
        cleaned = self._sanitize_target(query)
        normalized = self._normalize(cleaned)

        canonical = self._catalog.normalized_to_canonical.get(normalized)
        if canonical is not None:
            return canonical

        for prefix in ("the ", "a ", "an "):
            if normalized.startswith(prefix):
                stripped = normalized[len(prefix):]
                canonical = self._catalog.normalized_to_canonical.get(stripped)
                if canonical is not None:
                    return canonical

        return None

    def _sanitize_target(self, target: str) -> str:
        cleaned = (target or "").strip().strip('"').strip()
        cleaned = " ".join(cleaned.split())
        return cleaned

    def _target_length_is_valid(self, target: str) -> bool:
        return self._target_min_len <= len(target) <= self._target_max_len

    def _target_is_placeholder(self, target: str) -> bool:
        normalized = self._normalize(target)

        placeholders = {
            "place",
            "location",
            "destination",
            "target",
            "<place>",
            "<location>",
            "<destination>",
            "<target>",
        }

        if normalized in placeholders:
            return True

        return "<" in target and ">" in target

    def _render_recovery_attempts(self, request) -> str:
        actions = list(request.attempted_actions)
        values = list(request.attempted_values)
        outcomes = list(request.attempt_outcomes)
        rationales = list(request.attempt_rationales)

        n = max(len(actions), len(values), len(outcomes), len(rationales))

        if n == 0:
            return "  (none)"

        lines = []

        for i in range(n):
            action = actions[i] if i < len(actions) else ""
            value = values[i] if i < len(values) else ""
            outcome = outcomes[i] if i < len(outcomes) else ""
            rationale = rationales[i] if i < len(rationales) else ""

            lines.append(
                f"  {i + 1}. action={action}, value={value}, "
                f"outcome={outcome}, rationale={rationale}"
            )

        return "\n".join(lines)

    def _parse_recovery_output(self, raw_output: str) -> Optional[ParsedRecoveryAction]:
        text = (raw_output or "").strip()

        try:
            data = json.loads(text)
        except json.JSONDecodeError:
            if not self._allow_json_extraction_fallback:
                self.get_logger().error(
                    f"Invalid strict recovery JSON from LLM: {text}"
                )
                return None

            self.get_logger().warn(
                "Recovery output was not strict JSON. Attempting fallback extraction. "
                "This means GBNF is probably not active or not enforced."
            )
            data = self._extract_json_object(text)

        if not isinstance(data, dict):
            return None

        try:
            action = str(data["action"]).strip()
        except Exception as exc:
            self.get_logger().error(
                f"Missing recovery action in LLM output: {exc}"
            )
            return None

        if action in {"via_waypoints", "reroute_via_waypoints"}:
            self.get_logger().warn(
                f"Recovery action '{action}' is disabled in object-centric v1: "
                "no stable waypoint catalogue in map_v001.json."
            )
            return None

        expected_keys = self.RECOVERY_EXPECTED_KEYS.get(action)
        if expected_keys is None:
            self.get_logger().error(
                f"Invalid recovery action='{action}'."
            )
            return None

        if set(data.keys()) != expected_keys:
            self.get_logger().error(
                f"Invalid {action} keys: got={sorted(data.keys())}, "
                f"expected={sorted(expected_keys)}"
            )
            return None

        try:
            rationale = str(data["rationale"]).strip()
            confidence = int(data["confidence"])
        except Exception as exc:
            self.get_logger().error(
                f"Invalid common recovery fields in LLM output: {exc}"
            )
            return None

        if action == "retry_target":
            try:
                target_object_tag = self._sanitize_target(str(data["target_object_tag"]))
                target_intent_hint = str(data.get("target_intent_hint", "")).strip()[:80]
            except Exception as exc:
                self.get_logger().error(
                    f"Invalid retry_target fields in LLM output: {exc}"
                )
                return None

            return ParsedRecoveryAction(
                action=action,
                target_object_tag=target_object_tag,
                target_intent_hint=target_intent_hint,
                waypoints=[],
                wait_seconds=0,
                responsible_object_key="",
                operator_message="",
                rationale=rationale,
                confidence=confidence,
            )

        if action == "wait_then_replan":
            try:
                wait_seconds = int(data["wait_seconds"])
            except Exception as exc:
                self.get_logger().error(
                    f"Invalid wait_then_replan fields in LLM output: {exc}"
                )
                return None

            return ParsedRecoveryAction(
                action=action,
                target_object_tag="",
                target_intent_hint="",
                waypoints=[],
                wait_seconds=wait_seconds,
                responsible_object_key="",
                operator_message="",
                rationale=rationale,
                confidence=confidence,
            )

        if action in {"open_door_then_replan", "clear_object_then_replan"}:
            try:
                responsible_object_key = self._sanitize_target(
                    str(data["responsible_object_key"])
                )
                operator_message = str(data["operator_message"]).strip()
            except Exception as exc:
                self.get_logger().error(
                    f"Invalid {action} fields in LLM output: {exc}"
                )
                return None

            return ParsedRecoveryAction(
                action=action,
                target_object_tag="",
                target_intent_hint="",
                waypoints=[],
                wait_seconds=0,
                responsible_object_key=responsible_object_key,
                operator_message=operator_message,
                rationale=rationale,
                confidence=confidence,
            )

        return ParsedRecoveryAction(
            action="give_up",
            target_object_tag="",
            target_intent_hint="",
            waypoints=[],
            wait_seconds=0,
            responsible_object_key="",
            operator_message="",
            rationale=rationale,
            confidence=confidence,
        )

    def _validate_recovery_and_fill_response(
        self,
        response,
        parsed: ParsedRecoveryAction,
        request,
        raw_output: str,
    ):
        if parsed.action not in self.RECOVERY_ACTIONS:
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message=f"Invalid recovery action='{parsed.action}'.",
            )

        if not (0 <= parsed.confidence <= 100):
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message=(
                    f"Invalid recovery confidence={parsed.confidence}. "
                    "Expected 0..100."
                ),
            )

        if parsed.confidence < self._min_confidence_percent:
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message=(
                    f"Rejected low-confidence recovery proposal: "
                    f"confidence={parsed.confidence}, "
                    f"minimum={self._min_confidence_percent}."
                ),
            )

        if not parsed.rationale:
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message="Recovery rationale cannot be empty.",
            )

        validators = {
            "retry_target": self._validate_retry_target_recovery,
            "wait_then_replan": self._validate_wait_then_replan_recovery,
            "open_door_then_replan": self._validate_open_door_then_replan_recovery,
            "clear_object_then_replan": self._validate_clear_object_then_replan_recovery,
            "give_up": self._validate_give_up_recovery,
        }

        return validators[parsed.action](
            response=response,
            parsed=parsed,
            request=request,
            raw_output=raw_output,
        )

    def _validate_retry_target_recovery(
        self,
        response,
        parsed: ParsedRecoveryAction,
        request,
        raw_output: str,
    ):
        if not parsed.target_object_tag:
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message="retry_target requires non-empty target_object_tag.",
            )

        if self._target_is_placeholder(parsed.target_object_tag):
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message=f"Rejected placeholder target_object_tag='{parsed.target_object_tag}'.",
            )

        resolved_tag = self._object_store.resolve_tag_or_alias(parsed.target_object_tag)
        if resolved_tag is None:
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message=(
                    f"target_object_tag='{parsed.target_object_tag}' is not a navigable "
                    f"tag in this scene. Known: {', '.join(self._object_store.navigable_tag_vocabulary)}"
                ),
            )

        original_tag = self._normalize(
            getattr(request, "original_object_tag", "") or request.original_target or ""
        )
        if self._normalize(resolved_tag) == original_tag:
            # Reject only when there is a single instance of this tag; if multiple
            # instances exist (e.g. six chairs) the orchestrator can exclude the
            # blocked object_key and resolve to a different one.
            instance_count = len(self._object_store.rows_for_tag(resolved_tag))
            if instance_count <= 1:
                return self._fill_recovery_failure(
                    response=response,
                    raw_output=raw_output,
                    message=(
                        f"Recovery target_object_tag='{resolved_tag}' repeats the only "
                        f"instance of this tag — no alternative exists."
                    ),
                )

        attempted_tags = {
            self._normalize(v) for v in request.attempted_values if v
        }
        if self._normalize(resolved_tag) in attempted_tags:
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message=(
                    f"Recovery target_object_tag='{resolved_tag}' was already attempted."
                ),
            )

        self._fill_recovery_success_common(
            response=response,
            parsed=parsed,
            raw_output=raw_output,
            message=(
                f"Accepted recovery retry_target: object_tag='{resolved_tag}', "
                f"intent_hint='{parsed.target_intent_hint}', confidence={parsed.confidence}."
            ),
        )
        response.action = "retry_target"
        response.target = ""
        response.target_object_tag = resolved_tag
        response.target_intent_hint = parsed.target_intent_hint
        response.target_object_key = ""
        response.waypoints = []

        self.get_logger().info(
            f"[RECOVERY] Accepted retry_target: "
            f"target_object_tag='{resolved_tag}', "
            f"target_intent_hint='{parsed.target_intent_hint}', "
            f"confidence={parsed.confidence}, "
            f"rationale='{parsed.rationale}'"
        )

        return response


    def _validate_wait_then_replan_recovery(
        self,
        response,
        parsed: ParsedRecoveryAction,
        request,
        raw_output: str,
    ):
        if not (1 <= int(parsed.wait_seconds) <= 30):
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message=(
                    f"wait_then_replan wait_seconds={parsed.wait_seconds} invalid. "
                    "Expected 1..30."
                ),
            )

        waits_used = int(getattr(request, "deterministic_waits_used", 0))
        wait_cap = int(getattr(request, "deterministic_wait_cap", 0))

        if waits_used < wait_cap:
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message=(
                    "wait_then_replan ineligible: deterministic short-circuit not exhausted."
                ),
            )

        self._fill_recovery_success_common(
            response=response,
            parsed=parsed,
            raw_output=raw_output,
            message=(
                f"Accepted recovery wait_then_replan wait_seconds={parsed.wait_seconds}, "
                f"confidence={parsed.confidence}."
            ),
        )
        response.action = "wait_then_replan"
        response.target = ""
        response.waypoints = []
        response.wait_seconds = int(parsed.wait_seconds)

        self.get_logger().info(
            f"[RECOVERY] Accepted wait_then_replan: "
            f"wait_seconds={parsed.wait_seconds}, "
            f"confidence={parsed.confidence}, "
            f"rationale='{parsed.rationale}'"
        )

        return response

    def _validate_open_door_then_replan_recovery(
        self,
        response,
        parsed: ParsedRecoveryAction,
        request,
        raw_output: str,
    ):
        object_error = self._validate_operator_object_action_common(
            parsed=parsed,
            request=request,
            action_name="open_door_then_replan",
        )
        if object_error is not None:
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message=object_error,
            )

        if not bool(getattr(request, "responsible_openable", False)):
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message="open_door_then_replan ineligible: responsible object is not openable.",
            )

        self._fill_recovery_success_common(
            response=response,
            parsed=parsed,
            raw_output=raw_output,
            message=(
                f"Accepted recovery open_door_then_replan for "
                f"responsible_object_key='{parsed.responsible_object_key}', "
                f"confidence={parsed.confidence}."
            ),
        )
        response.action = "open_door_then_replan"
        response.target = ""
        response.waypoints = []
        response.responsible_object_key = parsed.responsible_object_key
        response.operator_message = parsed.operator_message

        self.get_logger().info(
            f"[RECOVERY] Accepted open_door_then_replan: "
            f"responsible_object_key='{parsed.responsible_object_key}', "
            f"operator_message='{parsed.operator_message}', "
            f"confidence={parsed.confidence}, "
            f"rationale='{parsed.rationale}'"
        )

        return response

    def _validate_clear_object_then_replan_recovery(
        self,
        response,
        parsed: ParsedRecoveryAction,
        request,
        raw_output: str,
    ):
        object_error = self._validate_operator_object_action_common(
            parsed=parsed,
            request=request,
            action_name="clear_object_then_replan",
        )
        if object_error is not None:
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message=object_error,
            )

        if not bool(getattr(request, "responsible_clearable", False)):
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message="clear_object_then_replan ineligible: responsible object is not clearable.",
            )

        object_state = (
            getattr(request, "responsible_object_state", "") or ""
        ).strip()
        if object_state not in {"movable", "semi-static"}:
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message=(
                    "clear_object_then_replan ineligible: "
                    f"object_state='{object_state}' is not movable or semi-static."
                ),
            )

        object_tag = (
            getattr(request, "responsible_object_tag", "") or ""
        ).strip()
        if self._tag_is_door_or_gate(object_tag):
            return self._fill_recovery_failure(
                response=response,
                raw_output=raw_output,
                message="clear_object_then_replan ineligible: door/gate should be opened, not cleared.",
            )

        self._fill_recovery_success_common(
            response=response,
            parsed=parsed,
            raw_output=raw_output,
            message=(
                f"Accepted recovery clear_object_then_replan for "
                f"responsible_object_key='{parsed.responsible_object_key}', "
                f"confidence={parsed.confidence}."
            ),
        )
        response.action = "clear_object_then_replan"
        response.target = ""
        response.waypoints = []
        response.responsible_object_key = parsed.responsible_object_key
        response.operator_message = parsed.operator_message

        self.get_logger().info(
            f"[RECOVERY] Accepted clear_object_then_replan: "
            f"responsible_object_key='{parsed.responsible_object_key}', "
            f"operator_message='{parsed.operator_message}', "
            f"confidence={parsed.confidence}, "
            f"rationale='{parsed.rationale}'"
        )

        return response

    def _validate_operator_object_action_common(
        self,
        parsed: ParsedRecoveryAction,
        request,
        action_name: str,
    ) -> Optional[str]:
        if (getattr(request, "match_type", "") or "").strip() != "verified":
            return f"{action_name} ineligible: responsible object match is not verified."

        request_object_key = (
            getattr(request, "responsible_object_key", "") or ""
        ).strip()
        if not request_object_key:
            return f"{action_name} ineligible: request responsible_object_key is empty."

        if parsed.responsible_object_key != request_object_key:
            return (
                f"{action_name} ineligible: response responsible_object_key "
                f"'{parsed.responsible_object_key}' does not match request key "
                f"'{request_object_key}'."
            )

        safety_class = (
            getattr(request, "responsible_safety_class", "") or "none"
        ).strip()
        if safety_class != "none":
            return f"{action_name} ineligible: safety_class='{safety_class}'."

        operator_error = self._validate_operator_message(parsed.operator_message)
        if operator_error is not None:
            return f"{action_name} ineligible: {operator_error}"

        return None

    def _validate_operator_message(self, operator_message: str) -> Optional[str]:
        if not operator_message:
            return "operator_message cannot be empty."

        if len(operator_message) > 160:
            return f"operator_message too long: len={len(operator_message)}, max=160."

        if "\n" in operator_message or "\r" in operator_message:
            return "operator_message must not contain newlines."

        return None

    def _validate_give_up_recovery(
        self,
        response,
        parsed: ParsedRecoveryAction,
        request,
        raw_output: str,
    ):
        self._fill_recovery_success_common(
            response=response,
            parsed=parsed,
            raw_output=raw_output,
            message="LLM recovery chose give_up.",
        )
        response.action = "give_up"
        response.target = ""
        response.waypoints = []
        response.operator_message = self._make_give_up_operator_message(parsed.rationale)

        self.get_logger().warn(
            f"[RECOVERY] LLM chose give_up: rationale='{parsed.rationale}', "
            f"confidence={parsed.confidence}"
        )

        return response

    def _fill_recovery_success_common(
        self,
        response,
        parsed: ParsedRecoveryAction,
        raw_output: str,
        message: str,
    ):
        response.success = True
        response.action = parsed.action
        response.target = ""
        response.waypoints = []
        response.rationale = parsed.rationale
        response.confidence_percent = int(parsed.confidence)
        response.raw_output = raw_output
        response.message = message
        response.responsible_object_key = ""
        response.operator_message = ""
        response.wait_seconds = 0
        response.target_object_tag = ""
        response.target_intent_hint = ""
        response.target_object_key = ""
        return response

    def _canonicalize_attempted_values(self, attempted_values) -> Set[str]:
        canonicals: Set[str] = set()

        for value in attempted_values:
            if not value:
                continue

            canonical = self._canonicalize_query(str(value))
            if canonical is not None:
                canonicals.add(canonical)
                continue

            for part in str(value).split(","):
                part = part.strip()
                if not part:
                    continue
                canonical = self._canonicalize_query(part)
                if canonical is not None:
                    canonicals.add(canonical)

        return canonicals

    def _canonicalize_attempted_chains(self, attempted_values) -> Set[tuple]:
        chains: Set[tuple] = set()

        for value in attempted_values:
            if not value:
                continue

            parts = [
                part.strip()
                for part in str(value).split(",")
                if part.strip()
            ]

            if len(parts) <= 1:
                continue

            canonical_parts = []
            valid = True

            for part in parts:
                canonical = self._canonicalize_query(part)
                if canonical is None:
                    valid = False
                    break
                canonical_parts.append(canonical)

            if valid and canonical_parts:
                chains.add(tuple(canonical_parts))

        return chains

    @staticmethod
    def _normalize(text: str) -> str:
        return " ".join(text.strip().lower().replace("_", " ").split())

    @staticmethod
    def _tag_is_door_or_gate(tag: str) -> bool:
        normalized = " ".join((tag or "").strip().lower().split())
        return "door" in normalized or "gate" in normalized

    @staticmethod
    def _make_give_up_operator_message(rationale: str) -> str:
        base = "No safe semantic recovery was found. Operator intervention required."
        rationale = " ".join((rationale or "").strip().split())
        if not rationale:
            return base

        message = f"{base} Reason: {rationale}"
        return message[:160]

    @staticmethod
    def _extract_json_object(text: str):
        start = text.find("{")
        end = text.rfind("}")

        if start < 0 or end <= start:
            return None

        candidate = text[start:end + 1]

        try:
            return json.loads(candidate)
        except json.JSONDecodeError:
            return None

    @staticmethod
    def _fill_recovery_failure(response, raw_output: str, message: str):
        response.success = False
        response.action = "give_up"
        response.target = ""
        response.waypoints = []
        response.rationale = ""
        response.confidence_percent = 0
        response.raw_output = raw_output
        response.message = message
        response.responsible_object_key = ""
        response.operator_message = ""
        response.wait_seconds = 0
        response.target_object_tag = ""
        response.target_intent_hint = ""
        response.target_object_key = ""
        return response


def main(args=None):
    rclpy.init(args=args)

    node = NavigatorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info(
            "Keyboard interrupt received. Shutting down navigator node."
        )
    finally:
        executor.remove_node(node)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()