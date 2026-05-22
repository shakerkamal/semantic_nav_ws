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

from semantic_nav_interfaces.srv import ParseSemanticCommand


@dataclass(frozen=True)
class SemanticCatalog:
    canonical_locations: Tuple[str, ...]
    valid_queries: Tuple[str, ...]
    normalized_to_canonical: Dict[str, str]


@dataclass(frozen=True)
class LLMIntent:
    action: str
    target: str
    confidence: int

class NavigatorNode(Node):
    """
    LLM semantic intent parser for semantic navigation.

    Provides:
      /parse_semantic_command
        semantic_nav_interfaces/srv/ParseSemanticCommand

    Calls:
      /llama/generate_response
        llama_msgs/action/GenerateResponse

    Expected GBNF-constrained LLM output:
      {"action":"navigate","target":"kitchen","confidence":95}
      {"action":"clarify","target":"","confidence":85}
      {"action":"reject","target":"","confidence":95}

    Service response mapping:
      action=navigate -> intent=navigate_to_location
      action=clarify  -> intent=clarify
      action=reject   -> intent=reject

    Safety boundary:
      This node never emits poses, x/y/yaw, cmd_vel, Nav2 goals,
      planner IDs, or behavior-tree commands.
    """

    def __init__(self):
        super().__init__("navigator_node")

        default_semantic_db_path = os.path.join(
            get_package_share_directory("semantic_nav_semantics"),
            "config",
            "semantic_db.json",
        )

        default_grammar_path = os.path.join(
            get_package_share_directory("semantic_nav_llm"),
            "config",
            "semantic_intent.gbnf",
        )

        self.declare_parameter("service_name", "/parse_semantic_command")
        self.declare_parameter("llama_action", "/llama/generate_response")
        self.declare_parameter("semantic_db_path", default_semantic_db_path)
        self.declare_parameter("grammar_path", default_grammar_path)

        self.declare_parameter("llama_wait_timeout_sec", 30.0)
        self.declare_parameter("llm_send_goal_timeout_sec", 10.0)
        self.declare_parameter("llm_result_timeout_sec", 60.0)

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
        self._semantic_db_path = (
            self.get_parameter("semantic_db_path")
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

        self._callback_group = ReentrantCallbackGroup()

        self._catalog = self._load_semantic_catalog(self._semantic_db_path)
        self._gbnf_grammar = self._load_gbnf(self._grammar_path)

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

        self.get_logger().info(
            "NavigatorNode initialized: "
            f"service='{self._service_name}', "
            f"llama_action='{self._llama_action_name}', "
            f"semantic_db_path='{self._semantic_db_path}', "
            f"grammar_path='{self._grammar_path}', "
            f"canonical_locations={len(self._catalog.canonical_locations)}, "
            f"valid_queries={len(self._catalog.valid_queries)}, "
            f"min_confidence_percent={self._min_confidence_percent}, "
            f"target_len={self._target_min_len}..{self._target_max_len}, "
            f"max_tokens={self._max_tokens}"
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
    
    def _call_llama(self, prompt: str, gbnf_grammar: str) -> Optional[str]:
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

        for field_name in ["n_predict", "max_tokens", "max_new_tokens"]:
            if hasattr(sc, field_name):
                setattr(sc, field_name, int(self._max_tokens))
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

        # grammar_schema is for schema-style constraints in versions that expose it.
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
        
        # Exact schema only.
        if set(data.keys()) != {"action", "target", "confidence"}:
            self.get_logger().error(
                f"Invalid LLM JSON keys: got={sorted(data.keys())}, "
                "expected=['action', 'confidence', 'target']"
            )
            return None

        try:
            action = str(data["action"]).strip()
            target = self._sanitize_target(str(data["target"]))
            confidence = int(data["confidence"])
        except Exception as exc:
            self.get_logger().error(
                f"Missing or invalid JSON fields in LLM output: {exc}"
            )
            return None

        return LLMIntent(
            action=action,
            target=target,
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

        if parsed.target:
            return self._fill_failure(
                response=response,
                raw_output=raw_output,
                message=(
                    f"Action '{parsed.action}' must use empty target, "
                    f"but got target='{parsed.target}'."
                ),
            )

        if parsed.action == "clarify":
            response.success = True
            response.intent = "clarify"
            response.location_query = ""
            response.canonical_location_id = ""
            response.confidence_percent = int(parsed.confidence)
            response.location_known = False
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
        response.location_query = ""
        response.canonical_location_id = ""
        response.confidence_percent = int(parsed.confidence)
        response.location_known = False
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
        target = self._sanitize_target(parsed.target)

        if parsed.confidence < self._min_confidence_percent:
            return self._fill_failure(
                response=response,
                raw_output=raw_output,
                message=(
                    f"Rejected low-confidence navigation intent: "
                    f"confidence={parsed.confidence}, "
                    f"minimum={self._min_confidence_percent}."
                ),
            )

        if not target:
            return self._fill_failure(
                response=response,
                raw_output=raw_output,
                message="Navigate action requires non-empty target.",
            )

        if not self._target_length_is_valid(target):
            return self._fill_failure(
                response=response,
                raw_output=raw_output,
                message=(
                    f"Target length invalid: len={len(target)}, "
                    f"allowed={self._target_min_len}..{self._target_max_len}."
                ),
            )

        if self._target_is_placeholder(target):
            return self._fill_failure(
                response=response,
                raw_output=raw_output,
                message=f"Rejected placeholder target='{target}'.",
            )

        canonical_location_id = self._canonicalize_query(target)

        if canonical_location_id is None:
            return self._fill_failure(
                response=response,
                raw_output=raw_output,
                message=(
                    f"LLM target='{target}' is not known in semantic_db.json. "
                    "Navigation intent rejected before resolution/validation/execution."
                ),
            )

        response.success = True
        response.intent = "navigate_to_location"
        response.location_query = target
        response.canonical_location_id = canonical_location_id
        response.confidence_percent = int(parsed.confidence)
        response.location_known = True
        response.raw_output = raw_output
        response.message = (
            f"Accepted navigation intent: target='{target}', "
            f"canonical_location_id='{canonical_location_id}', "
            f"confidence={parsed.confidence}."
        )

        self.get_logger().info(
            f"[LLM_INTENT] Accepted navigate: "
            f"target='{target}', "
            f"canonical_location_id='{canonical_location_id}', "
            f"confidence={parsed.confidence}"
        )

        return response
    
    @staticmethod
    def _fill_failure(response, raw_output: str, message: str):
        response.success = False
        response.intent = "reject"
        response.location_query = ""
        response.canonical_location_id = ""
        response.confidence_percent = 0
        response.location_known = False
        response.raw_output = raw_output
        response.message = message
        return response

    def _build_prompt(self, command: str) -> str:
        return f"""You are a robotics navigation agent for a mobile robot using ROS 2 Nav2.
        Return exactly one JSON object matching this schema:
        {{"action":"navigate|clarify|reject","target":"string","confidence":0-100}}

        Task:
        Infer the best semantic destination from the user command.

        Rules:
        - Use action "navigate" when the user names a place or expresses a need that implies a place.
        - Use action "clarify" when the user wants navigation but the destination is unclear.
        - Use action "reject" for raw robot motion commands or non-navigation commands.
        - For navigate, target must be a short common place or functional destination.
        - For clarify or reject, target must be "".
        - Do not output coordinates, poses, velocity commands, Nav2 commands, or explanations.
        - Do not use articles such as "the", "a", or "an" in the target.
        - No markdown. No prose.

        Examples:
        User: I am hungry
        Output: {{"action":"navigate","target":"kitchen","confidence":95}}

        User: I am tired
        Output: {{"action":"navigate","target":"bedroom","confidence":90}}

        User: Drive forward two meters
        Output: {{"action":"reject","target":"","confidence":95}}

        User: Take me there
        Output: {{"action":"clarify","target":"","confidence":85}}

        User:
        {command}
        """


    def _load_semantic_catalog(self, db_path: str) -> SemanticCatalog:
        if not db_path:
            raise ValueError("semantic_db_path cannot be empty.")

        if not os.path.exists(db_path):
            raise FileNotFoundError(f"Semantic DB not found at '{db_path}'.")

        with open(db_path, "r", encoding="utf-8") as f:
            data = json.load(f)

        locations = data.get("locations")
        if not isinstance(locations, dict) or not locations:
            raise ValueError(
                "Semantic DB must contain non-empty object field 'locations'."
            )

        canonical_locations: List[str] = []
        valid_queries_set: Set[str] = set()
        normalized_to_canonical: Dict[str, str] = {}

        for location_id, record in locations.items():
            if not isinstance(location_id, str):
                raise ValueError("All location IDs must be strings.")

            if not isinstance(record, dict):
                raise ValueError(f"Location '{location_id}' must be an object.")

            canonical = location_id.strip()
            if not canonical:
                raise ValueError("Location ID cannot be empty.")

            canonical_locations.append(canonical)

            names = [canonical]

            aliases = record.get("aliases", [])
            if aliases is None:
                aliases = []

            if not isinstance(aliases, list):
                raise ValueError(f"Location '{location_id}' aliases must be a list.")

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

        if "root ::=" not in grammar:
            raise ValueError("GBNF grammar must define a root rule.")

        self.get_logger().info(
            f"Loaded strict free-target GBNF grammar from '{grammar_path}' "
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

    @staticmethod
    def _normalize(text: str) -> str:
        return " ".join(text.strip().lower().replace("_", " ").split())
    
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