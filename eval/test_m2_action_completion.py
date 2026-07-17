"""Static regression tests for the M2 operator-action completion barrier."""

import ast
import os
import re
import xml.etree.ElementTree as ET

import yaml


EVAL_DIR = os.path.dirname(os.path.abspath(__file__))
REPO_ROOT = os.path.dirname(EVAL_DIR)
PLUGIN_DIR = os.path.join(
    REPO_ROOT, "src", "semantic_nav_nav2_plugins"
)
BT_XML = os.path.join(PLUGIN_DIR, "config", "semantic_recovery_bt.xml")
HEADER = os.path.join(
    PLUGIN_DIR,
    "include",
    "semantic_nav_nav2_plugins",
    "operator_prompt.hpp",
)
SOURCE = os.path.join(PLUGIN_DIR, "src", "operator_prompt.cpp")
TRIGGER = os.path.join(EVAL_DIR, "enroute_blockage_trigger.py")
SCENARIOS = os.path.join(EVAL_DIR, "enroute_scenarios.yaml")


def _read(path: str) -> str:
    with open(path, encoding="utf-8") as stream:
        return stream.read()


def _extract_function(source: str, name: str):
    tree = ast.parse(source)
    function = next(
        node
        for node in tree.body
        if isinstance(node, ast.FunctionDef) and node.name == name
    )
    module = ast.Module(body=[function], type_ignores=[])
    ast.fix_missing_locations(module)
    namespace = {"Tuple": tuple}
    exec(compile(module, TRIGGER, "exec"), namespace)  # noqa: S102
    return namespace[name]


def test_operator_branches_wait_for_action_and_map_before_costmap_clear():
    root = ET.parse(BT_XML).getroot()

    for sequence_name in (
        "OpenDoorThenReplanBranch",
        "ClearObjectThenReplanBranch",
    ):
        sequence = root.find(f".//Sequence[@name='{sequence_name}']")
        assert sequence is not None

        children = list(sequence)
        assert [child.tag for child in children] == [
            "OperatorPrompt",
            "WaitForBarrierClear",
            "ClearEntireCostmap",
            "ClearEntireCostmap",
        ]

        prompt = children[0]
        assert prompt.get("wait_for_action_completion") == "true"
        assert prompt.get("action_completion_timeout_ms") == "30000"
        assert prompt.get("action_request_topic") == \
            "/operator_action_request"
        assert prompt.get("action_completion_topic") == \
            "/operator_action_completion"


def test_m1_fresh_path_order_is_preserved():
    root = ET.parse(BT_XML).getroot()
    pipeline = root.find(
        ".//PipelineSequence[@name='NavigateWithValidation']"
    )
    assert pipeline is not None

    children = list(pipeline)
    validate_index = next(
        i for i, child in enumerate(children)
        if child.tag == "ValidateSemantic"
    )
    planner_index = next(
        i for i, child in enumerate(children)
        if child.find(".//ComputePathToPose") is not None
    )
    gate_index = next(
        i for i, child in enumerate(children)
        if child.tag == "PathClearCondition"
    )
    follow_index = next(
        i for i, child in enumerate(children)
        if child.find(".//FollowPath") is not None
    )

    assert validate_index < planner_index < gate_index < follow_index


def test_operator_prompt_declares_completion_phase_and_ports():
    header = _read(HEADER)
    source = _read(SOURCE)

    assert "kWaitActionCompletion" in header
    assert "action_request_pub_" in header
    assert "action_completion_sub_" in header

    for port in (
        "wait_for_action_completion",
        "action_completion_timeout_ms",
        "action_request_topic",
        "action_completion_topic",
    ):
        assert f'"{port}"' in source

    assert "expected_action_token_" in source
    assert "action_request_pub_->publish" in source
    assert "msg->data != expected_action_token_" in source


def test_operator_action_token_contract():
    parse_action_token = _extract_function(_read(TRIGGER), "parse_action_token")

    assert parse_action_token(
        "event-123|door:903|open_door_then_replan"
    ) == (
        "event-123",
        "door:903",
        "open_door_then_replan",
    )

    for invalid in (
        "",
        "event-only",
        "event|object",
        "|door:903|action",
        "event||action",
        "event|door:903|",
    ):
        try:
            parse_action_token(invalid)
        except ValueError:
            pass
        else:
            raise AssertionError(f"invalid token accepted: {invalid!r}")


def test_deleted_state_is_not_published_from_delete_request():
    source = _read(TRIGGER)

    delete_method = re.search(
        r"    def _delete\(self\).*?"
        r"(?=    def _on_delete_response)",
        source,
        re.S,
    )
    response_method = re.search(
        r"    def _on_delete_response\(self, future\).*?"
        r"(?=    def _publish_action_completion)",
        source,
        re.S,
    )

    assert delete_method is not None
    assert response_method is not None
    assert 'String(data="deleted")' not in delete_method.group(0)

    response = response_method.group(0)
    assert "if not response.success:" in response
    assert 'String(data="deleted")' in response
    assert response.index("if not response.success:") < response.index(
        'String(data="deleted")'
    )


def test_scenario_topics_and_settle_interval():
    with open(SCENARIOS, encoding="utf-8") as stream:
        data = yaml.safe_load(stream)

    common = data["common"]
    assert common["operator_action_request_topic"] == \
        "/operator_action_request"
    assert common["operator_action_completion_topic"] == \
        "/operator_action_completion"
    assert common["operator_action_settle_sec"] == 0.5

    assert data["scenarios"]["S2"]["expected_directive"] == \
        "open_door_then_replan"
    assert data["scenarios"]["S3"]["expected_directive"] == \
        "clear_object_then_replan"
