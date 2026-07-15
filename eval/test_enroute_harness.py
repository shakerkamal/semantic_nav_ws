"""Tests for the en-route ablation harness (eval/ scripts are not a package,
so tests add eval/ to sys.path)."""
import os
import sys

import yaml

EVAL_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, EVAL_DIR)

SCENARIOS_PATH = os.path.join(EVAL_DIR, "enroute_scenarios.yaml")

REQUIRED_SCENARIO_KEYS = {
    "goal_query", "nl_command", "intent_hint", "trigger",
    "blocker", "detector", "expected_directive",
}


import xml.etree.ElementTree as ET

WS_ROOT = os.path.dirname(EVAL_DIR)
GEO_BT = os.path.join(
    WS_ROOT, "src", "semantic_nav_nav2_plugins", "config",
    "semantic_recovery_bt_geometric.xml",
)


def test_geometric_bt_has_no_semantic_branch():
    tree = ET.parse(GEO_BT)
    tags = {el.tag for el in tree.iter()}
    # The whole Tier-3 branch must be gone.
    for forbidden in ("CaptureBlockageContext", "QuerySemanticContext",
                      "EscalateToLLMRecovery", "OperatorPrompt", "Switch3"):
        assert forbidden not in tags, f"{forbidden} must not be in B-GEO"
    # Tier 1 + Tier 2 must survive intact.
    for required in ("ValidateSemantic", "RateController", "ComputePathToPose",
                     "FollowPath", "ClearEntireCostmap", "BackUp"):
        assert required in tags, f"{required} missing from B-GEO"
    # Tier-1 PARITY with the current semantic tree (158c726): the outer
    # RecoveryNode retries 3x, and BOTH in-place single-retry wraps survive —
    # B-GEO must differ from B-LLM in the recovery child ONLY.
    recoveries = {el.get("name"): el.get("number_of_retries")
                  for el in tree.iter("RecoveryNode")}
    assert recoveries.get("SemanticRecovery") == "3"
    assert recoveries.get("ComputePathToPose") == "1"
    assert recoveries.get("FollowPath") == "1"
    assert len(recoveries) == 3


def test_scenarios_yaml_complete():
    with open(SCENARIOS_PATH) as f:
        data = yaml.safe_load(f)
    assert set(data["scenarios"].keys()) == {"S1", "S2", "S3", "S4", "S5"}
    for name, sc in data["scenarios"].items():
        missing = REQUIRED_SCENARIO_KEYS - set(sc.keys())
        assert not missing, f"{name} missing {missing}"
    # GT directives the parser scores against (spec section 3).
    assert data["scenarios"]["S1"]["expected_directive"] == "none"
    assert data["scenarios"]["S2"]["expected_directive"] == "open_door_then_replan"
    assert data["scenarios"]["S3"]["expected_directive"] == "clear_object_then_replan"
    assert data["scenarios"]["S4"]["expected_directive"] == "wait_then_replan"
    assert data["scenarios"]["S5"]["expected_directive"] == "retry_target"
    # Perception-only contract: detectors never carry affordance fields.
    for name, sc in data["scenarios"].items():
        det = sc["detector"]
        if det is not None:
            assert "openable" not in det and "clearable" not in det \
                and "safety_class" not in det, \
                f"{name}: detector must publish perception only"
