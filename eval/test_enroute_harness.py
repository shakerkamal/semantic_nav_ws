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
