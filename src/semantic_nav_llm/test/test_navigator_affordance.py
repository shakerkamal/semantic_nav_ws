# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Unit tests for navigator affordance-inference parsing (no ROS runtime).

Uses NavigatorNode.__new__ to exercise the pure parse method without the full
node __init__ (which needs rclpy + llama_ros).
"""

from semantic_nav_llm.navigator_node import NavigatorNode


class _DummyLogger:
    def error(self, *a, **k):
        pass

    def warn(self, *a, **k):
        pass

    def info(self, *a, **k):
        pass


def _parser():
    node = NavigatorNode.__new__(NavigatorNode)
    node._allow_json_extraction_fallback = False
    node.get_logger = lambda: _DummyLogger()
    return node


def test_parse_affordance_output():
    node = _parser()
    parsed = node._parse_affordance_output(
        '{"openable":true,"clearable":false,"safety_class":"none","confidence":85}'
    )
    assert parsed == {"openable": True, "clearable": False,
                      "safety_class": "none", "confidence": 85}


def test_parse_affordance_rejects_bad_safety_class():
    node = _parser()
    assert node._parse_affordance_output(
        '{"openable":true,"clearable":false,"safety_class":"robot","confidence":85}'
    ) is None


def test_parse_affordance_rejects_extra_keys():
    node = _parser()
    assert node._parse_affordance_output(
        '{"openable":true,"clearable":false,"safety_class":"none",'
        '"confidence":85,"pose":[1,2]}'
    ) is None


def test_parse_affordance_rejects_non_json():
    node = _parser()
    assert node._parse_affordance_output("not json at all") is None
