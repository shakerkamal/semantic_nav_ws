# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Unit tests for open-set affordance routing in dynamic object ingestion.

Uses LocalObjectQueryNode.__new__ to exercise _classify_dynamic_object
without the full node __init__ (which needs rclpy + a live ROS graph),
mirroring the pattern already used for NavigatorNode/NavigationOrchestrator
elsewhere in this codebase.
"""
import os

from ament_index_python.packages import get_package_share_directory

from semantic_nav_semantics.affordance_classification import InferredAffordance
from semantic_nav_semantics.local_object_query_node import (
    LocalObjectQueryNode,
    load_object_action_attributes,
)


def _load_table():
    path = os.path.join(
        get_package_share_directory("semantic_nav_semantics"),
        "config",
        "object_action_attributes.json",
    )
    return load_object_action_attributes(path)


def _node(open_set_enabled, inferred=None):
    node = LocalObjectQueryNode.__new__(LocalObjectQueryNode)
    node._action_attrs = _load_table()
    node._open_set_inference_enabled = lambda: open_set_enabled
    calls = []

    def _fake_infer(tag, caption):
        calls.append((tag, caption))
        return inferred

    node._infer_affordance = _fake_infer
    return node, calls


def test_known_tag_never_calls_inference():
    node, calls = _node(open_set_enabled=True, inferred=InferredAffordance(
        True, True, "human", 99))
    safety, openable, clearable = node._classify_dynamic_object("chair", "a chair")
    assert calls == []  # table already classifies "chair"; no inference call
    assert safety == "none"
    assert clearable is True
    assert openable is False


def test_unclassifiable_tag_uses_inference_when_enabled():
    inferred = InferredAffordance(True, False, "none", 85)
    node, calls = _node(open_set_enabled=True, inferred=inferred)
    safety, openable, clearable = node._classify_dynamic_object(
        "room partition", "a folding room partition"
    )
    assert calls == [("room partition", "a folding room partition")]
    assert openable is True
    assert clearable is False
    assert safety == "none"


def test_unclassifiable_tag_falls_back_when_disabled():
    node, calls = _node(open_set_enabled=False, inferred=InferredAffordance(
        True, True, "human", 99))
    safety, openable, clearable = node._classify_dynamic_object(
        "room partition", "a folding room partition"
    )
    assert calls == []  # open-set inference disabled -- must not be called
    assert safety == "none"
    assert openable is False
    assert clearable is False


def test_unclassifiable_tag_falls_back_when_inference_returns_none():
    # Service unavailable, timed out, or below the confidence floor --
    # _infer_affordance already returns None for all of these; the caller
    # must fall back to the table default rather than error.
    node, calls = _node(open_set_enabled=True, inferred=None)
    safety, openable, clearable = node._classify_dynamic_object(
        "room partition", "a folding room partition"
    )
    assert calls == [("room partition", "a folding room partition")]
    assert safety == "none"
    assert openable is False
    assert clearable is False
