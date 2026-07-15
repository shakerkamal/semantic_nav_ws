# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Mirrors semantic_nav_orchestrator/test/test_affordance_classification.py --
the module here is a deliberate duplicate (see the module docstring), so
its behavior must stay identical."""
from semantic_nav_semantics.affordance_classification import (
    InferredAffordance, accept_inference, tag_is_classifiable,
)

TAGS = {"door", "chair", "person", "refrigerator"}


def test_known_tag_is_classifiable():
    assert tag_is_classifiable("chair", TAGS) is True
    assert tag_is_classifiable("Chair ", TAGS) is True  # normalized


def test_door_substring_is_classifiable():
    assert tag_is_classifiable("folding door", TAGS) is True  # substring rule


def test_novel_tag_is_unclassifiable():
    assert tag_is_classifiable("room partition", TAGS) is False
    assert tag_is_classifiable("room partition", TAGS, door_substring=False) is False


def test_accept_inference_confidence_floor():
    hi = InferredAffordance(True, False, "none", 80)
    lo = InferredAffordance(True, False, "none", 40)
    assert accept_inference(hi, 60) is True
    assert accept_inference(lo, 60) is False
