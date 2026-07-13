# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Dynamic overlay observations are affordance-classified by the TABLE.

A ConceptGraph-style provider reports WHAT it perceives (tag, caption, state,
geometry); openable/clearable/safety_class are the semantic layer's judgment,
applied at ingestion in _handle_dynamic_objects via attributes_for_tag — the
same table that classifies persistent-map objects. These tests pin the table
contract the en-route ablation scenarios (S3 chair, S4 person) depend on.
"""

import os

from ament_index_python.packages import get_package_share_directory

from semantic_nav_semantics.local_object_query_node import (
    attributes_for_tag,
    load_object_action_attributes,
)


def _load_table():
    path = os.path.join(
        get_package_share_directory("semantic_nav_semantics"),
        "config",
        "object_action_attributes.json",
    )
    return load_object_action_attributes(path)


def test_person_classifies_as_human_safety():
    safety, openable, clearable = attributes_for_tag(_load_table(), "person")
    assert safety == "human"
    assert not openable
    assert not clearable


def test_chair_classifies_as_clearable():
    safety, openable, clearable = attributes_for_tag(_load_table(), "chair")
    assert safety == "none"
    assert clearable
    assert not openable


def test_unknown_tag_keeps_restrictive_defaults():
    safety, openable, clearable = attributes_for_tag(
        _load_table(), "totally novel gadget"
    )
    assert safety == "none"
    assert not openable
    assert not clearable
