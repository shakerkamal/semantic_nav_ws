# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Verify the kitchen door object is present and well-formed in map_v001.json."""

import os

from ament_index_python.packages import get_package_share_directory
from semantic_nav_semantics.semantic_store import load_semantic_store


def _map_path():
    share = get_package_share_directory("semantic_nav_semantics")
    return os.path.join(share, "config", "map_v001.json")


def _affordances_path():
    share = get_package_share_directory("semantic_nav_semantics")
    return os.path.join(share, "config", "object_intent_affordances.json")


def test_door_119_present_and_semi_static():
    store = load_semantic_store(
        map_path=_map_path(), affordances_path=_affordances_path()
    )
    assert store.object_key_exists("door:119")
    row = store.by_object_key["door:119"]
    assert row.object_tag == "door"
    assert row.object_state == "semi-static"
