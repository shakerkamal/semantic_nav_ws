#!/usr/bin/env python3
"""Static regression tests for the corrected M3 barrier-clearance gate."""

from __future__ import annotations

import os
import xml.etree.ElementTree as ET


HERE = os.path.dirname(os.path.abspath(__file__))
REPO = os.path.abspath(os.path.join(HERE, ".."))
PLUGIN = os.path.join(REPO, "src", "semantic_nav_nav2_plugins")
BT = os.path.join(PLUGIN, "config", "semantic_recovery_bt.xml")


def test_operator_branches_wait_for_map_clearance_before_final_clear():
    root = ET.parse(BT).getroot()

    for name in (
        "OpenDoorThenReplanBranch",
        "ClearObjectThenReplanBranch",
    ):
        sequence = root.find(f".//Sequence[@name='{name}']")
        assert sequence is not None
        children = list(sequence)
        assert [child.tag for child in children] == [
            "OperatorPrompt",
            "WaitForBarrierClear",
            "ClearEntireCostmap",
            "ClearEntireCostmap",
        ]

        gate = children[1]
        # Recentred 2026-07-17 (S2 r3 vs r4 controlled comparison): the
        # monitored window anchors on the MATCHED object's bbox, while the
        # measured centroid/extent stay as the secondary observed region --
        # a captured centroid can sit on a wall whose cells never clear.
        assert gate.get("barrier_center") == "{responsible_bbox_center}"
        assert gate.get("barrier_bbox_extent") == "{responsible_bbox_extent}"
        assert gate.get("observed_blockage_center") == "{blockage_centroid}"
        assert gate.get("observed_blockage_extent_m") == "{blockage_extent_m}"
        assert gate.get("initial_dwell_s") == "12.0"
        assert gate.get("second_dwell_s") == "12.0"
        assert gate.get("poll_interval_s") == "2.0"
        assert gate.get("cleanup_local_grids") == "true"
        assert gate.get("cleanup_filter_scans") == "false"
        assert gate.get("cleanup_service") == "/rtabmap/cleanup_local_grids"
        assert gate.get("required_post_cleanup_clear_samples") == "2"


def test_plugin_is_built_registered_and_declares_dependencies():
    cmake = open(os.path.join(PLUGIN, "CMakeLists.txt"), encoding="utf-8").read()
    package = open(os.path.join(PLUGIN, "package.xml"), encoding="utf-8").read()
    register = open(
        os.path.join(PLUGIN, "src", "register_nodes.cpp"), encoding="utf-8"
    ).read()

    assert "src/wait_for_barrier_clear.cpp" in cmake
    assert "find_package(nav2_msgs REQUIRED)" in cmake
    # rtabmap_msgs is deliberately QUIET/optional: without it the node skips
    # cached-grid cleanup instead of failing the whole package build.
    assert "find_package(rtabmap_msgs QUIET)" in cmake
    assert "<depend>nav2_msgs</depend>" in package
    assert "<depend>rtabmap_msgs</depend>" in package
    assert '"WaitForBarrierClear"' in register


def test_discarded_slam_freeze_is_not_in_eval_runtime():
    trigger = open(
        os.path.join(HERE, "enroute_blockage_trigger.py"), encoding="utf-8"
    ).read()
    runner = open(
        os.path.join(HERE, "run_enroute_trial.sh"), encoding="utf-8"
    ).read()

    assert "/rtabmap/pause" not in trigger
    assert "/rtabmap/resume" not in trigger
    assert "/rtabmap/pause" not in runner
    assert "/rtabmap/resume" not in runner


def test_cleanup_is_not_hidden_in_evaluation_trigger():
    trigger = open(
        os.path.join(HERE, "enroute_blockage_trigger.py"), encoding="utf-8"
    ).read()
    assert "cleanup_local_grids" not in trigger
