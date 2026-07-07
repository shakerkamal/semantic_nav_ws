# Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
"""Unit tests for the OccupancyGrid -> CostGrid adapter (no ROS runtime)."""

from types import SimpleNamespace

from semantic_nav_orchestrator.costmap_adapter import occupancygrid_to_costgrid


def _fake_occupancy_grid():
    return SimpleNamespace(
        info=SimpleNamespace(
            resolution=0.05,
            width=3,
            height=2,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=-1.0, y=-2.0, z=0.0)
            ),
        ),
        data=[0, 100, -1, 0, 50, 0],
    )


def test_adapter_copies_geometry_and_data():
    grid = occupancygrid_to_costgrid(_fake_occupancy_grid())
    assert grid.resolution == 0.05
    assert grid.width == 3
    assert grid.height == 2
    assert grid.origin_x == -1.0
    assert grid.origin_y == -2.0
    assert list(grid.data) == [0, 100, -1, 0, 50, 0]


def test_adapter_value_lookup_matches_grid():
    grid = occupancygrid_to_costgrid(_fake_occupancy_grid())
    assert grid.value(1, 0) == 100   # row 0, col 1
    assert grid.value(2, 0) == -1    # unknown preserved
