import math
from semantic_nav_semantics.semantic_store import ObjectRow
from semantic_nav_semantics.standoff_planner import StandoffPlanner


def make_row(center, extent=(1.0, 0.5, 0.5)):
    return ObjectRow(
        source_key="object_1", object_key="chair:1", object_id=1,
        object_tag="chair", normalized_tag="chair",
        object_caption="x", object_state="movable",
        bbox_center=center, bbox_extent=extent, bbox_volume=extent[0] * extent[1] * extent[2],
    )


def test_standoff_is_outside_bbox_and_faces_object():
    row = make_row(center=(5.0, 0.0, 0.0), extent=(2.0, 1.0, 1.0))
    planner = StandoffPlanner(robot_footprint_radius=0.22, clearance_margin=0.20)
    pose = planner.plan(row, robot_xy=(0.0, 0.0))

    gx, gy = pose.position_xy
    # standoff distance: 0.5*2.0 + 0.22 + 0.20 = 1.42
    expected_dist = 1.42
    actual_dist = math.hypot(5.0 - gx, 0.0 - gy)
    assert abs(actual_dist - expected_dist) < 1e-6

    # yaw faces from goal towards object (+x direction)
    assert abs(pose.yaw - 0.0) < 1e-6


def test_at_object_uses_fallback_direction():
    row = make_row(center=(0.0, 0.0, 0.0))
    planner = StandoffPlanner(robot_footprint_radius=0.22, clearance_margin=0.20)
    pose = planner.plan(row, robot_xy=(0.0, 0.0))
    gx, gy = pose.position_xy
    # fallback vector is (1, 0), so goal is to the -x side
    assert gx < 0.0
    assert abs(gy) < 1e-6


def test_yaw_45_degrees_for_diagonal_object():
    row = make_row(center=(1.0, 1.0, 0.0), extent=(0.1, 0.1, 0.1))
    planner = StandoffPlanner(robot_footprint_radius=0.0, clearance_margin=0.0)
    pose = planner.plan(row, robot_xy=(0.0, 0.0))
    assert abs(pose.yaw - math.pi / 4) < 1e-6
