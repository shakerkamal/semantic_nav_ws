#!/usr/bin/env bash
# CLOSE the bedroom room partition: spawn the panel across the narrow gap
# between the two FoldingDoor_01 screens at partition:121 (-2.5068, -1.3503).
#
# The world's FoldingDoor models are <static> (no opening mechanism), so they
# are used as GAP-FORMING geometry: slid toward each other along x=-2.46 to
# leave a ~0.9m passage at y=-1.35 (see worlds/small_house_semantic.world).
#
# Open-set scenario (spec 21.4): the blocker is tagged "room partition" in
# map_v001.json (object_121), a NOVEL non-door tag the affordance table cannot
# classify. Like the AWS door it is DYNAMIC -- spawned/deleted at runtime so the
# costmap OBSTACLE layer tracks it. Do NOT map the house with it present.
#
# The panel's 0.9 m width is along its local X; the gap runs along Y
# (map extent [0.2, 0.9, 2.0]), so it is spawned yaw=1.5708 (90 deg) to fill it.
set -e
PX=${PX:--2.5068}; PY=${PY:--1.3503}; PYAW=${PYAW:-1.5708}
PANEL_SDF=${PANEL_SDF:-$(ros2 pkg prefix semantic_nav_bringup)/share/semantic_nav_bringup/models/door_scenario/door_scenario_panel.sdf}
ros2 run gazebo_ros spawn_entity.py \
  -entity door_scenario_panel \
  -file "$PANEL_SDF" \
  -x "$PX" -y "$PY" -z 0.0 -Y "$PYAW"
