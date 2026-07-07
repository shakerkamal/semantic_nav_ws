#!/usr/bin/env bash
# CLOSE the door: spawn the AWS residential door at door:119 (4.86223, -0.677227).
#
# The door is DYNAMIC on purpose -- spawned/deleted at runtime so the costmap
# OBSTACLE layer tracks it. Do NOT map the house with the door present, or it
# gets baked into RTAB-Map's static map and can never be "opened" again.
set -e
DX=${DX:-4.86223}; DY=${DY:--0.677227}; DYAW=${DYAW:--1.55768}
ros2 run gazebo_ros spawn_entity.py \
  -entity aws_robomaker_residential_Door_01 \
  -database aws_robomaker_residential_Door_01 \
  -x "$DX" -y "$DY" -z 0.0 -Y "$DYAW"
