#!/usr/bin/env bash
# OPEN the bedroom room partition: delete the panel so the doorway is clear.
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \
  '{name: door_scenario_panel}'
