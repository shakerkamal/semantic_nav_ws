#!/usr/bin/env bash
# OPEN the door: delete the AWS residential door so the doorway is clear.
ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \
  '{name: aws_robomaker_residential_Door_01}'
