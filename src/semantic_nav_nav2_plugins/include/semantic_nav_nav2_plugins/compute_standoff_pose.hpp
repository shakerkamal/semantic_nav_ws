// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"

namespace semantic_nav_nav2_plugins
{

/**
 * @brief BT action that computes a standoff pose in front of a KNOWN object,
 * for Tier-3's "approach the detected candidate before sampling" step
 * (spec: en-route ablation Part A, 2026-07-15).
 *
 * Ports the EXACT algorithm already proven up-front
 * (semantic_nav_semantics/standoff_planner.py:StandoffPlanner.plan) rather
 * than inventing new geometry: the goal is placed at
 * (half max bbox extent + robot_footprint_radius + clearance_margin) along
 * the robot->object vector, facing back toward the object. Requires a real
 * object reference (responsible_bbox_center/extent), unlike the blind
 * DriveOnHeading approach this replaces when a candidate IS known -- see
 * CaptureBlockageContext's own doc comment for why en-route historically
 * could not compute an object-aware standoff (the blocker's identity was
 * exactly what needed discovering). Once QuerySemanticContext's first,
 * wide-radius pass finds SOME candidate (verified or inferred), this node
 * has a real bbox to stand off from.
 *
 * Always returns SUCCESS if a TF robot pose is available (a data-capture
 * node, not a gate) so the tree can decide separately whether to use the
 * computed standoff_goal or fall back to blind approach.
 */
class ComputeStandoffPose : public BT::SyncActionNode
{
public:
  ComputeStandoffPose(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

  /**
   * @brief Pure standoff-pose computation (unit-testable, no ROS/TF).
   *
   * Mirrors StandoffPlanner.plan() exactly: d = half_extent_xy +
   * robot_footprint_radius + clearance_margin; goal is d metres from the
   * object's bbox center, on the side facing the robot; yaw faces from the
   * goal back toward the object. When the robot is (numerically) already at
   * the object's center, falls back to a fixed +x direction, same as the
   * Python planner.
   */
  static geometry_msgs::msg::PoseStamped computeStandoffPose(
    double robot_x,
    double robot_y,
    double bbox_center_x,
    double bbox_center_y,
    double bbox_extent_x,
    double bbox_extent_y,
    double robot_footprint_radius,
    double clearance_margin,
    const std::string & frame_id);

private:
  rclcpp::Node::SharedPtr node_;
};

}  // namespace semantic_nav_nav2_plugins
