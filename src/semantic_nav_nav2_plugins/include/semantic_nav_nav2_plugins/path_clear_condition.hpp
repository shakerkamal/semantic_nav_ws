// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace semantic_nav_nav2_plugins
{

/**
 * @brief BT condition node that checks whether the active path corridor is blocked.
 *
 * Preferred input is the BT blackboard path="{path}" produced by ComputePathToPose.
 * The /plan subscription is only a fallback/debug source.
 *
 * Returns:
 *   SUCCESS: no path/costmap data yet, or corridor is clear.
 *   FAILURE: lethal cells persist inside the sampled path corridor for debounce_ticks.
 *
 * On FAILURE, writes:
 *   blockage_centroid
 *   blockage_extent_m
 */
class PathClearCondition : public BT::ConditionNode
{
public:
  PathClearCondition(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

  /**
   * @brief Pure helper for unit tests.
   *
   * Samples the first lookahead_m metres of path against the occupancy grid.
   * Around each path pose, samples cells within sample_radius_m.
   */
  static bool isCorridorBlocked(
    const nav_msgs::msg::Path & path,
    const nav_msgs::msg::OccupancyGrid & costmap,
    int lethal_threshold,
    double lookahead_m,
    double sample_radius_m,
    geometry_msgs::msg::Point & centroid_out,
    float & extent_out);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr plan_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

  std::mutex data_mutex_;
  nav_msgs::msg::Path::SharedPtr latest_plan_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

  int blocked_count_{0};
  geometry_msgs::msg::Point last_centroid_{};
  float last_extent_{0.0f};
};

}  // namespace semantic_nav_nav2_plugins