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
 * @brief Metrics returned by PathClearCondition::isCorridorBlocked.
 *
 * Severity fields (blocked_poses, blocked_fraction, max_run_length_m) drive the
 * allow_geometric_detour_first gating in tick(): minor local obstacles that Nav2
 * can replan around return any_blocked=true but low severity, so tick() returns
 * SUCCESS and lets Nav2 handle them without escalating to semantic recovery.
 */
struct BlockageMetrics
{
  bool any_blocked{false};
  int blocked_poses{0};          // path poses with at least one lethal sample
  int total_poses{0};            // total path poses checked within lookahead
  double blocked_fraction{0.0};  // blocked_poses / total_poses
  double max_run_length_m{0.0};  // longest consecutive blocked stretch (path distance)
  geometry_msgs::msg::Point centroid{};
  float extent_m{0.0f};          // approx blocked-region diameter
};

/**
 * @brief BT condition node that checks whether the active path corridor is blocked.
 *
 * Preferred input is the BT blackboard path="{path}" produced by ComputePathToPose.
 * The /plan subscription is only a fallback/debug source.
 *
 * Returns:
 *   SUCCESS: corridor clear, or blockage is minor (Nav2 can replan around it).
 *   FAILURE: significant blockage persists for debounce_ticks consecutive ticks.
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
   * @brief Pure helper exposed for unit tests.
   *
   * Samples the first lookahead_m metres of path against the occupancy grid
   * and returns raw blockage metrics. Severity gating (min_blocked_samples etc.)
   * is applied in tick(), not here.
   *
   * When sample_radius_m == 0.0, checks only the costmap cell containing each
   * plan pose (no distance filter applied).
   */
  static BlockageMetrics isCorridorBlocked(
    const nav_msgs::msg::Path & path,
    const nav_msgs::msg::OccupancyGrid & costmap,
    int lethal_threshold,
    double lookahead_m,
    double sample_radius_m);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
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
