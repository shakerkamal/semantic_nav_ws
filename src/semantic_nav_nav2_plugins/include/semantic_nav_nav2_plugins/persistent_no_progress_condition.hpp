// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#pragma once

#include <chrono>
#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/condition_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"

namespace semantic_nav_nav2_plugins
{

/**
 * @brief Deterministic obstacle evidence used by the execution-stall monitor.
 */
struct ExecutionObstacleEvidence
{
  bool blocked{false};
  int lethal_cells{0};
  geometry_msgs::msg::Point centroid{};
  float extent_m{0.0F};
  std::string source{"none"};

  // Diagnostic split of the forward-corridor cells: cost==100 is physical
  // obstacle evidence, cost==99 is the walls' inscribed-inflation band --
  // a planning artifact that must never count as an obstacle by itself
  // (S3 2026-07-17 false stall). Always populated, independent of the
  // threshold used for the blocking decision, so runtime logs show what
  // the counted cells actually were.
  int true_lethal_cells{0};
  int inscribed_cells{0};
};

/**
 * @brief Interrupt FollowPath when execution is persistently stalled in front
 * of local obstacle evidence, even though the global planner still returns a
 * nominally valid path.
 *
 * This is intentionally a BT condition node. Inside a ReactiveSequence it is
 * re-ticked before FollowPath on every BT cycle. It returns SUCCESS during
 * normal motion and FAILURE only when both conditions hold:
 *
 *   1. the robot has moved less than minimum_progress_m for
 *      observation_window_s; and
 *   2. local lethal-cost evidence has persisted for obstacle_persistence_s.
 *
 * On FAILURE, ReactiveSequence halts its running FollowPath child and lets the
 * existing outer RecoveryNode continue with geometric/semantic recovery.
 */
class PersistentNoProgressCondition : public BT::ConditionNode
{
public:
  PersistentNoProgressCondition(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

  /**
   * @brief Pure occupancy-grid helper exposed for unit tests.
   *
   * Evidence is accepted from either:
   * - an expanded corridor around the current Smac path; or
   * - a short forward corridor from the robot along the immediate path
   *   direction. The second test catches an obstacle that Smac has curved
   *   around mathematically but that the controller cannot physically pass.
   */
  static ExecutionObstacleEvidence detectObstacleEvidence(
    const nav_msgs::msg::Path & path,
    const nav_msgs::msg::OccupancyGrid & costmap,
    double robot_x,
    double robot_y,
    int lethal_threshold,
    double obstacle_lookahead_m,
    double path_sample_radius_m,
    double forward_lateral_tolerance_m,
    double min_forward_distance_m,
    int min_lethal_cells);

private:
  using SteadyClock = std::chrono::steady_clock;

  void resetProgressAnchor(
    double robot_x,
    double robot_y,
    const SteadyClock::time_point & now);

  void clearObstacleTimer();

  static bool goalChanged(
    const geometry_msgs::msg::PoseStamped & previous,
    const geometry_msgs::msg::PoseStamped & current,
    double position_tolerance_m);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

  std::mutex data_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;

  bool progress_anchor_valid_{false};
  double progress_anchor_x_{0.0};
  double progress_anchor_y_{0.0};
  SteadyClock::time_point progress_anchor_time_{};

  bool obstacle_timer_valid_{false};
  SteadyClock::time_point obstacle_since_{};

  bool previous_goal_valid_{false};
  geometry_msgs::msg::PoseStamped previous_goal_{};
};

}  // namespace semantic_nav_nav2_plugins
