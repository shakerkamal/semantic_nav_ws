// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "semantic_nav_nav2_plugins/path_clear_condition.hpp"

namespace semantic_nav_nav2_plugins
{

/**
 * @brief BT action node that samples the current path against the local costmap
 * and writes blockage_centroid / blockage_extent_m to the blackboard.
 *
 * Always returns SUCCESS — it is a data-capture node, not a gate.
 * If no path or costmap data is available the outputs are left unset.
 * Used as the first step of SemanticRecoveryBranch so QuerySemanticContext
 * receives accurate spatial context even though PathClearCondition is no
 * longer in the primary navigation sequence.
 */
class CaptureBlockageContext : public BT::SyncActionNode
{
public:
  CaptureBlockageContext(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;

  static BT::PortsList providedPorts();

  /**
   * @brief Geometric fallback centroid when path-sampling finds no lethal cell.
   *
   * The stale {path}'s first lookahead metres can sit behind the robot (an old
   * plan) or the blocker can fall outside the rolling local costmap, so
   * isCorridorBlocked returns any_blocked=false and the centroid is never set.
   * Rather than leave it at the default (0,0) — which sends the responsible
   * -object match to the map origin — anchor at the robot and step forward
   * ALONG THE PATH by lookahead_m. The blocker is on the path between the robot
   * and the goal, so this lands on/near it regardless of the robot's heading
   * (it may have rotated away during Tier-2 recovery). Empty path -> the robot's
   * own position (best effort).
   */
  static geometry_msgs::msg::Point fallbackCentroidAlongPath(
    const nav_msgs::msg::Path & path,
    double robot_x,
    double robot_y,
    double lookahead_m);

  /**
   * @brief Perception-grounded fallback: nearest lethal-cell cluster to the
   * robot's current position, read directly from the local costmap.
   *
   * When the path is completely empty (a fully-sealed corridor: the planner
   * cannot find ANY route, e.g. S2's closed door), fallbackCentroidAlongPath
   * has nothing to project along and would return the robot's raw pose --
   * which can be a couple of metres from the actual blocker after a Tier-2
   * backup moves the robot further away (found 2026-07-15, S2: robot at
   * (2.808,-0.116), true door at (4.862,-0.677), match found an unrelated
   * "trash bin" instead of the door). The costmap the robot is stopped in
   * front of already shows the real obstacle as lethal cells, so search
   * that directly instead of guessing geometrically. Returns false (leaves
   * out_centroid untouched) if no lethal cell is found within
   * search_radius_m or the costmap is malformed/empty -- callers fall back
   * further to fallbackCentroidAlongPath in that case.
   */
  static bool nearestLethalCentroidNearRobot(
    const nav_msgs::msg::OccupancyGrid & costmap,
    double robot_x,
    double robot_y,
    double search_radius_m,
    int lethal_threshold,
    geometry_msgs::msg::Point & out_centroid);

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

  std::mutex data_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
};

}  // namespace semantic_nav_nav2_plugins
