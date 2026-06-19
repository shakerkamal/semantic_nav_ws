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

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_sub_;

  std::mutex data_mutex_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_costmap_;
};

}  // namespace semantic_nav_nav2_plugins
