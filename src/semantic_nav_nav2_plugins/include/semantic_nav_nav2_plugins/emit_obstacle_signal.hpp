// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

namespace semantic_nav_nav2_plugins
{

/**
 * @brief Publish one polite obstacle-clear signal for animate blockers.
 *
 * Modes:
 * - emit_enabled=false: returns FAILURE so the enclosing Fallback selects
 *   passive waiting;
 * - publish_signal=false: gate-only SUCCESS without publication;
 * - both true: publishes "polite_clear:<safety_class>" and logs at INFO.
 */
class EmitObstacleSignal : public BT::SyncActionNode
{
public:
  EmitObstacleSignal(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus tick() override;
  static BT::PortsList providedPorts();

private:
  rclcpp::Node::SharedPtr node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  std::string signal_topic_{"/robot_obstacle_signal"};
};

}  // namespace semantic_nav_nav2_plugins
