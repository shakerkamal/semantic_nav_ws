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
 * @brief BT sync-action node that publishes one obstacle-signal message.
 *
 * Modes:
 *  - emit_enabled=false:
 *      returns FAILURE so the enclosing Sequence fails and the outer Fallback
 *      can take the passive-wait branch.
 *
 *  - emit_enabled=true, publish_signal=false:
 *      gate-only mode; returns SUCCESS without publishing.
 *
 *  - emit_enabled=true, publish_signal=true:
 *      publishes "polite_clear:<signal_class>" on signal_topic and returns SUCCESS.
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
};

}  // namespace semantic_nav_nav2_plugins