// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "rclcpp/rclcpp.hpp"
#include "semantic_nav_interfaces/srv/operator_decision.hpp"
#include "std_msgs/msg/string.hpp"

namespace semantic_nav_nav2_plugins
{

/**
 * @brief BT stateful action for operator acknowledgement and action completion.
 *
 * The OperatorDecision service records the operator's decision. When
 * wait_for_action_completion is false, acknowledgement preserves the legacy
 * behavior and returns SUCCESS immediately.
 *
 * When wait_for_action_completion is true, acknowledgement publishes a keyed
 * action request and this node remains RUNNING until the identical token is
 * received on action_completion_topic. This prevents costmap clearing and
 * replanning before the requested environmental action has completed.
 *
 * Token format:
 *   recovery_event_id|responsible_object_key|directive_action
 *
 * Phases:
 *   kWaitService          - bounded service availability polling.
 *   kWaitResponse         - bounded OperatorDecision response wait.
 *   kWaitActionCompletion - bounded keyed environmental-action wait.
 *
 * Uses FutureAndRequestId so pending Humble service requests are removed when
 * the node is halted.
 */
class OperatorPrompt : public BT::StatefulActionNode
{
public:
  using ServiceT = semantic_nav_interfaces::srv::OperatorDecision;
  using OperatorClient = rclcpp::Client<ServiceT>;
  using OperatorFuture = typename OperatorClient::FutureAndRequestId;

  OperatorPrompt(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts();

private:
  enum class Phase
  {
    kWaitService,
    kWaitResponse,
    kWaitActionCompletion
  };

  void onActionCompletion(const std_msgs::msg::String::SharedPtr msg);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  OperatorClient::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr confirmed_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr action_request_pub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr action_completion_sub_;

  Phase phase_{Phase::kWaitService};
  std::chrono::steady_clock::time_point phase_deadline_;

  int service_ready_timeout_ms_{2000};
  int response_timeout_ms_{120000};
  int action_completion_timeout_ms_{30000};

  bool wait_for_action_completion_{false};
  bool action_completed_{false};
  std::string expected_action_token_;

  std::optional<OperatorFuture> future_;
};

}  // namespace semantic_nav_nav2_plugins
