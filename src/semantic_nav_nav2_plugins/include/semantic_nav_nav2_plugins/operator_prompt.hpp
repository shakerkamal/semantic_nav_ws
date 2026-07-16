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
 * @brief BT stateful action that sends an OperatorDecision service request.
 *
 * Blocks the BT tick (not the ROS spin) until the operator acknowledges or
 * the response_timeout_ms fires. Returns SUCCESS on acknowledgement, FAILURE
 * on rejection, timeout, service unavailability, or null response.
 *
 * Phases:
 *   kWaitService  — poll service_is_ready() up to service_ready_timeout_ms.
 *   kWaitResponse — wait for the async response up to response_timeout_ms.
 *
 * Uses std::optional<FutureAndRequestId> so pending requests can be cancelled
 * on halt, matching the established QuerySemanticContext pattern.
 *
 * On acknowledged=true, ALSO publishes responsible_object_key to
 * confirmed_object_topic. The service response itself (OperatorDecision.srv)
 * has no way to signal a simulation-specific action (e.g. deleting a spawned
 * Gazebo obstacle) -- that would wrongly couple the operator-decision
 * interface (also used by a real deployment with no Gazebo at all) to
 * simulation concerns. This publish is the seam eval-only tooling can
 * subscribe to instead, without touching the operator interface itself.
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
    kWaitResponse
  };

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  OperatorClient::SharedPtr client_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr confirmed_pub_;

  Phase phase_{Phase::kWaitService};
  std::chrono::steady_clock::time_point phase_deadline_;

  int service_ready_timeout_ms_{2000};
  int response_timeout_ms_{120000};

  std::optional<OperatorFuture> future_;
};

}  // namespace semantic_nav_nav2_plugins
