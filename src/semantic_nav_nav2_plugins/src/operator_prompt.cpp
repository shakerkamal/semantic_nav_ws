// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include "semantic_nav_nav2_plugins/operator_prompt.hpp"

#include <algorithm>
#include <string>

namespace semantic_nav_nav2_plugins
{

OperatorPrompt::OperatorPrompt(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string service_name{"/operator_decision"};
  getInput("service_name", service_name);

  // Dedicated callback group + executor, spun from onRunning(): bt_navigator's
  // client node is never added to any executor, so a client on its default
  // callback group would never resolve its future -- the prompt would time out
  // even when the operator answered (QuerySemanticContext pattern).
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  client_ = node_->create_client<ServiceT>(
    service_name, rmw_qos_profile_services_default, callback_group_);

  std::string confirmed_topic{"/operator_confirmed_object"};
  getInput("confirmed_object_topic", confirmed_topic);
  confirmed_pub_ = node_->create_publisher<std_msgs::msg::String>(
    confirmed_topic, rclcpp::QoS(10));
}

BT::NodeStatus OperatorPrompt::onStart()
{
  if (future_) {
    client_->remove_pending_request(*future_);
    future_.reset();
  }

  service_ready_timeout_ms_ = 2000;
  response_timeout_ms_ = 120000;
  getInput("service_ready_timeout_ms", service_ready_timeout_ms_);
  getInput("response_timeout_ms", response_timeout_ms_);
  service_ready_timeout_ms_ = std::max(0, service_ready_timeout_ms_);
  response_timeout_ms_ = std::max(1, response_timeout_ms_);

  phase_ = Phase::kWaitService;
  phase_deadline_ =
    std::chrono::steady_clock::now() +
    std::chrono::milliseconds(service_ready_timeout_ms_);

  return onRunning();
}

BT::NodeStatus OperatorPrompt::onRunning()
{
  callback_group_executor_.spin_some(std::chrono::nanoseconds(0));

  const auto now = std::chrono::steady_clock::now();

  switch (phase_) {
    case Phase::kWaitService:
      if (!client_->service_is_ready()) {
        if (now < phase_deadline_) {
          return BT::NodeStatus::RUNNING;
        }
        RCLCPP_WARN(
          node_->get_logger(),
          "[OperatorPrompt] /operator_decision not ready before timeout");
        return BT::NodeStatus::FAILURE;
      }
      {
        auto request = std::make_shared<ServiceT::Request>();
        getInput("prompt_text", request->prompt_text);
        getInput("responsible_object_key", request->responsible_object_key);
        getInput("failure_stage", request->failure_stage);
        getInput("directive_action", request->directive_action);
        getInput("recovery_event_id", request->recovery_event_id);

        future_ = client_->async_send_request(request);
        phase_ = Phase::kWaitResponse;
        phase_deadline_ =
          std::chrono::steady_clock::now() +
          std::chrono::milliseconds(response_timeout_ms_);
        return BT::NodeStatus::RUNNING;
      }

    case Phase::kWaitResponse:
      if (!future_) {
        return BT::NodeStatus::FAILURE;
      }

      if (future_->future.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready)
      {
        if (now < phase_deadline_) {
          return BT::NodeStatus::RUNNING;
        }
        client_->remove_pending_request(*future_);
        future_.reset();
        RCLCPP_WARN(
          node_->get_logger(),
          "[OperatorPrompt] operator_decision response timeout");
        return BT::NodeStatus::FAILURE;
      }
      {
        auto response = future_->future.get();
        future_.reset();

        if (!response) {
          RCLCPP_WARN(
            node_->get_logger(),
            "[OperatorPrompt] operator_decision returned null response");
          return BT::NodeStatus::FAILURE;
        }

        RCLCPP_INFO(
          node_->get_logger(),
          "[OperatorPrompt] acknowledged=%s note='%s'",
          response->acknowledged ? "true" : "false",
          response->operator_note.c_str());

        if (response->acknowledged) {
          std::string responsible_object_key;
          getInput("responsible_object_key", responsible_object_key);
          std_msgs::msg::String confirmed_msg;
          confirmed_msg.data = responsible_object_key;
          confirmed_pub_->publish(confirmed_msg);
        }

        return response->acknowledged ?
          BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
      }
  }

  return BT::NodeStatus::FAILURE;
}

void OperatorPrompt::onHalted()
{
  if (future_) {
    client_->remove_pending_request(*future_);
    future_.reset();
  }
  phase_ = Phase::kWaitService;
}

BT::PortsList OperatorPrompt::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "service_name", "/operator_decision", "OperatorDecision service name"),
    BT::InputPort<std::string>(
      "prompt_text", "", "Human-readable prompt for the operator"),
    BT::InputPort<std::string>(
      "responsible_object_key", "", "Object key for traceability"),
    BT::InputPort<std::string>(
      "failure_stage", "execution", "BT failure stage for logging"),
    BT::InputPort<std::string>(
      "directive_action", "", "Directive action for JSONL traceability"),
    BT::InputPort<std::string>(
      "recovery_event_id", "", "Recovery event ID for JSONL traceability"),
    BT::InputPort<int>(
      "service_ready_timeout_ms", 2000, "Max wait for service availability (ms)"),
    BT::InputPort<int>(
      "response_timeout_ms", 120000, "Max wait for operator response (ms)"),
    BT::InputPort<std::string>(
      "confirmed_object_topic", "/operator_confirmed_object",
      "Published with responsible_object_key when acknowledged=true -- the "
      "seam eval-only tooling (e.g. a blockage trigger script) can subscribe "
      "to for simulation-specific follow-up actions (deleting a spawned "
      "obstacle), without coupling the operator-decision interface itself "
      "to simulation concerns"),
  };
}

}  // namespace semantic_nav_nav2_plugins
