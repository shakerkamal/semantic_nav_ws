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

  // bt_navigator's client node is not added to an executor. Keep the service
  // future and completion subscription in a dedicated callback group that is
  // explicitly spun from onRunning().
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

  std::string action_request_topic{"/operator_action_request"};
  getInput("action_request_topic", action_request_topic);

  rclcpp::QoS action_qos(rclcpp::KeepLast(10));
  action_qos.reliable();
  action_qos.transient_local();

  action_request_pub_ = node_->create_publisher<std_msgs::msg::String>(
    action_request_topic, action_qos);

  std::string action_completion_topic{"/operator_action_completion"};
  getInput("action_completion_topic", action_completion_topic);

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;
  action_completion_sub_ = node_->create_subscription<std_msgs::msg::String>(
    action_completion_topic,
    action_qos,
    [this](const std_msgs::msg::String::SharedPtr msg) {
      onActionCompletion(msg);
    },
    subscription_options);
}

BT::NodeStatus OperatorPrompt::onStart()
{
  if (future_) {
    client_->remove_pending_request(*future_);
    future_.reset();
  }

  service_ready_timeout_ms_ = 2000;
  response_timeout_ms_ = 120000;
  action_completion_timeout_ms_ = 30000;
  wait_for_action_completion_ = false;
  action_completed_ = false;
  expected_action_token_.clear();

  getInput("service_ready_timeout_ms", service_ready_timeout_ms_);
  getInput("response_timeout_ms", response_timeout_ms_);
  getInput("action_completion_timeout_ms", action_completion_timeout_ms_);
  getInput("wait_for_action_completion", wait_for_action_completion_);

  service_ready_timeout_ms_ = std::max(0, service_ready_timeout_ms_);
  response_timeout_ms_ = std::max(1, response_timeout_ms_);
  action_completion_timeout_ms_ = std::max(1, action_completion_timeout_ms_);

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

      if (
        future_->future.wait_for(std::chrono::milliseconds(0)) !=
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

        if (!response->acknowledged) {
          return BT::NodeStatus::FAILURE;
        }

        std::string responsible_object_key;
        getInput("responsible_object_key", responsible_object_key);

        // Keep the existing acknowledgement trace topic for compatibility.
        std_msgs::msg::String confirmed_msg;
        confirmed_msg.data = responsible_object_key;
        confirmed_pub_->publish(confirmed_msg);

        if (!wait_for_action_completion_) {
          return BT::NodeStatus::SUCCESS;
        }

        std::string recovery_event_id;
        std::string directive_action;
        getInput("recovery_event_id", recovery_event_id);
        getInput("directive_action", directive_action);

        const auto invalid_token_part = [](const std::string & value) {
            return value.empty() || value.find('|') != std::string::npos;
          };

        if (
          invalid_token_part(recovery_event_id) ||
          invalid_token_part(responsible_object_key) ||
          invalid_token_part(directive_action))
        {
          RCLCPP_WARN(
            node_->get_logger(),
            "[OperatorPrompt] cannot wait for action completion: "
            "event_id, object_key, and directive_action must be non-empty "
            "and must not contain '|'");
          return BT::NodeStatus::FAILURE;
        }

        expected_action_token_ =
          recovery_event_id + "|" +
          responsible_object_key + "|" +
          directive_action;
        action_completed_ = false;

        std_msgs::msg::String action_request;
        action_request.data = expected_action_token_;
        action_request_pub_->publish(action_request);

        phase_ = Phase::kWaitActionCompletion;
        phase_deadline_ =
          std::chrono::steady_clock::now() +
          std::chrono::milliseconds(action_completion_timeout_ms_);

        RCLCPP_INFO(
          node_->get_logger(),
          "[OperatorPrompt] action_requested token='%s' "
          "completion_timeout_ms=%d",
          expected_action_token_.c_str(),
          action_completion_timeout_ms_);

        return BT::NodeStatus::RUNNING;
      }

    case Phase::kWaitActionCompletion:
      if (action_completed_) {
        RCLCPP_INFO(
          node_->get_logger(),
          "[OperatorPrompt] action_completed token='%s'",
          expected_action_token_.c_str());

        expected_action_token_.clear();
        action_completed_ = false;
        return BT::NodeStatus::SUCCESS;
      }

      if (now < phase_deadline_) {
        return BT::NodeStatus::RUNNING;
      }

      RCLCPP_WARN(
        node_->get_logger(),
        "[OperatorPrompt] action completion timeout token='%s'",
        expected_action_token_.c_str());

      expected_action_token_.clear();
      action_completed_ = false;
      return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::FAILURE;
}

void OperatorPrompt::onActionCompletion(
  const std_msgs::msg::String::SharedPtr msg)
{
  if (!msg || expected_action_token_.empty()) {
    return;
  }

  if (msg->data != expected_action_token_) {
    RCLCPP_DEBUG(
      node_->get_logger(),
      "[OperatorPrompt] ignoring unrelated action completion token='%s'",
      msg->data.c_str());
    return;
  }

  action_completed_ = true;
}

void OperatorPrompt::onHalted()
{
  if (future_) {
    client_->remove_pending_request(*future_);
    future_.reset();
  }

  phase_ = Phase::kWaitService;
  action_completed_ = false;
  expected_action_token_.clear();
}

BT::PortsList OperatorPrompt::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "service_name", "/operator_decision",
      "OperatorDecision service name"),
    BT::InputPort<std::string>(
      "prompt_text", "",
      "Human-readable prompt for the operator"),
    BT::InputPort<std::string>(
      "responsible_object_key", "",
      "Object key for traceability and completion correlation"),
    BT::InputPort<std::string>(
      "failure_stage", "execution",
      "BT failure stage for logging"),
    BT::InputPort<std::string>(
      "directive_action", "",
      "Directive action and completion-correlation token field"),
    BT::InputPort<std::string>(
      "recovery_event_id", "",
      "Recovery event ID and completion-correlation token field"),
    BT::InputPort<int>(
      "service_ready_timeout_ms", 2000,
      "Max wait for service availability (ms)"),
    BT::InputPort<int>(
      "response_timeout_ms", 120000,
      "Max wait for operator response (ms)"),
    BT::InputPort<bool>(
      "wait_for_action_completion", false,
      "Keep RUNNING until the keyed environmental action completes"),
    BT::InputPort<int>(
      "action_completion_timeout_ms", 30000,
      "Max wait for keyed environmental-action completion (ms)"),
    BT::InputPort<std::string>(
      "action_request_topic", "/operator_action_request",
      "Publishes event_id|object_key|directive_action after acknowledgement"),
    BT::InputPort<std::string>(
      "action_completion_topic", "/operator_action_completion",
      "Receives the identical token after the environmental action completes"),
    BT::InputPort<std::string>(
      "confirmed_object_topic", "/operator_confirmed_object",
      "Legacy trace topic carrying responsible_object_key on acknowledgement"),
  };
}

}  // namespace semantic_nav_nav2_plugins
