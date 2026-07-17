// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include "semantic_nav_nav2_plugins/emit_obstacle_signal.hpp"

namespace semantic_nav_nav2_plugins
{

EmitObstacleSignal::EmitObstacleSignal(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
  std::string topic{"/robot_obstacle_signal"};
  getInput("signal_topic", topic);
  signal_topic_ = topic;
  pub_ = node_->create_publisher<std_msgs::msg::String>(
    topic, rclcpp::SystemDefaultsQoS());
}

BT::NodeStatus EmitObstacleSignal::tick()
{
  bool emit_enabled{true};
  getInput("emit_enabled", emit_enabled);
  if (!emit_enabled) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[EmitObstacleSignal] signal branch disabled; passive wait selected");
    return BT::NodeStatus::FAILURE;
  }

  bool publish_signal{true};
  getInput("publish_signal", publish_signal);
  if (!publish_signal) {
    return BT::NodeStatus::SUCCESS;
  }

  std::string signal_class{"generic"};
  getInput("signal_class", signal_class);

  std_msgs::msg::String msg;
  msg.data = "polite_clear:" + signal_class;
  pub_->publish(msg);

  RCLCPP_INFO(
    node_->get_logger(),
    "[EmitObstacleSignal] published=true topic='%s' class='%s' payload='%s'",
    signal_topic_.c_str(),
    signal_class.c_str(),
    msg.data.c_str());
  return BT::NodeStatus::SUCCESS;
}

BT::PortsList EmitObstacleSignal::providedPorts()
{
  return {
    BT::InputPort<bool>(
      "emit_enabled", true,
      "If false, returns FAILURE so passive-wait branch can run"),
    BT::InputPort<bool>(
      "publish_signal", true,
      "If false, acts as a gate only and does not publish"),
    BT::InputPort<std::string>(
      "signal_class", "generic",
      "Safety class from responsible object"),
    BT::InputPort<std::string>(
      "signal_topic", "/robot_obstacle_signal",
      "Topic on which to publish the std_msgs/String signal"),
  };
}

}  // namespace semantic_nav_nav2_plugins
