// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include "semantic_nav_nav2_plugins/dynamic_wait.hpp"

#include <algorithm>
#include <chrono>

namespace semantic_nav_nav2_plugins
{

DynamicWait::DynamicWait(
  const std::string & name,
  const BT::NodeConfiguration & config)
: BT::StatefulActionNode(name, config)
{
  // Deliberately do not read ports here. directive_wait_seconds is populated
  // by EscalateToLLMRecovery after the tree has already been instantiated.
}

BT::PortsList DynamicWait::providedPorts()
{
  return {
    BT::InputPort<int>(
      "wait_duration",
      "Late-bound wait duration in seconds"),
    BT::InputPort<int>(
      "default_wait_duration", 5,
      "Fallback duration when wait_duration is unavailable or invalid"),
    BT::InputPort<int>(
      "max_wait_duration", 60,
      "Upper safety bound for the selected duration"),
    BT::OutputPort<int>(
      "selected_wait_seconds",
      "Duration selected by the node after validation and clamping")
  };
}

BT::NodeStatus DynamicWait::onStart()
{
  int fallback_seconds{5};
  int max_seconds{60};
  getInput("default_wait_duration", fallback_seconds);
  getInput("max_wait_duration", max_seconds);

  max_seconds = std::max(0, max_seconds);
  fallback_seconds = std::clamp(fallback_seconds, 0, max_seconds);

  int requested_seconds{fallback_seconds};
  const auto requested_result = getInput("wait_duration", requested_seconds);
  if (!requested_result || requested_seconds < 0) {
    requested_seconds = fallback_seconds;
  }

  selected_wait_seconds_ = std::clamp(requested_seconds, 0, max_seconds);
  setOutput("selected_wait_seconds", selected_wait_seconds_);

  deadline_ = std::chrono::steady_clock::now() +
    std::chrono::seconds(selected_wait_seconds_);

  if (selected_wait_seconds_ == 0) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus DynamicWait::onRunning()
{
  if (std::chrono::steady_clock::now() >= deadline_) {
    return BT::NodeStatus::SUCCESS;
  }
  return BT::NodeStatus::RUNNING;
}

void DynamicWait::onHalted()
{
  selected_wait_seconds_ = 0;
  deadline_ = std::chrono::steady_clock::time_point{};
}

}  // namespace semantic_nav_nav2_plugins
