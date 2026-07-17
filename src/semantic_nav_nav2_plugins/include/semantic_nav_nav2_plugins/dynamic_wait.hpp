// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#pragma once

#include <chrono>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"

namespace semantic_nav_nav2_plugins
{

/**
 * @brief Non-blocking wait whose duration is read when the node is ticked.
 *
 * Nav2 Humble's standard Wait action reads wait_duration in its constructor.
 * That is unsafe for directive_wait_seconds because EscalateToLLMRecovery
 * writes the blackboard value later, during execution. DynamicWait defers all
 * input reads to onStart(), after the recovery directive exists.
 */
class DynamicWait : public BT::StatefulActionNode
{
public:
  DynamicWait(
    const std::string & name,
    const BT::NodeConfiguration & config);

  static BT::PortsList providedPorts();

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

private:
  std::chrono::steady_clock::time_point deadline_{};
  int selected_wait_seconds_{0};
};

}  // namespace semantic_nav_nav2_plugins
