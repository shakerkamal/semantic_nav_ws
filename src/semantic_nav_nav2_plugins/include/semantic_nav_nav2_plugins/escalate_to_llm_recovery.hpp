// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "semantic_nav_interfaces/srv/request_recovery.hpp"
#include "semantic_nav_nav2_plugins/robot_pose_util.hpp"

namespace semantic_nav_nav2_plugins
{

/**
 * @brief BT service node wrapping /request_recovery.
 *
 * Sends failure/blockage context to the orchestrator and writes normalized
 * directive fields to output ports. On retry_target it also updates the Nav2
 * blackboard key "goal" so the next RecoveryNode primary-child retry computes
 * a fresh path to the new target. Accepted give_up returns SUCCESS here so the
 * downstream Switch3 can visibly enter the ForceFailure branch; terminal_fail,
 * rejected, duplicate, timeout/no response, and unknown actions return FAILURE.
 */
class EscalateToLLMRecovery
  : public nav2_behavior_tree::BtServiceNode<
      semantic_nav_interfaces::srv::RequestRecovery>
{
public:
  using ServiceT = semantic_nav_interfaces::srv::RequestRecovery;
  using Base = nav2_behavior_tree::BtServiceNode<ServiceT>;

  EscalateToLLMRecovery(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  /// Guard missing robot/goal pose before issuing /request_recovery.
  BT::NodeStatus tick() override;

  void on_tick() override;

  BT::NodeStatus on_completion(
    std::shared_ptr<ServiceT::Response> response) override;

  static BT::PortsList providedPorts();

private:
  bool readRobotPose(geometry_msgs::msg::PoseStamped & robot_pose) const;

  geometry_msgs::msg::PoseStamped pending_robot_pose_;
};

}  // namespace semantic_nav_nav2_plugins
