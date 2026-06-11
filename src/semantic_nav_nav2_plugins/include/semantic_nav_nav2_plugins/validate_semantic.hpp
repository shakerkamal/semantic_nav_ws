// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#pragma once

#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "semantic_nav_interfaces/srv/validate_pose.hpp"

namespace semantic_nav_nav2_plugins
{

/**
 * @brief BT service node wrapping /validate_pose_goal with goal-keyed caching.
 *
 * ValidateSemantic lives inside the RecoveryNode primary child. It calls the
 * validator only when the goal changes; repeated PipelineSequence ticks for the
 * same goal reuse the cached result instead of generating repeated service
 * traffic. A validation failure returns FAILURE so the RecoveryNode enters its
 * recovery child.
 */
class ValidateSemantic
  : public nav2_behavior_tree::BtServiceNode<
      semantic_nav_interfaces::srv::ValidatePose>
{
public:
  using ServiceT = semantic_nav_interfaces::srv::ValidatePose;
  using Base = nav2_behavior_tree::BtServiceNode<ServiceT>;

  ValidateSemantic(
    const std::string & service_node_name,
    const BT::NodeConfiguration & conf);

  /// Return cached status for an unchanged goal, otherwise issue service call.
  BT::NodeStatus tick() override;

  /// Populate the ValidatePose request from the current pending goal.
  void on_tick() override;

  /// Map valid=true -> SUCCESS, valid=false -> FAILURE, and update cache.
  BT::NodeStatus on_completion(
    std::shared_ptr<ServiceT::Response> response) override;

  static BT::PortsList providedPorts();

private:
  bool readGoal(geometry_msgs::msg::PoseStamped & goal_pose) const;

  bool sameGoal(
    const geometry_msgs::msg::PoseStamped & a,
    const geometry_msgs::msg::PoseStamped & b) const;

  bool have_cached_goal_{false};
  bool cached_valid_{false};
  std::string cached_message_;
  geometry_msgs::msg::PoseStamped cached_goal_;
  geometry_msgs::msg::PoseStamped pending_goal_;
};

}  // namespace semantic_nav_nav2_plugins
