// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#pragma once

#include <chrono>
#include <future>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "semantic_nav_interfaces/srv/match_responsible_object.hpp"
#include "semantic_nav_interfaces/srv/refresh_local_objects.hpp"

namespace semantic_nav_nav2_plugins
{

/**
 * @brief BT stateful action node that queries local semantic context for recovery.
 *
 * Sequence:
 *   1. Wait briefly for /refresh_local_objects.
 *   2. Call /refresh_local_objects.
 *   3. Wait briefly for /match_responsible_object.
 *   4. Call /match_responsible_object.
 *   5. Write responsible_object_* outputs.
 *
 * On service unavailability, timeout, null response, or no match, this node writes
 * safe default outputs and returns SUCCESS so EscalateToLLMRecovery is still reached.
 */
class QuerySemanticContext : public BT::StatefulActionNode
{
public:
  using RefreshSrv = semantic_nav_interfaces::srv::RefreshLocalObjects;
  using MatchSrv = semantic_nav_interfaces::srv::MatchResponsibleObject;

  QuerySemanticContext(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts();

private:
  using RefreshClient = rclcpp::Client<RefreshSrv>;
  using MatchClient = rclcpp::Client<MatchSrv>;
  using RefreshFuture = typename RefreshClient::FutureAndRequestId;
  using MatchFuture = typename MatchClient::FutureAndRequestId;

  enum class Phase
  {
    kWaitRefreshService,
    kWaitRefreshResponse,
    kWaitMatchService,
    kWaitMatchResponse
  };

  BT::NodeStatus sendRefreshRequest();
  BT::NodeStatus sendMatchRequest();

  void writeDefaultOutputs();
  void writeDefaultObjectOutputs();
  void writeDbOutputsFromRefresh();
  bool readRobotPose(geometry_msgs::msg::PoseStamped & pose_out) const;
  void clearPendingRequests();

  rclcpp::Node::SharedPtr node_;
  RefreshClient::SharedPtr refresh_client_;
  MatchClient::SharedPtr match_client_;

  Phase phase_{Phase::kWaitRefreshService};
  std::chrono::steady_clock::time_point phase_deadline_;

  int service_ready_timeout_ms_{2000};
  int response_timeout_ms_{5000};

  std::optional<RefreshFuture> refresh_future_;
  std::optional<MatchFuture> match_future_;
  RefreshSrv::Response::SharedPtr refresh_response_;
};

}  // namespace semantic_nav_nav2_plugins