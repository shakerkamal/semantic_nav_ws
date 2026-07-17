// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "rclcpp/rclcpp.hpp"
#include "semantic_nav_interfaces/msg/object_instance.hpp"
#include "semantic_nav_interfaces/srv/refresh_local_objects.hpp"

namespace semantic_nav_nav2_plugins
{

/**
 * @brief Wait until the same perceived animate obstacle no longer overlaps the
 * original blocked region.
 *
 * The node is intentionally class-agnostic. A person, dog, cat, or another
 * dynamic object is handled from the exact object key and current bbox. The
 * safety class determines the recovery policy elsewhere; it does not determine
 * a hard-coded radius here.
 */
class WaitForDynamicObstacleDeparture : public BT::StatefulActionNode
{
public:
  using RefreshService = semantic_nav_interfaces::srv::RefreshLocalObjects;
  using RefreshClient = rclcpp::Client<RefreshService>;
  using RefreshFuture = typename RefreshClient::FutureAndRequestId;
  using ObjectInstance = semantic_nav_interfaces::msg::ObjectInstance;

  struct AxisAlignedFootprint
  {
    geometry_msgs::msg::Point center{};
    double half_x{0.0};
    double half_y{0.0};
    bool valid{false};
  };

  WaitForDynamicObstacleDeparture(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts();

  static AxisAlignedFootprint makeFootprint(
    const geometry_msgs::msg::Point & center,
    const geometry_msgs::msg::Vector3 & extent,
    double fallback_radius_m,
    double padding_m);

  static bool footprintsOverlap(
    const AxisAlignedFootprint & lhs,
    const AxisAlignedFootprint & rhs);

  static bool footprintOverlapsCircle(
    const AxisAlignedFootprint & footprint,
    const geometry_msgs::msg::Point & circle_center,
    double circle_radius_m);

  static bool objectStillBlocks(
    const ObjectInstance & object,
    const AxisAlignedFootprint & original_object_region,
    const geometry_msgs::msg::Point & original_blockage_center,
    double original_blockage_radius_m,
    double current_object_padding_m);

private:
  enum class Phase
  {
    kWaitService,
    kWaitResponse,
    kWaitPollInterval
  };

  BT::NodeStatus startRefreshRequest();
  BT::NodeStatus evaluateRefreshResponse();
  void scheduleNextPoll(const std::string & status);
  void clearPendingRequest();
  bool readRobotPose(geometry_msgs::msg::PoseStamped & pose_out) const;
  bool objectObservationExpired(const ObjectInstance & object) const;
  void publishOutputs(const std::string & status);

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;
  RefreshClient::SharedPtr refresh_client_;
  std::optional<RefreshFuture> refresh_future_;

  Phase phase_{Phase::kWaitService};
  std::chrono::steady_clock::time_point phase_deadline_{};
  std::chrono::steady_clock::time_point overall_deadline_{};

  std::string responsible_object_key_;
  geometry_msgs::msg::Point original_blockage_center_{};
  geometry_msgs::msg::Point original_bbox_center_{};
  geometry_msgs::msg::Vector3 original_bbox_extent_{};
  AxisAlignedFootprint original_object_region_{};

  double original_blockage_radius_m_{0.20};
  double fallback_dynamic_radius_m_{0.30};
  double footprint_padding_m_{0.05};
  double current_object_padding_m_{0.02};
  double poll_interval_s_{0.5};
  double timeout_s_{30.0};
  float query_radius_m_{4.0F};
  int required_nonblocking_samples_{3};
  int service_ready_timeout_ms_{2000};
  int response_timeout_ms_{3000};

  int nonblocking_streak_{0};
  std::string departure_status_{"not_started"};
};

}  // namespace semantic_nav_nav2_plugins
