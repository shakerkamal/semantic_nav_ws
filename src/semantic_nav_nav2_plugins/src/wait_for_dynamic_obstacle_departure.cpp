// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include "semantic_nav_nav2_plugins/wait_for_dynamic_obstacle_departure.hpp"

#include <algorithm>
#include <cmath>
#include <future>

#include "semantic_nav_nav2_plugins/robot_pose_util.hpp"

namespace semantic_nav_nav2_plugins
{
namespace
{

std::chrono::steady_clock::duration secondsDuration(double seconds)
{
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(std::max(0.0, seconds)));
}

bool finitePoint(const geometry_msgs::msg::Point & point)
{
  return std::isfinite(point.x) && std::isfinite(point.y);
}

bool finiteExtent(const geometry_msgs::msg::Vector3 & extent)
{
  return std::isfinite(extent.x) && std::isfinite(extent.y);
}

}  // namespace

WaitForDynamicObstacleDeparture::WaitForDynamicObstacleDeparture(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  std::string service_name{"/refresh_local_objects"};
  getInput("refresh_service", service_name);
  refresh_client_ = node_->create_client<RefreshService>(
    service_name,
    rmw_qos_profile_services_default,
    callback_group_);
}

BT::NodeStatus WaitForDynamicObstacleDeparture::onStart()
{
  clearPendingRequest();

  responsible_object_key_.clear();
  original_blockage_center_ = geometry_msgs::msg::Point();
  original_bbox_center_ = geometry_msgs::msg::Point();
  original_bbox_extent_ = geometry_msgs::msg::Vector3();

  fallback_dynamic_radius_m_ = 0.30;
  footprint_padding_m_ = 0.05;
  current_object_padding_m_ = 0.02;
  original_blockage_radius_m_ = 0.20;
  poll_interval_s_ = 0.5;
  timeout_s_ = 30.0;
  query_radius_m_ = 4.0F;
  required_nonblocking_samples_ = 3;
  service_ready_timeout_ms_ = 2000;
  response_timeout_ms_ = 3000;

  getInput("responsible_object_key", responsible_object_key_);
  getInput("original_blockage_center", original_blockage_center_);
  getInput("original_bbox_center", original_bbox_center_);
  getInput("original_bbox_extent", original_bbox_extent_);

  float original_blockage_extent_m{0.0F};
  getInput("original_blockage_extent_m", original_blockage_extent_m);
  getInput("fallback_dynamic_radius_m", fallback_dynamic_radius_m_);
  getInput("footprint_padding_m", footprint_padding_m_);
  getInput("current_object_padding_m", current_object_padding_m_);
  getInput("poll_interval_s", poll_interval_s_);
  getInput("timeout_s", timeout_s_);
  getInput("query_radius_m", query_radius_m_);
  getInput("required_nonblocking_samples", required_nonblocking_samples_);
  getInput("service_ready_timeout_ms", service_ready_timeout_ms_);
  getInput("response_timeout_ms", response_timeout_ms_);

  fallback_dynamic_radius_m_ = std::max(0.05, fallback_dynamic_radius_m_);
  footprint_padding_m_ = std::max(0.0, footprint_padding_m_);
  current_object_padding_m_ = std::max(0.0, current_object_padding_m_);
  original_blockage_radius_m_ = std::max(
    0.15,
    0.5 * std::max(0.0, static_cast<double>(original_blockage_extent_m)) + 0.05);
  poll_interval_s_ = std::max(0.05, poll_interval_s_);
  timeout_s_ = std::max(poll_interval_s_, timeout_s_);
  query_radius_m_ = std::max(0.5F, query_radius_m_);
  required_nonblocking_samples_ = std::max(1, required_nonblocking_samples_);
  service_ready_timeout_ms_ = std::max(0, service_ready_timeout_ms_);
  response_timeout_ms_ = std::max(1, response_timeout_ms_);

  if (responsible_object_key_.empty()) {
    departure_status_ = "missing_responsible_object_key";
    publishOutputs(departure_status_);
    RCLCPP_ERROR(
      node_->get_logger(),
      "[WaitForDynamicObstacleDeparture] responsible_object_key is empty; "
      "refusing to infer departure from anonymous occupancy");
    return BT::NodeStatus::FAILURE;
  }

  original_object_region_ = makeFootprint(
    original_bbox_center_,
    original_bbox_extent_,
    fallback_dynamic_radius_m_,
    footprint_padding_m_);

  nonblocking_streak_ = 0;
  departure_status_ = "waiting_for_refresh_service";
  publishOutputs(departure_status_);

  const auto now = std::chrono::steady_clock::now();
  phase_ = Phase::kWaitService;
  phase_deadline_ = now + std::chrono::milliseconds(service_ready_timeout_ms_);
  overall_deadline_ = now + secondsDuration(timeout_s_);

  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForDynamicObstacleDeparture] started key='%s' "
    "original_bbox_center=(%.3f,%.3f) half_extents=(%.3f,%.3f) "
    "blockage_center=(%.3f,%.3f) blockage_radius=%.3fm "
    "required_clear_samples=%d timeout=%.1fs",
    responsible_object_key_.c_str(),
    original_object_region_.center.x,
    original_object_region_.center.y,
    original_object_region_.half_x,
    original_object_region_.half_y,
    original_blockage_center_.x,
    original_blockage_center_.y,
    original_blockage_radius_m_,
    required_nonblocking_samples_,
    timeout_s_);

  return onRunning();
}

BT::NodeStatus WaitForDynamicObstacleDeparture::onRunning()
{
  callback_group_executor_.spin_some(std::chrono::nanoseconds(0));
  const auto now = std::chrono::steady_clock::now();

  if (now >= overall_deadline_) {
    clearPendingRequest();
    departure_status_ = "departure_timeout";
    publishOutputs(departure_status_);
    RCLCPP_WARN(
      node_->get_logger(),
      "[WaitForDynamicObstacleDeparture] object '%s' remained blocking or "
      "unconfirmed for %.1fs",
      responsible_object_key_.c_str(), timeout_s_);
    return BT::NodeStatus::FAILURE;
  }

  switch (phase_) {
    case Phase::kWaitService:
      if (!refresh_client_->service_is_ready()) {
        if (now < phase_deadline_) {
          return BT::NodeStatus::RUNNING;
        }
        scheduleNextPoll("refresh_service_unavailable_retrying");
        return BT::NodeStatus::RUNNING;
      }
      return startRefreshRequest();

    case Phase::kWaitResponse:
      return evaluateRefreshResponse();

    case Phase::kWaitPollInterval:
      if (now < phase_deadline_) {
        return BT::NodeStatus::RUNNING;
      }
      phase_ = Phase::kWaitService;
      phase_deadline_ = now + std::chrono::milliseconds(service_ready_timeout_ms_);
      return BT::NodeStatus::RUNNING;
  }

  return BT::NodeStatus::FAILURE;
}

void WaitForDynamicObstacleDeparture::onHalted()
{
  clearPendingRequest();
  nonblocking_streak_ = 0;
  departure_status_ = "halted";
  publishOutputs(departure_status_);
}

BT::PortsList WaitForDynamicObstacleDeparture::providedPorts()
{
  return {
    BT::InputPort<std::string>(
      "responsible_object_key", "", "Exact dynamic object key to track"),
    BT::InputPort<geometry_msgs::msg::Point>(
      "original_blockage_center", "Measured blockage center in map frame"),
    BT::InputPort<float>(
      "original_blockage_extent_m", 0.0F,
      "Measured blockage extent used for the original occupied region"),
    BT::InputPort<geometry_msgs::msg::Point>(
      "original_bbox_center", "Original matched object bbox center"),
    BT::InputPort<geometry_msgs::msg::Vector3>(
      "original_bbox_extent", "Original matched object bbox extent"),
    BT::InputPort<double>(
      "fallback_dynamic_radius_m", 0.30,
      "Class-agnostic fallback radius used only when bbox geometry is missing"),
    BT::InputPort<double>(
      "footprint_padding_m", 0.05,
      "Padding applied to the original dynamic-object bbox"),
    BT::InputPort<double>(
      "current_object_padding_m", 0.02,
      "Small padding applied to each refreshed object bbox"),
    BT::InputPort<double>("poll_interval_s", 0.5, "Refresh poll interval"),
    BT::InputPort<double>("timeout_s", 30.0, "Maximum departure wait"),
    BT::InputPort<float>(
      "query_radius_m", 4.0F, "RefreshLocalObjects query radius"),
    BT::InputPort<int>(
      "required_nonblocking_samples", 3,
      "Consecutive absent/non-overlapping observations required"),
    BT::InputPort<std::string>(
      "refresh_service", "/refresh_local_objects",
      "RefreshLocalObjects service name"),
    BT::InputPort<std::string>("global_frame", "map", "Robot pose frame"),
    BT::InputPort<std::string>(
      "robot_base_frame", "base_footprint", "Robot base frame"),
    BT::InputPort<double>(
      "transform_tolerance_s", 0.1, "TF lookup tolerance"),
    BT::InputPort<int>(
      "service_ready_timeout_ms", 2000,
      "Bounded service-readiness wait per poll"),
    BT::InputPort<int>(
      "response_timeout_ms", 3000,
      "Bounded RefreshLocalObjects response wait"),
    BT::OutputPort<int>(
      "departure_clear_samples", "Current consecutive non-blocking samples"),
    BT::OutputPort<std::string>(
      "departure_status", "Current/final departure diagnosis"),
  };
}

WaitForDynamicObstacleDeparture::AxisAlignedFootprint
WaitForDynamicObstacleDeparture::makeFootprint(
  const geometry_msgs::msg::Point & center,
  const geometry_msgs::msg::Vector3 & extent,
  double fallback_radius_m,
  double padding_m)
{
  AxisAlignedFootprint footprint;
  footprint.center = center;
  fallback_radius_m = std::max(0.05, fallback_radius_m);
  padding_m = std::max(0.0, padding_m);

  if (!finitePoint(center)) {
    return footprint;
  }

  if (finiteExtent(extent) && std::abs(extent.x) > 0.0 && std::abs(extent.y) > 0.0) {
    footprint.half_x = 0.5 * std::abs(extent.x) + padding_m;
    footprint.half_y = 0.5 * std::abs(extent.y) + padding_m;
  } else {
    footprint.half_x = fallback_radius_m;
    footprint.half_y = fallback_radius_m;
  }
  footprint.valid = true;
  return footprint;
}

bool WaitForDynamicObstacleDeparture::footprintsOverlap(
  const AxisAlignedFootprint & lhs,
  const AxisAlignedFootprint & rhs)
{
  if (!lhs.valid || !rhs.valid) {
    return true;  // Conservative: unknown geometry is still blocking.
  }
  return std::abs(lhs.center.x - rhs.center.x) <= lhs.half_x + rhs.half_x &&
         std::abs(lhs.center.y - rhs.center.y) <= lhs.half_y + rhs.half_y;
}

bool WaitForDynamicObstacleDeparture::footprintOverlapsCircle(
  const AxisAlignedFootprint & footprint,
  const geometry_msgs::msg::Point & circle_center,
  double circle_radius_m)
{
  if (!footprint.valid || !finitePoint(circle_center)) {
    return true;
  }
  circle_radius_m = std::max(0.0, circle_radius_m);
  const double dx = std::max(
    std::abs(circle_center.x - footprint.center.x) - footprint.half_x, 0.0);
  const double dy = std::max(
    std::abs(circle_center.y - footprint.center.y) - footprint.half_y, 0.0);
  return std::hypot(dx, dy) <= circle_radius_m;
}

bool WaitForDynamicObstacleDeparture::objectStillBlocks(
  const ObjectInstance & object,
  const AxisAlignedFootprint & original_object_region,
  const geometry_msgs::msg::Point & original_blockage_center,
  double original_blockage_radius_m,
  double current_object_padding_m)
{
  const auto current = makeFootprint(
    object.bbox_center,
    object.bbox_extent,
    0.20,
    current_object_padding_m);
  return footprintsOverlap(current, original_object_region) ||
         footprintOverlapsCircle(
           current, original_blockage_center, original_blockage_radius_m);
}

BT::NodeStatus WaitForDynamicObstacleDeparture::startRefreshRequest()
{
  auto request = std::make_shared<RefreshService::Request>();
  if (!readRobotPose(request->robot_pose)) {
    scheduleNextPoll("robot_pose_unavailable_retrying");
    return BT::NodeStatus::RUNNING;
  }

  request->blockage_centroid = original_blockage_center_;
  request->radius_m = query_radius_m_;
  request->semantic_map_id = "";
  request->base_map_version = "";

  refresh_future_ = refresh_client_->async_send_request(request);
  phase_ = Phase::kWaitResponse;
  phase_deadline_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(response_timeout_ms_);
  departure_status_ = "refresh_requested";
  publishOutputs(departure_status_);
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForDynamicObstacleDeparture::evaluateRefreshResponse()
{
  if (!refresh_future_) {
    scheduleNextPoll("refresh_future_missing_retrying");
    return BT::NodeStatus::RUNNING;
  }

  const auto now = std::chrono::steady_clock::now();
  if (refresh_future_->future.wait_for(std::chrono::milliseconds(0)) !=
      std::future_status::ready)
  {
    if (now < phase_deadline_) {
      return BT::NodeStatus::RUNNING;
    }
    refresh_client_->remove_pending_request(*refresh_future_);
    refresh_future_.reset();
    scheduleNextPoll("refresh_response_timeout_retrying");
    return BT::NodeStatus::RUNNING;
  }

  const auto response = refresh_future_->future.get();
  refresh_future_.reset();
  if (!response) {
    scheduleNextPoll("refresh_null_response_retrying");
    return BT::NodeStatus::RUNNING;
  }

  const ObjectInstance * matched = nullptr;
  for (const auto & object : response->objects) {
    if (object.object_key == responsible_object_key_) {
      matched = &object;
      break;
    }
  }

  bool blocking = false;
  std::string evidence;
  if (matched == nullptr) {
    evidence = "object_absent";
  } else if (objectObservationExpired(*matched)) {
    evidence = "object_ttl_expired";
  } else {
    blocking = objectStillBlocks(
      *matched,
      original_object_region_,
      original_blockage_center_,
      original_blockage_radius_m_,
      current_object_padding_m_);
    evidence = blocking ? "same_object_overlaps_blocked_region" :
      "same_object_moved_outside_blocked_region";
  }

  if (blocking) {
    nonblocking_streak_ = 0;
    departure_status_ = evidence;
    publishOutputs(departure_status_);
    RCLCPP_INFO(
      node_->get_logger(),
      "[WaitForDynamicObstacleDeparture] key='%s' blocking=true evidence=%s "
      "clear_streak=0/%d",
      responsible_object_key_.c_str(), evidence.c_str(),
      required_nonblocking_samples_);
    scheduleNextPoll(departure_status_);
    return BT::NodeStatus::RUNNING;
  }

  ++nonblocking_streak_;
  departure_status_ = evidence;
  publishOutputs(departure_status_);
  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForDynamicObstacleDeparture] key='%s' blocking=false evidence=%s "
    "clear_streak=%d/%d",
    responsible_object_key_.c_str(), evidence.c_str(),
    nonblocking_streak_, required_nonblocking_samples_);

  if (nonblocking_streak_ >= required_nonblocking_samples_) {
    departure_status_ = "dynamic_obstacle_departed";
    publishOutputs(departure_status_);
    return BT::NodeStatus::SUCCESS;
  }

  scheduleNextPoll(departure_status_);
  return BT::NodeStatus::RUNNING;
}

void WaitForDynamicObstacleDeparture::scheduleNextPoll(
  const std::string & status)
{
  departure_status_ = status;
  publishOutputs(departure_status_);
  phase_ = Phase::kWaitPollInterval;
  phase_deadline_ = std::chrono::steady_clock::now() +
    secondsDuration(poll_interval_s_);
}

void WaitForDynamicObstacleDeparture::clearPendingRequest()
{
  if (refresh_future_) {
    refresh_client_->remove_pending_request(*refresh_future_);
    refresh_future_.reset();
  }
}

bool WaitForDynamicObstacleDeparture::readRobotPose(
  geometry_msgs::msg::PoseStamped & pose_out) const
{
  std::string global_frame{"map"};
  std::string robot_base_frame{"base_footprint"};
  double transform_tolerance_s{0.1};
  getInput("global_frame", global_frame);
  getInput("robot_base_frame", robot_base_frame);
  getInput("transform_tolerance_s", transform_tolerance_s);
  return readCurrentRobotPose(
    config(), global_frame, robot_base_frame, transform_tolerance_s, pose_out);
}

bool WaitForDynamicObstacleDeparture::objectObservationExpired(
  const ObjectInstance & object) const
{
  if (object.ttl_sec <= 0.0F) {
    return false;
  }
  if (object.observation_stamp.sec == 0 && object.observation_stamp.nanosec == 0U) {
    return false;  // Missing timestamp: conservative, keep treating as present.
  }

  const rclcpp::Time stamp(
    object.observation_stamp,
    node_->get_clock()->get_clock_type());
  const auto age = node_->now() - stamp;
  return age.seconds() > static_cast<double>(object.ttl_sec);
}

void WaitForDynamicObstacleDeparture::publishOutputs(
  const std::string & status)
{
  setOutput("departure_clear_samples", nonblocking_streak_);
  setOutput("departure_status", status);
}

}  // namespace semantic_nav_nav2_plugins
