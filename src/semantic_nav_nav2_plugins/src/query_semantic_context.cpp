// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include "semantic_nav_nav2_plugins/query_semantic_context.hpp"

#include <algorithm>
#include <chrono>
#include <string>

namespace semantic_nav_nav2_plugins
{

QuerySemanticContext::QuerySemanticContext(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string refresh_service{"/refresh_local_objects"};
  std::string match_service{"/match_responsible_object"};

  getInput("refresh_service", refresh_service);
  getInput("match_service", match_service);

  // Dedicated callback group + executor so spin_some() in onRunning() resolves
  // service futures without relying on bt_navigator's main executor to tick at
  // the right moment.
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  refresh_client_ = node_->create_client<RefreshSrv>(
    refresh_service, rmw_qos_profile_services_default, callback_group_);
  match_client_ = node_->create_client<MatchSrv>(
    match_service, rmw_qos_profile_services_default, callback_group_);
}

BT::NodeStatus QuerySemanticContext::onStart()
{
  clearPendingRequests();

  refresh_response_ = nullptr;
  service_ready_timeout_ms_ = 2000;
  response_timeout_ms_ = 5000;

  getInput("service_ready_timeout_ms", service_ready_timeout_ms_);
  getInput("response_timeout_ms", response_timeout_ms_);

  service_ready_timeout_ms_ = std::max(0, service_ready_timeout_ms_);
  response_timeout_ms_ = std::max(1, response_timeout_ms_);

  phase_ = Phase::kWaitRefreshService;
  phase_deadline_ =
    std::chrono::steady_clock::now() +
    std::chrono::milliseconds(service_ready_timeout_ms_);

  return onRunning();
}

BT::NodeStatus QuerySemanticContext::onRunning()
{
  // Drain any pending service responses before checking futures.
  callback_group_executor_.spin_some(std::chrono::nanoseconds(0));

  const auto now = std::chrono::steady_clock::now();

  switch (phase_) {
    case Phase::kWaitRefreshService:
      if (!refresh_client_->service_is_ready()) {
        if (now < phase_deadline_) {
          return BT::NodeStatus::RUNNING;
        }

        RCLCPP_WARN(
          node_->get_logger(),
          "[QuerySemanticContext] /refresh_local_objects not ready before timeout");
        writeDefaultOutputs();
        return BT::NodeStatus::SUCCESS;
      }

      return sendRefreshRequest();

    case Phase::kWaitRefreshResponse:
      if (!refresh_future_) {
        writeDefaultOutputs();
        return BT::NodeStatus::SUCCESS;
      }

      if (refresh_future_->future.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready)
      {
        if (now < phase_deadline_) {
          return BT::NodeStatus::RUNNING;
        }

        refresh_client_->remove_pending_request(*refresh_future_);
        refresh_future_.reset();

        RCLCPP_WARN(
          node_->get_logger(),
          "[QuerySemanticContext] /refresh_local_objects response timeout");
        writeDefaultOutputs();
        return BT::NodeStatus::SUCCESS;
      }

      refresh_response_ = refresh_future_->future.get();
      refresh_future_.reset();

      if (!refresh_response_) {
        RCLCPP_WARN(
          node_->get_logger(),
          "[QuerySemanticContext] /refresh_local_objects returned null response");
        writeDefaultOutputs();
        return BT::NodeStatus::SUCCESS;
      }

      phase_ = Phase::kWaitMatchService;
      phase_deadline_ =
        std::chrono::steady_clock::now() +
        std::chrono::milliseconds(service_ready_timeout_ms_);

      return onRunning();

    case Phase::kWaitMatchService:
      if (!match_client_->service_is_ready()) {
        if (now < phase_deadline_) {
          return BT::NodeStatus::RUNNING;
        }

        RCLCPP_WARN(
          node_->get_logger(),
          "[QuerySemanticContext] /match_responsible_object not ready before timeout");
        writeDefaultObjectOutputs();
        writeDbOutputsFromRefresh();
        return BT::NodeStatus::SUCCESS;
      }

      return sendMatchRequest();

    case Phase::kWaitMatchResponse:
      if (!match_future_) {
        writeDefaultObjectOutputs();
        writeDbOutputsFromRefresh();
        return BT::NodeStatus::SUCCESS;
      }

      if (match_future_->future.wait_for(std::chrono::milliseconds(0)) !=
        std::future_status::ready)
      {
        if (now < phase_deadline_) {
          return BT::NodeStatus::RUNNING;
        }

        match_client_->remove_pending_request(*match_future_);
        match_future_.reset();

        RCLCPP_WARN(
          node_->get_logger(),
          "[QuerySemanticContext] /match_responsible_object response timeout");
        writeDefaultObjectOutputs();
        writeDbOutputsFromRefresh();
        return BT::NodeStatus::SUCCESS;
      }

      {
        const auto response = match_future_->future.get();
        match_future_.reset();

        writeDbOutputsFromRefresh();

        if (!response) {
          RCLCPP_WARN(
            node_->get_logger(),
            "[QuerySemanticContext] /match_responsible_object returned null response");
          writeDefaultObjectOutputs();
          return BT::NodeStatus::SUCCESS;
        }

        if (!response->success) {
          RCLCPP_DEBUG(
            node_->get_logger(),
            "[QuerySemanticContext] no responsible object matched: %s",
            response->message.c_str());
          writeDefaultObjectOutputs();
          return BT::NodeStatus::SUCCESS;
        }

        setOutput("responsible_object_key", response->responsible_object_key);
        setOutput("responsible_object_tag", response->responsible_object_tag);
        setOutput("responsible_object_state", response->responsible_object_state);
        setOutput("responsible_safety_class", response->safety_class);
        setOutput("responsible_openable", response->openable);
        setOutput("responsible_clearable", response->clearable);
        setOutput("responsible_match_type", response->match_type);
        setOutput("responsible_state_detail", response->state_detail);
        setOutput("responsible_traversability", response->traversability);

        RCLCPP_DEBUG(
          node_->get_logger(),
          "[QuerySemanticContext] match_type='%s' key='%s' tag='%s' safety='%s'",
          response->match_type.c_str(),
          response->responsible_object_key.c_str(),
          response->responsible_object_tag.c_str(),
          response->safety_class.c_str());

        return BT::NodeStatus::SUCCESS;
      }
  }

  writeDefaultOutputs();
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus QuerySemanticContext::sendRefreshRequest()
{
  auto request = std::make_shared<RefreshSrv::Request>();

  if (!readRobotPose(request->robot_pose)) {
    RCLCPP_WARN(
      node_->get_logger(),
      "[QuerySemanticContext] robot_pose and goal unavailable; using zero pose for context query");
  }

  geometry_msgs::msg::Point blockage_centroid;
  if (!getInput<geometry_msgs::msg::Point>(
    "blockage_centroid",
    blockage_centroid))
  {
    blockage_centroid = geometry_msgs::msg::Point{};
  }
  request->blockage_centroid = blockage_centroid;

  float radius_m{4.0f};
  getInput("radius_m", radius_m);
  request->radius_m = std::max(0.1f, radius_m);

  refresh_future_ = refresh_client_->async_send_request(request);

  phase_ = Phase::kWaitRefreshResponse;
  phase_deadline_ =
    std::chrono::steady_clock::now() +
    std::chrono::milliseconds(response_timeout_ms_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus QuerySemanticContext::sendMatchRequest()
{
  if (!refresh_response_) {
    writeDefaultOutputs();
    return BT::NodeStatus::SUCCESS;
  }

  auto request = std::make_shared<MatchSrv::Request>();

  geometry_msgs::msg::Point blockage_centroid;
  if (!getInput<geometry_msgs::msg::Point>(
    "blockage_centroid",
    blockage_centroid))
  {
    blockage_centroid = geometry_msgs::msg::Point{};
  }
  request->blockage_centroid = blockage_centroid;

  float blockage_extent_m{0.0f};
  getInput("blockage_extent_m", blockage_extent_m);
  request->blockage_extent_m = blockage_extent_m;

  request->objects = refresh_response_->objects;

  match_future_ = match_client_->async_send_request(request);

  phase_ = Phase::kWaitMatchResponse;
  phase_deadline_ =
    std::chrono::steady_clock::now() +
    std::chrono::milliseconds(response_timeout_ms_);

  return BT::NodeStatus::RUNNING;
}

void QuerySemanticContext::onHalted()
{
  clearPendingRequests();
  refresh_response_ = nullptr;
  phase_ = Phase::kWaitRefreshService;
}

void QuerySemanticContext::clearPendingRequests()
{
  if (refresh_future_) {
    refresh_client_->remove_pending_request(*refresh_future_);
    refresh_future_.reset();
  }

  if (match_future_) {
    match_client_->remove_pending_request(*match_future_);
    match_future_.reset();
  }
}

void QuerySemanticContext::writeDefaultOutputs()
{
  writeDefaultObjectOutputs();
  setOutput("local_db_version", 0);
  setOutput("local_db_source", std::string("static_snapshot"));
}

void QuerySemanticContext::writeDefaultObjectOutputs()
{
  setOutput("responsible_object_key", std::string(""));
  setOutput("responsible_object_tag", std::string(""));
  setOutput("responsible_object_state", std::string(""));
  setOutput("responsible_safety_class", std::string("none"));
  setOutput("responsible_openable", false);
  setOutput("responsible_clearable", false);
  setOutput("responsible_match_type", std::string(""));
  setOutput("responsible_state_detail", std::string(""));
  setOutput("responsible_traversability", std::string(""));
}

void QuerySemanticContext::writeDbOutputsFromRefresh()
{
  if (!refresh_response_) {
    setOutput("local_db_version", 0);
    setOutput("local_db_source", std::string("static_snapshot"));
    return;
  }

  setOutput("local_db_version", static_cast<int>(refresh_response_->db_version));

  if (refresh_response_->source.empty()) {
    setOutput("local_db_source", std::string("static_snapshot"));
  } else {
    setOutput("local_db_source", refresh_response_->source);
  }
}

bool QuerySemanticContext::readRobotPose(
  geometry_msgs::msg::PoseStamped & pose_out) const
{
  if (config().blackboard->get<geometry_msgs::msg::PoseStamped>(
      "robot_pose",
      pose_out))
  {
    return true;
  }

  return config().blackboard->get<geometry_msgs::msg::PoseStamped>(
    "goal",
    pose_out);
}

BT::PortsList QuerySemanticContext::providedPorts()
{
  return {
    // ---- Inputs ----
    BT::InputPort<geometry_msgs::msg::Point>(
      "blockage_centroid",
      "Centroid written by PathClearCondition; missing input is treated as zero"),
    BT::InputPort<float>(
      "blockage_extent_m",
      0.0f,
      "Extent written by PathClearCondition; zero default"),
    BT::InputPort<float>(
      "radius_m",
      4.0f,
      "Local semantic query radius"),
    BT::InputPort<std::string>(
      "refresh_service",
      "/refresh_local_objects",
      "RefreshLocalObjects service name"),
    BT::InputPort<std::string>(
      "match_service",
      "/match_responsible_object",
      "MatchResponsibleObject service name"),
    BT::InputPort<int>(
      "service_ready_timeout_ms",
      2000,
      "Max non-blocking wait for service availability"),
    BT::InputPort<int>(
      "response_timeout_ms",
      5000,
      "Max non-blocking wait for each service response"),

    // ---- Outputs ----
    BT::OutputPort<std::string>("responsible_object_key", ""),
    BT::OutputPort<std::string>("responsible_object_tag", ""),
    BT::OutputPort<std::string>("responsible_object_state", ""),
    BT::OutputPort<std::string>("responsible_safety_class", ""),
    BT::OutputPort<bool>("responsible_openable", ""),
    BT::OutputPort<bool>("responsible_clearable", ""),
    BT::OutputPort<std::string>("responsible_match_type", ""),
    BT::OutputPort<std::string>("responsible_state_detail", ""),
    BT::OutputPort<std::string>("responsible_traversability", ""),
    BT::OutputPort<int>("local_db_version", ""),
    BT::OutputPort<std::string>("local_db_source", ""),
  };
}

}  // namespace semantic_nav_nav2_plugins