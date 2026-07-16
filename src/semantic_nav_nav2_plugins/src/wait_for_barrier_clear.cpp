// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include "semantic_nav_nav2_plugins/wait_for_barrier_clear.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <limits>
#include <string>

namespace semantic_nav_nav2_plugins
{

namespace
{

double secondsToNonNegative(double value)
{
  return std::max(0.0, value);
}

std::chrono::steady_clock::duration secondsDuration(double seconds)
{
  return std::chrono::duration_cast<std::chrono::steady_clock::duration>(
    std::chrono::duration<double>(secondsToNonNegative(seconds)));
}

}  // namespace

WaitForBarrierClear::WaitForBarrierClear(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::StatefulActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  std::string map_topic{"/map"};
  std::string global_costmap_topic{"/global_costmap/costmap"};
  std::string local_clear_service{"/local_costmap/clear_entirely_local_costmap"};
  std::string global_clear_service{"/global_costmap/clear_entirely_global_costmap"};
  std::string cleanup_service{"/rtabmap/cleanup_local_grids"};

  getInput("map_topic", map_topic);
  getInput("global_costmap_topic", global_costmap_topic);
  getInput("local_clear_service", local_clear_service);
  getInput("global_clear_service", global_clear_service);
  getInput("cleanup_service", cleanup_service);

  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.reliable();
  map_qos.transient_local();

  rclcpp::SubscriptionOptions subscription_options;
  subscription_options.callback_group = callback_group_;

  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic,
    map_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      onMap(msg);
    },
    subscription_options);

  global_costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    global_costmap_topic,
    map_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      onGlobalCostmap(msg);
    },
    subscription_options);

  clear_local_client_ = node_->create_client<ClearService>(
    local_clear_service,
    rmw_qos_profile_services_default,
    callback_group_);
  clear_global_client_ = node_->create_client<ClearService>(
    global_clear_service,
    rmw_qos_profile_services_default,
    callback_group_);
#if SEMANTIC_NAV_HAS_RTABMAP_CLEANUP_LOCAL_GRIDS
  cleanup_client_ = node_->create_client<CleanupService>(
    cleanup_service,
    rmw_qos_profile_services_default,
    callback_group_);
#else
  (void)cleanup_service;
  RCLCPP_WARN_ONCE(
    node_->get_logger(),
    "[WaitForBarrierClear] this build does not provide "
    "rtabmap_msgs/srv/CleanupLocalGrids; cached-grid cleanup will be "
    "skipped, while live /map and global-costmap clearance verification "
    "remains enabled");
#endif
}

BT::NodeStatus WaitForBarrierClear::onStart()
{
  abandonPendingRequests();

  barrier_center_ = geometry_msgs::msg::Point();
  barrier_extent_m_ = 0.0F;
  clear_radius_m_ = 0.30;
  lethal_threshold_ = 100;
  max_lethal_fraction_ = 0.15;
  min_observed_cells_ = 8;

  initial_dwell_s_ = 12.0;
  second_dwell_s_ = 12.0;
  poll_interval_s_ = 2.0;
  max_pre_cleanup_polls_ = 6;
  max_post_cleanup_polls_ = 6;
  required_post_cleanup_clear_samples_ = 2;

  cleanup_local_grids_ = true;
  cleanup_radius_cells_ = 1;
  cleanup_filter_scans_ = false;
  service_ready_timeout_ms_ = 2000;
  service_response_timeout_ms_ = 30000;
  fresh_map_timeout_ms_ = 10000;

  getInput("barrier_center", barrier_center_);
  getInput("barrier_extent_m", barrier_extent_m_);
  getInput("clear_radius_m", clear_radius_m_);
  getInput("lethal_threshold", lethal_threshold_);
  getInput("max_lethal_fraction", max_lethal_fraction_);
  getInput("min_observed_cells", min_observed_cells_);

  getInput("initial_dwell_s", initial_dwell_s_);
  getInput("second_dwell_s", second_dwell_s_);
  getInput("poll_interval_s", poll_interval_s_);
  getInput("max_pre_cleanup_polls", max_pre_cleanup_polls_);
  getInput("max_post_cleanup_polls", max_post_cleanup_polls_);
  getInput(
    "required_post_cleanup_clear_samples",
    required_post_cleanup_clear_samples_);

  getInput("cleanup_local_grids", cleanup_local_grids_);
  getInput("cleanup_radius_cells", cleanup_radius_cells_);
  getInput("cleanup_filter_scans", cleanup_filter_scans_);
  getInput("service_ready_timeout_ms", service_ready_timeout_ms_);
  getInput("service_response_timeout_ms", service_response_timeout_ms_);
  getInput("fresh_map_timeout_ms", fresh_map_timeout_ms_);

  barrier_extent_m_ = std::max(0.0F, barrier_extent_m_);
  clear_radius_m_ = std::max(0.0, clear_radius_m_);
  lethal_threshold_ = std::clamp(lethal_threshold_, 0, 100);
  max_lethal_fraction_ = std::clamp(max_lethal_fraction_, 0.0, 1.0);
  min_observed_cells_ = std::max(1, min_observed_cells_);

  initial_dwell_s_ = secondsToNonNegative(initial_dwell_s_);
  second_dwell_s_ = secondsToNonNegative(second_dwell_s_);
  poll_interval_s_ = secondsToNonNegative(poll_interval_s_);
  max_pre_cleanup_polls_ = std::max(1, max_pre_cleanup_polls_);
  max_post_cleanup_polls_ = std::max(1, max_post_cleanup_polls_);
  required_post_cleanup_clear_samples_ =
    std::max(1, required_post_cleanup_clear_samples_);

  cleanup_radius_cells_ = std::max(1, cleanup_radius_cells_);
  service_ready_timeout_ms_ = std::max(0, service_ready_timeout_ms_);
  service_response_timeout_ms_ = std::max(1, service_response_timeout_ms_);
  fresh_map_timeout_ms_ = std::max(1, fresh_map_timeout_ms_);

  pre_poll_index_ = 0;
  post_poll_index_ = 0;
  post_clear_streak_ = 0;
  cleanup_modified_ = -2;
  clearance_status_ = "initial_dwell";
  publishOutputs(clearance_status_);

  phase_ = Phase::kDwellBeforePrePoll;
  phase_deadline_ =
    std::chrono::steady_clock::now() + secondsDuration(initial_dwell_s_);

  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForBarrierClear] started center=(%.3f,%.3f) "
    "extent=%.3fm radius=%.3fm initial_dwell=%.1fs second_dwell=%.1fs",
    barrier_center_.x,
    barrier_center_.y,
    barrier_extent_m_,
    std::max(clear_radius_m_, barrier_extent_m_ / 2.0),
    initial_dwell_s_,
    second_dwell_s_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForBarrierClear::onRunning()
{
  callback_group_executor_.spin_some(std::chrono::nanoseconds(0));
  const auto now = std::chrono::steady_clock::now();

  switch (phase_) {
    case Phase::kDwellBeforePrePoll:
      if (now < phase_deadline_) {
        return BT::NodeStatus::RUNNING;
      }
      beginPrePoll();
      return BT::NodeStatus::RUNNING;

    case Phase::kWaitPreClearResponses:
      if (!clearRequestsFinished() && now < phase_deadline_) {
        return BT::NodeStatus::RUNNING;
      }
      if (!clearRequestsFinished()) {
        RCLCPP_WARN(
          node_->get_logger(),
          "[WaitForBarrierClear] costmap-clear response timeout; "
          "continuing with freshest maps");
        if (clear_local_future_) {
          clear_local_client_->remove_pending_request(*clear_local_future_);
          clear_local_future_.reset();
        }
        if (clear_global_future_) {
          clear_global_client_->remove_pending_request(*clear_global_future_);
          clear_global_future_.reset();
        }
      }
      phase_ = Phase::kWaitPrePollInterval;
      phase_deadline_ = now + secondsDuration(poll_interval_s_);
      return BT::NodeStatus::RUNNING;

    case Phase::kWaitPrePollInterval:
      if (now < phase_deadline_) {
        return BT::NodeStatus::RUNNING;
      }
      return evaluatePrePoll();

    case Phase::kWaitCleanupService:
#if SEMANTIC_NAV_HAS_RTABMAP_CLEANUP_LOCAL_GRIDS
      return handleCleanupServiceWait();
#else
      beginPostVerification();
      return BT::NodeStatus::RUNNING;
#endif

    case Phase::kWaitCleanupResponse:
#if SEMANTIC_NAV_HAS_RTABMAP_CLEANUP_LOCAL_GRIDS
      return handleCleanupResponse();
#else
      beginPostVerification();
      return BT::NodeStatus::RUNNING;
#endif

    case Phase::kWaitFreshMapAfterCleanup:
      if (
        cleanup_modified_ <= 0 ||
        map_generation_ > map_generation_before_cleanup_)
      {
        beginPostVerification();
        return BT::NodeStatus::RUNNING;
      }
      if (now >= phase_deadline_) {
        clearance_status_ = "cleanup_no_fresh_map";
        publishOutputs(clearance_status_);
        RCLCPP_WARN(
          node_->get_logger(),
          "[WaitForBarrierClear] cleanup modified %d grid(s), but no fresh "
          "/map arrived before timeout",
          cleanup_modified_);
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;

    case Phase::kWaitPostClearResponses:
      if (!clearRequestsFinished() && now < phase_deadline_) {
        return BT::NodeStatus::RUNNING;
      }
      if (!clearRequestsFinished()) {
        if (clear_local_future_) {
          clear_local_client_->remove_pending_request(*clear_local_future_);
          clear_local_future_.reset();
        }
        if (clear_global_future_) {
          clear_global_client_->remove_pending_request(*clear_global_future_);
          clear_global_future_.reset();
        }
      }
      phase_ = Phase::kWaitPostPollInterval;
      phase_deadline_ = now + secondsDuration(poll_interval_s_);
      return BT::NodeStatus::RUNNING;

    case Phase::kWaitPostPollInterval:
      if (now < phase_deadline_) {
        return BT::NodeStatus::RUNNING;
      }
      return evaluatePostPoll();
  }

  return BT::NodeStatus::FAILURE;
}

void WaitForBarrierClear::onHalted()
{
  abandonPendingRequests();
  pre_poll_index_ = 0;
  post_poll_index_ = 0;
  post_clear_streak_ = 0;
  clearance_status_ = "halted";
  publishOutputs(clearance_status_);
}

BT::PortsList WaitForBarrierClear::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::Point>(
      "barrier_center", "Responsible blockage centroid in the map frame"),
    BT::InputPort<float>(
      "barrier_extent_m", 0.0F,
      "Measured blockage extent; half is used as a minimum sample radius"),
    BT::InputPort<double>(
      "clear_radius_m", 0.30,
      "Minimum circular footprint radius sampled around the barrier"),
    BT::InputPort<int>(
      "lethal_threshold", 100,
      "Occupancy value treated as lethal in /map and global costmap"),
    BT::InputPort<double>(
      "max_lethal_fraction", 0.15,
      "Maximum lethal fraction accepted as clear"),
    BT::InputPort<int>(
      "min_observed_cells", 8,
      "Minimum non-unknown cells required to confirm clearance"),

    BT::InputPort<double>(
      "initial_dwell_s", 12.0,
      "Stationary first dwell while map_always_update refreshes the map"),
    BT::InputPort<double>(
      "second_dwell_s", 12.0,
      "Second stationary dwell when the first check is still blocked"),
    BT::InputPort<double>(
      "poll_interval_s", 2.0,
      "Costmap-settle interval between clear request and inspection"),
    BT::InputPort<int>(
      "max_pre_cleanup_polls", 6,
      "Maximum live-map clearance checks before cleanup"),
    BT::InputPort<int>(
      "max_post_cleanup_polls", 6,
      "Maximum stabilized clearance checks after cleanup"),
    BT::InputPort<int>(
      "required_post_cleanup_clear_samples", 2,
      "Consecutive clear map+costmap samples required after cleanup"),

    BT::InputPort<bool>(
      "cleanup_local_grids", true,
      "Stabilize cached RTAB-Map local grids after live clearance"),
    BT::InputPort<int>(
      "cleanup_radius_cells", 1,
      "CleanupLocalGrids radius in map cells"),
    BT::InputPort<bool>(
      "cleanup_filter_scans", false,
      "Must normally remain false; scan filtering is non-reversible"),

    BT::InputPort<int>(
      "service_ready_timeout_ms", 2000,
      "Maximum wait for cleanup_local_grids service readiness"),
    BT::InputPort<int>(
      "service_response_timeout_ms", 30000,
      "Maximum wait for costmap-clear or cleanup service response"),
    BT::InputPort<int>(
      "fresh_map_timeout_ms", 10000,
      "Maximum wait for republished /map after modified local grids"),

    BT::InputPort<std::string>(
      "map_topic", "/map", "RTAB-Map occupancy-grid topic"),
    BT::InputPort<std::string>(
      "global_costmap_topic", "/global_costmap/costmap",
      "Nav2 global OccupancyGrid topic"),
    BT::InputPort<std::string>(
      "local_clear_service", "/local_costmap/clear_entirely_local_costmap",
      "Nav2 local costmap clear service"),
    BT::InputPort<std::string>(
      "global_clear_service", "/global_costmap/clear_entirely_global_costmap",
      "Nav2 global costmap clear service"),
    BT::InputPort<std::string>(
      "cleanup_service", "/rtabmap/cleanup_local_grids",
      "RTAB-Map cached-local-grid cleanup service"),

    BT::OutputPort<int>(
      "cleanup_modified", "Number of RTAB-Map local grids modified"),
    BT::OutputPort<std::string>(
      "clearance_status", "Final or current barrier-clearance diagnosis"),
  };
}

WaitForBarrierClear::RegionMetrics WaitForBarrierClear::sampleRegion(
  const nav_msgs::msg::OccupancyGrid & grid,
  const geometry_msgs::msg::Point & center,
  double radius_m,
  int lethal_threshold,
  double max_lethal_fraction,
  int min_observed_cells)
{
  RegionMetrics metrics;

  const auto width = static_cast<std::size_t>(grid.info.width);
  const auto height = static_cast<std::size_t>(grid.info.height);
  const double resolution = static_cast<double>(grid.info.resolution);

  if (
    width == 0 || height == 0 || resolution <= 0.0 ||
    grid.data.size() != width * height ||
    !std::isfinite(center.x) || !std::isfinite(center.y))
  {
    return metrics;
  }

  radius_m = std::max(0.0, radius_m);
  lethal_threshold = std::clamp(lethal_threshold, 0, 100);
  max_lethal_fraction = std::clamp(max_lethal_fraction, 0.0, 1.0);
  min_observed_cells = std::max(1, min_observed_cells);

  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const int center_mx = static_cast<int>(
    std::floor((center.x - origin_x) / resolution));
  const int center_my = static_cast<int>(
    std::floor((center.y - origin_y) / resolution));
  const int cells = static_cast<int>(std::ceil(radius_m / resolution));
  const double radius_sq = radius_m * radius_m + 1e-12;

  for (int my = center_my - cells; my <= center_my + cells; ++my) {
    if (my < 0 || my >= static_cast<int>(height)) {
      continue;
    }
    for (int mx = center_mx - cells; mx <= center_mx + cells; ++mx) {
      if (mx < 0 || mx >= static_cast<int>(width)) {
        continue;
      }

      const double wx = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
      const double wy = origin_y + (static_cast<double>(my) + 0.5) * resolution;
      const double dx = wx - center.x;
      const double dy = wy - center.y;
      if (dx * dx + dy * dy > radius_sq) {
        continue;
      }

      const int value = static_cast<int>(
        grid.data[static_cast<std::size_t>(my) * width +
        static_cast<std::size_t>(mx)]);
      if (value < 0) {
        continue;
      }

      ++metrics.observed_cells;
      if (value >= lethal_threshold) {
        ++metrics.lethal_cells;
      }
    }
  }

  if (metrics.observed_cells > 0) {
    metrics.lethal_fraction =
      static_cast<double>(metrics.lethal_cells) /
      static_cast<double>(metrics.observed_cells);
  }

  metrics.clear =
    metrics.observed_cells >= static_cast<std::size_t>(min_observed_cells) &&
    metrics.lethal_fraction <= max_lethal_fraction;
  return metrics;
}

void WaitForBarrierClear::onMap(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_map_ = msg;
  ++map_generation_;
}

void WaitForBarrierClear::onGlobalCostmap(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_global_costmap_ = msg;
  ++global_generation_;
}

void WaitForBarrierClear::beginPrePoll()
{
  clearance_status_ = "pre_cleanup_poll";
  publishOutputs(clearance_status_);
  requestCostmapClears();
  phase_ = Phase::kWaitPreClearResponses;
  phase_deadline_ =
    std::chrono::steady_clock::now() +
    std::chrono::milliseconds(service_response_timeout_ms_);
}

BT::NodeStatus WaitForBarrierClear::evaluatePrePoll()
{
  const auto map = mapMetrics();
  const auto global = globalMetrics();

  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForBarrierClear] pre poll=%d/%d map(clear=%s observed=%zu "
    "lethal=%zu frac=%.3f gen=%zu) global(clear=%s observed=%zu "
    "lethal=%zu frac=%.3f gen=%zu)",
    pre_poll_index_ + 1,
    max_pre_cleanup_polls_,
    map.clear ? "true" : "false",
    map.observed_cells,
    map.lethal_cells,
    map.lethal_fraction,
    map_generation_,
    global.clear ? "true" : "false",
    global.observed_cells,
    global.lethal_cells,
    global.lethal_fraction,
    global_generation_);

  if (map.clear && global.clear) {
    clearance_status_ = "live_map_and_global_costmap_clear";
    publishOutputs(clearance_status_);
    beginCleanup();
    return BT::NodeStatus::RUNNING;
  }

  ++pre_poll_index_;
  if (pre_poll_index_ >= max_pre_cleanup_polls_) {
    clearance_status_ = "barrier_not_cleared_before_timeout";
    publishOutputs(clearance_status_);
    RCLCPP_WARN(
      node_->get_logger(),
      "[WaitForBarrierClear] live barrier remained blocked or unconfirmed "
      "after %d poll(s)",
      max_pre_cleanup_polls_);
    return BT::NodeStatus::FAILURE;
  }

  scheduleNextPrePoll();
  return BT::NodeStatus::RUNNING;
}

void WaitForBarrierClear::scheduleNextPrePoll()
{
  double dwell = 0.0;
  if (pre_poll_index_ == 1) {
    dwell = second_dwell_s_;
  }

  phase_ = Phase::kDwellBeforePrePoll;
  phase_deadline_ =
    std::chrono::steady_clock::now() + secondsDuration(dwell);

  if (dwell > 0.0) {
    RCLCPP_INFO(
      node_->get_logger(),
      "[WaitForBarrierClear] first check still blocked; dwelling %.1fs "
      "again without moving",
      dwell);
  }
}

void WaitForBarrierClear::beginCleanup()
{
  if (!cleanup_local_grids_) {
    cleanup_modified_ = 0;
    beginPostVerification();
    return;
  }

#if SEMANTIC_NAV_HAS_RTABMAP_CLEANUP_LOCAL_GRIDS
  clearance_status_ = "waiting_cleanup_service";
  publishOutputs(clearance_status_);
  phase_ = Phase::kWaitCleanupService;
  phase_deadline_ =
    std::chrono::steady_clock::now() +
    std::chrono::milliseconds(service_ready_timeout_ms_);
#else
  // -3 means requested by policy but not available in this build.
  cleanup_modified_ = -3;
  clearance_status_ = "cleanup_interface_unavailable_skipped";
  publishOutputs(clearance_status_);
  RCLCPP_WARN(
    node_->get_logger(),
    "[WaitForBarrierClear] cleanup_local_grids requested, but the installed "
    "rtabmap_msgs package does not generate CleanupLocalGrids. Continuing "
    "with consecutive live /map and global-costmap clearance checks.");
  beginPostVerification();
#endif
}

#if SEMANTIC_NAV_HAS_RTABMAP_CLEANUP_LOCAL_GRIDS
BT::NodeStatus WaitForBarrierClear::handleCleanupServiceWait()
{
  const auto now = std::chrono::steady_clock::now();
  if (!cleanup_client_->service_is_ready()) {
    if (now < phase_deadline_) {
      return BT::NodeStatus::RUNNING;
    }

    clearance_status_ = "cleanup_service_unavailable";
    publishOutputs(clearance_status_);
    RCLCPP_WARN(
      node_->get_logger(),
      "[WaitForBarrierClear] /rtabmap/cleanup_local_grids unavailable");
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<CleanupService::Request>();
  request->radius = cleanup_radius_cells_;
  request->filter_scans = cleanup_filter_scans_;

  map_generation_before_cleanup_ = map_generation_;
  cleanup_future_ = cleanup_client_->async_send_request(request);
  phase_ = Phase::kWaitCleanupResponse;
  phase_deadline_ =
    now + std::chrono::milliseconds(service_response_timeout_ms_);
  clearance_status_ = "cleanup_requested";
  publishOutputs(clearance_status_);

  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForBarrierClear] cleanup_local_grids requested radius=%d "
    "filter_scans=%s",
    cleanup_radius_cells_,
    cleanup_filter_scans_ ? "true" : "false");

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForBarrierClear::handleCleanupResponse()
{
  if (!cleanup_future_) {
    clearance_status_ = "cleanup_future_missing";
    publishOutputs(clearance_status_);
    return BT::NodeStatus::FAILURE;
  }

  const auto now = std::chrono::steady_clock::now();
  if (!futureReady(*cleanup_future_)) {
    if (now < phase_deadline_) {
      return BT::NodeStatus::RUNNING;
    }

    cleanup_client_->remove_pending_request(*cleanup_future_);
    cleanup_future_.reset();
    clearance_status_ = "cleanup_response_timeout";
    publishOutputs(clearance_status_);
    RCLCPP_WARN(
      node_->get_logger(),
      "[WaitForBarrierClear] cleanup_local_grids response timeout");
    return BT::NodeStatus::FAILURE;
  }

  const auto response = cleanup_future_->future.get();
  cleanup_future_.reset();
  if (!response) {
    clearance_status_ = "cleanup_null_response";
    publishOutputs(clearance_status_);
    return BT::NodeStatus::FAILURE;
  }

  cleanup_modified_ = response->modified;
  if (cleanup_modified_ < 0) {
    clearance_status_ = "cleanup_error";
    publishOutputs(clearance_status_);
    RCLCPP_WARN(
      node_->get_logger(),
      "[WaitForBarrierClear] cleanup_local_grids returned modified=%d",
      cleanup_modified_);
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForBarrierClear] cleanup_local_grids completed modified=%d",
    cleanup_modified_);

  clearance_status_ = "waiting_fresh_map_after_cleanup";
  publishOutputs(clearance_status_);
  phase_ = Phase::kWaitFreshMapAfterCleanup;
  phase_deadline_ =
    now + std::chrono::milliseconds(fresh_map_timeout_ms_);
  return BT::NodeStatus::RUNNING;
}

#endif

void WaitForBarrierClear::beginPostVerification()
{
  post_poll_index_ = 0;
  post_clear_streak_ = 0;
  clearance_status_ = "post_cleanup_verification";
  publishOutputs(clearance_status_);
  beginPostPoll();
}

void WaitForBarrierClear::beginPostPoll()
{
  requestCostmapClears();
  phase_ = Phase::kWaitPostClearResponses;
  phase_deadline_ =
    std::chrono::steady_clock::now() +
    std::chrono::milliseconds(service_response_timeout_ms_);
}

BT::NodeStatus WaitForBarrierClear::evaluatePostPoll()
{
  const auto map = mapMetrics();
  const auto global = globalMetrics();
  const bool clear = map.clear && global.clear;

  if (clear) {
    ++post_clear_streak_;
  } else {
    post_clear_streak_ = 0;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForBarrierClear] post poll=%d/%d map(clear=%s frac=%.3f) "
    "global(clear=%s frac=%.3f) streak=%d/%d cleanup_modified=%d",
    post_poll_index_ + 1,
    max_post_cleanup_polls_,
    map.clear ? "true" : "false",
    map.lethal_fraction,
    global.clear ? "true" : "false",
    global.lethal_fraction,
    post_clear_streak_,
    required_post_cleanup_clear_samples_,
    cleanup_modified_);

  if (post_clear_streak_ >= required_post_cleanup_clear_samples_) {
    clearance_status_ = "barrier_clear_stabilized";
    publishOutputs(clearance_status_);
    RCLCPP_INFO(
      node_->get_logger(),
      "[WaitForBarrierClear] barrier clear and stabilized; replanning allowed");
    return BT::NodeStatus::SUCCESS;
  }

  ++post_poll_index_;
  if (post_poll_index_ >= max_post_cleanup_polls_) {
    clearance_status_ = "barrier_reappeared_or_unconfirmed_after_cleanup";
    publishOutputs(clearance_status_);
    RCLCPP_WARN(
      node_->get_logger(),
      "[WaitForBarrierClear] barrier did not remain clear after cleanup");
    return BT::NodeStatus::FAILURE;
  }

  beginPostPoll();
  return BT::NodeStatus::RUNNING;
}

void WaitForBarrierClear::requestCostmapClears()
{
  if (clear_local_future_) {
    clear_local_client_->remove_pending_request(*clear_local_future_);
    clear_local_future_.reset();
  }
  if (clear_global_future_) {
    clear_global_client_->remove_pending_request(*clear_global_future_);
    clear_global_future_.reset();
  }

  auto request = std::make_shared<ClearService::Request>();
  if (clear_local_client_->service_is_ready()) {
    clear_local_future_ = clear_local_client_->async_send_request(request);
  } else {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "[WaitForBarrierClear] local costmap clear service unavailable");
  }

  request = std::make_shared<ClearService::Request>();
  if (clear_global_client_->service_is_ready()) {
    clear_global_future_ = clear_global_client_->async_send_request(request);
  } else {
    RCLCPP_WARN_THROTTLE(
      node_->get_logger(),
      *node_->get_clock(),
      5000,
      "[WaitForBarrierClear] global costmap clear service unavailable");
  }
}

bool WaitForBarrierClear::clearRequestsFinished()
{
  bool finished = true;

  if (clear_local_future_) {
    if (futureReady(*clear_local_future_)) {
      clear_local_future_->future.get();
      clear_local_future_.reset();
    } else {
      finished = false;
    }
  }

  if (clear_global_future_) {
    if (futureReady(*clear_global_future_)) {
      clear_global_future_->future.get();
      clear_global_future_.reset();
    } else {
      finished = false;
    }
  }

  return finished;
}

void WaitForBarrierClear::abandonPendingRequests()
{
  if (clear_local_future_) {
    clear_local_client_->remove_pending_request(*clear_local_future_);
    clear_local_future_.reset();
  }
  if (clear_global_future_) {
    clear_global_client_->remove_pending_request(*clear_global_future_);
    clear_global_future_.reset();
  }
#if SEMANTIC_NAV_HAS_RTABMAP_CLEANUP_LOCAL_GRIDS
  if (cleanup_future_) {
    cleanup_client_->remove_pending_request(*cleanup_future_);
    cleanup_future_.reset();
  }
#endif
}

WaitForBarrierClear::RegionMetrics WaitForBarrierClear::mapMetrics() const
{
  if (!latest_map_) {
    return RegionMetrics();
  }
  return sampleRegion(
    *latest_map_,
    barrier_center_,
    std::max(clear_radius_m_, barrier_extent_m_ / 2.0),
    lethal_threshold_,
    max_lethal_fraction_,
    min_observed_cells_);
}

WaitForBarrierClear::RegionMetrics WaitForBarrierClear::globalMetrics() const
{
  if (!latest_global_costmap_) {
    return RegionMetrics();
  }
  return sampleRegion(
    *latest_global_costmap_,
    barrier_center_,
    std::max(clear_radius_m_, barrier_extent_m_ / 2.0),
    lethal_threshold_,
    max_lethal_fraction_,
    min_observed_cells_);
}

void WaitForBarrierClear::publishOutputs(const std::string & status)
{
  setOutput("cleanup_modified", cleanup_modified_);
  setOutput("clearance_status", status);
}

bool WaitForBarrierClear::futureReady(const ClearFuture & future)
{
  return future.future.wait_for(std::chrono::milliseconds(0)) ==
         std::future_status::ready;
}

#if SEMANTIC_NAV_HAS_RTABMAP_CLEANUP_LOCAL_GRIDS
bool WaitForBarrierClear::futureReady(const CleanupFuture & future)
{
  return future.future.wait_for(std::chrono::milliseconds(0)) ==
         std::future_status::ready;
}
#endif

}  // namespace semantic_nav_nav2_plugins