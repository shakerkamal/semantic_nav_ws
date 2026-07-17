// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include "semantic_nav_nav2_plugins/wait_for_barrier_clear.hpp"

#include <algorithm>
#include <cmath>
#include <future>
#include <string>

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

const char * boolText(bool value)
{
  return value ? "true" : "false";
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
  std::string local_costmap_topic{"/local_costmap/costmap"};
  std::string local_clear_service{"/local_costmap/clear_entirely_local_costmap"};
  std::string global_clear_service{"/global_costmap/clear_entirely_global_costmap"};
  std::string cleanup_service{"/rtabmap/cleanup_local_grids"};

  getInput("map_topic", map_topic);
  getInput("global_costmap_topic", global_costmap_topic);
  getInput("local_costmap_topic", local_costmap_topic);
  getInput("local_clear_service", local_clear_service);
  getInput("global_clear_service", global_clear_service);
  getInput("cleanup_service", cleanup_service);

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;

  rclcpp::QoS map_qos(rclcpp::KeepLast(1));
  map_qos.reliable();
  map_qos.transient_local();
  map_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    map_topic,
    map_qos,
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {onMap(msg);},
    options);

  global_costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    global_costmap_topic,
    rclcpp::SystemDefaultsQoS(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      onGlobalCostmap(msg);
    },
    options);

  local_costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    local_costmap_topic,
    rclcpp::SystemDefaultsQoS(),
    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {onLocalCostmap(msg);},
    options);

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
    "[WaitForBarrierClear] CleanupLocalGrids interface unavailable; "
    "live map/global/local verification remains enabled");
#endif
}

BT::NodeStatus WaitForBarrierClear::onStart()
{
  abandonPendingRequests();

  barrier_center_ = geometry_msgs::msg::Point();
  barrier_bbox_extent_ = geometry_msgs::msg::Vector3();
  observed_blockage_center_ = geometry_msgs::msg::Point();
  observed_blockage_extent_m_ = 0.0F;

  clear_radius_m_ = 0.30;
  bbox_padding_m_ = 0.05;
  use_bbox_footprint_ = true;
  observed_min_radius_m_ = 0.15;
  observed_padding_m_ = 0.05;
  observed_center_max_gap_m_ = 0.15;
  map_lethal_threshold_ = 100;
  costmap_lethal_threshold_ = 90;
  max_lethal_fraction_ = 0.15;
  max_lethal_cells_ = 1;
  min_observed_cells_ = 8;
  require_fresh_costmaps_ = true;
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
  fresh_costmap_timeout_ms_ = 5000;
  fresh_map_timeout_ms_ = 10000;

  getInput("barrier_center", barrier_center_);
  getInput("barrier_bbox_extent", barrier_bbox_extent_);
  getInput("observed_blockage_center", observed_blockage_center_);
  if (!getInput("observed_blockage_extent_m", observed_blockage_extent_m_)) {
    getInput("barrier_extent_m", observed_blockage_extent_m_);
  }

  getInput("clear_radius_m", clear_radius_m_);
  getInput("bbox_padding_m", bbox_padding_m_);
  getInput("use_bbox_footprint", use_bbox_footprint_);
  getInput("observed_min_radius_m", observed_min_radius_m_);
  getInput("observed_padding_m", observed_padding_m_);
  getInput("observed_center_max_gap_m", observed_center_max_gap_m_);
  getInput("map_lethal_threshold", map_lethal_threshold_);
  getInput("costmap_lethal_threshold", costmap_lethal_threshold_);
  getInput("max_lethal_fraction", max_lethal_fraction_);
  getInput("max_lethal_cells", max_lethal_cells_);
  getInput("min_observed_cells", min_observed_cells_);
  getInput("require_fresh_costmaps", require_fresh_costmaps_);
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
  getInput("fresh_costmap_timeout_ms", fresh_costmap_timeout_ms_);
  getInput("fresh_map_timeout_ms", fresh_map_timeout_ms_);

  clear_radius_m_ = std::max(0.05, clear_radius_m_);
  bbox_padding_m_ = std::max(0.0, bbox_padding_m_);
  observed_min_radius_m_ = std::max(0.05, observed_min_radius_m_);
  observed_padding_m_ = std::max(0.0, observed_padding_m_);
  observed_center_max_gap_m_ = std::max(0.0, observed_center_max_gap_m_);
  observed_blockage_extent_m_ = std::max(0.0F, observed_blockage_extent_m_);
  map_lethal_threshold_ = std::clamp(map_lethal_threshold_, 0, 100);
  costmap_lethal_threshold_ = std::clamp(costmap_lethal_threshold_, 0, 100);
  max_lethal_fraction_ = std::clamp(max_lethal_fraction_, 0.0, 1.0);
  max_lethal_cells_ = std::max(0, max_lethal_cells_);
  min_observed_cells_ = std::max(1, min_observed_cells_);
  initial_dwell_s_ = std::max(0.0, initial_dwell_s_);
  second_dwell_s_ = std::max(0.0, second_dwell_s_);
  poll_interval_s_ = std::max(0.0, poll_interval_s_);
  max_pre_cleanup_polls_ = std::max(1, max_pre_cleanup_polls_);
  max_post_cleanup_polls_ = std::max(1, max_post_cleanup_polls_);
  required_post_cleanup_clear_samples_ =
    std::max(1, required_post_cleanup_clear_samples_);
  cleanup_radius_cells_ = std::max(1, cleanup_radius_cells_);
  service_ready_timeout_ms_ = std::max(0, service_ready_timeout_ms_);
  service_response_timeout_ms_ = std::max(1, service_response_timeout_ms_);
  fresh_costmap_timeout_ms_ = std::max(1, fresh_costmap_timeout_ms_);
  fresh_map_timeout_ms_ = std::max(1, fresh_map_timeout_ms_);

  semantic_footprint_ = computeSemanticFootprint(
    barrier_center_, barrier_bbox_extent_, clear_radius_m_, bbox_padding_m_,
    use_bbox_footprint_);
  observed_radius_m_ = computeObservedRadius(
    observed_min_radius_m_, observed_blockage_extent_m_, observed_padding_m_);
  use_observed_region_ = shouldUseObservedRegion(
    semantic_footprint_, observed_blockage_center_, observed_center_max_gap_m_);

  pre_poll_index_ = 0;
  post_poll_index_ = 0;
  post_clear_streak_ = 0;
  cleanup_modified_ = -2;
  clearance_status_ = "initial_dwell";
  publishOutputs(clearance_status_);

  phase_ = Phase::kInitialDwell;
  phase_deadline_ = std::chrono::steady_clock::now() +
    secondsDuration(initial_dwell_s_);

  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForBarrierClear] started center=(%.3f,%.3f) "
    "half_extents=(%.3f,%.3f) bbox_used=%s observed_used=%s "
    "observed_center=(%.3f,%.3f) observed_radius=%.3fm "
    "raw_before_clear=true initial_dwell=%.1fs second_dwell=%.1fs",
    semantic_footprint_.center.x,
    semantic_footprint_.center.y,
    semantic_footprint_.half_x,
    semantic_footprint_.half_y,
    boolText(semantic_footprint_.uses_bbox),
    boolText(use_observed_region_),
    observed_blockage_center_.x,
    observed_blockage_center_.y,
    observed_radius_m_,
    initial_dwell_s_,
    second_dwell_s_);

  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus WaitForBarrierClear::onRunning()
{
  callback_group_executor_.spin_some(std::chrono::nanoseconds(0));
  const auto now = std::chrono::steady_clock::now();

  switch (phase_) {
    case Phase::kInitialDwell:
      if (now < phase_deadline_) {
        return BT::NodeStatus::RUNNING;
      }
      return evaluateRawPrePoll();

    case Phase::kRawPrePollInterval:
      if (now < phase_deadline_) {
        return BT::NodeStatus::RUNNING;
      }
      return evaluateRawPrePoll();

    case Phase::kWaitPreClearResponses:
      if (!clearRequestsFinished() && now < phase_deadline_) {
        return BT::NodeStatus::RUNNING;
      }
      if (!clearRequestsFinished()) {
        RCLCPP_WARN(
          node_->get_logger(),
          "[WaitForBarrierClear] pre-clean costmap-clear response timeout; "
          "waiting for fresh costmap generations");
        if (clear_local_future_) {
          clear_local_client_->remove_pending_request(*clear_local_future_);
          clear_local_future_.reset();
        }
        if (clear_global_future_) {
          clear_global_client_->remove_pending_request(*clear_global_future_);
          clear_global_future_.reset();
        }
      }
      phase_ = Phase::kWaitPreFreshCostmaps;
      phase_deadline_ = now +
        std::chrono::milliseconds(fresh_costmap_timeout_ms_);
      return BT::NodeStatus::RUNNING;

    case Phase::kWaitPreFreshCostmaps:
      if (costmapsFreshAfterClear() || !require_fresh_costmaps_) {
        phase_ = Phase::kWaitPreVerificationInterval;
        phase_deadline_ = now + secondsDuration(poll_interval_s_);
        return BT::NodeStatus::RUNNING;
      }
      if (now >= phase_deadline_) {
        clearance_status_ = "pre_clear_no_fresh_costmaps";
        publishOutputs(clearance_status_);
        return BT::NodeStatus::FAILURE;
      }
      return BT::NodeStatus::RUNNING;

    case Phase::kWaitPreVerificationInterval:
      if (now < phase_deadline_) {
        return BT::NodeStatus::RUNNING;
      }
      return evaluatePreVerification();

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
      if (cleanup_modified_ <= 0 || map_generation_ > map_generation_before_cleanup_) {
        beginPostVerification();
        return BT::NodeStatus::RUNNING;
      }
      if (now >= phase_deadline_) {
        clearance_status_ = "cleanup_no_fresh_map";
        publishOutputs(clearance_status_);
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
      phase_ = Phase::kWaitPostFreshCostmaps;
      phase_deadline_ = now +
        std::chrono::milliseconds(fresh_costmap_timeout_ms_);
      return BT::NodeStatus::RUNNING;

    case Phase::kWaitPostFreshCostmaps:
      if (costmapsFreshAfterClear() || !require_fresh_costmaps_) {
        phase_ = Phase::kWaitPostPollInterval;
        phase_deadline_ = now + secondsDuration(poll_interval_s_);
        return BT::NodeStatus::RUNNING;
      }
      if (now >= phase_deadline_) {
        clearance_status_ = "post_clear_no_fresh_costmaps";
        publishOutputs(clearance_status_);
        return BT::NodeStatus::FAILURE;
      }
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
      "barrier_center", "Matched object's bbox center"),
    BT::InputPort<geometry_msgs::msg::Vector3>(
      "barrier_bbox_extent", "Matched object's axis-aligned bbox extent"),
    BT::InputPort<geometry_msgs::msg::Point>(
      "observed_blockage_center", "Measured blockage centroid"),
    BT::InputPort<float>(
      "observed_blockage_extent_m", 0.0F, "Measured blockage extent"),
    BT::InputPort<float>(
      "barrier_extent_m", 0.0F,
      "Backward-compatible alias for observed_blockage_extent_m"),
    BT::InputPort<double>(
      "clear_radius_m", 0.30,
      "Class-agnostic circular fallback when bbox geometry is unavailable"),
    BT::InputPort<double>(
      "bbox_padding_m", 0.05, "Padding around valid semantic bbox geometry"),
    BT::InputPort<bool>(
      "use_bbox_footprint", true,
      "Use bbox geometry; false preserves a center-circle check for doorway barriers"),
    BT::InputPort<double>(
      "observed_min_radius_m", 0.15,
      "Minimum local-costmap radius for measured occupancy"),
    BT::InputPort<double>(
      "observed_padding_m", 0.05,
      "Padding around the measured blockage extent"),
    BT::InputPort<double>(
      "observed_center_max_gap_m", 0.15,
      "Maximum gap from the semantic bbox to accept measured occupancy"),
    BT::InputPort<int>(
      "map_lethal_threshold", 100, "Lethal occupancy threshold for /map"),
    BT::InputPort<int>(
      "costmap_lethal_threshold", 90,
      "Lethal/high-cost threshold for local/global costmaps"),
    BT::InputPort<double>(
      "max_lethal_fraction", 0.15, "Maximum lethal fraction accepted"),
    BT::InputPort<int>(
      "max_lethal_cells", 1, "Absolute lethal-cell cap accepted as noise"),
    BT::InputPort<int>(
      "min_observed_cells", 8, "Minimum known cells required"),
    BT::InputPort<bool>(
      "require_fresh_costmaps", true,
      "Require local/global messages newer than each clear request"),
    BT::InputPort<double>(
      "initial_dwell_s", 12.0, "Initial stationary mapping dwell"),
    BT::InputPort<double>(
      "second_dwell_s", 12.0, "Additional dwell after first blocked poll"),
    BT::InputPort<double>("poll_interval_s", 2.0, "Settle/poll interval"),
    BT::InputPort<int>(
      "max_pre_cleanup_polls", 6, "Maximum raw/fresh polls before cleanup"),
    BT::InputPort<int>(
      "max_post_cleanup_polls", 6, "Maximum stabilized polls after cleanup"),
    BT::InputPort<int>(
      "required_post_cleanup_clear_samples", 2,
      "Consecutive clear post-cleanup samples"),
    BT::InputPort<bool>(
      "cleanup_local_grids", true, "Enable RTAB cached-grid cleanup"),
    BT::InputPort<int>(
      "cleanup_radius_cells", 1, "CleanupLocalGrids radius"),
    BT::InputPort<bool>(
      "cleanup_filter_scans", false,
      "Normally false; scan filtering is non-reversible"),
    BT::InputPort<int>(
      "service_ready_timeout_ms", 2000, "Cleanup service readiness timeout"),
    BT::InputPort<int>(
      "service_response_timeout_ms", 30000, "Service response timeout"),
    BT::InputPort<int>(
      "fresh_costmap_timeout_ms", 5000,
      "Wait for local/global messages after clear"),
    BT::InputPort<int>(
      "fresh_map_timeout_ms", 10000,
      "Wait for /map after modified local grids"),
    BT::InputPort<std::string>("map_topic", "/map", "RTAB-Map grid topic"),
    BT::InputPort<std::string>(
      "global_costmap_topic", "/global_costmap/costmap", "Global costmap topic"),
    BT::InputPort<std::string>(
      "local_costmap_topic", "/local_costmap/costmap", "Local costmap topic"),
    BT::InputPort<std::string>(
      "local_clear_service", "/local_costmap/clear_entirely_local_costmap",
      "Local costmap clear service"),
    BT::InputPort<std::string>(
      "global_clear_service", "/global_costmap/clear_entirely_global_costmap",
      "Global costmap clear service"),
    BT::InputPort<std::string>(
      "cleanup_service", "/rtabmap/cleanup_local_grids",
      "RTAB cached-grid cleanup service"),
    BT::OutputPort<int>(
      "cleanup_modified", "Number of RTAB local grids modified"),
    BT::OutputPort<std::string>(
      "clearance_status", "Current/final clearance diagnosis"),
  };
}

WaitForBarrierClear::AxisAlignedFootprint
WaitForBarrierClear::computeSemanticFootprint(
  const geometry_msgs::msg::Point & center,
  const geometry_msgs::msg::Vector3 & bbox_extent,
  double fallback_radius_m,
  double bbox_padding_m,
  bool use_bbox_footprint)
{
  AxisAlignedFootprint footprint;
  footprint.center = center;
  fallback_radius_m = std::max(0.05, fallback_radius_m);
  bbox_padding_m = std::max(0.0, bbox_padding_m);
  if (!finitePoint(center)) {
    return footprint;
  }

  if (use_bbox_footprint && finiteExtent(bbox_extent) &&
      std::abs(bbox_extent.x) > 0.0 &&
      std::abs(bbox_extent.y) > 0.0)
  {
    footprint.half_x = 0.5 * std::abs(bbox_extent.x) + bbox_padding_m;
    footprint.half_y = 0.5 * std::abs(bbox_extent.y) + bbox_padding_m;
    footprint.uses_bbox = true;
  } else {
    footprint.half_x = fallback_radius_m;
    footprint.half_y = fallback_radius_m;
    footprint.uses_bbox = false;
  }
  footprint.valid = true;
  return footprint;
}

WaitForBarrierClear::RegionMetrics WaitForBarrierClear::sampleFootprintRegion(
  const nav_msgs::msg::OccupancyGrid & grid,
  const AxisAlignedFootprint & footprint,
  int lethal_threshold,
  double max_lethal_fraction,
  int max_lethal_cells,
  int min_observed_cells)
{
  RegionMetrics metrics;
  if (!footprint.valid || !finitePoint(footprint.center)) {
    return metrics;
  }

  const std::size_t width = grid.info.width;
  const std::size_t height = grid.info.height;
  const double resolution = grid.info.resolution;
  if (width == 0U || height == 0U || resolution <= 0.0 ||
      grid.data.size() != width * height)
  {
    return metrics;
  }

  lethal_threshold = std::clamp(lethal_threshold, 0, 100);
  max_lethal_fraction = std::clamp(max_lethal_fraction, 0.0, 1.0);
  max_lethal_cells = std::max(0, max_lethal_cells);
  min_observed_cells = std::max(1, min_observed_cells);

  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const int min_mx = static_cast<int>(std::floor(
    (footprint.center.x - footprint.half_x - origin_x) / resolution));
  const int max_mx = static_cast<int>(std::floor(
    (footprint.center.x + footprint.half_x - origin_x) / resolution));
  const int min_my = static_cast<int>(std::floor(
    (footprint.center.y - footprint.half_y - origin_y) / resolution));
  const int max_my = static_cast<int>(std::floor(
    (footprint.center.y + footprint.half_y - origin_y) / resolution));

  for (int my = min_my; my <= max_my; ++my) {
    if (my < 0 || my >= static_cast<int>(height)) {
      continue;
    }
    for (int mx = min_mx; mx <= max_mx; ++mx) {
      if (mx < 0 || mx >= static_cast<int>(width)) {
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

  if (metrics.observed_cells > 0U) {
    metrics.lethal_fraction =
      static_cast<double>(metrics.lethal_cells) /
      static_cast<double>(metrics.observed_cells);
  }
  metrics.clear =
    metrics.observed_cells >= static_cast<std::size_t>(min_observed_cells) &&
    metrics.lethal_cells <= static_cast<std::size_t>(max_lethal_cells) &&
    metrics.lethal_fraction <= max_lethal_fraction;
  return metrics;
}

WaitForBarrierClear::RegionMetrics WaitForBarrierClear::sampleCircularRegion(
  const nav_msgs::msg::OccupancyGrid & grid,
  const geometry_msgs::msg::Point & center,
  double radius_m,
  int lethal_threshold,
  double max_lethal_fraction,
  int max_lethal_cells,
  int min_observed_cells)
{
  RegionMetrics metrics;
  const std::size_t width = grid.info.width;
  const std::size_t height = grid.info.height;
  const double resolution = grid.info.resolution;
  if (width == 0U || height == 0U || resolution <= 0.0 ||
      grid.data.size() != width * height || !finitePoint(center))
  {
    return metrics;
  }

  radius_m = std::max(0.0, radius_m);
  lethal_threshold = std::clamp(lethal_threshold, 0, 100);
  max_lethal_fraction = std::clamp(max_lethal_fraction, 0.0, 1.0);
  max_lethal_cells = std::max(0, max_lethal_cells);
  min_observed_cells = std::max(1, min_observed_cells);

  const double origin_x = grid.info.origin.position.x;
  const double origin_y = grid.info.origin.position.y;
  const int center_mx = static_cast<int>(std::floor((center.x - origin_x) / resolution));
  const int center_my = static_cast<int>(std::floor((center.y - origin_y) / resolution));
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

  if (metrics.observed_cells > 0U) {
    metrics.lethal_fraction =
      static_cast<double>(metrics.lethal_cells) /
      static_cast<double>(metrics.observed_cells);
  }
  metrics.clear =
    metrics.observed_cells >= static_cast<std::size_t>(min_observed_cells) &&
    metrics.lethal_cells <= static_cast<std::size_t>(max_lethal_cells) &&
    metrics.lethal_fraction <= max_lethal_fraction;
  return metrics;
}

double WaitForBarrierClear::computeObservedRadius(
  double minimum_radius_m,
  float observed_extent_m,
  double observed_padding_m)
{
  return std::max(
    std::max(0.05, minimum_radius_m),
    0.5 * std::max(0.0, static_cast<double>(observed_extent_m)) +
    std::max(0.0, observed_padding_m));
}

bool WaitForBarrierClear::shouldUseObservedRegion(
  const AxisAlignedFootprint & semantic_footprint,
  const geometry_msgs::msg::Point & observed_center,
  double maximum_gap_m)
{
  if (!semantic_footprint.valid || !finitePoint(observed_center)) {
    return false;
  }
  const double dx = std::max(
    std::abs(observed_center.x - semantic_footprint.center.x) -
    semantic_footprint.half_x,
    0.0);
  const double dy = std::max(
    std::abs(observed_center.y - semantic_footprint.center.y) -
    semantic_footprint.half_y,
    0.0);
  return std::hypot(dx, dy) <= std::max(0.0, maximum_gap_m);
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

void WaitForBarrierClear::onLocalCostmap(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  latest_local_costmap_ = msg;
  ++local_generation_;
}

BT::NodeStatus WaitForBarrierClear::evaluateRawPrePoll()
{
  const auto map = mapMetrics();
  const auto global = globalMetrics(false);
  const auto local = localMetrics(false);
  const bool clear = map.clear && global.clear && local.clear;

  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForBarrierClear] raw pre poll=%d/%d "
    "map(clear=%s sem=%zu/%zu) "
    "global(clear=%s sem=%zu/%zu gen=%zu) "
    "local(clear=%s sem=%zu/%zu obs=%zu/%zu gen=%zu)",
    pre_poll_index_ + 1,
    max_pre_cleanup_polls_,
    boolText(map.clear),
    map.semantic_region.lethal_cells,
    map.semantic_region.observed_cells,
    boolText(global.clear),
    global.semantic_region.lethal_cells,
    global.semantic_region.observed_cells,
    global_generation_,
    boolText(local.clear),
    local.semantic_region.lethal_cells,
    local.semantic_region.observed_cells,
    local.observed_region.lethal_cells,
    local.observed_region.observed_cells,
    local_generation_);

  if (clear) {
    beginPreClearVerification();
    return BT::NodeStatus::RUNNING;
  }

  ++pre_poll_index_;
  if (pre_poll_index_ >= max_pre_cleanup_polls_) {
    clearance_status_ = "barrier_not_cleared_before_timeout";
    publishOutputs(clearance_status_);
    RCLCPP_WARN(
      node_->get_logger(),
      "[WaitForBarrierClear] raw live occupancy remained blocked or "
      "unconfirmed after %d poll(s); cleanup was not called",
      max_pre_cleanup_polls_);
    return BT::NodeStatus::FAILURE;
  }

  scheduleNextRawPrePoll(pre_poll_index_ == 1 ? second_dwell_s_ : poll_interval_s_);
  return BT::NodeStatus::RUNNING;
}

void WaitForBarrierClear::scheduleNextRawPrePoll(double dwell_s)
{
  clearance_status_ = "waiting_next_raw_pre_poll";
  publishOutputs(clearance_status_);
  phase_ = Phase::kRawPrePollInterval;
  phase_deadline_ = std::chrono::steady_clock::now() + secondsDuration(dwell_s);
}

void WaitForBarrierClear::beginPreClearVerification()
{
  clearance_status_ = "raw_sources_clear_requesting_costmap_refresh";
  publishOutputs(clearance_status_);
  requestCostmapClears();
  phase_ = Phase::kWaitPreClearResponses;
  phase_deadline_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(service_response_timeout_ms_);
}

BT::NodeStatus WaitForBarrierClear::evaluatePreVerification()
{
  const auto map = mapMetrics();
  const auto global = globalMetrics(require_fresh_costmaps_);
  const auto local = localMetrics(require_fresh_costmaps_);
  const bool clear = map.clear && global.clear && local.clear;

  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForBarrierClear] fresh pre verification "
    "map(clear=%s) global(clear=%s fresh=%s) local(clear=%s fresh=%s)",
    boolText(map.clear), boolText(global.clear), boolText(global.fresh),
    boolText(local.clear), boolText(local.fresh));

  if (clear) {
    clearance_status_ = "raw_and_fresh_sources_clear";
    publishOutputs(clearance_status_);
    beginCleanup();
    return BT::NodeStatus::RUNNING;
  }

  ++pre_poll_index_;
  if (pre_poll_index_ >= max_pre_cleanup_polls_) {
    clearance_status_ = "barrier_reappeared_after_costmap_refresh";
    publishOutputs(clearance_status_);
    return BT::NodeStatus::FAILURE;
  }

  scheduleNextRawPrePoll(poll_interval_s_);
  return BT::NodeStatus::RUNNING;
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
  phase_deadline_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(service_ready_timeout_ms_);
#else
  cleanup_modified_ = -3;
  clearance_status_ = "cleanup_interface_unavailable_skipped";
  publishOutputs(clearance_status_);
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
    return BT::NodeStatus::FAILURE;
  }

  auto request = std::make_shared<CleanupService::Request>();
  request->radius = cleanup_radius_cells_;
  request->filter_scans = cleanup_filter_scans_;
  map_generation_before_cleanup_ = map_generation_;
  cleanup_future_ = cleanup_client_->async_send_request(request);
  phase_ = Phase::kWaitCleanupResponse;
  phase_deadline_ = now +
    std::chrono::milliseconds(service_response_timeout_ms_);
  clearance_status_ = "cleanup_requested";
  publishOutputs(clearance_status_);

  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForBarrierClear] cleanup_local_grids requested radius=%d "
    "filter_scans=%s after raw and fresh map/global/local confirmation",
    cleanup_radius_cells_, boolText(cleanup_filter_scans_));
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
    return BT::NodeStatus::FAILURE;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForBarrierClear] cleanup_local_grids completed modified=%d",
    cleanup_modified_);

  if (cleanup_modified_ > 0) {
    phase_ = Phase::kWaitFreshMapAfterCleanup;
    phase_deadline_ = now + std::chrono::milliseconds(fresh_map_timeout_ms_);
    clearance_status_ = "waiting_fresh_map_after_cleanup";
    publishOutputs(clearance_status_);
    return BT::NodeStatus::RUNNING;
  }

  beginPostVerification();
  return BT::NodeStatus::RUNNING;
}
#endif

void WaitForBarrierClear::beginPostVerification()
{
  requestCostmapClears();
  phase_ = Phase::kWaitPostClearResponses;
  phase_deadline_ = std::chrono::steady_clock::now() +
    std::chrono::milliseconds(service_response_timeout_ms_);
  clearance_status_ = "post_cleanup_costmap_refresh";
  publishOutputs(clearance_status_);
}

BT::NodeStatus WaitForBarrierClear::evaluatePostPoll()
{
  const auto map = mapMetrics();
  const auto global = globalMetrics(require_fresh_costmaps_);
  const auto local = localMetrics(require_fresh_costmaps_);
  const bool clear = map.clear && global.clear && local.clear;

  ++post_poll_index_;
  if (clear) {
    ++post_clear_streak_;
  } else {
    post_clear_streak_ = 0;
  }

  RCLCPP_INFO(
    node_->get_logger(),
    "[WaitForBarrierClear] post poll=%d/%d map(clear=%s) "
    "global(clear=%s fresh=%s) local(clear=%s fresh=%s) streak=%d/%d "
    "cleanup_modified=%d",
    post_poll_index_, max_post_cleanup_polls_, boolText(map.clear),
    boolText(global.clear), boolText(global.fresh),
    boolText(local.clear), boolText(local.fresh),
    post_clear_streak_, required_post_cleanup_clear_samples_, cleanup_modified_);

  if (post_clear_streak_ >= required_post_cleanup_clear_samples_) {
    clearance_status_ = "barrier_clear_stabilized";
    publishOutputs(clearance_status_);
    RCLCPP_INFO(
      node_->get_logger(),
      "[WaitForBarrierClear] barrier clear and stabilized; replanning allowed");
    return BT::NodeStatus::SUCCESS;
  }

  if (post_poll_index_ >= max_post_cleanup_polls_) {
    clearance_status_ = "post_cleanup_clearance_unstable";
    publishOutputs(clearance_status_);
    return BT::NodeStatus::FAILURE;
  }

  beginPostVerification();
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

  global_generation_before_clear_ = global_generation_;
  local_generation_before_clear_ = local_generation_;

  if (clear_local_client_->service_is_ready()) {
    clear_local_future_ = clear_local_client_->async_send_request(
      std::make_shared<ClearService::Request>());
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "[WaitForBarrierClear] local costmap clear service not ready");
  }

  if (clear_global_client_->service_is_ready()) {
    clear_global_future_ = clear_global_client_->async_send_request(
      std::make_shared<ClearService::Request>());
  } else {
    RCLCPP_WARN(
      node_->get_logger(),
      "[WaitForBarrierClear] global costmap clear service not ready");
  }
}

bool WaitForBarrierClear::clearRequestsFinished()
{
  bool finished = true;
  if (clear_local_future_) {
    if (futureReady(*clear_local_future_)) {
      (void)clear_local_future_->future.get();
      clear_local_future_.reset();
    } else {
      finished = false;
    }
  }
  if (clear_global_future_) {
    if (futureReady(*clear_global_future_)) {
      (void)clear_global_future_->future.get();
      clear_global_future_.reset();
    } else {
      finished = false;
    }
  }
  return finished;
}

bool WaitForBarrierClear::costmapsFreshAfterClear() const
{
  return global_generation_ > global_generation_before_clear_ &&
         local_generation_ > local_generation_before_clear_;
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

WaitForBarrierClear::SourceMetrics WaitForBarrierClear::sourceMetrics(
  const nav_msgs::msg::OccupancyGrid::SharedPtr & grid,
  bool fresh,
  bool include_observed_region,
  int lethal_threshold) const
{
  SourceMetrics metrics;
  metrics.fresh = fresh;
  metrics.observed_region_used = include_observed_region && use_observed_region_;
  if (!grid) {
    return metrics;
  }

  metrics.semantic_region = sampleFootprintRegion(
    *grid,
    semantic_footprint_,
    lethal_threshold,
    max_lethal_fraction_,
    max_lethal_cells_,
    min_observed_cells_);

  bool observed_clear = true;
  if (metrics.observed_region_used) {
    metrics.observed_region = sampleCircularRegion(
      *grid,
      observed_blockage_center_,
      observed_radius_m_,
      lethal_threshold,
      max_lethal_fraction_,
      max_lethal_cells_,
      min_observed_cells_);
    observed_clear = metrics.observed_region.clear;
  }

  metrics.clear = metrics.fresh && metrics.semantic_region.clear && observed_clear;
  return metrics;
}

WaitForBarrierClear::SourceMetrics WaitForBarrierClear::mapMetrics() const
{
  return sourceMetrics(latest_map_, true, false, map_lethal_threshold_);
}

WaitForBarrierClear::SourceMetrics WaitForBarrierClear::globalMetrics(
  bool require_fresh) const
{
  const bool fresh = !require_fresh ||
    global_generation_ > global_generation_before_clear_;
  return sourceMetrics(
    latest_global_costmap_, fresh, false, costmap_lethal_threshold_);
}

WaitForBarrierClear::SourceMetrics WaitForBarrierClear::localMetrics(
  bool require_fresh) const
{
  const bool fresh = !require_fresh ||
    local_generation_ > local_generation_before_clear_;
  return sourceMetrics(
    latest_local_costmap_, fresh, true, costmap_lethal_threshold_);
}

bool WaitForBarrierClear::allSourcesClear(bool require_fresh_costmaps) const
{
  return mapMetrics().clear &&
         globalMetrics(require_fresh_costmaps).clear &&
         localMetrics(require_fresh_costmaps).clear;
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
