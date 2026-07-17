// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav2_msgs/srv/clear_entire_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

#if __has_include("rtabmap_msgs/srv/cleanup_local_grids.hpp")
#define SEMANTIC_NAV_HAS_RTABMAP_CLEANUP_LOCAL_GRIDS 1
#include "rtabmap_msgs/srv/cleanup_local_grids.hpp"
#else
#define SEMANTIC_NAV_HAS_RTABMAP_CLEANUP_LOCAL_GRIDS 0
#endif

namespace semantic_nav_nav2_plugins
{

/**
 * @brief Confirm that a removed barrier has disappeared from the live map and
 * both Nav2 costmaps before cached RTAB-Map local grids are cleaned.
 *
 * Geometry is object-agnostic:
 * - valid semantic bbox geometry is sampled as an axis-aligned footprint;
 * - a generic circular fallback is used only when bbox geometry is absent;
 * - the measured blockage region is sampled separately in the local costmap.
 *
 * Raw occupancy is inspected before any costmap clear request. This prevents a
 * live dynamic object from being hidden temporarily by the clear service.
 */
class WaitForBarrierClear : public BT::StatefulActionNode
{
public:
  using ClearService = nav2_msgs::srv::ClearEntireCostmap;
  using ClearClient = rclcpp::Client<ClearService>;
  using ClearFuture = typename ClearClient::FutureAndRequestId;

#if SEMANTIC_NAV_HAS_RTABMAP_CLEANUP_LOCAL_GRIDS
  using CleanupService = rtabmap_msgs::srv::CleanupLocalGrids;
  using CleanupClient = rclcpp::Client<CleanupService>;
  using CleanupFuture = typename CleanupClient::FutureAndRequestId;
#endif

  struct RegionMetrics
  {
    std::size_t observed_cells{0};
    std::size_t lethal_cells{0};
    double lethal_fraction{0.0};
    bool clear{false};
  };

  struct AxisAlignedFootprint
  {
    geometry_msgs::msg::Point center{};
    double half_x{0.0};
    double half_y{0.0};
    bool uses_bbox{false};
    bool valid{false};
  };

  struct SourceMetrics
  {
    RegionMetrics semantic_region{};
    RegionMetrics observed_region{};
    bool observed_region_used{false};
    bool fresh{true};
    bool clear{false};
  };

  WaitForBarrierClear(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts();

  static AxisAlignedFootprint computeSemanticFootprint(
    const geometry_msgs::msg::Point & center,
    const geometry_msgs::msg::Vector3 & bbox_extent,
    double fallback_radius_m,
    double bbox_padding_m,
    bool use_bbox_footprint = true);

  static RegionMetrics sampleFootprintRegion(
    const nav_msgs::msg::OccupancyGrid & grid,
    const AxisAlignedFootprint & footprint,
    int lethal_threshold,
    double max_lethal_fraction,
    int max_lethal_cells,
    int min_observed_cells);

  static RegionMetrics sampleCircularRegion(
    const nav_msgs::msg::OccupancyGrid & grid,
    const geometry_msgs::msg::Point & center,
    double radius_m,
    int lethal_threshold,
    double max_lethal_fraction,
    int max_lethal_cells,
    int min_observed_cells);

  static double computeObservedRadius(
    double minimum_radius_m,
    float observed_extent_m,
    double observed_padding_m);

  static bool shouldUseObservedRegion(
    const AxisAlignedFootprint & semantic_footprint,
    const geometry_msgs::msg::Point & observed_center,
    double maximum_gap_m);

private:
  enum class Phase
  {
    kInitialDwell,
    kRawPrePollInterval,
    kWaitPreClearResponses,
    kWaitPreFreshCostmaps,
    kWaitPreVerificationInterval,
    kWaitCleanupService,
    kWaitCleanupResponse,
    kWaitFreshMapAfterCleanup,
    kWaitPostClearResponses,
    kWaitPostFreshCostmaps,
    kWaitPostPollInterval
  };

  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onGlobalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onLocalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  BT::NodeStatus evaluateRawPrePoll();
  BT::NodeStatus evaluatePreVerification();
  void scheduleNextRawPrePoll(double dwell_s = 0.0);
  void beginPreClearVerification();

  void beginCleanup();
#if SEMANTIC_NAV_HAS_RTABMAP_CLEANUP_LOCAL_GRIDS
  BT::NodeStatus handleCleanupServiceWait();
  BT::NodeStatus handleCleanupResponse();
#endif
  void beginPostVerification();
  BT::NodeStatus evaluatePostPoll();

  void requestCostmapClears();
  bool clearRequestsFinished();
  bool costmapsFreshAfterClear() const;
  void abandonPendingRequests();

  SourceMetrics sourceMetrics(
    const nav_msgs::msg::OccupancyGrid::SharedPtr & grid,
    bool fresh,
    bool include_observed_region,
    int lethal_threshold) const;
  SourceMetrics mapMetrics() const;
  SourceMetrics globalMetrics(bool require_fresh) const;
  SourceMetrics localMetrics(bool require_fresh) const;
  bool allSourcesClear(bool require_fresh_costmaps) const;

  void publishOutputs(const std::string & status);

  static bool futureReady(const ClearFuture & future);
#if SEMANTIC_NAV_HAS_RTABMAP_CLEANUP_LOCAL_GRIDS
  static bool futureReady(const CleanupFuture & future);
#endif

  rclcpp::Node::SharedPtr node_;
  rclcpp::CallbackGroup::SharedPtr callback_group_;
  rclcpp::executors::SingleThreadedExecutor callback_group_executor_;

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr global_costmap_sub_;
  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr local_costmap_sub_;

  ClearClient::SharedPtr clear_local_client_;
  ClearClient::SharedPtr clear_global_client_;
#if SEMANTIC_NAV_HAS_RTABMAP_CLEANUP_LOCAL_GRIDS
  CleanupClient::SharedPtr cleanup_client_;
#endif

  std::optional<ClearFuture> clear_local_future_;
  std::optional<ClearFuture> clear_global_future_;
#if SEMANTIC_NAV_HAS_RTABMAP_CLEANUP_LOCAL_GRIDS
  std::optional<CleanupFuture> cleanup_future_;
#endif

  nav_msgs::msg::OccupancyGrid::SharedPtr latest_map_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_global_costmap_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_local_costmap_;
  std::size_t map_generation_{0};
  std::size_t global_generation_{0};
  std::size_t local_generation_{0};
  std::size_t global_generation_before_clear_{0};
  std::size_t local_generation_before_clear_{0};
  std::size_t map_generation_before_cleanup_{0};

  Phase phase_{Phase::kInitialDwell};
  std::chrono::steady_clock::time_point phase_deadline_{};

  geometry_msgs::msg::Point barrier_center_{};
  geometry_msgs::msg::Vector3 barrier_bbox_extent_{};
  AxisAlignedFootprint semantic_footprint_{};
  geometry_msgs::msg::Point observed_blockage_center_{};
  float observed_blockage_extent_m_{0.0F};
  double observed_radius_m_{0.20};
  bool use_observed_region_{false};

  double clear_radius_m_{0.30};
  double bbox_padding_m_{0.05};
  bool use_bbox_footprint_{true};
  double observed_min_radius_m_{0.15};
  double observed_padding_m_{0.05};
  double observed_center_max_gap_m_{0.15};

  int map_lethal_threshold_{100};
  int costmap_lethal_threshold_{90};
  double max_lethal_fraction_{0.15};
  int max_lethal_cells_{1};
  int min_observed_cells_{8};
  bool require_fresh_costmaps_{true};

  double initial_dwell_s_{12.0};
  double second_dwell_s_{12.0};
  double poll_interval_s_{2.0};
  int max_pre_cleanup_polls_{6};
  int max_post_cleanup_polls_{6};
  int required_post_cleanup_clear_samples_{2};

  bool cleanup_local_grids_{true};
  int cleanup_radius_cells_{1};
  bool cleanup_filter_scans_{false};
  int service_ready_timeout_ms_{2000};
  int service_response_timeout_ms_{30000};
  int fresh_costmap_timeout_ms_{5000};
  int fresh_map_timeout_ms_{10000};

  int pre_poll_index_{0};
  int post_poll_index_{0};
  int post_clear_streak_{0};
  int cleanup_modified_{-2};
  std::string clearance_status_{"not_started"};
};

}  // namespace semantic_nav_nav2_plugins
