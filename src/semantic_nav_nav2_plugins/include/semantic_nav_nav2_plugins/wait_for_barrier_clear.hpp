// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#pragma once

#include <chrono>
#include <cstddef>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp_v3/action_node.h"
#include "geometry_msgs/msg/point.hpp"
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
 * @brief Wait for a removed semantic barrier to disappear from the live map.
 *
 * This node ports the rover's deterministic up-front re-observation policy into
 * the BT-led en-route recovery path. It never commands motion. The rover is
 * already at a standoff facing the responsible object, so RTAB-Map can use
 * map_always_update=true and Grid/RayTracing=true while the robot remains
 * stationary.
 *
 * Sequence:
 *   1. Dwell while the live /map and global costmap converge.
 *   2. Best-effort clear Nav2 local/global costmaps and poll both maps.
 *   3. Require the barrier footprint to be clear in BOTH /map and the global
 *      costmap before calling /rtabmap/cleanup_local_grids.
 *   4. Cleanup cached per-node local grids with filter_scans=false.
 *   5. Wait for RTAB-Map's republished map (when grids were modified), clear
 *      Nav2 costmaps again, and require consecutive clear samples.
 *
 * cleanup_local_grids is intentionally invoked only after the current live map
 * is clear. Calling it while the barrier is still occupied could make stale or
 * geometrically incorrect evidence persistent.
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

  WaitForBarrierClear(
    const std::string & name,
    const BT::NodeConfiguration & conf);

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  static BT::PortsList providedPorts();

  static RegionMetrics sampleRegion(
    const nav_msgs::msg::OccupancyGrid & grid,
    const geometry_msgs::msg::Point & center,
    double radius_m,
    int lethal_threshold,
    double max_lethal_fraction,
    int min_observed_cells);

private:
  enum class Phase
  {
    kDwellBeforePrePoll,
    kWaitPreClearResponses,
    kWaitPrePollInterval,
    kWaitCleanupService,
    kWaitCleanupResponse,
    kWaitFreshMapAfterCleanup,
    kWaitPostClearResponses,
    kWaitPostPollInterval
  };

  void onMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
  void onGlobalCostmap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  void beginPrePoll();
  BT::NodeStatus evaluatePrePoll();
  void scheduleNextPrePoll();

  void beginCleanup();
#if SEMANTIC_NAV_HAS_RTABMAP_CLEANUP_LOCAL_GRIDS
  BT::NodeStatus handleCleanupServiceWait();
  BT::NodeStatus handleCleanupResponse();
#endif
  void beginPostVerification();

  void beginPostPoll();
  BT::NodeStatus evaluatePostPoll();

  void requestCostmapClears();
  bool clearRequestsFinished();
  void abandonPendingRequests();

  RegionMetrics mapMetrics() const;
  RegionMetrics globalMetrics() const;
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
  std::size_t map_generation_{0};
  std::size_t global_generation_{0};
  std::size_t map_generation_before_cleanup_{0};

  Phase phase_{Phase::kDwellBeforePrePoll};
  std::chrono::steady_clock::time_point phase_deadline_;

  geometry_msgs::msg::Point barrier_center_;
  // Must remain float because the shared BT blackboard key
  // {blockage_extent_m} is produced as float by the existing nodes.
  float barrier_extent_m_{0.0F};
  double clear_radius_m_{0.30};
  int lethal_threshold_{100};
  double max_lethal_fraction_{0.15};
  int min_observed_cells_{8};

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
  int fresh_map_timeout_ms_{10000};

  int pre_poll_index_{0};
  int post_poll_index_{0};
  int post_clear_streak_{0};
  int cleanup_modified_{-2};
  std::string clearance_status_{"not_started"};
};

}  // namespace semantic_nav_nav2_plugins