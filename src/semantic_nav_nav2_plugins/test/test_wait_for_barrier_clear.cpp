// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include <gtest/gtest.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "semantic_nav_nav2_plugins/wait_for_barrier_clear.hpp"

namespace
{
nav_msgs::msg::OccupancyGrid makeGrid(
  int width, int height, float resolution, int8_t fill)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = static_cast<uint32_t>(width);
  grid.info.height = static_cast<uint32_t>(height);
  grid.info.resolution = resolution;
  grid.data.assign(static_cast<std::size_t>(width * height), fill);
  return grid;
}

geometry_msgs::msg::Point point(double x, double y)
{
  geometry_msgs::msg::Point value;
  value.x = x;
  value.y = y;
  return value;
}

geometry_msgs::msg::Vector3 extent(double x, double y)
{
  geometry_msgs::msg::Vector3 value;
  value.x = x;
  value.y = y;
  return value;
}
}  // namespace

TEST(WaitForBarrierClearTest, validBboxIsUsedWithoutClassSpecificRadius)
{
  const auto footprint =
    semantic_nav_nav2_plugins::WaitForBarrierClear::computeSemanticFootprint(
      point(0.0, 0.0), extent(0.80, 0.35), 0.30, 0.05);
  EXPECT_TRUE(footprint.valid);
  EXPECT_TRUE(footprint.uses_bbox);
  EXPECT_NEAR(footprint.half_x, 0.45, 1e-9);
  EXPECT_NEAR(footprint.half_y, 0.225, 1e-9);
}

TEST(WaitForBarrierClearTest, doorwayCanUseCenterCircleInsteadOfFullDoorBbox)
{
  const auto footprint =
    semantic_nav_nav2_plugins::WaitForBarrierClear::computeSemanticFootprint(
      point(4.862, -0.677), extent(0.20, 0.90), 0.30, 0.05, false);
  EXPECT_FALSE(footprint.uses_bbox);
  EXPECT_NEAR(footprint.half_x, 0.30, 1e-9);
  EXPECT_NEAR(footprint.half_y, 0.30, 1e-9);
}

TEST(WaitForBarrierClearTest, missingBboxUsesGenericFallback)
{
  const auto footprint =
    semantic_nav_nav2_plugins::WaitForBarrierClear::computeSemanticFootprint(
      point(0.0, 0.0), extent(0.0, 0.0), 0.30, 0.05);
  EXPECT_FALSE(footprint.uses_bbox);
  EXPECT_NEAR(footprint.half_x, 0.30, 1e-9);
  EXPECT_NEAR(footprint.half_y, 0.30, 1e-9);
}

TEST(WaitForBarrierClearTest, sparseDynamicEvidenceFailsAbsoluteCap)
{
  auto grid = makeGrid(40, 40, 0.05f, 0);
  grid.data[20 * 40 + 20] = 100;
  grid.data[20 * 40 + 21] = 100;
  grid.data[21 * 40 + 20] = 100;
  const auto footprint =
    semantic_nav_nav2_plugins::WaitForBarrierClear::computeSemanticFootprint(
      point(1.0, 1.0), extent(0.70, 0.70), 0.30, 0.05);
  const auto metrics =
    semantic_nav_nav2_plugins::WaitForBarrierClear::sampleFootprintRegion(
      grid, footprint, 90, 0.15, 1, 8);
  EXPECT_FALSE(metrics.clear);
  EXPECT_EQ(metrics.lethal_cells, 3u);
}

TEST(WaitForBarrierClearTest, oneNoisyCellIsAccepted)
{
  auto grid = makeGrid(40, 40, 0.05f, 0);
  grid.data[20 * 40 + 20] = 100;
  const auto footprint =
    semantic_nav_nav2_plugins::WaitForBarrierClear::computeSemanticFootprint(
      point(1.0, 1.0), extent(0.70, 0.70), 0.30, 0.05);
  const auto metrics =
    semantic_nav_nav2_plugins::WaitForBarrierClear::sampleFootprintRegion(
      grid, footprint, 90, 0.15, 1, 8);
  EXPECT_TRUE(metrics.clear);
}

TEST(WaitForBarrierClearTest, observedLegClusterAssociatesWithBbox)
{
  const auto footprint =
    semantic_nav_nav2_plugins::WaitForBarrierClear::computeSemanticFootprint(
      point(-2.507, -1.350), extent(0.70, 0.70), 0.30, 0.05);
  EXPECT_TRUE(
    semantic_nav_nav2_plugins::WaitForBarrierClear::shouldUseObservedRegion(
      footprint, point(-2.125, -1.608), 0.15));
}

TEST(WaitForBarrierClearTest, distantWallClusterIsRejected)
{
  const auto footprint =
    semantic_nav_nav2_plugins::WaitForBarrierClear::computeSemanticFootprint(
      point(4.862, -0.677), extent(0.20, 0.90), 0.30, 0.05);
  EXPECT_FALSE(
    semantic_nav_nav2_plugins::WaitForBarrierClear::shouldUseObservedRegion(
      footprint, point(4.822, -1.355), 0.15));
}


TEST(WaitForBarrierClearTest, rawGateRequiresOnlyMapForMapConfirmedChange)
{
  // Mode A's mandatory raw prerequisite is "/map confirms the physical
  // change" -- NOT "raw map AND raw global AND raw local all clear". Raw
  // Nav2 costmaps may contain exactly the stale data their clear services
  // exist to remove, which would otherwise deadlock the gate.
  using Node = semantic_nav_nav2_plugins::WaitForBarrierClear;

  EXPECT_TRUE(Node::rawGateSatisfied("map_confirmed_change", true));
  EXPECT_FALSE(Node::rawGateSatisfied("map_confirmed_change", false));

  // Mode B records raw occupancy as a diagnostic only: departure was already
  // independently proven by WaitForDynamicObstacleDeparture.
  EXPECT_TRUE(Node::rawGateSatisfied("track_confirmed_departure", true));
  EXPECT_TRUE(Node::rawGateSatisfied("track_confirmed_departure", false));
}

TEST(WaitForBarrierClearTest, preCleanupEligibilityDoesNotRequireGlobalClearance)
{
  using Node = semantic_nav_nav2_plugins::WaitForBarrierClear;

  // Explicit intervention: /map and the fresh local costmap are sufficient
  // to invoke cached-grid cleanup. The global costmap may contain the stale
  // evidence that cleanup is intended to remove.
  EXPECT_TRUE(Node::preCleanupSourcesReady(
    "map_confirmed_change", true, true));

  EXPECT_FALSE(Node::preCleanupSourcesReady(
    "map_confirmed_change", false, true));

  EXPECT_FALSE(Node::preCleanupSourcesReady(
    "map_confirmed_change", true, false));

  // Tracked departure was confirmed independently. The local costmap is the
  // hard pre-cleanup occupancy gate.
  EXPECT_TRUE(Node::preCleanupSourcesReady(
    "track_confirmed_departure", false, true));

  EXPECT_FALSE(Node::preCleanupSourcesReady(
    "track_confirmed_departure", true, false));
}

TEST(WaitForBarrierClearTest, verifiedSourcesFollowClearanceMode)
{
  using Node = semantic_nav_nav2_plugins::WaitForBarrierClear;

  // Mode A post-gate: /map (physical-change proof) AND the fresh local costmap
  // (immediate traversability) must verify. The global costmap is excluded --
  // consistent with preCleanupSourcesReady -- because its inflation layer can
  // amplify a single tolerated /map residual cell into a region-flooding stale
  // occupancy that cleanup_local_grids cannot remove.
  EXPECT_TRUE(Node::verifiedSourcesClear("map_confirmed_change", true, true, true));
  EXPECT_TRUE(Node::verifiedSourcesClear("map_confirmed_change", true, false, true));
  EXPECT_FALSE(Node::verifiedSourcesClear("map_confirmed_change", false, true, true));
  EXPECT_FALSE(Node::verifiedSourcesClear("map_confirmed_change", true, true, false));

  // Mode B: the fresh LOCAL costmap is the hard gate; /map and global
  // residuals are advisory (rays through the vacated region may never
  // terminate, so they can stay stale indefinitely).
  EXPECT_TRUE(Node::verifiedSourcesClear("track_confirmed_departure", false, false, true));
  EXPECT_FALSE(Node::verifiedSourcesClear("track_confirmed_departure", true, true, false));
}

TEST(WaitForBarrierClearTest, mapConfirmedPostGateIgnoresGlobalInflation)
{
  using Node = semantic_nav_nav2_plugins::WaitForBarrierClear;

  // Mechanism B: after the operator intervention, /map
  // confirms the object is gone (sem <= max_lethal_cells) and the fresh local
  // costmap confirms immediate traversability, but the GLOBAL costmap inflates
  // the single tolerated /map residual into ~60 lethal-by-threshold cells that
  // cleanup_local_grids cannot carve. Requiring global here makes success hinge
  // on how deeply RTAB happened to bake the blocker that run -- so the post-gate
  // must NOT require the global costmap.
  EXPECT_TRUE(Node::verifiedSourcesClear("map_confirmed_change", true, false, true));

  // /map stays mandatory: if the physical change is not confirmed on /map, the
  // post-gate fails regardless of the other sources.
  EXPECT_FALSE(Node::verifiedSourcesClear("map_confirmed_change", false, false, true));
  EXPECT_FALSE(Node::verifiedSourcesClear("map_confirmed_change", false, true, true));

  // The fresh local costmap stays a hard gate: no immediate traversability ->
  // fail, so a still-present blocker can never be waved through.
  EXPECT_FALSE(Node::verifiedSourcesClear("map_confirmed_change", true, true, false));
}

TEST(WaitForBarrierClearTest, freshnessGateFollowsClearanceMode)
{
  using Node = semantic_nav_nav2_plugins::WaitForBarrierClear;

  // Mode A requires BOTH the local and global costmaps to refresh after the
  // clear before verification proceeds.
  EXPECT_TRUE(Node::freshnessSatisfiedAfterClear("map_confirmed_change", true, true));
  EXPECT_FALSE(Node::freshnessSatisfiedAfterClear("map_confirmed_change", true, false));
  EXPECT_FALSE(Node::freshnessSatisfiedAfterClear("map_confirmed_change", false, true));

  // Mode B: global occupancy is advisory, so global freshness is advisory too
  // -- only the local costmap (the hard gate) must refresh. A late global
  // publication must not fail a confirmed tracked departure.
  EXPECT_TRUE(Node::freshnessSatisfiedAfterClear("track_confirmed_departure", true, false));
  EXPECT_FALSE(Node::freshnessSatisfiedAfterClear("track_confirmed_departure", false, true));
}

TEST(WaitForBarrierClearTest, freshMapAfterCleanupIsAdvisoryOnlyInTrackMode)
{
  using Node = semantic_nav_nav2_plugins::WaitForBarrierClear;

  // Mode A: a modified-grid cleanup that yields no fresh /map is a hard
  // failure -- the physical change was never confirmed on the map.
  EXPECT_FALSE(Node::cleanupMapWaitIsAdvisory("map_confirmed_change"));

  // Mode B: /map is advisory; a missing fresh /map after the advisory cleanup
  // continues with the local hard-gate verification instead of failing (the
  // rover's /map can be frozen while parked during the stationary dwell).
  EXPECT_TRUE(Node::cleanupMapWaitIsAdvisory("track_confirmed_departure"));
}

TEST(WaitForBarrierClearTest, observedRadiusIsCappedByMaximum)
{
  using Node = semantic_nav_nav2_plugins::WaitForBarrierClear;

  // radius = min(max(min_r, extent/2 + padding), max_r): a wide measured
  // blockage must not grow the sampled region onto corridor walls.
  EXPECT_NEAR(Node::computeObservedRadius(0.15, 0.20F, 0.05, 0.30), 0.15, 1e-6);
  EXPECT_NEAR(Node::computeObservedRadius(0.15, 0.40F, 0.05, 0.30), 0.25, 1e-6);
  EXPECT_NEAR(Node::computeObservedRadius(0.15, 1.45F, 0.05, 0.30), 0.30, 1e-6);
}

TEST(WaitForBarrierClearTest, negativeOriginUsesFloorIndexing)
{
  auto grid = makeGrid(20, 20, 0.1f, 0);
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -1.0;
  grid.data[5 * 20 + 5] = 100;
  const auto metrics =
    semantic_nav_nav2_plugins::WaitForBarrierClear::sampleCircularRegion(
      grid, point(-0.45, -0.45), 0.08, 90, 0.15, 0, 1);
  EXPECT_FALSE(metrics.clear);
  EXPECT_EQ(metrics.lethal_cells, 1u);
}
