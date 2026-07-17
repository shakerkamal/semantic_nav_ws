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
