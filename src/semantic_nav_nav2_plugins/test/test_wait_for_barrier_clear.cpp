// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include <gtest/gtest.h>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "semantic_nav_nav2_plugins/wait_for_barrier_clear.hpp"

namespace
{

nav_msgs::msg::OccupancyGrid makeGrid(
  int width,
  int height,
  float resolution,
  int8_t fill)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = static_cast<uint32_t>(width);
  grid.info.height = static_cast<uint32_t>(height);
  grid.info.resolution = resolution;
  grid.data.assign(
    static_cast<std::size_t>(width * height), fill);
  return grid;
}

geometry_msgs::msg::Point point(double x, double y)
{
  geometry_msgs::msg::Point p;
  p.x = x;
  p.y = y;
  return p;
}

}  // namespace

TEST(WaitForBarrierClearTest, exposesM3Ports)
{
  const auto ports =
    semantic_nav_nav2_plugins::WaitForBarrierClear::providedPorts();

  EXPECT_GT(ports.count("barrier_center"), 0u);
  EXPECT_GT(ports.count("barrier_extent_m"), 0u);
  EXPECT_GT(ports.count("initial_dwell_s"), 0u);
  EXPECT_GT(ports.count("cleanup_service"), 0u);
  EXPECT_GT(ports.count("cleanup_modified"), 0u);
  EXPECT_GT(ports.count("clearance_status"), 0u);
}

TEST(WaitForBarrierClearTest, registersWithoutError)
{
  BT::BehaviorTreeFactory factory;
  EXPECT_NO_THROW(
    factory.registerNodeType<
      semantic_nav_nav2_plugins::WaitForBarrierClear>(
      "WaitForBarrierClear"));
}

TEST(WaitForBarrierClearTest, clearObservedRegionPasses)
{
  const auto grid = makeGrid(40, 40, 0.05f, 0);
  const auto metrics =
    semantic_nav_nav2_plugins::WaitForBarrierClear::sampleRegion(
    grid, point(1.0, 1.0), 0.30, 100, 0.15, 8);

  EXPECT_TRUE(metrics.clear);
  EXPECT_GE(metrics.observed_cells, 8u);
  EXPECT_EQ(metrics.lethal_cells, 0u);
}

TEST(WaitForBarrierClearTest, lethalBarrierFails)
{
  auto grid = makeGrid(40, 40, 0.05f, 0);
  for (int my = 16; my <= 24; ++my) {
    for (int mx = 16; mx <= 24; ++mx) {
      grid.data[static_cast<std::size_t>(my * 40 + mx)] = 100;
    }
  }

  const auto metrics =
    semantic_nav_nav2_plugins::WaitForBarrierClear::sampleRegion(
    grid, point(1.0, 1.0), 0.30, 100, 0.15, 8);

  EXPECT_FALSE(metrics.clear);
  EXPECT_GT(metrics.lethal_fraction, 0.15);
}

TEST(WaitForBarrierClearTest, unknownRegionIsUnconfirmed)
{
  const auto grid = makeGrid(40, 40, 0.05f, -1);
  const auto metrics =
    semantic_nav_nav2_plugins::WaitForBarrierClear::sampleRegion(
    grid, point(1.0, 1.0), 0.30, 100, 0.15, 8);

  EXPECT_FALSE(metrics.clear);
  EXPECT_EQ(metrics.observed_cells, 0u);
}

TEST(WaitForBarrierClearTest, negativeOriginUsesFloorIndexing)
{
  auto grid = makeGrid(20, 20, 0.1f, 0);
  grid.info.origin.position.x = -1.0;
  grid.info.origin.position.y = -1.0;
  grid.data[5 * 20 + 5] = 100;

  const auto metrics =
    semantic_nav_nav2_plugins::WaitForBarrierClear::sampleRegion(
    grid, point(-0.45, -0.45), 0.08, 100, 0.15, 1);

  EXPECT_FALSE(metrics.clear);
  EXPECT_EQ(metrics.lethal_cells, 1u);
}
