// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include <gtest/gtest.h>

#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "semantic_nav_nav2_plugins/persistent_no_progress_condition.hpp"

namespace
{

nav_msgs::msg::OccupancyGrid makeCostmap(
  int width,
  int height,
  float resolution,
  float origin_x = 0.0F,
  float origin_y = 0.0F)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = static_cast<uint32_t>(width);
  grid.info.height = static_cast<uint32_t>(height);
  grid.info.resolution = resolution;
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.data.assign(static_cast<std::size_t>(width * height), 0);
  return grid;
}

void setCellValue(
  nav_msgs::msg::OccupancyGrid & grid,
  int mx,
  int my,
  int8_t value)
{
  const std::size_t index =
    static_cast<std::size_t>(my) * grid.info.width +
    static_cast<std::size_t>(mx);
  grid.data.at(index) = value;
}

void setLethalCell(
  nav_msgs::msg::OccupancyGrid & grid,
  int mx,
  int my)
{
  const std::size_t index =
    static_cast<std::size_t>(my) * grid.info.width +
    static_cast<std::size_t>(mx);
  grid.data.at(index) = 100;
}

nav_msgs::msg::Path makeDetouringPath()
{
  nav_msgs::msg::Path path;
  const double points[][2] = {
    {1.05, 1.05},
    {1.25, 1.15},
    {1.45, 1.35},
    {1.65, 1.55},
    {1.95, 1.70},
    {2.30, 1.70},
  };

  for (const auto & point : points) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = point[0];
    pose.pose.position.y = point[1];
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace

TEST(PersistentNoProgressConditionTest, providesRequiredPorts)
{
  const auto ports =
    semantic_nav_nav2_plugins::PersistentNoProgressCondition::providedPorts();

  ASSERT_GT(ports.count("path"), 0u);
  ASSERT_GT(ports.count("goal"), 0u);
  ASSERT_GT(ports.count("observation_window_s"), 0u);
  ASSERT_GT(ports.count("blockage_centroid"), 0u);
  ASSERT_GT(ports.count("blockage_extent_m"), 0u);
  ASSERT_GT(ports.count("monitor_status"), 0u);

  ASSERT_NE(ports.at("blockage_extent_m").type(), nullptr);
  EXPECT_EQ(*ports.at("blockage_extent_m").type(), typeid(float));
}

TEST(PersistentNoProgressConditionTest, detectsObstacleAheadWhenPathCurvesAroundIt)
{
  auto grid = makeCostmap(60, 40, 0.1F);
  auto path = makeDetouringPath();

  // A 3x3 obstacle cluster is directly ahead of the robot around (1.65,1.05),
  // while the fresh path bends upward around it. The expanded centerline test
  // can miss this placement; the forward execution corridor must still catch it.
  for (int my = 9; my <= 11; ++my) {
    for (int mx = 15; mx <= 17; ++mx) {
      setLethalCell(grid, mx, my);
    }
  }

  const auto evidence =
    semantic_nav_nav2_plugins::PersistentNoProgressCondition::
    detectObstacleEvidence(
      path, grid, 1.05, 1.05,
      90, 1.5, 0.10, 0.45, 0.15, 3);

  EXPECT_TRUE(evidence.blocked);
  EXPECT_GE(evidence.lethal_cells, 3);
  EXPECT_EQ(evidence.source, "forward_execution_corridor");
  EXPECT_GT(evidence.centroid.x, 1.4);
  EXPECT_GT(evidence.extent_m, 0.0F);
}

TEST(PersistentNoProgressConditionTest, ignoresObstacleBehindRobot)
{
  auto grid = makeCostmap(60, 40, 0.1F);
  auto path = makeDetouringPath();

  for (int my = 9; my <= 11; ++my) {
    for (int mx = 4; mx <= 6; ++mx) {
      setLethalCell(grid, mx, my);
    }
  }

  const auto evidence =
    semantic_nav_nav2_plugins::PersistentNoProgressCondition::
    detectObstacleEvidence(
      path, grid, 1.05, 1.05,
      90, 1.5, 0.10, 0.45, 0.15, 3);

  EXPECT_FALSE(evidence.blocked);
}

TEST(PersistentNoProgressConditionTest, detectsObstacleInsideExpandedPathCorridor)
{
  auto grid = makeCostmap(60, 40, 0.1F);
  auto path = makeDetouringPath();

  // Cell center near the second path segment, inside the 0.20 m expanded
  // controller corridor.
  setLethalCell(grid, 13, 12);

  const auto evidence =
    semantic_nav_nav2_plugins::PersistentNoProgressCondition::
    detectObstacleEvidence(
      path, grid, 1.05, 1.05,
      90, 1.5, 0.20, 0.30, 0.15, 3);

  EXPECT_TRUE(evidence.blocked);
  EXPECT_EQ(evidence.source, "expanded_path_corridor");
}

TEST(PersistentNoProgressConditionTest, malformedCostmapFailsOpen)
{
  auto grid = makeCostmap(20, 20, 0.1F);
  grid.data.resize(3);
  auto path = makeDetouringPath();

  const auto evidence =
    semantic_nav_nav2_plugins::PersistentNoProgressCondition::
    detectObstacleEvidence(
      path, grid, 1.05, 1.05,
      90, 1.5, 0.20, 0.40, 0.15, 3);

  EXPECT_FALSE(evidence.blocked);
}


TEST(PersistentNoProgressConditionTest, inscribedInflationAloneDoesNotBlockAtTrueLethalThreshold)
{
  // S3 2026-07-17 final stall: 45 "lethal" cells at threshold 90 were the
  // partition walls' INSCRIBED band (cost 99), counted as obstacle evidence
  // while the rotation shim was still settling -- killing an attempt that
  // was about to succeed. Inflation is a planning artifact, not physical
  // evidence: at threshold 100 an inscribed-only corridor must not block,
  // and the 99/100 diagnostic split must expose what was actually there.
  auto grid = makeCostmap(60, 40, 0.1F);
  auto path = makeDetouringPath();

  for (int my = 9; my <= 11; ++my) {
    for (int mx = 15; mx <= 17; ++mx) {
      setCellValue(grid, mx, my, 99);
    }
  }

  const auto evidence =
    semantic_nav_nav2_plugins::PersistentNoProgressCondition::
    detectObstacleEvidence(
      path, grid, 1.05, 1.05,
      100, 1.5, 0.10, 0.45, 0.15, 3);

  EXPECT_FALSE(evidence.blocked);
  EXPECT_EQ(evidence.true_lethal_cells, 0);
  // The corridor's longitudinal/lateral window clips the painted 3x3
  // cluster's edges; >=3 proves the split counting without depending on
  // exact clipping geometry (same bound the reference test uses).
  EXPECT_GE(evidence.inscribed_cells, 3);
}

TEST(PersistentNoProgressConditionTest, trueLethalCellsStillBlockAtThreshold100)
{
  auto grid = makeCostmap(60, 40, 0.1F);
  auto path = makeDetouringPath();

  // Inscribed side walls plus a genuine 100-cost obstacle: the stall must
  // fire from the exact-100 cells and the diagnostics must count both.
  for (int my = 9; my <= 11; ++my) {
    setCellValue(grid, 15, my, 99);
    setCellValue(grid, 16, my, 100);
    setCellValue(grid, 17, my, 100);
  }

  const auto evidence =
    semantic_nav_nav2_plugins::PersistentNoProgressCondition::
    detectObstacleEvidence(
      path, grid, 1.05, 1.05,
      100, 1.5, 0.10, 0.45, 0.15, 3);

  EXPECT_TRUE(evidence.blocked);
  EXPECT_GE(evidence.true_lethal_cells, 3);
  EXPECT_GE(evidence.inscribed_cells, 1);
}

TEST(PersistentNoProgressConditionTest, registersWithBehaviorTreeFactory)
{
  BT::BehaviorTreeFactory factory;
  EXPECT_NO_THROW(
    factory.registerNodeType<
      semantic_nav_nav2_plugins::PersistentNoProgressCondition>(
      "PersistentNoProgressCondition"));
}
