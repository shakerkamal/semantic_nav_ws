// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include <gtest/gtest.h>

#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "semantic_nav_interfaces/msg/object_instance.hpp"
#include "semantic_nav_nav2_plugins/wait_for_dynamic_obstacle_departure.hpp"

namespace
{
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

TEST(WaitForDynamicObstacleDepartureTest, usesBboxForAnyDynamicClass)
{
  const auto footprint =
    semantic_nav_nav2_plugins::WaitForDynamicObstacleDeparture::makeFootprint(
      point(1.0, 2.0), extent(0.80, 0.35), 0.30, 0.05);
  EXPECT_TRUE(footprint.valid);
  EXPECT_NEAR(footprint.half_x, 0.45, 1e-9);
  EXPECT_NEAR(footprint.half_y, 0.225, 1e-9);
}

TEST(WaitForDynamicObstacleDepartureTest, missingBboxUsesGenericFallback)
{
  const auto footprint =
    semantic_nav_nav2_plugins::WaitForDynamicObstacleDeparture::makeFootprint(
      point(0.0, 0.0), extent(0.0, 0.0), 0.30, 0.05);
  EXPECT_TRUE(footprint.valid);
  EXPECT_NEAR(footprint.half_x, 0.30, 1e-9);
  EXPECT_NEAR(footprint.half_y, 0.30, 1e-9);
}

TEST(WaitForDynamicObstacleDepartureTest, movedObjectNoLongerBlocks)
{
  using Node = semantic_nav_nav2_plugins::WaitForDynamicObstacleDeparture;
  const auto original = Node::makeFootprint(
    point(0.0, 0.0), extent(0.70, 0.70), 0.30, 0.05);

  semantic_nav_interfaces::msg::ObjectInstance object;
  object.object_key = "dog:1";
  object.bbox_center = point(1.5, 0.0);
  object.bbox_extent = extent(0.80, 0.35);

  EXPECT_FALSE(Node::objectStillBlocks(
    object, original, point(0.0, 0.0), 0.20, 0.02));
}

TEST(WaitForDynamicObstacleDepartureTest, separatedLegLikeDetectionStillBlocks)
{
  using Node = semantic_nav_nav2_plugins::WaitForDynamicObstacleDeparture;
  const auto original = Node::makeFootprint(
    point(-2.507, -1.350), extent(0.70, 0.70), 0.30, 0.05);

  semantic_nav_interfaces::msg::ObjectInstance object;
  object.object_key = "person:902";
  object.bbox_center = point(-2.125, -1.608);
  object.bbox_extent = extent(0.20, 0.20);

  EXPECT_TRUE(Node::objectStillBlocks(
    object, original, point(-2.250, -1.700), 0.20, 0.02));
}

TEST(WaitForDynamicObstacleDepartureTest, onlyDynamicOverlaySourceAllowsTracking)
{
  // Departure tracking is meaningful only for live-perceived objects: a
  // static catalog record trivially "overlaps the blocked region" forever
  // (S3 2026-07-17: 'room partition:121' burned the full 30s timeout).
  // Routing is by source PROVENANCE, never by state/safety/tag heuristics.
  semantic_nav_interfaces::msg::ObjectInstance object;

  object.source = "dynamic_overlay";
  EXPECT_TRUE(
    semantic_nav_nav2_plugins::WaitForDynamicObstacleDeparture::
    sourceAllowsDepartureTracking(object));

  object.source = "persistent_map";
  EXPECT_FALSE(
    semantic_nav_nav2_plugins::WaitForDynamicObstacleDeparture::
    sourceAllowsDepartureTracking(object));

  object.source = "";
  EXPECT_FALSE(
    semantic_nav_nav2_plugins::WaitForDynamicObstacleDeparture::
    sourceAllowsDepartureTracking(object));
}
