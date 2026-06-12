// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include <gtest/gtest.h>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "semantic_nav_nav2_plugins/escalate_to_llm_recovery.hpp"
#include "semantic_nav_nav2_plugins/validate_semantic.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "semantic_nav_nav2_plugins/path_clear_condition.hpp"
#include "semantic_nav_nav2_plugins/query_semantic_context.hpp"

TEST(ValidateSemanticTest, hasGoalPoseInputPort)
{
  auto ports = semantic_nav_nav2_plugins::ValidateSemantic::providedPorts();
  ASSERT_GT(ports.count("goal_pose"), 0u);
  EXPECT_EQ(ports.at("goal_pose").direction(), BT::PortDirection::INPUT);
}

TEST(ValidateSemanticTest, hasValidationReasonOutputPort)
{
  auto ports = semantic_nav_nav2_plugins::ValidateSemantic::providedPorts();
  ASSERT_GT(ports.count("validation_reason"), 0u);
  EXPECT_EQ(ports.at("validation_reason").direction(), BT::PortDirection::OUTPUT);
}

TEST(ValidateSemanticTest, inheritsServiceNamePortFromBase)
{
  auto ports = semantic_nav_nav2_plugins::ValidateSemantic::providedPorts();
  EXPECT_GT(ports.count("service_name"), 0u);
}

TEST(EscalateToLLMRecoveryTest, hasServiceNamePort)
{
  auto ports = semantic_nav_nav2_plugins::EscalateToLLMRecovery::providedPorts();
  EXPECT_GT(ports.count("service_name"), 0u);
}

TEST(EscalateToLLMRecoveryTest, hasFailureStageInputPort)
{
  auto ports = semantic_nav_nav2_plugins::EscalateToLLMRecovery::providedPorts();
  ASSERT_GT(ports.count("failure_stage"), 0u);
  EXPECT_EQ(ports.at("failure_stage").direction(), BT::PortDirection::INPUT);
}

TEST(EscalateToLLMRecoveryTest, hasDirectiveActionOutputPort)
{
  auto ports = semantic_nav_nav2_plugins::EscalateToLLMRecovery::providedPorts();
  ASSERT_GT(ports.count("directive_action"), 0u);
  EXPECT_EQ(ports.at("directive_action").direction(), BT::PortDirection::OUTPUT);
}

TEST(EscalateToLLMRecoveryTest, hasDirectiveTargetPoseOutputPort)
{
  auto ports = semantic_nav_nav2_plugins::EscalateToLLMRecovery::providedPorts();
  EXPECT_GT(ports.count("directive_target_pose"), 0u);
}

TEST(EscalateToLLMRecoveryTest, hasDirectiveWaitSecondsOutputPort)
{
  auto ports = semantic_nav_nav2_plugins::EscalateToLLMRecovery::providedPorts();
  EXPECT_GT(ports.count("directive_wait_seconds"), 0u);
}

TEST(EscalateToLLMRecoveryTest, hasRecoveryEventIdOutputPort)
{
  auto ports = semantic_nav_nav2_plugins::EscalateToLLMRecovery::providedPorts();
  EXPECT_GT(ports.count("recovery_event_id"), 0u);
}

TEST(EscalateToLLMRecoveryTest, hasLocalDbVersionInputPort)
{
  auto ports = semantic_nav_nav2_plugins::EscalateToLLMRecovery::providedPorts();
  ASSERT_GT(ports.count("local_db_version"), 0u);
  EXPECT_EQ(ports.at("local_db_version").direction(), BT::PortDirection::INPUT);
}

TEST(PluginRegistrationTest, currentNodesRegisterWithoutError)
{
  BT::BehaviorTreeFactory factory;

  EXPECT_NO_THROW(
    factory.registerNodeType<semantic_nav_nav2_plugins::ValidateSemantic>(
      "ValidateSemantic"));

  EXPECT_NO_THROW(
    factory.registerNodeType<semantic_nav_nav2_plugins::EscalateToLLMRecovery>(
      "EscalateToLLMRecovery"));

  EXPECT_NO_THROW(
    factory.registerNodeType<semantic_nav_nav2_plugins::PathClearCondition>(
      "PathClearCondition"));

  EXPECT_NO_THROW(
    factory.registerNodeType<semantic_nav_nav2_plugins::QuerySemanticContext>(
      "QuerySemanticContext"));
}

// ---- PathClearCondition -------------------------------------------------

static nav_msgs::msg::OccupancyGrid makeCostmap(
  int width,
  int height,
  float resolution,
  float origin_x,
  float origin_y,
  int8_t fill_cost)
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.info.width = width;
  grid.info.height = height;
  grid.info.resolution = resolution;
  grid.info.origin.position.x = origin_x;
  grid.info.origin.position.y = origin_y;
  grid.data.assign(static_cast<size_t>(width * height), fill_cost);
  return grid;
}

static nav_msgs::msg::Path makeStraightPath(
  double start_x,
  double start_y,
  double step,
  int count)
{
  nav_msgs::msg::Path path;
  for (int i = 0; i < count; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = start_x + static_cast<double>(i) * step;
    pose.pose.position.y = start_y;
    path.poses.push_back(pose);
  }
  return path;
}

TEST(PathClearConditionTest, hasPathInputPort)
{
  const auto ports = semantic_nav_nav2_plugins::PathClearCondition::providedPorts();
  ASSERT_GT(ports.count("path"), 0u);
  EXPECT_EQ(ports.at("path").direction(), BT::PortDirection::INPUT);
}

TEST(PathClearConditionTest, hasSampleRadiusInputPort)
{
  const auto ports = semantic_nav_nav2_plugins::PathClearCondition::providedPorts();
  ASSERT_GT(ports.count("sample_radius_m"), 0u);
  EXPECT_EQ(ports.at("sample_radius_m").direction(), BT::PortDirection::INPUT);
}

TEST(PathClearConditionTest, hasBlockageCentroidOutputPort)
{
  const auto ports = semantic_nav_nav2_plugins::PathClearCondition::providedPorts();
  ASSERT_GT(ports.count("blockage_centroid"), 0u);
  EXPECT_EQ(ports.at("blockage_centroid").direction(), BT::PortDirection::OUTPUT);
}

TEST(PathClearConditionTest, clearCostmapReturnsFalse)
{
  auto costmap = makeCostmap(20, 20, 0.1f, 0.0f, 0.0f, 0);
  auto path = makeStraightPath(0.05, 0.55, 0.1, 15);

  geometry_msgs::msg::Point centroid;
  float extent = 0.0f;

  const bool blocked =
    semantic_nav_nav2_plugins::PathClearCondition::isCorridorBlocked(
      path, costmap, 90, 1.5, 0.05, centroid, extent);

  EXPECT_FALSE(blocked);
}

TEST(PathClearConditionTest, lethalCellInsideSampleRadiusReturnsTrue)
{
  auto costmap = makeCostmap(20, 20, 0.1f, 0.0f, 0.0f, 0);

  // Cell center is approximately (0.55, 0.55).
  costmap.data[5 * 20 + 5] = 100;

  auto path = makeStraightPath(0.05, 0.55, 0.1, 15);

  geometry_msgs::msg::Point centroid;
  float extent = 0.0f;

  const bool blocked =
    semantic_nav_nav2_plugins::PathClearCondition::isCorridorBlocked(
      path, costmap, 90, 1.5, 0.08, centroid, extent);

  EXPECT_TRUE(blocked);
  EXPECT_NEAR(centroid.x, 0.55, 0.11);
  EXPECT_NEAR(centroid.y, 0.55, 0.11);
  EXPECT_GT(extent, 0.0f);
}

TEST(PathClearConditionTest, lethalCellOutsideLookaheadReturnsFalse)
{
  auto costmap = makeCostmap(50, 20, 0.1f, 0.0f, 0.0f, 0);

  // Cell center around x=3.05m, beyond 1.5m lookahead.
  costmap.data[5 * 50 + 30] = 100;

  auto path = makeStraightPath(0.05, 0.55, 0.1, 40);

  geometry_msgs::msg::Point centroid;
  float extent = 0.0f;

  const bool blocked =
    semantic_nav_nav2_plugins::PathClearCondition::isCorridorBlocked(
      path, costmap, 90, 1.5, 0.08, centroid, extent);

  EXPECT_FALSE(blocked);
}

TEST(PathClearConditionTest, negativeOriginUsesFloorIndexing)
{
  auto costmap = makeCostmap(20, 20, 0.1f, -1.0f, -1.0f, 0);

  // World point near (-0.45, -0.45) maps correctly only with floor-style indexing.
  // Grid cell (5,5) center is (-0.45, -0.45).
  costmap.data[5 * 20 + 5] = 100;

  auto path = makeStraightPath(-0.45, -0.45, 0.1, 5);

  geometry_msgs::msg::Point centroid;
  float extent = 0.0f;

  const bool blocked =
    semantic_nav_nav2_plugins::PathClearCondition::isCorridorBlocked(
      path, costmap, 90, 1.0, 0.08, centroid, extent);

  EXPECT_TRUE(blocked);
}

TEST(PathClearConditionTest, malformedCostmapReturnsFalse)
{
  auto costmap = makeCostmap(20, 20, 0.1f, 0.0f, 0.0f, 0);
  costmap.data.resize(3);

  auto path = makeStraightPath(0.05, 0.55, 0.1, 10);

  geometry_msgs::msg::Point centroid;
  float extent = 0.0f;

  const bool blocked =
    semantic_nav_nav2_plugins::PathClearCondition::isCorridorBlocked(
      path, costmap, 90, 1.0, 0.08, centroid, extent);

  EXPECT_FALSE(blocked);
}

// ---- QuerySemanticContext -----------------------------------------------

TEST(QuerySemanticContextTest, hasBlockageCentroidInputPort)
{
  const auto ports = semantic_nav_nav2_plugins::QuerySemanticContext::providedPorts();
  ASSERT_GT(ports.count("blockage_centroid"), 0u);
  EXPECT_EQ(ports.at("blockage_centroid").direction(), BT::PortDirection::INPUT);
}

TEST(QuerySemanticContextTest, hasServiceReadyTimeoutInputPort)
{
  const auto ports = semantic_nav_nav2_plugins::QuerySemanticContext::providedPorts();
  ASSERT_GT(ports.count("service_ready_timeout_ms"), 0u);
  EXPECT_EQ(
    ports.at("service_ready_timeout_ms").direction(),
    BT::PortDirection::INPUT);
}

TEST(QuerySemanticContextTest, hasResponseTimeoutInputPort)
{
  const auto ports = semantic_nav_nav2_plugins::QuerySemanticContext::providedPorts();
  ASSERT_GT(ports.count("response_timeout_ms"), 0u);
  EXPECT_EQ(
    ports.at("response_timeout_ms").direction(),
    BT::PortDirection::INPUT);
}

TEST(QuerySemanticContextTest, hasResponsibleObjectKeyOutputPort)
{
  const auto ports = semantic_nav_nav2_plugins::QuerySemanticContext::providedPorts();
  ASSERT_GT(ports.count("responsible_object_key"), 0u);
  EXPECT_EQ(
    ports.at("responsible_object_key").direction(),
    BT::PortDirection::OUTPUT);
}

TEST(QuerySemanticContextTest, hasResponsibleSafetyClassOutputPort)
{
  const auto ports = semantic_nav_nav2_plugins::QuerySemanticContext::providedPorts();
  ASSERT_GT(ports.count("responsible_safety_class"), 0u);
  EXPECT_EQ(
    ports.at("responsible_safety_class").direction(),
    BT::PortDirection::OUTPUT);
}

TEST(QuerySemanticContextTest, hasLocalDbVersionOutputPort)
{
  const auto ports = semantic_nav_nav2_plugins::QuerySemanticContext::providedPorts();
  ASSERT_GT(ports.count("local_db_version"), 0u);
  EXPECT_EQ(
    ports.at("local_db_version").direction(),
    BT::PortDirection::OUTPUT);
}

TEST(QuerySemanticContextTest, hasLocalDbSourceOutputPort)
{
  const auto ports = semantic_nav_nav2_plugins::QuerySemanticContext::providedPorts();
  ASSERT_GT(ports.count("local_db_source"), 0u);
  EXPECT_EQ(
    ports.at("local_db_source").direction(),
    BT::PortDirection::OUTPUT);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
