// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include <gtest/gtest.h>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "semantic_nav_nav2_plugins/escalate_to_llm_recovery.hpp"
#include "semantic_nav_nav2_plugins/validate_semantic.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "semantic_nav_nav2_plugins/path_clear_condition.hpp"
#include "semantic_nav_nav2_plugins/capture_blockage_context.hpp"
#include "semantic_nav_nav2_plugins/query_semantic_context.hpp"
#include "semantic_nav_nav2_plugins/emit_obstacle_signal.hpp"
#include "semantic_nav_nav2_plugins/operator_prompt.hpp"

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

TEST(EscalateToLLMRecoveryTest, directiveWaitSecondsPortTypeIsInt)
{
  const auto ports = semantic_nav_nav2_plugins::EscalateToLLMRecovery::providedPorts();

  ASSERT_GT(ports.count("directive_wait_seconds"), 0u);
  EXPECT_EQ(
    ports.at("directive_wait_seconds").direction(),
    BT::PortDirection::OUTPUT);

  ASSERT_NE(ports.at("directive_wait_seconds").type(), nullptr);
  EXPECT_EQ(*ports.at("directive_wait_seconds").type(), typeid(int));
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
    
  EXPECT_NO_THROW(
    factory.registerNodeType<semantic_nav_nav2_plugins::EmitObstacleSignal>(
      "EmitObstacleSignal"));

  EXPECT_NO_THROW(
    factory.registerNodeType<semantic_nav_nav2_plugins::OperatorPrompt>(
      "OperatorPrompt"));
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

  auto metrics =
    semantic_nav_nav2_plugins::PathClearCondition::isCorridorBlocked(
      path, costmap, 90, 1.5, 0.05);

  EXPECT_FALSE(metrics.any_blocked);
}

TEST(PathClearConditionTest, lethalCellInsideSampleRadiusReturnsTrue)
{
  auto costmap = makeCostmap(20, 20, 0.1f, 0.0f, 0.0f, 0);

  // Cell center is approximately (0.55, 0.55).
  costmap.data[5 * 20 + 5] = 100;

  auto path = makeStraightPath(0.05, 0.55, 0.1, 15);

  auto metrics =
    semantic_nav_nav2_plugins::PathClearCondition::isCorridorBlocked(
      path, costmap, 90, 1.5, 0.08);

  EXPECT_TRUE(metrics.any_blocked);
  EXPECT_NEAR(metrics.centroid.x, 0.55, 0.11);
  EXPECT_NEAR(metrics.centroid.y, 0.55, 0.11);
  EXPECT_GT(metrics.extent_m, 0.0f);
}

TEST(PathClearConditionTest, lethalCellOutsideLookaheadReturnsFalse)
{
  auto costmap = makeCostmap(50, 20, 0.1f, 0.0f, 0.0f, 0);

  // Cell center around x=3.05m, beyond 1.5m lookahead.
  costmap.data[5 * 50 + 30] = 100;

  auto path = makeStraightPath(0.05, 0.55, 0.1, 40);

  auto metrics =
    semantic_nav_nav2_plugins::PathClearCondition::isCorridorBlocked(
      path, costmap, 90, 1.5, 0.08);

  EXPECT_FALSE(metrics.any_blocked);
}

TEST(PathClearConditionTest, negativeOriginUsesFloorIndexing)
{
  auto costmap = makeCostmap(20, 20, 0.1f, -1.0f, -1.0f, 0);

  // World point near (-0.45, -0.45) maps correctly only with floor-style indexing.
  // Grid cell (5,5) center is (-0.45, -0.45).
  costmap.data[5 * 20 + 5] = 100;

  auto path = makeStraightPath(-0.45, -0.45, 0.1, 5);

  auto metrics =
    semantic_nav_nav2_plugins::PathClearCondition::isCorridorBlocked(
      path, costmap, 90, 1.0, 0.08);

  EXPECT_TRUE(metrics.any_blocked);
}

TEST(PathClearConditionTest, malformedCostmapReturnsFalse)
{
  auto costmap = makeCostmap(20, 20, 0.1f, 0.0f, 0.0f, 0);
  costmap.data.resize(3);

  auto path = makeStraightPath(0.05, 0.55, 0.1, 10);

  auto metrics =
    semantic_nav_nav2_plugins::PathClearCondition::isCorridorBlocked(
      path, costmap, 90, 1.0, 0.08);

  EXPECT_FALSE(metrics.any_blocked);
}

TEST(PathClearConditionTest, sampleRadiusZeroChecksContainingCell)
{
  // Costmap 10x10, resolution=0.1m, origin=(0,0). Cell (0,5) is lethal.
  auto costmap = makeCostmap(10, 10, 0.1f, 0.0f, 0.0f, 0);
  costmap.data[5 * 10 + 0] = 100;

  // Plan pose at (0.07, 0.55): inside cell (0,5) but NOT at the cell center (0.05, 0.55).
  // Old code: distance (0.07-0.05)^2+(0.55-0.55)^2 = 0.02 > 0.0 → skipped → no detection.
  // New code: sample_radius_m==0 skips the distance filter → cell checked → detected.
  auto path = makeStraightPath(0.07, 0.55, 0.1, 5);

  auto metrics =
    semantic_nav_nav2_plugins::PathClearCondition::isCorridorBlocked(
      path, costmap, 90, 1.5, 0.0);

  EXPECT_TRUE(metrics.any_blocked);
}

TEST(PathClearConditionTest, singleLethalPoseHasMinorMetrics)
{
  // Only one path pose is blocked — severity metrics should be small.
  auto costmap = makeCostmap(20, 20, 0.1f, 0.0f, 0.0f, 0);
  costmap.data[5 * 20 + 5] = 100;  // cell at ~(0.55, 0.55)

  // 15 poses along y=0.55; only the pose near x=0.55 hits the blocked cell.
  auto path = makeStraightPath(0.05, 0.55, 0.1, 15);

  auto metrics =
    semantic_nav_nav2_plugins::PathClearCondition::isCorridorBlocked(
      path, costmap, 90, 1.5, 0.08);

  EXPECT_TRUE(metrics.any_blocked);
  EXPECT_EQ(metrics.blocked_poses, 1);
  EXPECT_LT(metrics.blocked_fraction, 0.1);
  EXPECT_LT(metrics.max_run_length_m, 0.2);
}

TEST(PathClearConditionTest, fullRowBlockedHasLargeMetrics)
{
  // Entire row at y≈0.55 is lethal — severity metrics should be large.
  auto costmap = makeCostmap(20, 20, 0.1f, 0.0f, 0.0f, 0);
  for (int mx = 0; mx < 20; ++mx) {
    costmap.data[5 * 20 + mx] = 100;
  }

  // 15 poses along y=0.55, step=0.1m → all within lookahead are blocked.
  auto path = makeStraightPath(0.05, 0.55, 0.1, 15);

  auto metrics =
    semantic_nav_nav2_plugins::PathClearCondition::isCorridorBlocked(
      path, costmap, 90, 1.5, 0.05);

  EXPECT_TRUE(metrics.any_blocked);
  EXPECT_EQ(metrics.blocked_poses, metrics.total_poses);
  EXPECT_NEAR(metrics.blocked_fraction, 1.0, 0.01);
  EXPECT_GT(metrics.max_run_length_m, 1.0);  // 15 poses × 0.1m step ≈ 1.4m run
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

// ---- EmitObstacleSignal -------------------------------------------------

TEST(EmitObstacleSignalTest, hasEmitEnabledInputPort)
{
  const auto ports = semantic_nav_nav2_plugins::EmitObstacleSignal::providedPorts();
  ASSERT_GT(ports.count("emit_enabled"), 0u);
  EXPECT_EQ(ports.at("emit_enabled").direction(), BT::PortDirection::INPUT);
}

TEST(EmitObstacleSignalTest, hasPublishSignalInputPort)
{
  const auto ports = semantic_nav_nav2_plugins::EmitObstacleSignal::providedPorts();
  ASSERT_GT(ports.count("publish_signal"), 0u);
  EXPECT_EQ(ports.at("publish_signal").direction(), BT::PortDirection::INPUT);
}

TEST(EmitObstacleSignalTest, hasSignalClassInputPort)
{
  const auto ports = semantic_nav_nav2_plugins::EmitObstacleSignal::providedPorts();
  ASSERT_GT(ports.count("signal_class"), 0u);
  EXPECT_EQ(ports.at("signal_class").direction(), BT::PortDirection::INPUT);
}

TEST(EmitObstacleSignalTest, hasSignalTopicInputPort)
{
  const auto ports = semantic_nav_nav2_plugins::EmitObstacleSignal::providedPorts();
  ASSERT_GT(ports.count("signal_topic"), 0u);
  EXPECT_EQ(ports.at("signal_topic").direction(), BT::PortDirection::INPUT);
}

TEST(EmitObstacleSignalTest, registersWithoutError)
{
  BT::BehaviorTreeFactory factory;
  EXPECT_NO_THROW(
    factory.registerNodeType<semantic_nav_nav2_plugins::EmitObstacleSignal>(
      "EmitObstacleSignal"));
}

// ---- OperatorPrompt -------------------------------------------------------

TEST(OperatorPromptTest, hasServiceNameInputPort)
{
  const auto ports = semantic_nav_nav2_plugins::OperatorPrompt::providedPorts();
  ASSERT_GT(ports.count("service_name"), 0u);
  EXPECT_EQ(ports.at("service_name").direction(), BT::PortDirection::INPUT);
}

TEST(OperatorPromptTest, hasPromptTextInputPort)
{
  const auto ports = semantic_nav_nav2_plugins::OperatorPrompt::providedPorts();
  ASSERT_GT(ports.count("prompt_text"), 0u);
  EXPECT_EQ(ports.at("prompt_text").direction(), BT::PortDirection::INPUT);
}

TEST(OperatorPromptTest, hasResponsibleObjectKeyInputPort)
{
  const auto ports = semantic_nav_nav2_plugins::OperatorPrompt::providedPorts();
  ASSERT_GT(ports.count("responsible_object_key"), 0u);
  EXPECT_EQ(ports.at("responsible_object_key").direction(), BT::PortDirection::INPUT);
}

TEST(OperatorPromptTest, hasFailureStageInputPort)
{
  const auto ports = semantic_nav_nav2_plugins::OperatorPrompt::providedPorts();
  ASSERT_GT(ports.count("failure_stage"), 0u);
  EXPECT_EQ(ports.at("failure_stage").direction(), BT::PortDirection::INPUT);
}

TEST(OperatorPromptTest, hasDirectiveActionInputPort)
{
  const auto ports = semantic_nav_nav2_plugins::OperatorPrompt::providedPorts();
  ASSERT_GT(ports.count("directive_action"), 0u);
  EXPECT_EQ(ports.at("directive_action").direction(), BT::PortDirection::INPUT);
}

TEST(OperatorPromptTest, hasRecoveryEventIdInputPort)
{
  const auto ports = semantic_nav_nav2_plugins::OperatorPrompt::providedPorts();
  ASSERT_GT(ports.count("recovery_event_id"), 0u);
  EXPECT_EQ(ports.at("recovery_event_id").direction(), BT::PortDirection::INPUT);
}

TEST(OperatorPromptTest, hasConfirmedObjectTopicInputPort)
{
  // 2026-07-15: OperatorDecision.srv has no way to signal a simulation
  // -specific follow-up (e.g. deleting a spawned Gazebo obstacle) -- the
  // door stayed put even after an operator confirmed opening it, since
  // nothing was watching for the confirmation. This port is the seam
  // eval-only tooling subscribes to instead of coupling the operator
  // interface itself to Gazebo.
  const auto ports = semantic_nav_nav2_plugins::OperatorPrompt::providedPorts();
  ASSERT_GT(ports.count("confirmed_object_topic"), 0u);
  EXPECT_EQ(ports.at("confirmed_object_topic").direction(), BT::PortDirection::INPUT);
}

TEST(OperatorPromptTest, registersWithoutError)
{
  BT::BehaviorTreeFactory factory;
  EXPECT_NO_THROW(
    factory.registerNodeType<semantic_nav_nav2_plugins::OperatorPrompt>(
      "OperatorPrompt"));
}

TEST(CaptureBlockageContextTest, fallbackProjectsAheadOfRobotAlongPath)
{
  // Path runs west (decreasing x) toward the gap; robot sits at x=-1.3 having
  // stopped ~1.2 m short of a blocker near x=-2.5. The fallback must land ~1 m
  // further along the path (toward the blocker), NOT at the origin.
  auto path = makeStraightPath(0.0, -0.5, -0.1, 40);   // x: 0.0 -> -3.9
  auto c = semantic_nav_nav2_plugins::CaptureBlockageContext::
    fallbackCentroidAlongPath(path, -1.3, -0.5, 1.0);
  EXPECT_NEAR(c.x, -2.3, 0.15);   // ~1 m past the robot along the path
  EXPECT_NEAR(c.y, -0.5, 0.05);
}

TEST(CaptureBlockageContextTest, fallbackEmptyPathReturnsRobotPose)
{
  nav_msgs::msg::Path empty;
  auto c = semantic_nav_nav2_plugins::CaptureBlockageContext::
    fallbackCentroidAlongPath(empty, 2.0, -0.7, 1.0);
  EXPECT_NEAR(c.x, 2.0, 1e-6);
  EXPECT_NEAR(c.y, -0.7, 1e-6);
}

TEST(CaptureBlockageContextTest, fallbackClampsToPathEndWhenShort)
{
  // Lookahead exceeds the remaining path -> returns the last pose, not (0,0).
  auto path = makeStraightPath(0.0, 0.0, -0.1, 5);     // x: 0.0 -> -0.4
  auto c = semantic_nav_nav2_plugins::CaptureBlockageContext::
    fallbackCentroidAlongPath(path, 0.0, 0.0, 5.0);
  EXPECT_NEAR(c.x, -0.4, 1e-6);
  EXPECT_NEAR(c.y, 0.0, 1e-6);
}

TEST(CaptureBlockageContextTest, nearestLethalCentroidFindsClusterNearRobot)
{
  // A fully-sealed corridor (S2 door): ComputePathToPose finds NO path at
  // all, so fallbackCentroidAlongPath has nothing to project along and would
  // otherwise return the robot's raw position -- which can be a couple of
  // metres from the actual blocker after a Tier-2 backup (found 2026-07-15,
  // S2: robot at (2.808,-0.116), true door at (4.862,-0.677), match found an
  // unrelated "trash bin" instead). The costmap is the PERCEPTION-GROUNDED
  // source of truth: it already shows the door as lethal cells right in
  // front of the stopped robot, so search it directly instead of guessing.
  auto costmap = makeCostmap(40, 40, 0.1f, 0.0f, 0.0f, 0);
  // Lethal cluster centred around world (1.55, 0.05) -- ~1.5m ahead of the
  // robot at (0,0), row my=0 so its world y (0.05) sits right at the robot.
  for (int mx = 14; mx <= 16; ++mx) {
    costmap.data[0 * 40 + mx] = 100;
  }
  geometry_msgs::msg::Point out;
  bool found = semantic_nav_nav2_plugins::CaptureBlockageContext::
    nearestLethalCentroidNearRobot(costmap, 0.0, 0.0, 2.0, 90, out);
  EXPECT_TRUE(found);
  EXPECT_NEAR(out.x, 1.55, 0.1);
  EXPECT_NEAR(out.y, 0.05, 0.1);
}

TEST(CaptureBlockageContextTest, nearestLethalCentroidReturnsFalseWhenNoneNearby)
{
  auto costmap = makeCostmap(40, 40, 0.1f, 0.0f, 0.0f, 0);
  geometry_msgs::msg::Point out;
  bool found = semantic_nav_nav2_plugins::CaptureBlockageContext::
    nearestLethalCentroidNearRobot(costmap, 0.0, 0.0, 2.0, 90, out);
  EXPECT_FALSE(found);
}

TEST(CaptureBlockageContextTest, nearestLethalCentroidIgnoresCellsBeyondSearchRadius)
{
  auto costmap = makeCostmap(40, 40, 0.1f, 0.0f, 0.0f, 0);
  // Lethal cell at world (3.95, 0.05) -- outside a 2.0m search radius from (0,0).
  costmap.data[0 * 40 + 39] = 100;
  geometry_msgs::msg::Point out;
  bool found = semantic_nav_nav2_plugins::CaptureBlockageContext::
    nearestLethalCentroidNearRobot(costmap, 0.0, 0.0, 2.0, 90, out);
  EXPECT_FALSE(found);
}

TEST(CaptureBlockageContextTest, nearestLethalCentroidMalformedCostmapReturnsFalse)
{
  nav_msgs::msg::OccupancyGrid costmap;   // width/height 0
  geometry_msgs::msg::Point out;
  bool found = semantic_nav_nav2_plugins::CaptureBlockageContext::
    nearestLethalCentroidNearRobot(costmap, 0.0, 0.0, 2.0, 90, out);
  EXPECT_FALSE(found);
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
