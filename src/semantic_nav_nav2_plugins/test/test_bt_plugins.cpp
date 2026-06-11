// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include <gtest/gtest.h>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "semantic_nav_nav2_plugins/escalate_to_llm_recovery.hpp"
#include "semantic_nav_nav2_plugins/validate_semantic.hpp"

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

TEST(PluginRegistrationTest, bothNodesRegisterWithoutError)
{
  BT::BehaviorTreeFactory factory;
  EXPECT_NO_THROW(
    factory.registerNodeType<semantic_nav_nav2_plugins::ValidateSemantic>(
      "ValidateSemantic"));
  EXPECT_NO_THROW(
    factory.registerNodeType<semantic_nav_nav2_plugins::EscalateToLLMRecovery>(
      "EscalateToLLMRecovery"));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
