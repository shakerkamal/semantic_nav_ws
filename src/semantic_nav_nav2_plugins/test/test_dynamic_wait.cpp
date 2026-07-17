// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include <gtest/gtest.h>

#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "semantic_nav_nav2_plugins/dynamic_wait.hpp"

namespace
{

BT::BehaviorTreeFactory makeFactory()
{
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<semantic_nav_nav2_plugins::DynamicWait>(
    "DynamicWait");
  return factory;
}

}  // namespace

TEST(DynamicWait, ReadsBlackboardValueOnlyWhenTicked)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();

  const std::string xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <DynamicWait
          wait_duration="{wait_seconds}"
          default_wait_duration="5"
          max_wait_duration="60"
          selected_wait_seconds="{selected_wait_seconds}"/>
      </BehaviorTree>
    </root>)";

  // Tree construction occurs before the directive value exists. This must not
  // read uninitialised memory or throw.
  auto tree = factory.createTreeFromText(xml, blackboard);

  blackboard->set<int>("wait_seconds", 0);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  int selected{-1};
  blackboard->get<int>("selected_wait_seconds", selected);
  EXPECT_EQ(selected, 0);
}

TEST(DynamicWait, UsesFallbackForMissingLateBoundValue)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();

  const std::string xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <DynamicWait
          default_wait_duration="0"
          max_wait_duration="60"
          selected_wait_seconds="{selected_wait_seconds}"/>
      </BehaviorTree>
    </root>)";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  int selected{-1};
  blackboard->get<int>("selected_wait_seconds", selected);
  EXPECT_EQ(selected, 0);
}

TEST(DynamicWait, RejectsNegativeDirectiveAndUsesFallback)
{
  auto factory = makeFactory();
  auto blackboard = BT::Blackboard::create();
  blackboard->set<int>("wait_seconds", -9);

  const std::string xml = R"(
    <root main_tree_to_execute="MainTree">
      <BehaviorTree ID="MainTree">
        <DynamicWait
          wait_duration="{wait_seconds}"
          default_wait_duration="0"
          max_wait_duration="60"
          selected_wait_seconds="{selected_wait_seconds}"/>
      </BehaviorTree>
    </root>)";

  auto tree = factory.createTreeFromText(xml, blackboard);
  EXPECT_EQ(tree.tickRoot(), BT::NodeStatus::SUCCESS);

  int selected{-1};
  blackboard->get<int>("selected_wait_seconds", selected);
  EXPECT_EQ(selected, 0);
}
