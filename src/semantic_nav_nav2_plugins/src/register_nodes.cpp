// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.

#define BT_PLUGIN_EXPORT

#include "behaviortree_cpp_v3/bt_factory.h"
#include "semantic_nav_nav2_plugins/escalate_to_llm_recovery.hpp"
#include "semantic_nav_nav2_plugins/validate_semantic.hpp"
#include "semantic_nav_nav2_plugins/path_clear_condition.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<semantic_nav_nav2_plugins::ValidateSemantic>(
    "ValidateSemantic");
  factory.registerNodeType<semantic_nav_nav2_plugins::EscalateToLLMRecovery>(
    "EscalateToLLMRecovery");
  factory.registerNodeType<semantic_nav_nav2_plugins::PathClearCondition>(
    "PathClearCondition");
}
