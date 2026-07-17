// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#define BT_PLUGIN_EXPORT
#include "behaviortree_cpp_v3/bt_factory.h"

#include "semantic_nav_nav2_plugins/escalate_to_llm_recovery.hpp"
#include "semantic_nav_nav2_plugins/validate_semantic.hpp"
#include "semantic_nav_nav2_plugins/path_clear_condition.hpp"
#include "semantic_nav_nav2_plugins/capture_blockage_context.hpp"
#include "semantic_nav_nav2_plugins/query_semantic_context.hpp"
#include "semantic_nav_nav2_plugins/emit_obstacle_signal.hpp"
#include "semantic_nav_nav2_plugins/operator_prompt.hpp"
#include "semantic_nav_nav2_plugins/compute_standoff_pose.hpp"
#include "semantic_nav_nav2_plugins/dynamic_wait.hpp"
#include "semantic_nav_nav2_plugins/has_responsible_object_candidate.hpp"
#include "semantic_nav_nav2_plugins/persistent_no_progress_condition.hpp"
#include "semantic_nav_nav2_plugins/wait_for_dynamic_obstacle_departure.hpp"
#include "semantic_nav_nav2_plugins/wait_for_barrier_clear.hpp"

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<semantic_nav_nav2_plugins::ValidateSemantic>(
    "ValidateSemantic");
  factory.registerNodeType<semantic_nav_nav2_plugins::EscalateToLLMRecovery>(
    "EscalateToLLMRecovery");
  factory.registerNodeType<semantic_nav_nav2_plugins::PathClearCondition>(
    "PathClearCondition");
  factory.registerNodeType<semantic_nav_nav2_plugins::CaptureBlockageContext>(
    "CaptureBlockageContext");
  factory.registerNodeType<semantic_nav_nav2_plugins::QuerySemanticContext>(
    "QuerySemanticContext");
  factory.registerNodeType<semantic_nav_nav2_plugins::EmitObstacleSignal>(
    "EmitObstacleSignal");
  factory.registerNodeType<semantic_nav_nav2_plugins::OperatorPrompt>(
    "OperatorPrompt");
  factory.registerNodeType<semantic_nav_nav2_plugins::ComputeStandoffPose>(
    "ComputeStandoffPose");
  factory.registerNodeType<semantic_nav_nav2_plugins::DynamicWait>(
    "DynamicWait");
  factory.registerNodeType<semantic_nav_nav2_plugins::HasResponsibleObjectCandidate>(
    "HasResponsibleObjectCandidate");
  factory.registerNodeType<
    semantic_nav_nav2_plugins::WaitForDynamicObstacleDeparture>(
    "WaitForDynamicObstacleDeparture");
  factory.registerNodeType<semantic_nav_nav2_plugins::WaitForBarrierClear>(
    "WaitForBarrierClear");
  factory.registerNodeType<semantic_nav_nav2_plugins::PersistentNoProgressCondition>(
    "PersistentNoProgressCondition");
}
