// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include "semantic_nav_nav2_plugins/escalate_to_llm_recovery.hpp"

#include <algorithm>
#include <string>

namespace semantic_nav_nav2_plugins
{

EscalateToLLMRecovery::EscalateToLLMRecovery(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: Base(service_node_name, conf)
{}

BT::NodeStatus EscalateToLLMRecovery::tick()
{
  // If a service request is already in flight, let BtServiceNode complete it.
  if (status() == BT::NodeStatus::RUNNING) {
    return Base::tick();
  }

  if (!readRobotPose(pending_robot_pose_)) {
    setOutput("directive_action", std::string("give_up"));
    setOutput("directive_operator_message", std::string("missing_robot_pose"));
    return BT::NodeStatus::FAILURE;
  }

  return Base::tick();
}

void EscalateToLLMRecovery::on_tick()
{
  request_->header.stamp = node_->now();
  request_->header.frame_id = pending_robot_pose_.header.frame_id;
  request_->trigger_source = "bt_recovery_plugin";

  std::string failure_stage;
  getInput("failure_stage", failure_stage);
  request_->failure_stage = failure_stage.empty() ? "execution" : failure_stage;

  std::string nav2_message;
  getInput("nav2_message", nav2_message);
  request_->nav2_message = nav2_message;

  request_->robot_pose = pending_robot_pose_;

  std::string responsible_object_key;
  std::string responsible_object_tag;
  std::string responsible_object_state;
  std::string responsible_safety_class;
  bool responsible_openable{false};
  bool responsible_clearable{false};
  std::string responsible_match_type;
  std::string responsible_state_detail;
  std::string responsible_traversability;

  getInput("responsible_object_key", responsible_object_key);
  getInput("responsible_object_tag", responsible_object_tag);
  getInput("responsible_object_state", responsible_object_state);
  getInput("responsible_safety_class", responsible_safety_class);
  getInput("responsible_openable", responsible_openable);
  getInput("responsible_clearable", responsible_clearable);
  getInput("responsible_match_type", responsible_match_type);
  getInput("responsible_state_detail", responsible_state_detail);
  getInput("responsible_traversability", responsible_traversability);

  request_->responsible_object_key = responsible_object_key;
  request_->responsible_object_tag = responsible_object_tag;
  request_->responsible_object_state = responsible_object_state;
  request_->responsible_safety_class =
    responsible_safety_class.empty() ? "none" : responsible_safety_class;
  request_->responsible_openable = responsible_openable;
  request_->responsible_clearable = responsible_clearable;
  request_->responsible_match_type =
    responsible_match_type.empty() ? "none" : responsible_match_type;
  request_->responsible_state_detail = responsible_state_detail;
  request_->responsible_traversability = responsible_traversability;

  geometry_msgs::msg::Point blockage_centroid;
  getInput("blockage_centroid", blockage_centroid);
  request_->blockage_centroid = blockage_centroid;

  float blockage_extent_m{0.0f};
  getInput("blockage_extent_m", blockage_extent_m);
  request_->blockage_extent_m = blockage_extent_m;

  // Diagnostic BT-side attempt counter only. The orchestrator ledger remains
  // authoritative and is returned as attempts_used/retry_cap in the response.
  int semantic_attempt_index{0};
  if (config().blackboard) {
    config().blackboard->get<int>(
      "semantic_recovery_attempt_index", semantic_attempt_index);
    semantic_attempt_index = std::max(0, semantic_attempt_index);
    config().blackboard->set<int>(
      "semantic_recovery_attempt_index", semantic_attempt_index + 1);
  }
  request_->bt_attempt_index = static_cast<uint16_t>(semantic_attempt_index);

  std::string original_object_tag;
  std::string original_intent_hint;
  std::string current_target_object_key;
  getInput("original_object_tag", original_object_tag);
  getInput("original_intent_hint", original_intent_hint);
  getInput("current_target_object_key", current_target_object_key);
  request_->original_object_tag = original_object_tag;
  request_->original_intent_hint = original_intent_hint;
  request_->current_target_object_key = current_target_object_key;

  int local_db_version{0};
  getInput("local_db_version", local_db_version);
  request_->local_db_version = static_cast<uint32_t>(std::max(0, local_db_version));
  request_->local_db_stamp = node_->now();

  std::string local_db_source;
  getInput("local_db_source", local_db_source);
  request_->local_db_source = local_db_source.empty() ? "static_snapshot" : local_db_source;

  // M3 does not yet generate a stable debounce key. Keep explicit but empty.
  request_->debounce_key = "";
}

BT::NodeStatus EscalateToLLMRecovery::on_completion(
  std::shared_ptr<ServiceT::Response> response)
{
  if (!response) {
    setOutput("directive_action", std::string("give_up"));
    setOutput("directive_operator_message", std::string("no_response"));
    return BT::NodeStatus::FAILURE;
  }

  const std::string & action = response->action;
  const std::string & status_text = response->status;

  setOutput("directive_action", action);
  setOutput("directive_target_object_key", response->target_object_key);
  setOutput("directive_target_object_tag", response->target_object_tag);
  setOutput("directive_target_intent_hint", response->target_intent_hint);
  const int wait_seconds = std::clamp(
    static_cast<int>(response->wait_seconds),
    0,
    60);

  const int signal_attempts = std::clamp(
    static_cast<int>(response->signal_attempts),
    1,
    5);

  setOutput("directive_wait_seconds", wait_seconds);
  setOutput("directive_emit_signal_during_wait", response->emit_signal_during_wait);
  setOutput("directive_signal_attempts", signal_attempts);
  setOutput("directive_operator_message", response->operator_message);
  setOutput("directive_rationale", response->rationale);
  setOutput("directive_confidence_percent", static_cast<int>(response->confidence_percent));
  setOutput("directive_escalate_to_operator", response->escalate_to_operator);
  setOutput("recovery_event_id", response->recovery_event_id);

  if (status_text == "terminal_fail" || status_text == "rejected" ||
    status_text == "duplicate" || action.empty())
  {
    return BT::NodeStatus::FAILURE;
  }

  if (action == "retry_target") {
    if (config().blackboard) {
      config().blackboard->set<geometry_msgs::msg::PoseStamped>(
        "goal", response->target_pose);
      // Optional compatibility key for design-doc wording; Nav2 XML below uses "goal".
      config().blackboard->set<geometry_msgs::msg::PoseStamped>(
        "goal_pose", response->target_pose);
      config().blackboard->set<std::string>(
        "current_target_object_key", response->target_object_key);
    }
    setOutput("directive_target_pose", response->target_pose);
    return BT::NodeStatus::SUCCESS;
  }

  if (action == "wait_then_replan" || action == "give_up" ||
      action == "open_door_then_replan" || action == "clear_object_then_replan")
  {
    return BT::NodeStatus::SUCCESS;
  }

  setOutput("directive_operator_message", std::string("unknown_recovery_action:" + action));
  return BT::NodeStatus::FAILURE;
}

BT::PortsList EscalateToLLMRecovery::providedPorts()
{
  return providedBasicPorts({
    // ---- Inputs ----
    BT::InputPort<std::string>(
      "failure_stage", "execution", "execution | validation | bt_recovery"),
    BT::InputPort<std::string>(
      "nav2_message", "", "Nav2 error or validation reason"),

    BT::InputPort<std::string>(
      "global_frame", "map", "Frame the robot pose is reported in"),
    BT::InputPort<std::string>(
      "robot_base_frame", "base_link", "Robot base frame; the rover uses base_footprint"),
    BT::InputPort<double>(
      "transform_tolerance_s", 0.1, "TF lookup tolerance when reading the current robot pose"),

    BT::InputPort<std::string>("responsible_object_key", "", ""),
    BT::InputPort<std::string>("responsible_object_tag", "", ""),
    BT::InputPort<std::string>("responsible_object_state", "", ""),
    BT::InputPort<std::string>("responsible_safety_class", "none", ""),
    BT::InputPort<bool>("responsible_openable", false, ""),
    BT::InputPort<bool>("responsible_clearable", false, ""),
    BT::InputPort<std::string>("responsible_match_type", "none", "verified | inferred | none"),
    BT::InputPort<std::string>("responsible_state_detail", "", ""),
    BT::InputPort<std::string>("responsible_traversability", "", ""),

    BT::InputPort<geometry_msgs::msg::Point>("blockage_centroid", ""),
    BT::InputPort<float>("blockage_extent_m", 0.0f, ""),

    BT::InputPort<std::string>("original_object_tag", "", ""),
    BT::InputPort<std::string>("original_intent_hint", "", ""),
    BT::InputPort<std::string>("current_target_object_key", "", ""),
    BT::InputPort<int>("local_db_version", 0, ""),
    BT::InputPort<std::string>("local_db_source", "static_snapshot", ""),

    // ---- Outputs ----
    BT::OutputPort<std::string>(
      "directive_action",
        "retry_target | wait_then_replan | open_door_then_replan | clear_object_then_replan | give_up"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "directive_target_pose", "New standoff pose for retry_target"),
    BT::OutputPort<std::string>("directive_target_object_key", ""),
    BT::OutputPort<std::string>("directive_target_object_tag", ""),
    BT::OutputPort<std::string>("directive_target_intent_hint", ""),
    BT::OutputPort<int>("directive_wait_seconds", ""),
    BT::OutputPort<bool>("directive_emit_signal_during_wait", ""),
    BT::OutputPort<int>("directive_signal_attempts", ""),
    BT::OutputPort<std::string>("directive_operator_message", ""),
    BT::OutputPort<std::string>("directive_rationale", ""),
    BT::OutputPort<int>("directive_confidence_percent", ""),
    BT::OutputPort<bool>("directive_escalate_to_operator", ""),
    BT::OutputPort<std::string>("recovery_event_id", ""),
  });
}

bool EscalateToLLMRecovery::readRobotPose(
  geometry_msgs::msg::PoseStamped & robot_pose) const
{
  std::string global_frame{"map"};
  std::string robot_base_frame{"base_link"};
  double transform_tolerance_s{0.1};

  getInput("global_frame", global_frame);
  getInput("robot_base_frame", robot_base_frame);
  getInput("transform_tolerance_s", transform_tolerance_s);

  return readCurrentRobotPose(
    config(),
    global_frame,
    robot_base_frame,
    transform_tolerance_s,
    robot_pose);
}

}  // namespace semantic_nav_nav2_plugins
