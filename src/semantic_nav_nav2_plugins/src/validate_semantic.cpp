// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include "semantic_nav_nav2_plugins/validate_semantic.hpp"

#include <cmath>
#include <string>

namespace semantic_nav_nav2_plugins
{
namespace
{
constexpr double kGoalTolerance = 1.0e-5;

bool nearlyEqual(const double a, const double b)
{
  return std::fabs(a - b) <= kGoalTolerance;
}
}  // namespace

ValidateSemantic::ValidateSemantic(
  const std::string & service_node_name,
  const BT::NodeConfiguration & conf)
: Base(service_node_name, conf)
{}

BT::NodeStatus ValidateSemantic::tick()
{
  // If a service request is already in flight, let BtServiceNode complete it.
  if (status() == BT::NodeStatus::RUNNING) {
    return Base::tick();
  }

  geometry_msgs::msg::PoseStamped goal_pose;
  if (!readGoal(goal_pose)) {
    const std::string reason{"missing_goal"};
    setOutput("validation_reason", reason);
    return BT::NodeStatus::FAILURE;
  }

  if (have_cached_goal_ && sameGoal(goal_pose, cached_goal_)) {
    setOutput("validation_reason", cached_message_);
    return cached_valid_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  pending_goal_ = goal_pose;
  return Base::tick();
}

void ValidateSemantic::on_tick()
{
  request_->goal = pending_goal_;
  request_->planner_id = "";
  request_->use_start = false;
}

BT::NodeStatus ValidateSemantic::on_completion(
  std::shared_ptr<ServiceT::Response> response)
{
  have_cached_goal_ = true;
  cached_goal_ = pending_goal_;
  cached_valid_ = response && response->valid;
  cached_message_ = response ? response->message : std::string("no_response");

  setOutput("validation_reason", cached_message_);
  return cached_valid_ ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
}

BT::PortsList ValidateSemantic::providedPorts()
{
  return providedBasicPorts({
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal_pose",
      "Pose to validate; if missing, reads blackboard key 'goal'"),
    BT::OutputPort<std::string>(
      "validation_reason",
      "ValidatePose message, or missing_goal/no_response"),
  });
}

bool ValidateSemantic::readGoal(geometry_msgs::msg::PoseStamped & goal_pose) const
{
  if (getInput<geometry_msgs::msg::PoseStamped>("goal_pose", goal_pose)) {
    return true;
  }

  if (config().blackboard &&
    config().blackboard->get<geometry_msgs::msg::PoseStamped>("goal", goal_pose))
  {
    return true;
  }

  return false;
}

bool ValidateSemantic::sameGoal(
  const geometry_msgs::msg::PoseStamped & a,
  const geometry_msgs::msg::PoseStamped & b) const
{
  const auto & ap = a.pose.position;
  const auto & bp = b.pose.position;
  const auto & ao = a.pose.orientation;
  const auto & bo = b.pose.orientation;

  return a.header.frame_id == b.header.frame_id &&
         nearlyEqual(ap.x, bp.x) && nearlyEqual(ap.y, bp.y) && nearlyEqual(ap.z, bp.z) &&
         nearlyEqual(ao.x, bo.x) && nearlyEqual(ao.y, bo.y) &&
         nearlyEqual(ao.z, bo.z) && nearlyEqual(ao.w, bo.w);
}

}  // namespace semantic_nav_nav2_plugins
