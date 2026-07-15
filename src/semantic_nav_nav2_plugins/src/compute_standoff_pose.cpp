// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include "semantic_nav_nav2_plugins/compute_standoff_pose.hpp"

#include <cmath>

#include "semantic_nav_nav2_plugins/robot_pose_util.hpp"

namespace semantic_nav_nav2_plugins
{

namespace
{
constexpr double kEps = 1e-6;
}  // namespace

ComputeStandoffPose::ComputeStandoffPose(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");
}

BT::NodeStatus ComputeStandoffPose::tick()
{
  std::string global_frame{"map"};
  std::string robot_base_frame{"base_footprint"};
  double transform_tolerance_s{0.1};
  getInput("global_frame", global_frame);
  getInput("robot_base_frame", robot_base_frame);
  getInput("transform_tolerance_s", transform_tolerance_s);

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!readCurrentRobotPose(
      config(), global_frame, robot_base_frame, transform_tolerance_s, robot_pose))
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "[ComputeStandoffPose] TF robot pose unavailable; standoff_goal left UNSET");
    return BT::NodeStatus::SUCCESS;
  }

  geometry_msgs::msg::Point bbox_center;
  geometry_msgs::msg::Vector3 bbox_extent;
  getInput("responsible_bbox_center", bbox_center);
  getInput("responsible_bbox_extent", bbox_extent);

  double robot_footprint_radius{0.1};
  double clearance_margin{0.20};
  getInput("robot_footprint_radius", robot_footprint_radius);
  getInput("clearance_margin", clearance_margin);

  const auto standoff = computeStandoffPose(
    robot_pose.pose.position.x, robot_pose.pose.position.y,
    bbox_center.x, bbox_center.y,
    bbox_extent.x, bbox_extent.y,
    robot_footprint_radius, clearance_margin,
    global_frame);

  setOutput("standoff_goal", standoff);

  RCLCPP_INFO(
    node_->get_logger(),
    "[ComputeStandoffPose] robot=(%.3f,%.3f) bbox_center=(%.3f,%.3f)"
    " bbox_extent=(%.3f,%.3f) -> standoff=(%.3f,%.3f)",
    robot_pose.pose.position.x, robot_pose.pose.position.y,
    bbox_center.x, bbox_center.y, bbox_extent.x, bbox_extent.y,
    standoff.pose.position.x, standoff.pose.position.y);

  return BT::NodeStatus::SUCCESS;
}

geometry_msgs::msg::PoseStamped ComputeStandoffPose::computeStandoffPose(
  double robot_x,
  double robot_y,
  double bbox_center_x,
  double bbox_center_y,
  double bbox_extent_x,
  double bbox_extent_y,
  double robot_footprint_radius,
  double clearance_margin,
  const std::string & frame_id)
{
  double vx = bbox_center_x - robot_x;
  double vy = bbox_center_y - robot_y;
  double norm = std::hypot(vx, vy);
  if (norm < kEps) {
    vx = 1.0;
    vy = 0.0;
    norm = 1.0;
  }

  const double half_extent_xy = 0.5 * std::max(bbox_extent_x, bbox_extent_y);
  const double d = half_extent_xy + robot_footprint_radius + clearance_margin;

  const double ux = vx / norm;
  const double uy = vy / norm;
  const double gx = bbox_center_x - d * ux;
  const double gy = bbox_center_y - d * uy;

  const double yaw = std::atan2(bbox_center_y - gy, bbox_center_x - gx);

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = frame_id;
  pose.pose.position.x = gx;
  pose.pose.position.y = gy;
  pose.pose.position.z = 0.0;
  pose.pose.orientation.z = std::sin(yaw / 2.0);
  pose.pose.orientation.w = std::cos(yaw / 2.0);
  return pose;
}

BT::PortsList ComputeStandoffPose::providedPorts()
{
  return {
    BT::InputPort<geometry_msgs::msg::Point>(
      "responsible_bbox_center",
      "Detected/matched object's bbox center (map frame)"),
    BT::InputPort<geometry_msgs::msg::Vector3>(
      "responsible_bbox_extent",
      "Detected/matched object's bbox extent"),
    BT::InputPort<double>(
      "robot_footprint_radius",
      0.1,
      "Robot footprint radius (rover default 0.1m; see rover_semantic_nav_params.yaml)"),
    BT::InputPort<double>(
      "clearance_margin",
      0.20,
      "Extra clearance beyond footprint + half object extent"),
    BT::InputPort<std::string>(
      "global_frame",
      "map",
      "Frame the robot pose and standoff_goal are reported in"),
    BT::InputPort<std::string>(
      "robot_base_frame",
      "base_footprint",
      "Robot base frame for the TF pose (both robots publish it)"),
    BT::InputPort<double>(
      "transform_tolerance_s",
      0.1,
      "TF lookup tolerance when reading the current robot pose"),
    BT::OutputPort<geometry_msgs::msg::PoseStamped>(
      "standoff_goal",
      "Computed standoff pose: (half max bbox extent + footprint + margin)"
      " from the object's bbox center, on the side facing the robot, facing"
      " back toward the object -- mirrors"
      " semantic_nav_semantics/standoff_planner.py:StandoffPlanner.plan()"),
  };
}

}  // namespace semantic_nav_nav2_plugins
