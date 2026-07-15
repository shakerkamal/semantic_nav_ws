// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include "semantic_nav_nav2_plugins/capture_blockage_context.hpp"

#include <cmath>
#include <cstddef>
#include <limits>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "semantic_nav_nav2_plugins/robot_pose_util.hpp"

namespace semantic_nav_nav2_plugins
{

CaptureBlockageContext::CaptureBlockageContext(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::SyncActionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string costmap_topic{"/local_costmap/costmap"};
  getInput("local_costmap_topic", costmap_topic);

  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic,
    rclcpp::SystemDefaultsQoS(),
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_costmap_ = msg;
    });
}

BT::NodeStatus CaptureBlockageContext::tick()
{
  nav_msgs::msg::Path path;
  const bool have_path =
    getInput<nav_msgs::msg::Path>("path", path) && !path.poses.empty();

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    costmap = latest_costmap_;
  }

  double lookahead_m{3.0};
  int lethal_threshold{90};
  double sample_radius_m{0.05};
  getInput("lookahead_m", lookahead_m);
  getInput("lethal_threshold", lethal_threshold);
  getInput("sample_radius_m", sample_radius_m);

  // Primary path: centroid of the lethal cells actually sampled on the path.
  if (have_path && costmap) {
    const BlockageMetrics metrics = PathClearCondition::isCorridorBlocked(
      path, *costmap, lethal_threshold, lookahead_m, sample_radius_m);

    if (metrics.any_blocked) {
      setOutput("blockage_centroid", metrics.centroid);
      setOutput("blockage_extent_m", metrics.extent_m);

      RCLCPP_INFO(
        node_->get_logger(),
        "[CaptureBlockageContext] centroid=(%.3f,%.3f) extent=%.3fm source=measured"
        " poses=%d/%d fraction=%.2f",
        metrics.centroid.x, metrics.centroid.y, metrics.extent_m,
        metrics.blocked_poses, metrics.total_poses, metrics.blocked_fraction);
      return BT::NodeStatus::SUCCESS;
    }
  }

  // Fallback: sampling found no lethal cell (stale/short path, or the blocker
  // fell outside the rolling local costmap). Anchor the centroid at the robot
  // and step forward ALONG THE PATH so the responsible-object match searches
  // next to the robot -- right where the blocker is -- instead of the map
  // origin, which is what an unset centroid defaults to.
  double fallback_lookahead_m{1.0};
  double fallback_extent_m{0.6};
  std::string global_frame{"map"};
  std::string robot_base_frame{"base_footprint"};
  double transform_tolerance_s{0.1};
  getInput("fallback_lookahead_m", fallback_lookahead_m);
  getInput("fallback_extent_m", fallback_extent_m);
  getInput("global_frame", global_frame);
  getInput("robot_base_frame", robot_base_frame);
  getInput("transform_tolerance_s", transform_tolerance_s);

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!readCurrentRobotPose(
      config(), global_frame, robot_base_frame, transform_tolerance_s, robot_pose))
  {
    RCLCPP_WARN(
      node_->get_logger(),
      "[CaptureBlockageContext] no lethal cells sampled and TF robot pose"
      " unavailable; blockage_centroid left UNSET (recovery lacks spatial context)");
    return BT::NodeStatus::SUCCESS;
  }

  const geometry_msgs::msg::Point centroid = fallbackCentroidAlongPath(
    path, robot_pose.pose.position.x, robot_pose.pose.position.y,
    fallback_lookahead_m);
  setOutput("blockage_centroid", centroid);
  setOutput("blockage_extent_m", static_cast<float>(fallback_extent_m));

  RCLCPP_INFO(
    node_->get_logger(),
    "[CaptureBlockageContext] centroid=(%.3f,%.3f) extent=%.3fm source=fallback"
    " robot=(%.3f,%.3f) path_poses=%zu (no lethal cells sampled)",
    centroid.x, centroid.y, fallback_extent_m,
    robot_pose.pose.position.x, robot_pose.pose.position.y,
    have_path ? path.poses.size() : 0UL);

  return BT::NodeStatus::SUCCESS;
}

geometry_msgs::msg::Point CaptureBlockageContext::fallbackCentroidAlongPath(
  const nav_msgs::msg::Path & path,
  double robot_x,
  double robot_y,
  double lookahead_m)
{
  geometry_msgs::msg::Point centroid;
  centroid.x = robot_x;
  centroid.y = robot_y;
  centroid.z = 0.0;

  if (path.poses.empty()) {
    return centroid;   // best effort: the robot's own position
  }

  // Nearest path pose to the robot.
  std::size_t nearest = 0;
  double best = std::numeric_limits<double>::max();
  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - robot_x;
    const double dy = path.poses[i].pose.position.y - robot_y;
    const double d = (dx * dx) + (dy * dy);
    if (d < best) {
      best = d;
      nearest = i;
    }
  }

  // Step forward along the path from there by lookahead_m (toward the goal, so
  // toward the blocker), clamping to the path end.
  std::size_t idx = nearest;
  double travelled = 0.0;
  for (std::size_t i = nearest + 1; i < path.poses.size(); ++i) {
    travelled += std::hypot(
      path.poses[i].pose.position.x - path.poses[i - 1].pose.position.x,
      path.poses[i].pose.position.y - path.poses[i - 1].pose.position.y);
    idx = i;
    if (travelled >= lookahead_m) {
      break;
    }
  }

  centroid.x = path.poses[idx].pose.position.x;
  centroid.y = path.poses[idx].pose.position.y;
  centroid.z = 0.0;
  return centroid;
}

BT::PortsList CaptureBlockageContext::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>(
      "path",
      "BT blackboard path from ComputePathToPose"),
    BT::InputPort<double>(
      "lookahead_m",
      3.0,
      "Metres of path to scan for blocked cells"),
    BT::InputPort<int>(
      "lethal_threshold",
      90,
      "OccupancyGrid cost treated as lethal"),
    BT::InputPort<double>(
      "sample_radius_m",
      0.05,
      "Sampling radius per pose; 0.0 checks only the containing cell"),
    BT::InputPort<std::string>(
      "local_costmap_topic",
      "/local_costmap/costmap",
      "OccupancyGrid topic"),
    BT::InputPort<double>(
      "fallback_lookahead_m",
      1.0,
      "When no lethal cell is sampled, project this far along the path ahead of"
      " the robot for the fallback centroid"),
    BT::InputPort<double>(
      "fallback_extent_m",
      0.6,
      "Blockage extent assumed for the fallback centroid"),
    BT::InputPort<std::string>(
      "global_frame",
      "map",
      "Frame the robot pose (and centroid) are reported in"),
    BT::InputPort<std::string>(
      "robot_base_frame",
      "base_footprint",
      "Robot base frame for the fallback TF pose (both robots publish it)"),
    BT::InputPort<double>(
      "transform_tolerance_s",
      0.1,
      "TF lookup tolerance when reading the current robot pose"),
    BT::OutputPort<geometry_msgs::msg::Point>(
      "blockage_centroid",
      "Centroid of blocked costmap cells"),
    BT::OutputPort<float>(
      "blockage_extent_m",
      "Approximate blocked-region diameter in metres"),
  };
}

}  // namespace semantic_nav_nav2_plugins
