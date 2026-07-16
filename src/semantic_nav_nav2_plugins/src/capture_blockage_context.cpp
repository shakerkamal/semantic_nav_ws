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

  // Dedicated callback group + executor, spun from tick(): bt_navigator's
  // client node is never added to any executor, so a subscription on its
  // default callback group would silently never deliver (same reason
  // QuerySemanticContext spins its own group for service futures).
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions sub_options;
  sub_options.callback_group = callback_group_;

  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic,
    rclcpp::SystemDefaultsQoS(),
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_costmap_ = msg;
    },
    sub_options);
}

BT::NodeStatus CaptureBlockageContext::tick()
{
  callback_group_executor_.spin_some(std::chrono::nanoseconds(0));

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

  // No lethal cell on the path (stale/short path, blocker outside the
  // rolling local costmap, OR -- a fully-sealed corridor like S2's closed
  // door -- the planner found NO path at all, so there is nothing to sample).
  // Read the robot's TF pose; every fallback tier below is anchored on it.
  double fallback_lookahead_m{1.0};
  double fallback_extent_m{0.6};
  double fallback_search_radius_m{2.0};
  std::string global_frame{"map"};
  std::string robot_base_frame{"base_footprint"};
  double transform_tolerance_s{0.1};
  getInput("fallback_lookahead_m", fallback_lookahead_m);
  getInput("fallback_extent_m", fallback_extent_m);
  getInput("fallback_search_radius_m", fallback_search_radius_m);
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
  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;

  // Second tier: PERCEPTION-GROUNDED -- search the costmap the robot is
  // actually stopped in front of for the nearest lethal cell, instead of
  // guessing a centroid from geometry alone. This is what catches a fully
  // -sealed corridor: the path is empty so there is nothing to project
  // along, but the costmap already shows the real obstacle right there.
  geometry_msgs::msg::Point costmap_centroid;
  if (costmap && nearestLethalCentroidNearRobot(
      *costmap, robot_x, robot_y, fallback_search_radius_m,
      lethal_threshold, costmap_centroid))
  {
    setOutput("blockage_centroid", costmap_centroid);
    setOutput("blockage_extent_m", static_cast<float>(fallback_extent_m));

    RCLCPP_INFO(
      node_->get_logger(),
      "[CaptureBlockageContext] centroid=(%.3f,%.3f) extent=%.3fm"
      " source=costmap_near_robot robot=(%.3f,%.3f) (no lethal cell on path,"
      " found one near the robot instead)",
      costmap_centroid.x, costmap_centroid.y, fallback_extent_m,
      robot_x, robot_y);
    return BT::NodeStatus::SUCCESS;
  }

  // Last resort: pure geometric projection (no costmap, or genuinely nothing
  // lethal within search radius either). Anchor at the robot and step
  // forward ALONG THE PATH (if any) so the match at least searches next to
  // the robot instead of the map origin, which is what an unset centroid
  // defaults to.
  const geometry_msgs::msg::Point centroid = fallbackCentroidAlongPath(
    path, robot_x, robot_y, fallback_lookahead_m);
  setOutput("blockage_centroid", centroid);
  setOutput("blockage_extent_m", static_cast<float>(fallback_extent_m));

  RCLCPP_INFO(
    node_->get_logger(),
    "[CaptureBlockageContext] centroid=(%.3f,%.3f) extent=%.3fm source=fallback"
    " robot=(%.3f,%.3f) path_poses=%zu (no lethal cells sampled, none found"
    " near robot either)",
    centroid.x, centroid.y, fallback_extent_m,
    robot_x, robot_y,
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

bool CaptureBlockageContext::nearestLethalCentroidNearRobot(
  const nav_msgs::msg::OccupancyGrid & costmap,
  double robot_x,
  double robot_y,
  double search_radius_m,
  int lethal_threshold,
  geometry_msgs::msg::Point & out_centroid)
{
  const auto & info = costmap.info;
  if (info.width == 0 || info.height == 0 || info.resolution <= 0.0f) {
    return false;
  }

  const auto expected_size =
    static_cast<std::size_t>(info.width) * static_cast<std::size_t>(info.height);
  if (costmap.data.size() < expected_size) {
    return false;
  }

  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const double resolution = static_cast<double>(info.resolution);

  const int radius_cells = std::max(
    0, static_cast<int>(std::ceil(search_radius_m / resolution)));
  const int rx = static_cast<int>(std::floor((robot_x - origin_x) / resolution));
  const int ry = static_cast<int>(std::floor((robot_y - origin_y) / resolution));

  double sx = 0.0;
  double sy = 0.0;
  int count = 0;

  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int mx = rx + dx;
      const int my = ry + dy;

      if (mx < 0 || my < 0 ||
        mx >= static_cast<int>(info.width) ||
        my >= static_cast<int>(info.height))
      {
        continue;
      }

      const double wx = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
      const double wy = origin_y + (static_cast<double>(my) + 0.5) * resolution;

      if (std::hypot(wx - robot_x, wy - robot_y) > search_radius_m) {
        continue;
      }

      const std::size_t index =
        static_cast<std::size_t>(my) * static_cast<std::size_t>(info.width) +
        static_cast<std::size_t>(mx);

      if (index >= costmap.data.size()) {
        continue;
      }

      if (static_cast<int>(costmap.data[index]) >= lethal_threshold) {
        sx += wx;
        sy += wy;
        ++count;
      }
    }
  }

  if (count == 0) {
    return false;
  }

  out_centroid.x = sx / static_cast<double>(count);
  out_centroid.y = sy / static_cast<double>(count);
  out_centroid.z = 0.0;
  return true;
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
    BT::InputPort<double>(
      "fallback_search_radius_m",
      2.0,
      "When no lethal cell is sampled on the path, search this far around the"
      " robot's TF position in the costmap for the nearest lethal cell before"
      " falling back to pure path/robot-pose geometry"),
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
