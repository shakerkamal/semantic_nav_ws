// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include "semantic_nav_nav2_plugins/persistent_no_progress_condition.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <utility>

#include "semantic_nav_nav2_plugins/path_clear_condition.hpp"
#include "semantic_nav_nav2_plugins/robot_pose_util.hpp"

namespace semantic_nav_nav2_plugins
{
namespace
{

constexpr double kEpsilon = 1e-6;

std::size_t nearestPathIndex(
  const nav_msgs::msg::Path & path,
  double robot_x,
  double robot_y)
{
  std::size_t nearest = 0;
  double best_squared_distance = std::numeric_limits<double>::max();

  for (std::size_t i = 0; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - robot_x;
    const double dy = path.poses[i].pose.position.y - robot_y;
    const double squared_distance = (dx * dx) + (dy * dy);
    if (squared_distance < best_squared_distance) {
      best_squared_distance = squared_distance;
      nearest = i;
    }
  }

  return nearest;
}

nav_msgs::msg::Path pathFromIndex(
  const nav_msgs::msg::Path & path,
  std::size_t first_index)
{
  nav_msgs::msg::Path forward_path;
  forward_path.header = path.header;
  if (first_index >= path.poses.size()) {
    return forward_path;
  }

  forward_path.poses.insert(
    forward_path.poses.end(),
    path.poses.begin() + static_cast<std::ptrdiff_t>(first_index),
    path.poses.end());
  return forward_path;
}

bool immediatePathDirection(
  const nav_msgs::msg::Path & path,
  std::size_t nearest_index,
  double robot_x,
  double robot_y,
  double target_distance_m,
  double & ux,
  double & uy)
{
  if (path.poses.empty() || nearest_index >= path.poses.size()) {
    return false;
  }

  std::size_t target_index = nearest_index;
  for (std::size_t i = nearest_index; i < path.poses.size(); ++i) {
    const double dx = path.poses[i].pose.position.x - robot_x;
    const double dy = path.poses[i].pose.position.y - robot_y;
    target_index = i;
    if (std::hypot(dx, dy) >= target_distance_m) {
      break;
    }
  }

  double dx = path.poses[target_index].pose.position.x - robot_x;
  double dy = path.poses[target_index].pose.position.y - robot_y;
  double norm = std::hypot(dx, dy);

  if (norm <= kEpsilon && nearest_index + 1 < path.poses.size()) {
    dx = path.poses[nearest_index + 1].pose.position.x -
      path.poses[nearest_index].pose.position.x;
    dy = path.poses[nearest_index + 1].pose.position.y -
      path.poses[nearest_index].pose.position.y;
    norm = std::hypot(dx, dy);
  }

  if (norm <= kEpsilon) {
    return false;
  }

  ux = dx / norm;
  uy = dy / norm;
  return true;
}

}  // namespace

PersistentNoProgressCondition::PersistentNoProgressCondition(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string costmap_topic{"/local_costmap/costmap"};
  getInput("local_costmap_topic", costmap_topic);

  // bt_navigator's client node is not spun by a normal executor. Use the same
  // dedicated callback-group pattern as the package's other subscription-based
  // BT plugins so fresh local-costmap messages are delivered from tick().
  callback_group_ = node_->create_callback_group(
    rclcpp::CallbackGroupType::MutuallyExclusive, false);
  callback_group_executor_.add_callback_group(
    callback_group_, node_->get_node_base_interface());

  rclcpp::SubscriptionOptions options;
  options.callback_group = callback_group_;
  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic,
    rclcpp::SystemDefaultsQoS(),
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_costmap_ = std::move(msg);
    },
    options);
}

BT::NodeStatus PersistentNoProgressCondition::tick()
{
  callback_group_executor_.spin_some(std::chrono::nanoseconds(0));

  double observation_window_s{8.0};
  double minimum_progress_m{0.10};
  double obstacle_persistence_s{2.0};
  double obstacle_lookahead_m{1.5};
  double path_sample_radius_m{0.20};
  double forward_lateral_tolerance_m{0.40};
  double min_forward_distance_m{0.15};
  int min_lethal_cells{3};
  int lethal_threshold{90};
  std::string global_frame{"map"};
  std::string robot_base_frame{"base_footprint"};
  double transform_tolerance_s{0.1};
  double goal_reset_tolerance_m{0.05};

  getInput("observation_window_s", observation_window_s);
  getInput("minimum_progress_m", minimum_progress_m);
  getInput("obstacle_persistence_s", obstacle_persistence_s);
  getInput("obstacle_lookahead_m", obstacle_lookahead_m);
  getInput("path_sample_radius_m", path_sample_radius_m);
  getInput("forward_lateral_tolerance_m", forward_lateral_tolerance_m);
  getInput("min_forward_distance_m", min_forward_distance_m);
  getInput("min_lethal_cells", min_lethal_cells);
  getInput("lethal_threshold", lethal_threshold);
  getInput("global_frame", global_frame);
  getInput("robot_base_frame", robot_base_frame);
  getInput("transform_tolerance_s", transform_tolerance_s);
  getInput("goal_reset_tolerance_m", goal_reset_tolerance_m);

  observation_window_s = std::max(0.1, observation_window_s);
  minimum_progress_m = std::max(0.0, minimum_progress_m);
  obstacle_persistence_s = std::max(0.0, obstacle_persistence_s);
  obstacle_lookahead_m = std::max(0.1, obstacle_lookahead_m);
  path_sample_radius_m = std::max(0.0, path_sample_radius_m);
  forward_lateral_tolerance_m = std::max(0.0, forward_lateral_tolerance_m);
  min_forward_distance_m = std::max(0.0, min_forward_distance_m);
  min_lethal_cells = std::max(1, min_lethal_cells);
  lethal_threshold = std::max(0, std::min(100, lethal_threshold));
  goal_reset_tolerance_m = std::max(0.0, goal_reset_tolerance_m);

  geometry_msgs::msg::PoseStamped robot_pose;
  if (!readCurrentRobotPose(
      config(), global_frame, robot_base_frame,
      transform_tolerance_s, robot_pose))
  {
    progress_anchor_valid_ = false;
    clearObstacleTimer();
    setOutput("no_progress_elapsed_s", 0.0);
    setOutput("monitor_status", std::string("tf_unavailable_fail_open"));
    return BT::NodeStatus::SUCCESS;
  }

  const auto now = SteadyClock::now();
  const double robot_x = robot_pose.pose.position.x;
  const double robot_y = robot_pose.pose.position.y;

  geometry_msgs::msg::PoseStamped current_goal;
  if (getInput("goal", current_goal)) {
    if (!previous_goal_valid_ ||
      goalChanged(previous_goal_, current_goal, goal_reset_tolerance_m))
    {
      previous_goal_ = current_goal;
      previous_goal_valid_ = true;
      resetProgressAnchor(robot_x, robot_y, now);
      RCLCPP_INFO(
        node_->get_logger(),
        "[PersistentNoProgressCondition] monitoring new goal=(%.3f,%.3f) "
        "window=%.1fs minimum_progress=%.2fm",
        current_goal.pose.position.x, current_goal.pose.position.y,
        observation_window_s, minimum_progress_m);
    }
  }

  if (!progress_anchor_valid_) {
    resetProgressAnchor(robot_x, robot_y, now);
  }

  const double displacement = std::hypot(
    robot_x - progress_anchor_x_,
    robot_y - progress_anchor_y_);

  if (displacement >= minimum_progress_m && minimum_progress_m > 0.0) {
    resetProgressAnchor(robot_x, robot_y, now);
    setOutput("no_progress_elapsed_s", 0.0);
    setOutput("monitor_status", std::string("progress_observed"));
    return BT::NodeStatus::SUCCESS;
  }

  const double no_progress_elapsed_s =
    std::chrono::duration<double>(now - progress_anchor_time_).count();
  setOutput("no_progress_elapsed_s", no_progress_elapsed_s);

  nav_msgs::msg::Path path;
  const bool have_path = getInput("path", path) && !path.poses.empty();

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    costmap = latest_costmap_;
  }

  if (!have_path || !costmap) {
    clearObstacleTimer();
    setOutput(
      "monitor_status",
      std::string(have_path ? "costmap_unavailable_fail_open" :
      "path_unavailable_fail_open"));
    return BT::NodeStatus::SUCCESS;
  }

  const ExecutionObstacleEvidence evidence = detectObstacleEvidence(
    path, *costmap, robot_x, robot_y, lethal_threshold,
    obstacle_lookahead_m, path_sample_radius_m,
    forward_lateral_tolerance_m, min_forward_distance_m,
    min_lethal_cells);

  if (!evidence.blocked) {
    clearObstacleTimer();
    setOutput("monitor_status", std::string("stationary_without_obstacle"));
    return BT::NodeStatus::SUCCESS;
  }

  setOutput("blockage_centroid", evidence.centroid);
  setOutput("blockage_extent_m", evidence.extent_m);

  if (!obstacle_timer_valid_) {
    obstacle_timer_valid_ = true;
    obstacle_since_ = now;
  }

  const double obstacle_elapsed_s =
    std::chrono::duration<double>(now - obstacle_since_).count();

  if (no_progress_elapsed_s >= observation_window_s &&
    obstacle_elapsed_s >= obstacle_persistence_s)
  {
    setOutput("monitor_status", std::string("stalled_with_obstacle"));

    RCLCPP_WARN(
      node_->get_logger(),
      "[PersistentNoProgressCondition] execution stalled: moved=%.3fm "
      "during %.2fs, obstacle persisted %.2fs, source=%s lethal_cells=%d "
      "centroid=(%.3f,%.3f) extent=%.3fm; interrupting FollowPath",
      displacement, no_progress_elapsed_s, obstacle_elapsed_s,
      evidence.source.c_str(), evidence.lethal_cells,
      evidence.centroid.x, evidence.centroid.y, evidence.extent_m);

    // A failed tick is consumed by the outer RecoveryNode. Start the next main
    // navigation attempt with a fresh observation window instead of preserving
    // a latched failure across geometric or semantic recovery.
    resetProgressAnchor(robot_x, robot_y, now);
    return BT::NodeStatus::FAILURE;
  }

  setOutput("monitor_status", std::string("stationary_with_obstacle_monitoring"));
  return BT::NodeStatus::SUCCESS;
}

ExecutionObstacleEvidence
PersistentNoProgressCondition::detectObstacleEvidence(
  const nav_msgs::msg::Path & path,
  const nav_msgs::msg::OccupancyGrid & costmap,
  double robot_x,
  double robot_y,
  int lethal_threshold,
  double obstacle_lookahead_m,
  double path_sample_radius_m,
  double forward_lateral_tolerance_m,
  double min_forward_distance_m,
  int min_lethal_cells)
{
  ExecutionObstacleEvidence evidence;

  const auto & info = costmap.info;
  if (path.poses.empty() || info.width == 0 || info.height == 0 ||
    info.resolution <= 0.0F)
  {
    return evidence;
  }

  const std::size_t expected_size =
    static_cast<std::size_t>(info.width) *
    static_cast<std::size_t>(info.height);
  if (costmap.data.size() < expected_size) {
    return evidence;
  }

  const std::size_t nearest_index = nearestPathIndex(path, robot_x, robot_y);
  const nav_msgs::msg::Path forward_path = pathFromIndex(path, nearest_index);

  // First test: use a wider-than-centerline corridor around the fresh Smac
  // path. This represents the controller/footprint clearance that may be
  // missing from a mathematically valid centerline path.
  const BlockageMetrics path_metrics = PathClearCondition::isCorridorBlocked(
    forward_path, costmap, lethal_threshold,
    obstacle_lookahead_m, path_sample_radius_m);
  if (path_metrics.any_blocked) {
    evidence.blocked = true;
    evidence.lethal_cells = std::max(1, path_metrics.blocked_poses);
    evidence.centroid = path_metrics.centroid;
    evidence.extent_m = path_metrics.extent_m;
    evidence.source = "expanded_path_corridor";
    return evidence;
  }

  // Second test: inspect a short rectangular corridor directly ahead along the
  // immediate path direction. Smac may bend its centerline around a person,
  // while the controller still cannot enter that bend from the current pose.
  double ux = 0.0;
  double uy = 0.0;
  const double direction_target_m = std::min(0.5, obstacle_lookahead_m);
  if (!immediatePathDirection(
      path, nearest_index, robot_x, robot_y,
      direction_target_m, ux, uy))
  {
    return evidence;
  }

  const double resolution = static_cast<double>(info.resolution);
  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const double search_radius = std::hypot(
    obstacle_lookahead_m, forward_lateral_tolerance_m);
  const int radius_cells = static_cast<int>(
    std::ceil(search_radius / resolution));
  const int robot_mx = static_cast<int>(
    std::floor((robot_x - origin_x) / resolution));
  const int robot_my = static_cast<int>(
    std::floor((robot_y - origin_y) / resolution));

  double sum_x = 0.0;
  double sum_y = 0.0;
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::lowest();
  double max_y = std::numeric_limits<double>::lowest();
  int lethal_cells = 0;

  for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
    for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
      const int mx = robot_mx + dx;
      const int my = robot_my + dy;
      if (mx < 0 || my < 0 ||
        mx >= static_cast<int>(info.width) ||
        my >= static_cast<int>(info.height))
      {
        continue;
      }

      const double wx = origin_x +
        (static_cast<double>(mx) + 0.5) * resolution;
      const double wy = origin_y +
        (static_cast<double>(my) + 0.5) * resolution;
      const double rel_x = wx - robot_x;
      const double rel_y = wy - robot_y;
      const double longitudinal = (rel_x * ux) + (rel_y * uy);
      const double lateral = std::abs((-rel_x * uy) + (rel_y * ux));

      if (longitudinal < min_forward_distance_m ||
        longitudinal > obstacle_lookahead_m ||
        lateral > forward_lateral_tolerance_m)
      {
        continue;
      }

      const std::size_t index =
        static_cast<std::size_t>(my) *
        static_cast<std::size_t>(info.width) +
        static_cast<std::size_t>(mx);
      if (static_cast<int>(costmap.data[index]) < lethal_threshold) {
        continue;
      }

      sum_x += wx;
      sum_y += wy;
      min_x = std::min(min_x, wx);
      min_y = std::min(min_y, wy);
      max_x = std::max(max_x, wx);
      max_y = std::max(max_y, wy);
      ++lethal_cells;
    }
  }

  if (lethal_cells < std::max(1, min_lethal_cells)) {
    return evidence;
  }

  evidence.blocked = true;
  evidence.lethal_cells = lethal_cells;
  evidence.centroid.x = sum_x / static_cast<double>(lethal_cells);
  evidence.centroid.y = sum_y / static_cast<double>(lethal_cells);
  evidence.centroid.z = 0.0;
  evidence.extent_m = static_cast<float>(std::max(
    (max_x - min_x) + resolution,
    (max_y - min_y) + resolution));
  evidence.source = "forward_execution_corridor";
  return evidence;
}

BT::PortsList PersistentNoProgressCondition::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>(
      "path", "Fresh path produced by ComputePathToPose"),
    BT::InputPort<geometry_msgs::msg::PoseStamped>(
      "goal", "Current NavigateToPose goal; changes reset the monitor"),
    BT::InputPort<double>(
      "observation_window_s", 8.0,
      "Robot must remain below minimum_progress_m for this duration"),
    BT::InputPort<double>(
      "minimum_progress_m", 0.10,
      "Translation that resets the no-progress observation window"),
    BT::InputPort<double>(
      "obstacle_persistence_s", 2.0,
      "Obstacle evidence must persist this long before escalation"),
    BT::InputPort<double>(
      "obstacle_lookahead_m", 1.5,
      "Distance ahead of the robot/path inspected for obstacle evidence"),
    BT::InputPort<double>(
      "path_sample_radius_m", 0.20,
      "Expanded sampling radius around the Smac centerline"),
    BT::InputPort<double>(
      "forward_lateral_tolerance_m", 0.40,
      "Half-width of the short forward execution corridor"),
    BT::InputPort<double>(
      "min_forward_distance_m", 0.15,
      "Ignore lethal cells immediately under/behind the robot"),
    BT::InputPort<int>(
      "min_lethal_cells", 3,
      "Minimum lethal cells for forward-corridor evidence"),
    BT::InputPort<int>(
      "lethal_threshold", 90,
      "OccupancyGrid value treated as lethal"),
    BT::InputPort<std::string>(
      "local_costmap_topic", "/local_costmap/costmap",
      "Local costmap OccupancyGrid topic"),
    BT::InputPort<std::string>(
      "global_frame", "map",
      "Frame used for robot pose and local-costmap sampling"),
    BT::InputPort<std::string>(
      "robot_base_frame", "base_footprint",
      "Robot base frame used for the TF pose"),
    BT::InputPort<double>(
      "transform_tolerance_s", 0.1,
      "TF lookup tolerance"),
    BT::InputPort<double>(
      "goal_reset_tolerance_m", 0.05,
      "Goal-position change that resets persistent monitor state"),
    BT::OutputPort<geometry_msgs::msg::Point>(
      "blockage_centroid",
      "Measured local blockage centroid when obstacle evidence exists"),
    BT::OutputPort<float>(
      "blockage_extent_m",
      "Measured local blockage extent; kept float for shared BT key type"),
    BT::OutputPort<double>(
      "no_progress_elapsed_s",
      "Current time since the robot last moved minimum_progress_m"),
    BT::OutputPort<std::string>(
      "monitor_status",
      "Execution monitor diagnostic state"),
  };
}

void PersistentNoProgressCondition::resetProgressAnchor(
  double robot_x,
  double robot_y,
  const SteadyClock::time_point & now)
{
  progress_anchor_valid_ = true;
  progress_anchor_x_ = robot_x;
  progress_anchor_y_ = robot_y;
  progress_anchor_time_ = now;
  clearObstacleTimer();
}

void PersistentNoProgressCondition::clearObstacleTimer()
{
  obstacle_timer_valid_ = false;
}

bool PersistentNoProgressCondition::goalChanged(
  const geometry_msgs::msg::PoseStamped & previous,
  const geometry_msgs::msg::PoseStamped & current,
  double position_tolerance_m)
{
  if (previous.header.frame_id != current.header.frame_id) {
    return true;
  }

  return std::hypot(
    previous.pose.position.x - current.pose.position.x,
    previous.pose.position.y - current.pose.position.y) > position_tolerance_m;
}

}  // namespace semantic_nav_nav2_plugins
