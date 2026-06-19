// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include "semantic_nav_nav2_plugins/path_clear_condition.hpp"

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <string>
#include <unordered_set>
#include <vector>

namespace semantic_nav_nav2_plugins
{

PathClearCondition::PathClearCondition(
  const std::string & name,
  const BT::NodeConfiguration & conf)
: BT::ConditionNode(name, conf)
{
  node_ = config().blackboard->get<rclcpp::Node::SharedPtr>("node");

  std::string plan_topic{"/plan"};
  std::string costmap_topic{"/local_costmap/costmap"};
  getInput("plan_topic", plan_topic);
  getInput("local_costmap_topic", costmap_topic);

  const auto qos = rclcpp::SystemDefaultsQoS();

  plan_sub_ = node_->create_subscription<nav_msgs::msg::Path>(
    plan_topic,
    qos,
    [this](nav_msgs::msg::Path::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_plan_ = msg;
    });

  costmap_sub_ = node_->create_subscription<nav_msgs::msg::OccupancyGrid>(
    costmap_topic,
    qos,
    [this](nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
      std::lock_guard<std::mutex> lock(data_mutex_);
      latest_costmap_ = msg;
    });
}

BT::NodeStatus PathClearCondition::tick()
{
  nav_msgs::msg::Path path_from_blackboard;
  const bool have_blackboard_path =
    getInput<nav_msgs::msg::Path>("path", path_from_blackboard) &&
    !path_from_blackboard.poses.empty();

  nav_msgs::msg::Path path;
  nav_msgs::msg::OccupancyGrid::SharedPtr costmap;

  {
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (have_blackboard_path) {
      path = path_from_blackboard;
    } else if (latest_plan_ && !latest_plan_->poses.empty()) {
      path = *latest_plan_;
    }

    costmap = latest_costmap_;
  }

  // No data yet — treat as clear so startup timing does not trigger false recovery.
  if (path.poses.empty() || !costmap) {
    blocked_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }

  int lethal_threshold{90};
  double lookahead_m{1.5};
  double sample_radius_m{0.05};
  int debounce_ticks{2};
  double min_blocked_length_m{0.45};
  int min_blocked_samples{4};
  double blocked_fraction_threshold{0.30};
  bool allow_geometric_detour_first{true};

  getInput("lethal_threshold", lethal_threshold);
  getInput("lookahead_m", lookahead_m);
  getInput("sample_radius_m", sample_radius_m);
  getInput("debounce_ticks", debounce_ticks);
  getInput("min_blocked_length_m", min_blocked_length_m);
  getInput("min_blocked_samples", min_blocked_samples);
  getInput("blocked_fraction_threshold", blocked_fraction_threshold);
  getInput("allow_geometric_detour_first", allow_geometric_detour_first);

  debounce_ticks = std::max(1, debounce_ticks);
  sample_radius_m = std::max(0.0, sample_radius_m);

  const BlockageMetrics metrics = isCorridorBlocked(
    path,
    *costmap,
    lethal_threshold,
    lookahead_m,
    sample_radius_m);

  if (!metrics.any_blocked) {
    blocked_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }

  // Severity gating: if blockage is minor (few poses, short run, low fraction)
  // return SUCCESS and let Nav2 replan around it without escalating to recovery.
  if (allow_geometric_detour_first) {
    const bool minor =
      (metrics.blocked_poses < min_blocked_samples) ||
      (metrics.max_run_length_m < min_blocked_length_m) ||
      (metrics.blocked_fraction < blocked_fraction_threshold);
    if (minor) {
      blocked_count_ = 0;
      return BT::NodeStatus::SUCCESS;
    }
  }

  // Significant blockage — apply debounce before firing FAILURE.
  last_centroid_ = metrics.centroid;
  last_extent_ = metrics.extent_m;
  blocked_count_ = std::min(blocked_count_ + 1, debounce_ticks + 1);

  if (blocked_count_ < debounce_ticks) {
    return BT::NodeStatus::SUCCESS;
  }

  setOutput("blockage_centroid", last_centroid_);
  setOutput("blockage_extent_m", last_extent_);

  RCLCPP_DEBUG(
    node_->get_logger(),
    "[PathClearCondition] FAILURE: poses=%d/%d fraction=%.2f run=%.2fm"
    " centroid=(%.3f,%.3f) extent=%.3f ticks=%d",
    metrics.blocked_poses,
    metrics.total_poses,
    metrics.blocked_fraction,
    metrics.max_run_length_m,
    last_centroid_.x,
    last_centroid_.y,
    last_extent_,
    blocked_count_);

  return BT::NodeStatus::FAILURE;
}

BlockageMetrics PathClearCondition::isCorridorBlocked(
  const nav_msgs::msg::Path & path,
  const nav_msgs::msg::OccupancyGrid & costmap,
  int lethal_threshold,
  double lookahead_m,
  double sample_radius_m)
{
  BlockageMetrics metrics;

  if (path.poses.empty()) {
    return metrics;
  }

  const auto & info = costmap.info;

  if (info.width == 0 || info.height == 0 || info.resolution <= 0.0f) {
    return metrics;
  }

  const auto expected_size =
    static_cast<std::size_t>(info.width) * static_cast<std::size_t>(info.height);

  if (costmap.data.size() < expected_size) {
    return metrics;
  }

  const double origin_x = info.origin.position.x;
  const double origin_y = info.origin.position.y;
  const double resolution = static_cast<double>(info.resolution);

  const int radius_cells = std::max(
    0,
    static_cast<int>(std::ceil(sample_radius_m / resolution)));

  // Globally unique blocked cell positions — for centroid and extent.
  std::vector<geometry_msgs::msg::Point> all_blocked_points;
  std::unordered_set<std::size_t> seen_indices;

  double travelled_m = 0.0;
  double prev_x = path.poses.front().pose.position.x;
  double prev_y = path.poses.front().pose.position.y;

  // Run-length tracking (consecutive blocked path poses).
  bool run_in_progress = false;
  double run_start_dist = 0.0;

  for (const auto & pose_stamped : path.poses) {
    const double px = pose_stamped.pose.position.x;
    const double py = pose_stamped.pose.position.y;

    travelled_m += std::hypot(px - prev_x, py - prev_y);
    prev_x = px;
    prev_y = py;

    if (travelled_m > lookahead_m) {
      break;
    }

    const int cx = static_cast<int>(std::floor((px - origin_x) / resolution));
    const int cy = static_cast<int>(std::floor((py - origin_y) / resolution));

    bool this_pose_blocked = false;

    for (int dy = -radius_cells; dy <= radius_cells; ++dy) {
      for (int dx = -radius_cells; dx <= radius_cells; ++dx) {
        const int mx = cx + dx;
        const int my = cy + dy;

        if (mx < 0 || my < 0 ||
          mx >= static_cast<int>(info.width) ||
          my >= static_cast<int>(info.height))
        {
          continue;
        }

        const double wx = origin_x + (static_cast<double>(mx) + 0.5) * resolution;
        const double wy = origin_y + (static_cast<double>(my) + 0.5) * resolution;

        // When sample_radius_m == 0, skip the distance filter so the containing
        // cell is always checked regardless of where the plan pose falls within it.
        if (sample_radius_m > 0.0 && std::hypot(wx - px, wy - py) > sample_radius_m) {
          continue;
        }

        const std::size_t index =
          static_cast<std::size_t>(my) * static_cast<std::size_t>(info.width) +
          static_cast<std::size_t>(mx);

        if (index >= costmap.data.size()) {
          continue;
        }

        if (static_cast<int>(costmap.data[index]) >= lethal_threshold) {
          this_pose_blocked = true;
          if (seen_indices.insert(index).second) {
            geometry_msgs::msg::Point pt;
            pt.x = wx;
            pt.y = wy;
            pt.z = 0.0;
            all_blocked_points.push_back(pt);
          }
        }
      }
    }

    metrics.total_poses++;

    if (this_pose_blocked) {
      metrics.blocked_poses++;
      if (!run_in_progress) {
        run_start_dist = travelled_m;
        run_in_progress = true;
      }
    } else {
      if (run_in_progress) {
        const double run_len = travelled_m - run_start_dist;
        metrics.max_run_length_m = std::max(metrics.max_run_length_m, run_len);
        run_in_progress = false;
      }
    }
  }

  // Close a run that extends to the end of the lookahead window.
  if (run_in_progress) {
    metrics.max_run_length_m =
      std::max(metrics.max_run_length_m, travelled_m - run_start_dist);
  }

  if (all_blocked_points.empty()) {
    return metrics;
  }

  metrics.any_blocked = true;
  metrics.blocked_fraction = metrics.total_poses > 0
    ? static_cast<double>(metrics.blocked_poses) / static_cast<double>(metrics.total_poses)
    : 0.0;

  // Centroid of all blocked cells.
  double sx = 0.0;
  double sy = 0.0;
  for (const auto & p : all_blocked_points) {
    sx += p.x;
    sy += p.y;
  }
  metrics.centroid.x = sx / static_cast<double>(all_blocked_points.size());
  metrics.centroid.y = sy / static_cast<double>(all_blocked_points.size());
  metrics.centroid.z = 0.0;

  // Extent: approximate blocked-region diameter.
  double max_radius = 0.0;
  for (const auto & p : all_blocked_points) {
    max_radius = std::max(
      max_radius,
      std::hypot(p.x - metrics.centroid.x, p.y - metrics.centroid.y));
  }
  metrics.extent_m = static_cast<float>((2.0 * max_radius) + resolution);

  return metrics;
}

BT::PortsList PathClearCondition::providedPorts()
{
  return {
    BT::InputPort<nav_msgs::msg::Path>(
      "path",
      "BT blackboard path from ComputePathToPose; preferred over /plan topic"),
    BT::InputPort<double>(
      "lookahead_m",
      1.5,
      "Metres of active path to inspect"),
    BT::InputPort<int>(
      "lethal_threshold",
      90,
      "OccupancyGrid value treated as lethal"),
    BT::InputPort<int>(
      "debounce_ticks",
      2,
      "Consecutive significant-blockage ticks required before FAILURE"),
    BT::InputPort<double>(
      "sample_radius_m",
      0.05,
      "Radius around each path pose sampled for lethal cells; 0.0 checks the containing cell only"),
    BT::InputPort<double>(
      "min_blocked_length_m",
      0.45,
      "Minimum continuous blocked path length (m) before semantic recovery triggers"),
    BT::InputPort<int>(
      "min_blocked_samples",
      4,
      "Minimum number of blocked path poses before semantic recovery triggers"),
    BT::InputPort<double>(
      "blocked_fraction_threshold",
      0.30,
      "Minimum fraction of checked path poses that must be blocked"),
    BT::InputPort<bool>(
      "allow_geometric_detour_first",
      true,
      "If true, minor blockages return SUCCESS and let Nav2 replan around them"),
    BT::InputPort<std::string>(
      "plan_topic",
      "/plan",
      "Fallback nav_msgs/Path topic"),
    BT::InputPort<std::string>(
      "local_costmap_topic",
      "/local_costmap/costmap",
      "nav_msgs/OccupancyGrid topic"),
    BT::OutputPort<geometry_msgs::msg::Point>(
      "blockage_centroid",
      "Average world position of lethal sampled cells"),
    BT::OutputPort<float>(
      "blockage_extent_m",
      "Approximate blocked-region diameter in metres"),
  };
}

}  // namespace semantic_nav_nav2_plugins
