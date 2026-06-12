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

  // No data yet. Treat as clear so startup timing does not trigger false recovery.
  if (path.poses.empty() || !costmap) {
    blocked_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }

  int lethal_threshold{90};
  double lookahead_m{1.5};
  double sample_radius_m{0.05};
  int debounce_ticks{2};

  getInput("lethal_threshold", lethal_threshold);
  getInput("lookahead_m", lookahead_m);
  getInput("sample_radius_m", sample_radius_m);
  getInput("debounce_ticks", debounce_ticks);

  debounce_ticks = std::max(1, debounce_ticks);
  sample_radius_m = std::max(0.0, sample_radius_m);

  geometry_msgs::msg::Point centroid;
  float extent{0.0f};

  const bool blocked = isCorridorBlocked(
    path,
    *costmap,
    lethal_threshold,
    lookahead_m,
    sample_radius_m,
    centroid,
    extent);

  if (!blocked) {
    blocked_count_ = 0;
    return BT::NodeStatus::SUCCESS;
  }

  last_centroid_ = centroid;
  last_extent_ = extent;
  blocked_count_ = std::min(blocked_count_ + 1, debounce_ticks + 1);

  if (blocked_count_ < debounce_ticks) {
    return BT::NodeStatus::SUCCESS;
  }

  setOutput("blockage_centroid", last_centroid_);
  setOutput("blockage_extent_m", last_extent_);

  RCLCPP_DEBUG(
    node_->get_logger(),
    "[PathClearCondition] blocked centroid=(%.3f, %.3f) extent=%.3f ticks=%d",
    last_centroid_.x,
    last_centroid_.y,
    last_extent_,
    blocked_count_);

  return BT::NodeStatus::FAILURE;
}

bool PathClearCondition::isCorridorBlocked(
  const nav_msgs::msg::Path & path,
  const nav_msgs::msg::OccupancyGrid & costmap,
  int lethal_threshold,
  double lookahead_m,
  double sample_radius_m,
  geometry_msgs::msg::Point & centroid_out,
  float & extent_out)
{
  centroid_out = geometry_msgs::msg::Point{};
  extent_out = 0.0f;

  if (path.poses.empty()) {
    return false;
  }

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
    0,
    static_cast<int>(std::ceil(sample_radius_m / resolution)));

  std::vector<geometry_msgs::msg::Point> blocked_points;
  std::unordered_set<std::size_t> seen_indices;

  double travelled_m = 0.0;
  double prev_x = path.poses.front().pose.position.x;
  double prev_y = path.poses.front().pose.position.y;

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

        if (std::hypot(wx - px, wy - py) > sample_radius_m) {
          continue;
        }

        const std::size_t index =
          static_cast<std::size_t>(my) * static_cast<std::size_t>(info.width) +
          static_cast<std::size_t>(mx);

        if (index >= costmap.data.size()) {
          continue;
        }

        const auto raw_cost = costmap.data[index];

        if (static_cast<int>(raw_cost) >= lethal_threshold) {
          if (seen_indices.insert(index).second) {
            geometry_msgs::msg::Point pt;
            pt.x = wx;
            pt.y = wy;
            pt.z = 0.0;
            blocked_points.push_back(pt);
          }
        }
      }
    }
  }

  if (blocked_points.empty()) {
    return false;
  }

  double sx = 0.0;
  double sy = 0.0;

  for (const auto & p : blocked_points) {
    sx += p.x;
    sy += p.y;
  }

  centroid_out.x = sx / static_cast<double>(blocked_points.size());
  centroid_out.y = sy / static_cast<double>(blocked_points.size());
  centroid_out.z = 0.0;

  double max_radius = 0.0;
  for (const auto & p : blocked_points) {
    max_radius = std::max(
      max_radius,
      std::hypot(p.x - centroid_out.x, p.y - centroid_out.y));
  }

  extent_out = static_cast<float>((2.0 * max_radius) + resolution);
  return true;
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
      "Consecutive blocked ticks required before FAILURE"),
    BT::InputPort<double>(
      "sample_radius_m",
      0.05,
      "Radius around each path pose sampled for lethal cells"),
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