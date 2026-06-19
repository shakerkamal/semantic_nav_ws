// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#include "semantic_nav_nav2_plugins/capture_blockage_context.hpp"

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
  if (!getInput<nav_msgs::msg::Path>("path", path) || path.poses.empty()) {
    return BT::NodeStatus::SUCCESS;
  }

  nav_msgs::msg::OccupancyGrid::SharedPtr costmap;
  {
    std::lock_guard<std::mutex> lock(data_mutex_);
    costmap = latest_costmap_;
  }

  if (!costmap) {
    return BT::NodeStatus::SUCCESS;
  }

  double lookahead_m{3.0};
  int lethal_threshold{90};
  double sample_radius_m{0.05};
  getInput("lookahead_m", lookahead_m);
  getInput("lethal_threshold", lethal_threshold);
  getInput("sample_radius_m", sample_radius_m);

  const BlockageMetrics metrics = PathClearCondition::isCorridorBlocked(
    path, *costmap, lethal_threshold, lookahead_m, sample_radius_m);

  if (metrics.any_blocked) {
    setOutput("blockage_centroid", metrics.centroid);
    setOutput("blockage_extent_m", metrics.extent_m);

    RCLCPP_DEBUG(
      node_->get_logger(),
      "[CaptureBlockageContext] centroid=(%.3f,%.3f) extent=%.3fm"
      " poses=%d/%d fraction=%.2f",
      metrics.centroid.x, metrics.centroid.y, metrics.extent_m,
      metrics.blocked_poses, metrics.total_poses, metrics.blocked_fraction);
  }

  return BT::NodeStatus::SUCCESS;
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
    BT::OutputPort<geometry_msgs::msg::Point>(
      "blockage_centroid",
      "Centroid of blocked costmap cells"),
    BT::OutputPort<float>(
      "blockage_extent_m",
      "Approximate blocked-region diameter in metres"),
  };
}

}  // namespace semantic_nav_nav2_plugins
