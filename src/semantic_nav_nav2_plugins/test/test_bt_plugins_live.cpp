// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
//
// Live-delivery tests: the host node the plugins hang off is deliberately
// NEVER spun by the test. This mirrors production exactly — nav2's
// BtActionServer creates client_node_ and hands it to the blackboard without
// adding it to any executor, so a plugin only receives topic/service data it
// spins for itself (the QuerySemanticContext callback-group pattern).
#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <thread>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/point.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include "rclcpp/rclcpp.hpp"
#include "semantic_nav_interfaces/srv/operator_decision.hpp"
#include "semantic_nav_nav2_plugins/capture_blockage_context.hpp"
#include "semantic_nav_nav2_plugins/operator_prompt.hpp"
#include "semantic_nav_nav2_plugins/path_clear_condition.hpp"

using namespace std::chrono_literals;

namespace
{

nav_msgs::msg::OccupancyGrid makeLethalCostmap()
{
  nav_msgs::msg::OccupancyGrid grid;
  grid.header.frame_id = "map";
  grid.info.width = 40;
  grid.info.height = 40;
  grid.info.resolution = 0.05f;
  grid.info.origin.position.x = 0.0;
  grid.info.origin.position.y = 0.0;
  grid.data.assign(40 * 40, 100);
  return grid;
}

nav_msgs::msg::Path makePathAcrossGrid()
{
  nav_msgs::msg::Path path;
  path.header.frame_id = "map";
  for (int i = 0; i < 10; ++i) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.pose.position.x = 0.2 + 0.1 * static_cast<double>(i);
    pose.pose.position.y = 1.0;
    path.poses.push_back(pose);
  }
  return path;
}

}  // namespace

TEST(PathClearConditionLiveTest, seesPublishedCostmapWithoutHostSpin)
{
  auto host = std::make_shared<rclcpp::Node>("pcc_live_host");
  auto pub_node = std::make_shared<rclcpp::Node>("pcc_live_pub");
  auto costmap_pub = pub_node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", rclcpp::SystemDefaultsQoS());

  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", host);
  blackboard->set<nav_msgs::msg::Path>("path", makePathAcrossGrid());

  BT::NodeConfiguration conf;
  conf.blackboard = blackboard;
  conf.input_ports["path"] = "{path}";
  conf.input_ports["debounce_ticks"] = "1";
  conf.input_ports["allow_geometric_detour_first"] = "false";
  conf.output_ports["blockage_centroid"] = "{blockage_centroid}";
  conf.output_ports["blockage_extent_m"] = "{blockage_extent_m}";

  semantic_nav_nav2_plugins::PathClearCondition cond("path_clear", conf);

  const auto grid = makeLethalCostmap();
  BT::NodeStatus status = BT::NodeStatus::SUCCESS;
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    costmap_pub->publish(grid);
    std::this_thread::sleep_for(50ms);
    status = cond.executeTick();
    if (status == BT::NodeStatus::FAILURE) {
      break;
    }
  }

  // Fully lethal corridor + debounce 1 + severity gate off: the ONLY way this
  // stays SUCCESS is the subscription never being serviced.
  EXPECT_EQ(status, BT::NodeStatus::FAILURE);
}

TEST(CaptureBlockageContextLiveTest, measuredCentroidFromPublishedCostmapWithoutHostSpin)
{
  auto host = std::make_shared<rclcpp::Node>("cbc_live_host");
  auto pub_node = std::make_shared<rclcpp::Node>("cbc_live_pub");
  auto costmap_pub = pub_node->create_publisher<nav_msgs::msg::OccupancyGrid>(
    "/local_costmap/costmap", rclcpp::SystemDefaultsQoS());

  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", host);
  blackboard->set<nav_msgs::msg::Path>("path", makePathAcrossGrid());
  // No tf_buffer on the blackboard: every fallback tier bails out, so the
  // centroid can ONLY be written by the source=measured tier, which needs the
  // published costmap to have been received.
  BT::NodeConfiguration conf;
  conf.blackboard = blackboard;
  conf.input_ports["path"] = "{path}";
  conf.output_ports["blockage_centroid"] = "{blockage_centroid}";
  conf.output_ports["blockage_extent_m"] = "{blockage_extent_m}";

  semantic_nav_nav2_plugins::CaptureBlockageContext capture("capture", conf);

  const auto grid = makeLethalCostmap();
  geometry_msgs::msg::Point centroid;
  bool centroid_set = false;
  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (std::chrono::steady_clock::now() < deadline) {
    costmap_pub->publish(grid);
    std::this_thread::sleep_for(50ms);
    capture.executeTick();
    if (blackboard->get<geometry_msgs::msg::Point>("blockage_centroid", centroid)) {
      centroid_set = true;
      break;
    }
  }

  ASSERT_TRUE(centroid_set);
  // Measured centroid of the blocked poses lies on the path row.
  EXPECT_NEAR(centroid.y, 1.0, 0.2);
  EXPECT_GT(centroid.x, 0.1);
  EXPECT_LT(centroid.x, 1.3);
}

TEST(OperatorPromptLiveTest, resolvesAcknowledgedResponseWithoutHostSpin)
{
  using ServiceT = semantic_nav_interfaces::srv::OperatorDecision;

  auto host = std::make_shared<rclcpp::Node>("op_live_host");
  auto server_node = std::make_shared<rclcpp::Node>("op_live_server");
  auto server = server_node->create_service<ServiceT>(
    "/operator_decision",
    [](const std::shared_ptr<ServiceT::Request>,
    std::shared_ptr<ServiceT::Response> response) {
      response->acknowledged = true;
      response->operator_note = "operator_confirmed";
    });

  rclcpp::executors::SingleThreadedExecutor server_exec;
  server_exec.add_node(server_node);
  std::thread server_thread([&server_exec]() {server_exec.spin();});

  auto blackboard = BT::Blackboard::create();
  blackboard->set<rclcpp::Node::SharedPtr>("node", host);

  BT::NodeConfiguration conf;
  conf.blackboard = blackboard;
  conf.input_ports["prompt_text"] = "open the door";
  conf.input_ports["responsible_object_key"] = "door:903";
  conf.input_ports["response_timeout_ms"] = "2500";

  semantic_nav_nav2_plugins::OperatorPrompt prompt("operator_prompt", conf);

  BT::NodeStatus status = BT::NodeStatus::RUNNING;
  const auto deadline = std::chrono::steady_clock::now() + 6s;
  while (std::chrono::steady_clock::now() < deadline) {
    status = prompt.executeTick();
    if (status != BT::NodeStatus::RUNNING) {
      break;
    }
    std::this_thread::sleep_for(20ms);
  }

  server_exec.cancel();
  server_thread.join();

  // The server answered acknowledged=true; the only way to end up FAILURE is
  // the client future never resolving because nothing spins the host node.
  EXPECT_EQ(status, BT::NodeStatus::SUCCESS);
}

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  testing::InitGoogleTest(&argc, argv);
  const int result = RUN_ALL_TESTS();
  rclcpp::shutdown();
  return result;
}
