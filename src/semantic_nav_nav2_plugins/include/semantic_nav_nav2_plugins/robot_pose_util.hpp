// Copyright 2026 Md Shaker Ibna Kamal. Apache-2.0.
#pragma once

#include <memory>
#include <string>

#include "behaviortree_cpp_v3/bt_factory.h"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "nav2_util/robot_utils.hpp"
#include "tf2_ros/buffer.h"

namespace semantic_nav_nav2_plugins
{

/**
 * @brief Read the robot's CURRENT pose from TF via bt_navigator's blackboard tf_buffer.
 *
 * Recovery context must be gathered where the robot actually is, not where it was
 * trying to go. There is deliberately no fall-back to the "goal" blackboard key:
 * NavigateToPose never publishes a "robot_pose" key, so such a fall-back silently
 * centres every semantic query on the destination and the responsible-object match
 * then looks several metres away from the real blocker.
 *
 * @return false if TF cannot supply the pose; callers must treat that as a failure
 *         rather than substituting some other pose.
 */
inline bool readCurrentRobotPose(
  const BT::NodeConfiguration & conf,
  const std::string & global_frame,
  const std::string & robot_base_frame,
  double transform_tolerance_s,
  geometry_msgs::msg::PoseStamped & pose_out)
{
  if (!conf.blackboard) {
    return false;
  }

  std::shared_ptr<tf2_ros::Buffer> tf_buffer;
  if (!conf.blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", tf_buffer) ||
    !tf_buffer)
  {
    return false;
  }

  return nav2_util::getCurrentPose(
    pose_out,
    *tf_buffer,
    global_frame,
    robot_base_frame,
    transform_tolerance_s);
}

}  // namespace semantic_nav_nav2_plugins
