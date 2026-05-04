import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient, CancelResponse, GoalResponse
from action_msgs.msg import GoalStatus

from nav2_msgs.action import NavigateToPose
from semantic_nav_interfaces.action import ExecutePose


class SemanticNavExecutor(Node):
    def __init__(self):
        super().__init__('semantic_nav_executor')

        self._nav2_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._server = ActionServer(
            self,
            ExecutePose,
            'execute_pose',
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback,
        )

        self._active_nav2_goal_handle = None
        self._lock = threading.Lock()

    def goal_callback(self, goal_request: ExecutePose.Goal):
        frame_id = goal_request.pose.header.frame_id
        if frame_id != 'map':
            self.get_logger().warn(f"Rejected goal with frame_id='{frame_id}', expected 'map'")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, _cancel_request):
        with self._lock:
            if self._active_nav2_goal_handle is not None:
                self._active_nav2_goal_handle.cancel_goal_async()
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        if not self._nav2_client.wait_for_server(timeout_sec=10.0):
            goal_handle.abort()
            result = ExecutePose.Result()
            result.success = False
            result.message = 'Nav2 navigate_to_pose action server not available'
            return result

        nav2_goal = NavigateToPose.Goal()
        nav2_goal.pose = goal_handle.request.pose
        nav2_goal.behavior_tree = goal_handle.request.behavior_tree

        if nav2_goal.pose.header.stamp.sec == 0 and nav2_goal.pose.header.stamp.nanosec == 0:
            nav2_goal.pose.header.stamp = self.get_clock().now().to_msg()

        send_goal_future = self._nav2_client.send_goal_async(
            nav2_goal,
            feedback_callback=lambda msg: self._forward_feedback(goal_handle, msg),
        )
        rclpy.spin_until_future_complete(self, send_goal_future)

        nav2_goal_handle = send_goal_future.result()
        if nav2_goal_handle is None or not nav2_goal_handle.accepted:
            goal_handle.abort()
            result = ExecutePose.Result()
            result.success = False
            result.message = 'Nav2 rejected the goal'
            return result

        with self._lock:
            self._active_nav2_goal_handle = nav2_goal_handle

        result_future = nav2_goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        with self._lock:
            self._active_nav2_goal_handle = None

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            result = ExecutePose.Result()
            result.success = False
            result.message = 'Goal canceled'
            return result

        nav2_result = result_future.result()
        if nav2_result is None:
            goal_handle.abort()
            result = ExecutePose.Result()
            result.success = False
            result.message = 'No result returned from Nav2'
            return result

        status = nav2_result.status
        self.get_logger().info(f"Nav2 result status: {status}")

        if status == GoalStatus.STATUS_ABORTED:  # STATUS_ABORTED
            goal_handle.abort()
            result = ExecutePose.Result()
            result.success = False
            result.message = 'Navigation aborted by Nav2'
            return result

        elif status == GoalStatus.STATUS_CANCELED:  # STATUS_CANCELED
            goal_handle.canceled()
            result = ExecutePose.Result()
            result.success = False
            result.message = 'Navigation canceled by Nav2'
            return result
        
        elif status == GoalStatus.STATUS_SUCCEEDED:  # STATUS_SUCCEEDED
            goal_handle.succeed()
            result = ExecutePose.Result()
            result.success = True
            result.message = 'Navigation succeeded'
            return result
        
        else:
            goal_handle.abort()
            result = ExecutePose.Result()
            result.success = False
            result.message = f'Unexpected status code: {status}'
            return result

    def _forward_feedback(self, goal_handle, feedback_msg):
        fb = feedback_msg.feedback
        feedback = ExecutePose.Feedback()
        feedback.current_pose = fb.current_pose
        feedback.navigation_time = fb.navigation_time
        feedback.estimated_time_remaining = fb.estimated_time_remaining
        feedback.number_of_recoveries = fb.number_of_recoveries
        feedback.distance_remaining = fb.distance_remaining
        goal_handle.publish_feedback(feedback)


def main(args=None):
    rclpy.init(args=args)
    node = SemanticNavExecutor()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()