#!/usr/bin/env python
from skiros2_msgs.action import TestAction
import threading
import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node

# Based on: https://github.com/ros2/examples/blob/iron/rclpy/actions/minimal_action_server/examples_rclpy_minimal_action_server/server_single_goal.py

class TestActionServer(Node):
    """Minimal action server that processes one goal at a time."""

    def __init__(self):
        super().__init__('minimal_action_server')
        self._goal_handle = None
        self._goal_lock = threading.Lock()
        self._action_server = ActionServer(
            self,
            TestAction,
            'test_action',
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def goal_callback(self, goal_request):
        """Accept or reject a client request to begin an action."""
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle):
        with self._goal_lock:
            # This server only allows one goal at a time
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                # Abort the existing goal
                self._goal_handle.abort()
            self._goal_handle = goal_handle

        goal_handle.execute()

    def cancel_callback(self, goal):
        """Accept or reject a client request to cancel an action."""
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """Execute the goal."""
        self.get_logger().info('Executing goal...')

        # Append the seeds for the Fibonacci sequence
        feedback_msg = TestAction.Feedback()
        feedback_msg.progress = 0

        # Start executing the action
        for i in range(goal_handle.request.ticks):
            # If goal is flagged as no longer active (ie. another goal was accepted),
            # then stop executing
            if not goal_handle.is_active:
                self.get_logger().info('Goal aborted')
                return TestAction.Result()

            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info('Goal canceled')
                return TestAction.Result()

            # Update Fibonacci sequence
            feedback_msg.progress = i
            #feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i-1])

            self.get_logger().info('Publishing feedback: {0}'.format(feedback_msg.progress))

            # Publish the feedback
            goal_handle.publish_feedback(feedback_msg)

            # Sleep for demonstration purposes
            time.sleep(1)

        goal_handle.succeed()

        # Populate result message
        result = TestAction.Result()
        result.result = goal_handle.request.ticks

        self.get_logger().info('Returning result: {0}'.format(result.result))

        return result


def main(args=None):
    rclpy.init(args=args)

    action_server = TestActionServer()

    # We use a MultiThreadedExecutor to handle incoming goal requests concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(action_server, executor=executor)

    action_server.destroy()
    rclpy.shutdown()


if __name__ == '__main__':
    main()