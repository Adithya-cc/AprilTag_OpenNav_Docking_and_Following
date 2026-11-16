#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from opennav_following_msgs.action import FollowObject


class FollowObjectClient(Node):

    def __init__(self):
        super().__init__("follow_object_client")

        self._client = ActionClient(
            self,
            FollowObject,
            "/follow_object"
        )

    def send_goal(self):
        self.get_logger().info("Waiting for FollowObject action server...")
        self._client.wait_for_server()

        goal = FollowObject.Goal()
        goal.pose_topic = "/detected_person"
        goal.tracked_frame = ""          # Leave empty when using pose_topic
        goal.max_duration.sec = -1       # Run indefinitely
        goal.max_duration.nanosec = 0

        self.get_logger().info("Sending goal...")

        future = self._client.send_goal_async(
            goal,
            feedback_callback=self.feedback_callback
        )
        future.add_done_callback(self.goal_response_callback)
        self.send_goal()
        
    def goal_response_callback(self, future):
        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted")

        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

        states = {
            FollowObject.Feedback.NONE: "NONE",
            FollowObject.Feedback.INITIAL_PERCEPTION: "INITIAL_PERCEPTION",
            FollowObject.Feedback.CONTROLLING: "CONTROLLING",
            FollowObject.Feedback.STOPPING: "STOPPING",
            FollowObject.Feedback.RETRY: "RETRY",
        }

        state_name = states.get(feedback.state, "UNKNOWN")

        self.get_logger().info(
            f"State: {state_name}, "
            f"Following time: {feedback.following_time.sec}s, "
            f"Retries: {feedback.num_retries}"
        )

    def result_callback(self, future):
        result = future.result().result

        self.get_logger().info("Goal completed")
        self.get_logger().info(
            f"Elapsed time: {result.total_elapsed_time.sec}s"
        )
        self.get_logger().info(
            f"Retries: {result.num_retries}"
        )

        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)

    node = FollowObjectClient()
    rclpy.spin(node)


if __name__ == "__main__":
    main()