#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from apriltag_msgs.msg import AprilTagDetectionArray
from geometry_msgs.msg import PoseStamped
import tf2_ros
from rclpy.time import Time


class DockPoseBridge(Node):
    def __init__(self):
        super().__init__('dock_pose_bridge')

        self.declare_parameter('camera_frame', 'camera_color_optical_frame')
        self.declare_parameter('tag_frame', 'dock_frame')

        self.camera_frame = self.get_parameter('camera_frame').value
        self.tag_frame = self.get_parameter('tag_frame').value

        self.detection_subscription = self.create_subscription(
            AprilTagDetectionArray,
            '/detections',
            self.detection_callback,
            10)

        self.publisher = self.create_publisher(PoseStamped, '/detected_dock_pose', 10)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

    def detection_callback(self, msg):
        # Request the LATEST available transform rather than the exact
        # detection message stamp. Insisting on an exact past stamp is
        # fragile if apriltag_node's detection timestamp and the TF
        # broadcast timestamp aren't in perfect lockstep (clock sync,
        # processing delay, etc). tf2_echo's default behavior (which
        # works) does exactly this: ask for latest, not a pinned instant.
        lookup_time = rclpy.time.Time()

        if not self.tfBuffer.can_transform(
                self.camera_frame, self.tag_frame, lookup_time,
                timeout=rclpy.duration.Duration(seconds=0.2)):
            self.get_logger().warn(
                f"TF not ready yet: {self.camera_frame} -> {self.tag_frame}")
            return

        try:
            transform = self.tfBuffer.lookup_transform(
                self.camera_frame, self.tag_frame, lookup_time)
        except tf2_ros.TransformException as ex:
            self.get_logger().warn(f"TF lookup failed: {ex}")
            return

        pose = PoseStamped()
        pose.header.frame_id = self.camera_frame
        # CRITICAL: stamp with the transform's own time, not now().
        # docking_server checks (node.now() - stamp) against
        # external_detection_timeout, and also needs this exact stamp
        # to resolve camera_frame -> fixed_frame (odom) in its own TF buffer.
        pose.header.stamp = transform.header.stamp
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.publisher.publish(pose)
        print("published pose")


def main(args=None):
    rclpy.init(args=args)
    node = DockPoseBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()