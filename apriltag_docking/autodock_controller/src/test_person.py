#! /usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from cv_bridge import CvBridge
import cv2
from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2
from sensor_msgs.msg import Image
import mediapipe as mp
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from mediapipe.tasks import python
from mediapipe.tasks.python import vision
import numpy as np
from visualization_msgs.msg import Marker


# ── Stable upper-body landmark indices (MediaPipe Pose) ──────────────────────
# Using only shoulders (11,12) and hips (23,24).
# These are the landmarks most likely to be visible and inside the frame
# regardless of how close the person is to the camera.
# Avoid: feet (27-32), knees (25,26), wrists/elbows (13-16) — all frequently
# out-of-frame or occluded, which drags the centroid toward the image edges.
TORSO_LANDMARK_IDS = [11, 12, 23, 24]   # left_shoulder, right_shoulder, left_hip, right_hip

# Depth averaging kernel half-size (pixels).
# We sample a small square neighbourhood around the centroid pixel instead of
# a single point, which dramatically reduces noise from NaN holes in the cloud.
DEPTH_KERNEL_HALF = 8   # => 17×17 pixel region


class PersonFollower(Node):
    def __init__(self):
        super().__init__("person_follower")
        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image, "/camera_depth_frame/image_raw", self.callback, 10)
        self.depth_sub = self.create_subscription(
            PointCloud2, "/camera_depth_frame/points", self.depth_callback, 10)

        self.pose = self.create_publisher(PoseStamped, "/detecteds_person", 10)
        self.velocity_publisher = self.create_publisher(Twist, "/dummy/cmd_vel", 10)
        self.marker_pub = self.create_publisher(Marker, "/person_marker", 10)

        # State
        self.last_error = 0.0
        self.last_depth = 0.0
        self.depth_image = None
        self.person_detected = False
        self.depth_mm = 0.0
        self.velocity_msg = Twist()

        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_holistic = mp.solutions.holistic
        self.mp_pose = mp.solutions.pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        self.tf_broadcaster = TransformBroadcaster(self)

        self.kp_angular = 0.005
        self.kp_linear = 0.7
        self.x_center = None
        self.y_center = None
        self.image_center = None

        base_options = python.BaseOptions(
            model_asset_path='/home/adithya-si3510/autodock_ros2_ws/src/apriltag_docking/'
                             'autodock_controller/param/deeplabv3.tflite')
        options = vision.ImageSegmenterOptions(
            base_options=base_options, output_category_mask=True)
        self.segmenter = vision.ImageSegmenter.create_from_options(options)

        self.BG_COLOR = (192, 192, 192)
        self.MASK_COLOR = (0, 255, 0)

    # ─────────────────────────────────────────────────────────────────────────
    # Depth callback
    # ─────────────────────────────────────────────────────────────────────────
    def depth_callback(self, data):
        try:
            self.depth_image = data
        except Exception as e:
            self.get_logger().error(f"Error in depth callback: {e}")

    # ─────────────────────────────────────────────────────────────────────────
    # FIX 1 — compute centroid from stable TORSO landmarks only
    # ─────────────────────────────────────────────────────────────────────────
    def _torso_centroid(self, landmarks, img_w, img_h):
        """
        Return (x_px, y_px) centroid using only the 4 torso landmarks.

        Why this works:
          - Shoulders and hips are nearly always inside the camera frame even
            when the person is very close.
          - Feet / ankles are clipped off-screen at close range, causing the
            naive all-landmark centroid to drift toward the TOP of the image
            (low pixel-y) — exactly the bug you observed.
          - Constraining to torso landmarks keeps the centroid in the middle of
            the visible upper body regardless of distance.
        """
        xs, ys = [], []
        for idx in TORSO_LANDMARK_IDS:
            lm = landmarks[idx]
            # MediaPipe normalises to [0,1]; visibility > 0.5 means reliable
            if lm.visibility > 0.5:
                xs.append(lm.x * img_w)
                ys.append(lm.y * img_h)

        if not xs:
            # Fallback: use all visible landmarks rather than crashing
            for lm in landmarks:
                if lm.visibility > 0.5:
                    xs.append(lm.x * img_w)
                    ys.append(lm.y * img_h)

        if not xs:
            return img_w / 2.0, img_h / 2.0  # last resort default

        return float(np.mean(xs)), float(np.mean(ys))

    # ─────────────────────────────────────────────────────────────────────────
    # FIX 2 — average depth over a small region instead of one pixel
    # ─────────────────────────────────────────────────────────────────────────
    def _sample_depth_region(self, cx, cy):
        """
        Sample a DEPTH_KERNEL_HALF-radius square around (cx, cy) in the point
        cloud and return the median (x, y, z) of valid points.

        Why this works:
          - A single pixel UV lookup often lands on a NaN hole (reflective
            surface, mixed-depth edge) and read_points_list silently skips it,
            returning whichever valid point happens to be first — often a stray
            point near the image corner.
          - Sampling a 17×17 neighbourhood and taking the median is robust to
            scattered NaN holes and gives a stable 3-D position.
        """
        w = self.depth_image.width
        h = self.depth_image.height

        # Build flat row-major indices for the neighbourhood.
        # ROS2 Humble's read_points_list expects uvs as a 1-D sequence of
        # integers (flat indices = row * width + col), NOT (u, v) tuples.
        flat_indices = []
        for dv in range(-DEPTH_KERNEL_HALF, DEPTH_KERNEL_HALF + 1):
            for du in range(-DEPTH_KERNEL_HALF, DEPTH_KERNEL_HALF + 1):
                u = int(np.clip(cx + du, 0, w - 1))
                v = int(np.clip(cy + dv, 0, h - 1))
                flat_indices.append(int(v * w + u))

        # Deduplicate (edge clamping creates duplicates)
        flat_indices = list(set(flat_indices))

        pts = point_cloud2.read_points_list(
            self.depth_image,
            field_names=['x', 'y', 'z'],
            skip_nans=True,
            uvs=flat_indices
        )

        if not pts:
            return None

        arr = np.array([[p[0], p[1], p[2]] for p in pts], dtype=np.float32)

        # Remove outliers with a simple Z-range filter (optional but helpful)
        z_vals = arr[:, 2]
        z_med = np.median(z_vals)
        mask = np.abs(z_vals - z_med) < 2.0   # keep pts within 2 m of median
        arr = arr[mask]

        if len(arr) == 0:
            return None

        return float(np.median(arr[:, 0])), \
               float(np.median(arr[:, 1])), \
               float(np.median(arr[:, 2]))

    # ─────────────────────────────────────────────────────────────────────────
    def publish_person_marker(self, x, y, z):
        marker = Marker()
        marker.header.frame_id = "camera_depth_optical_frame"
        marker.header.stamp = self.depth_image.header.stamp
        marker.ns = "person"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.3
        marker.scale.y = 0.3
        marker.scale.z = 0.3
        marker.color.r = 1.0
        marker.color.a = 1.0
        marker.lifetime.sec = 1
        self.marker_pub.publish(marker)

    # ─────────────────────────────────────────────────────────────────────────
    def callback(self, data):
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        rgb_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)
        self.segmentation_frame = self.cv_image.copy()
        self.results = self.mp_pose.process(rgb_image)

        if self.results.pose_landmarks is None:
            print("No Human")
            if self.person_detected:
                self.vel_control(0.0, -0.5)
            else:
                self.vel_control(0.0, 0.0)
            return

        self.person_detected = True
        landmarks = self.results.pose_landmarks.landmark
        img_h, img_w = self.cv_image.shape[:2]
        self.image_center = img_w / 2.0

        # ── FIX 1: torso centroid ──────────────────────────────────────────
        self.x_center, self.y_center = self._torso_centroid(landmarks, img_w, img_h)

        self.get_logger().info(
            f"torso centroid  x_px={self.x_center:.1f}  y_px={self.y_center:.1f}")

        # Clamp to image bounds
        self.x_center = float(np.clip(self.x_center, 0, img_w - 1))
        self.y_center = float(np.clip(self.y_center, 0, img_h - 1))

        # Bounding box from all landmarks (for display only)
        x_min = min(lm.x for lm in landmarks)
        x_max = max(lm.x for lm in landmarks)
        y_min = min(lm.y for lm in landmarks)
        y_max = max(lm.y for lm in landmarks)
        cv2.rectangle(
            self.cv_image,
            (int(x_min * img_w), int(y_min * img_h)),
            (int(x_max * img_w), int(y_max * img_h)),
            (0, 255, 0), 2)

        # Segmentation
        seg_mp = mp.Image(image_format=mp.ImageFormat.SRGB,
                          data=self.segmentation_frame)
        seg_result = self.segmenter.segment(seg_mp)
        category_mask = seg_result.category_mask
        image_data = seg_mp.numpy_view()
        fg = np.full(image_data.shape, self.MASK_COLOR, dtype=np.uint8)
        bg = np.full(image_data.shape, self.BG_COLOR,  dtype=np.uint8)
        condition = np.stack((category_mask.numpy_view(),) * 3, axis=-1) > 0.2
        self.segmentation_frame = np.where(condition, fg, bg)
        self.mp_drawing.draw_landmarks(
            self.segmentation_frame,
            self.results.pose_landmarks,
            self.mp_holistic.POSE_CONNECTIONS)

        # Crosshair overlays
        cx, cy = int(self.x_center), int(self.y_center)
        for frame in (self.cv_image, self.segmentation_frame):
            cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (255, 0, 0), 3)
            cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (255, 0, 0), 3)
        cv2.line(self.segmentation_frame, (img_w // 2, 0), (img_w // 2, img_h), (0, 0, 255), 2)
        cv2.line(self.segmentation_frame, (0, cy), (img_w, cy), (0, 0, 255), 2)

        # ── FIX 2: robust depth sampling ──────────────────────────────────
        if self.depth_image is None:
            self.vel_control(0.0, 0.0)
            return

        result = self._sample_depth_region(self.x_center, self.y_center)
        if result is None:
            self.get_logger().warn("No valid depth points near centroid")
            self.vel_control(0.0, 0.0)
            return

        x_3d, y_3d, z_3d = result
        self.depth_mm = z_3d   # z is forward distance in metres for RealSense

        self.get_logger().info(
            f"3-D pose  x={x_3d:.3f}  y={y_3d:.3f}  z={z_3d:.3f}")

        self.publish_person_marker(x_3d, y_3d, z_3d)

        # Publish PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header.frame_id = "camera_depth_frame"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.pose.position.x = x_3d
        pose_msg.pose.position.y = y_3d
        pose_msg.pose.position.z = z_3d
        pose_msg.pose.orientation.w = 1.0
        self.pose.publish(pose_msg)

        # Broadcast TF
        t = TransformStamped()
        t.header.stamp = self.depth_image.header.stamp
        t.header.frame_id = self.depth_image.header.frame_id
        t.child_frame_id = 'detected_person'
        t.transform.translation.x = x_3d
        t.transform.translation.y = y_3d
        t.transform.translation.z = z_3d
        t.transform.rotation.x = -0.5
        t.transform.rotation.y =  0.5
        t.transform.rotation.z = -0.5
        t.transform.rotation.w =  0.5
        self.tf_broadcaster.sendTransform(t)

        self.draw_landmarks_and_box()
        self.move_robot()

    # ─────────────────────────────────────────────────────────────────────────
    def draw_landmarks_and_box(self):
        img_h, img_w = self.cv_image.shape[:2]
        cv2.circle(self.cv_image, (int(self.x_center), int(self.y_center)), 5, (0, 0, 255), -1)
        lms = self.results.pose_landmarks.landmark
        x_min = min(lm.x for lm in lms)
        x_max = max(lm.x for lm in lms)
        y_min = min(lm.y for lm in lms)
        y_max = max(lm.y for lm in lms)
        cv2.rectangle(self.cv_image,
                      (int(x_min * img_w), int(y_min * img_h)),
                      (int(x_max * img_w), int(y_max * img_h)),
                      (0, 255, 0), 2)
        self.mp_drawing.draw_landmarks(self.cv_image, self.results.pose_landmarks)

    def move_robot(self):
        Kp_l   = 0.4
        Kp_yaw = 0.00065
        Kd_yaw = 0.00007
        Kd_l   = 0.37

        x_error = self.x_center - self.image_center - 3.0

        if self.depth_mm > 0.3:
            if x_error > 10:
                top, bottom = "Right==>", "Go Forward"
            elif x_error < -10:
                top, bottom = "<==Left",  "Go Forward"
            else:
                top, bottom = "Centre",   "Go Forward"
        else:
            self.vel_control(0.0, 0.0)
            top, bottom = "Centre", "Stopped"

        P_yaw = -(Kp_yaw * x_error)
        D_yaw = ((x_error - self.last_error) / 0.6) * Kd_yaw
        D_l   = ((self.depth_mm - self.last_depth) / 0.6) * Kd_l

        self.last_depth = self.depth_mm
        self.last_error = x_error

        self.display_text_on_image(bottom, top)

    def vel_control(self, vel_x, vel_spin):
        twist_msg = Twist()
        twist_msg.linear.x  = vel_x
        twist_msg.angular.z = vel_spin
        self.velocity_publisher.publish(twist_msg)

    def display_text_on_image(self, bottom, top):
        img_h, img_w = self.cv_image.shape[:2]
        font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        cv2.putText(self.cv_image, bottom, (300, img_h - 30),
                    font, 1, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(self.cv_image, top,    (300, 30),
                    font, 1, (0, 100, 0),   1, cv2.LINE_AA)
        cv2.imshow('Person Detection',   self.cv_image)
        cv2.imshow('Person Segmentation', self.segmentation_frame)
        cv2.waitKey(3)


def main():
    rclpy.init()
    node = PersonFollower()
    rclpy.spin(node)
    cv2.destroyAllWindows()
    rclpy.shutdown()


if __name__ == "__main__":
    main()