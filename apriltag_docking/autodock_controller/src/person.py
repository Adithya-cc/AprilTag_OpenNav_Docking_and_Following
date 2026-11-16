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
from std_msgs.msg import ColorRGBA

class PersonFollower(Node):
    def __init__(self):
        super().__init__("person_follower")
        self.bridge = CvBridge()
        
        # Subscribe to the RGB image topic and depth image topic
        self.image_sub = self.create_subscription(Image, "/camera_depth_frame/image_raw", self.callback, 10)
        self.depth_sub = self.create_subscription(PointCloud2, "/camera_depth_frame/points", self.depth_callback, 10)

        # Publisher for robot velocity commands
        self.pose = self.create_publisher(PoseStamped,"/detected_person",10)
        self.velocity_publisher = self.create_publisher(Twist, "/dummy/cmd_vel", 10)
        self.marker_pub = self.create_publisher(
                Marker,
                "/person_marker",
                10
            )
        # Initialize variables
        self.last_error = 0.0
        self.last_depth = 0.0
        self.depth_image = None
        self.x_y_z_pts = None
        self.person_detected = False
        self.depth_mm = 0
        self.velocity_msg = Twist()
        self.velocity_msg = Twist()
        self.velocity_msg.linear.y = 0.0
        self.velocity_msg.linear.z = 0.0
        self.velocity_msg.angular.x = 0.0
        self.velocity_msg.angular.y = 0.0
        self.mp_drawing = mp.solutions.drawing_utils
        self.mp_holistic = mp.solutions.holistic
        # Initialize MediaPipe Pose module
        self.mp_pose = mp.solutions.pose.Pose(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        # Initialize the TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        self.kp_angular=0.005
        self.kp_linear=0.7
        self.x_center=None
        self.image_center=None
        self.buffer=10

        # Create the options that will be used for ImageSegmenter
        base_options = python.BaseOptions(model_asset_path='/home/adithya-si3510/autodock_ros2_ws/src/apriltag_docking/autodock_controller/param/deeplabv3.tflite')
        options = vision.ImageSegmenterOptions(base_options=base_options,output_category_mask=True)
        self.segmenter = vision.ImageSegmenter.create_from_options(options)

        self.BG_COLOR = (192, 192, 192) # gray
        self.MASK_COLOR = (0, 255, 0) # white

    def depth_callback(self, data):
        try:
            # Convert depth image to OpenCV format
            self.depth_image = data
            print(f"{self.depth_image.width},{self.depth_image.height}")
        except Exception as e:
            self.get_logger().error(f"Error converting depth image: {e}")

    def publish_person_marker(self, x, y, z):

        marker = Marker()

        marker.header.frame_id = "camera_depth_optical_frame"
        marker.header.stamp = self.depth_image.header.stamp

        marker.ns = "person"
        marker.id = 0

        marker.type = Marker.SPHERE
        marker.action = Marker.ADD

        # Position
        marker.pose.position.x = float(x)
        marker.pose.position.y = float(y)
        marker.pose.position.z = float(z)

        # No rotation
        marker.pose.orientation.x = 0.0
        marker.pose.orientation.y = 0.0
        marker.pose.orientation.z = 0.0
        marker.pose.orientation.w = 1.0

        # Sphere size (meters)
        marker.scale.x = 5.0
        marker.scale.y = 5.0
        marker.scale.z = 5.0

        # Red color
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        marker.lifetime.sec = 0   # 0 = forever

        self.marker_pub.publish(marker)

    def callback(self, data):

        # Convert RGB image to OpenCV format
        self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        rgb_cv_image = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2RGB)

        self.segmentation_frame=self.cv_image
        self.results = self.mp_pose.process(rgb_cv_image)

        if self.results.pose_landmarks is not None:
            # Person detected
            self.person_detected = True
            landmarks = self.results.pose_landmarks.landmark

            # Calculate centroid
            x_centroid = sum([landmark.x for landmark in landmarks]) / len(landmarks)
            y_centroid = sum([landmark.y for landmark in landmarks]) / len(landmarks)
            self.x_center = x_centroid * self.cv_image.shape[1]
            self.y_center = y_centroid * self.cv_image.shape[0]
            self.image_center = self.cv_image.shape[1] / 2

            cv2.circle(self.cv_image, (int(x_centroid * self.cv_image.shape[1]), int(y_centroid * self.cv_image.shape[0])), 5, (0, 0, 255), -1)

            x_min = min([landmark.x for landmark in landmarks])
            x_max = max([landmark.x for landmark in landmarks])
            y_min = min([landmark.y for landmark in landmarks])
            y_max = max([landmark.y for landmark in landmarks])

            cv2.rectangle(self.cv_image, (int(x_min * self.cv_image.shape[1]), int(y_min * self.cv_image.shape[0])),
                          (int(x_max * self.cv_image.shape[1]), int(y_max * self.cv_image.shape[0])), (0, 255, 0), 2)
            self.segmentation_frame = mp.Image(image_format=mp.ImageFormat.SRGB, data=self.segmentation_frame)
            # mask for the segmented image
            segmentation_result = self.segmenter.segment(self.segmentation_frame)
            category_mask = segmentation_result.category_mask

            image_data = self.segmentation_frame.numpy_view()
            fg_image = np.zeros(image_data.shape, dtype=np.uint8)
            fg_image[:] = self.MASK_COLOR
            bg_image = np.zeros(image_data.shape, dtype=np.uint8)
            bg_image[:] = self.BG_COLOR
            
            condition = np.stack((category_mask.numpy_view(),) * 3, axis=-1) > 0.2

            self.segmentation_frame = np.where(condition, fg_image, bg_image)
            self.mp_drawing.draw_landmarks(self.segmentation_frame, self.results.pose_landmarks,self.mp_holistic.POSE_CONNECTIONS)

            if self.results.pose_landmarks is not None:
                cv2.line(self.cv_image, (int(self.x_center-15), int(self.y_center)), (int(self.x_center+15), int(self.y_center)), (255, 0, 0), 3) 
                cv2.line(self.cv_image, (int(self.x_center), int(self.y_center-15)), (int(self.x_center), int(self.y_center+15)), (255, 0, 0), 3)
                cv2.line(self.segmentation_frame, (int(self.x_center-15), int(self.y_center)), (int(self.x_center+15), int(self.y_center)), (255, 0, 0), 3) 
                cv2.line(self.segmentation_frame, (int(self.x_center), int(self.y_center-15)), (int(self.x_center), int(self.y_center+15)), (255, 0, 0), 3)
                cv2.line(self.segmentation_frame, (int(350), int(0)), (int(350), int(500)), (0, 0, 255), 2) 
                cv2.line(self.segmentation_frame, (int(0), int(self.y_center)), (int(700), int(self.y_center)), (0, 0, 255), 2)
            
            # Setting the limit for co-ordinates
            self.limiting_loop()
            # self.y_center = 1.0 
            # Check if depth information is available
            flat_indices = []
            if self.depth_image is not None:
                w = self.depth_image.width
                h = self.depth_image.height
                u = int(max(0, min(self.x_center, self.depth_image.width - 1)))
                v = int(max(0, min(self.y_center, self.depth_image.height - 1)))
                flat_indices.append(int(v * w + u))
                # Deduplicate (edge clamping creates duplicates)
                flat_indices = list(flat_indices)
                self.x_y_z_pts = point_cloud2.read_points_list(self.depth_image,field_names=['x','y','z'], skip_nans=True,uvs=flat_indices)
                # self.x_y_z_pts = self.y_center* self.depth_image.width + self.x_center
                # self.depth_mm = self.depth_image[int(self.y_center), int(self.x_center)]
                print(self.x_y_z_pts)
                print(f"center x:{self.x_center} y:{self.y_center}")
            else:
                self.vel_control(0.0, 0.0)
            # Draw landmarks and bounding box
            flat_index = int((v * self.depth_image.width) + u)

            # 4. Extract the point safely
            # if flat_index < len(self.x_y_z_pts):
            #     target_point = self.x_y_z_pts[flat_index]
                
            #     x_3d = target_point.x
            #     y_3d = target_point.y
            #     z_3d = target_point.z
            x_3d, y_3d, z_3d = self.x_y_z_pts[0][0], self.x_y_z_pts[0][1], self.x_y_z_pts[0][2]
            # 1. Create and populate the PoseStamped message
            pose_msg = PoseStamped()
            # z_3d=1.0
            print(f"Depth is x:{x_3d}, y:{y_3d}, z:{z_3d}")
            self.publish_person_marker(x_3d,y_3d,z_3d)
            # Essential: Sync header timestamp and frame_id with the camera data
            pose_msg.header.frame_id = self.depth_image.header.frame_id
            pose_msg.header.stamp = self.depth_image.header.stamp
            # Set 3D Position (converted to float explicitly for ROS serialization)
            pose_msg.pose.position.x = float(x_3d)
            pose_msg.pose.position.y = float(y_3d)
            pose_msg.pose.position.z = float(z_3d)
            
            # Set Neutral Orientation (Identity Quaternion)
            pose_msg.pose.orientation.x = 0.0
            pose_msg.pose.orientation.y = 0.0
            pose_msg.pose.orientation.z = 0.0
            pose_msg.pose.orientation.w = 1.0

            # 2. Publish the pose to /detected_person
            self.pose.publish(pose_msg)



            # --- 2. Create and Broadcast TF Transform ---
            t = TransformStamped()
            
            # Read frame data dynamically from the camera sensor
            t.header.stamp = self.depth_image.header.stamp
            t.header.frame_id = self.depth_image.header.frame_id#"camera_depth_frame"
            # t.header.stamp = self.get_clock().now().to_msg()
            # Name of the new coordinate frame being generated
            t.child_frame_id = 'detected_person'
            
            # Assign the translation (3D Coordinates)
            t.transform.translation.x = float(x_3d)
            t.transform.translation.y = float(y_3d)
            t.transform.translation.z = float(z_3d)
            
            # Assign a stable uniform orientation matrix (No rotation)
            t.transform.rotation.x = -0.5
            t.transform.rotation.y = 0.5
            t.transform.rotation.z = -0.5
            t.transform.rotation.w = 0.5

            # Send the transformation out to the tf tree
            self.tf_broadcaster.sendTransform(t)
            self.draw_landmarks_and_box()
            # cv2.imshow('Person Detection', self.cv_image)
            # cv2.imshow('Person Segmentation', self.segmentation_frame)
            # Move the robot based on the detected person
            self.move_robot()

        else: # No Person Detected
            print("No Human")
            return
            if self.person_detected == True:  # Person was detected atleast once before
                self.vel_control(0.0 , -0.5)

                # Put text on the screen
                top = "Searching Person"
                bottom = "Stop"
                self.display_text_on_image(bottom,top)
            
            else:  # Person was never detected
                self.vel_control(0.0, 0.0)

    def draw_landmarks_and_box(self):

        cv2.circle(self.cv_image, (int(self.x_center), int(self.y_center)), 5, (0, 0, 255), -1)

        x_min = min([landmark.x for landmark in self.results.pose_landmarks.landmark])
        x_max = max([landmark.x for landmark in self.results.pose_landmarks.landmark])
        y_min = min([landmark.y for landmark in self.results.pose_landmarks.landmark])
        y_max = max([landmark.y for landmark in self.results.pose_landmarks.landmark])

        cv2.rectangle(self.cv_image, (int(x_min * self.cv_image.shape[1]), int(y_min * self.cv_image.shape[0])),
                      (int(x_max * self.cv_image.shape[1]), int(y_max * self.cv_image.shape[0])), (0, 255, 0), 2)
        self.mp_drawing.draw_landmarks(self.cv_image, self.results.pose_landmarks)

    def move_robot(self):
        # Constants for PD controller
        Kp_l = 0.4
        Kp_yaw = 0.00065
        Kd_yaw = 0.00007
        Kd_l = 0.37

        # Calculating the error
        x_error = self.x_center - self.image_center -3.0

        if self.depth_mm > 3:

            # Determine the direction to move based on the person's position
            if x_error > 10:
                top = "Right==>"
                bottom = "Go Forward"
            elif x_error < -10:
               top = "<==Left"
               bottom = "Go Forward"
            else:
               top = "Centre"
               bottom = "Go Forward"
        else:
            # Stop the robot if depth information is insufficient
            self.vel_control(0.0, 0.0)
            top = "Centre"
            bottom = "Stopped"

        # Proportional and Derivative Drive
        P_x = Kp_l * self.depth_mm
        P_yaw = -(Kp_yaw * x_error)
        D_yaw = ((x_error - self.last_error) / 0.6) * Kd_yaw
        D_l = ((self.depth_mm - self.last_depth) / 0.6) * Kd_l

        self.last_depth = self.depth_mm
        self.last_error = x_error

        # Publish the Twist message to move the robot
        # self.vel_control((P_x + D_l), (P_yaw + D_yaw))

        # Display text on the image
        self.display_text_on_image(bottom, top)
        

    
    def limiting_loop(self):
        if self.x_center > 700:
                self.x_center=699.0
        if self.y_center> 500:
                self.y_center=499.0
    
    def vel_control(self, vel_x, vel_spin):
        twist_msg = Twist()
        twist_msg.linear.x = vel_x
        twist_msg.angular.z = vel_spin
        self.velocity_publisher.publish(twist_msg)

    def display_text_on_image(self, bottom, top):
        # Display text on the image
        img = self.cv_image
        text1 =bottom
        text2 =top
        txt1_location = (300, 450)
        txt2_location = (300, 30)
        font = cv2.FONT_HERSHEY_COMPLEX_SMALL
        fontScale = 1
        fontColor_lt = (255, 255, 255)
        fontColor_at = (0, 100, 0)
        thickness = 1
        lineType = cv2.LINE_AA

        cv2.putText(img, text1, txt1_location, font, fontScale, fontColor_lt, thickness, lineType)
        cv2.putText(img, text2, txt2_location, font, fontScale, fontColor_at, thickness, lineType)
        cv2.imshow('Person Detection', self.cv_image)
        cv2.imshow('Person Segmentation', self.segmentation_frame)
        cv2.waitKey(3)
        
def main():
    rclpy.init()
    Mynode = PersonFollower()
    rclpy.spin(Mynode)
    cv2.destroyAllWindows()
    rclpy.shutdown()

if __name__ == "__main__":
    main()