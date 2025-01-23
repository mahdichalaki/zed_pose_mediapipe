#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import mediapipe as mp
import numpy as np
from std_msgs.msg import Float32
import pyzed.sl as sl

# Function to calculate angle between three points
def calculate_angle(a, b, c):
    a, b, c = np.array(a), np.array(b), np.array(c)
    radians = np.arctan2(c[1] - b[1], c[0] - b[0]) - np.arctan2(a[1] - b[1], a[0] - b[0])
    angle = np.abs(radians * 180.0 / np.pi)
    return 360 - angle if angle > 180.0 else angle

class PoseAssess(Node):
    def __init__(self):
        super().__init__('pose_assess')

        # Initialize ZED Camera
        self.zed = sl.Camera()
        init_params = sl.InitParameters()
        init_params.camera_resolution = sl.RESOLUTION.VGA
        init_params.depth_mode = sl.DEPTH_MODE.PERFORMANCE
        init_params.coordinate_units = sl.UNIT.METER
        init_params.sdk_verbose = 1
        
        # Open the camera
        err = self.zed.open(init_params)
        if err != sl.ERROR_CODE.SUCCESS:
            self.get_logger().error(f"Error opening ZED camera: {err}")
            exit(1)
        
        # Set runtime parameters
        self.runtime_parameters = sl.RuntimeParameters()
        
        # Create images to store camera frames
        self.left_image_zed = sl.Mat()
        self.right_image_zed = sl.Mat()
        
        # Initialize MediaPipe pose
        self.mp_pose = mp.solutions.pose
        self.pose = self.mp_pose.Pose()  # Initialize once
        self.mp_drawing = mp.solutions.drawing_utils

        # ROS2 Parameters
        self.declare_parameter('display_output', True)
        self.declare_parameter('assessment_rate', 20)
        self.display_output = self.get_parameter('display_output').get_parameter_value().bool_value
        self.assessment_rate = self.get_parameter('assessment_rate').get_parameter_value().integer_value

        # Timer to assess pose for the camera input
        self.assess_timer = self.create_timer(1.0 / self.assessment_rate, self.assess_pose)

        # Publishers to publish pose angles
        self.angle_publishers = self.create_angle_publishers()

        self.get_logger().info("Pose Assessment Node with ZED Camera Started")

    def create_angle_publishers(self):
        publishers = {}
        for camera in ['left', 'right']:
            for side in ['left', 'right']:
                for part in ['arm', 'torso']:
                    topic_name = f'/pose_assess/{side}_{part}_ang_{camera}_camera'
                    publishers[f'{side}_{part}_{camera}'] = self.create_publisher(Float32, topic_name, 10)
        return publishers

    def capture_frames(self):
        """Capture frames from the ZED camera."""
        if self.zed.grab(self.runtime_parameters) == sl.ERROR_CODE.SUCCESS:
            self.zed.retrieve_image(self.left_image_zed, sl.VIEW.LEFT)
            self.zed.retrieve_image(self.right_image_zed, sl.VIEW.RIGHT)
            left_frame = cv2.cvtColor(self.left_image_zed.get_data(), cv2.COLOR_BGRA2BGR)  # Convert to BGR format
            right_frame = cv2.cvtColor(self.right_image_zed.get_data(), cv2.COLOR_BGRA2BGR)  # Convert to BGR format
            return left_frame, right_frame
        else:
            self.get_logger().warn("Failed to grab images from ZED camera.")
            return None, None

    def assess_pose(self):
        """Assess the pose using the captured frames from ZED camera."""
        left_frame, right_frame = self.capture_frames()
        if left_frame is not None:
            self.process_frame(left_frame, camera="left")
        if right_frame is not None:
            self.process_frame(right_frame, camera="right")

    def process_frame(self, frame, camera="left"):
        """Process a single frame to detect pose and publish angles."""
        # Resize the image to a smaller resolution to speed up processing
        resized_frame = cv2.resize(frame, (320, 240))

        # Convert the frame to RGB
        image_rgb = cv2.cvtColor(resized_frame, cv2.COLOR_BGR2RGB)

        # Process the frame with MediaPipe Pose
        results = self.pose.process(image_rgb)

        # Draw pose landmarks
        image_bgr = cv2.cvtColor(image_rgb, cv2.COLOR_RGB2BGR)
        if results.pose_landmarks:
            self.mp_drawing.draw_landmarks(image_bgr, results.pose_landmarks, self.mp_pose.POSE_CONNECTIONS)

            # Extract landmarks
            landmarks = results.pose_landmarks.landmark

            # Calculate angles
            angles = self.calculate_angles(landmarks)

            # Publish calculated angles
            for key, angle in angles.items():
                self.angle_publishers[f'{key}_{camera}'].publish(Float32(data=angle))

            if self.display_output:
                self.display_results(image_bgr, landmarks, angles, camera)

    def calculate_angles(self, landmarks):
        """Calculate angles between keypoints using landmark data."""
        keypoints = {
            'left_arm': ['LEFT_SHOULDER', 'LEFT_ELBOW', 'LEFT_WRIST'],
            'right_arm': ['RIGHT_SHOULDER', 'RIGHT_ELBOW', 'RIGHT_WRIST'],
            'left_torso': ['LEFT_HIP', 'LEFT_SHOULDER', 'LEFT_ELBOW'],
            'right_torso': ['RIGHT_HIP', 'RIGHT_SHOULDER', 'RIGHT_ELBOW']
        }
        angles = {}
        for key, points in keypoints.items():
            a = [landmarks[getattr(self.mp_pose.PoseLandmark, points[0]).value].x, 
                 landmarks[getattr(self.mp_pose.PoseLandmark, points[0]).value].y]
            b = [landmarks[getattr(self.mp_pose.PoseLandmark, points[1]).value].x, 
                 landmarks[getattr(self.mp_pose.PoseLandmark, points[1]).value].y]
            c = [landmarks[getattr(self.mp_pose.PoseLandmark, points[2]).value].x, 
                 landmarks[getattr(self.mp_pose.PoseLandmark, points[2]).value].y]
            angles[key] = calculate_angle(a, b, c)
        return angles

    def display_results(self, image, landmarks, angles, camera):
        """Display the results on the image feed."""
        # Display angle values on the image
        for key, angle in angles.items():
            if 'left' in key:
                x, y = landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW.value].x, landmarks[self.mp_pose.PoseLandmark.LEFT_ELBOW.value].y
            else:
                x, y = landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW.value].x, landmarks[self.mp_pose.PoseLandmark.RIGHT_ELBOW.value].y
            cv2.putText(image, f'{key}: {int(angle)}',
                        tuple(np.multiply([x, y], [320, 240]).astype(int)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2, cv2.LINE_AA)

        # Show the feed
        window_name = f'Mediapipe Feed - {camera.capitalize()} Camera'
        cv2.imshow(window_name, image)
        cv2.waitKey(1)

    def destroy_pose(self):
        """Cleanup MediaPipe resources."""
        self.pose.close()  # Properly close the MediaPipe Pose instance

    def destroy_node(self):
        """Override destroy_node to include cleanup."""
        self.destroy_pose()
        self.zed.close()  # Properly close the ZED Camera
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    pose_assess = PoseAssess()
    rclpy.spin(pose_assess)
    pose_assess.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
