# ROS2 Package for Pose Assessment and Logging using Mediapipe

This repository contains two ROS2 nodes for real-time human pose assessment and data logging using a ZED stereo camera and MediaPipe. These nodes are designed to capture and process pose data, calculate angles of key body parts, and log them to a CSV file for further analysis.

---

## Features

1. **Pose Assessment**:
   - Captures frames from the ZED stereo camera.
   - Processes pose landmarks using MediaPipe Pose.
   - Calculates angles for arms and torso for both left and right views.
   - Publishes the calculated angles as ROS2 topics.

2. **Data Logging**:
   - Subscribes to the pose assessment topics.
   - Logs the angles with timestamps into a CSV file.
   - Provides real-time feedback via ROS2 logs.

---

## Nodes

### 1. PoseAssess Node

- **Description**:  
  Captures frames from the ZED camera, processes them using MediaPipe Pose, calculates angles for key body parts, and publishes these angles as ROS2 topics.

- **Key Functionalities**:
  - ZED camera integration.
  - MediaPipe Pose-based angle calculation.
  - ROS2 topic publishing for calculated angles.
  - Optional display of annotated frames.

- **Published Topics**:
  - `/pose_assess/left_arm_ang_left_camera`
  - `/pose_assess/right_arm_ang_left_camera`
  - `/pose_assess/left_torso_ang_left_camera`
  - `/pose_assess/right_torso_ang_left_camera`
  - `/pose_assess/left_arm_ang_right_camera`
  - `/pose_assess/right_arm_ang_right_camera`
  - `/pose_assess/left_torso_ang_right_camera`
  - `/pose_assess/right_torso_ang_right_camera`

- **Parameters**:
  - `display_output` (bool): Enables or disables the display of annotated frames. Default: `True`.
  - `assessment_rate` (int): Frame processing rate (Hz). Default: `20`.

---

### 2. PoseLogger Node

- **Description**:  
  Subscribes to pose assessment topics and logs the data to a CSV file with timestamps.

- **Key Functionalities**:
  - Logs angles for arms and torso from both left and right cameras.
  - Saves data to a CSV file for offline analysis.
  - Provides real-time ROS2 log updates.

- **Subscribed Topics**:
  - `/pose_assess/left_arm_ang_left_camera`
  - `/pose_assess/right_arm_ang_left_camera`
  - `/pose_assess/left_torso_ang_left_camera`
  - `/pose_assess/right_torso_ang_left_camera`
  - `/pose_assess/left_arm_ang_right_camera`
  - `/pose_assess/right_arm_ang_right_camera`
  - `/pose_assess/left_torso_ang_right_camera`
  - `/pose_assess/right_torso_ang_right_camera`

---

## Getting Started

### Dependencies

Ensure the following dependencies are installed:

- ROS2 (Humble recommended)
- [ZED SDK](https://www.stereolabs.com/zed-sdk/)
- Python packages:
  - `rclpy`
  - `sensor_msgs`
  - `std_msgs`
  - `cv_bridge`
  - `mediapipe`
  - `opencv-python`
  - `numpy`
  - `pyzed`
  
### Running the Pose Assessment Node

    `ros2 run <package_name> pose_assess --ros-args -p display_output:=True -p assessment_rate:=20`

### Running the Pose Logger Node

    `ros2 run <package_name> pose_logger`

---

## Example Workflow

1. Start the PoseAssess node to process pose data from the ZED camera.
2. Start the PoseLogger node to log the published angles.
3. Analyze the generated pose_log.csv file for further insights.
