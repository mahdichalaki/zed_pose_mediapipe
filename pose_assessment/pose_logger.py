import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import csv
import os
from time import time

class PoseLogger(Node):
    def __init__(self):
        super().__init__('pose_logger')

        # Define the CSV file
        self.csv_file = 'pose_log.csv'
        self.csv_file_path = os.path.join(os.getcwd(), self.csv_file)

        # Open the CSV file in write mode
        self.csv_file_handle = open(self.csv_file_path, mode='w', newline='')
        self.csv_writer = csv.writer(self.csv_file_handle)

        # Write the header to the CSV file
        self.csv_writer.writerow([
            'timestamp', 
            'left_arm_angle_left', 'right_arm_angle_left',
            'left_torso_angle_left', 'right_torso_angle_left',
            'left_arm_angle_right', 'right_arm_angle_right',
            'left_torso_angle_right', 'right_torso_angle_right'
        ])

        # Start time reference
        self.start_time = time()

        # Subscribers for all 8 topics
        self.create_subscription(Float32, '/pose_assess/left_arm_ang_left_camera', self.left_arm_ang_left_callback, 10)
        self.create_subscription(Float32, '/pose_assess/right_arm_ang_left_camera', self.right_arm_ang_left_callback, 10)
        self.create_subscription(Float32, '/pose_assess/left_torso_ang_left_camera', self.left_torso_ang_left_callback, 10)
        self.create_subscription(Float32, '/pose_assess/right_torso_ang_left_camera', self.right_torso_ang_left_callback, 10)

        self.create_subscription(Float32, '/pose_assess/left_arm_ang_right_camera', self.left_arm_ang_right_callback, 10)
        self.create_subscription(Float32, '/pose_assess/right_arm_ang_right_camera', self.right_arm_ang_right_callback, 10)
        self.create_subscription(Float32, '/pose_assess/left_torso_ang_right_camera', self.left_torso_ang_right_callback, 10)
        self.create_subscription(Float32, '/pose_assess/right_torso_ang_right_camera', self.right_torso_ang_right_callback, 10)

        # Initialize data storage
        self.data = {
            'left_arm_angle_left': None,
            'right_arm_angle_left': None,
            'left_torso_ang_left': None,
            'right_torso_ang_left': None,
            'left_arm_angle_right': None,
            'right_arm_angle_right': None,
            'left_torso_ang_right': None,
            'right_torso_ang_right': None,
        }

    def log_data(self):
        # Calculate the timestamp relative to the start time
        timestamp = time() - self.start_time

        # Check if all data points are available
        if None not in self.data.values():
            with open(self.csv_file_path, mode='a') as file:  # Updated this line to use csv_file_path
                writer = csv.writer(file)
                writer.writerow([timestamp] + list(self.data.values()))
            self.get_logger().info(f"Data logged at {timestamp:.2f} seconds")
            # Reset the data
            for key in self.data:
                self.data[key] = None

    # Callback functions for each subscriber
    def left_arm_ang_left_callback(self, msg):
        self.data['left_arm_angle_left'] = msg.data
        self.log_data()

    def right_arm_ang_left_callback(self, msg):
        self.data['right_arm_angle_left'] = msg.data
        self.log_data()

    def left_torso_ang_left_callback(self, msg):
        self.get_logger().info(f"Received left torso angle (left camera): {msg.data}")
        self.data['left_torso_ang_left'] = msg.data
        self.log_data()

    def right_torso_ang_left_callback(self, msg):
        self.get_logger().info(f"Received right torso angle (left camera): {msg.data}")
        self.data['right_torso_ang_left'] = msg.data
        self.log_data()

    def left_arm_ang_right_callback(self, msg):
        self.data['left_arm_angle_right'] = msg.data
        self.log_data()

    def right_arm_ang_right_callback(self, msg):
        self.data['right_arm_angle_right'] = msg.data
        self.log_data()

    def left_torso_ang_right_callback(self, msg):
        self.get_logger().info(f"Received left torso angle (right camera): {msg.data}")
        self.data['left_torso_ang_right'] = msg.data
        self.log_data()

    def right_torso_ang_right_callback(self, msg):
        self.get_logger().info(f"Received right torso angle (right camera): {msg.data}")
        self.data['right_torso_ang_right'] = msg.data
        self.log_data()

    def destroy_node(self):
        # Close the CSV file when the node is destroyed
        self.csv_file_handle.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    pose_logger = PoseLogger()
    rclpy.spin(pose_logger)
    pose_logger.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
