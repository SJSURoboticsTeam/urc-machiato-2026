#!/usr/bin/env python3

"""
Sensor Simulator Node for URC 2026 Rover Autonomy Testing

This node simulates GPS, IMU, and camera sensors when Gazebo is not available,
providing realistic sensor data for testing autonomy algorithms.
"""

import math

# OpenCV for camera simulation
import cv2
import numpy as np
import sys
import os
import rclpy
from geometry_msgs.msg import TransformStamped, PoseStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node

from autonomy_utilities import QoSProfiles, ParameterManager

# ROS2 message types
from sensor_msgs.msg import CameraInfo, Image, Imu, NavSatFix
from tf2_ros import TransformBroadcaster


class SensorSimulator(Node):
    """Simulates rover sensors for testing autonomy systems."""

    def __init__(self):
        super().__init__("sensor_simulator")

        # Standardized parameter management
        self.param_manager = ParameterManager(self)

        # Define parameter specifications
        # Note: use_sim_time is a standard ROS2 parameter handled automatically
        param_specs = {
            "gps_noise_std": {
                "default": 0.5,  # Reduced for better test consistency
                "type": float,
                "description": "GPS position noise standard deviation (meters)",
            },
            "imu_noise_std": {
                "default": 0.01,
                "type": float,
                "description": "IMU noise standard deviation (rad/s, m/s²)",
            },
            "camera_width": {
                "default": 640,
                "type": int,
                "description": "Camera image width in pixels",
            },
            "camera_height": {
                "default": 480,
                "type": int,
                "description": "Camera image height in pixels",
            },
            "camera_fps": {
                "default": 30.0,
                "type": float,
                "description": "Camera frame rate",
            },
        }

        # Declare and get parameters
        params = self.param_manager.declare_parameters(param_specs)

        # Extract parameter values
        self.use_sim_time = self.get_parameter("use_sim_time").value
        self.gps_noise_std = params["gps_noise_std"]
        self.imu_noise_std = params["imu_noise_std"]
        self.camera_width = params["camera_width"]
        self.camera_height = params["camera_height"]
        self.camera_fps = params["camera_fps"]

        # Simulation state
        self.start_time = self.get_clock().now()
        self.current_pose = {
            "x": 0.0,
            "y": 0.0,
            "z": 0.0,
            "roll": 0.0,
            "pitch": 0.0,
            "yaw": 0.0,
        }

        # Base GPS coordinates (Standard reference location)
        self.base_lat = 35.0
        self.base_lon = -117.0
        self.base_alt = 100.0

        # Use standardized QoS profiles
        sensor_qos = QoSProfiles.sensor_data()

        # Publishers
        self.gps_publisher = self.create_publisher(NavSatFix, "/gps/fix", sensor_qos)
        self.imu_publisher = self.create_publisher(Imu, "/imu", sensor_qos)
        self.camera_publisher = self.create_publisher(
            Image, "/rover/camera/image_raw", sensor_qos
        )
        self.camera_info_publisher = self.create_publisher(
            CameraInfo, "/rover/camera/camera_info", sensor_qos
        )
        self.odom_publisher = self.create_publisher(
            Odometry, "/odom", QoSProfiles.state_data()
        )
        self.slam_pose_publisher = self.create_publisher(
            PoseStamped, "/slam/pose", QoSProfiles.state_data()
        )

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Timers for sensor publishing
        gps_period = 1.0  # 1 Hz GPS
        imu_period = 1.0 / 100.0  # 100 Hz IMU
        camera_period = 1.0 / self.camera_fps  # Camera FPS
        odom_period = 1.0 / 30.0  # 30 Hz odometry

        self.gps_timer = self.create_timer(gps_period, self.publish_gps)
        self.imu_timer = self.create_timer(imu_period, self.publish_imu)
        self.camera_timer = self.create_timer(camera_period, self.publish_camera)
        self.odom_timer = self.create_timer(odom_period, self.publish_odometry)

        # Initialize camera intrinsics
        self.camera_matrix = np.array(
            [
                [self.camera_width, 0, self.camera_width / 2],
                [0, self.camera_height, self.camera_height / 2],
                [0, 0, 1],
            ]
        )
        self.dist_coeffs = np.zeros(5)

        self.get_logger().info("Sensor Simulator initialized")

    def publish_gps(self):
        """Publish GPS fix message with realistic noise."""
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps_link"

        # Add noise to base position
        noise_x = np.random.normal(0, self.gps_noise_std)
        noise_y = np.random.normal(0, self.gps_noise_std)

        # Convert local offset to lat/lon (approximate)
        earth_radius = 6371000.0  # meters
        lat_offset = (
            (self.current_pose["y"] + noise_y) / earth_radius * (180.0 / math.pi)
        )
        lon_offset = (
            (self.current_pose["x"] + noise_x)
            / (earth_radius * math.cos(math.radians(self.base_lat)))
            * (180.0 / math.pi)
        )

        msg.latitude = self.base_lat + lat_offset
        msg.longitude = self.base_lon + lon_offset
        msg.altitude = (
            self.base_alt
            + self.current_pose["z"]
            + np.random.normal(0, self.gps_noise_std * 2)
        )

        # GPS covariance (3m horizontal, 6m vertical accuracy)
        msg.position_covariance = [9.0, 0.0, 0.0, 0.0, 9.0, 0.0, 0.0, 0.0, 36.0]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        msg.status.status = 0  # STATUS_FIX
        msg.status.service = 1  # SERVICE_GPS

        self.gps_publisher.publish(msg)

    def publish_imu(self):
        """Publish IMU data with realistic noise."""
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu_link"

        # Simulate some motion (small oscillations)
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        motion_freq = 0.1  # Hz

        # Angular velocity (simulate small rotations)
        msg.angular_velocity.x = 0.01 * math.sin(
            2 * math.pi * motion_freq * current_time
        ) + np.random.normal(0, self.imu_noise_std)
        msg.angular_velocity.y = 0.005 * math.cos(
            2 * math.pi * motion_freq * current_time
        ) + np.random.normal(0, self.imu_noise_std)
        msg.angular_velocity.z = 0.0 + np.random.normal(0, self.imu_noise_std)

        # Linear acceleration (gravity + motion)
        msg.linear_acceleration.x = np.random.normal(0, self.imu_noise_std)
        msg.linear_acceleration.y = np.random.normal(0, self.imu_noise_std)
        msg.linear_acceleration.z = -3.71 + np.random.normal(
            0, self.imu_noise_std
        )  # Mars gravity

        # Orientation (simplified - assume mostly level)
        msg.orientation.w = 1.0
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0

        # Covariances
        msg.angular_velocity_covariance = [
            self.imu_noise_std**2,
            0.0,
            0.0,
            0.0,
            self.imu_noise_std**2,
            0.0,
            0.0,
            0.0,
            self.imu_noise_std**2,
        ]

        msg.linear_acceleration_covariance = [
            self.imu_noise_std**2,
            0.0,
            0.0,
            0.0,
            self.imu_noise_std**2,
            0.0,
            0.0,
            0.0,
            self.imu_noise_std**2,
        ]

        msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]

        self.imu_publisher.publish(msg)

    def publish_camera(self):
        """Publish simulated camera image."""
        # Create a simple test pattern
        image = np.zeros((self.camera_height, self.camera_width, 3), dtype=np.uint8)

        # Add some simulated Mars-like terrain
        # Create a simple gradient background (sky)
        for y in range(self.camera_height):
            color = int(255 * (1 - y / self.camera_height))
            image[y, :, :] = [color, color * 0.8, color * 0.6]  # Reddish sky

        # Add some "rocks" (dark circles)
        for i in range(5):
            center_x = np.random.randint(50, self.camera_width - 50)
            center_y = np.random.randint(
                self.camera_height // 2, self.camera_height - 50
            )
            radius = np.random.randint(20, 50)
            cv2.circle(image, (center_x, center_y), radius, (100, 80, 60), -1)

        # Convert to ROS Image message
        msg = Image()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height = self.camera_height
        msg.width = self.camera_width
        msg.encoding = "rgb8"
        msg.is_bigendian = False
        msg.step = self.camera_width * 3
        msg.data = image.tobytes()

        self.camera_publisher.publish(msg)

        # Publish camera info
        info_msg = CameraInfo()
        info_msg.header = msg.header
        info_msg.height = self.camera_height
        info_msg.width = self.camera_width
        info_msg.distortion_model = "plumb_bob"
        info_msg.d = self.dist_coeffs.tolist()
        info_msg.k = self.camera_matrix.flatten().tolist()
        info_msg.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info_msg.p = self.camera_matrix.flatten().tolist() + [0.0, 0.0, 0.0]

        self.camera_info_publisher.publish(info_msg)

    def publish_odometry(self):
        """Publish odometry data."""
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"

        # Simulate slow movement
        current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        self.current_pose["x"] = 0.1 * math.sin(0.1 * current_time)  # Slow oscillation
        self.current_pose["y"] = 0.05 * current_time  # Slow forward motion

        msg.pose.pose.position.x = self.current_pose["x"]
        msg.pose.pose.position.y = self.current_pose["y"]
        msg.pose.pose.position.z = self.current_pose["z"]

        # Orientation
        msg.pose.pose.orientation.w = math.cos(self.current_pose["yaw"] / 2)
        msg.pose.pose.orientation.z = math.sin(self.current_pose["yaw"] / 2)

        # Covariances
        msg.pose.covariance = [
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.0,
            0.01,
        ]

        # Velocity (simulate wheel motion)
        msg.twist.twist.linear.x = 0.05  # Slow forward velocity
        msg.twist.twist.angular.z = 0.01 * math.sin(0.2 * current_time)

        self.odom_publisher.publish(msg)

        # Also publish SLAM pose for testing
        slam_msg = PoseStamped()
        slam_msg.header = msg.header
        slam_msg.header.frame_id = "map"
        slam_msg.pose = msg.pose.pose
        self.slam_pose_publisher.publish(slam_msg)

        # Publish TF transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.current_pose["x"]
        t.transform.translation.y = self.current_pose["y"]
        t.transform.translation.z = self.current_pose["z"]
        t.transform.rotation = msg.pose.pose.orientation

        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = SensorSimulator()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
