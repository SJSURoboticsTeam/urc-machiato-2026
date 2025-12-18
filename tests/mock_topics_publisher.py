#!/usr/bin/env python3
"""
Mock Topics Publisher for Integration Testing

Publishes mock ROS2 topics needed for integration tests.
This runs continuously in the background during testing.
"""

import signal
import sys
import threading
import time

import rclpy
from geometry_msgs.msg import Twist, TwistStamped
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Image, Imu, JointState, NavSatFix
from std_msgs.msg import Float32MultiArray, Header


class MockTopicsPublisher(Node):
    """Node that publishes mock topics for integration testing."""

    def __init__(self):
        super().__init__("mock_topics_publisher")

        # GPS publisher
        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.create_timer(1.0, self.publish_gps)

        # IMU publisher
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.create_timer(0.1, self.publish_imu)

        # Depth camera publisher
        self.depth_pub = self.create_publisher(Image, "/camera/depth/image_raw", 10)
        self.create_timer(0.5, self.publish_depth)

        # Cmd_vel publisher (static)
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 10)
        self.publish_cmd_vel()  # Publish once

        # Teleoperation topics
        self.joint_pub = self.create_publisher(
            JointState, "/teleoperation/joint_states", 10
        )
        self.chassis_pub = self.create_publisher(
            TwistStamped, "/teleoperation/chassis_velocity", 10
        )
        self.temp_pub = self.create_publisher(
            Float32MultiArray, "/teleoperation/motor_temperatures", 10
        )
        self.status_pub = self.create_publisher(
            BatteryState, "/teleoperation/system_status", 10
        )

        self.create_timer(0.2, self.publish_teleop_topics)

        self.get_logger().info("Mock topics publisher initialized")

    def publish_gps(self):
        """Publish mock GPS data."""
        msg = NavSatFix()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        msg.latitude = 35.0
        msg.longitude = -117.0
        msg.altitude = 100.0
        msg.position_covariance = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        self.gps_pub.publish(msg)

    def publish_imu(self):
        """Publish mock IMU data."""
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "imu"
        msg.orientation.w = 1.0  # Identity quaternion
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81  # Gravity
        self.imu_pub.publish(msg)

    def publish_depth(self):
        """Publish mock depth camera data."""
        msg = Image()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "camera_depth"
        msg.height = 480
        msg.width = 640
        msg.encoding = "32FC1"
        msg.is_bigendian = False
        msg.step = 640 * 4  # 4 bytes per float
        # Empty data array - just for topic presence
        msg.data = []
        self.depth_pub.publish(msg)

    def publish_cmd_vel(self):
        """Publish static cmd_vel."""
        msg = Twist()
        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = 0.0
        self.cmd_vel_pub.publish(msg)

    def publish_teleop_topics(self):
        """Publish teleoperation topics."""
        now = self.get_clock().now().to_msg()

        # Joint states
        joint_msg = JointState()
        joint_msg.header.stamp = now
        joint_msg.header.frame_id = "teleop"
        joint_msg.name = ["joint1", "joint2", "joint3"]
        joint_msg.position = [0.0, 0.0, 0.0]
        joint_msg.velocity = [0.0, 0.0, 0.0]
        joint_msg.effort = [0.0, 0.0, 0.0]
        self.joint_pub.publish(joint_msg)

        # Chassis velocity
        chassis_msg = TwistStamped()
        chassis_msg.header.stamp = now
        chassis_msg.header.frame_id = "teleop"
        self.chassis_pub.publish(chassis_msg)

        # Motor temperatures
        temp_msg = Float32MultiArray()
        temp_msg.data = [25.0, 26.0, 24.5, 25.5, 24.8, 26.2]
        self.temp_pub.publish(temp_msg)

        # System status
        status_msg = BatteryState()
        status_msg.header.stamp = now
        status_msg.voltage = 24.0
        status_msg.current = 2.0
        status_msg.charge = 95.0
        status_msg.capacity = 100.0
        status_msg.design_capacity = 100.0
        status_msg.percentage = 0.95
        self.status_pub.publish(status_msg)


def signal_handler(signum, frame):
    """Handle shutdown signals."""
    print("\n🛑 Shutting down mock topics publisher...")
    if rclpy.ok():
        rclpy.shutdown()
    sys.exit(0)


def main():
    """Main entry point."""
    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    # Initialize ROS2
    rclpy.init()

    # Create and run publisher
    publisher = MockTopicsPublisher()

    try:
        print("🚀 Mock topics publisher running...")
        print("📡 Publishing topics:")
        print("   /gps/fix")
        print("   /imu/data")
        print("   /camera/depth/image_raw")
        print("   /cmd_vel")
        print("   /teleoperation/joint_states")
        print("   /teleoperation/chassis_velocity")
        print("   /teleoperation/motor_temperatures")
        print("   /teleoperation/system_status")
        print("\nPress Ctrl+C to stop")

        rclpy.spin(publisher)
    except KeyboardInterrupt:
        pass
    finally:
        publisher.destroy_node()
        rclpy.shutdown()
        print("✅ Mock topics publisher stopped")


if __name__ == "__main__":
    main()
