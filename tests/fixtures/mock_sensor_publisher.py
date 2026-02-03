#!/usr/bin/env python3
"""
Mock Sensor Publisher for Integration Testing

Publishes odometry and SLAM pose data with proper QoS to match BT orchestrator expectations.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
import time
import math


class MockSensorPublisher(Node):
    """Publishes sensor data with proper QoS for BT orchestrator."""

    def __init__(self):
        super().__init__("mock_sensor_publisher")

        # Use QoS matching BT orchestrator expectations (reliable, transient local, deadline)
        from rclpy.duration import Duration

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )
        # Add deadline to match BT orchestrator QoS
        try:
            qos_profile.deadline = Duration(nanoseconds=100000000)  # 100ms deadline
        except Exception:
            pass  # Deadline may not be available in all ROS2 versions

        self.odom_pub = self.create_publisher(Odometry, "/odom", qos_profile)
        self.slam_pub = self.create_publisher(PoseStamped, "/slam/pose", qos_profile)
        self.timer = self.create_timer(0.1, self.publish_data)
        self.start_time = time.time()
        self.get_logger().info("Mock sensor publisher started with proper QoS")

    def publish_data(self):
        """Publish odometry and SLAM pose data."""
        current_time = time.time() - self.start_time

        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = 0.1 * math.sin(0.1 * current_time)
        odom.pose.pose.position.y = 0.05 * current_time
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation.w = 1.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = 0.0
        odom.twist.twist.linear.x = 0.05
        odom.twist.twist.angular.z = 0.01
        self.odom_pub.publish(odom)

        # Publish SLAM pose
        slam = PoseStamped()
        slam.header.stamp = self.get_clock().now().to_msg()
        slam.header.frame_id = "map"
        slam.pose.position.x = odom.pose.pose.position.x
        slam.pose.position.y = odom.pose.pose.position.y
        slam.pose.position.z = 0.0
        slam.pose.orientation = odom.pose.pose.orientation
        self.slam_pub.publish(slam)


def main():
    rclpy.init()
    node = MockSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
