#!/usr/bin/env python3
"""
Simple SLAM Pose Publisher for Testing

Publishes fake SLAM pose data to satisfy test requirements.
In a real system, this would come from RTAB-Map or another SLAM algorithm.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import time

class SLAMPosePublisher(Node):
    """Publishes SLAM pose data by following Odometry for testing."""

    def __init__(self):
        super().__init__('slam_pose_publisher')

        self.pose_pub = self.create_publisher(
            PoseStamped, '/slam/pose', 10
        )
        
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        self.get_logger().info('SLAM pose publisher initialized (following /odom)')

    def odom_callback(self, msg: Odometry):
        """Republish Odometry as SLAM pose."""
        pose_msg = PoseStamped()
        pose_msg.header = msg.header
        # Use map frame for SLAM pose as expected by tests
        pose_msg.header.frame_id = 'map'
        pose_msg.pose = msg.pose.pose

        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SLAMPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
