#!/usr/bin/env python3
"""
Odom to SLAM Pose Bridge

Republishes /odom as geometry_msgs/PoseWithCovarianceStamped on slam/pose
for testing the SLAM stack without RTAB-Map. Used when use_slam_from_odom is true
in slam.launch.py.

Author: URC 2026 Autonomy Team
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry


class OdomToSlamPoseBridge(Node):
    """Republish /odom as slam/pose (PoseWithCovarianceStamped)."""

    def __init__(self) -> None:
        super().__init__("odom_to_slam_pose_bridge")
        self.sub = self.create_subscription(Odometry, "/odom", self.on_odom, 10)
        self.pub = self.create_publisher(PoseWithCovarianceStamped, "slam/pose", 10)
        self.get_logger().info("Odom to slam/pose bridge started")

    def on_odom(self, msg: Odometry) -> None:
        out = PoseWithCovarianceStamped()
        out.header = msg.header
        out.pose = msg.pose
        self.pub.publish(out)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomToSlamPoseBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
