#!/usr/bin/env python3
"""
SLAM ROS2 Nodes for Integration Testing

Provides depth processing and GPS fusion nodes needed for SLAM integration tests.
These are simplified mock implementations for testing purposes.
"""

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Image, Imu, NavSatFix


class DepthProcessorNode(Node):
    """ROS2 node for depth processing in SLAM."""

    def __init__(self):
        super().__init__("depth_processor_node")

        # Subscribers
        self.depth_sub = self.create_subscription(
            Image, "/camera/depth/image_raw", self.depth_callback, 10
        )

        # Publishers
        self.processed_depth_pub = self.create_publisher(
            Image, "/slam/depth/processed", 10
        )

        self.get_logger().info("Depth Processor Node initialized")

    def depth_callback(self, msg: Image):
        """Process incoming depth images."""
        # Create processed depth image (mock processing)
        processed = Image()
        processed.header = msg.header
        processed.height = msg.height
        processed.width = msg.width
        processed.encoding = msg.encoding
        processed.is_bigendian = msg.is_bigendian
        processed.step = msg.step

        # Simple mock processing: just copy the data
        processed.data = msg.data

        self.processed_depth_pub.publish(processed)


class GPSFusionNode(Node):
    """ROS2 node for GPS-IMU fusion in SLAM."""

    def __init__(self):
        super().__init__("gps_fusion_node")

        # Subscribers
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 10
        )
        self.imu_sub = self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)

        # Publishers
        self.fused_odom_pub = self.create_publisher(Odometry, "/slam/odom/fused", 10)

        # State
        self.last_gps = None
        self.last_imu = None

        self.get_logger().info("GPS Fusion Node initialized")

    def gps_callback(self, msg: NavSatFix):
        """Handle GPS data."""
        self.last_gps = msg

        # Publish fused odometry when we have both GPS and IMU
        if self.last_imu:
            self.publish_fused_odom()

    def imu_callback(self, msg: Imu):
        """Handle IMU data."""
        self.last_imu = msg

        # Publish fused odometry when we have both GPS and IMU
        if self.last_gps:
            self.publish_fused_odom()

    def publish_fused_odom(self):
        """Publish fused odometry data."""
        if not self.last_gps or not self.last_imu:
            return

        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"

        # Mock fusion: use GPS position, IMU orientation
        odom.pose.pose.position.x = self.last_gps.longitude * 1000  # Mock conversion
        odom.pose.pose.position.y = self.last_gps.latitude * 1000
        odom.pose.pose.position.z = self.last_gps.altitude

        odom.pose.pose.orientation = self.last_imu.orientation

        self.fused_odom_pub.publish(odom)


def main():
    """Main entry point - run both nodes."""
    import signal
    import sys

    def signal_handler(signum, frame):
        """Handle shutdown signals."""
        print("\n Shutting down SLAM nodes...")
        sys.exit(0)

    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rclpy.init()

    # Create nodes
    depth_processor = DepthProcessorNode()
    gps_fusion = GPSFusionNode()

    try:
        print("[IGNITE] SLAM nodes running...")
        print("[ANTENNA] Processing depth images and GPS fusion")
        # Spin both nodes
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(depth_processor)
        executor.add_node(gps_fusion)

        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        print("[PASS] SLAM nodes stopping...")
        depth_processor.destroy_node()
        gps_fusion.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
