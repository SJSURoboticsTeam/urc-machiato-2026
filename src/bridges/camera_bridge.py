#!/usr/bin/env python3
"""
RealSense Camera Bridge for SLAM

Bridges RealSense camera topics to the topics expected by the SLAM depth processor.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo


class CameraBridge(Node):
    """Bridge RealSense camera topics to SLAM-expected topics."""

    def __init__(self):
        super().__init__('camera_bridge')

        # Publishers for SLAM-expected topics
        self.rgb_pub = self.create_publisher(
            Image, 'camera/rgb/image_raw', 10
        )
        self.depth_pub = self.create_publisher(
            Image, 'camera/depth/image_raw', 10
        )
        self.depth_info_pub = self.create_publisher(
            CameraInfo, 'camera/depth/camera_info', 10
        )

        # Subscribers to RealSense topics
        self.color_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw', self.color_callback, 10
        )
        self.depth_sub = self.create_subscription(
            Image, '/camera/camera/depth/image_rect_raw', self.depth_callback, 10
        )
        self.depth_info_sub = self.create_subscription(
            CameraInfo, '/camera/camera/depth/camera_info', self.depth_info_callback, 10
        )

        self.get_logger().info('Camera bridge initialized - bridging RealSense to SLAM topics')

    def color_callback(self, msg):
        """Forward color image to SLAM RGB topic."""
        self.rgb_pub.publish(msg)

    def depth_callback(self, msg):
        """Forward depth image to SLAM depth topic."""
        self.depth_pub.publish(msg)

    def depth_info_callback(self, msg):
        """Forward depth camera info to SLAM camera info topic."""
        self.depth_info_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
