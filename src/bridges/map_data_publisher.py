#!/usr/bin/env python3
"""
Map Data Publisher for Frontend

Publishes map data to the frontend visualization system.
In a real system, this would come from the SLAM system.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class MapDataPublisher(Node):
    """Publishes map data for frontend visualization."""

    def __init__(self):
        super().__init__('map_data_publisher')

        self.map_pub = self.create_publisher(
            String, '/frontend/map_data', 10
        )

        # Timer to publish map data at 10Hz
        self.timer = self.create_timer(0.1, self.publish_map_data)

        self.get_logger().info('Map data publisher initialized')

    def publish_map_data(self):
        """Publish fake map data for frontend."""
        # Fake map data (would come from SLAM in real system)
        map_data = {
            "type": "occupancy_grid",
            "robot": {
                "position": {"x": 0.0, "y": 0.0, "z": 0.0},
                "orientation": {"w": 1.0, "x": 0.0, "y": 0.0, "z": 0.0},
                "timestamp": time.time()
            },
            "map": {
                "width": 100,
                "height": 100,
                "resolution": 0.1,
                "origin": {"x": -5.0, "y": -5.0, "z": 0.0},
                "data": []  # Empty map for now
            },
            "features": [],
            "timestamp": time.time()
        }

        msg = String()
        msg.data = json.dumps(map_data)
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = MapDataPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
