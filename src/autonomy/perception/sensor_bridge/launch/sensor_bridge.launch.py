#!/usr/bin/env python3
"""
Launch file for Sensor Bridge Node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for sensor bridge."""

    # Get package share directory
    pkg_dir = get_package_share_directory("autonomy_sensor_bridge")

    return LaunchDescription(
        [
            Node(
                package="autonomy_sensor_bridge",
                executable="websocket_bridge",
                name="websocket_bridge",
                output="screen",
                parameters=[
                    os.path.join(pkg_dir, "config", "sensor_bridge.yaml")
                ],
            )
        ]
    )



