#!/usr/bin/env python3
"""
Launch file for LED Status Node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for LED status."""

    # Get package share directory
    pkg_dir = get_package_share_directory("autonomy_led_status")

    return LaunchDescription(
        [
            Node(
                package="autonomy_led_status",
                executable="led_status_node",
                name="led_status_node",
                output="screen",
                parameters=[
                    os.path.join(pkg_dir, "config", "led_status_config.yaml")
                ],
            )
        ]
    )
