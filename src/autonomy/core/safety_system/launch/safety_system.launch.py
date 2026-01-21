#!/usr/bin/env python3
"""
Launch file for Safety System Nodes
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for safety system."""

    return LaunchDescription(
        [
            Node(
                package="autonomy_safety_system",
                executable="safety_monitor",
                name="safety_monitor",
                output="screen",
            )
        ]
    )



