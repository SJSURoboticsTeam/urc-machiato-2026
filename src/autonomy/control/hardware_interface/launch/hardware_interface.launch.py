#!/usr/bin/env python3
"""
Launch file for Hardware Interface Node
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for hardware interface."""

    # Get package share directory
    pkg_dir = get_package_share_directory("hardware_interface")

    return LaunchDescription(
        [
            Node(
                package="hardware_interface",
                executable="hardware_interface_node",
                name="hardware_interface",
                output="screen",
                parameters=[
                    {
                        "can_port": "/dev/ttyACM0",
                        "can_baudrate": 115200,
                        "control_rate_hz": 50.0,
                        "telemetry_rate_hz": 10.0,
                    }
                ],
                remappings=[
                    # ROS2 standard topic remappings
                    ("/cmd_vel", "/cmd_vel"),
                    ("/hardware/joint_states", "/hardware/joint_states"),
                    ("/hardware/chassis_velocity", "/hardware/chassis_velocity"),
                ],
            )
        ]
    )
