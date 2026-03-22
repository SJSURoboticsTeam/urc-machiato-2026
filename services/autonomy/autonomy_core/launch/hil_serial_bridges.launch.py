#!/usr/bin/env python3
"""HIL: rosbridge (WebSocket) + serial CAN hardware_interface in one launch.

- WebSocket: same stack as ``rosbridge_stack.launch.py`` (default port **9090**).
- CAN: ``hardware_interface`` uses serial ``can_port`` (SLCAN-style), defaulting from
  env ``URC_STM32_SERIAL`` or ``/dev/ttyACM0``.

Example::

    source install/setup.bash
    ros2 launch autonomy_core hil_serial_bridges.launch.py

With explicit device::

    ros2 launch autonomy_core hil_serial_bridges.launch.py can_port:=/dev/ttyUSB0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    pkg_share = get_package_share_directory("autonomy_core")
    rosbridge_launch = os.path.join(pkg_share, "launch", "rosbridge_stack.launch.py")

    can_port = LaunchConfiguration("can_port")
    can_baudrate = LaunchConfiguration("can_baudrate")
    rosbridge_port = LaunchConfiguration("rosbridge_port")
    address = LaunchConfiguration("address")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "can_port",
                default_value=EnvironmentVariable(
                    "URC_STM32_SERIAL", default_value="/dev/ttyACM0"
                ),
                description="Serial device for STM32/SLCAN (or set URC_STM32_SERIAL)",
            ),
            DeclareLaunchArgument(
                "can_baudrate",
                default_value="115200",
                description="Serial baud rate for can_port",
            ),
            DeclareLaunchArgument(
                "rosbridge_port",
                default_value="9090",
                description="Rosbridge WebSocket port",
            ),
            DeclareLaunchArgument(
                "address",
                default_value="0.0.0.0",
                description="Rosbridge bind address",
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(rosbridge_launch),
                launch_arguments=[
                    ("rosbridge_port", rosbridge_port),
                    ("address", address),
                ],
            ),
            Node(
                package="autonomy_core",
                executable="hardware_interface",
                name="hardware_interface",
                output="screen",
                parameters=[
                    {"can_port": can_port, "can_baudrate": can_baudrate},
                ],
            ),
        ]
    )
