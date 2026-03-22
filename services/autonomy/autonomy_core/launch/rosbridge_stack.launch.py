#!/usr/bin/env python3
"""Rosbridge WebSocket + rosapi for dashboard and tooling.

Default **port 9090** matches ``services/dashboard`` (``useROS.js``, ``ROSDataSource.ts``).

Requires::

    sudo apt install ros-${ROS_DISTRO}-rosbridge-server ros-${ROS_DISTRO}-rosapi

Run::

    ros2 launch autonomy_core rosbridge_stack.launch.py

Use another port only if you also point the dashboard at ``ws://localhost:<port>``::

    ros2 launch autonomy_core rosbridge_stack.launch.py rosbridge_port:=9091
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    port = LaunchConfiguration("rosbridge_port")
    address = LaunchConfiguration("address")
    retry = LaunchConfiguration("retry_startup_delay")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "rosbridge_port",
                default_value="9090",
                description="WebSocket port (dashboard default ws://localhost:9090)",
            ),
            DeclareLaunchArgument(
                "address",
                default_value="0.0.0.0",
                description="Bind address; use 127.0.0.1 for local-only",
            ),
            DeclareLaunchArgument(
                "retry_startup_delay",
                default_value="2.0",
                description="Seconds before rosbridge retries binding the port",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Forward use_sim_time to nodes",
            ),
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_websocket",
                output="screen",
                parameters=[
                    {"port": port},
                    {"address": address},
                    {"retry_startup_delay": retry},
                    {"use_sim_time": use_sim_time},
                ],
            ),
            Node(
                package="rosapi",
                executable="rosapi_node",
                name="rosapi",
                output="screen",
                parameters=[{"use_sim_time": use_sim_time}],
            ),
        ]
    )
