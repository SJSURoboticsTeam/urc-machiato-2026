#!/usr/bin/env python3
"""
Mission System Launch File - Integrated with teleoperation and control systems submodules

Launches:
- Mission Executor (autonomy system)
- WebSocket Mission Bridge (ROS <-> WebSocket communication)
- ROS Bridge Server (for teleoperation frontend)
- Control Systems Bridge (for drive/arm control via submodules)
- Teleoperation integration (references submodule)
"""

import os

from launch.actions import ExecuteProcess
from launch.launch_description import LaunchDescription  # type: ignore
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for integrated mission system"""

    # Get workspace paths for submodules
    # File is at: tools/scripts/launch/mission_system.launch.py
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    teleoperation_path = os.path.join(workspace_root, "submodules", "teleoperation")
    control_systems_path = os.path.join(workspace_root, "submodules", "control-systems")

    return LaunchDescription(
        [
            # ===========================================
            # ROS BRIDGE SERVER (for Teleoperation & Testing)
            # ===========================================
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_server",
                output="screen",
                parameters=[
                    {"port": 9090},
                    {"address": "0.0.0.0"},
                    {"retry_startup_delay": 2.0},
                ],
            ),
            # ===========================================
            # CORE AUTONOMY NODES (Lifecycle-managed)
            # ===========================================
            Node(
                package="autonomy_state_management",
                executable="adaptive_state_machine",
                name="adaptive_state_machine",
                output="screen"
            ),
            Node(
                package="autonomy_slam",
                executable="slam_node",
                name="slam_orchestrator",
                output="screen"
            ),
            Node(
                package="autonomy_navigation",
                executable="navigation_node",
                name="navigation_node",
                output="screen"
            ),
            # ===========================================
            # TELEOPERATION STATUS MONITOR
            # ===========================================
            ExecuteProcess(
                cmd=["echo", f"Teleoperation submodule path: {teleoperation_path}"],
                output="screen",
                name="teleoperation_info",
            ),
            ExecuteProcess(
                cmd=["echo", f"Control systems submodule path: {control_systems_path}"],
                output="screen",
                name="control_systems_info",
            ),
        ]
    )
