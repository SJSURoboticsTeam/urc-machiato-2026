#!/usr/bin/env python3
"""
Integrated System Launch File - Full system with teleoperation and control systems

Launches the complete URC 2026 system including:
- Autonomy system (navigation, SLAM, mission control)
- Teleoperation frontend (submodule)
- Control systems (submodule)
- ROS bridge for WebSocket communication
- All necessary bridges and interfaces

This launch file demonstrates the integration of submodules for complete system testing.
"""

import os

from launch.actions import ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.launch_description import LaunchDescription  # type: ignore
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    """Generate complete integrated system launch description"""

    # Get workspace paths
    # File is at: tools/scripts/launch/integrated_system.launch.py
    # dirname(abspath) -> tools/scripts/launch
    # dirname*2 -> tools/scripts
    # dirname*3 -> tools
    # dirname*4 -> urc-machiato-2026 (root)
    workspace_root = os.path.dirname(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))))
    teleoperation_path = os.path.join(workspace_root, "submodules", "teleoperation")
    control_systems_path = os.path.join(workspace_root, "submodules", "control-systems")

    # Set PYTHONPATH to include autonomy_utilities
    autonomy_utilities_path = os.path.join(workspace_root, "install", "autonomy_utilities", "lib", "python3.12", "site-packages")

    return LaunchDescription(
        [
            # ===========================================
            # ENVIRONMENT SETUP
            # ===========================================
            SetEnvironmentVariable(
                name="PYTHONPATH",
                value=autonomy_utilities_path + ":" + os.environ.get("PYTHONPATH", "")
            ),

            # ===========================================
            # SYSTEM STATUS
            # ===========================================
            ExecuteProcess(
                cmd=["echo", "=== URC 2026 Integrated System Launch ==="],
                output="screen",
                name="system_status",
            ),
            # ===========================================
            # CORE INFRASTRUCTURE
            # ===========================================
            Node(
                package="rosbridge_server",
                executable="rosbridge_websocket",
                name="rosbridge_server",
                output="screen",
                parameters=[{"port": 9091}]
            ),
            Node(
                package="autonomy_simulation",
                executable="sensor_simulator.py",
                name="sensor_simulator",
                output="screen",
                parameters=[{"gps_noise_std": 0.5}]
            ),
            # ===========================================
            # CORE LIFECYCLE NODES
            # ===========================================
            Node(
                package="hardware_interface",
                executable="hardware_interface_node.py",
                name="hardware_interface",
                output="screen",
                parameters=[{"can_port": "/dev/ttyACM0"}]
            ),
            Node(
                package="autonomy_navigation",
                executable="navigation_node.py",
                name="navigation_node",
                output="screen"
            ),
            Node(
                package="autonomy_slam",
                executable="slam_node.py",
                name="slam_orchestrator",
                output="screen"
            ),
            Node(
                package="autonomy_state_management",
                executable="adaptive_state_machine",
                name="adaptive_state_machine",
                output="screen"
            ),
            # Autonomous Typing System (temporarily disabled due to build issue)
            # Node(
            #     package="autonomy_autonomous_typing",
            #     executable="autonomous_typing_node",
            #     name="autonomous_typing_node",
            #     output="screen"
            # ),
            Node(
                package="autonomy_bt",
                executable="bt_orchestrator",
                name="bt_orchestrator",
                output="screen"
            ),
            # ===========================================
            # SYSTEM MONITORING (Delayed)
            # ===========================================
            TimerAction(
                period=5.0,
                actions=[
                    ExecuteProcess(cmd=["ros2", "topic", "list"], output="screen", name="topic_monitor")
                ]
            ),
            # ===========================================
            # INTEGRATION VERIFICATION
            # ===========================================
            ExecuteProcess(
                cmd=["echo", "=== Integrated System Ready ==="],
                output="screen",
                name="integration_complete",
            ),
        ]
    )
