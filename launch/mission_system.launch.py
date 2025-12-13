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

from launch_ros.actions import Node

from launch.actions import ExecuteProcess
from launch.launch_description import LaunchDescription  # type: ignore


def generate_launch_description():
    """Generate launch description for integrated mission system"""

    # Get workspace paths for submodules
    workspace_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    teleoperation_path = os.path.join(workspace_root, 'submodules', 'teleoperation')
    control_systems_path = os.path.join(workspace_root, 'submodules', 'control-systems')

    return LaunchDescription([
        # ===========================================
        # ROS BRIDGE SERVER (for Teleoperation & Testing)
        # ===========================================
        Node(
            package='rosbridge_server',
            executable='rosbridge_websocket',
            name='rosbridge_server',
            output='screen',
            parameters=[
                {'port': 9090},
                {'address': '0.0.0.0'},
                {'retry_startup_delay': 2.0}
            ]
        ),

        # ===========================================
        # STATE MACHINE DIRECTOR
        # ===========================================
        ExecuteProcess(
            cmd=[
                'python3', '/home/ubuntu/urc-machiato-2026/tests/state_machine_director_node.py'
            ],
            output='screen',
            name='state_machine_director'
        ),

        # ===========================================
        # SLAM PROCESSING NODES
        # ===========================================
        ExecuteProcess(
            cmd=[
                'python3', '/home/ubuntu/urc-machiato-2026/tests/slam_nodes.py'
            ],
            output='screen',
            name='slam_nodes'
        ),

        # ===========================================
        # NAVIGATION SERVICES
        # ===========================================
        ExecuteProcess(
            cmd=[
                'python3', '/home/ubuntu/urc-machiato-2026/tests/navigation_service_node.py'
            ],
            output='screen',
            name='navigation_service'
        ),

        # ===========================================
        # TELEOPERATION STATUS MONITOR
        # ===========================================
        ExecuteProcess(
            cmd=['echo', f'Teleoperation submodule path: {teleoperation_path}'],
            output='screen',
            name='teleoperation_info'
        ),

        ExecuteProcess(
            cmd=['echo', f'Control systems submodule path: {control_systems_path}'],
            output='screen',
            name='control_systems_info'
        ),
    ])
