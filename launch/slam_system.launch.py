#!/usr/bin/env python3
"""
SLAM System Launch Script

Launches the complete SLAM system with RealSense camera integration.
"""

import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for SLAM system."""

    return LaunchDescription([
        # RealSense Camera Node
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='camera',
            namespace='camera',
            parameters=[{
                'camera_namespace': 'camera',
                'enable_color': True,
                'enable_depth': True,
                'color_width': 640,
                'color_height': 480,
                'depth_width': 848,
                'depth_height': 480,
                'color_fps': 30.0,
                'depth_fps': 30.0,
            }],
            output='screen',
        ),

        # Communication Bridge (handles all data routing)
        TimerAction(
            period=3.0,
            actions=[
                Node(
                    package='communication_bridge',
                    executable='communication_bridge',
                    name='communication_bridge',
                    output='screen',
                )
            ]
        ),

        # SLAM Orchestrator (consolidated SLAM processing)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='autonomy_slam',
                    executable='slam_node',
                    name='slam_orchestrator',
                    output='screen',
                )
            ]
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
