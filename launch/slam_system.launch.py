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

        # Camera Bridge (delay to let camera start)
        TimerAction(
            period=3.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', 'src/bridges/camera_bridge.py'],
                    output='screen',
                    name='camera_bridge'
                )
            ]
        ),

        # SLAM Depth Processor (delay to let camera bridge start)
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package='autonomy_slam',
                    executable='depth_processor',
                    name='depth_processor',
                    output='screen',
                )
            ]
        ),

        # SLAM Pose Publisher (delay to let depth processor start)
        TimerAction(
            period=7.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', 'src/bridges/slam_pose_publisher.py'],
                    output='screen',
                    name='slam_pose_publisher'
                )
            ]
        ),

        # Map Data Publisher (delay to let pose publisher start)
        TimerAction(
            period=9.0,
            actions=[
                ExecuteProcess(
                    cmd=['python3', 'src/bridges/map_data_publisher.py'],
                    output='screen',
                    name='map_data_publisher'
                )
            ]
        ),
    ])


if __name__ == '__main__':
    generate_launch_description()
