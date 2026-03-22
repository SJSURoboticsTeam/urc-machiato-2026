#!/usr/bin/env python3
"""
SLAM Pipeline Launch for URC 2026

Launches RealSense D435 driver, depth processor, optional odom->slam/pose bridge,
and SLAM orchestrator. Use when running with RealSense hardware or simulation
that provides /odom. For full RTAB-Map, launch rtabmap_ros separately and set
use_slam_from_odom:=false so slam/pose comes from RTAB-Map.

Startup order: realsense_driver (or external camera) -> depth_processor;
slam/pose from RTAB-Map or odom_to_slam_pose_bridge; slam_orchestrator is
lifecycle-managed (configure then activate).

Usage:
  ros2 launch autonomy_core slam.launch.py
  ros2 launch autonomy_core slam.launch.py use_realsense:=false use_slam_from_odom:=true
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def get_package_config_path(package: str, config_file: str) -> str:
    """Return path to config file in package share, or empty if not found."""
    try:
        pkg_share = get_package_share_directory(package)
        return os.path.join(pkg_share, "config", config_file)
    except Exception:
        return ""


def generate_launch_description() -> LaunchDescription:
    use_realsense = LaunchConfiguration("use_realsense", default="true")
    use_slam_from_odom = LaunchConfiguration("use_slam_from_odom", default="true")
    realsense_params = get_package_config_path("autonomy_core", "realsense_d435.yaml")

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "use_realsense",
                default_value="true",
                description="Launch RealSense D435 driver (set false if using realsense2_camera or sim).",
            ),
            DeclareLaunchArgument(
                "use_slam_from_odom",
                default_value="true",
                description="Republish /odom as slam/pose (set false when using RTAB-Map).",
            ),
            # RealSense D435 driver (publishes camera/rgb/image_raw, camera/depth/image_raw, camera/depth/camera_info)
            Node(
                package="autonomy_core",
                executable="realsense_driver",
                name="realsense_driver",
                output="screen",
                parameters=(
                    [realsense_params]
                    if realsense_params and os.path.isfile(realsense_params)
                    else []
                ),
                condition=IfCondition(use_realsense),
            ),
            # Depth processor (subscribes to camera/*, publishes slam/depth/processed, slam/rgb/image, slam/depth/camera_info)
            Node(
                package="autonomy_core",
                executable="depth_processor",
                name="depth_processor",
                output="screen",
            ),
            # Optional: republish /odom as slam/pose for testing without RTAB-Map
            Node(
                package="autonomy_core",
                executable="odom_to_slam_pose_bridge",
                name="odom_to_slam_pose_bridge",
                output="screen",
                condition=IfCondition(use_slam_from_odom),
            ),
            # SLAM orchestrator (lifecycle node: subscribes to slam/pose, slam/pose/fused, /odom; writes blackboard)
            Node(
                package="autonomy_core",
                executable="slam_node",
                name="slam_orchestrator",
                output="screen",
            ),
            # Lifecycle: configure slam_orchestrator after 2s
            TimerAction(
                period=2.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "lifecycle",
                            "set",
                            "/slam_orchestrator",
                            "configure",
                        ],
                        name="lifecycle_configure_slam",
                        output="screen",
                    ),
                ],
            ),
            # Lifecycle: activate slam_orchestrator after 3s
            TimerAction(
                period=3.0,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            "ros2",
                            "lifecycle",
                            "set",
                            "/slam_orchestrator",
                            "activate",
                        ],
                        name="lifecycle_activate_slam",
                        output="screen",
                    ),
                ],
            ),
        ]
    )
