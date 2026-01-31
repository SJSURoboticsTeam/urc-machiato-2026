#!/usr/bin/env python3
"""
URC 2026 Unified Launch System

Single configurable launch file for all autonomy subsystems.
Supports multiple modes: simulation, hardware, competition

Usage:
    ros2 launch autonomy_core unified.launch.py mode:=simulation
    ros2 launch autonomy_core unified.launch.py mode:=hardware
    ros2 launch autonomy_core unified.launch.py mode:=competition

Author: URC 2026 Launch Team
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def get_package_path(package_name: str) -> str:
    """Get package share directory path."""
    try:
        return get_package_share_directory(package_name)
    except Exception:
        return ""


def generate_launch_description():
    """Generate the unified launch description."""
    
    # Declare launch arguments
    mode_arg = DeclareLaunchArgument(
        "mode",
        default_value="simulation",
        description="Launch mode: simulation, hardware, or competition",
        choices=["simulation", "hardware", "competition", "dev"],
    )
    
    debug_arg = DeclareLaunchArgument(
        "debug",
        default_value="false",
        description="Enable debug mode with verbose logging",
    )
    
    use_rviz_arg = DeclareLaunchArgument(
        "use_rviz",
        default_value="true",
        description="Launch RViz for visualization",
    )
    
    namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value="",
        description="Robot namespace",
    )
    
    # Configuration
    mode = LaunchConfiguration("mode")
    debug = LaunchConfiguration("debug")
    use_rviz = LaunchConfiguration("use_rviz")
    namespace = LaunchConfiguration("namespace")
    
    # Get package paths
    autonomy_core_share = get_package_path("autonomy_core")
    
    # Environment setup
    set_env = GroupAction([
        SetEnvironmentVariable("URC_ENV", mode),
        SetEnvironmentVariable("URC_DEBUG", debug),
    ])
    
    # Navigation nodes
    navigation_nodes = GroupAction(
        actions=[
            Node(
                package="autonomy_core",
                executable="navigation_node",
                name="navigation_node",
                namespace=namespace,
                output="screen",
                parameters=[{
                    "mode": mode,
                    "debug": debug,
                }],
                remappings=[
                    ("/cmd_vel", "/cmd_vel/autonomy"),
                ],
            ),
            Node(
                package="autonomy_core",
                executable="path_planner",
                name="path_planner",
                namespace=namespace,
                output="screen",
                parameters=[{
                    "mode": mode,
                }],
            ),
            Node(
                package="autonomy_core",
                executable="motion_controller",
                name="motion_controller",
                namespace=namespace,
                output="screen",
            ),
        ],
    )
    
    # Safety nodes (always active)
    safety_nodes = GroupAction(
        actions=[
            Node(
                package="autonomy_core",
                executable="safety_monitor",
                name="safety_monitor",
                namespace=namespace,
                output="screen",
                parameters=[{
                    "emergency_stop_enabled": True,
                    "mode": mode,
                }],
            ),
            Node(
                package="autonomy_core",
                executable="safety_watchdog",
                name="safety_watchdog",
                namespace=namespace,
                output="screen",
            ),
            Node(
                package="autonomy_core",
                executable="proximity_monitor",
                name="proximity_monitor",
                namespace=namespace,
                output="screen",
            ),
        ],
    )
    
    # Perception nodes (mode-dependent)
    perception_nodes = GroupAction(
        actions=[
            Node(
                package="autonomy_core",
                executable="computer_vision_node",
                name="computer_vision_node",
                namespace=namespace,
                output="screen",
                parameters=[{
                    "simulation_mode": PythonExpression(
                        ["'", mode, "' == 'simulation'"]
                    ),
                }],
            ),
            Node(
                package="autonomy_core",
                executable="slam_node",
                name="slam_node",
                namespace=namespace,
                output="screen",
            ),
        ],
    )
    
    # Simulation-specific nodes
    simulation_nodes = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'simulation'"])
        ),
        actions=[
            Node(
                package="autonomy_core",
                executable="sensor_simulator",
                name="sensor_simulator",
                namespace=namespace,
                output="screen",
            ),
            LogInfo(msg="Simulation mode: Loading simulated sensors"),
        ],
    )
    
    # Competition-specific configuration
    competition_setup = GroupAction(
        condition=IfCondition(
            PythonExpression(["'", mode, "' == 'competition'"])
        ),
        actions=[
            LogInfo(msg="Competition mode: Enabling strict safety limits"),
            SetEnvironmentVariable("URC_SAFETY_LEVEL", "HIGH"),
        ],
    )
    
    # Build launch description
    return LaunchDescription([
        # Arguments
        mode_arg,
        debug_arg,
        use_rviz_arg,
        namespace_arg,
        
        # Environment
        set_env,
        
        # Log startup
        LogInfo(msg=["Launching URC 2026 Autonomy in ", mode, " mode"]),
        
        # Safety nodes (always first)
        safety_nodes,
        
        # Navigation
        navigation_nodes,
        
        # Perception
        perception_nodes,
        
        # Mode-specific
        simulation_nodes,
        competition_setup,
    ])
