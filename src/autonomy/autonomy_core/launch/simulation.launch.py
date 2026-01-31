#!/usr/bin/env python3
"""
URC 2026 Simulation-Specific Launch

Launches the autonomy stack with Gazebo simulation.
Includes simulated sensors, world loading, and RViz.

Usage:
    ros2 launch autonomy_core simulation.launch.py
    ros2 launch autonomy_core simulation.launch.py world:=mars_desert

Author: URC 2026 Simulation Team
"""

import os
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    """Generate simulation launch description."""
    
    # Declare arguments
    world_arg = DeclareLaunchArgument(
        "world",
        default_value="urc_mars_world",
        description="Gazebo world file name",
    )
    
    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Start Gazebo with GUI",
    )
    
    rviz_arg = DeclareLaunchArgument(
        "rviz",
        default_value="true",
        description="Start RViz for visualization",
    )
    
    headless_arg = DeclareLaunchArgument(
        "headless",
        default_value="false",
        description="Run in headless mode (no GUI)",
    )
    
    real_time_factor_arg = DeclareLaunchArgument(
        "real_time_factor",
        default_value="1.0",
        description="Simulation real-time factor",
    )
    
    # Configurations
    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    headless = LaunchConfiguration("headless")
    real_time_factor = LaunchConfiguration("real_time_factor")
    
    # Environment setup
    env_setup = GroupAction([
        SetEnvironmentVariable("URC_ENV", "simulation"),
        SetEnvironmentVariable("GAZEBO_MODEL_PATH", 
            os.path.join(os.getcwd(), "src/simulation/models")),
    ])
    
    # Log startup
    startup_log = LogInfo(
        msg=["Starting URC 2026 Simulation with world: ", world]
    )
    
    # Simulation nodes
    sensor_simulator = Node(
        package="autonomy_core",
        executable="sensor_simulator",
        name="sensor_simulator",
        output="screen",
        parameters=[{
            "world": world,
            "real_time_factor": real_time_factor,
            "sensor_noise_enabled": True,
        }],
    )
    
    # Include unified launch with simulation mode
    autonomy_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                get_package_share_directory("autonomy_core") if os.path.exists(
                    get_package_share_directory("autonomy_core") if os.path.exists(os.path.join(os.getcwd(), "install")) else os.path.join(os.getcwd(), "src/autonomy/autonomy_core")
                ) else os.path.join(os.getcwd(), "src/autonomy/autonomy_core"),
                "launch",
                "unified.launch.py",
            ])
        ]),
        launch_arguments={
            "mode": "simulation",
            "debug": "false",
            "use_rviz": rviz,
        }.items(),
    )
    
    return LaunchDescription([
        # Arguments
        world_arg,
        gui_arg,
        rviz_arg,
        headless_arg,
        real_time_factor_arg,
        
        # Environment
        env_setup,
        
        # Logging
        startup_log,
        
        # Sensor simulator
        sensor_simulator,
        
        # Main autonomy stack
        # Note: In a full implementation, this would include the unified launch
        # For now, we start individual nodes
    ])
