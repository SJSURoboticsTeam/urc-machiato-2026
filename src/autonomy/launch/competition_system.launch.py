#!/usr/bin/env python3
"""
Competition System Launch File

Launches all competition-ready components for URC 2026 with optimized performance:
- Hardware Interface (ROS2 ↔ STM32)
- Terrain Intelligence
- Competition Safety (Geofencing)
- Competition Bridge (Telemetry & Control)
- Autonomous Missions (Keyboard Typing, Sample Collection)

Optimized for minimal latency, memory usage, and maximum throughput.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNode, ComposableNodeContainer, Node


def generate_launch_description():
    """Generate launch description for optimized competition system."""

    # Declare launch arguments for optimization
    use_intra_process = LaunchConfiguration("use_intra_process", default="true")
    use_composable_nodes = LaunchConfiguration("use_composable_nodes", default="true")
    real_time_priority = LaunchConfiguration("real_time_priority", default="true")

    # Get package directories
    autonomy_dir = get_package_share_directory("autonomy_core")

    return LaunchDescription(
        [
            # Launch arguments
            DeclareLaunchArgument("use_intra_process", default_value="true"),
            DeclareLaunchArgument("use_composable_nodes", default_value="true"),
            DeclareLaunchArgument("real_time_priority", default_value="true"),

            # High-Performance Composable Node Container for Zero-Copy
            ComposableNodeContainer(
                name="autonomy_container",
                namespace="",
                package="rclcpp_components",
                executable="component_container_mt",
                composable_node_descriptions=[
                    # Vision Processing (Zero-Copy Source)
                    ComposableNode(
                        package="vision_processing",
                        plugin="vision_processing::VisionProcessingNode",
                        name="vision_processor",
                        extra_arguments=[{"use_intra_process_comms": use_intra_process}],
                        parameters=[{
                            "processing_rate_hz": 15.0,
                            "enable_shared_memory": True,
                            "use_zero_copy": True,
                        }],
                    ),
                    # Terrain Intelligence (Zero-Copy Consumer)
                    ComposableNode(
                        package="terrain_intelligence",
                        plugin="terrain_intelligence::TerrainAnalyzer",
                        name="terrain_analyzer",
                        extra_arguments=[{"use_intra_process_comms": use_intra_process}],
                    ),
                ],
                output="screen",
            ),

            # Hardware Interface (separate for real-time CAN communication)
            Node(
                package="missions",
                executable="autonomous_keyboard_mission",
                name="autonomous_keyboard_mission",
                output="screen",
                parameters=[
                    {
                        "keyboard_search_timeout": 60.0,
                        "navigation_timeout": 120.0,
                        "positioning_timeout": 30.0,
                        "typing_timeout": 60.0,
                        "approach_distance": 0.5,
                        "typing_sequence": "URC2026",
                    }
                ],
                arguments=["--ros-args", "--log-level", "WARN"],
            ),
            # Sample Collection Mission (uses centralized vision)
            Node(
                package="missions",
                executable="sample_collection_mission",
                name="sample_collection_mission",
                output="screen",
                parameters=[
                    {
                        "max_samples": 5,
                        "sample_search_timeout": 300.0,
                        "approach_timeout": 60.0,
                        "excavation_timeout": 120.0,
                        "sample_approach_distance": 0.2,
                        "excavation_depth": 0.15,
                        "sample_cache_slots": 6,
                    }
                ],
                arguments=["--ros-args", "--log-level", "WARN"],
            ),
            # Hardware Interface (separate for real-time CAN communication)
            Node(
                package="hardware_interface",
                executable="hardware_interface_node",
                name="hardware_interface",
                output="screen",
                parameters=[
                    {
                        "can_port": "/dev/ttyACM0",
                        "can_baudrate": 115200,
                        "control_rate_hz": 50.0,
                        "telemetry_rate_hz": 10.0,
                        "use_direct_can_bypass": True,  # Enable emergency bypass
                        "enable_message_prioritization": True,
                        "real_time_priority": real_time_priority,
                    }
                ],
                arguments=["--ros-args", "--log-level", "WARN"],
            ),
            # Competition Safety (real-time priority with direct CAN)
            Node(
                package="competition_safety",
                executable="geofencing",
                name="competition_geofencing",
                output="screen",
                parameters=[
                    {
                        "competition_boundary_file": "config/competition_boundary.json",
                        "geofence_check_rate": 1.0,
                        "boundary_violation_timeout": 5.0,
                        "emergency_stop_on_violation": True,
                        "can_port": "/dev/ttyACM0",  # Direct CAN for emergency stops
                    }
                ],
                arguments=["--ros-args", "--log-level", "WARN"],
            ),
            # Competition Communication Bridge (optimized telemetry)
            Node(
                package="competition_bridge",
                executable="competition_bridge",
                name="competition_bridge",
                output="screen",
                parameters=[
                    {
                        "websocket_port": 8080,
                        "telemetry_rate_hz": 10.0,  # Increased for better monitoring
                        "max_websocket_clients": 5,  # Reduced to minimize overhead
                        "competition_log_file": "competition_telemetry.jsonl",
                        "enable_data_logging": True,
                        "use_compressed_telemetry": True,  # Reduce bandwidth
                        "enable_message_filtering": True,  # Only send relevant data
                    }
                ],
                arguments=["--ros-args", "--log-level", "WARN"],
            ),
            # Mission Executor (existing, optimized)
            Node(
                package="mission_control",
                executable="mission_executor",
                name="mission_executor",
                output="screen",
                arguments=["--ros-args", "--log-level", "INFO"],
            ),
            # Navigation Stack (existing, optimized)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(autonomy_dir, "launch", "navigation.launch.py")]
                )
            ),
            # SLAM (existing, optimized)
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    [os.path.join(autonomy_dir, "launch", "slam.launch.py")]
                )
            ),
        ]
    )
