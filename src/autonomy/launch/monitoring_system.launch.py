#!/usr/bin/env python3
"""
Launch file for the complete monitoring system including QoS profiling,
safety monitoring, and adaptive telemetry.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate launch description for monitoring system."""

    # Declare launch arguments
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false',
        description='Use simulation time'
    )

    declare_qos_profiling_rate = DeclareLaunchArgument(
        'qos_profiling_rate', default_value='1.0',
        description='QoS profiling update rate (Hz)'
    )

    declare_safety_monitoring_rate = DeclareLaunchArgument(
        'safety_monitoring_rate', default_value='10.0',
        description='Safety monitoring evaluation rate (Hz)'
    )

    declare_adaptive_telemetry = DeclareLaunchArgument(
        'adaptive_telemetry_enabled', default_value='true',
        description='Enable adaptive telemetry'
    )

    declare_urc_band = DeclareLaunchArgument(
        'urc_band', default_value='900mhz',
        description='Initial URC band (900mhz or 2.4ghz)'
    )

    declare_urc_subband = DeclareLaunchArgument(
        'urc_subband', default_value='low',
        description='Initial 900MHz subband (low, mid, high)'
    )

    # QoS Profiler Node
    qos_profiler_node = Node(
        package='autonomy_state_management',
        executable='qos_profiler',
        name='qos_profiler',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'profiling_rate': LaunchConfiguration('qos_profiling_rate'),
            'alert_thresholds': {
                'latency_critical': 500.0,
                'packet_loss_critical': 0.1,
                'bandwidth_critical': 0.9
            }
        }],
        output='screen'
    )

    # Safety Monitor Node
    safety_monitor_node = Node(
        package='autonomy_state_management',
        executable='safety_monitor',
        name='safety_monitor',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'monitoring_rate': LaunchConfiguration('safety_monitoring_rate'),
            'communication_timeout': 5.0,
            'control_loop_timeout': 0.1,
            'emergency_stop_response_time': 0.05,
            'geofence_tolerance': 5.0
        }],
        output='screen'
    )

    # Enhanced Competition Bridge Node
    competition_bridge_node = Node(
        package='bridges',
        executable='competition_bridge',
        name='competition_bridge',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'adaptive_telemetry_enabled': LaunchConfiguration('adaptive_telemetry_enabled'),
            'telemetry_rate_hz': 5.0,
            'websocket_port': 8080,
            'max_websocket_clients': 10,
            'competition_log_file': 'competition_telemetry.jsonl',
            'enable_data_logging': True,
            # Adaptive telemetry parameters
            'min_telemetry_rate': 1.0,
            'max_telemetry_rate': 10.0,
            'bandwidth_target_utilization': 0.7,
            'latency_target_ms': 100.0,
            'adaptation_rate': 0.1,
            'bandwidth_measurement_window': 10.0
        }],
        output='screen'
    )

    # URC Band Configuration Service Node (for dynamic band switching)
    urc_band_manager_node = Node(
        package='autonomy_state_management',
        executable='urc_band_manager',
        name='urc_band_manager',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'initial_band': LaunchConfiguration('urc_band'),
            'initial_subband': LaunchConfiguration('urc_subband'),
            'band_switching_enabled': True,
            'interference_monitoring': True
        }],
        output='screen'
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_qos_profiling_rate,
        declare_safety_monitoring_rate,
        declare_adaptive_telemetry,
        declare_urc_band,
        declare_urc_subband,

        qos_profiler_node,
        safety_monitor_node,
        competition_bridge_node,
        urc_band_manager_node
    ])


