#!/usr/bin/env python3
"""
Adaptive State Machine Launch File

Launches the complete adaptive state machine system with context monitoring,
policy engine, and dashboard integration.
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition


def generate_launch_description():
    """Generate launch description for adaptive state machine."""

    # Declare launch arguments
    adaptive_enabled_arg = DeclareLaunchArgument(
        'adaptive_enabled',
        default_value='true',
        description='Enable adaptive state machine features'
    )

    context_update_rate_arg = DeclareLaunchArgument(
        'context_update_rate',
        default_value='1.0',
        description='Context monitoring update rate (Hz)'
    )

    dashboard_update_rate_arg = DeclareLaunchArgument(
        'dashboard_update_rate',
        default_value='2.0',
        description='Dashboard update rate (Hz)'
    )

    adaptation_check_rate_arg = DeclareLaunchArgument(
        'adaptation_check_rate',
        default_value='0.5',
        description='Adaptation policy check rate (Hz)'
    )

    # Get package directory
    state_management_dir = get_package_share_directory('autonomy_state_machine')

    # Adaptive State Machine Node
    adaptive_state_machine_node = Node(
        package='autonomy_state_machine',
        executable='adaptive_state_machine',
        name='adaptive_state_machine',
        output='screen',
        parameters=[{
            'enable_adaptive_transitions': LaunchConfiguration('adaptive_enabled'),
            'context_update_rate': LaunchConfiguration('context_update_rate'),
            'adaptation_check_rate': LaunchConfiguration('adaptation_check_rate'),
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    # Monitoring Service Node
    monitoring_service_node = Node(
        package='autonomy_state_machine',
        executable='monitoring_service',
        name='monitoring_service',
        output='screen',
        parameters=[{
            'dashboard_update_rate': LaunchConfiguration('dashboard_update_rate'),
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    # Context Evaluator Node (standalone for enhanced monitoring)
    context_evaluator_node = Node(
        package='autonomy_state_machine',
        executable='context_evaluator_node',
        name='context_evaluator',
        output='screen',
        parameters=[{
            'update_rate': LaunchConfiguration('context_update_rate'),
        }],
        arguments=['--ros-args', '--log-level', 'DEBUG'],
    )

    # Adaptive Policy Engine Node (standalone for policy testing)
    policy_engine_node = Node(
        package='autonomy_state_machine',
        executable='adaptive_policy_engine_node',
        name='adaptive_policy_engine',
        output='screen',
        parameters=[{
            'check_rate': LaunchConfiguration('adaptation_check_rate'),
        }],
        arguments=['--ros-args', '--log-level', 'INFO'],
    )

    # Dashboard Bridge Node (for web interface integration)
    dashboard_bridge_node = Node(
        package='autonomy_state_machine',
        executable='dashboard_bridge',
        name='dashboard_bridge',
        output='screen',
        parameters=[{
            'websocket_port': 8081,  # Different from main competition bridge
            'update_rate': LaunchConfiguration('dashboard_update_rate'),
        }],
        arguments=['--ros-args', '--log-level', 'WARN'],
        condition=IfCondition(LaunchConfiguration('adaptive_enabled'))
    )

    # Create launch description
    return LaunchDescription([
        # Launch arguments
        adaptive_enabled_arg,
        context_update_rate_arg,
        dashboard_update_rate_arg,
        adaptation_check_rate_arg,

        # Core adaptive system
        GroupAction([
            adaptive_state_machine_node,
            monitoring_service_node,
        ]),

        # Optional enhanced monitoring (enable with adaptive_enabled=true)
        GroupAction([
            context_evaluator_node,
            policy_engine_node,
            dashboard_bridge_node,
        ], condition=IfCondition(LaunchConfiguration('adaptive_enabled'))),
    ])


if __name__ == '__main__':
    generate_launch_description()
