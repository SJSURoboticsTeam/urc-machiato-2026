#!/usr/bin/env python3
"""
Integrated Bridge System Launch File

Launches all bridge components for complete ROS2 ↔ Hardware ↔ Teleoperation integration.

Components:
1. Hardware Interface Node - ROS2 to CAN bridge with protocol adapter
2. Teleoperation WebSocket Bridge - ROS2 to Socket.IO (optional)

Author: URC 2026 Integration Team
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for integrated bridge system."""
    
    # Declare launch arguments
    can_port_arg = DeclareLaunchArgument(
        'can_port',
        default_value='/dev/ttyACM0',
        description='Primary CAN serial device path'
    )
    
    can_protocol_arg = DeclareLaunchArgument(
        'can_protocol',
        default_value='teleoperation',
        description='CAN protocol adapter type (teleoperation or main)'
    )
    
    can_baudrate_arg = DeclareLaunchArgument(
        'can_baudrate',
        default_value='115200',
        description='CAN serial baudrate'
    )
    
    enable_teleop_bridge_arg = DeclareLaunchArgument(
        'enable_teleop_bridge',
        default_value='true',
        description='Enable teleoperation WebSocket bridge'
    )
    
    teleop_server_url_arg = DeclareLaunchArgument(
        'teleop_server_url',
        # Use environment-based API URL
        from infrastructure.config.environment import get_env_manager
        env = get_env_manager()
        default_value=env.get_api_url(),
        description='Teleoperation Socket.IO server URL'
    )
    
    control_rate_arg = DeclareLaunchArgument(
        'control_rate_hz',
        default_value='50.0',
        description='Control loop rate (Hz)'
    )
    
    telemetry_rate_arg = DeclareLaunchArgument(
        'telemetry_rate_hz',
        default_value='10.0',
        description='Telemetry publishing rate (Hz)'
    )
    
    # Hardware Interface Node
    hardware_interface_node = Node(
        package='hardware_interface',
        executable='hardware_interface_node',
        name='hardware_interface',
        output='screen',
        parameters=[{
            'can_port': LaunchConfiguration('can_port'),
            'can_fallback_devices': ['/dev/ttyAMA10', '/dev/ttyUSB0', '/dev/ttyACM1'],
            'can_baudrate': LaunchConfiguration('can_baudrate'),
            'can_protocol': LaunchConfiguration('can_protocol'),
            'control_rate_hz': LaunchConfiguration('control_rate_hz'),
            'telemetry_rate_hz': LaunchConfiguration('telemetry_rate_hz'),
        }],
        respawn=True,
        respawn_delay=2.0
    )
    
    # Teleoperation WebSocket Bridge (conditional)
    # Note: This is commented out until the node is packaged
    # teleop_bridge_node = Node(
    #     package='teleop_bridge',
    #     executable='teleop_websocket_bridge',
    #     name='teleop_websocket_bridge',
    #     output='screen',
    #     parameters=[{
    #         'server_url': LaunchConfiguration('teleop_server_url'),
    #         'status_publish_rate': 10.0,
    #         'command_timeout': 0.5,
    #         'enable_auto_reconnect': True
    #     }],
    #     condition=IfCondition(LaunchConfiguration('enable_teleop_bridge'))
    # )
    
    # Log startup info
    startup_log = LogInfo(
        msg=[
            '\n',
            '='*60, '\n',
            'URC 2026 Integrated Bridge System\n',
            '='*60, '\n',
            'CAN Protocol: ', LaunchConfiguration('can_protocol'), '\n',
            'CAN Port: ', LaunchConfiguration('can_port'), '\n',
            'CAN Baudrate: ', LaunchConfiguration('can_baudrate'), '\n',
            'Control Rate: ', LaunchConfiguration('control_rate_hz'), ' Hz\n',
            'Telemetry Rate: ', LaunchConfiguration('telemetry_rate_hz'), ' Hz\n',
            'Teleop Bridge: ', LaunchConfiguration('enable_teleop_bridge'), '\n',
            'Teleop Server: ', LaunchConfiguration('teleop_server_url'), '\n',
            '='*60, '\n',
            'Starting hardware interface...\n',
            '='*60
        ]
    )
    
    return LaunchDescription([
        # Launch arguments
        can_port_arg,
        can_protocol_arg,
        can_baudrate_arg,
        enable_teleop_bridge_arg,
        teleop_server_url_arg,
        control_rate_arg,
        telemetry_rate_arg,
        
        # Startup log
        startup_log,
        
        # Nodes
        hardware_interface_node,
        # teleop_bridge_node,  # Uncomment when packaged
    ])
