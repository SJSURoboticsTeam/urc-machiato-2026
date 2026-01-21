#!/usr/bin/env python3
"""
Teleoperation WebSocket/Socket.IO Bridge

Bidirectional communication bridge between ROS2 and teleoperation Socket.IO server.
Enables frontend gamepad control and real-time status updates.

Communication Paths:
1. Frontend → Socket.IO → ROS2 (driveCommands, driveHoming, emergencyStop)
2. ROS2 → Socket.IO → Frontend (systemStatus, diagnostics, feedback)

Author: URC 2026 Bridge Integration Team
"""

import asyncio
import socketio
import logging
from typing import Dict, Any, Optional, Callable
from dataclasses import dataclass
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist, TwistStamped
from sensor_msgs.msg import BatteryState, Imu, JointState
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
from std_msgs.msg import Bool, String

logger = logging.getLogger(__name__)


@dataclass
class TeleopConfig:
    """Configuration for teleoperation bridge."""
    server_url: str = "http://localhost:5000"
    reconnect_delay: float = 2.0
    status_publish_rate: float = 10.0  # Hz
    command_timeout: float = 0.5  # s
    enable_auto_reconnect: bool = True


class TeleopWebSocketBridge(Node):
    """
    Bidirectional Socket.IO bridge for teleoperation.
    
    Receives commands from frontend and publishes ROS2 topics.
    Subscribes to ROS2 topics and emits to frontend.
    """
    
    def __init__(self, config: Optional[TeleopConfig] = None):
        super().__init__('teleop_websocket_bridge')
        
        self.config = config or TeleopConfig()
        
        # Socket.IO client
        self.sio = socketio.AsyncClient(
            reconnection=self.config.enable_auto_reconnect,
            reconnection_delay=self.config.reconnect_delay
        )
        
        # Connection state
        self.is_connected = False
        self.connection_attempts = 0
        self.last_command_time = 0.0
        
        # Statistics
        self.commands_received = 0
        self.commands_published = 0
        self.status_sent = 0
        self.errors = 0
        
        # Setup Socket.IO event handlers
        self._setup_socketio_handlers()
        
        # Setup ROS2 publishers (for commands from frontend)
        self._setup_ros2_publishers()
        
        # Setup ROS2 subscribers (for status to frontend)
        self._setup_ros2_subscribers()
        
        # Create timers
        self.status_timer = self.create_timer(
            1.0 / self.config.status_publish_rate,
            self._publish_status_callback
        )
        
        self.command_timeout_timer = self.create_timer(
            0.1,
            self._check_command_timeout
        )
        
        # Latest data for status publishing
        self.latest_data = {
            'velocity_feedback': None,
            'battery': None,
            'imu': None,
            'diagnostics': None,
            'emergency_stop': False
        }
        
        self.get_logger().info("Teleoperation WebSocket bridge initialized")
    
    def _setup_socketio_handlers(self):
        """Setup Socket.IO event handlers."""
        
        @self.sio.event
        async def connect():
            """Handle connection to teleoperation server."""
            self.is_connected = True
            self.connection_attempts += 1
            self.get_logger().info(
                f"✓ Connected to teleoperation server: {self.config.server_url}"
            )
        
        @self.sio.event
        async def disconnect():
            """Handle disconnection from server."""
            self.is_connected = False
            self.get_logger().warning("Disconnected from teleoperation server")
        
        @self.sio.event
        async def connect_error(data):
            """Handle connection error."""
            self.errors += 1
            self.get_logger().error(f"Connection error: {data}")
        
        @self.sio.event
        async def driveCommands(data):
            """
            Handle drive commands from frontend.
            
            Expected format:
            {
                'xVel': 0.5,      # m/s (linear x)
                'yVel': 0.0,      # m/s (linear y)
                'rotVel': 15.0    # deg/s (angular z)
            }
            """
            try:
                self.commands_received += 1
                self.last_command_time = time.time()
                
                # Create ROS2 Twist message
                twist = Twist()
                twist.linear.x = float(data.get('xVel', 0.0))
                twist.linear.y = float(data.get('yVel', 0.0))
                
                # Convert deg/s to rad/s
                rot_deg = float(data.get('rotVel', 0.0))
                twist.angular.z = rot_deg * 0.0174533  # deg to rad
                
                # Publish to ROS2
                self.teleop_cmd_pub.publish(twist)
                self.commands_published += 1
                
                self.get_logger().debug(
                    f"Drive command: x={twist.linear.x:.3f} "
                    f"y={twist.linear.y:.3f} rot={twist.angular.z:.3f}"
                )
                
            except Exception as e:
                self.errors += 1
                self.get_logger().error(f"Error processing drive command: {e}")
        
        @self.sio.event
        async def driveHoming(data):
            """Handle homing sequence request."""
            try:
                self.get_logger().info("Homing sequence requested from frontend")
                
                # Publish homing request
                homing_msg = Bool()
                homing_msg.data = True
                self.homing_pub.publish(homing_msg)
                
            except Exception as e:
                self.errors += 1
                self.get_logger().error(f"Error processing homing request: {e}")
        
        @self.sio.event
        async def emergencyStop(data):
            """Handle emergency stop from frontend."""
            try:
                self.get_logger().warning("⚠️  EMERGENCY STOP from frontend")
                
                # Publish emergency stop
                stop_msg = Bool()
                stop_msg.data = True
                self.emergency_stop_pub.publish(stop_msg)
                
                # Also publish zero velocity
                zero_twist = Twist()
                self.teleop_cmd_pub.publish(zero_twist)
                
                self.latest_data['emergency_stop'] = True
                
            except Exception as e:
                self.errors += 1
                self.get_logger().error(f"Error processing emergency stop: {e}")
        
        @self.sio.event
        async def requestStatus(data):
            """Handle status request from frontend."""
            await self._emit_status()
    
    def _setup_ros2_publishers(self):
        """Setup ROS2 publishers for commands from frontend."""
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Teleop velocity commands
        self.teleop_cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel/teleop',
            qos
        )
        
        # Homing requests
        self.homing_pub = self.create_publisher(
            Bool,
            '/hardware/homing_request',
            qos
        )
        
        # Emergency stop
        self.emergency_stop_pub = self.create_publisher(
            Bool,
            '/emergency_stop',
            qos
        )
        
        self.get_logger().info("ROS2 publishers created")
    
    def _setup_ros2_subscribers(self):
        """Setup ROS2 subscribers for status to frontend."""
        
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )
        
        # Velocity feedback
        self.create_subscription(
            TwistStamped,
            '/hardware/velocity_feedback',
            self._velocity_feedback_callback,
            qos
        )
        
        # Battery status
        self.create_subscription(
            BatteryState,
            '/hardware/battery',
            self._battery_callback,
            qos
        )
        
        # IMU data
        self.create_subscription(
            Imu,
            '/hardware/imu',
            self._imu_callback,
            qos
        )
        
        # Diagnostics
        self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self._diagnostics_callback,
            qos
        )
        
        # Emergency stop status
        self.create_subscription(
            Bool,
            '/emergency_stop_active',
            self._emergency_stop_status_callback,
            qos
        )
        
        self.get_logger().info("ROS2 subscribers created")
    
    def _velocity_feedback_callback(self, msg: TwistStamped):
        """Handle velocity feedback from hardware."""
        self.latest_data['velocity_feedback'] = {
            'x': msg.twist.linear.x,
            'y': msg.twist.linear.y,
            'rot': msg.twist.angular.z * 57.2957795131  # rad/s to deg/s
        }
    
    def _battery_callback(self, msg: BatteryState):
        """Handle battery status."""
        self.latest_data['battery'] = {
            'voltage': msg.voltage,
            'current': msg.current,
            'percentage': msg.percentage,
            'temperature': msg.temperature
        }
    
    def _imu_callback(self, msg: Imu):
        """Handle IMU data."""
        self.latest_data['imu'] = {
            'orientation': {
                'x': msg.orientation.x,
                'y': msg.orientation.y,
                'z': msg.orientation.z,
                'w': msg.orientation.w
            },
            'angular_velocity': {
                'x': msg.angular_velocity.x,
                'y': msg.angular_velocity.y,
                'z': msg.angular_velocity.z
            },
            'linear_acceleration': {
                'x': msg.linear_acceleration.x,
                'y': msg.linear_acceleration.y,
                'z': msg.linear_acceleration.z
            }
        }
    
    def _diagnostics_callback(self, msg: DiagnosticArray):
        """Handle diagnostics."""
        diagnostics = []
        for status in msg.status:
            diagnostics.append({
                'name': status.name,
                'level': status.level,
                'message': status.message
            })
        self.latest_data['diagnostics'] = diagnostics
    
    def _emergency_stop_status_callback(self, msg: Bool):
        """Handle emergency stop status."""
        self.latest_data['emergency_stop'] = msg.data
    
    async def _emit_status(self):
        """Emit system status to frontend."""
        if not self.is_connected:
            return
        
        try:
            status_data = {
                'timestamp': time.time(),
                'velocity': self.latest_data['velocity_feedback'],
                'battery': self.latest_data['battery'],
                'imu': self.latest_data['imu'],
                'diagnostics': self.latest_data['diagnostics'],
                'emergency_stop': self.latest_data['emergency_stop'],
                'bridge_stats': {
                    'commands_received': self.commands_received,
                    'commands_published': self.commands_published,
                    'status_sent': self.status_sent,
                    'errors': self.errors
                }
            }
            
            await self.sio.emit('systemStatus', status_data)
            self.status_sent += 1
            
        except Exception as e:
            self.errors += 1
            self.get_logger().error(f"Error emitting status: {e}")
    
    def _publish_status_callback(self):
        """Timer callback to publish status."""
        if self.is_connected:
            # Schedule async emit in event loop
            asyncio.create_task(self._emit_status())
    
    def _check_command_timeout(self):
        """Check if commands have timed out (safety feature)."""
        if self.last_command_time == 0.0:
            return
        
        time_since_command = time.time() - self.last_command_time
        
        if time_since_command > self.config.command_timeout:
            # Send zero velocity if no commands received
            zero_twist = Twist()
            self.teleop_cmd_pub.publish(zero_twist)
            self.last_command_time = 0.0  # Reset
    
    async def connect(self):
        """Connect to teleoperation server."""
        try:
            self.get_logger().info(f"Connecting to {self.config.server_url}...")
            await self.sio.connect(self.config.server_url)
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to connect: {e}")
            return False
    
    async def disconnect(self):
        """Disconnect from server."""
        if self.is_connected:
            await self.sio.disconnect()
            self.get_logger().info("Disconnected from teleoperation server")
    
    def get_stats(self) -> Dict[str, Any]:
        """Get bridge statistics."""
        return {
            'is_connected': self.is_connected,
            'server_url': self.config.server_url,
            'connection_attempts': self.connection_attempts,
            'commands_received': self.commands_received,
            'commands_published': self.commands_published,
            'status_sent': self.status_sent,
            'errors': self.errors,
            'last_command_time': self.last_command_time
        }


async def main():
    """Main entry point for running bridge standalone."""
    rclpy.init()
    
    # Create config
    config = TeleopConfig(
        server_url="http://localhost:5000",
        status_publish_rate=10.0,
        command_timeout=0.5
    )
    
    # Create bridge
    bridge = TeleopWebSocketBridge(config)
    
    # Connect to server
    connected = await bridge.connect()
    
    if not connected:
        bridge.get_logger().error("Failed to connect to teleoperation server")
        return
    
    try:
        # Spin ROS2 node
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        bridge.get_logger().info("Shutting down...")
    finally:
        await bridge.disconnect()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    asyncio.run(main())
