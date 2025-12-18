#!/usr/bin/env python3
"""
Communication Bridge - Unified WebSocket to ROS2 Bridge

Consolidates WebSocket connectivity, ROS2 publishing, and data routing for:
- Mission commands
- SLAM sensor data (IMU, GPS)
- CAN bus mock data
- System status messages

This replaces websocket_mission_bridge.py, websocket_slam_bridge.py, and slam_data_bridge.py
"""

import asyncio
import json
import threading
import time
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster

from simulation.can.can_bus_mock_simulator import CANBusMockSimulator


class CommunicationBridge(Node):
    """
    Unified Communication Bridge

    Handles all WebSocket to ROS2 data flow:
    - WebSocket connections and message routing
    - Mission command forwarding
    - SLAM sensor data publishing
    - CAN bus mock data simulation
    - System status broadcasting
    """

    def __init__(self):
        super().__init__("communication_bridge")

        # Initialize CAN simulator
        self.can_simulator = CANBusMockSimulator()

        # TF broadcaster for coordinate frames
        self.tf_broadcaster = TransformBroadcaster(self)

        # Frame IDs for SLAM
        self.declare_parameter("imu_frame", "imu_link")
        self.declare_parameter("gps_frame", "gps_link")
        self.declare_parameter("base_frame", "base_link")
        self.declare_parameter("map_frame", "map")

        self.imu_frame = self.get_parameter("imu_frame").value
        self.gps_frame = self.get_parameter("gps_frame").value
        self.base_frame = self.get_parameter("base_frame").value
        self.map_frame = self.get_parameter("map_frame").value

        # Publishers for mission control
        self.mission_cmd_pub = self.create_publisher(String, "/mission/commands", 10)
        self.can_data_pub = self.create_publisher(String, "/can/sensor_data", 10)
        self.system_status_pub = self.create_publisher(String, "/system/status", 10)

        # Publishers for SLAM system
        self.imu_pub = self.create_publisher(Imu, "/imu/data_slam", 10)
        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix_slam", 10)

        # Publishers for frontend
        self.map_pub = self.create_publisher(String, "/frontend/map_data", 10)
        self.pose_pub = self.create_publisher(PoseStamped, "/frontend/robot_pose", 10)
        self.path_pub = self.create_publisher(Path, "/frontend/robot_path", 10)

        # Subscribers for CAN data requests
        self.can_request_sub = self.create_subscription(
            String, "/can/data_request", self.handle_can_request, 10
        )

        # Subscribers to WebSocket data (would come from external WebSocket server)
        self.imu_sub = self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 10
        )

        # Message buffer for batch processing
        self.message_buffer = []
        self.buffer_lock = threading.Lock()

        self.get_logger().info("Communication Bridge initialized")

    def handle_can_request(self, msg):
        """Handle CAN data requests."""
        try:
            request = json.loads(msg.data)
            can_data = self.can_simulator.get_sensor_data(
                request.get("sensor_type", "imu")
            )
            response_msg = String()
            response_msg.data = json.dumps(
                {
                    "sensor_type": request.get("sensor_type", "imu"),
                    "reading": can_data,
                    "timestamp": time.time(),
                }
            )
            self.can_data_pub.publish(response_msg)
        except Exception as e:
            self.get_logger().error(f"Error handling CAN request: {e}")

    def imu_callback(self, msg: Imu):
        """Process IMU data for SLAM."""
        # Republish with proper frame ID for SLAM
        slam_msg = Imu()
        slam_msg.header = msg.header
        slam_msg.header.frame_id = self.imu_frame
        slam_msg.orientation = msg.orientation
        slam_msg.orientation_covariance = msg.orientation_covariance
        slam_msg.angular_velocity = msg.angular_velocity
        slam_msg.angular_velocity_covariance = msg.angular_velocity_covariance
        slam_msg.linear_acceleration = msg.linear_acceleration
        slam_msg.linear_acceleration_covariance = msg.linear_acceleration_covariance

        self.imu_pub.publish(slam_msg)

        # Publish simplified data for frontend
        frontend_msg = String()
        frontend_msg.data = json.dumps(
            {
                "type": "imu",
                "accel": {
                    "x": msg.linear_acceleration.x,
                    "y": msg.linear_acceleration.y,
                    "z": msg.linear_acceleration.z,
                },
                "gyro": {
                    "x": msg.angular_velocity.x,
                    "y": msg.angular_velocity.y,
                    "z": msg.angular_velocity.z,
                },
                "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            }
        )
        self.system_status_pub.publish(frontend_msg)

    def gps_callback(self, msg: NavSatFix):
        """Process GPS data for SLAM."""
        # Republish with proper frame ID for SLAM
        slam_msg = NavSatFix()
        slam_msg.header = msg.header
        slam_msg.header.frame_id = self.gps_frame
        slam_msg.latitude = msg.latitude
        slam_msg.longitude = msg.longitude
        slam_msg.altitude = msg.altitude
        slam_msg.position_covariance = msg.position_covariance
        slam_msg.position_covariance_type = msg.position_covariance_type

        self.gps_pub.publish(slam_msg)

        # Publish simplified data for frontend
        frontend_msg = String()
        frontend_msg.data = json.dumps(
            {
                "type": "gps",
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "altitude": msg.altitude,
                "timestamp": msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9,
            }
        )
        self.system_status_pub.publish(frontend_msg)

    def publish_mission_command(self, command_data: Dict[str, Any]):
        """Publish mission command to ROS2."""
        msg = String()
        msg.data = json.dumps(command_data)
        self.mission_cmd_pub.publish(msg)
        self.get_logger().info(f"Published mission command: {command_data}")

    def publish_system_status(self, status_data: Dict[str, Any]):
        """Publish system status update."""
        msg = String()
        msg.data = json.dumps(status_data)
        self.system_status_pub.publish(msg)

    async def websocket_handler(self, websocket, path):
        """Handle WebSocket connections."""
        self.get_logger().info(f"WebSocket connection established: {path}")
        try:
            async for message in websocket:
                try:
                    data = json.loads(message)
                    message_type = data.get("type", "unknown")

                    if message_type == "mission_command":
                        self.publish_mission_command(data)
                    elif message_type == "system_status":
                        self.publish_system_status(data)
                    elif message_type == "can_request":
                        # Handle CAN requests via ROS2 topic
                        can_msg = String()
                        can_msg.data = json.dumps(data)
                        # This would normally go to a CAN request topic
                        self.get_logger().info(f"CAN request: {data}")
                    else:
                        self.get_logger().warning(
                            f"Unknown message type: {message_type}"
                        )

                    # Echo acknowledgment
                    await websocket.send(
                        json.dumps({"status": "received", "type": message_type})
                    )

                except json.JSONDecodeError:
                    self.get_logger().error(f"Invalid JSON message: {message}")
                    await websocket.send(
                        json.dumps({"status": "error", "message": "Invalid JSON"})
                    )
                except Exception as e:
                    self.get_logger().error(f"Error processing message: {e}")
                    await websocket.send(
                        json.dumps({"status": "error", "message": str(e)})
                    )

        except websockets.exceptions.ConnectionClosed:
            self.get_logger().info("WebSocket connection closed")
        except Exception as e:
            self.get_logger().error(f"WebSocket handler error: {e}")

    def start_websocket_server(self):
        """Start WebSocket server in background thread."""

        async def run_server():
            server = await websockets.serve(
                self.websocket_handler,
                "localhost",
                8766,
                ping_interval=None,  # Disable ping/pong for simplicity
            )
            self.get_logger().info("WebSocket server started on ws://localhost:8766")
            await server.wait_closed()

        def run_async():
            asyncio.run(run_server())

        websocket_thread = threading.Thread(target=run_async, daemon=True)
        websocket_thread.start()

    def publish_tf_transforms(self):
        """Publish static TF transforms for coordinate frames."""
        # Base to IMU transform (example - adjust as needed)
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = self.base_frame
        transform.child_frame_id = self.imu_frame
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.1
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)

    # Create and start the bridge
    bridge = CommunicationBridge()

    # Start WebSocket server
    bridge.start_websocket_server()

    # Create timer for periodic tasks
    bridge.create_timer(0.1, bridge.publish_tf_transforms)  # 10Hz TF updates

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
