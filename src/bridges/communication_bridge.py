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
import websockets
from typing import Any, Dict, Optional

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Path
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import String, Float32
from tf2_ros import TransformBroadcaster

from simulation.can.can_bus_mock_simulator import CANBusMockSimulator
from utilities import QoSProfiles, ParameterManager, PerformanceMonitor, CommunicationError, ValidationError

class CommunicationBridge(LifecycleNode):
    """
    Unified Communication Bridge with Lifecycle management.
    """

    def __init__(self):
        super().__init__("communication_bridge")

        # Logger for standardized error handling
        from utilities import NodeLogger
        self.logger = NodeLogger(self, "communication_bridge")

        # Performance monitoring
        self.perf_monitor = PerformanceMonitor(self, "communication_bridge")

        # Initialize CAN simulator
        self.can_simulator = CANBusMockSimulator()

        # Simulation state tracking for mission tests
        self.mission_active = False
        self.current_mission = None
        self._current_status_override = None

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configure publishers and subscribers."""
        self.get_logger().info("Configuring Communication Bridge...")

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Standardized parameter management
        self.param_manager = ParameterManager(self)

        # Define parameter specifications
        param_specs = {
            "imu_frame": {
                "default": "imu_link",
                "type": str,
                "description": "Frame ID for IMU data"
            },
            "gps_frame": {
                "default": "gps_link",
                "type": str,
                "description": "Frame ID for GPS data"
            },
            "base_frame": {
                "default": "base_link",
                "type": str,
                "description": "Frame ID for robot base"
            },
            "map_frame": {
                "default": "map",
                "type": str,
                "description": "Frame ID for map coordinate system"
            }
        }

        # Declare and get parameters
        params = self.param_manager.declare_parameters(param_specs)

        # Extract parameter values
        self.imu_frame = params["imu_frame"]
        self.gps_frame = params["gps_frame"]
        self.base_frame = params["base_frame"]
        self.map_frame = params["map_frame"]

        # Publishers with standardized QoS
        self.mission_cmd_pub = self.create_lifecycle_publisher(
            String, "/mission/commands", QoSProfiles.command_data()
        )
        self.mission_status_pub = self.create_lifecycle_publisher(
            String, "/mission/status", QoSProfiles.state_data()
        )
        self.mission_progress_pub = self.create_lifecycle_publisher(
            Float32, "/mission/progress", QoSProfiles.state_data()
        )
        self.can_data_pub = self.create_lifecycle_publisher(
            String, "/can/sensor_data", QoSProfiles.sensor_data()
        )
        self.system_status_pub = self.create_lifecycle_publisher(
            String, "/system/status", QoSProfiles.diagnostic_data()
        )

        # SLAM & Frontend publishers
        self.imu_pub = self.create_lifecycle_publisher(
            Imu, "/imu/data_slam", QoSProfiles.high_frequency_sensor()
        )
        self.gps_pub = self.create_lifecycle_publisher(
            NavSatFix, "/gps/fix_slam", QoSProfiles.sensor_data()
        )
        self.map_pub = self.create_lifecycle_publisher(
            String, "/frontend/map_data", QoSProfiles.state_data()
        )

        # Subscribers
        self.can_request_sub = self.create_subscription(String, "/can/data_request", self.handle_can_request, 10)
        self.imu_sub = self.create_subscription(Imu, "/imu/data", self.imu_callback, 10)
        self.gps_sub = self.create_subscription(NavSatFix, "/gps/fix", self.gps_callback, 10)
        
        # Re-using mission command subscription from legacy bridge
        self.mission_command_sub = self.create_subscription(String, "/mission/commands", self.handle_mission_command, 10)

        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Start the WebSocket server and timers."""
        self.get_logger().info("Activating Communication Bridge...")
        self.start_websocket_server()
        self.tf_timer = self.create_timer(0.1, self.publish_tf_transforms)
        self.status_timer = self.create_timer(1.0, self.publish_mission_status)
        self.perf_timer = self.create_timer(30.0, self._log_performance_report)
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Stop timers and server."""
        self.get_logger().info("Deactivating Communication Bridge...")
        self.tf_timer.cancel()
        self.status_timer.cancel()
        return TransitionCallbackReturn.SUCCESS

    def handle_can_request(self, msg):
        """Handle CAN data requests with consistent error handling."""
        try:
            request = json.loads(msg.data)
            if not isinstance(request, dict):
                raise ValidationError("CAN request must be a JSON object")

            sensor_type = request.get("sensor_type", "imu")
            can_data = self.can_simulator.get_sensor_data(sensor_type)

            response_msg = String()
            response_msg.data = json.dumps(
                {
                    "sensor_type": sensor_type,
                    "reading": can_data,
                    "timestamp": time.time(),
                }
            )
            self.can_data_pub.publish(response_msg)
        self.perf_monitor.track_message("/can/sensor_data", len(response_msg.data))
        except (ValidationError, json.JSONDecodeError) as e:
            self.logger.warn("CAN request validation failed", error=str(e), operation="can_request")
        except Exception as e:
            self.logger.error("CAN request processing failed", error=e, operation="can_request")

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
        self.logger.info("Published mission command", command_type=command_data.get("command", "unknown"))

    def publish_system_status(self, status_data: Dict[str, Any]):
        """Publish system status update."""
        msg = String()
        msg.data = json.dumps(status_data)
        self.system_status_pub.publish(msg)

    async def websocket_handler(self, websocket, path):
        """Handle WebSocket connections."""
        self.logger.info("WebSocket connection established", path=path)
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
                    self.perf_monitor.track_message("websocket_in", len(message))

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

    def publish_mission_status(self):
        """Publish mission status and map data updates (consolidated)."""
        status = "idle"
        if self.mission_active:
            status = "active"
        if self._current_status_override:
            status = self._current_status_override

        status_data = {
            "active": self.mission_active,
            "status": status,
            "timestamp": self.get_clock().now().nanoseconds / 1e9
        }
        msg = String()
        msg.data = json.dumps(status_data)
        self.mission_status_pub.publish(msg)
        self.perf_monitor.track_message("/mission/status", len(msg.data))

        # Also publish map data for frontend
        map_data = {
            "type": "occupancy_grid",
            "robot": {
                "x": 0.0, "y": 0.0, "heading": 0.0,
                "timestamp": status_data["timestamp"]
            },
            "map": {"width": 100, "height": 100, "data": []}
        }
        map_msg = String()
        map_msg.data = json.dumps(map_data)
        self.map_pub.publish(map_msg)

    def handle_mission_command(self, msg):
        """Handle mission commands (merging logic from legacy mission bridge)."""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get("command", "").lower()
            
            if "start" in command:
                self.mission_active = True
                self._current_status_override = "active"
                self.start_mission_simulation()
            elif "stop" in command:
                self.mission_active = False
                self._current_status_override = "stopped"
                self.publish_mission_status()
        except Exception as e:
            self.get_logger().error(f"Error in mission command: {e}")

    def start_mission_simulation(self):
        """Mock mission simulation for testing."""
        def run_sim():
            for i in range(0, 101, 20):
                if not self.mission_active: break
                msg = Float32()
                msg.data = float(i)
                self.mission_progress_pub.publish(msg)
                if i == 100: self._current_status_override = "completed"
                time.sleep(0.5)
        threading.Thread(target=run_sim, daemon=True).start()

    def _log_performance_report(self):
        """Log performance metrics every 30 seconds."""
        self.perf_monitor.log_performance_summary()

    def start_websocket_server(self):
        """Start WebSocket server in background thread."""

        async def run_server():
            server = await websockets.serve(
                self.websocket_handler,
                "0.0.0.0",
                8765,
                ping_interval=None,  # Disable ping/pong for simplicity
            )
            self.get_logger().info("WebSocket server started on ws://0.0.0.0:8765")
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
