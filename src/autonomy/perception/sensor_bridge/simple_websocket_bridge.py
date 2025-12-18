#!/usr/bin/env python3
"""
Simple WebSocket Bridge - Essential ROS2 WebSocket communication

Converts WebSocket JSON data to ROS2 topics for sensor data.
Simplified version focusing on core functionality.
"""

import asyncio
import json
from typing import Optional

import rclpy
import websockets
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import Float32


class SimpleWebsocketBridge(Node):
    """
    Simple WebSocket to ROS2 bridge.

    Receives JSON sensor data via WebSocket and publishes to ROS2 topics.
    """

    def __init__(self):
        super().__init__("websocket_bridge")

        # ROS2 publishers
        self.imu_pub = self.create_publisher(Imu, "/imu/data", 10)
        self.gps_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.battery_pub = self.create_publisher(BatteryState, "/battery/state", 10)
        self.temp_pub = self.create_publisher(Float32, "/temperature", 10)

        # WebSocket connection
        self.websocket: Optional[websockets.WebSocketServerProtocol] = None
        self.websocket_url = self.declare_parameter(
            "websocket_url", "ws://localhost:8765"
        ).value

        # Start WebSocket server
        self.create_timer(1.0, self._connect_websocket)

        self.get_logger().info("Simple WebSocket Bridge initialized")

    def _connect_websocket(self):
        """Attempt to connect to WebSocket server."""
        if self.websocket:
            return  # Already connected

        try:
            # Simple WebSocket client connection
            import threading

            threading.Thread(target=self._websocket_loop, daemon=True).start()
        except Exception as e:
            self.get_logger().error(f"WebSocket connection failed: {e}")

    def _websocket_loop(self):
        """WebSocket client loop."""

        async def connect():
            try:
                async with websockets.connect(self.websocket_url) as websocket:
                    self.websocket = websocket
                    self.get_logger().info(
                        f"Connected to WebSocket: {self.websocket_url}"
                    )

                    async for message in websocket:
                        try:
                            data = json.loads(message)
                            self._process_sensor_data(data)
                        except json.JSONDecodeError:
                            self.get_logger().warn("Invalid JSON received")
                        except Exception as e:
                            self.get_logger().error(f"Error processing message: {e}")

            except Exception as e:
                self.get_logger().error(f"WebSocket error: {e}")
                self.websocket = None

        asyncio.run(connect())

    def _process_sensor_data(self, data: dict):
        """Process incoming sensor data and publish to ROS2 topics."""

        # IMU data
        if "imu" in data:
            imu_msg = Imu()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu"

            imu_data = data["imu"]
            imu_msg.linear_acceleration.x = imu_data.get("accel_x", 0.0)
            imu_msg.linear_acceleration.y = imu_data.get("accel_y", 0.0)
            imu_msg.linear_acceleration.z = imu_data.get("accel_z", 9.81)

            imu_msg.angular_velocity.x = imu_data.get("gyro_x", 0.0)
            imu_msg.angular_velocity.y = imu_data.get("gyro_y", 0.0)
            imu_msg.angular_velocity.z = imu_data.get("gyro_z", 0.0)

            self.imu_pub.publish(imu_msg)

        # GPS data
        if "gps" in data:
            gps_msg = NavSatFix()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = "gps"

            gps_data = data["gps"]
            gps_msg.latitude = gps_data.get("latitude", 0.0)
            gps_msg.longitude = gps_data.get("longitude", 0.0)
            gps_msg.altitude = gps_data.get("altitude", 0.0)

            self.gps_pub.publish(gps_msg)

        # Battery data
        if "battery" in data:
            battery_msg = BatteryState()
            battery_msg.header.stamp = self.get_clock().now().to_msg()

            battery_data = data["battery"]
            battery_msg.voltage = battery_data.get("voltage", 24.0)
            battery_msg.current = battery_data.get("current", 0.0)
            battery_msg.percentage = battery_data.get("charge_level", 100.0) / 100.0

            self.battery_pub.publish(battery_msg)

        # Temperature data
        if "imu" in data and "temperature" in data["imu"]:
            temp_msg = Float32()
            temp_msg.data = data["imu"]["temperature"]
            self.temp_pub.publish(temp_msg)


def main():
    """Main entry point."""
    rclpy.init()
    node = SimpleWebsocketBridge()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
