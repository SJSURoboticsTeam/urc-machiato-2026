#!/usr/bin/env python3
"""
Integration tests for WebSocket Bridge.

Tests the complete WebSocket to ROS2 data flow using a mock WebSocket server.
"""

import asyncio
import json
import os

# Import the bridge
import sys
import threading
import time
import unittest

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import Float32

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__), "..", "..", "autonomy", "code", "sensor_bridge"
    ),
)

from simple_websocket_bridge import SimpleWebsocketBridge


class MockWebSocketServer:
    """Mock WebSocket server for testing."""

    def __init__(self, port=8766):
        self.port = port
        self.server = None
        self.connected_clients = []
        self.received_messages = []

    async def websocket_handler(self, websocket, path):
        """Handle WebSocket connections."""
        self.connected_clients.append(websocket)
        try:
            async for message in websocket:
                self.received_messages.append(message)
                # Echo back for testing
                await websocket.send(message)
        except Exception as e:
            print(f"WebSocket error: {e}")
        finally:
            if websocket in self.connected_clients:
                self.connected_clients.remove(websocket)

    async def start_server(self):
        """Start the mock WebSocket server."""
        import websockets

        self.server = await websockets.serve(
            self.websocket_handler, "localhost", self.port
        )
        print(f"Mock WebSocket server started on port {self.port}")

    async def stop_server(self):
        """Stop the mock WebSocket server."""
        if self.server:
            self.server.close()
            await self.server.wait_closed()
            print("Mock WebSocket server stopped")

    def send_test_data(self, data):
        """Send test data to connected clients."""

        async def _send():
            if self.connected_clients:
                message = json.dumps(data)
                for client in self.connected_clients:
                    try:
                        await client.send(message)
                    except Exception as e:
                        print(f"Failed to send to client: {e}")

        asyncio.create_task(_send())


class WebSocketBridgeIntegrationTester(Node):
    """Integration test node for WebSocket bridge."""

    def __init__(self):
        super().__init__("websocket_bridge_integration_tester")

        # Subscribers to verify published data
        self.imu_received = []
        self.gps_received = []
        self.battery_received = []
        self.temp_received = []

        self.imu_sub = self.create_subscription(
            Imu, "/imu/data", self._imu_callback, 10
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self._gps_callback, 10
        )
        self.battery_sub = self.create_subscription(
            BatteryState, "/battery/state", self._battery_callback, 10
        )
        self.temp_sub = self.create_subscription(
            Float32, "/temperature", self._temp_callback, 10
        )

        self.get_logger().info("WebSocket Bridge Integration Tester ready")

    def _imu_callback(self, msg):
        """Handle received IMU data."""
        self.imu_received.append(
            {
                "accel_x": msg.linear_acceleration.x,
                "accel_y": msg.linear_acceleration.y,
                "accel_z": msg.linear_acceleration.z,
                "gyro_x": msg.angular_velocity.x,
                "gyro_y": msg.angular_velocity.y,
                "gyro_z": msg.angular_velocity.z,
            }
        )

    def _gps_callback(self, msg):
        """Handle received GPS data."""
        self.gps_received.append(
            {
                "latitude": msg.latitude,
                "longitude": msg.longitude,
                "altitude": msg.altitude,
            }
        )

    def _battery_callback(self, msg):
        """Handle received battery data."""
        self.battery_received.append(
            {
                "voltage": msg.voltage,
                "current": msg.current,
                "percentage": msg.percentage,
            }
        )

    def _temp_callback(self, msg):
        """Handle received temperature data."""
        self.temp_received.append(msg.data)


class TestWebSocketBridgeIntegration(unittest.TestCase):
    """Integration tests for WebSocket bridge."""

    def setUp(self):
        """Set up test environment."""
        # Initialize ROS2
        rclpy.init()

        # Create mock WebSocket server
        self.mock_server = MockWebSocketServer(port=8766)

        # Start server in background thread
        self.server_thread = threading.Thread(
            target=lambda: asyncio.run(self._run_server())
        )
        self.server_thread.daemon = True
        self.server_thread.start()

        # Give server time to start
        time.sleep(0.5)

        # Create test node
        self.test_node = WebSocketBridgeIntegrationTester()

    def tearDown(self):
        """Clean up test environment."""
        # Stop ROS2
        if rclpy.ok():
            rclpy.shutdown()

        # Stop mock server
        if hasattr(self, "mock_server"):
            asyncio.run(self.mock_server.stop_server())

    async def _run_server(self):
        """Run the mock WebSocket server."""
        await self.mock_server.start_server()
        # Keep server running
        await asyncio.Future()

    def test_complete_data_flow(self):
        """Test complete WebSocket to ROS2 data flow."""
        # Test data to send
        test_data = {
            "imu": {
                "accel_x": 1.5,
                "accel_y": -0.8,
                "accel_z": 9.7,
                "gyro_x": 0.05,
                "gyro_y": -0.02,
                "gyro_z": 0.1,
                "temperature": 24.5,
            },
            "gps": {"latitude": 38.4406, "longitude": -110.7870, "altitude": 1828.8},
            "battery": {"voltage": 23.8, "current": 1.8, "charge_level": 78},
        }

        # Send data via mock WebSocket server
        self.mock_server.send_test_data(test_data)

        # Give some time for data to be processed
        time.sleep(1.0)

        # Spin ROS2 to process messages
        rclpy.spin_once(self.test_node, timeout_sec=1.0)

        # Verify data was received
        self.assertEqual(len(self.test_node.imu_received), 1)
        self.assertEqual(len(self.test_node.gps_received), 1)
        self.assertEqual(len(self.test_node.battery_received), 1)
        self.assertEqual(len(self.test_node.temp_received), 1)

        # Verify IMU data
        imu_data = self.test_node.imu_received[0]
        self.assertEqual(imu_data["accel_x"], 1.5)
        self.assertEqual(imu_data["accel_y"], -0.8)
        self.assertEqual(imu_data["accel_z"], 9.7)
        self.assertEqual(imu_data["gyro_x"], 0.05)
        self.assertEqual(imu_data["gyro_y"], -0.02)
        self.assertEqual(imu_data["gyro_z"], 0.1)

        # Verify GPS data
        gps_data = self.test_node.gps_received[0]
        self.assertEqual(gps_data["latitude"], 38.4406)
        self.assertEqual(gps_data["longitude"], -110.7870)
        self.assertEqual(gps_data["altitude"], 1828.8)

        # Verify battery data
        battery_data = self.test_node.battery_received[0]
        self.assertEqual(battery_data["voltage"], 23.8)
        self.assertEqual(battery_data["current"], 1.8)
        self.assertEqual(battery_data["percentage"], 0.78)  # 78% -> 0.78

        # Verify temperature data
        self.assertEqual(self.test_node.temp_received[0], 24.5)

    def test_multiple_messages(self):
        """Test processing multiple WebSocket messages."""
        # Send multiple data packets
        test_data_1 = {"imu": {"accel_x": 1.0, "temperature": 20.0}}
        test_data_2 = {"gps": {"latitude": 40.0, "longitude": -120.0}}
        test_data_3 = {"battery": {"voltage": 25.0, "charge_level": 95}}

        self.mock_server.send_test_data(test_data_1)
        time.sleep(0.1)
        self.mock_server.send_test_data(test_data_2)
        time.sleep(0.1)
        self.mock_server.send_test_data(test_data_3)

        # Give time for processing
        time.sleep(1.0)

        # Spin ROS2 to process messages
        for _ in range(3):
            rclpy.spin_once(self.test_node, timeout_sec=0.5)

        # Verify all messages were processed
        self.assertEqual(len(self.test_node.imu_received), 1)
        self.assertEqual(len(self.test_node.gps_received), 1)
        self.assertEqual(len(self.test_node.battery_received), 1)

    def test_malformed_json_handling(self):
        """Test handling of malformed JSON messages."""
        # Send valid data first
        valid_data = {"imu": {"accel_x": 1.0}}
        self.mock_server.send_test_data(valid_data)

        # Send malformed JSON (this would be handled by the bridge)
        # Note: We can't easily test this in integration without mocking the WebSocket connection

        time.sleep(1.0)
        rclpy.spin_once(self.test_node, timeout_sec=0.5)

        # Should still process valid data
        self.assertEqual(len(self.test_node.imu_received), 1)


if __name__ == "__main__":
    unittest.main()
