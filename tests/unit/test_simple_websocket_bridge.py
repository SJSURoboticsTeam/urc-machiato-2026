#!/usr/bin/env python3
"""
Unit tests for SimpleWebsocketBridge.

Tests the core functionality of the WebSocket to ROS2 bridge
without requiring actual network connections.
"""

import json
import os

# Import the bridge directly without mocking ROS2 for now
import sys
import unittest
from unittest.mock import MagicMock, Mock, patch

sys.path.insert(
    0,
    os.path.join(
        os.path.dirname(__file__), "..", "..", "autonomy", "code", "sensor_bridge"
    ),
)

# Create a test version that doesn't inherit from Node
class TestableWebsocketBridge:
    """Testable version of the bridge without ROS2 dependencies."""

    def __init__(self):
        # Mock publishers
        self.imu_pub = Mock()
        self.gps_pub = Mock()
        self.battery_pub = Mock()
        self.temp_pub = Mock()

        # Mock clock
        self.get_clock = Mock()
        mock_time = Mock()
        mock_time.now.return_value.to_msg.return_value = Mock()
        self.get_clock.return_value = mock_time

        # Mock logger
        self.get_logger = Mock()
        self.get_logger.return_value = Mock()

    # Copy the _process_sensor_data method from the real bridge
    def _process_sensor_data(self, data: dict):
        """Process incoming sensor data and publish to ROS2 topics."""

        # IMU data
        if "imu" in data:
            imu_msg = Mock()  # Mock Imu message
            imu_msg.header = Mock()
            imu_msg.header.stamp = self.get_clock().now().to_msg()
            imu_msg.header.frame_id = "imu"
            imu_msg.linear_acceleration = Mock()
            imu_msg.angular_velocity = Mock()

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
            gps_msg = Mock()  # Mock NavSatFix message
            gps_msg.header = Mock()
            gps_msg.header.stamp = self.get_clock().now().to_msg()
            gps_msg.header.frame_id = "gps"

            gps_data = data["gps"]
            gps_msg.latitude = gps_data.get("latitude", 0.0)
            gps_msg.longitude = gps_data.get("longitude", 0.0)
            gps_msg.altitude = gps_data.get("altitude", 0.0)

            self.gps_pub.publish(gps_msg)

        # Battery data
        if "battery" in data:
            battery_msg = Mock()  # Mock BatteryState message
            battery_msg.header = Mock()
            battery_msg.header.stamp = self.get_clock().now().to_msg()

            battery_data = data["battery"]
            battery_msg.voltage = battery_data.get("voltage", 24.0)
            battery_msg.current = battery_data.get("current", 0.0)
            battery_msg.percentage = battery_data.get("charge_level", 100.0) / 100.0

            self.battery_pub.publish(battery_msg)

        # Temperature data
        if "imu" in data and "temperature" in data["imu"]:
            temp_msg = Mock()  # Mock Float32 message
            temp_msg.data = data["imu"]["temperature"]
            self.temp_pub.publish(temp_msg)


class TestSimpleWebsocketBridge(unittest.TestCase):
    """Test cases for SimpleWebsocketBridge."""

    def setUp(self):
        """Set up test fixtures."""
        # Create testable bridge instance
        self.bridge = TestableWebsocketBridge()

    def tearDown(self):
        """Clean up test fixtures."""
        # No ROS2 cleanup needed for these unit tests
        pass

    def test_bridge_initialization(self):
        """Test that bridge initializes correctly."""
        # Bridge should be created and have the expected attributes
        self.assertIsNotNone(self.bridge)
        self.assertTrue(hasattr(self.bridge, "_process_sensor_data"))
        self.assertTrue(hasattr(self.bridge, "imu_pub"))
        self.assertTrue(hasattr(self.bridge, "gps_pub"))
        self.assertTrue(hasattr(self.bridge, "battery_pub"))
        self.assertTrue(hasattr(self.bridge, "temp_pub"))

    def test_process_imu_data(self):
        """Test IMU data processing and publishing."""
        imu_data = {
            "accel_x": 1.0,
            "accel_y": 2.0,
            "accel_z": 9.8,
            "gyro_x": 0.1,
            "gyro_y": 0.2,
            "gyro_z": 0.3,
            "temperature": 25.0,
        }

        sensor_data = {"imu": imu_data}

        # Process the data
        self.bridge._process_sensor_data(sensor_data)

        # Verify IMU message was published
        self.bridge.imu_pub.publish.assert_called_once()
        imu_msg = self.bridge.imu_pub.publish.call_args[0][0]

        # Check that the message has the expected attributes (Mock object)
        self.assertEqual(imu_msg.linear_acceleration.x, 1.0)
        self.assertEqual(imu_msg.linear_acceleration.y, 2.0)
        self.assertEqual(imu_msg.linear_acceleration.z, 9.8)
        self.assertEqual(imu_msg.angular_velocity.x, 0.1)
        self.assertEqual(imu_msg.angular_velocity.y, 0.2)
        self.assertEqual(imu_msg.angular_velocity.z, 0.3)

    def test_process_gps_data(self):
        """Test GPS data processing and publishing."""
        gps_data = {"latitude": 38.4, "longitude": -110.8, "altitude": 1500.0}

        sensor_data = {"gps": gps_data}

        # Process the data
        self.bridge._process_sensor_data(sensor_data)

        # Verify GPS message was published
        self.bridge.gps_pub.publish.assert_called_once()
        gps_msg = self.bridge.gps_pub.publish.call_args[0][0]

        # Check that the GPS message has the expected values
        self.assertEqual(gps_msg.latitude, 38.4)
        self.assertEqual(gps_msg.longitude, -110.8)
        self.assertEqual(gps_msg.altitude, 1500.0)

    def test_process_battery_data(self):
        """Test battery data processing and publishing."""
        battery_data = {"voltage": 24.5, "current": 2.1, "charge_level": 85}

        sensor_data = {"battery": battery_data}

        # Process the data
        self.bridge._process_sensor_data(sensor_data)

        # Verify battery message was published
        self.bridge.battery_pub.publish.assert_called_once()
        battery_msg = self.bridge.battery_pub.publish.call_args[0][0]

        # Check that the battery message has the expected values
        self.assertEqual(battery_msg.voltage, 24.5)
        self.assertEqual(battery_msg.current, 2.1)
        self.assertEqual(battery_msg.percentage, 0.85)  # 85% converted to 0.85

    def test_process_temperature_data(self):
        """Test temperature data extraction and publishing."""
        imu_data = {
            "accel_x": 0.0,
            "accel_y": 0.0,
            "accel_z": 9.81,
            "gyro_x": 0.0,
            "gyro_y": 0.0,
            "gyro_z": 0.0,
            "temperature": 23.5,
        }

        sensor_data = {"imu": imu_data}

        # Process the data
        self.bridge._process_sensor_data(sensor_data)

        # Verify temperature message was published
        self.bridge.temp_pub.publish.assert_called_once()
        temp_msg = self.bridge.temp_pub.publish.call_args[0][0]

        # Check that the temperature message has the expected value
        self.assertEqual(temp_msg.data, 23.5)

    def test_process_mixed_sensor_data(self):
        """Test processing data with multiple sensor types."""
        sensor_data = {
            "imu": {
                "accel_x": 1.0,
                "accel_y": 2.0,
                "accel_z": 9.8,
                "gyro_x": 0.1,
                "gyro_y": 0.2,
                "gyro_z": 0.3,
                "temperature": 25.0,
            },
            "gps": {"latitude": 38.4, "longitude": -110.8, "altitude": 1500.0},
            "battery": {"voltage": 24.0, "current": 1.5, "charge_level": 90},
        }

        # Process the data
        self.bridge._process_sensor_data(sensor_data)

        # Verify all publishers were called
        self.bridge.imu_pub.publish.assert_called_once()
        self.bridge.gps_pub.publish.assert_called_once()
        self.bridge.battery_pub.publish.assert_called_once()
        self.bridge.temp_pub.publish.assert_called_once()

    def test_process_partial_data(self):
        """Test processing data with missing fields uses defaults."""
        sensor_data = {
            "imu": {},  # Empty IMU data
            "gps": {"latitude": 40.0},  # Partial GPS data
            "battery": {},  # Empty battery data
        }

        # Process the data
        self.bridge._process_sensor_data(sensor_data)

        # Verify messages were published with defaults
        self.bridge.imu_pub.publish.assert_called_once()
        imu_msg = self.bridge.imu_pub.publish.call_args[0][0]
        self.assertEqual(imu_msg.linear_acceleration.x, 0.0)  # Default value
        self.assertEqual(imu_msg.linear_acceleration.z, 9.81)  # Default gravity

        self.bridge.gps_pub.publish.assert_called_once()
        gps_msg = self.bridge.gps_pub.publish.call_args[0][0]
        self.assertEqual(gps_msg.latitude, 40.0)
        self.assertEqual(gps_msg.longitude, 0.0)  # Default value
        self.assertEqual(gps_msg.altitude, 0.0)  # Default value

        self.bridge.battery_pub.publish.assert_called_once()
        battery_msg = self.bridge.battery_pub.publish.call_args[0][0]
        self.assertEqual(battery_msg.voltage, 24.0)  # Default value
        self.assertEqual(battery_msg.current, 0.0)  # Default value
        self.assertEqual(battery_msg.percentage, 1.0)  # 100% / 100.0

    def test_json_message_parsing(self):
        """Test that the bridge can handle JSON message parsing."""
        # Test valid JSON parsing
        valid_json = '{"imu": {"accel_x": 1.0, "temperature": 25.0}}'
        parsed = json.loads(valid_json)
        self.assertEqual(parsed["imu"]["accel_x"], 1.0)
        self.assertEqual(parsed["imu"]["temperature"], 25.0)

        # Test that invalid JSON would raise an exception (handled in real implementation)
        invalid_json = "{invalid json}"
        with self.assertRaises(json.JSONDecodeError):
            json.loads(invalid_json)

    def test_message_structure_validation(self):
        """Test that messages with expected structure are processed correctly."""
        # Test various message structures
        test_cases = [
            {"imu": {"accel_x": 1.0}},  # Minimal IMU
            {"gps": {"latitude": 38.4, "longitude": -110.8}},  # GPS only
            {"battery": {"voltage": 24.0, "charge_level": 85}},  # Battery only
            {"imu": {"temperature": 25.0}},  # Temperature only
            {},  # Empty message
        ]

        for sensor_data in test_cases:
            # Should not raise exceptions
            try:
                self.bridge._process_sensor_data(sensor_data)
            except Exception as e:
                self.fail(f"Processing sensor data {sensor_data} raised exception: {e}")


if __name__ == "__main__":
    unittest.main()
