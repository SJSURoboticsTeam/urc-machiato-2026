#!/usr/bin/env python3
"""
Hardware Validation Suite - Competition Ready
Simple, focused hardware validation for competition deployment.
"""

import os
import subprocess
import sys
import time
import unittest

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String


class HardwareValidationSuite(unittest.TestCase):
    """
    Simple hardware validation tests for competition readiness.

    Tests focus on:
    - Sensor data availability and reasonableness
    - Actuator responsiveness
    - Communication reliability
    - Basic system health
    """

    @classmethod
    def setUpClass(cls):
        """Set up ROS2 environment for hardware tests."""
        rclpy.init(args=[])
        cls.node = Node("hardware_validation_test")

        # Test configuration
        cls.test_duration = 5.0  # seconds per test
        cls.tolerance = 0.1  # 10% tolerance for reasonableness checks

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2 environment."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        """Reset test state."""
        self.received_messages = []
        self.test_start_time = time.time()

    def test_sensor_data_availability(self):
        """Test that sensors are publishing data."""
        self.get_logger().info("Testing sensor data availability...")

        # Test GPS data
        gps_received = self._wait_for_topic(
            "/gps/fix", NavSatFix, timeout=self.test_duration
        )
        self.assertTrue(gps_received, "GPS sensor not publishing data")

        # Test IMU data
        imu_received = self._wait_for_topic(
            "/imu/data", Imu, timeout=self.test_duration
        )
        self.assertTrue(imu_received, "IMU sensor not publishing data")

        # Test battery data
        battery_received = self._wait_for_topic(
            "/battery/status", Float32, timeout=self.test_duration
        )
        self.assertTrue(battery_received, "Battery sensor not publishing data")

    def test_sensor_data_reasonableness(self):
        """Test that sensor data values are reasonable."""
        self.get_logger().info("Testing sensor data reasonableness...")

        # GPS should be within reasonable bounds (competition site coordinates)
        gps_data = self._get_latest_message(
            "/gps/fix", NavSatFix, timeout=self.test_duration
        )
        self.assertIsNotNone(gps_data, "No GPS data received")

        # Basic GPS bounds check (adjust for your competition location)
        self.assertGreater(gps_data.latitude, -90, "GPS latitude out of range")
        self.assertLess(gps_data.latitude, 90, "GPS latitude out of range")
        self.assertGreater(gps_data.longitude, -180, "GPS longitude out of range")
        self.assertLess(gps_data.longitude, 180, "GPS longitude out of range")

        # IMU should have reasonable acceleration values
        imu_data = self._get_latest_message(
            "/imu/data", Imu, timeout=self.test_duration
        )
        self.assertIsNotNone(imu_data, "No IMU data received")

        # Acceleration should be within reasonable bounds (not free-falling)
        accel_x = imu_data.linear_acceleration.x
        accel_y = imu_data.linear_acceleration.y
        accel_z = imu_data.linear_acceleration.z

        # Gravity should be ~9.8 m/s² in Z direction (with some tolerance)
        expected_gravity = 9.8
        self.assertAlmostEqual(
            abs(accel_z),
            expected_gravity,
            delta=expected_gravity * 0.5,
            msg="IMU acceleration Z-axis unreasonable",
        )

    def test_actuator_responsiveness(self):
        """Test that actuators respond to commands."""
        self.get_logger().info("Testing actuator responsiveness...")

        # Test motor command response
        motor_command_topic = "/motor/command"
        motor_response_topic = "/motor/status"

        # Send test command
        self._publish_test_message(motor_command_topic, String(data="TEST_COMMAND"))

        # Wait for response
        response_received = self._wait_for_topic(
            motor_response_topic, String, timeout=self.test_duration
        )
        self.assertTrue(response_received, "Motor actuator not responding to commands")

    def test_communication_reliability(self):
        """Test CAN bus and ROS2 communication reliability."""
        self.get_logger().info("Testing communication reliability...")

        # Test CAN data availability
        can_data_received = self._wait_for_topic(
            "/can/sensor_data", String, timeout=self.test_duration
        )
        self.assertTrue(can_data_received, "CAN bus data not available")

        # Test message frequency (should be reasonably frequent)
        message_count = self._count_messages(
            "/imu/data", Imu, duration=self.test_duration
        )
        expected_min_messages = 5  # At least 1 Hz
        self.assertGreaterEqual(
            message_count,
            expected_min_messages,
            f"IMU message rate too low: {message_count} messages in {self.test_duration}s",
        )

    def test_system_health_endpoints(self):
        """Test that system health endpoints are responding."""
        self.get_logger().info("Testing system health endpoints...")

        # Test state machine health
        state_healthy = self._call_health_service("/state_machine/health_check")
        self.assertTrue(state_healthy, "State machine health check failed")

        # Test mission control health
        mission_healthy = self._call_health_service("/mission/health_check")
        self.assertTrue(mission_healthy, "Mission control health check failed")

        # Test emergency system health
        emergency_healthy = self._call_health_service("/emergency/health_check")
        self.assertTrue(emergency_healthy, "Emergency system health check failed")

    def test_power_system_health(self):
        """Test power system status."""
        self.get_logger().info("Testing power system health...")

        # Battery voltage should be reasonable
        battery_data = self._get_latest_message(
            "/battery/status", Float32, timeout=self.test_duration
        )
        self.assertIsNotNone(battery_data, "No battery data received")

        # Battery voltage should be > 10V (adjust based on your battery specs)
        min_voltage = 10.0
        self.assertGreater(
            battery_data.data,
            min_voltage,
            f"Battery voltage too low: {battery_data.data}V",
        )

    # Helper methods

    def _wait_for_topic(self, topic_name, msg_type, timeout=5.0):
        """Wait for at least one message on a topic."""
        received = False

        def callback(msg):
            nonlocal received
            received = True

        subscription = self.node.create_subscription(msg_type, topic_name, callback, 10)

        start_time = time.time()
        while not received and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(subscription)
        return received

    def _get_latest_message(self, topic_name, msg_type, timeout=5.0):
        """Get the latest message from a topic."""
        latest_msg = None

        def callback(msg):
            nonlocal latest_msg
            latest_msg = msg

        subscription = self.node.create_subscription(msg_type, topic_name, callback, 10)

        start_time = time.time()
        while (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(subscription)
        return latest_msg

    def _count_messages(self, topic_name, msg_type, duration=5.0):
        """Count messages received on a topic over a duration."""
        count = 0

        def callback(msg):
            nonlocal count
            count += 1

        subscription = self.node.create_subscription(msg_type, topic_name, callback, 10)

        start_time = time.time()
        while (time.time() - start_time) < duration:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(subscription)
        return count

    def _publish_test_message(self, topic_name, message):
        """Publish a test message."""
        publisher = self.node.create_publisher(type(message), topic_name, 10)
        publisher.publish(message)
        self.node.destroy_publisher(publisher)

    def _call_health_service(self, service_name):
        """Call a health check service and return success status."""
        try:
            from std_srvs.srv import Trigger

            client = self.node.create_client(Trigger, service_name)

            if not client.wait_for_service(timeout_sec=2.0):
                return False

            request = Trigger.Request()
            future = client.call_async(request)

            start_time = time.time()
            while not future.done() and (time.time() - start_time) < 5.0:
                rclpy.spin_once(self.node, timeout_sec=0.1)

            if future.done():
                response = future.result()
                client.destroy()
                return response.success

            client.destroy()
            return False

        except Exception as e:
            self.get_logger().error(f"Health service call failed: {e}")
            return False

    def get_logger(self):
        """Get logger for test output."""
        return self.node.get_logger()


def run_hardware_validation():
    """Run hardware validation suite."""
    print("🔧 Running Hardware Validation Suite...")
    print("=" * 50)

    # Load test suite
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromTestCase(HardwareValidationSuite)

    # Run tests with verbose output
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Summary
    print("\n" + "=" * 50)
    if result.wasSuccessful():
        print("✅ HARDWARE VALIDATION PASSED")
        print("System is ready for competition deployment")
    else:
        print("❌ HARDWARE VALIDATION FAILED")
        print(f"Failed tests: {len(result.failures)}")
        print(f"Errors: {len(result.errors)}")
        print("Check hardware connections and sensor calibration")

    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_hardware_validation()
    sys.exit(0 if success else 1)
