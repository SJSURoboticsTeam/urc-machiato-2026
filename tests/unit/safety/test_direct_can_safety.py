#!/usr/bin/env python3
"""
Tests for Direct CAN Safety System.

Validates critical safety functionality that bypasses ROS2 middleware
for immediate hardware response to emergency conditions.
"""

import unittest
from unittest.mock import Mock, patch
import serial
import time
import threading
from typing import Optional

# Add autonomy to path
import os
import sys
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
AUTONOMY_ROOT = os.path.join(PROJECT_ROOT, "src", "autonomy")
sys.path.insert(0, AUTONOMY_ROOT)

try:
    from core.safety_system.direct_can_safety import DirectCANSafety, SafetyCommand
except ImportError:
    # Fallback for testing without full module
    DirectCANSafety = None
    SafetyCommand = None


@unittest.skipIf(DirectCANSafety is None, "DirectCANSafety module not available")
class TestDirectCANSafety(unittest.TestCase):
    """Test direct CAN safety functionality."""

    @patch('serial.Serial')
    def setUp(self, mock_serial):
        """Set up test fixtures."""
        self.mock_serial = mock_serial
        # Mock the CanSerial import
        with patch('core.safety_system.direct_can_safety.CanSerial'):
            self.can_safety = DirectCANSafety(can_port='/dev/ttyACM0')

    def test_initialization(self):
        """Test DirectCANSafety initialization."""
        with patch('core.safety_system.direct_can_safety.CanSerial') as mock_can_serial:
            safety = DirectCANSafety(can_port='/dev/ttyACM0')

            # Verify initialization
            self.assertIsInstance(safety, DirectCANSafety)
            self.assertEqual(safety.can_port, '/dev/ttyACM0')
            self.assertIsInstance(safety.command_lock, threading.Lock)
            mock_can_serial.assert_called_once_with('/dev/ttyACM0')

    def test_emergency_stop(self):
        """Test emergency stop command."""
        # Mock successful command execution
        self.can_safety.can_serial = Mock()
        self.can_safety.can_serial.send_command = Mock(return_value=True)

        result = self.can_safety.emergency_stop("TEST")

        self.assertTrue(result)
        self.can_safety.can_serial.send_command.assert_called_with(
            "PRIORITY:EMERGENCY_STOP:TEST"
        )

    def test_emergency_stop_with_reason(self):
        """Test emergency stop with specific reason."""
        self.can_safety.can_serial = Mock()
        self.can_safety.can_serial.send_command = Mock(return_value=True)

        result = self.can_safety.emergency_stop("SOFTWARE_ESTOP")

        self.assertTrue(result)
        self.can_safety.can_serial.send_command.assert_called_with(
            "PRIORITY:EMERGENCY_STOP:SOFTWARE_ESTOP"
        )

    def test_boundary_violation_stop(self):
        """Test boundary violation stop."""
        self.can_safety.can_serial = Mock()
        self.can_safety.can_serial.send_command = Mock(return_value=True)

        result = self.can_safety.boundary_violation_stop()

        self.assertTrue(result)
        self.can_safety.can_serial.send_command.assert_called_with(
            "PRIORITY:BOUNDARY_VIOLATION_STOP"
        )

    def test_motor_brake(self):
        """Test motor brake command."""
        self.can_safety.can_serial = Mock()
        self.can_safety.can_serial.send_command = Mock(return_value=True)

        result = self.can_safety.motor_brake()

        self.assertTrue(result)
        self.can_safety.can_serial.send_command.assert_called_with(
            "PRIORITY:MOTOR_BRAKE"
        )

    def test_connection_failure_emergency_stop(self):
        """Test behavior when CAN connection fails during emergency stop."""
        # Simulate connection failure
        self.can_safety.can_serial = None
        self.can_safety.connected = False

        result = self.can_safety.emergency_stop("TEST")

        self.assertFalse(result)
        self.assertFalse(self.can_safety.is_connected())

    def test_connection_failure_boundary_stop(self):
        """Test behavior when CAN connection fails during boundary stop."""
        # Simulate connection failure
        self.can_safety.can_serial = None
        self.can_safety.connected = False

        result = self.can_safety.boundary_violation_stop()

        self.assertFalse(result)
        self.assertFalse(self.can_safety.is_connected())

    def test_command_lock_thread_safety(self):
        """Test that commands are thread-safe."""
        self.can_safety.can_serial = Mock()
        self.can_safety.can_serial.send_command = Mock(return_value=True)

        # Test concurrent emergency stops
        results = []
        def emergency_worker():
            result = self.can_safety.emergency_stop("THREAD_TEST")
            results.append(result)

        threads = []
        for i in range(5):
            thread = threading.Thread(target=emergency_worker)
            threads.append(thread)
            thread.start()

        for thread in threads:
            thread.join()

        # All commands should succeed
        self.assertEqual(len(results), 5)
        self.assertTrue(all(results))

        # Verify all commands were sent
        self.assertEqual(self.can_safety.can_serial.send_command.call_count, 5)

    def test_send_command_success(self):
        """Test successful command sending."""
        self.can_safety.can_serial = Mock()
        self.can_safety.can_serial.send_command = Mock(return_value=True)

        result = self.can_safety._send_command("TEST_COMMAND")

        self.assertTrue(result)
        self.can_safety.can_serial.send_command.assert_called_with("TEST_COMMAND")

    def test_send_command_failure(self):
        """Test command sending failure."""
        self.can_safety.can_serial = Mock()
        self.can_safety.can_serial.send_command = Mock(return_value=False)

        result = self.can_safety._send_command("TEST_COMMAND")

        self.assertFalse(result)
        self.can_safety.can_serial.send_command.assert_called_with("TEST_COMMAND")

    def test_send_command_exception(self):
        """Test command sending with exception."""
        self.can_safety.can_serial = Mock()
        self.can_safety.can_serial.send_command = Mock(side_effect=Exception("CAN Error"))

        result = self.can_safety._send_command("TEST_COMMAND")

        self.assertFalse(result)

    def test_is_connected(self):
        """Test connection status checking."""
        # Test connected state
        self.can_safety.can_serial = Mock()
        self.can_safety.connected = True
        self.assertTrue(self.can_safety.is_connected())

        # Test disconnected state
        self.can_safety.can_serial = None
        self.can_safety.connected = False
        self.assertFalse(self.can_safety.is_connected())

    def test_connect_success(self):
        """Test successful CAN connection."""
        with patch('core.safety_system.direct_can_safety.CanSerial') as mock_can_serial:
            safety = DirectCANSafety.__new__(DirectCANSafety)
            safety.can_port = '/dev/ttyACM0'
            safety.logger = Mock()

            result = safety.connect()

            self.assertTrue(result)
            self.assertTrue(safety.connected)
            mock_can_serial.assert_called_once_with('/dev/ttyACM0')

    def test_connect_failure(self):
        """Test CAN connection failure."""
        with patch('core.safety_system.direct_can_safety.CanSerial') as mock_can_serial:
            mock_can_serial.side_effect = Exception("Connection failed")

            safety = DirectCANSafety.__new__(DirectCANSafety)
            safety.can_port = '/dev/ttyACM0'
            safety.logger = Mock()

            result = safety.connect()

            self.assertFalse(result)
            self.assertFalse(safety.connected)

    def test_reconnect_after_failure(self):
        """Test reconnection after initial failure."""
        with patch('core.safety_system.direct_can_safety.CanSerial') as mock_can_serial:
            # First call fails
            mock_can_serial.side_effect = [Exception("Connection failed"), Mock()]

            safety = DirectCANSafety.__new__(DirectCANSafety)
            safety.can_port = '/dev/ttyACM0'
            safety.logger = Mock()

            # Initial connection fails
            safety.connect()
            self.assertFalse(safety.connected)

            # Second connection succeeds
            safety.connect()
            self.assertTrue(safety.connected)

    def test_emergency_stop_timeout(self):
        """Test emergency stop with timeout."""
        self.can_safety.can_serial = Mock()
        # Simulate slow response
        self.can_safety.can_serial.send_command = Mock(return_value=True)

        start_time = time.time()
        result = self.can_safety.emergency_stop("TIMEOUT_TEST")
        end_time = time.time()

        # Command should complete quickly (no artificial timeout in current implementation)
        self.assertTrue(result)
        self.assertLess(end_time - start_time, 1.0)  # Should be fast

    def test_multiple_emergency_stops(self):
        """Test multiple emergency stops in sequence."""
        self.can_safety.can_serial = Mock()
        self.can_safety.can_serial.send_command = Mock(return_value=True)

        # Send multiple emergency stops
        for i in range(10):
            result = self.can_safety.emergency_stop(f"TEST_{i}")
            self.assertTrue(result)

        # Verify all commands were sent
        self.assertEqual(self.can_safety.can_serial.send_command.call_count, 10)

    def test_safety_command_enum(self):
        """Test SafetyCommand enum values."""
        if SafetyCommand:
            self.assertEqual(SafetyCommand.EMERGENCY_STOP.value, "EMERGENCY_STOP")
            self.assertEqual(SafetyCommand.BOUNDARY_VIOLATION_STOP.value, "BOUNDARY_VIOLATION_STOP")
            self.assertEqual(SafetyCommand.MOTOR_BRAKE.value, "MOTOR_BRAKE")

    @patch('time.sleep')
    def test_command_retry_logic(self, mock_sleep):
        """Test command retry logic on failure."""
        # Note: Current implementation doesn't have retry logic,
        # but this test ensures we can add it later
        self.can_safety.can_serial = Mock()
        self.can_safety.can_serial.send_command = Mock(return_value=False)

        result = self.can_safety.emergency_stop("RETRY_TEST")

        self.assertFalse(result)
        # Should only try once in current implementation
        self.can_safety.can_serial.send_command.assert_called_once()


class TestDirectCANSafetyIntegration(unittest.TestCase):
    """Integration tests for Direct CAN Safety."""

    def test_safety_system_initialization(self):
        """Test safety system initializes correctly."""
        with patch('core.safety_system.direct_can_safety.CanSerial'):
            safety = DirectCANSafety(can_port='/dev/ttyACM0')

            # Verify safety system is ready
            self.assertIsNotNone(safety.command_lock)
            self.assertIsNotNone(safety.logger)

    def test_competition_safety_scenario(self):
        """Test safety response in competition scenario."""
        with patch('core.safety_system.direct_can_safety.CanSerial'):
            safety = DirectCANSafety()

            # Simulate competition emergency stop sequence
            # 1. Emergency stop triggered
            # 2. Boundary violation detected
            # 3. Motor brake applied

            # Mock the CAN serial for testing
            safety.can_serial = Mock()
            safety.can_serial.send_command = Mock(return_value=True)

            # Execute emergency sequence
            estop_result = safety.emergency_stop("COMPETITION_ESTOP")
            boundary_result = safety.boundary_violation_stop()
            brake_result = safety.motor_brake()

            # All safety commands should succeed
            self.assertTrue(estop_result)
            self.assertTrue(boundary_result)
            self.assertTrue(brake_result)

            # Verify command sequence
            expected_calls = [
                "PRIORITY:EMERGENCY_STOP:COMPETITION_ESTOP",
                "PRIORITY:BOUNDARY_VIOLATION_STOP",
                "PRIORITY:MOTOR_BRAKE"
            ]
            actual_calls = [call[0][0] for call in safety.can_serial.send_command.call_args_list]
            self.assertEqual(actual_calls, expected_calls)


if __name__ == '__main__':
    unittest.main()



