#!/usr/bin/env python3
"""
Emergency Stop System Tests - Competition Critical
Tests the hierarchical emergency stop system for competition safety.
"""

import time
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class TestEmergencyStopSystem(unittest.TestCase):
    """Test emergency stop system functionality."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS2 environment."""
        if not rclpy.ok():
            rclpy.init(args=[])
        cls.node = Node("test_emergency_node")

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2 environment."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures."""
        # Import here to avoid ROS2 init issues
        from src.autonomy.core.safety_system.emergency_stop_system import (
            EmergencyStopLevel,
            EmergencyStopSystem,
        )

        self.system = EmergencyStopSystem()
        # Make EmergencyStopLevel available for tests
        global EmergencyStopLevel
        EmergencyStopLevel = EmergencyStopLevel
        self.emergency_states = []

        # Subscribe to emergency status
        self.status_sub = self.node.create_subscription(
            String, "/emergency/status", self._emergency_callback, 10
        )

    def tearDown(self):
        """Clean up after each test."""
        if hasattr(self, "status_sub"):
            self.node.destroy_subscription(self.status_sub)
        if hasattr(self, "system"):
            self.system.destroy_node()

    def _emergency_callback(self, msg):
        """Callback for emergency status updates."""
        import json

        try:
            data = json.loads(msg.data)
            self.emergency_states.append(data)
        except json.JSONDecodeError as e:
            # Log the error but don't crash
            print(f"Failed to parse emergency status: {msg.data}, error: {e}")
        except Exception as e:
            print(f"Unexpected error in emergency callback: {e}")

    def test_emergency_hierarchy(self):
        """Test emergency stop level hierarchy and escalation."""
        # Test soft stop - least severe
        initial_level = self.system.current_level
        self.system.trigger_emergency_stop(
            EmergencyStopLevel.SOFT_STOP, "Test soft stop"
        )
        time.sleep(0.2)  # Allow async processing

        # Verify system entered soft stop state
        self.assertEqual(
            self.system.current_level,
            EmergencyStopLevel.SOFT_STOP,
            "System should enter soft stop state",
        )
        self.assertNotEqual(
            self.system.current_level,
            initial_level,
            "Emergency stop should change system state",
        )

        # Verify emergency states were published
        self.assertTrue(
            len(self.emergency_states) > 0, "Emergency status should be published"
        )
        latest_state = self.emergency_states[-1]
        self.assertEqual(
            latest_state.get("level"), "soft_stop", "Published state should match"
        )

        # Reset and test hard stop - more severe
        reset_response = Trigger.Response()
        self.system._handle_reset(Trigger.Request(), reset_response)
        time.sleep(0.1)

        self.assertTrue(reset_response.success, "Reset should succeed")
        self.assertIsNone(
            self.system.current_level, "Reset should clear emergency state"
        )

        # Test hard stop
        self.system.trigger_emergency_stop(
            EmergencyStopLevel.HARD_STOP, "Test hard stop"
        )
        time.sleep(0.2)

        self.assertEqual(
            self.system.current_level,
            EmergencyStopLevel.HARD_STOP,
            "System should enter hard stop state",
        )

        # Verify hierarchy: hard stop should be more severe than soft stop
        latest_hard_state = self.emergency_states[-1]
        self.assertEqual(
            latest_hard_state.get("level"), "hard_stop", "Hard stop should be published"
        )

    def test_emergency_escalation(self):
        """Test automatic escalation on repeated triggers."""
        # Reset system to clean state
        reset_response = Trigger.Response()
        self.system._handle_reset(Trigger.Request(), reset_response)
        self.assertTrue(reset_response.success)
        time.sleep(0.2)

        # Clear any previous states
        self.emergency_states.clear()

        # Trigger multiple soft stops within escalation window to cause escalation
        for i in range(4):  # More than threshold (3)
            self.system.trigger_emergency_stop(
                EmergencyStopLevel.SOFT_STOP, f"Trigger {i+1}"
            )
            # Spin to process messages
            rclpy.spin_once(self.node, timeout_sec=0.1)
            if i < 3:  # Don't sleep after last trigger
                time.sleep(0.1)

        # Wait for escalation to take effect and spin to receive messages
        time.sleep(0.5)
        rclpy.spin_once(self.node, timeout_sec=0.1)

        # Should escalate to emergency shutdown after 3+ triggers
        self.assertEqual(
            self.system.current_level,
            EmergencyStopLevel.EMERGENCY_SHUTDOWN,
            f"Should escalate to emergency shutdown after repeated triggers. "
            f"Current level: {self.system.current_level}, trigger count: {self.system.trigger_count}",
        )

        # Debug: Print received states
        print(f"Received emergency states: {len(self.emergency_states)}")
        for state in self.emergency_states:
            print(f"  State: {state}")

        # Verify escalation was published
        shutdown_states = [
            s for s in self.emergency_states if s.get("level") == "shutdown"
        ]
        self.assertTrue(
            len(shutdown_states) > 0,
            f"Emergency shutdown should be published. Received states: {self.emergency_states}",
        )

    def test_manual_override(self):
        """Test manual emergency stop override."""
        # Enable manual override
        request = SetBool.Request()
        request.data = True
        response = SetBool.Response()

        self.system._handle_manual_stop(request, response)
        self.assertTrue(response.success)

        # Try to trigger automatic stop (should be blocked)
        self.system.trigger_emergency_stop(
            EmergencyStopLevel.SOFT_STOP, "Should be blocked"
        )
        time.sleep(0.5)

        # Should not trigger because manual override is active
        self.assertNotEqual(self.system.current_level, EmergencyStopLevel.SOFT_STOP)

    def test_recovery_procedures(self):
        """Test recovery from emergency stops."""
        # Reset system first
        self.system._handle_reset(Trigger.Request(), Trigger.Response())
        time.sleep(0.1)

        # Trigger soft stop
        self.system.trigger_emergency_stop(
            EmergencyStopLevel.SOFT_STOP, "Test recovery"
        )
        time.sleep(0.5)

        self.assertEqual(self.system.current_level, EmergencyStopLevel.SOFT_STOP)

        # Test recovery with mocked publishing to avoid ROS2 teardown issues
        with patch.object(self.system.system_state_pub, "publish"):
            # Manually trigger recovery (normally done by timer)
            self.system._attempt_recovery(EmergencyStopLevel.SOFT_STOP)

            # Wait for recovery to complete
            time.sleep(0.2)

            # After recovery, system should be clear
            self.assertIsNone(
                self.system.current_level, "Recovery should clear emergency level"
            )

        # Test manual reset as well
        self.system.trigger_emergency_stop(EmergencyStopLevel.SOFT_STOP, "Test reset")
        time.sleep(0.2)

        reset_response = Trigger.Response()
        reset_result = self.system._handle_reset(Trigger.Request(), reset_response)
        self.assertTrue(reset_result.success, "Manual reset should succeed")
        self.assertIsNone(
            self.system.current_level, "Reset should clear emergency level"
        )

    def test_emergency_services(self):
        """Test ROS2 services for emergency system."""
        # Test health check service
        health_request = Trigger.Request()
        health_response = Trigger.Response()

        self.system._handle_health_check(health_request, health_response)
        self.assertTrue(health_response.success)
        self.assertIn("Emergency system healthy", health_response.message)

        # Test reset service
        reset_request = Trigger.Request()
        reset_response = Trigger.Response()

        # First trigger emergency
        self.system.trigger_emergency_stop(
            EmergencyStopLevel.SOFT_STOP, "For reset test"
        )

        # Then reset
        self.system._handle_reset(reset_request, reset_response)
        self.assertTrue(reset_response.success)

        # Should be cleared
        self.assertIsNone(self.system.current_level)

    def test_emergency_triggers(self):
        """Test various emergency trigger sources."""
        # Test collision trigger
        from std_msgs.msg import Bool

        collision_msg = Bool()
        collision_msg.data = True

        self.system._handle_collision(collision_msg)
        time.sleep(0.5)

        self.assertEqual(self.system.current_level, EmergencyStopLevel.SOFT_STOP)

        # Reset and test motor fault
        self.system._handle_reset(Trigger.Request(), Trigger.Response())
        time.sleep(0.1)

        motor_msg = String()
        motor_msg.data = "CRITICAL: Motor overcurrent"
        self.system._handle_motor_fault(motor_msg)
        time.sleep(0.5)

        self.assertEqual(self.system.current_level, EmergencyStopLevel.HARD_STOP)

    def test_emergency_status_publishing(self):
        """Test that emergency status is published correctly."""
        # Trigger emergency
        self.system.trigger_emergency_stop(EmergencyStopLevel.SOFT_STOP, "Status test")
        time.sleep(0.5)

        # Should have received status updates
        self.assertTrue(len(self.emergency_states) > 0)

        # Check latest status
        latest_status = self.emergency_states[-1]
        self.assertIn("level", latest_status)
        self.assertEqual(latest_status["level"], "soft_stop")


if __name__ == "__main__":
    unittest.main()
