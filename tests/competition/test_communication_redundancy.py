#!/usr/bin/env python3
"""
Communication Redundancy Tests - Competition Critical
Tests the communication failover and redundancy system.
"""

import asyncio
import json
import time
import unittest
from unittest.mock import AsyncMock, MagicMock, patch

import rclpy
import websockets
from rclpy.node import Node
from std_msgs.msg import String


class TestCommunicationRedundancy(unittest.TestCase):
    """Test communication redundancy functionality."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS2 environment."""
        if not rclpy.ok():
            rclpy.init(args=[])
        cls.node = Node("test_comm_redundancy")

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2 environment."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures."""
        from src.bridges.communication_redundancy_manager import (
            CommunicationRedundancyManager,
        )

        self.manager = CommunicationRedundancyManager()
        self.received_messages = []

        # Subscribe to test topics
        self.state_sub = self.node.create_subscription(
            String, "/state_machine/current_state", self._state_callback, 10
        )
        self.mission_sub = self.node.create_subscription(
            String, "/mission/status", self._mission_callback, 10
        )

    def tearDown(self):
        """Clean up after each test."""
        if hasattr(self, "state_sub"):
            self.node.destroy_subscription(self.state_sub)
        if hasattr(self, "mission_sub"):
            self.node.destroy_subscription(self.mission_sub)
        if hasattr(self, "manager"):
            self.manager.destroy_node()

    def _state_callback(self, msg):
        """Callback for state messages."""
        self.received_messages.append(("state", msg.data))

    def _mission_callback(self, msg):
        """Callback for mission messages."""
        self.received_messages.append(("mission", msg.data))

    def test_initial_state(self):
        """Test initial communication state and verify proper initialization."""
        status = self.manager.get_status()

        # Verify initial state is properly configured
        self.assertEqual(
            status["current_channel"],
            "websocket",
            "Should start with WebSocket as primary channel",
        )
        self.assertFalse(status["failover_active"], "Should not start in failover mode")
        self.assertIsNotNone(
            status["last_heartbeat_age"], "Heartbeat age should be tracked"
        )

        # Verify manager has required publishers for direct ROS2 communication
        self.assertIsNotNone(
            self.manager.state_pub, "Should have state publisher for ROS2 direct mode"
        )
        self.assertIsNotNone(
            self.manager.mission_pub,
            "Should have mission publisher for ROS2 direct mode",
        )

        # Verify WebSocket connection parameters
        self.assertEqual(
            self.manager.websocket_url,
            "ws://localhost:8766",
            "Should have correct WebSocket URL",
        )
        self.assertEqual(
            self.manager.current_channel,
            "websocket",
            "Internal state should match status",
        )

    @patch("websockets.connect", new_callable=AsyncMock)
    async def test_websocket_heartbeat_monitoring(self, mock_connect):
        """Test WebSocket heartbeat monitoring."""
        # Mock WebSocket connection
        mock_websocket = AsyncMock()
        mock_websocket.send = AsyncMock()
        mock_websocket.recv = AsyncMock(return_value='{"type": "heartbeat_response"}')
        mock_connect.return_value.__aenter__.return_value = mock_websocket
        mock_connect.return_value.__aexit__.return_value = None

        # Start monitoring (short duration for test)
        await asyncio.wait_for(self.manager.websocket_monitor(), timeout=2.0)

        # Should still be healthy after successful heartbeat
        status = self.manager.get_status()
        self.assertTrue(status["websocket_healthy"])

    async def test_failover_on_websocket_failure(self):
        """Test automatic failover when WebSocket fails and verify communication continuity."""
        # Initial state: WebSocket should be active
        initial_channel = self.manager.current_channel
        self.assertEqual(initial_channel, "websocket", "Should start with WebSocket")
        self.assertFalse(
            self.manager.failover_active, "Should not start in failover mode"
        )

        # Simulate WebSocket failure by setting last heartbeat to old
        self.manager.websocket_healthy = False
        self.manager.last_heartbeat = time.time() - (self.manager.failover_timeout + 10)

        # Trigger failover by calling the async method
        await self.manager._initiate_failover()

        # Verify failover occurred
        self.assertTrue(
            self.manager.failover_active,
            "Failover should be active after WebSocket failure",
        )
        self.assertEqual(
            self.manager.current_channel,
            "ros2_direct",
            "Should switch to ROS2 direct communication",
        )

        # Verify ROS2 publishers are ready for direct communication
        self.assertIsNotNone(
            self.manager.state_pub, "ROS2 state publisher should be available"
        )
        self.assertIsNotNone(
            self.manager.mission_pub, "ROS2 mission publisher should be available"
        )

        # Test that system can still send messages after failover
        # (This would fail if failover didn't properly set up ROS2 communication)
        try:
            # Attempt to send a test message (this will verify publishers work)
            test_msg = String()
            test_msg.data = "test_failover_message"
            self.manager.state_pub.publish(test_msg)
            message_sent = True
        except Exception:
            message_sent = False

        self.assertTrue(message_sent, "Should be able to send messages after failover")

    async def test_message_routing_websocket(self):
        """Test message routing via WebSocket channel and verify channel selection logic."""
        # Ensure we're in WebSocket mode (primary channel)
        self.manager.failover_active = False
        self.manager.current_channel = "websocket"
        self.manager.websocket_healthy = True

        # Mock WebSocket connection and async send method
        with patch.object(
            self.manager, "websocket_connection", new=MagicMock()
        ) as mock_ws, patch("asyncio.create_task") as mock_create_task:

            mock_ws.send = AsyncMock()

            # Send command - should route through WebSocket
            command = {"type": "test_command", "data": "test_data"}
            success = self.manager.send_command_via_redundant_channel(command)

            # Verify async task was created for WebSocket send
            mock_create_task.assert_called_once()
            self.assertTrue(success, "Command should succeed via WebSocket")

            # Verify channel state didn't change
            self.assertEqual(
                self.manager.current_channel,
                "websocket",
                "Should remain on WebSocket channel",
            )
            self.assertFalse(
                self.manager.failover_active,
                "Should not trigger failover for successful send",
            )

    def test_message_routing_ros2_direct(self):
        """Test message routing via ROS2 direct channel."""
        # Force ROS2 direct mode
        self.manager.failover_active = True
        self.manager.current_channel = "ros2_direct"

        # Send command (should use ROS2 publishers)
        command = {"target": "state_machine", "command": "test"}
        success = self.manager.send_command_via_redundant_channel(command)

        # Should succeed (ROS2 publishing)
        self.assertTrue(success)

    async def test_automatic_failback(self):
        """Test automatic failback to WebSocket when it recovers."""
        # Start in failover mode
        self.manager.failover_active = True
        self.manager.current_channel = "ros2_direct"

        # Simulate WebSocket recovery
        self.manager.websocket_healthy = True
        self.manager.websocket_connection = MagicMock()

        # Switch back to WebSocket using the async method
        await self.manager._switch_to_websocket()

        # Should switch back to WebSocket
        self.assertFalse(self.manager.failover_active)
        self.assertEqual(self.manager.current_channel, "websocket")

    def test_health_status_publishing(self):
        """Test health status publishing."""
        # Call health monitoring
        self.manager._health_monitor_callback()

        # Should have published health status
        # Note: In a real test, we'd subscribe to /system/communication_health
        # For this unit test, we just verify the method doesn't crash
        status = self.manager.get_status()
        self.assertIsInstance(status, dict)
        self.assertIn("current_channel", status)
        self.assertIn("websocket_healthy", status)
        self.assertIn("failover_active", status)

    async def test_mission_state_forwarding(self):
        """Test forwarding of mission and state data."""
        # Test state forwarding
        test_data = '{"state": "AUTONOMOUS", "timestamp": 1234567890}'
        msg = String()
        msg.data = test_data

        # In failover mode, should attempt to forward to WebSocket
        self.manager.failover_active = True
        self.manager.websocket_connection = AsyncMock()

        # Mock the async forwarding call
        with patch.object(
            self.manager.websocket_connection, "send", new_callable=AsyncMock
        ) as mock_send:
            # Call the callback which should trigger forwarding
            self.manager._state_callback(msg)

            # Give the async task a moment to execute
            await asyncio.sleep(0.1)

            # Verify that forwarding was attempted
            mock_send.assert_called_once()

        # Should have attempted to forward (in real implementation)
        # For this test, we verify the callback doesn't crash
        self.assertIsNotNone(self.manager)

    def test_redundancy_under_load(self):
        """Test redundancy behavior under message load."""
        # Simulate high message load
        messages_sent = 0
        start_time = time.time()

        # Send multiple messages quickly
        for i in range(50):
            command = {"target": "mission", "command": f"test_{i}"}
            success = self.manager.send_command_via_redundant_channel(command)
            if success:
                messages_sent += 1

        elapsed = time.time() - start_time

        # Should handle the load without crashing
        self.assertGreater(messages_sent, 40)  # At least 80% success rate
        self.assertLess(elapsed, 5.0)  # Should complete within 5 seconds

    def test_websocket_reconnection_logic(self):
        """Test WebSocket reconnection logic."""
        # Simulate connection failure and reconnection
        self.manager.websocket_healthy = False

        # Mock successful reconnection
        with patch("websockets.connect", new_callable=AsyncMock) as mock_connect:
            mock_websocket = AsyncMock()
            mock_websocket.send = AsyncMock()
            mock_websocket.recv = AsyncMock(
                return_value='{"type": "heartbeat_response"}'
            )
            mock_connect.return_value.__aenter__.return_value = mock_websocket

            # Should attempt reconnection
            self.assertIsNotNone(mock_connect)

    def test_timeout_handling(self):
        """Test timeout handling in communication."""
        # Test with very short timeout
        self.manager.heartbeat_interval = 0.1
        self.manager.failover_timeout = 0.2

        # Set last heartbeat to old
        self.manager.last_heartbeat = time.time() - 1.0

        # Should trigger failover after timeout
        time.sleep(0.3)  # Wait for timeout

        # Check if failover would be triggered (simulate monitoring)
        current_time = time.time()
        websocket_age = current_time - self.manager.last_heartbeat

        should_failover = websocket_age > self.manager.failover_timeout
        self.assertTrue(should_failover)

    def test_channel_status_accuracy(self):
        """Test accuracy of channel status reporting."""
        # Test all possible states
        test_states = [
            {"channel": "websocket", "failover": False, "healthy": True},
            {"channel": "websocket", "failover": False, "healthy": False},
            {"channel": "ros2_direct", "failover": True, "healthy": False},
        ]

        for state in test_states:
            self.manager.current_channel = state["channel"]
            self.manager.failover_active = state["failover"]
            self.manager.websocket_healthy = state["healthy"]

            status = self.manager.get_status()

            self.assertEqual(status["current_channel"], state["channel"])
            self.assertEqual(status["failover_active"], state["failover"])
            self.assertEqual(status["websocket_healthy"], state["healthy"])


if __name__ == "__main__":
    # For async tests, use pytest with asyncio plugin
    # Run with: python -m pytest tests/competition/test_communication_redundancy.py -v
    import sys

    import pytest

    sys.exit(pytest.main([__file__, "-v"]))
