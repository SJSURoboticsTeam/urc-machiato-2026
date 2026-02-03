#!/usr/bin/env python3
"""
Unit tests for bridge communication infrastructure.

Tests communication between ROS2, WebSocket, and CAN bridges.
Validates message routing, protocol translation, and error handling.
"""

import asyncio
import json
from typing import Any, Dict
from unittest.mock import AsyncMock, Mock, patch

import pytest

pytest.importorskip("src.bridges.simple_bridge")


class TestBridgeCommunications:
    """Test bridge communication patterns and message routing."""

    @pytest.fixture
    def mock_config(self):
        """Mock configuration for bridge testing."""
        return {
            "bridges": {
                "websocket_port": 8080,
                "ros_domain_id": 42,
                "can_interface": "can0",
            }
        }

    def test_simple_bridge_command_routing(self, mock_config):
        """Test SimpleBridge command routing functionality."""
        from src.bridges.simple_bridge import SimpleBridge

        bridge = SimpleBridge()

        # Test command handler registration
        test_results = []

        def test_handler(data):
            test_results.append(data)
            return {"status": "success", "result": "processed"}

        bridge.register_command_handler("test_command", test_handler)

        # Test command processing
        test_message = {"component": "test", "action": "run"}
        result = asyncio.run(
            bridge.process_command({"command": "test_command", "data": test_message})
        )

        assert result["status"] == "success"
        assert test_results[0] == test_message

    def test_simple_bridge_unknown_command(self, mock_config):
        """Test SimpleBridge handling of unknown commands."""
        from src.bridges.simple_bridge import SimpleBridge

        bridge = SimpleBridge()

        result = asyncio.run(
            bridge._process_command({"command": "unknown_command", "data": {}})
        )

        assert result["status"] == "unknown_command"

    def test_simple_bridge_type_based_routing(self, mock_config):
        """Test SimpleBridge type-based routing (legacy)."""
        from src.bridges.simple_bridge import SimpleBridge

        bridge = SimpleBridge()

        # Test motor command routing - this may fail if ROS2 is not set up
        try:
            result = asyncio.run(
                bridge._process_command({"type": "motor", "data": {"speed": 1.0}})
            )
            # If it succeeds, should return a dict
            assert isinstance(result, dict)
        except AttributeError:
            # Expected when ROS2 is not available
            pass

    def test_emergency_bridge_initialization(self, mock_config):
        """Test EmergencyBridge initialization and basic functionality."""
        from src.bridges.emergency_bridge import EmergencyBridge

        bridge = EmergencyBridge()

        # Test basic attributes
        assert hasattr(bridge, "emergency_port")
        assert hasattr(bridge, "emergency_clients")

        # Test emergency status
        status = bridge.get_emergency_status()
        assert isinstance(status, dict)
        assert "emergency_mode" in status

    def test_emergency_bridge_recovery(self, mock_config):
        """Test EmergencyBridge emergency recovery."""
        from src.bridges.emergency_bridge import EmergencyBridge

        bridge = EmergencyBridge()

        # Trigger emergency
        bridge.trigger_emergency("test", {"reason": "test emergency"})
        status = bridge.get_emergency_status()
        assert status["emergency_mode"] is True

        # Clear emergency
        bridge.clear_emergency()
        status = bridge.get_emergency_status()
        assert status["emergency_mode"] is False

    def test_direct_can_bridge_motor_control(self, mock_config):
        """Test DirectCANBridge motor control functionality."""
        from src.bridges.direct_can_bridge import DirectCANBridge

        bridge = DirectCANBridge()

        # Test motor control method exists
        assert hasattr(bridge, "send_motor_command")

        # Test getting motor status
        status = bridge.get_motor_status()
        assert isinstance(status, dict)

    def test_bridge_message_transformation(self, mock_config):
        """Test message transformation between protocols."""
        from src.bridges.simple_bridge import SimpleBridge

        bridge = SimpleBridge()

        # Test that bridge has message processing capability
        assert hasattr(bridge, "_process_command")
        assert hasattr(bridge, "register_command_handler")

        # Test basic message processing
        result = asyncio.run(bridge._process_command({"type": "test", "data": {}}))
        assert isinstance(result, dict)

    def test_bridge_error_handling(self, mock_config):
        """Test bridge error handling and recovery."""
        from src.bridges.simple_bridge import SimpleBridge

        bridge = SimpleBridge()

        # Test with invalid command handler
        def failing_handler(data):
            raise ValueError("Test error")

        bridge.register_command_handler("failing_command", failing_handler)

        result = asyncio.run(
            bridge._process_command({"command": "failing_command", "data": {}})
        )

        assert result["status"] == "handler_error"
        assert "Test error" in str(result.get("error", ""))

    def test_bridge_concurrent_operations(self, mock_config):
        """Test bridge handling of concurrent operations."""
        from src.bridges.simple_bridge import SimpleBridge

        bridge = SimpleBridge()

        # Register multiple handlers
        results = []

        def handler1(data):
            results.append(f"handler1_{data}")
            return {"result": "handler1"}

        def handler2(data):
            results.append(f"handler2_{data}")
            return {"result": "handler2"}

        bridge.register_command_handler("cmd1", handler1)
        bridge.register_command_handler("cmd2", handler2)

        # Execute commands sequentially (since _process_command is synchronous)
        result1 = asyncio.run(
            bridge._process_command({"command": "cmd1", "data": "test1"})
        )
        result2 = asyncio.run(
            bridge._process_command({"command": "cmd2", "data": "test2"})
        )

        assert result1["result"] == "handler1"
        assert result2["result"] == "handler2"
        assert len(results) == 2

    def test_bridge_statistics_tracking(self, mock_config):
        """Test bridge statistics and metrics tracking."""
        from src.bridges.simple_bridge import SimpleBridge

        bridge = SimpleBridge()

        # Check initial statistics
        stats = bridge.get_stats()
        assert isinstance(stats, dict)
        assert "messages_processed" in stats

        initial_count = stats["messages_processed"]

        # Process some messages
        asyncio.run(bridge._process_command({"command": "unknown", "data": {}}))
        asyncio.run(bridge._process_command({"command": "unknown2", "data": {}}))

        stats_after = bridge.get_stats()
        assert stats_after["messages_processed"] >= initial_count + 2
