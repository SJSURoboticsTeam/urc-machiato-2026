#!/usr/bin/env python3
"""
Unit Tests for Waypoint Navigation Mission

Tests the waypoint navigation mission ROS2 node that handles:
- GPS waypoint navigation
- SLAM pose integration
- Navigation stack coordination
- Real-time progress monitoring

Author: URC 2026 Test Suite
"""

import pytest
import math
from unittest.mock import Mock, MagicMock, patch, AsyncMock
import sys
import os
from typing import Dict, List, Any

# Add repo root so missions package is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
from missions.waypoint_navigation_mission import (
    WaypointNavigationMission,
    WaypointState,
)


class TestWaypointState:
    """Test WaypointState enum."""

    @pytest.mark.unit
    def test_waypoint_states(self):
        """Test all waypoint state values."""
        assert WaypointState.IDLE.value == "idle"
        assert WaypointState.INITIALIZING.value == "initializing"
        assert WaypointState.NAVIGATING.value == "navigating"
        assert WaypointState.APPROACHING.value == "approaching"
        assert WaypointState.ARRIVED.value == "arrived"
        assert WaypointState.FAILED.value == "failed"
        assert WaypointState.ABORTED.value == "aborted"


class TestWaypointNavigationMission:
    """Test WaypointNavigationMission ROS2 node."""

    @pytest.fixture
    def mock_node(self):
        """Create mock ROS2 node."""
        node = Mock()
        node.create_subscription = Mock()
        node.create_publisher = Mock()
        node.create_client = Mock()
        node.create_timer = Mock()
        node.get_logger = Mock()
        node.get_logger.return_value.info = Mock()
        node.get_logger.return_value.warn = Mock()
        node.get_logger.return_value.error = Mock()
        node.get_logger.return_value.debug = Mock()
        node.get_clock.return_value.now.return_value = Mock()
        node.get_clock.return_value.now.return_value.nanoseconds = 1000000000
        return node

    @pytest.fixture
    def waypoint_mission(self, mock_node):
        """Create WaypointNavigationMission instance."""
        with patch("missions.waypoint_navigation_mission.Node", return_value=mock_node):
            mission = WaypointNavigationMission()
            return mission

    @pytest.mark.unit
    def test_initialization(self, waypoint_mission, mock_node):
        """Test WaypointNavigationMission initialization."""
        # Verify ROS2 interfaces are created
        assert mock_node.create_subscription.call_count >= 3  # Multiple subscriptions
        assert mock_node.create_publisher.call_count >= 2  # Multiple publishers
        assert mock_node.create_client.call_count >= 1  # Service client

        # Verify initial state
        assert waypoint_mission.state == WaypointState.IDLE
        assert waypoint_mission.current_waypoint_index == 0
        assert len(waypoint_mission.waypoints) == 0

        # Verify parameters
        assert waypoint_mission.waypoint_tolerance > 0
        assert waypoint_mission.heading_tolerance > 0

    @pytest.mark.unit
    def test_load_waypoints_from_config(self, waypoint_mission):
        """Test loading waypoints from configuration."""
        config = {
            "waypoints": [
                {"name": "Point A", "x": 1.0, "y": 2.0, "heading": 45.0},
                {"name": "Point B", "x": 3.0, "y": 4.0, "heading": 90.0},
            ]
        }

        waypoint_mission._load_waypoints_from_config(config)

        assert len(waypoint_mission.waypoints) == 2
        assert waypoint_mission.waypoints[0]["name"] == "Point A"
        assert waypoint_mission.waypoints[0]["position"] == [1.0, 2.0]
        assert waypoint_mission.waypoints[0]["heading"] == 45.0

    @pytest.mark.unit
    def test_calculate_distance_to_waypoint(self, waypoint_mission):
        """Test distance calculation to waypoint."""
        current_pos = [0.0, 0.0, 0.0]
        waypoint = {"position": [3.0, 4.0], "heading": 0.0}

        distance = waypoint_mission._calculate_distance_to_waypoint(
            current_pos, waypoint
        )

        # Should be 5.0 (3-4-5 triangle)
        assert abs(distance - 5.0) < 0.01

    @pytest.mark.unit
    def test_calculate_heading_error(self, waypoint_mission):
        """Test heading error calculation."""
        current_heading = 0.0  # Facing north
        target_heading = 90.0  # Facing east

        error = waypoint_mission._calculate_heading_error(
            current_heading, target_heading
        )

        assert abs(error - 90.0) < 0.01

    @pytest.mark.unit
    def test_is_waypoint_reached(self, waypoint_mission):
        """Test waypoint reached detection."""
        # Set tolerances
        waypoint_mission.waypoint_tolerance = 1.0
        waypoint_mission.heading_tolerance = 5.0

        # Test case: within tolerance
        current_pos = [1.0, 1.0, 0.0]
        waypoint = {"position": [1.1, 1.1], "heading": 2.0}

        reached = waypoint_mission._is_waypoint_reached(current_pos, waypoint)
        assert reached is True

        # Test case: outside tolerance
        current_pos = [0.0, 0.0, 0.0]
        waypoint = {"position": [2.0, 2.0], "heading": 0.0}

        reached = waypoint_mission._is_waypoint_reached(current_pos, waypoint)
        assert reached is False

    @pytest.mark.unit
    def test_state_transitions(self, waypoint_mission):
        """Test state machine transitions."""
        # Start in IDLE
        assert waypoint_mission.state == WaypointState.IDLE

        # Transition to INITIALIZING
        waypoint_mission._set_state(WaypointState.INITIALIZING)
        assert waypoint_mission.state == WaypointState.INITIALIZING

        # Transition to NAVIGATING
        waypoint_mission._set_state(WaypointState.NAVIGATING)
        assert waypoint_mission.state == WaypointState.NAVIGATING

    @pytest.mark.unit
    def test_command_callback_start(self, waypoint_mission):
        """Test command callback for start command."""
        msg = Mock()
        msg.data = "start"

        # Mock the start_mission method
        waypoint_mission.start_mission = Mock()

        waypoint_mission._command_callback(msg)

        waypoint_mission.start_mission.assert_called_once()

    @pytest.mark.unit
    def test_command_callback_stop(self, waypoint_mission):
        """Test command callback for stop command."""
        msg = Mock()
        msg.data = "stop"

        waypoint_mission.stop_mission = Mock()

        waypoint_mission._command_callback(msg)

        waypoint_mission.stop_mission.assert_called_once()

    @pytest.mark.unit
    def test_command_callback_unknown(self, waypoint_mission):
        """Test command callback for unknown command."""
        msg = Mock()
        msg.data = "unknown_command"

        waypoint_mission._command_callback(msg)

        # Should log warning
        waypoint_mission.get_logger.return_value.warn.assert_called_with(
            "Unknown command: unknown_command"
        )


class TestWaypointNavigationIntegration:
    """Integration tests for waypoint navigation."""

    @pytest.fixture
    def mock_node(self):
        """Create mock ROS2 node."""
        node = Mock()
        node.create_subscription = Mock()
        node.create_publisher = Mock()
        node.create_client = Mock()
        node.create_timer = Mock()
        node.get_logger = Mock()
        node.get_logger.return_value.info = Mock()
        node.get_logger.return_value.warn = Mock()
        node.get_logger.return_value.error = Mock()
        node.get_logger.return_value.debug = Mock()
        node.get_clock = Mock()
        node.get_clock.return_value.now = Mock()
        node.get_clock.return_value.now.return_value.nanoseconds = 1000000000
        return node

    @pytest.fixture
    def waypoint_mission(self, mock_node):
        """Create WaypointNavigationMission instance."""
        with patch("missions.waypoint_navigation_mission.Node", return_value=mock_node):
            mission = WaypointNavigationMission()
            return mission

    @pytest.mark.integration
    def test_complete_navigation_workflow(self, waypoint_mission):
        """Test complete waypoint navigation workflow."""
        # Setup waypoints
        waypoints_config = [
            {"name": "Start", "x": 0.0, "y": 0.0, "heading": 0.0},
            {"name": "Middle", "x": 5.0, "y": 0.0, "heading": 0.0},
            {"name": "End", "x": 10.0, "y": 0.0, "heading": 0.0},
        ]

        # Load waypoints
        waypoint_mission._load_waypoints_from_config({"waypoints": waypoints_config})

        # Mock navigation client
        mock_future = Mock()
        mock_future.result.return_value = Mock()
        mock_future.result.return_value.success = True

        waypoint_mission.navigation_client.call_async = Mock(return_value=mock_future)

        # Start mission
        waypoint_mission.start_mission()

        # Should transition to INITIALIZING
        assert waypoint_mission.state == WaypointState.INITIALIZING

        # Simulate successful navigation to first waypoint
        waypoint_mission._navigation_done_callback(mock_future)

        # Should move to next waypoint
        assert waypoint_mission.current_waypoint_index == 1

    @pytest.mark.integration
    def test_navigation_failure_handling(self, waypoint_mission):
        """Test navigation failure handling."""
        # Setup waypoint
        waypoint_mission._load_waypoints_from_config(
            {"waypoints": [{"name": "Test", "x": 1.0, "y": 0.0, "heading": 0.0}]}
        )

        # Mock navigation failure
        mock_future = Mock()
        mock_future.result.return_value = Mock()
        mock_future.result.return_value.success = False

        waypoint_mission.navigation_client.call_async = Mock(return_value=mock_future)

        # Start mission
        waypoint_mission.start_mission()

        # Simulate navigation failure
        waypoint_mission._navigation_done_callback(mock_future)

        # Should transition to FAILED state
        assert waypoint_mission.state == WaypointState.FAILED

    @pytest.mark.integration
    def test_pose_and_gps_callbacks(self, waypoint_mission):
        """Test pose and GPS data callbacks."""
        # Mock pose message
        pose_msg = Mock()
        pose_msg.pose.position.x = 1.0
        pose_msg.pose.position.y = 2.0
        pose_msg.pose.position.z = 0.0

        # Call pose callback
        waypoint_mission._pose_callback(pose_msg)

        # Should update current pose
        assert waypoint_mission.current_pose[0] == 1.0
        assert waypoint_mission.current_pose[1] == 2.0

        # Mock GPS message
        gps_msg = Mock()
        gps_msg.latitude = 40.0
        gps_msg.longitude = -74.0

        # Call GPS callback
        waypoint_mission._gps_callback(gps_msg)

        # Should update GPS position
        assert waypoint_mission.current_gps[0] == 40.0
        assert waypoint_mission.current_gps[1] == -74.0


class TestWaypointNavigationPerformance:
    """Performance tests for waypoint navigation."""

    @pytest.fixture
    def mock_node(self):
        """Create mock ROS2 node."""
        node = Mock()
        node.get_logger = Mock()
        node.get_logger.return_value.info = Mock()
        return node

    @pytest.fixture
    def waypoint_mission(self, mock_node):
        """Create WaypointNavigationMission instance."""
        with patch("missions.waypoint_navigation_mission.Node", return_value=mock_node):
            mission = WaypointNavigationMission()
            return mission

    @pytest.mark.performance
    def test_distance_calculation_performance(
        self, waypoint_mission, performance_monitor
    ):
        """Test performance of distance calculations."""
        performance_monitor.start()

        # Perform many distance calculations
        for i in range(1000):
            current_pos = [i * 0.1, i * 0.1, 0.0]
            waypoint = {"position": [i * 0.1 + 1.0, i * 0.1 + 1.0], "heading": 0.0}

            distance = waypoint_mission._calculate_distance_to_waypoint(
                current_pos, waypoint
            )

        elapsed = performance_monitor.get_elapsed_time()

        # Should complete quickly (< 0.1 seconds for 1000 calculations)
        assert elapsed < 0.1, f"Distance calculation too slow: {elapsed} seconds"

    @pytest.mark.performance
    def test_waypoint_validation_performance(
        self, waypoint_mission, performance_monitor
    ):
        """Test performance of waypoint validation."""
        performance_monitor.start()

        # Create many waypoints
        waypoints = [
            {"position": [i * 0.1, i * 0.1], "heading": 0.0} for i in range(100)
        ]

        # Perform validation checks
        current_pos = [0.0, 0.0, 0.0]
        for waypoint in waypoints:
            reached = waypoint_mission._is_waypoint_reached(current_pos, waypoint)

        elapsed = performance_monitor.get_elapsed_time()

        # Should complete quickly (< 0.05 seconds for 100 validations)
        assert elapsed < 0.05, f"Waypoint validation too slow: {elapsed} seconds"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
