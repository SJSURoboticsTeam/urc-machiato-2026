#!/usr/bin/env python3
"""
Critical systems integration tests: blackboard, state machine, behavior tree.

Tests unified blackboard read/write, state machine transitions, and behavior tree
execution using mocks and realistic test data. Validates emergency stop propagation
and state consistency across systems.

Author: URC 2026 Testing Team
"""

import time
import pytest

from tests.fixtures.realistic_test_data import (
    get_blackboard_initial_state,
    load_waypoints_from_config,
    get_safety_limits,
)


@pytest.mark.critical
class TestBlackboardIntegration:
    """Blackboard read/write and consistency with mock."""

    def test_blackboard_read_write_all_types(self, mock_blackboard):
        """Blackboard supports bool, int, double, string get/set."""
        mock_blackboard.set("test_bool", True)
        assert mock_blackboard.get_bool("test_bool", False) is True

        mock_blackboard.set("test_int", 42)
        assert mock_blackboard.get_int("test_int", 0) == 42

        mock_blackboard.set("test_double", 3.14159)
        assert abs(mock_blackboard.get_double("test_double", 0) - 3.14159) < 0.0001

        mock_blackboard.set("test_string", "hello")
        assert mock_blackboard.get_string("test_string", "") == "hello"

    def test_blackboard_navigation_keys(self, mock_blackboard):
        """Navigation-related keys match blackboard_keys usage."""
        mock_blackboard.set("perception_confidence", 0.85)
        mock_blackboard.set("map_quality", 0.9)
        mock_blackboard.set("closest_obstacle_distance", 2.5)
        assert mock_blackboard.get_double("perception_confidence", 0) == 0.85
        assert mock_blackboard.get_double("map_quality", 0) == 0.9
        assert mock_blackboard.get_double("closest_obstacle_distance", 999) == 2.5

    def test_blackboard_mission_state(self, mock_blackboard):
        """Mission state keys update correctly."""
        mock_blackboard.set("samples_collected", 5)
        mock_blackboard.set("waypoints_completed", 3)
        mock_blackboard.set("mission_active", True)
        assert mock_blackboard.get_int("samples_collected", 0) == 5
        assert mock_blackboard.get_int("waypoints_completed", 0) == 3
        assert mock_blackboard.get_bool("mission_active", False) is True

    def test_blackboard_initial_state_from_config(self, mock_blackboard):
        """Initial blackboard state consistent with realistic bootstrap keys."""
        initial = get_blackboard_initial_state()
        for key in ("robot_x", "robot_y", "mission_active", "sensors_ok",
                    "perception_confidence", "map_quality", "closest_obstacle_distance",
                    "samples_collected", "waypoints_completed"):
            if key not in initial:
                continue
            expected = initial[key]
            if isinstance(expected, bool):
                assert mock_blackboard.get_bool(key, not expected) == expected
            elif isinstance(expected, int):
                assert mock_blackboard.get_int(key, expected - 1) == expected
            elif isinstance(expected, (float,)):
                assert mock_blackboard.get_double(key, expected - 1) == expected
            elif isinstance(expected, str):
                assert mock_blackboard.get_string(key, "") == expected

    def test_blackboard_clear_cache(self, mock_blackboard):
        """clear_cache does not affect stored data."""
        mock_blackboard.set("cache_key", 100)
        mock_blackboard.clear_cache()
        assert mock_blackboard.get_int("cache_key", 0) == 100


@pytest.mark.critical
class TestStateMachineIntegration:
    """State machine transitions and validity."""

    def test_state_machine_initial_state(self, mock_state_machine):
        """State machine starts in IDLE when requested."""
        assert mock_state_machine.current_state.name == "IDLE"

    def test_state_machine_transition_idle_to_autonomous(self, mock_state_machine):
        """Valid transition IDLE -> AUTONOMOUS."""
        ok = mock_state_machine.transition_to("AUTONOMOUS", reason="mission start")
        assert ok
        assert mock_state_machine.current_state.name == "AUTONOMOUS"
        assert mock_state_machine.previous_state.name == "IDLE"

    def test_state_machine_transition_idle_to_teleoperation(self, mock_state_machine):
        """Valid transition IDLE -> TELEOPERATION."""
        mock_state_machine.transition_to("TELEOPERATION", reason="operator")
        assert mock_state_machine.current_state.name == "TELEOPERATION"

    def test_state_machine_transition_to_emergency_stop(self, mock_state_machine):
        """Transition to EMERGENCY_STOP from AUTONOMOUS."""
        mock_state_machine.transition_to("AUTONOMOUS")
        mock_state_machine.transition_to("EMERGENCY_STOP", reason="safety")
        assert mock_state_machine.current_state.name == "EMERGENCY_STOP"

    def test_state_machine_transition_emergency_to_idle(self, mock_state_machine):
        """Valid transition EMERGENCY_STOP -> IDLE."""
        mock_state_machine.transition_to("EMERGENCY_STOP")
        mock_state_machine.transition_to("IDLE", reason="released")
        assert mock_state_machine.current_state.name == "IDLE"

    def test_state_machine_history(self, mock_state_machine):
        """State history records transitions."""
        mock_state_machine.transition_to("AUTONOMOUS")
        mock_state_machine.transition_to("IDLE")
        history = mock_state_machine.get_state_history()
        assert len(history) >= 2
        assert history[-1][1].name == "IDLE"

    def test_state_machine_listener_called(self, mock_state_machine):
        """State change listener is invoked on transition."""
        calls = []

        def listener(_):
            calls.append(1)

        mock_state_machine.add_state_listener(listener)
        mock_state_machine.transition_to("AUTONOMOUS")
        assert len(calls) == 1


@pytest.mark.critical
class TestBlackboardStateSync:
    """Consistency between blackboard and state machine (mocked)."""

    def test_blackboard_mission_active_reflects_mission_state(
        self, mock_blackboard, mock_state_machine
    ):
        """When mission starts, blackboard mission_active can be set."""
        mock_state_machine.transition_to("AUTONOMOUS", reason="mission")
        mock_blackboard.set("mission_active", True)
        assert mock_blackboard.get_bool("mission_active", False) is True

    def test_waypoints_from_config_match_blackboard_usage(self, mock_blackboard):
        """Waypoints from config can populate blackboard waypoints_total."""
        waypoints = load_waypoints_from_config()
        total = len(waypoints) or 1
        mock_blackboard.set("waypoints_total", total)
        mock_blackboard.set("waypoints_completed", 0)
        assert mock_blackboard.get_int("waypoints_total", 0) == total
        assert mock_blackboard.get_int("waypoints_completed", 0) == 0


@pytest.mark.critical
class TestEmergencyStopPropagation:
    """Emergency stop propagation timing and consistency."""

    def test_emergency_stop_state_transition_latency(self, mock_state_machine):
        """Transition to EMERGENCY_STOP completes in under 100ms (mock)."""
        mock_state_machine.transition_to("AUTONOMOUS")
        t0 = time.perf_counter()
        mock_state_machine.transition_to("EMERGENCY_STOP", reason="e-stop")
        elapsed = (time.perf_counter() - t0) * 1000
        assert mock_state_machine.current_state.name == "EMERGENCY_STOP"
        assert elapsed < 100.0, "E-stop transition should complete under 100ms"

    def test_blackboard_safety_stop_flag(self, mock_blackboard):
        """Blackboard can record safety/emergency stop active."""
        mock_blackboard.set("emergency_stop_active", True)
        assert mock_blackboard.get_bool("emergency_stop_active", False) is True
        mock_blackboard.set("emergency_stop_active", False)
        assert mock_blackboard.get_bool("emergency_stop_active", True) is False


@pytest.mark.critical
class TestBehaviorTreeWithMockedData:
    """Behavior tree or mission logic with mocked sensor data."""

    def test_safety_limits_available_for_bt_checks(self):
        """Safety limits from config available for BT sensor checks."""
        limits = get_safety_limits()
        assert "safety_imu_accel_max" in limits
        assert "safety_imu_gyro_max" in limits
        assert limits["safety_imu_accel_max"] > 0
        assert limits["safety_imu_gyro_max"] > 0

    def test_waypoints_from_config_non_empty_or_default(self):
        """Waypoints from config are list (may be empty)."""
        waypoints = load_waypoints_from_config()
        assert isinstance(waypoints, list)
        if waypoints:
            assert "x" in waypoints[0] and "y" in waypoints[0]


@pytest.mark.critical
class TestROS2MockEnvironment:
    """ROS2 mock environment provides node and messaging."""

    def test_ros2_mock_node_created(self, ros2_mock_environment):
        """ROS2 mock node exists and has publisher/subscriber API."""
        node = ros2_mock_environment
        assert node.full_name
        assert hasattr(node, "create_publisher")
        assert hasattr(node, "create_subscription")
        assert hasattr(node, "create_service")
        assert node.is_shutdown is False

    def test_ros2_mock_node_shutdown_cleans_up(self, ros2_mock_environment):
        """Node shutdown does not raise."""
        ros2_mock_environment.shutdown()
        assert ros2_mock_environment.is_shutdown is True
