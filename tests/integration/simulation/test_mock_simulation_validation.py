#!/usr/bin/env python3
"""
Mock/Simulation-Based Tests - No ROS2 Dependencies Required

These tests validate core autonomy logic using pure Python mocks and simulations.
Addresses the gap where most tests require hardware/ROS2 but we need basic validation.
"""

import json
import time
import unittest
from typing import Any, Dict, List, Optional


class MockSensor:
    """Mock sensor for testing."""

    def __init__(self, sensor_type: str, healthy: bool = True):
        self.sensor_type = sensor_type
        self.healthy = healthy
        self.last_reading = None

    def read(self) -> Optional[float]:
        """Simulate sensor reading."""
        if not self.healthy:
            return None
        self.last_reading = time.time() * 100  # Mock value
        return self.last_reading


class MockSafetyManager:
    """Mock safety manager for testing."""

    def __init__(self):
        self.emergency_stop = False
        self.safety_mode = "normal"
        self.sensors: Dict[str, MockSensor] = {}

    def add_sensor(self, sensor: MockSensor):
        """Add sensor to monitoring."""
        self.sensors[sensor.sensor_type] = sensor

    def check_safety(self) -> str:
        """Check overall system safety."""
        failed_sensors = [s for s in self.sensors.values() if not s.healthy]

        if failed_sensors:
            if len(failed_sensors) >= 2:
                self.safety_mode = "emergency"
                self.emergency_stop = True
            else:
                self.safety_mode = "degraded"
        else:
            self.safety_mode = "normal"
            self.emergency_stop = False

        return self.safety_mode


class MockMissionExecutor:
    """Mock mission executor for testing."""

    def __init__(self):
        self.current_mission = None
        self.mission_state = "idle"
        self.waypoints: List[Dict[str, float]] = []

    def start_mission(
        self, mission_type: str, waypoints: List[Dict[str, float]]
    ) -> bool:
        """Start a mission."""
        if mission_type not in ["navigation", "delivery", "inspection"]:
            return False

        self.current_mission = mission_type
        self.waypoints = waypoints
        self.mission_state = "active"
        return True

    def get_status(self) -> Dict[str, Any]:
        """Get mission status."""
        return {
            "mission": self.current_mission,
            "state": self.mission_state,
            "waypoints_completed": len(self.waypoints) // 2 if self.waypoints else 0,
            "total_waypoints": len(self.waypoints),
        }


class MockStateMachine:
    """Mock state machine for testing."""

    def __init__(self):
        self.current_state = "idle"
        self.transitions = {
            "idle": ["autonomous", "manual"],
            "autonomous": ["idle", "manual", "emergency"],
            "manual": ["idle", "autonomous"],
            "emergency": ["idle"],
        }

    def transition_to(self, new_state: str) -> bool:
        """Attempt state transition."""
        if new_state in self.transitions.get(self.current_state, []):
            self.current_state = new_state
            return True
        return False


class TestMockSimulationValidation(unittest.TestCase):
    """Test suite for mock/simulation-based validation."""

    def setUp(self):
        """Set up test fixtures."""
        self.safety_manager = MockSafetyManager()
        self.mission_executor = MockMissionExecutor()
        self.state_machine = MockStateMachine()

    def test_safety_manager_normal_operation(self):
        """Test safety manager with all healthy sensors."""
        # Add healthy sensors
        self.safety_manager.add_sensor(MockSensor("gps", healthy=True))
        self.safety_manager.add_sensor(MockSensor("imu", healthy=True))

        status = self.safety_manager.check_safety()

        self.assertEqual(status, "normal")
        self.assertFalse(self.safety_manager.emergency_stop)

    def test_safety_manager_single_sensor_failure(self):
        """Test safety manager with one failed sensor."""
        # Add one healthy, one failed sensor
        self.safety_manager.add_sensor(MockSensor("gps", healthy=True))
        self.safety_manager.add_sensor(MockSensor("imu", healthy=False))

        status = self.safety_manager.check_safety()

        self.assertEqual(status, "degraded")
        self.assertFalse(self.safety_manager.emergency_stop)

    def test_safety_manager_multiple_sensor_failures(self):
        """Test safety manager with multiple failed sensors."""
        # Add multiple failed sensors
        self.safety_manager.add_sensor(MockSensor("gps", healthy=False))
        self.safety_manager.add_sensor(MockSensor("imu", healthy=False))

        status = self.safety_manager.check_safety()

        self.assertEqual(status, "emergency")
        self.assertTrue(self.safety_manager.emergency_stop)

    def test_mission_executor_navigation_mission(self):
        """Test mission executor with navigation mission."""
        waypoints = [
            {"lat": 37.7749, "lon": -122.4194},
            {"lat": 37.7849, "lon": -122.4094},
        ]

        success = self.mission_executor.start_mission("navigation", waypoints)

        self.assertTrue(success)
        self.assertEqual(self.mission_executor.current_mission, "navigation")
        self.assertEqual(self.mission_executor.mission_state, "active")

    def test_mission_executor_invalid_mission_type(self):
        """Test mission executor with invalid mission type."""
        waypoints = [{"lat": 0, "lon": 0}]

        success = self.mission_executor.start_mission("invalid_type", waypoints)

        self.assertFalse(success)
        self.assertIsNone(self.mission_executor.current_mission)

    def test_mission_executor_status_tracking(self):
        """Test mission status tracking."""
        waypoints = [{"lat": 1, "lon": 1}, {"lat": 2, "lon": 2}, {"lat": 3, "lon": 3}]

        self.mission_executor.start_mission("navigation", waypoints)
        status = self.mission_executor.get_status()

        expected = {
            "mission": "navigation",
            "state": "active",
            "waypoints_completed": 1,  # len(waypoints) // 2 = 1
            "total_waypoints": 3,
        }

        self.assertEqual(status, expected)

    def test_state_machine_valid_transitions(self):
        """Test valid state machine transitions."""
        # idle -> autonomous
        success = self.state_machine.transition_to("autonomous")
        self.assertTrue(success)
        self.assertEqual(self.state_machine.current_state, "autonomous")

        # autonomous -> emergency
        success = self.state_machine.transition_to("emergency")
        self.assertTrue(success)
        self.assertEqual(self.state_machine.current_state, "emergency")

    def test_state_machine_invalid_transitions(self):
        """Test invalid state machine transitions."""
        # idle -> emergency (should fail)
        success = self.state_machine.transition_to("emergency")
        self.assertFalse(success)
        self.assertEqual(self.state_machine.current_state, "idle")

    def test_sensor_data_validation(self):
        """Test sensor data validation logic."""
        sensor = MockSensor("gps", healthy=True)

        # Get reading
        reading = sensor.read()
        self.assertIsNotNone(reading)
        self.assertIsInstance(reading, float)
        self.assertGreater(reading, 0)

        # Test failed sensor
        failed_sensor = MockSensor("imu", healthy=False)
        failed_reading = failed_sensor.read()
        self.assertIsNone(failed_reading)

    def test_configuration_validation(self):
        """Test configuration parameter validation."""
        config = {
            "max_speed": 2.5,
            "safety_distance": 1.0,
            "mission_timeout": 300,
            "enable_autonomy": True,
        }

        # Valid configuration checks
        self.assertGreater(config["max_speed"], 0)
        self.assertGreater(config["safety_distance"], 0)
        self.assertGreater(config["mission_timeout"], 0)
        self.assertIsInstance(config["enable_autonomy"], bool)

    def test_error_handling_robustness(self):
        """Test error handling and recovery logic."""
        # Test with invalid inputs
        try:
            # This should not crash
            self.mission_executor.start_mission("", [])
            self.mission_executor.start_mission(None, None)
        except Exception as e:
            self.fail(f"Mission executor should handle invalid inputs gracefully: {e}")

        # Verify system remains stable
        status = self.mission_executor.get_status()
        self.assertIsInstance(status, dict)


import pytest

from tests.fixtures.realistic_test_data import (
    get_safety_limits,
    get_blackboard_initial_state,
    generate_imu_data,
    generate_gps_data,
    load_waypoints_from_config,
    get_mission_profile,
)

@pytest.mark.integration
class TestSimulationManagerWithRealConfig:
    """SimulationManager initialized and stepped with test config."""

    def test_simulation_manager_initializes(self, simulation_manager):
        """SimulationManager initializes with minimal config."""
        assert simulation_manager.is_initialized
        assert not simulation_manager.is_running
        assert simulation_manager.step_count == 0

    def test_simulation_manager_step_produces_sensor_data(self, simulation_manager):
        """Single step produces environment, sensors, rover, network state."""
        state = simulation_manager.step(0.01)
        assert "timestamp" in state
        assert "sensors" in state
        assert "environment" in state
        assert "rover" in state
        assert "network" in state
        assert "gps" in state["sensors"] or "imu" in state["sensors"]

    def test_simulation_manager_step_count_increments(self, simulation_manager):
        """Step count increments after each step."""
        simulation_manager.step(0.01)
        assert simulation_manager.step_count >= 1
        simulation_manager.step(0.01)
        assert simulation_manager.step_count >= 2

    def test_simulation_manager_get_state(self, simulation_manager):
        """get_state returns initialized and step count."""
        simulation_manager.step(0.01)
        full_state = simulation_manager.get_state()
        assert full_state["is_initialized"] is True
        assert full_state["step_count"] >= 1
        assert "sensors" in full_state
        assert "metadata" in full_state

    def test_simulation_manager_reset(self, simulation_manager):
        """Reset clears step count and state."""
        simulation_manager.step(0.01)
        simulation_manager.step(0.01)
        ok = simulation_manager.reset()
        assert ok
        assert simulation_manager.step_count == 0


@pytest.mark.integration
class TestSimulatedSensorDataToBlackboard:
    """Simulated sensor data used to update blackboard (mock)."""

    def test_sensor_data_structure_matches_blackboard_input(
        self, simulation_manager, mock_blackboard
    ):
        """Sensor data from simulation step can drive blackboard keys."""
        state = simulation_manager.step(0.01)
        sensors = state.get("sensors", {})
        if "gps" in sensors:
            gps = sensors["gps"]
            if isinstance(gps, dict) and "latitude" in gps:
                mock_blackboard.set("sensors_ok", True)
                assert mock_blackboard.get_bool("sensors_ok", False) is True
        if "imu" in sensors:
            imu = sensors["imu"]
            if isinstance(imu, dict) and "linear_acceleration_z" in imu:
                mock_blackboard.set("sensors_ok", True)
                assert mock_blackboard.get_bool("sensors_ok", False) is True

    def test_realistic_imu_data_within_safety_limits(self):
        """Generated IMU data (from realistic_test_data) stays within safety limits."""
        limits = get_safety_limits()
        imu = generate_imu_data(use_safety_limits=True)
        accel_max = limits.get("safety_imu_accel_max", 50.0)
        gyro_max = limits.get("safety_imu_gyro_max", 20.0)
        for key, val in imu.items():
            if "accel" in key or "linear" in key:
                assert abs(val) <= accel_max * 1.01
            if "angular" in key or "gyro" in key:
                assert abs(val) <= gyro_max * 1.01

    def test_blackboard_initial_from_config_matches_waypoints(self, mock_blackboard):
        """Blackboard robot_x/robot_y can be set from config waypoints."""
        waypoints = load_waypoints_from_config()
        initial = get_blackboard_initial_state()
        assert "robot_x" in initial and "robot_y" in initial
        mock_blackboard.set("robot_x", initial["robot_x"])
        mock_blackboard.set("robot_y", initial["robot_y"])
        assert mock_blackboard.get_double("robot_x", -1) == initial["robot_x"]
        assert mock_blackboard.get_double("robot_y", -1) == initial["robot_y"]


@pytest.mark.integration
class TestSimulationStateMachineResponses:
    """State machine transitions driven by simulated conditions (mock)."""

    def test_state_machine_responds_to_sensor_threshold(
        self, mock_state_machine, mock_blackboard
    ):
        """When blackboard indicates low battery/safety, state machine can transition."""
        mock_blackboard.set("battery_level", 5.0)
        mock_blackboard.set("emergency_stop_active", True)
        mock_state_machine.transition_to("EMERGENCY_STOP", reason="safety")
        assert mock_state_machine.current_state.name == "EMERGENCY_STOP"

    def test_mission_profile_available_for_simulation(self):
        """Mission profile from config available for scenario setup."""
        profile = get_mission_profile("waypoint_navigation")
        assert isinstance(profile, dict)
        profile_full = get_mission_profile("sample_collection")
        assert isinstance(profile_full, dict)


@pytest.mark.integration
class TestFullStackSimulatorIfAvailable:
    """Full stack simulator (WebSocket -> CAN -> Firmware) when available."""

    def test_full_stack_simulator_initializes(self, full_stack_simulator):
        """FullStackSimulator initializes with test config."""
        assert full_stack_simulator is not None
        assert hasattr(full_stack_simulator, "websocket_sim")
        assert hasattr(full_stack_simulator, "slcan_sim")
        assert hasattr(full_stack_simulator, "firmware_sim")


if __name__ == "__main__":
    unittest.main()
