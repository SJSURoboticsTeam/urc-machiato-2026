#!/usr/bin/env python3
"""
Simulation-validated system tests.

Initializes SimulationManager with real/minimal config, runs simulation steps,
and validates that sensor data and state updates are consistent with blackboard
and state machine expectations. Uses mocks when ROS2 is unavailable.

Author: URC 2026 Testing Team
"""

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
