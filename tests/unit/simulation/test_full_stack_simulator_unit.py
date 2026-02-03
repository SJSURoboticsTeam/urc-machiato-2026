#!/usr/bin/env python3
"""
Unit Tests for Full Stack Simulator

Tests the complete communication stack orchestration including
component initialization, pipeline setup, and scenario execution.

Author: URC 2026 Testing Team
"""

import pytest
import asyncio
import time
import sys
from pathlib import Path
from typing import Dict, Any

# Add simulation to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

pytest.importorskip("simulation.network.websocket_server_simulator")
from simulation.integration.full_stack_simulator import (
    FullStackSimulator,
    CommunicationPath,
    ScenarioType,
    ScenarioResult,
    create_full_stack_simulator,
)


class TestFullStackSimulator:
    """Test suite for full stack simulator."""

    @pytest.fixture
    def simulator(self):
        """Create basic simulator for testing."""
        sim = create_full_stack_simulator("default")
        yield sim
        sim.shutdown()

    @pytest.fixture
    def perfect_simulator(self):
        """Create perfect simulator with no delays/errors."""
        sim = create_full_stack_simulator("perfect")
        yield sim
        sim.shutdown()

    # Initialization Tests

    def test_simulator_initialization(self, simulator):
        """Test simulator initializes all components."""
        assert simulator is not None
        assert simulator.websocket_sim is not None
        assert simulator.slcan_sim is not None
        assert simulator.firmware_sim is not None

    def test_ros2_state_initialization(self, simulator):
        """Test ROS2 state is properly initialized."""
        assert "cmd_vel_teleop" in simulator.ros2_state
        assert "cmd_vel_autonomy" in simulator.ros2_state
        assert "emergency_stop" in simulator.ros2_state
        assert simulator.ros2_state["emergency_stop"] is False

    def test_pipeline_setup(self, simulator):
        """Test communication pipeline is set up."""
        # Event handlers should be registered
        assert "driveCommands" in simulator.websocket_sim.event_handlers
        assert "emergencyStop" in simulator.websocket_sim.event_handlers

    def test_firmware_control_loop_started(self, simulator):
        """Test firmware simulator control loop is started."""
        # Firmware should be running
        time.sleep(0.1)
        assert simulator.firmware_sim.stats["control_cycles"] > 0

    # Scenario Execution Tests

    async def test_run_basic_velocity_scenario(self, perfect_simulator):
        """Test running basic velocity scenario."""
        result = await perfect_simulator.run_scenario(ScenarioType.BASIC_VELOCITY)

        assert result is not None
        assert result.scenario_type == ScenarioType.BASIC_VELOCITY
        assert result.success is True
        assert result.duration_s > 0

    async def test_run_emergency_stop_scenario(self, perfect_simulator):
        """Test running emergency stop scenario."""
        result = await perfect_simulator.run_scenario(ScenarioType.EMERGENCY_STOP)

        assert result is not None
        assert result.scenario_type == ScenarioType.EMERGENCY_STOP
        assert result.success is True
        # E-stop should be fast
        assert result.duration_s < 0.5

    async def test_run_network_failure_scenario(self, simulator):
        """Test running network failure scenario."""
        result = await simulator.run_scenario(ScenarioType.NETWORK_FAILURE)

        assert result is not None
        assert result.scenario_type == ScenarioType.NETWORK_FAILURE
        # May or may not succeed depending on packet loss

    async def test_run_firmware_fault_scenario(self, simulator):
        """Test running firmware fault scenario."""
        result = await simulator.run_scenario(ScenarioType.FIRMWARE_FAULT)

        assert result is not None
        assert result.scenario_type == ScenarioType.FIRMWARE_FAULT

    async def test_run_high_load_scenario(self, simulator):
        """Test running high load scenario."""
        result = await simulator.run_scenario(ScenarioType.HIGH_LOAD)

        assert result is not None
        assert result.scenario_type == ScenarioType.HIGH_LOAD
        assert result.messages_sent > 10  # Many messages

    async def test_run_recovery_scenario(self, simulator):
        """Test running recovery scenario."""
        result = await simulator.run_scenario(ScenarioType.RECOVERY)

        assert result is not None
        assert result.scenario_type == ScenarioType.RECOVERY

    async def test_scenario_result_structure(self, perfect_simulator):
        """Test scenario result has correct structure."""
        result = await perfect_simulator.run_scenario(ScenarioType.BASIC_VELOCITY)

        assert hasattr(result, "scenario_type")
        assert hasattr(result, "success")
        assert hasattr(result, "duration_s")
        assert hasattr(result, "messages_sent")
        assert hasattr(result, "messages_received")
        assert hasattr(result, "errors")
        assert hasattr(result, "metrics")
        assert hasattr(result, "timestamp")

    async def test_scenario_with_custom_parameters(self, perfect_simulator):
        """Test running scenario with custom parameters."""
        params = {"duration": 0.5, "velocity": 0.8}

        result = await perfect_simulator.run_scenario(
            ScenarioType.BASIC_VELOCITY, params
        )

        assert result is not None
        assert result.success is True

    # Test Suite Execution Tests

    async def test_run_full_test_suite(self, perfect_simulator):
        """Test running complete test suite."""
        summary = await perfect_simulator.run_test_suite()

        assert "total_scenarios" in summary
        assert "passed" in summary
        assert "failed" in summary
        assert "pass_rate" in summary
        assert "duration_s" in summary

        assert summary["total_scenarios"] > 0
        assert 0.0 <= summary["pass_rate"] <= 1.0

    async def test_test_suite_runs_all_scenarios(self, perfect_simulator):
        """Test suite runs all scenario types."""
        summary = await perfect_simulator.run_test_suite()

        # Should run at least 5 scenarios
        assert summary["total_scenarios"] >= 5

    async def test_test_suite_statistics(self, perfect_simulator):
        """Test suite tracks results correctly."""
        await perfect_simulator.run_test_suite()

        stats = perfect_simulator.stats
        assert stats["scenarios_run"] > 0
        assert (
            stats["scenarios_passed"] + stats["scenarios_failed"]
            == stats["scenarios_run"]
        )

    # Communication Path Tests

    async def test_teleoperation_path(self, perfect_simulator):
        """Test teleoperation communication path."""
        simulator = perfect_simulator

        # Simulate frontend drive command
        client_id = await simulator.websocket_sim.connect()
        await simulator.websocket_sim.receive(
            "driveCommands", {"linear": 0.5, "angular": 0.2}, client_id
        )

        await asyncio.sleep(0.1)

        # Check command reached ROS2 state
        assert simulator.ros2_state["cmd_vel_teleop"] is not None
        assert simulator.ros2_state["cmd_vel_teleop"]["linear"] == 0.5

    async def test_emergency_stop_propagation(self, perfect_simulator):
        """Test emergency stop propagates through stack."""
        simulator = perfect_simulator

        client_id = await simulator.websocket_sim.connect()
        await simulator.websocket_sim.receive("emergencyStop", {}, client_id)

        await asyncio.sleep(0.05)

        # Check e-stop reached firmware
        assert simulator.firmware_sim.emergency_stop_active is True

    async def test_homing_command_propagation(self, perfect_simulator):
        """Test homing command propagates to firmware."""
        simulator = perfect_simulator

        client_id = await simulator.websocket_sim.connect()
        await simulator.websocket_sim.receive("homingRequest", {}, client_id)

        await asyncio.sleep(0.05)

        # Check homing started
        from simulation.firmware.stm32_firmware_simulator import HomingState

        assert simulator.firmware_sim.homing_state == HomingState.HOMING

    # Event Logging Tests

    async def test_event_logging(self, perfect_simulator):
        """Test events are logged."""
        simulator = perfect_simulator

        client_id = await simulator.websocket_sim.connect()
        await simulator.websocket_sim.receive(
            "driveCommands", {"linear": 0.5, "angular": 0.0}, client_id
        )

        await asyncio.sleep(0.1)

        # Events should be logged
        assert len(simulator.event_queue) > 0

    async def test_event_queue_limit(self, simulator):
        """Test event queue respects size limit."""
        simulator.max_queue_size = 10

        client_id = await simulator.websocket_sim.connect()

        # Generate many events
        for i in range(50):
            await simulator.websocket_sim.receive(
                "driveCommands", {"linear": 0.5, "angular": 0.0}, client_id
            )

        await asyncio.sleep(0.1)

        # Queue should be limited
        assert len(simulator.event_queue) <= 10

    # Metrics Collection Tests

    def test_collect_metrics(self, simulator):
        """Test metrics collection."""
        metrics = simulator._collect_metrics()

        assert "websocket" in metrics
        assert "slcan" in metrics
        assert "firmware" in metrics

    def test_metrics_include_statistics(self, simulator):
        """Test metrics include component statistics."""
        metrics = simulator._collect_metrics()

        # WebSocket stats
        assert "connected_clients" in metrics["websocket"]
        assert "stats" in metrics["websocket"]

        # SLCAN stats
        assert "success_rate" in metrics["slcan"]

        # Firmware stats
        assert "num_motors" in metrics["firmware"]
        assert "emergency_stop" in metrics["firmware"]

    # Status and Reset Tests

    def test_get_status(self, simulator):
        """Test getting simulator status."""
        status = simulator.get_status()

        assert "components" in status
        assert "ros2_state" in status
        assert "stats" in status
        assert "scenario_history" in status

    def test_status_includes_all_components(self, simulator):
        """Test status includes all component states."""
        status = simulator.get_status()

        components = status["components"]
        assert "websocket" in components
        assert "slcan" in components
        assert "firmware" in components

    def test_reset_simulator(self, simulator):
        """Test resetting simulator state."""
        # Generate some activity
        asyncio.run(simulator.run_scenario(ScenarioType.BASIC_VELOCITY))

        initial_scenarios = simulator.stats["scenarios_run"]
        assert initial_scenarios > 0

        # Reset
        simulator.reset()

        # Stats should be cleared
        assert simulator.stats["scenarios_run"] == 0
        assert len(simulator.scenario_results) == 0
        assert len(simulator.event_queue) == 0

    # Shutdown Tests

    def test_shutdown_simulator(self, simulator):
        """Test clean shutdown."""
        # Should not raise exception
        simulator.shutdown()

        # Firmware should be stopped
        assert simulator.firmware_sim.running is False

    # Factory Function Tests

    def test_create_default_simulator(self):
        """Test factory creates default configuration."""
        sim = create_full_stack_simulator("default")
        assert sim is not None
        assert sim.websocket_sim.network_conditions.latency_ms == 50.0
        sim.shutdown()

    def test_create_perfect_simulator(self):
        """Test factory creates perfect configuration."""
        sim = create_full_stack_simulator("perfect")
        assert sim is not None
        assert sim.websocket_sim.network_conditions.latency_ms == 0.0
        assert sim.slcan_sim.error_rate == 0.0
        sim.shutdown()

    def test_create_stressed_simulator(self):
        """Test factory creates stressed configuration."""
        sim = create_full_stack_simulator("stressed")
        assert sim is not None
        assert sim.websocket_sim.network_conditions.latency_ms > 100.0
        sim.shutdown()

    # Error Handling Tests

    async def test_invalid_scenario_type(self, simulator):
        """Test handling invalid scenario type."""
        # This should either raise an exception or return an error result
        # depending on implementation
        try:
            result = await simulator.run_scenario("invalid_scenario")
            # If it returns, check it's marked as failure
            if result:
                assert result.success is False
        except (ValueError, AttributeError, TypeError):
            # Expected exception
            pass

    async def test_scenario_error_handling(self, simulator):
        """Test scenarios handle errors gracefully."""
        # Even with errors, should return result
        result = await simulator.run_scenario(ScenarioType.NETWORK_FAILURE)

        assert result is not None
        assert hasattr(result, "errors")
        # Errors list exists even if empty
        assert isinstance(result.errors, list)

    # Integration Tests

    async def test_multiple_scenarios_in_sequence(self, perfect_simulator):
        """Test running multiple scenarios sequentially."""
        simulator = perfect_simulator

        result1 = await simulator.run_scenario(ScenarioType.BASIC_VELOCITY)
        result2 = await simulator.run_scenario(ScenarioType.EMERGENCY_STOP)
        result3 = await simulator.run_scenario(ScenarioType.RECOVERY)

        assert result1.success is True
        assert result2.success is True
        assert result3.success is True

        # Results should be stored
        assert len(simulator.scenario_results) >= 3

    async def test_concurrent_components(self, perfect_simulator):
        """Test all components work together concurrently."""
        simulator = perfect_simulator

        # Start multiple actions
        client_id = await simulator.websocket_sim.connect()

        # Send commands
        await simulator.websocket_sim.receive(
            "driveCommands", {"linear": 0.5, "angular": 0.2}, client_id
        )

        # Wait for processing
        await asyncio.sleep(0.2)

        # Check all components active
        ws_stats = simulator.websocket_sim.get_statistics()
        slcan_stats = simulator.slcan_sim.get_statistics()
        firmware_status = simulator.firmware_sim.get_system_status()

        assert ws_stats["stats"]["messages_received"] > 0
        assert slcan_stats["frames_sent"] > 0
        assert firmware_status["uptime_s"] > 0


# Performance Tests
@pytest.mark.benchmark
class TestFullStackSimulatorPerformance:
    """Performance tests for full stack simulator."""

    async def test_scenario_execution_time(self, perfect_simulator):
        """Test scenario execution is reasonably fast."""
        start = time.time()

        await perfect_simulator.run_scenario(
            ScenarioType.BASIC_VELOCITY, {"duration": 0.1}
        )

        elapsed = time.time() - start

        # Should complete quickly (< 1 second)
        assert elapsed < 1.0, f"Scenario too slow: {elapsed:.3f}s"

    async def test_test_suite_throughput(self, perfect_simulator):
        """Test complete suite runs in reasonable time."""
        start = time.time()

        await perfect_simulator.run_test_suite()

        elapsed = time.time() - start

        # Complete suite should run in < 10 seconds
        assert elapsed < 10.0, f"Test suite too slow: {elapsed:.1f}s"

    async def test_message_throughput(self, perfect_simulator):
        """Test message processing throughput."""
        simulator = perfect_simulator
        client_id = await simulator.websocket_sim.connect()

        start = time.time()
        count = 100

        for i in range(count):
            await simulator.websocket_sim.receive(
                "driveCommands", {"linear": 0.5, "angular": 0.0}, client_id
            )

        elapsed = time.time() - start
        throughput = count / elapsed

        # Should handle at least 200 msg/s
        assert throughput > 200, f"Throughput too low: {throughput:.0f} msg/s"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-k", "not benchmark"])
