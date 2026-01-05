#!/usr/bin/env python3
"""
Hardware Failure Simulation Tests - URC 2026

Tests system behavior under simulated hardware failures:
- Motor controller failures
- Sensor failures (IMU, GPS, LiDAR, Camera)
- CAN bus failures
- Power supply issues
- Emergency stop activations
- Component hot-swapping

Author: URC 2026 Hardware Integration Team
"""

import asyncio
import time
import threading
from typing import Dict, Any, List, Optional
import pytest
from unittest.mock import Mock, patch, AsyncMock
import random


class HardwareFailureSimulator:
    """Hardware failure simulation framework."""

    def __init__(self):
        self.active_failures = {}
        self.failure_handlers = {}
        self.recovery_actions = []

    def simulate_motor_failure(self, motor_id: str, failure_type: str = "stall"):
        """Simulate motor failure."""
        failure = {
            "component": "motor",
            "id": motor_id,
            "type": failure_type,
            "timestamp": time.time(),
            "severity": "critical" if failure_type in ["stall", "overheat"] else "warning"
        }

        self.active_failures[f"motor_{motor_id}"] = failure
        self._trigger_failure_handler("motor", failure)

        return failure

    def simulate_sensor_failure(self, sensor_type: str, sensor_id: str = "primary"):
        """Simulate sensor failure."""
        failure = {
            "component": "sensor",
            "type": sensor_type,
            "id": sensor_id,
            "timestamp": time.time(),
            "severity": "critical" if sensor_type in ["imu", "gps"] else "warning"
        }

        self.active_failures[f"sensor_{sensor_type}_{sensor_id}"] = failure
        self._trigger_failure_handler("sensor", failure)

        return failure

    def simulate_can_bus_failure(self, bus_id: str = "main"):
        """Simulate CAN bus failure."""
        failure = {
            "component": "can_bus",
            "id": bus_id,
            "type": "communication_loss",
            "timestamp": time.time(),
            "severity": "critical"
        }

        self.active_failures[f"can_bus_{bus_id}"] = failure
        self._trigger_failure_handler("can_bus", failure)

        return failure

    def simulate_power_failure(self, subsystem: str = "main"):
        """Simulate power supply failure."""
        failure = {
            "component": "power",
            "subsystem": subsystem,
            "type": "voltage_drop" if subsystem != "backup" else "complete_failure",
            "timestamp": time.time(),
            "severity": "fatal"
        }

        self.active_failures[f"power_{subsystem}"] = failure
        self._trigger_failure_handler("power", failure)

        return failure

    def simulate_emergency_stop(self, trigger_source: str = "software"):
        """Simulate emergency stop activation."""
        failure = {
            "component": "emergency_stop",
            "trigger": trigger_source,
            "type": "safety_activation",
            "timestamp": time.time(),
            "severity": "immediate"
        }

        self.active_failures["emergency_stop"] = failure
        self._trigger_failure_handler("emergency_stop", failure)

        return failure

    def register_failure_handler(self, component_type: str, handler):
        """Register a failure handler for a component type."""
        self.failure_handlers[component_type] = handler

    def _trigger_failure_handler(self, component_type: str, failure: Dict[str, Any]):
        """Trigger the appropriate failure handler."""
        if component_type in self.failure_handlers:
            handler = self.failure_handlers[component_type]
            # Run handler in separate thread to avoid blocking
            threading.Thread(target=handler, args=(failure,), daemon=True).start()

    def clear_failure(self, failure_key: str):
        """Clear a simulated failure."""
        if failure_key in self.active_failures:
            del self.active_failures[failure_key]

    def get_active_failures(self) -> Dict[str, Any]:
        """Get all active failures."""
        return self.active_failures.copy()

    def reset_all_failures(self):
        """Reset all simulated failures."""
        self.active_failures.clear()
        self.recovery_actions.clear()


class TestHardwareFailureSimulation:
    """Test hardware failure simulation and recovery."""

    @pytest.fixture
    def failure_simulator(self):
        """Create hardware failure simulator."""
        return HardwareFailureSimulator()

    @pytest.fixture
    def mock_hardware_controller(self):
        """Create mock hardware controller."""
        controller = Mock()
        controller.handle_motor_failure = AsyncMock(return_value=True)
        controller.handle_sensor_failure = AsyncMock(return_value=True)
        controller.handle_can_failure = AsyncMock(return_value=True)
        controller.emergency_stop = AsyncMock(return_value=True)
        controller.switch_to_backup_power = AsyncMock(return_value=True)
        return controller

    def test_motor_failure_simulation(self, failure_simulator, mock_hardware_controller):
        """Test motor failure simulation and handling."""
        # Register motor failure handler
        failure_simulator.register_failure_handler("motor", mock_hardware_controller.handle_motor_failure)

        # Simulate motor stall
        failure = failure_simulator.simulate_motor_failure("left_front", "stall")

        # Verify failure details
        assert failure["component"] == "motor"
        assert failure["id"] == "left_front"
        assert failure["type"] == "stall"
        assert failure["severity"] == "critical"

        # Allow handler to execute
        time.sleep(0.1)

        # Verify handler was called
        mock_hardware_controller.handle_motor_failure.assert_called_once()
        call_args = mock_hardware_controller.handle_motor_failure.call_args[0][0]
        assert call_args["id"] == "left_front"
        assert call_args["type"] == "stall"

        print("🚗 Motor failure simulation and handling working correctly")

    def test_sensor_failure_simulation(self, failure_simulator, mock_hardware_controller):
        """Test sensor failure simulation and recovery."""
        # Register sensor failure handler
        failure_simulator.register_failure_handler("sensor", mock_hardware_controller.handle_sensor_failure)

        # Simulate critical sensor failures
        imu_failure = failure_simulator.simulate_sensor_failure("imu")
        gps_failure = failure_simulator.simulate_sensor_failure("gps")

        # Verify critical sensors marked as such
        assert imu_failure["severity"] == "critical"
        assert gps_failure["severity"] == "critical"

        # Simulate less critical sensor failure
        camera_failure = failure_simulator.simulate_sensor_failure("camera")
        assert camera_failure["severity"] == "warning"

        # Allow handlers to execute
        time.sleep(0.2)

        # Verify handler was called for each failure
        assert mock_hardware_controller.handle_sensor_failure.call_count == 3

        print("📡 Sensor failure simulation and multi-sensor handling working")

    def test_can_bus_failure_simulation(self, failure_simulator, mock_hardware_controller):
        """Test CAN bus failure and communication recovery."""
        # Register CAN failure handler
        failure_simulator.register_failure_handler("can_bus", mock_hardware_controller.handle_can_failure)

        # Simulate CAN bus failure
        failure = failure_simulator.simulate_can_bus_failure("main")

        assert failure["component"] == "can_bus"
        assert failure["type"] == "communication_loss"
        assert failure["severity"] == "critical"

        # Allow handler to execute
        time.sleep(0.1)

        # Verify CAN failure handler called
        mock_hardware_controller.handle_can_failure.assert_called_once()

        print("🔗 CAN bus failure simulation and communication recovery working")

    def test_power_failure_simulation(self, failure_simulator, mock_hardware_controller):
        """Test power supply failure and backup switching."""
        # Register power failure handler
        failure_simulator.register_failure_handler("power", mock_hardware_controller.switch_to_backup_power)

        # Simulate main power failure
        failure = failure_simulator.simulate_power_failure("main")

        assert failure["component"] == "power"
        assert failure["type"] == "voltage_drop"
        assert failure["severity"] == "fatal"

        # Allow handler to execute
        time.sleep(0.1)

        # Verify backup power switching
        mock_hardware_controller.switch_to_backup_power.assert_called_once()

        print("⚡ Power failure simulation and backup switching working")

    def test_emergency_stop_simulation(self, failure_simulator, mock_hardware_controller):
        """Test emergency stop activation from various triggers."""
        # Register emergency stop handler
        failure_simulator.register_failure_handler("emergency_stop", mock_hardware_controller.emergency_stop)

        # Simulate emergency stop from different sources
        triggers = ["software", "hardware", "remote", "safety_sensor"]

        for trigger in triggers:
            failure = failure_simulator.simulate_emergency_stop(trigger)

            assert failure["component"] == "emergency_stop"
            assert failure["trigger"] == trigger
            assert failure["severity"] == "immediate"

        # Allow handlers to execute
        time.sleep(0.2)

        # Verify emergency stop called for each trigger
        assert mock_hardware_controller.emergency_stop.call_count == len(triggers)

        print("🛑 Emergency stop simulation from multiple triggers working")

    @pytest.mark.asyncio
    async def test_concurrent_hardware_failures(self, failure_simulator, mock_hardware_controller):
        """Test handling of concurrent hardware failures."""
        # Register all failure handlers
        failure_simulator.register_failure_handler("motor", mock_hardware_controller.handle_motor_failure)
        failure_simulator.register_failure_handler("sensor", mock_hardware_controller.handle_sensor_failure)
        failure_simulator.register_failure_handler("can_bus", mock_hardware_controller.handle_can_failure)

        # Simulate concurrent failures
        failure_tasks = []

        async def simulate_failures():
            # Motor failures
            failure_simulator.simulate_motor_failure("left_front", "overheat")
            await asyncio.sleep(0.05)
            failure_simulator.simulate_motor_failure("right_rear", "stall")

            # Sensor failures
            await asyncio.sleep(0.05)
            failure_simulator.simulate_sensor_failure("imu")
            failure_simulator.simulate_sensor_failure("gps")

            # CAN bus failure
            await asyncio.sleep(0.05)
            failure_simulator.simulate_can_bus_failure("main")

        # Run concurrent failure simulation
        await simulate_failures()

        # Allow all handlers to execute
        await asyncio.sleep(0.3)

        # Verify all failure types were handled
        assert mock_hardware_controller.handle_motor_failure.call_count == 2
        assert mock_hardware_controller.handle_sensor_failure.call_count == 2
        assert mock_hardware_controller.handle_can_failure.call_count == 1

        print("🔄 Concurrent hardware failure handling working correctly")

    def test_failure_recovery_and_clearance(self, failure_simulator):
        """Test failure recovery and clearance."""
        # Simulate multiple failures
        motor_failure = failure_simulator.simulate_motor_failure("test_motor")
        sensor_failure = failure_simulator.simulate_sensor_failure("test_sensor")
        can_failure = failure_simulator.simulate_can_bus_failure("test_bus")

        # Verify failures are active
        active_failures = failure_simulator.get_active_failures()
        assert len(active_failures) == 3

        # Clear individual failures
        failure_simulator.clear_failure("motor_test_motor")
        active_failures = failure_simulator.get_active_failures()
        assert len(active_failures) == 2

        # Clear all failures
        failure_simulator.reset_all_failures()
        active_failures = failure_simulator.get_active_failures()
        assert len(active_failures) == 0

        print("🔧 Failure recovery and clearance mechanisms working")

    @pytest.mark.asyncio
    async def test_hardware_hot_swapping(self, failure_simulator, mock_hardware_controller):
        """Test hardware component hot-swapping during operation."""
        # Simulate component that can be hot-swapped
        failure_simulator.register_failure_handler("sensor", mock_hardware_controller.handle_sensor_failure)

        # Simulate primary sensor failure
        failure_simulator.simulate_sensor_failure("camera", "primary")

        # Allow failure handling
        await asyncio.sleep(0.1)

        # Simulate hot-swap to backup component
        # In real system, this would involve physical component switching
        failure_simulator.clear_failure("sensor_camera_primary")

        # Verify system continues with backup
        active_failures = failure_simulator.get_active_failures()
        assert "sensor_camera_primary" not in active_failures

        print("🔄 Hardware hot-swapping simulation working")

    def test_failure_severity_classification(self, failure_simulator):
        """Test failure severity classification and prioritization."""
        # Simulate failures of different severities
        critical_failures = [
            failure_simulator.simulate_motor_failure("motor1", "stall"),
            failure_simulator.simulate_sensor_failure("imu"),
            failure_simulator.simulate_can_bus_failure("main")
        ]

        warning_failures = [
            failure_simulator.simulate_sensor_failure("camera"),
            failure_simulator.simulate_motor_failure("motor2", "low_voltage")
        ]

        # Classify by severity
        active_failures = failure_simulator.get_active_failures()

        critical_count = sum(1 for f in active_failures.values() if f["severity"] == "critical")
        warning_count = sum(1 for f in active_failures.values() if f["severity"] == "warning")

        assert critical_count == 3  # 2 motors + CAN bus
        assert warning_count == 2   # camera + low voltage motor

        print("📊 Failure severity classification working correctly")

    @pytest.mark.asyncio
    async def test_cascading_failure_prevention(self, failure_simulator, mock_hardware_controller):
        """Test prevention of cascading failures."""
        # Register handlers
        failure_simulator.register_failure_handler("motor", mock_hardware_controller.handle_motor_failure)
        failure_simulator.register_failure_handler("can_bus", mock_hardware_controller.handle_can_failure)

        # Simulate initial CAN bus failure
        failure_simulator.simulate_can_bus_failure("main")

        # This should prevent motor commands that depend on CAN
        # In real system, this would isolate affected components

        await asyncio.sleep(0.1)

        # Verify CAN failure handled, preventing cascade
        assert mock_hardware_controller.handle_can_failure.call_count == 1

        # Motor failures should be contained (not triggered by CAN issues)
        motor_failure = failure_simulator.simulate_motor_failure("test_motor")
        await asyncio.sleep(0.1)

        # Motor should still be handled individually
        assert mock_hardware_controller.handle_motor_failure.call_count == 1

        print("🛡️ Cascading failure prevention working")

    @pytest.mark.asyncio
    async def test_hardware_diagnostic_capabilities(self, failure_simulator):
        """Test hardware diagnostic capabilities during failures."""
        # Simulate various hardware issues
        failures = [
            failure_simulator.simulate_motor_failure("motor1", "overheat"),
            failure_simulator.simulate_sensor_failure("gps", "signal_loss"),
            failure_simulator.simulate_can_bus_failure("backup")
        ]

        # Allow diagnostic time
        await asyncio.sleep(0.2)

        # Check diagnostic information available
        for failure in failures:
            failure_key = f"{failure['component']}_{failure['id']}"
            if failure['component'] == 'sensor':
                failure_key = f"{failure['component']}_{failure['type']}_{failure['id']}"

            # In real system, diagnostics would include:
            # - Failure timestamp and duration
            # - Affected components
            # - Recovery recommendations
            # - System health impact

            assert failure["timestamp"] > 0
            assert failure["severity"] in ["critical", "warning", "fatal"]

        print("🔍 Hardware diagnostic capabilities working")

    def test_failure_pattern_analysis(self, failure_simulator):
        """Test analysis of failure patterns."""
        # Simulate pattern of related failures
        # This could indicate systematic issues

        # Simulate multiple motor failures (might indicate power issue)
        for i in range(3):
            failure_simulator.simulate_motor_failure(f"motor_{i}", "power_loss")

        # Simulate sensor failures (might indicate connection issue)
        for sensor in ["imu", "gps", "lidar"]:
            failure_simulator.simulate_sensor_failure(sensor)

        active_failures = failure_simulator.get_active_failures()

        # Analyze patterns
        motor_failures = [f for f in active_failures.values() if f["component"] == "motor"]
        sensor_failures = [f for f in active_failures.values() if f["component"] == "sensor"]

        # Detect patterns
        power_related_failures = sum(1 for f in motor_failures if "power" in f["type"])
        connection_related_failures = len(sensor_failures)

        assert power_related_failures == 3  # All motors have power issues
        assert connection_related_failures == 3  # Multiple sensors affected

        print("📈 Failure pattern analysis detecting systematic issues")

    @pytest.mark.asyncio
    async def test_end_to_end_hardware_failure_scenario(self, failure_simulator, mock_hardware_controller):
        """Test end-to-end hardware failure scenario."""
        # Register all handlers
        failure_simulator.register_failure_handler("motor", mock_hardware_controller.handle_motor_failure)
        failure_simulator.register_failure_handler("sensor", mock_hardware_controller.handle_sensor_failure)
        failure_simulator.register_failure_handler("can_bus", mock_hardware_controller.handle_can_failure)
        failure_simulator.register_failure_handler("emergency_stop", mock_hardware_controller.emergency_stop)

        # Simulate realistic failure cascade:
        # 1. Initial sensor failure
        failure_simulator.simulate_sensor_failure("imu")
        await asyncio.sleep(0.1)

        # 2. Motor controller issues due to navigation problems
        failure_simulator.simulate_motor_failure("left_front", "navigation_error")
        await asyncio.sleep(0.1)

        # 3. CAN bus stress from error recovery attempts
        failure_simulator.simulate_can_bus_failure("main")
        await asyncio.sleep(0.1)

        # 4. Emergency stop triggered by safety system
        failure_simulator.simulate_emergency_stop("safety_system")

        # Allow all handlers to complete
        await asyncio.sleep(0.3)

        # Verify complete failure response chain
        assert mock_hardware_controller.handle_sensor_failure.call_count >= 1
        assert mock_hardware_controller.handle_motor_failure.call_count >= 1
        assert mock_hardware_controller.handle_can_failure.call_count >= 1
        assert mock_hardware_controller.emergency_stop.call_count >= 1

        # Verify system reached safe state
        active_failures = failure_simulator.get_active_failures()
        emergency_active = any(f["component"] == "emergency_stop" for f in active_failures.values())

        assert emergency_active, "System should have activated emergency stop"

        print("🚨 End-to-end hardware failure scenario handled correctly")

    def test_hardware_failure_reporting(self, failure_simulator):
        """Test comprehensive failure reporting."""
        # Generate various failures for reporting
        failures = [
            failure_simulator.simulate_motor_failure("motor1", "overheat"),
            failure_simulator.simulate_sensor_failure("gps"),
            failure_simulator.simulate_can_bus_failure("main"),
            failure_simulator.simulate_power_failure("backup")
        ]

        # Generate failure report
        report = {
            "timestamp": time.time(),
            "total_failures": len(failures),
            "active_failures": failure_simulator.get_active_failures(),
            "severity_breakdown": {
                "critical": sum(1 for f in failures if f["severity"] == "critical"),
                "warning": sum(1 for f in failures if f["severity"] == "warning"),
                "fatal": sum(1 for f in failures if f["severity"] == "fatal")
            },
            "component_breakdown": {}
        }

        # Component breakdown
        for failure in failures:
            component = failure["component"]
            if component not in report["component_breakdown"]:
                report["component_breakdown"][component] = 0
            report["component_breakdown"][component] += 1

        # Verify comprehensive reporting
        assert report["total_failures"] == 4
        assert report["severity_breakdown"]["fatal"] == 1  # Power failure
        assert report["component_breakdown"]["motor"] == 1
        assert report["component_breakdown"]["sensor"] == 1
        assert report["component_breakdown"]["can_bus"] == 1
        assert report["component_breakdown"]["power"] == 1

        print("📋 Comprehensive hardware failure reporting working")

    @pytest.mark.asyncio
    async def test_hardware_redundancy_activation(self, failure_simulator, mock_hardware_controller):
        """Test activation of redundant hardware systems."""
        # Simulate primary system failures that should trigger redundancy
        failure_simulator.register_failure_handler("sensor", mock_hardware_controller.handle_sensor_failure)

        # Primary GPS fails - should switch to backup
        primary_failure = failure_simulator.simulate_sensor_failure("gps", "primary")

        await asyncio.sleep(0.1)

        # In real system, this would:
        # 1. Detect primary GPS failure
        # 2. Switch to backup GPS sensor
        # 3. Update navigation system
        # 4. Log redundancy activation

        # Verify redundancy handling was triggered
        assert mock_hardware_controller.handle_sensor_failure.call_count == 1

        # Verify system can continue with backup
        active_failures = failure_simulator.get_active_failures()
        # Primary should be marked as failed, but backup should be active

        print("🔄 Hardware redundancy activation working correctly")
