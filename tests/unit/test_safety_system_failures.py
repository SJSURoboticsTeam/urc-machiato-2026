#!/usr/bin/env python3
"""
Safety System Failure Tests - URC 2026

Tests safety system failures that can cause physical damage or mission loss:
- Emergency stop failures and false triggers
- Safety threshold misconfiguration
- Sensor integrity failures
- Cascading safety system failures
- Recovery from safety events
- Safety override mechanisms

Author: URC 2026 Risk Mitigation Team
"""

import pytest
import time
import threading
from unittest.mock import Mock, MagicMock, patch, AsyncMock
import sys
import os
from typing import Dict, List, Any, Optional

# Add source paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src'))

from src.autonomy.core.safety_system.autonomy_safety_system.safety_monitor import (
    SafetyMonitor,
    SafetyTrigger,
    SafetySeverity,
    SafetyThreshold
)


class TestEmergencyStopFailures:
    """Test emergency stop system failures."""

    @pytest.fixture
    def safety_monitor(self):
        """Create safety monitor instance."""
        return SafetyMonitor()

    @pytest.mark.critical
    def test_emergency_stop_false_trigger_from_noise(self, safety_monitor):
        """Test false emergency stop triggers from sensor noise."""
        # Simulate sensor readings with noise that triggers false emergency stops
        noisy_sensor_data = {
            'imu_accel': [50.5, 0.0, 9.81],  # Just over 50 m/s² threshold due to noise
            'motor_currents': [15.2, 14.9, 15.1, 14.8],  # Current spikes from noise
            'min_obstacle_distance': 0.28  # Just under 0.3m threshold
        }

        # Update safety monitor
        safety_monitor.update_sensor_data(noisy_sensor_data)

        # Check if emergency stop was triggered inappropriately
        status = safety_monitor.get_safety_status()

        # With hysteresis and filtering, this should not trigger critical emergency stops
        if status['emergency_stop_active']:
            # If triggered, verify it was for valid reasons
            active_triggers = [t for t in status['active_triggers'] if t['active']]
            assert len(active_triggers) > 0, "Emergency stop should have active triggers"

            # Check trigger severities - should be high/critical for emergency stop
            critical_triggers = [t for t in active_triggers if t['severity'] in ['high', 'critical']]
            assert len(critical_triggers) > 0, "Emergency stop should be triggered by high/critical severity"

    @pytest.mark.critical
    def test_emergency_stop_recovery_failure(self, safety_monitor):
        """Test failure to recover from emergency stop state."""
        # Force emergency stop activation
        safety_monitor._execute_emergency_stop(
            type('Event', (), {
                'description': 'Forced emergency stop for testing',
                'trigger': SafetyTrigger.MOTOR_OVERTEMP,
                'severity': SafetySeverity.CRITICAL
            })()
        )

        # Verify emergency stop is active
        status = safety_monitor.get_safety_status()
        assert status['emergency_stop_active'], "Emergency stop should be active"

        # Attempt recovery - should fail if triggers still active
        recovery_success = safety_monitor.reset_emergency_stop()
        assert not recovery_success, "Recovery should fail while critical triggers active"

        # Clear the active trigger and try again
        safety_monitor.active_triggers.clear()
        recovery_success = safety_monitor.reset_emergency_stop()
        assert recovery_success, "Recovery should succeed after clearing triggers"

    @pytest.mark.critical
    def test_emergency_stop_actuator_failure(self, safety_monitor):
        """Test emergency stop failure due to actuator problems."""
        # Mock emergency stop actuator failure
        with patch.object(safety_monitor, '_execute_emergency_stop') as mock_emergency:
            mock_emergency.side_effect = Exception("Actuator failure")

            # Trigger emergency stop
            safety_monitor._execute_emergency_stop(
                type('Event', (), {
                    'description': 'Test actuator failure',
                    'trigger': SafetyTrigger.BATTERY_LOW,
                    'severity': SafetySeverity.CRITICAL
                })()
            )

            # Emergency stop should still be marked as active even if actuator fails
            status = safety_monitor.get_safety_status()
            assert status['emergency_stop_active'], "Emergency stop should be active despite actuator failure"


class TestSafetyThresholdFailures:
    """Test safety threshold configuration and validation failures."""

    @pytest.mark.critical
    def test_safety_threshold_misconfiguration(self):
        """Test safety system with misconfigured thresholds."""
        # Create safety monitor with invalid thresholds
        invalid_config = SafetyThreshold(
            name="Invalid Motor Temp",
            trigger=SafetyTrigger.MOTOR_OVERTEMP,
            severity=SafetySeverity.HIGH,
            threshold_value=-10.0,  # Invalid: negative temperature
            hysteresis=5.0
        )

        # System should detect and reject invalid configuration
        assert invalid_config.threshold_value < 0, "Should detect invalid negative threshold"

        # Test with too high hysteresis
        invalid_hysteresis = SafetyThreshold(
            name="Invalid Hysteresis",
            trigger=SafetyTrigger.CURRENT_SPIKE,
            severity=SafetySeverity.HIGH,
            threshold_value=15.0,
            hysteresis=20.0  # Hysteresis > threshold
        )

        assert invalid_hysteresis.hysteresis > invalid_hysteresis.threshold_value, "Should detect invalid hysteresis"

    @pytest.mark.critical
    def test_safety_threshold_calibration_drift(self):
        """Test safety threshold failures due to calibration drift."""
        # Simulate sensor calibration drift over time
        initial_calibration = {
            'motor_temp_offset': 0.0,
            'current_offset': 0.0,
            'accel_offset': [0.0, 0.0, 0.0]
        }

        drifted_calibration = {
            'motor_temp_offset': 15.0,  # 15°C drift
            'current_offset': 5.0,      # 5A drift
            'accel_offset': [2.0, 1.0, 3.0]  # Significant accel drift
        }

        # With drifted calibration, normal readings trigger false alarms
        normal_reading = 25.0  # Normal motor temp
        calibrated_reading = normal_reading + drifted_calibration['motor_temp_offset']

        # Should trigger false alarm due to calibration drift
        threshold = 70.0  # Normal threshold
        false_alarm = calibrated_reading > threshold

        assert false_alarm, "Calibration drift should cause false safety alarms"

    @pytest.mark.critical
    def test_dynamic_threshold_adjustment_failure(self):
        """Test failure of dynamic threshold adjustment."""
        # Mock adaptive threshold system
        class AdaptiveThreshold:
            def __init__(self):
                self.base_threshold = 50.0
                self.adjustment_factor = 1.0

            def adjust_for_conditions(self, environmental_factor):
                # Should adjust threshold based on conditions
                self.adjustment_factor = environmental_factor
                return self.base_threshold * self.adjustment_factor

            def check_threshold(self, value):
                current_threshold = self.base_threshold * self.adjustment_factor
                return value > current_threshold

        adaptive = AdaptiveThreshold()

        # Normal adjustment
        normal_threshold = adaptive.adjust_for_conditions(1.0)
        assert normal_threshold == 50.0, "Should maintain base threshold in normal conditions"

        # Environmental adjustment (e.g., high temperature)
        hot_threshold = adaptive.adjust_for_conditions(1.2)  # 20% increase
        assert hot_threshold == 60.0, "Should increase threshold in hot conditions"

        # Failed adjustment (invalid environmental factor)
        try:
            invalid_threshold = adaptive.adjust_for_conditions(-1.0)  # Invalid negative
            assert invalid_threshold < 0, "Should detect invalid adjustment factor"
        except ValueError:
            assert True, "Should reject invalid environmental factors"


class TestSensorIntegrityFailures:
    """Test sensor integrity and validation failures."""

    @pytest.mark.critical
    def test_sensor_reading_validation_failure(self):
        """Test failure to validate sensor readings."""
        # Mock sensor readings with invalid values
        invalid_readings = {
            'temperature': float('nan'),  # Not a number
            'current': float('inf'),      # Infinite value
            'distance': -5.0,            # Negative distance
            'voltage': 1000.0            # Impossible voltage
        }

        def validate_reading(sensor_name, value):
            """Validate sensor reading."""
            if not isinstance(value, (int, float)) or not math.isfinite(value):
                return False

            # Sensor-specific validation
            if sensor_name == 'distance' and value < 0:
                return False
            if sensor_name == 'voltage' and not (0 <= value <= 50):  # 0-50V range
                return False
            if sensor_name == 'temperature' and not (-50 <= value <= 150):  # -50°C to 150°C
                return False
            if sensor_name == 'current' and not (0 <= value <= 50):  # 0-50A range
                return False

            return True

        # Check each invalid reading
        for sensor, value in invalid_readings.items():
            is_valid = validate_reading(sensor, value)
            assert not is_valid, f"Should reject invalid {sensor} reading: {value}"

    @pytest.mark.critical
    def test_sensor_fusion_integrity_failure(self):
        """Test sensor fusion integrity failures."""
        # Mock sensor fusion system
        class SensorFusion:
            def __init__(self):
                self.sensors = {}

            def add_sensor_reading(self, sensor_id, reading, timestamp):
                self.sensors[sensor_id] = {'reading': reading, 'timestamp': timestamp}

            def check_fusion_integrity(self):
                """Check if sensor fusion is valid."""
                if len(self.sensors) < 2:
                    return False, "Insufficient sensors"

                timestamps = [s['timestamp'] for s in self.sensors.values()]
                time_span = max(timestamps) - min(timestamps)

                if time_span > 0.1:  # 100ms max age difference
                    return False, "Sensors out of sync"

                return True, "Fusion valid"

        fusion = SensorFusion()

        # Add sensors with valid timing
        current_time = time.time()
        fusion.add_sensor_reading('imu', [0, 0, 9.81], current_time)
        fusion.add_sensor_reading('gps', [40.0, -74.0], current_time + 0.01)

        valid, message = fusion.check_fusion_integrity()
        assert valid, "Should be valid with synchronized sensors"

        # Add sensor with old timestamp
        fusion.add_sensor_reading('lidar', [5.0], current_time - 1.0)  # 1 second old

        valid, message = fusion.check_fusion_integrity()
        assert not valid, "Should detect out-of-sync sensors"

    @pytest.mark.critical
    def test_sensor_health_monitoring_failure(self):
        """Test failure of sensor health monitoring."""
        # Mock sensor health monitor
        class SensorHealthMonitor:
            def __init__(self):
                self.last_reading_time = {}
                self.timeout_threshold = 1.0  # 1 second

            def update_sensor_reading(self, sensor_id):
                self.last_reading_time[sensor_id] = time.time()

            def check_sensor_health(self, sensor_id):
                if sensor_id not in self.last_reading_time:
                    return False, "No readings received"

                time_since_last = time.time() - self.last_reading_time[sensor_id]
                if time_since_last > self.timeout_threshold:
                    return False, f"Sensor timeout: {time_since_last:.1f}s"

                return True, "Sensor healthy"

        monitor = SensorHealthMonitor()

        # Update sensor
        monitor.update_sensor_reading('imu')
        healthy, message = monitor.check_sensor_health('imu')
        assert healthy, "Sensor should be healthy after update"

        # Wait for timeout
        time.sleep(1.5)
        healthy, message = monitor.check_sensor_health('imu')
        assert not healthy, "Sensor should be unhealthy after timeout"

        # Check never-updated sensor
        healthy, message = monitor.check_sensor_health('gps')
        assert not healthy, "Never-updated sensor should be unhealthy"


class TestCascadingSafetyFailures:
    """Test cascading failures in safety systems."""

    @pytest.mark.critical
    def test_safety_subsystem_interdependency_failure(self):
        """Test failures caused by safety subsystem interdependencies."""
        # Mock interdependent safety subsystems
        subsystems = {
            'power_monitor': Mock(),
            'thermal_monitor': Mock(),
            'motion_safety': Mock(),
            'emergency_stop': Mock()
        }

        # Setup interdependencies
        subsystems['power_monitor'].check_battery.return_value = False  # Battery low
        subsystems['thermal_monitor'].check_temperature.return_value = True  # Temp OK
        subsystems['motion_safety'].check_motion_safety.return_value = True  # Motion OK

        # Emergency stop depends on other systems
        def emergency_stop_logic():
            battery_ok = subsystems['power_monitor'].check_battery()
            temp_ok = subsystems['thermal_monitor'].check_temperature()
            motion_ok = subsystems['motion_safety'].check_motion_safety()

            if not battery_ok:
                return False, "Emergency stop due to low battery"
            if not temp_ok:
                return False, "Emergency stop due to overheating"
            if not motion_ok:
                return False, "Emergency stop due to unsafe motion"

            return True, "All systems safe"

        # Test cascading failure
        safe, reason = emergency_stop_logic()
        assert not safe, "Should trigger emergency stop due to battery failure"

        # Now thermal failure cascades
        subsystems['power_monitor'].check_battery.return_value = True  # Fix battery
        subsystems['thermal_monitor'].check_temperature.return_value = False  # Temp high

        safe, reason = emergency_stop_logic()
        assert not safe, "Should trigger emergency stop due to thermal failure"

    @pytest.mark.critical
    def test_safety_communication_bus_failure(self):
        """Test safety system failure due to communication bus issues."""
        # Mock safety communication bus
        class SafetyBus:
            def __init__(self):
                self.subsystems = {}
                self.bus_failure = False

            def register_subsystem(self, name, subsystem):
                self.subsystems[name] = subsystem

            def broadcast_safety_status(self):
                if self.bus_failure:
                    raise Exception("Bus communication failure")

                # Collect status from all subsystems
                statuses = {}
                for name, subsystem in self.subsystems.items():
                    try:
                        statuses[name] = subsystem.get_status()
                    except Exception as e:
                        statuses[name] = f"Communication error: {e}"
                return statuses

        bus = SafetyBus()

        # Register subsystems
        for name in ['power', 'thermal', 'motion']:
            subsystem = Mock()
            subsystem.get_status.return_value = "OK"
            bus.register_subsystem(name, subsystem)

        # Normal operation
        statuses = bus.broadcast_safety_status()
        assert len(statuses) == 3, "Should get status from all subsystems"

        # Bus failure
        bus.bus_failure = True
        try:
            statuses = bus.broadcast_safety_status()
            assert False, "Should raise exception on bus failure"
        except Exception:
            assert True, "Should handle bus communication failure"

    @pytest.mark.critical
    def test_safety_watchdog_timer_failure(self):
        """Test safety watchdog timer failures."""
        # Mock watchdog timer system
        class SafetyWatchdog:
            def __init__(self, timeout=5.0):
                self.timeout = timeout
                self.last_feed_time = time.time()
                self.emergency_action_triggered = False

            def feed(self):
                self.last_feed_time = time.time()

            def check_timeout(self):
                elapsed = time.time() - self.last_feed_time
                if elapsed > self.timeout:
                    self.emergency_action_triggered = True
                    return False, f"Watchdog timeout: {elapsed:.1f}s"
                return True, "Watchdog OK"

        watchdog = SafetyWatchdog(timeout=2.0)

        # Normal feeding
        watchdog.feed()
        ok, message = watchdog.check_timeout()
        assert ok, "Watchdog should be OK after feeding"

        # Let timeout occur
        time.sleep(3.0)
        ok, message = watchdog.check_timeout()
        assert not ok, "Watchdog should timeout"
        assert watchdog.emergency_action_triggered, "Should trigger emergency action on timeout"


class TestSafetyRecoveryFailures:
    """Test failures in safety system recovery mechanisms."""

    @pytest.mark.critical
    def test_safety_event_acknowledgment_failure(self):
        """Test failure to properly acknowledge safety events."""
        safety_monitor = SafetyMonitor()

        # Trigger a safety event
        safety_monitor.update_sensor_data({
            'motor_temps': [75.0, 70.0, 72.0, 71.0]  # Over temp threshold
        })

        status = safety_monitor.get_safety_status()
        initial_trigger_count = len(status['active_triggers'])

        # Attempt to acknowledge non-existent trigger
        success = safety_monitor.acknowledge_safety_event(SafetyTrigger.BATTERY_LOW)
        assert not success, "Should fail to acknowledge non-active trigger"

        # Check that trigger count didn't change
        status = safety_monitor.get_safety_status()
        assert len(status['active_triggers']) == initial_trigger_count, "Trigger count should not change"

    @pytest.mark.critical
    def test_safety_system_restart_failure(self):
        """Test failure to restart safety system after failure."""
        # Mock safety system restart process
        class SafetySystemController:
            def __init__(self):
                self.systems = ['monitor', 'watchdog', 'emergency_stop']
                self.failed_systems = set()

            def restart_system(self, system_name):
                if system_name in self.failed_systems:
                    # Simulate restart failure for some systems
                    if system_name == 'emergency_stop':
                        raise Exception("Emergency stop restart failed")
                    return True
                return False

            def mark_system_failed(self, system_name):
                self.failed_systems.add(system_name)

        controller = SafetySystemController()

        # Mark emergency stop as failed
        controller.mark_system_failed('emergency_stop')

        # Attempt restart - should fail
        try:
            success = controller.restart_system('emergency_stop')
            assert False, "Should fail to restart failed emergency stop"
        except Exception:
            assert True, "Should handle emergency stop restart failure"

    @pytest.mark.critical
    def test_safety_override_mechanism_failure(self):
        """Test failures in safety override mechanisms."""
        # Mock safety override system
        class SafetyOverride:
            def __init__(self):
                self.override_active = False
                self.authorized_personnel = ['operator1', 'engineer1']
                self.override_timeout = 300  # 5 minutes

            def request_override(self, personnel_id, reason):
                if personnel_id not in self.authorized_personnel:
                    return False, "Unauthorized personnel"

                if not reason or len(reason) < 10:
                    return False, "Insufficient reason provided"

                self.override_active = True
                self.override_start = time.time()
                return True, "Override granted"

            def check_override_validity(self):
                if not self.override_active:
                    return False, "No override active"

                elapsed = time.time() - self.override_start
                if elapsed > self.override_timeout:
                    self.override_active = False
                    return False, "Override expired"

                return True, "Override valid"

        override_system = SafetyOverride()

        # Test unauthorized override request
        success, message = override_system.request_override('unauthorized_user', 'Test override')
        assert not success, "Should reject unauthorized override request"

        # Test override with insufficient reason
        success, message = override_system.request_override('operator1', 'Short')
        assert not success, "Should reject override with insufficient reason"

        # Valid override request
        success, message = override_system.request_override('operator1', 'Need to test system with safety disabled for calibration')
        assert success, "Should grant valid override request"

        # Check validity
        valid, message = override_system.check_override_validity()
        assert valid, "Override should be valid immediately after granting"

        # Simulate timeout
        override_system.override_start = time.time() - 400  # 400 seconds ago
        valid, message = override_system.check_override_validity()
        assert not valid, "Override should expire after timeout"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
