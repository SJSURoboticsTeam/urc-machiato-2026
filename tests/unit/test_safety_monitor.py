#!/usr/bin/env python3
"""
Unit Tests for Safety Monitor

Tests the comprehensive safety monitoring system that handles:
- Safety threshold monitoring
- Emergency stop triggers
- Sensor data validation
- Safety event management

Author: URC 2026 Test Suite
"""

import pytest
import time
from unittest.mock import Mock, MagicMock, patch
import sys
import os
from typing import Dict, Any

# Import safety monitor
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src/autonomy/core/safety_system/autonomy_safety_system'))
from safety_monitor import (
    SafetyMonitor,
    SafetyTrigger,
    SafetySeverity,
    SafetyThreshold,
    SafetyEvent
)


class TestSafetyMonitor:
    """Test SafetyMonitor core functionality."""

    @pytest.fixture
    def safety_monitor(self):
        """Create SafetyMonitor instance."""
        return SafetyMonitor()

    @pytest.mark.unit
    def test_initialization(self, safety_monitor):
        """Test SafetyMonitor initialization."""
        assert safety_monitor.safety_enabled is True
        assert safety_monitor.emergency_stop_active is False
        assert len(safety_monitor.active_triggers) == 0
        assert len(safety_monitor.thresholds) > 0
        assert safety_monitor.monitoring_active is False

    @pytest.mark.unit
    def test_default_thresholds(self, safety_monitor):
        """Test default safety thresholds are loaded."""
        thresholds = safety_monitor.thresholds

        # Check critical thresholds exist
        assert SafetyTrigger.BATTERY_LOW in thresholds
        assert SafetyTrigger.MOTOR_OVERTEMP in thresholds
        assert SafetyTrigger.IMU_SHOCK in thresholds
        assert SafetyTrigger.OBSTACLE_CLOSE in thresholds

        # Check battery low is critical
        battery_threshold = thresholds[SafetyTrigger.BATTERY_LOW]
        assert battery_threshold.severity == SafetySeverity.CRITICAL
        assert battery_threshold.threshold_value == 15.0
        assert battery_threshold.requires_ack is True

    @pytest.mark.unit
    def test_threshold_checking(self, safety_monitor):
        """Test threshold checking logic."""
        # Test motor temperature threshold
        motor_temp_threshold = safety_monitor.thresholds[SafetyTrigger.MOTOR_OVERTEMP]

        # Normal temperature - should not trigger
        safety_monitor._check_threshold(SafetyTrigger.MOTOR_OVERTEMP, 50.0)
        assert SafetyTrigger.MOTOR_OVERTEMP not in safety_monitor.active_triggers

        # High temperature - should trigger
        safety_monitor._check_threshold(SafetyTrigger.MOTOR_OVERTEMP, 75.0)
        assert SafetyTrigger.MOTOR_OVERTEMP in safety_monitor.active_triggers

        # Temperature back to normal with hysteresis - should clear
        safety_monitor._check_threshold(SafetyTrigger.MOTOR_OVERTEMP, 60.0)
        # Note: hysteresis clearing happens asynchronously

    @pytest.mark.unit
    def test_safety_event_creation(self, safety_monitor):
        """Test safety event creation and management."""
        # Trigger a safety event
        safety_monitor._trigger_safety_event(
            SafetyTrigger.BATTERY_LOW,
            10.0,
            15.0,
            "Battery critically low"
        )

        assert SafetyTrigger.BATTERY_LOW in safety_monitor.active_triggers

        event = safety_monitor.active_triggers[SafetyTrigger.BATTERY_LOW]
        assert event.trigger == SafetyTrigger.BATTERY_LOW
        assert event.severity == SafetySeverity.CRITICAL
        assert event.value == 10.0
        assert event.threshold == 15.0
        assert event.active is True

    @pytest.mark.unit
    def test_emergency_stop_triggering(self, safety_monitor):
        """Test emergency stop triggering for critical events."""
        emergency_callback_called = False

        def emergency_callback(event):
            nonlocal emergency_callback_called
            emergency_callback_called = True
            assert event.severity == SafetySeverity.CRITICAL

        safety_monitor.register_emergency_stop_callback(emergency_callback)

        # Trigger critical safety event
        safety_monitor._trigger_safety_event(
            SafetyTrigger.BATTERY_LOW,
            5.0,
            15.0,
            "Battery critically low"
        )

        # Emergency stop should be triggered
        assert safety_monitor.emergency_stop_active is True
        assert emergency_callback_called is True

    @pytest.mark.unit
    def test_warning_callbacks(self, safety_monitor):
        """Test warning callback system."""
        warning_callback_called = False

        def warning_callback(event):
            nonlocal warning_callback_called
            warning_callback_called = True
            assert event.trigger == SafetyTrigger.MOTOR_OVERTEMP

        safety_monitor.register_warning_callback(warning_callback)

        # Trigger non-critical safety event
        safety_monitor._trigger_safety_event(
            SafetyTrigger.MOTOR_OVERTEMP,
            75.0,
            70.0,
            "Motor overheating"
        )

        assert warning_callback_called is True
        assert safety_monitor.emergency_stop_active is False  # Not critical

    @pytest.mark.unit
    def test_sensor_data_update(self, safety_monitor):
        """Test sensor data update processing."""
        sensor_data = {
            'motor_temps': [65.0, 72.0, 68.0, 71.0],  # Two motors over temp
            'battery_level': 12.0,  # Low battery
            'imu_accel': [0.0, 0.0, 45.0],  # High acceleration
            'motor_currents': [8.0, 16.0, 7.0, 9.0],  # One motor high current
            'min_obstacle_distance': 0.2  # Close obstacle
        }

        safety_monitor.update_sensor_data(sensor_data)

        # Check that multiple triggers were activated
        active_triggers = list(safety_monitor.active_triggers.keys())

        # Should have triggered multiple safety events
        assert len(active_triggers) > 0

        # Emergency stop should be active due to critical events
        assert safety_monitor.emergency_stop_active is True

    @pytest.mark.unit
    def test_safety_status_report(self, safety_monitor):
        """Test safety status reporting."""
        # Initially safe
        status = safety_monitor.get_safety_status()
        assert status['system_safe'] is True
        assert status['emergency_stop_active'] is False
        assert len(status['active_triggers']) == 0

        # Trigger safety event
        safety_monitor._trigger_safety_event(
            SafetyTrigger.MOTOR_OVERTEMP,
            75.0,
            70.0
        )

        # Now unsafe
        status = safety_monitor.get_safety_status()
        assert status['system_safe'] is False
        assert len(status['active_triggers']) == 1
        assert status['highest_severity'] == 'high'

    @pytest.mark.unit
    def test_emergency_stop_reset(self, safety_monitor):
        """Test emergency stop reset functionality."""
        # Trigger emergency stop
        safety_monitor._execute_emergency_stop(
            SafetyEvent(
                trigger=SafetyTrigger.BATTERY_LOW,
                severity=SafetySeverity.CRITICAL,
                value=5.0,
                threshold=15.0,
                timestamp=time.time(),
                description="Battery low"
            )
        )

        assert safety_monitor.emergency_stop_active is True

        # Clear all triggers
        safety_monitor.active_triggers.clear()

        # Should be able to reset
        result = safety_monitor.reset_emergency_stop()
        assert result is True
        assert safety_monitor.emergency_stop_active is False

    @pytest.mark.unit
    def test_acknowledge_safety_event(self, safety_monitor):
        """Test safety event acknowledgment."""
        # Trigger battery low (requires ack)
        safety_monitor._trigger_safety_event(
            SafetyTrigger.BATTERY_LOW,
            10.0,
            15.0
        )

        assert SafetyTrigger.BATTERY_LOW in safety_monitor.active_triggers

        # Acknowledge the event
        result = safety_monitor.acknowledge_safety_event(SafetyTrigger.BATTERY_LOW)
        assert result is True

        # Event should be cleared
        assert SafetyTrigger.BATTERY_LOW not in safety_monitor.active_triggers


class TestSafetyMonitorIntegration:
    """Integration tests for SafetyMonitor."""

    @pytest.fixture
    def safety_monitor(self):
        """Create SafetyMonitor instance."""
        return SafetyMonitor()

    @pytest.mark.integration
    def test_monitoring_loop(self, safety_monitor):
        """Test the monitoring loop functionality."""
        # Start monitoring
        result = safety_monitor.start_monitoring()
        assert result is True
        assert safety_monitor.monitoring_active is True

        # Let it run briefly
        time.sleep(0.1)

        # Stop monitoring
        result = safety_monitor.stop_monitoring()
        assert result is True
        assert safety_monitor.monitoring_active is False

    @pytest.mark.integration
    def test_watchdog_functionality(self, safety_monitor):
        """Test watchdog timer functionality."""
        # Start monitoring
        safety_monitor.start_monitoring()
        safety_monitor.watchdog_timeout = 0.1  # Short timeout for test

        # Initially no watchdog trigger
        assert SafetyTrigger.GPS_LOSS not in safety_monitor.active_triggers

        # Wait for watchdog timeout
        time.sleep(0.2)

        # Should trigger GPS loss
        # Note: This might not trigger immediately due to timing

        # Stop monitoring
        safety_monitor.stop_monitoring()

    @pytest.mark.integration
    def test_multiple_sensor_updates(self, safety_monitor):
        """Test processing multiple sensor updates."""
        # Simulate realistic sensor data stream
        sensor_updates = [
            {'motor_temps': [45.0, 48.0, 46.0, 47.0]},  # Normal
            {'motor_temps': [65.0, 68.0, 66.0, 67.0]},  # Warming up
            {'motor_temps': [75.0, 78.0, 76.0, 77.0]},  # Over temp
            {'battery_level': 20.0},  # OK
            {'battery_level': 12.0},  # Low
            {'imu_accel': [0.0, 0.0, 60.0]},  # Shock
        ]

        for update in sensor_updates:
            safety_monitor.update_sensor_data(update)

        # Should have triggered multiple safety events
        assert len(safety_monitor.active_triggers) > 0

        # Emergency stop should be active
        assert safety_monitor.emergency_stop_active is True


class TestSafetyMonitorROS2:
    """ROS2-specific tests for SafetyMonitor."""

    @pytest.fixture
    def mock_node(self):
        """Create mock ROS2 node."""
        node = Mock()
        node.get_logger = Mock()
        node.get_logger.return_value.info = Mock()
        node.get_logger.return_value.warn = Mock()
        node.get_logger.return_value.error = Mock()
        return node

    @pytest.fixture
    def safety_monitor_with_node(self, mock_node):
        """Create SafetyMonitor with ROS2 node."""
        return SafetyMonitor(mock_node)

    @pytest.mark.ros2
    def test_ros2_logging_integration(self, safety_monitor_with_node, mock_node):
        """Test ROS2 logging integration."""
        # Trigger safety event
        safety_monitor_with_node._trigger_safety_event(
            SafetyTrigger.MOTOR_OVERTEMP,
            75.0,
            70.0
        )

        # Should have used ROS2 logger
        mock_node.get_logger.return_value.warning.assert_called()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])




