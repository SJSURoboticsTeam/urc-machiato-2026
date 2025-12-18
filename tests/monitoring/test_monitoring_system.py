#!/usr/bin/env python3
"""
Test suite for the monitoring system including QoS profiling, safety monitoring,
and adaptive telemetry.
"""

import pytest
import time
import threading
from unittest.mock import Mock, patch, MagicMock
import rclpy
from rclpy.node import Node

from autonomy_state_management.qos_profiler import QoSProfiler
from autonomy_state_management.safety_monitor import SafetyMonitor
from bridges.competition_bridge import CompetitionBridge
from autonomy_state_management.urc_band_manager import URCBandManager


class TestMonitoringSystem:
    """Test the complete monitoring system integration."""

    @pytest.fixture
    def ros_context(self):
        """Setup ROS2 context for testing."""
        rclpy.init()
        yield
        rclpy.shutdown()

    @pytest.fixture
    def qos_profiler(self, ros_context):
        """Create QoS profiler for testing."""
        profiler = QoSProfiler()
        profiler.get_logger = Mock()  # Mock logger for testing
        return profiler

    @pytest.fixture
    def safety_monitor(self, ros_context):
        """Create safety monitor for testing."""
        monitor = SafetyMonitor()
        monitor.get_logger = Mock()
        return monitor

    @pytest.fixture
    def competition_bridge(self, ros_context):
        """Create competition bridge for testing."""
        bridge = CompetitionBridge()
        bridge.get_logger = Mock()
        return bridge

    @pytest.fixture
    def band_manager(self, ros_context):
        """Create URC band manager for testing."""
        manager = URCBandManager()
        manager.get_logger = Mock()
        return manager

    def test_qos_profiler_initialization(self, qos_profiler):
        """Test QoS profiler initializes correctly."""
        assert qos_profiler.profiling_rate == 1.0
        assert '900mhz' in qos_profiler.urc_band_config
        assert '2_4ghz' in qos_profiler.urc_band_config
        assert len(qos_profiler.topic_metrics) == 0  # No topics subscribed yet

    def test_safety_monitor_initialization(self, safety_monitor):
        """Test safety monitor initializes correctly."""
        assert safety_monitor.monitoring_rate == 10.0
        assert 'battery_safety' in safety_monitor.safety_properties
        assert 'geofence_compliance' in safety_monitor.safety_properties
        assert 'control_loop_timing' in safety_monitor.safety_properties

    def test_safety_property_evaluation(self, safety_monitor):
        """Test safety property evaluation."""
        # Test battery safety
        system_state = {
            'battery_level': 25.0,  # Above threshold
            'mission_active': True,
            'emergency_stop': False
        }

        satisfied, details = safety_monitor.safety_properties['battery_safety'].evaluate(system_state)
        assert satisfied == True

        # Test battery safety violation
        system_state['battery_level'] = 15.0  # Below threshold for active mission
        satisfied, details = safety_monitor.safety_properties['battery_safety'].evaluate(system_state)
        assert satisfied == False

    def test_geofence_safety(self, safety_monitor):
        """Test geofence safety property."""
        system_state = {
            'mission_active': True,
            'boundary_violation': False
        }

        satisfied, details = safety_monitor.safety_properties['geofence_compliance'].evaluate(system_state)
        assert satisfied == True

        # Test violation
        system_state['boundary_violation'] = True
        satisfied, details = safety_monitor.safety_properties['geofence_compliance'].evaluate(system_state)
        assert satisfied == False

    def test_control_loop_timing(self, safety_monitor):
        """Test control loop timing safety."""
        system_state = {
            'control_loop_active': True,
            'last_control_update': time.time()  # Current time
        }

        satisfied, details = safety_monitor.safety_properties['control_loop_timing'].evaluate(system_state)
        assert satisfied == True

        # Test timing violation
        system_state['last_control_update'] = time.time() - 0.5  # 500ms ago
        satisfied, details = safety_monitor.safety_properties['control_loop_timing'].evaluate(system_state)
        assert satisfied == False

    def test_adaptive_telemetry_initialization(self, competition_bridge):
        """Test adaptive telemetry initializes correctly."""
        assert competition_bridge.adaptive_enabled == True
        assert competition_bridge.adaptive_data['current_rate'] == 5.0
        assert '900mhz' in competition_bridge.urc_band_config
        assert competition_bridge.urc_band_config['900mhz']['max_bandwidth_mhz'] == 8.0

    def test_adaptive_telemetry_rate_adjustment(self, competition_bridge):
        """Test adaptive telemetry rate adjustment."""
        # Test with good conditions (should maintain or increase rate)
        competition_bridge.update_adaptive_telemetry(
            bandwidth_mbps=5.0,   # Well under 8MHz limit
            latency_ms=50.0,      # Good latency
            packet_loss=0.01      # Low packet loss
        )

        # Rate should not decrease with good conditions
        assert competition_bridge.adaptive_data['target_rate'] >= 5.0

        # Test with poor conditions (should decrease rate)
        competition_bridge.update_adaptive_telemetry(
            bandwidth_mbps=7.5,   # Near 8MHz limit (90%+ utilization)
            latency_ms=150.0,     # High latency
            packet_loss=0.05     # Moderate packet loss
        )

        # Rate should decrease with poor conditions
        assert competition_bridge.adaptive_data['target_rate'] <= 5.0

    def test_urc_band_manager_initialization(self, band_manager):
        """Test URC band manager initializes correctly."""
        assert band_manager.current_band == '900mhz'  # Default from params
        assert '900mhz' in band_manager.band_config
        assert '2_4ghz' in band_manager.band_config
        assert band_manager.band_config['900mhz']['max_bandwidth_mhz'] == 8.0

    def test_urc_band_switching(self, band_manager):
        """Test URC band switching functionality."""
        # Test 900MHz subband switching
        success = band_manager.set_band('900mhz', 'mid')
        assert success == True
        assert band_manager.current_band == '900mhz'
        assert band_manager.band_config['900mhz']['current_subband'] == 'mid'

        # Test switching to 2.4GHz
        success = band_manager.set_band('2_4ghz')
        assert success == True
        assert band_manager.current_band == '2_4ghz'

        # Test invalid band
        success = band_manager.set_band('invalid_band')
        assert success == False

    def test_urc_band_interference_simulation(self, band_manager):
        """Test interference measurement simulation."""
        # Test 900MHz interference measurement
        interference = band_manager.measure_interference('900mhz', 'low')
        assert 0.0 <= interference <= 1.0

        # Test 2.4GHz interference (should be higher on average)
        interference_2_4 = band_manager.measure_interference('2_4ghz')
        assert 0.0 <= interference_2_4 <= 1.0

    def test_network_quality_calculation(self, qos_profiler):
        """Test network quality score calculation."""
        # Test good network conditions
        quality = qos_profiler._calculate_network_quality(
            bandwidth=4.0, latency=50.0, packet_loss=0.01, signal_strength=0.9
        )
        assert quality > 0.8  # Should be high quality

        # Test poor network conditions
        quality = qos_profiler._calculate_network_quality(
            bandwidth=7.5, latency=200.0, packet_loss=0.1, signal_strength=0.3
        )
        assert quality < 0.5  # Should be low quality

    def test_safety_violation_handling(self, safety_monitor):
        """Test safety violation detection and handling."""
        # Simulate a battery safety violation
        system_state = {
            'battery_level': 5.0,  # Critically low
            'mission_active': True,
            'emergency_stop': False
        }

        # Evaluate the safety property
        satisfied, details = safety_monitor.safety_properties['battery_safety'].evaluate(system_state)
        assert satisfied == False

        # Check that violation was recorded
        assert safety_monitor.safety_properties['battery_safety'].violation_count > 0

    @patch('psutil.net_io_counters')
    def test_bandwidth_measurement(self, mock_net_io, qos_profiler):
        """Test bandwidth measurement functionality."""
        # Mock network I/O counters
        mock_net_io.return_value.bytes_sent = 1000000
        mock_net_io.return_value.bytes_recv = 2000000

        # First call to establish baseline
        qos_profiler._bandwidth_callback()

        # Simulate time passage and increased counters
        time.sleep(0.1)
        mock_net_io.return_value.bytes_sent = 2000000
        mock_net_io.return_value.bytes_recv = 4000000

        # Second call to measure bandwidth
        qos_profiler._bandwidth_callback()

        # Check that bandwidth measurements were recorded
        assert len(qos_profiler.network_metrics['bandwidth_up']) > 0
        assert len(qos_profiler.network_metrics['bandwidth_down']) > 0

    def test_monitoring_system_integration(self, qos_profiler, safety_monitor, competition_bridge):
        """Test integrated monitoring system functionality."""
        # Test that all components can coexist and share state

        # QoS profiler should be able to measure topics
        assert hasattr(qos_profiler, 'topic_metrics')

        # Safety monitor should have safety properties
        assert len(safety_monitor.safety_properties) > 0

        # Competition bridge should have adaptive telemetry
        assert hasattr(competition_bridge, 'adaptive_data')

        # All should be able to operate simultaneously (basic integration test)
        assert qos_profiler.profiling_rate > 0
        assert safety_monitor.monitoring_rate > 0
        assert competition_bridge.adaptive_enabled == True


if __name__ == '__main__':
    pytest.main([__file__])


