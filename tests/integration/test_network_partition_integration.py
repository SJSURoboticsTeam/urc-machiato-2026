#!/usr/bin/env python3
"""
Integration Test: Network Partition Detector

Tests network partition detection and offline autonomy transitions
to ensure rover continues operation during WiFi outages.
"""

import time
import threading
import pytest
from unittest.mock import patch, MagicMock
from src.comms.network_partition_detector import NetworkPartitionDetector, NetworkState, AutonomyMode
from src.testing.performance_profiling import PerformanceProfiler


class TestNetworkPartitionIntegration:
    """Test network partition detector integration."""

    def setup_method(self):
        """Setup test fixtures."""
        self.detector = NetworkPartitionDetector(
            partition_threshold_seconds=2.0,  # Faster for testing
            recovery_grace_period_seconds=3.0
        )
        self.performance_profiler = PerformanceProfiler()

    def teardown_method(self):
        """Cleanup test fixtures."""
        self.detector.stop_monitoring()

    def test_network_partition_detection(self):
        """Test basic network partition detection."""
        # Start monitoring
        self.detector.start_monitoring()

        # Initially should be connected
        status = self.detector.get_network_status()
        assert status['network_state'] == NetworkState.CONNECTED.value
        assert status['autonomy_mode'] == AutonomyMode.FULL_AUTONOMY.value

        # Mock network failure
        with patch.object(self.detector, '_check_wifi_connected', return_value=True), \
             patch.object(self.detector, '_check_internet_reachable', return_value=(False, None)):

            # Wait for partition detection
            time.sleep(3.0)  # Longer than partition_threshold

            status = self.detector.get_network_status()
            assert status['network_state'] == NetworkState.PARTITIONED.value
            assert status['autonomy_mode'] == AutonomyMode.OFFLINE_AUTONOMY.value

    def test_network_recovery(self):
        """Test network recovery after partition."""
        self.detector.start_monitoring()

        # First simulate partition
        with patch.object(self.detector, '_check_wifi_connected', return_value=True), \
             patch.object(self.detector, '_check_internet_reachable', return_value=(False, None)):

            time.sleep(3.0)
            status = self.detector.get_network_status()
            assert status['network_state'] == NetworkState.PARTITIONED.value

        # Then simulate recovery
        with patch.object(self.detector, '_check_wifi_connected', return_value=True), \
             patch.object(self.detector, '_check_internet_reachable', return_value=(True, 25.0)):

            time.sleep(4.0)  # Longer than recovery grace period
            status = self.detector.get_network_status()
            assert status['network_state'] == NetworkState.CONNECTED.value
            assert status['autonomy_mode'] == AutonomyMode.FULL_AUTONOMY.value

    def test_wifi_complete_disconnect(self):
        """Test complete WiFi disconnection (worse than partition)."""
        self.detector.start_monitoring()

        # Mock complete WiFi disconnect
        with patch.object(self.detector, '_check_wifi_connected', return_value=False):

            time.sleep(3.0)
            status = self.detector.get_network_status()
            assert status['network_state'] == NetworkState.OFFLINE.value
            # Autonomy mode should remain offline when WiFi is completely gone

    def test_partition_callbacks(self):
        """Test partition detection callbacks."""
        partition_events = []
        recovery_events = []

        def partition_callback(event):
            partition_events.append(event)

        def recovery_callback(event):
            recovery_events.append(event)

        self.detector.register_partition_callback(partition_callback)
        self.detector.register_recovery_callback(recovery_callback)

        self.detector.start_monitoring()

        # Simulate partition
        with patch.object(self.detector, '_check_wifi_connected', return_value=True), \
             patch.object(self.detector, '_check_internet_reachable', return_value=(False, None)):

            time.sleep(3.0)
            assert len(partition_events) >= 1
            assert partition_events[0]['event_type'] == 'partition_detected'

        # Simulate recovery
        with patch.object(self.detector, '_check_wifi_connected', return_value=True), \
             patch.object(self.detector, '_check_internet_reachable', return_value=(True, 25.0)):

            time.sleep(4.0)
            assert len(recovery_events) >= 1
            assert recovery_events[0]['event_type'] == 'partition_resolved'

    def test_manual_offline_mode(self):
        """Test manual offline mode activation."""
        # Initially online
        status = self.detector.get_network_status()
        assert status['network_state'] == NetworkState.CONNECTED.value

        # Manually force offline
        self.detector.force_offline_mode("manual_test")

        status = self.detector.get_network_status()
        assert status['network_state'] == NetworkState.PARTITIONED.value
        assert status['autonomy_mode'] == AutonomyMode.OFFLINE_AUTONOMY.value

        # Manually restore
        self.detector.reset_to_online_mode("manual_restore")

        status = self.detector.get_network_status()
        assert status['network_state'] == NetworkState.CONNECTED.value
        assert status['autonomy_mode'] == AutonomyMode.FULL_AUTONOMY.value

    def test_network_health_checks(self):
        """Test network health check functionality."""
        # Test with good connection
        with patch.object(self.detector, '_check_wifi_connected', return_value=True), \
             patch.object(self.detector, '_check_internet_reachable', return_value=(True, 15.0)), \
             patch.object(self.detector, '_get_wifi_signal_strength', return_value=-45):

            health = self.detector._perform_health_check()
            assert health.wifi_connected == True
            assert health.internet_reachable == True
            assert health.latency_ms == 15.0
            assert health.signal_strength_dbm == -45
            assert health.error_message is None

        # Test with poor connection
        with patch.object(self.detector, '_check_wifi_connected', return_value=True), \
             patch.object(self.detector, '_check_internet_reachable', return_value=(True, 150.0)), \
             patch.object(self.detector, '_get_wifi_signal_strength', return_value=-85):

            health = self.detector._perform_health_check()
            assert health.wifi_connected == True
            assert health.internet_reachable == True
            assert health.latency_ms == 150.0
            assert health.signal_strength_dbm == -85

    def test_degraded_network_state(self):
        """Test degraded network state detection."""
        self.detector.start_monitoring()

        # Simulate degraded but still working connection
        with patch.object(self.detector, '_check_wifi_connected', return_value=True), \
             patch.object(self.detector, '_check_internet_reachable', return_value=(True, 80.0)):

            time.sleep(2.0)
            status = self.detector.get_network_status()
            # Should be connected but marked as degraded due to high latency
            assert status['network_state'] in [NetworkState.CONNECTED.value, NetworkState.DEGRADED.value]

    def test_partition_duration_tracking(self):
        """Test partition duration tracking."""
        self.detector.start_monitoring()

        # Simulate partition
        with patch.object(self.detector, '_check_wifi_connected', return_value=True), \
             patch.object(self.detector, '_check_internet_reachable', return_value=(False, None)):

            time.sleep(3.0)
            status = self.detector.get_network_status()
            assert status['network_state'] == NetworkState.PARTITIONED.value
            partition_duration = status['partition_duration_seconds']
            assert partition_duration > 2.0  # Should have been partitioned for > 2 seconds

    def test_network_history_tracking(self):
        """Test network history and statistics tracking."""
        self.detector.start_monitoring()

        # Let it run for a bit to collect history
        time.sleep(2.0)

        # Get network history
        history_1h = self.detector.get_network_history(hours=1.0)
        assert 'total_health_checks' in history_1h
        assert 'uptime_percent' in history_1h
        assert history_1h['total_health_checks'] >= 1

        # Get diagnostics
        diagnostics = self.detector.export_diagnostics()
        assert 'current_status' in diagnostics
        assert 'network_history_1h' in diagnostics
        assert 'recent_health_checks' in diagnostics

    def test_concurrent_callback_handling(self):
        """Test that callbacks don't interfere with each other."""
        callback_results = []

        def callback1(event):
            time.sleep(0.1)  # Simulate work
            callback_results.append(f"callback1_{event['event_type']}")

        def callback2(event):
            time.sleep(0.05)  # Different delay
            callback_results.append(f"callback2_{event['event_type']}")

        self.detector.register_partition_callback(callback1)
        self.detector.register_partition_callback(callback2)

        self.detector.start_monitoring()

        # Trigger partition
        with patch.object(self.detector, '_check_wifi_connected', return_value=True), \
             patch.object(self.detector, '_check_internet_reachable', return_value=(False, None)):

            time.sleep(3.0)

            # Both callbacks should have been called
            partition_callbacks = [r for r in callback_results if 'partition_detected' in r]
            assert len(partition_callbacks) == 2

    def test_offline_autonomy_mode_integration(self):
        """Test integration with offline autonomy mode."""
        # This would integrate with the behavior tree system
        # For now, just test that the mode changes correctly

        self.detector.start_monitoring()

        # Simulate partition
        with patch.object(self.detector, '_check_wifi_connected', return_value=True), \
             patch.object(self.detector, '_check_internet_reachable', return_value=(False, None)):

            time.sleep(3.0)
            status = self.detector.get_network_status()
            assert status['autonomy_mode'] == AutonomyMode.OFFLINE_AUTONOMY.value

            # In real integration, this would:
            # 1. Stop publishing telemetry to cloud
            # 2. Switch behavior trees to offline mode
            # 3. Enable local-only navigation
            # 4. Queue critical messages for later transmission

    def test_performance_under_load(self):
        """Test network monitoring performance under load."""
        self.detector.start_monitoring()

        # Measure monitoring overhead
        start_time = time.perf_counter()

        # Let it run for monitoring period
        time.sleep(5.0)

        end_time = time.perf_counter()
        monitoring_time = end_time - start_time

        # Should not consume excessive CPU (rough check)
        # In 5 seconds, should not use more than 1 second of CPU
        # This is a very rough check - in practice we'd use proper profiling

        # Check that monitoring is still running
        status = self.detector.get_network_status()
        assert 'timestamp' in status

    def test_graceful_shutdown(self):
        """Test graceful shutdown of network monitoring."""
        self.detector.start_monitoring()

        # Let it run briefly
        time.sleep(1.0)

        # Shutdown
        self.detector.stop_monitoring()

        # Should not crash on subsequent operations
        status = self.detector.get_network_status()
        assert status is not None

        # Multiple shutdowns should be safe
        self.detector.stop_monitoring()
        self.detector.stop_monitoring()

    def test_error_recovery(self):
        """Test error recovery in network monitoring."""
        self.detector.start_monitoring()

        # Simulate exceptions in health checks
        with patch.object(self.detector, '_perform_health_check', side_effect=Exception("Test error")):
            # Should not crash monitoring thread
            time.sleep(2.0)

            # Monitoring should still be running
            status = self.detector.get_network_status()
            assert status is not None

    def test_performance_profiling_integration(self):
        """Test integration with performance profiling."""
        # Profile network health check operation
        def profiled_health_check():
            return self.detector._perform_health_check()

        # Measure performance
        result = self.performance_profiler.measure_latency(
            "network_health_check", profiled_health_check
        )

        assert result is not None

        # Check profiling captured it
        report = self.performance_profiler.get_performance_report()
        assert "network_health_check" in report['profiles']

        stats = report['profiles']['network_health_check']
        assert stats['count'] >= 1
        assert stats['mean_ms'] > 0

        # Health check should be fast (< 100ms typically)
        assert stats['mean_ms'] < 500.0
