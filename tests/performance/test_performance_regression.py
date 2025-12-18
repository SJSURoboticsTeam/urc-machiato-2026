#!/usr/bin/env python3
"""
Performance Regression Testing Framework

Tracks performance baselines and detects regressions in:
- State synchronization operations
- DDS domain management
- Dynamic configuration updates
- WebSocket redundancy operations
- Recovery coordination

Author: URC 2026 Autonomy Team
"""

import os
import sys
import time
import unittest
from statistics import mean, median, stdev

import psutil

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))


class PerformanceRegressionTest(unittest.TestCase):
    """Test performance baselines and detect regressions."""

    # Performance baselines (adjust based on target hardware)
    BASELINE_METRICS = {
        "state_sync_operation_time_ms": 1.0,  # Max time per state sync operation
        "state_sync_throughput_ops_sec": 1000,  # Min operations per second
        "config_update_time_ms": 5.0,  # Max time per config update
        "dds_registration_time_ms": 0.1,  # Max time per DDS registration
        "websocket_setup_time_ms": 0.1,  # Max time per WebSocket setup
        "memory_usage_mb": 50.0,  # Max baseline memory usage
        "cpu_usage_percent": 10.0,  # Max baseline CPU usage
        "recovery_coordination_time_sec": 2.0,  # Max recovery coordination time
    }

    REGRESSION_THRESHOLD = 1.5  # 150% of baseline = regression

    def setUp(self):
        """Setup performance test environment."""
        self.process = psutil.Process()
        self.baseline_memory = self.process.memory_info().rss / 1024 / 1024
        self.baseline_cpu = self.process.cpu_percent()

    def test_state_sync_performance_regression(self):
        """Test state synchronization performance against baseline."""
        print("[REFRESH] Testing State Sync Performance Regression...")

        from core.state_synchronization_manager import DistributedStateManager

        # Setup state manager
        mgr = DistributedStateManager("perf_test")
        mgr.start()
        mgr.register_node("perf_test")

        # Test individual operation latency
        latencies = []
        for i in range(100):
            start = time.perf_counter()
            mgr.update_state(f"perf_{i}", f"value_{i}")
            end = time.perf_counter()
            latencies.append((end - start) * 1000)  # Convert to ms

        avg_latency = mean(latencies)
        max_latency = max(latencies)
        p95_latency = sorted(latencies)[int(len(latencies) * 0.95)]

        print(".2f")
        print(".2f")
        print(".2f")

        # Check against baseline
        self.assertLess(
            avg_latency,
            self.BASELINE_METRICS["state_sync_operation_time_ms"]
            * self.REGRESSION_THRESHOLD,
            f"Average latency {avg_latency:.2f}ms exceeds regression threshold",
        )

        # Test throughput
        start_time = time.time()
        ops_count = 1000
        for i in range(ops_count):
            mgr.update_state(f"throughput_{i}", f"value_{i}")
        end_time = time.time()

        throughput = ops_count / (end_time - start_time)
        print(".0f")

        self.assertGreater(
            throughput,
            self.BASELINE_METRICS["state_sync_throughput_ops_sec"]
            / self.REGRESSION_THRESHOLD,
            f"Throughput {throughput:.0f} ops/sec below regression threshold",
        )

        mgr.stop()
        print("  [PASS] State sync performance within acceptable range")

    def test_dynamic_config_performance_regression(self):
        """Test dynamic configuration performance against baseline."""
        print(" Testing Dynamic Config Performance Regression...")

        from core.dynamic_config_manager import DynamicConfigManager

        config_mgr = DynamicConfigManager()
        config_mgr.register_node("perf_test", {"param": 0})

        # Test individual update latency
        latencies = []
        for i in range(100):
            start = time.perf_counter()
            config_mgr.update_node_config("perf_test", "param", i)
            end = time.perf_counter()
            latencies.append((end - start) * 1000)  # Convert to ms

        avg_latency = mean(latencies)
        max_latency = max(latencies)

        print(".2f")
        print(".2f")

        # Check against baseline
        self.assertLess(
            avg_latency,
            self.BASELINE_METRICS["config_update_time_ms"] * self.REGRESSION_THRESHOLD,
            f"Average config update latency {avg_latency:.2f}ms exceeds regression threshold",
        )

        # Test bulk updates
        start_time = time.time()
        bulk_updates = [
            {"node_name": "perf_test", "parameter_name": "param", "new_value": i}
            for i in range(50)
        ]
        config_mgr.update_multiple_configs(bulk_updates)
        end_time = time.time()

        bulk_time = (end_time - start_time) * 1000  # Convert to ms
        print(".2f")

        self.assertLess(
            bulk_time,
            self.BASELINE_METRICS["config_update_time_ms"]
            * 50
            * self.REGRESSION_THRESHOLD,
            f"Bulk config update time {bulk_time:.2f}ms exceeds regression threshold",
        )

        print("  [PASS] Dynamic config performance within acceptable range")

    def test_dds_domain_performance_regression(self):
        """Test DDS domain management performance against baseline."""
        print("[NETWORK] Testing DDS Domain Performance Regression...")

        from core.dds_domain_redundancy_manager import DDSDomainRedundancyManager

        # Test registration performance
        latencies = []
        for i in range(50):
            dds_mgr = DDSDomainRedundancyManager()
            start = time.perf_counter()
            dds_mgr.register_node(f"perf_node_{i}", f"ros2 run test test_node_{i}")
            end = time.perf_counter()
            latencies.append((end - start) * 1000)  # Convert to ms
            dds_mgr.stop()

        avg_latency = mean(latencies)
        max_latency = max(latencies)

        print(".2f")
        print(".2f")

        # Check against baseline
        self.assertLess(
            avg_latency,
            self.BASELINE_METRICS["dds_registration_time_ms"]
            * self.REGRESSION_THRESHOLD,
            f"Average DDS registration latency {avg_latency:.2f}ms exceeds regression threshold",
        )

        # Test domain health checking performance
        dds_mgr = DDSDomainRedundancyManager()
        dds_mgr.register_node("health_test", "echo test")

        health_check_times = []
        for i in range(20):
            start = time.perf_counter()
            health_score = dds_mgr._measure_domain_health(100)
            end = time.perf_counter()
            health_check_times.append((end - start) * 1000)

        avg_health_time = mean(health_check_times)
        print(".2f")

        dds_mgr.stop()
        print("  [PASS] DDS domain performance within acceptable range")

    def test_websocket_redundancy_performance_regression(self):
        """Test WebSocket redundancy performance against baseline."""
        print("[NETWORK] Testing WebSocket Redundancy Performance Regression...")

        from bridges.websocket_redundancy_manager import (
            EndpointPriority,
            WebSocketEndpoint,
            WebSocketRedundancyManager,
        )

        # Test setup performance
        setup_times = []
        for i in range(20):
            ws_mgr = WebSocketRedundancyManager()
            ws_mgr.start_redundancy_system()

            start = time.perf_counter()
            endpoint = WebSocketEndpoint(
                f"perf_endpoint_{i}", 8080 + i, EndpointPriority.PRIMARY
            )
            endpoint.is_running = True
            endpoint.response_time = 0.05
            endpoint.last_health_check = time.time()
            ws_mgr.add_endpoint(endpoint)
            end = time.perf_counter()

            setup_times.append((end - start) * 1000)  # Convert to ms
            ws_mgr.stop_redundancy_system()

        avg_setup_time = mean(setup_times)
        max_setup_time = max(setup_times)

        print(".2f")
        print(".2f")

        # Check against baseline
        self.assertLess(
            avg_setup_time,
            self.BASELINE_METRICS["websocket_setup_time_ms"]
            * self.REGRESSION_THRESHOLD,
            f"Average WebSocket setup time {avg_setup_time:.2f}ms exceeds regression threshold",
        )

        # Test health monitoring performance
        ws_mgr = WebSocketRedundancyManager()
        ws_mgr.start_redundancy_system()

        for i in range(10):
            endpoint = WebSocketEndpoint(
                f"health_endpoint_{i}", 8080 + i, EndpointPriority.PRIMARY
            )
            endpoint.is_running = True
            endpoint.response_time = 0.05
            endpoint.last_health_check = time.time()
            ws_mgr.add_endpoint(endpoint)

        health_check_times = []
        for i in range(50):
            start = time.perf_counter()
            ws_mgr._check_endpoint_health()
            end = time.perf_counter()
            health_check_times.append((end - start) * 1000)

        avg_health_check_time = mean(health_check_times)
        print(".2f")

        ws_mgr.stop_redundancy_system()
        print("  [PASS] WebSocket redundancy performance within acceptable range")

    def test_recovery_coordination_performance_regression(self):
        """Test recovery coordination performance against baseline."""
        print("[TOOL] Testing Recovery Coordination Performance Regression...")

        from bridges.websocket_redundancy_manager import (
            EndpointPriority,
            WebSocketEndpoint,
            WebSocketRedundancyManager,
        )
        from core.dds_domain_redundancy_manager import DDSDomainRedundancyManager
        from core.dynamic_config_manager import DynamicConfigManager
        from core.recovery_coordinator import RecoveryCoordinator
        from core.state_synchronization_manager import DistributedStateManager

        # Setup all systems
        recovery_coord = RecoveryCoordinator()
        state_mgr = DistributedStateManager("recovery_perf")
        dds_mgr = DDSDomainRedundancyManager()
        config_mgr = DynamicConfigManager()
        ws_mgr = WebSocketRedundancyManager()

        # Start systems
        state_mgr.start()
        dds_mgr.start()
        ws_mgr.start_redundancy_system()

        # Setup healthy state
        state_mgr.register_node("recovery_perf")
        state_mgr._trigger_election()

        dds_mgr.register_node("recovery_perf", "echo test")

        ws_endpoint = WebSocketEndpoint("recovery_perf", 8080, EndpointPriority.PRIMARY)
        ws_endpoint.is_running = True
        ws_endpoint.response_time = 0.05
        ws_endpoint.last_health_check = time.time()
        ws_mgr.add_endpoint(ws_endpoint)
        ws_mgr._check_endpoint_health()

        # Register with recovery coordinator
        recovery_coord.register_system_manager("state", state_mgr)
        recovery_coord.register_system_manager("dds", dds_mgr)
        recovery_coord.register_system_manager("config", config_mgr)
        recovery_coord.register_system_manager("websocket", ws_mgr)

        # Test recovery performance
        start_time = time.time()
        success = recovery_coord.initiate_recovery("Performance test recovery")

        # Wait for completion
        timeout = 30
        start_wait = time.time()
        while recovery_coord.recovery_active and (time.time() - start_wait) < timeout:
            time.sleep(0.1)

        end_time = time.time()
        recovery_time = end_time - start_time

        print(".2f")

        # Check against baseline
        self.assertLess(
            recovery_time,
            self.BASELINE_METRICS["recovery_coordination_time_sec"]
            * self.REGRESSION_THRESHOLD,
            f"Recovery coordination time {recovery_time:.2f}s exceeds regression threshold",
        )

        self.assertTrue(success, "Recovery should succeed")
        final_status = recovery_coord.get_recovery_status()
        self.assertEqual(
            final_status["current_phase"], "complete", "Recovery should complete"
        )

        # Cleanup
        state_mgr.stop()
        dds_mgr.stop()
        ws_mgr.stop_redundancy_system()

        print("  [PASS] Recovery coordination performance within acceptable range")

    def test_memory_usage_regression(self):
        """Test memory usage against baseline."""
        print(" Testing Memory Usage Regression...")

        # Get current memory usage
        current_memory = self.process.memory_info().rss / 1024 / 1024

        # Load all systems
        from bridges.websocket_redundancy_manager import WebSocketRedundancyManager
        from core.dds_domain_redundancy_manager import DDSDomainRedundancyManager
        from core.dynamic_config_manager import DynamicConfigManager
        from core.state_synchronization_manager import DistributedStateManager

        systems = []
        systems.append(DistributedStateManager("memory_test"))
        systems.append(DDSDomainRedundancyManager())
        systems.append(DynamicConfigManager())
        systems.append(WebSocketRedundancyManager())

        # Start systems
        for system in systems[:3]:  # Skip WebSocket for simplicity
            if hasattr(system, "start"):
                system.start()

        # Get memory usage with systems loaded
        loaded_memory = self.process.memory_info().rss / 1024 / 1024
        memory_delta = loaded_memory - current_memory

        print(".1f")
        print(".1f")
        print(".1f")

        # Check against baseline
        self.assertLess(
            loaded_memory,
            self.BASELINE_METRICS["memory_usage_mb"] * self.REGRESSION_THRESHOLD,
            f"Memory usage {loaded_memory:.1f}MB exceeds regression threshold",
        )

        # Cleanup
        for system in systems[:3]:
            if hasattr(system, "stop"):
                system.stop()

        print("  [PASS] Memory usage within acceptable range")

    def test_cpu_usage_regression(self):
        """Test CPU usage against baseline."""
        print("[LIGHTNING] Testing CPU Usage Regression...")

        # Test idle CPU
        time.sleep(1)
        idle_cpu = self.process.cpu_percent(interval=1)
        print(".1f")

        self.assertLess(
            idle_cpu,
            self.BASELINE_METRICS["cpu_usage_percent"] * self.REGRESSION_THRESHOLD,
            f"Idle CPU usage {idle_cpu:.1f}% exceeds regression threshold",
        )

        # Test CPU under load
        from core.state_synchronization_manager import DistributedStateManager

        mgr = DistributedStateManager("cpu_test")
        mgr.start()
        mgr.register_node("cpu_test")

        # Generate CPU load
        for i in range(10000):
            mgr.update_state(f"cpu_load_{i}", f"value_{i}")

        load_cpu = self.process.cpu_percent(interval=1)
        print(".1f")

        mgr.stop()

        # CPU under load should be reasonable (not a strict regression test)
        self.assertLess(
            load_cpu, 50.0, f"Load CPU usage {load_cpu:.1f}% excessively high"
        )

        print("  [PASS] CPU usage within acceptable range")

    def test_performance_trend_analysis(self):
        """Analyze performance trends across multiple runs."""
        print(" Testing Performance Trend Analysis...")

        # This would typically compare against historical data
        # For now, just ensure current performance is consistent

        from core.state_synchronization_manager import DistributedStateManager

        mgr = DistributedStateManager("trend_test")
        mgr.start()
        mgr.register_node("trend_test")

        # Run multiple measurement cycles
        measurements = []
        for cycle in range(5):
            start = time.time()
            for i in range(100):
                mgr.update_state(f"trend_{cycle}_{i}", f"value_{i}")
            end = time.time()
            measurements.append(end - start)

        avg_time = mean(measurements)
        std_dev = stdev(measurements) if len(measurements) > 1 else 0

        print(".3f")
        print(".3f")

        # Performance should be reasonably consistent
        cv = std_dev / avg_time if avg_time > 0 else 0  # Coefficient of variation
        self.assertLess(cv, 0.5, f"Performance variation {cv:.2f} too high")

        mgr.stop()
        print("  [PASS] Performance consistency within acceptable range")


if __name__ == "__main__":
    # Run performance regression tests
    unittest.main(verbosity=2)
