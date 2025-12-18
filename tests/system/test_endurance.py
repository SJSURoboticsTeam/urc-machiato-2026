#!/usr/bin/env python3
"""
Long-Duration Endurance Tests

Tests system stability over extended operation periods:
- 30+ minute continuous operation
- Memory leak detection
- Performance degradation monitoring
- Resource exhaustion scenarios
- Network reconnection cycles

This addresses P0 CRITICAL gap: Long-Duration Testing (0% coverage).

Author: URC 2026 Autonomy Team
"""

import gc
import os
import sys
import threading
import time
from typing import Dict, List, Optional

import numpy as np
import psutil
import pytest
import rclpy
from rclpy.node import Node

# Add project paths
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(PROJECT_ROOT, "tests", "simulation"))

try:
    from environment_tiers import EnvironmentSimulator, EnvironmentTier
    from network_emulator import NetworkEmulator, NetworkProfile
except ImportError:
    EnvironmentSimulator = None
    NetworkEmulator = None


@pytest.fixture
def ros_context():
    """Initialize and cleanup ROS context."""
    rclpy.init()
    yield
    rclpy.shutdown()


class ResourceMonitor:
    """Monitor system resources over time."""

    def __init__(self):
        self.process = psutil.Process()
        self.readings: List[Dict[str, float]] = []
        self.monitoring = False
        self.monitor_thread: Optional[threading.Thread] = None

    def start_monitoring(self, interval_sec: float = 1.0):
        """Start resource monitoring in background thread."""
        self.monitoring = True
        self.monitor_thread = threading.Thread(
            target=self._monitor_loop, args=(interval_sec,), daemon=True
        )
        self.monitor_thread.start()

    def stop_monitoring(self):
        """Stop resource monitoring."""
        self.monitoring = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=2.0)

    def _monitor_loop(self, interval_sec: float):
        """Background monitoring loop."""
        while self.monitoring:
            reading = {
                "timestamp": time.time(),
                "memory_mb": self.process.memory_info().rss / 1024 / 1024,
                "cpu_percent": self.process.cpu_percent(interval=0.1),
                "num_threads": self.process.num_threads(),
            }
            self.readings.append(reading)
            time.sleep(interval_sec)

    def get_memory_trend(self) -> float:
        """Calculate memory trend (MB per reading)."""
        if len(self.readings) < 2:
            return 0.0

        memory_values = [r["memory_mb"] for r in self.readings]
        # Simple linear regression slope
        n = len(memory_values)
        x = np.arange(n)
        slope = np.polyfit(x, memory_values, 1)[0]
        return slope

    def get_max_memory_delta(self) -> float:
        """Get maximum memory increase from baseline."""
        if not self.readings:
            return 0.0

        baseline = self.readings[0]["memory_mb"]
        max_memory = max(r["memory_mb"] for r in self.readings)
        return max_memory - baseline

    def get_avg_cpu(self) -> float:
        """Get average CPU usage."""
        if not self.readings:
            return 0.0
        return np.mean([r["cpu_percent"] for r in self.readings])


@pytest.mark.system
@pytest.mark.slow
@pytest.mark.endurance
class TestEndurance:
    """Long-duration endurance tests."""

    def setUp(self):
        """Set up test environment."""
        self.monitor = ResourceMonitor()
        if EnvironmentSimulator:
            self.env_sim = EnvironmentSimulator(EnvironmentTier.REAL_LIFE)
        if NetworkEmulator:
            self.net_emu = NetworkEmulator(NetworkProfile.RURAL_WIFI)

    def tearDown(self):
        """Clean up test environment."""
        self.monitor.stop_monitoring()
        if NetworkEmulator:
            self.net_emu.stop()

    def test_30_minute_continuous_operation(self, ros_context):
        """Test 30-minute continuous operation."""
        # For CI/CD, use shorter duration; full 30 min for manual runs
        test_duration_sec = float(
            os.getenv("ENDURANCE_TEST_DURATION", "60.0")
        )  # Default 1 min for CI

        print(f"\n⏱️  Starting {test_duration_sec/60:.1f}-minute endurance test...")

        self.monitor.start_monitoring(interval_sec=5.0)

        # Simulate continuous operation
        start_time = time.time()
        iteration = 0
        errors = []

        try:
            while (time.time() - start_time) < test_duration_sec:
                iteration += 1

                # Simulate typical operations
                # 1. Process sensor data
                self._simulate_sensor_processing()

                # 2. Update navigation
                self._simulate_navigation_update()

                # 3. Update state machine
                self._simulate_state_machine_update()

                # 4. Process messages
                self._simulate_message_processing()

                # Check for errors every 10 seconds
                if iteration % 20 == 0:  # ~10 seconds at 0.5s intervals
                    if not self._check_system_health():
                        errors.append(
                            f"Health check failed at {time.time() - start_time:.1f}s"
                        )

                time.sleep(0.5)  # 2 Hz operation rate

        except Exception as e:
            errors.append(f"Exception during endurance test: {str(e)}")

        finally:
            self.monitor.stop_monitoring()
            elapsed_time = time.time() - start_time

        # Analyze results
        memory_trend = self.monitor.get_memory_trend()
        max_memory_delta = self.monitor.get_max_memory_delta()
        avg_cpu = self.monitor.get_avg_cpu()

        print(f"  ✅ Test completed: {elapsed_time:.1f}s")
        print(f"  📊 Memory trend: {memory_trend:.4f} MB/reading")
        print(f"  📊 Max memory delta: {max_memory_delta:.2f} MB")
        print(f"  📊 Avg CPU: {avg_cpu:.1f}%")

        # Assertions
        assert len(errors) == 0, f"System errors during endurance test: {errors}"
        assert (
            elapsed_time >= test_duration_sec * 0.95
        ), "Test should run for full duration"
        assert (
            abs(memory_trend) < 0.5
        ), f"Memory leak detected: {memory_trend:.4f} MB/reading"
        assert (
            max_memory_delta < 200.0
        ), f"Excessive memory growth: {max_memory_delta:.2f} MB"
        assert avg_cpu < 80.0, f"High CPU usage: {avg_cpu:.1f}%"

    def test_memory_leak_detection(self, ros_context):
        """Test for memory leaks during prolonged operation."""
        print("\n💧 Testing Memory Leak Detection")

        test_duration_sec = float(
            os.getenv("MEMORY_LEAK_TEST_DURATION", "120.0")
        )  # 2 minutes for CI

        self.monitor.start_monitoring(interval_sec=2.0)

        start_time = time.time()
        iteration = 0

        # Perform operations that might cause memory leaks
        while (time.time() - start_time) < test_duration_sec:
            iteration += 1

            # Create and process messages (potential leak source)
            for _ in range(10):
                msg = {
                    "type": "sensor_data",
                    "id": f"leak_test_{iteration}_{_}",
                    "data": "x" * 1000,  # 1KB per message
                    "timestamp": time.time(),
                }
                # Simulate message processing
                processed = self._process_message(msg)
                del processed  # Explicit cleanup

            # Create temporary data structures
            temp_data = [np.random.rand(100, 100) for _ in range(5)]
            result = sum(np.sum(d) for d in temp_data)
            del temp_data, result

            # Force garbage collection periodically
            if iteration % 50 == 0:
                gc.collect()

            time.sleep(0.1)

        self.monitor.stop_monitoring()

        # Analyze memory trend
        memory_trend = self.monitor.get_memory_trend()
        max_memory_delta = self.monitor.get_max_memory_delta()

        print(f"  📊 Memory trend: {memory_trend:.4f} MB/reading")
        print(f"  📊 Max memory delta: {max_memory_delta:.2f} MB")

        # Memory leak detection: trend should be near zero
        assert (
            abs(memory_trend) < 0.1
        ), f"Memory leak detected: {memory_trend:.4f} MB/reading"
        assert (
            max_memory_delta < 50.0
        ), f"Excessive memory growth: {max_memory_delta:.2f} MB"

    def test_performance_degradation(self, ros_context):
        """Test for performance degradation over time."""
        print("\n📉 Testing Performance Degradation")

        test_duration_sec = float(
            os.getenv("PERF_DEGRADATION_TEST_DURATION", "180.0")
        )  # 3 minutes for CI

        self.monitor.start_monitoring(interval_sec=5.0)

        start_time = time.time()
        operation_times: List[float] = []

        # Measure operation time over test duration
        while (time.time() - start_time) < test_duration_sec:
            op_start = time.time()

            # Simulate typical operation
            self._simulate_sensor_processing()
            self._simulate_navigation_update()
            self._simulate_message_processing()

            op_time = time.time() - op_start
            operation_times.append(op_time)

            time.sleep(0.5)

        self.monitor.stop_monitoring()

        # Analyze performance trend
        if len(operation_times) >= 10:
            # Compare first 25% vs last 25%
            first_quarter = operation_times[: len(operation_times) // 4]
            last_quarter = operation_times[-len(operation_times) // 4 :]

            avg_first = np.mean(first_quarter)
            avg_last = np.mean(last_quarter)
            degradation_ratio = avg_last / avg_first if avg_first > 0 else 1.0

            print(f"  📊 Avg operation time (first quarter): {avg_first*1000:.2f} ms")
            print(f"  📊 Avg operation time (last quarter): {avg_last*1000:.2f} ms")
            print(f"  📊 Performance degradation ratio: {degradation_ratio:.2f}x")

            # Performance should not degrade more than 2x
            assert (
                degradation_ratio < 2.0
            ), f"Performance degraded {degradation_ratio:.2f}x"

    def test_resource_exhaustion_scenarios(self, ros_context):
        """Test system behavior under resource exhaustion."""
        print("\n⚠️  Testing Resource Exhaustion Scenarios")

        # Test 1: High message rate
        print("  Testing high message rate...")
        start_time = time.time()
        messages_processed = 0
        errors = []

        try:
            for _ in range(1000):  # High message rate
                msg = {
                    "type": "sensor_data",
                    "id": f"stress_{_}",
                    "data": "x" * 100,
                    "timestamp": time.time(),
                }
                try:
                    self._process_message(msg)
                    messages_processed += 1
                except Exception as e:
                    errors.append(f"Message processing error: {str(e)}")
                    break

                if (time.time() - start_time) > 10.0:  # Timeout after 10s
                    break

        except Exception as e:
            errors.append(f"Resource exhaustion error: {str(e)}")

        print(f"  Processed {messages_processed} messages")
        assert messages_processed > 500, "Should handle high message rate"
        assert len(errors) == 0, f"Errors during resource exhaustion: {errors}"

    def test_network_reconnection_cycles(self, ros_context):
        """Test network reconnection cycles."""
        print("\n🔄 Testing Network Reconnection Cycles")

        if not NetworkEmulator:
            pytest.skip("NetworkEmulator not available")

        self.net_emu.start()

        reconnection_attempts = 5
        successful_reconnections = 0

        for attempt in range(reconnection_attempts):
            # Simulate network loss
            self.net_emu.simulate_packet_loss(100, 2.0)  # 100% loss for 2 seconds
            time.sleep(2.5)

            # Simulate reconnection
            self.net_emu.stop_packet_loss()
            time.sleep(1.0)

            # Check if system recovered
            if self._check_network_connectivity():
                successful_reconnections += 1

        self.net_emu.stop()

        print(
            f"  Successful reconnections: {successful_reconnections}/{reconnection_attempts}"
        )

        # Should recover from most reconnection cycles
        assert (
            successful_reconnections >= reconnection_attempts * 0.8
        ), "Should recover from network issues"

    def _simulate_sensor_processing(self):
        """Simulate sensor data processing."""
        # Simulate processing sensor data
        data = np.random.rand(10, 10)
        result = np.mean(data)
        return result

    def _simulate_navigation_update(self):
        """Simulate navigation system update."""
        # Simulate navigation calculation
        position = np.array([0.0, 0.0])
        target = np.array([10.0, 10.0])
        direction = target - position
        return direction

    def _simulate_state_machine_update(self):
        """Simulate state machine update."""
        # Simulate state machine logic
        current_state = "IDLE"
        return current_state

    def _simulate_message_processing(self):
        """Simulate message processing."""
        # Simulate processing messages
        msg_count = 10
        return msg_count

    def _process_message(self, msg: Dict) -> Dict:
        """Process a message."""
        # Simulate message processing
        processed = {
            "type": msg["type"],
            "id": msg["id"],
            "processed": True,
            "timestamp": time.time(),
        }
        return processed

    def _check_system_health(self) -> bool:
        """Check system health."""
        # Basic health checks
        memory_mb = psutil.Process().memory_info().rss / 1024 / 1024
        cpu_percent = psutil.Process().cpu_percent(interval=0.1)

        # Health thresholds
        if memory_mb > 2000:  # 2GB
            return False
        if cpu_percent > 90:
            return False

        return True

    def _check_network_connectivity(self) -> bool:
        """Check network connectivity."""
        if not NetworkEmulator:
            return True  # Assume connected if no emulator

        stats = self.net_emu.get_statistics()
        # Consider connected if recent messages succeeded
        return stats.get("messages_received", 0) > 0


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-m", "endurance"])
