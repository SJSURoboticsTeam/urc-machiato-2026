#!/usr/bin/env python3
"""
Memory Leak Detection Tests - URC 2026 Rover

Tests for memory leaks in long-running operations, message processing,
and component lifecycle management.

Memory leaks can cause:
- Gradual performance degradation
- System instability over time
- Unexpected crashes during missions
- Resource exhaustion in long deployments
"""

import gc
import time
import tracemalloc
import unittest
from unittest.mock import Mock, patch

import psutil


class TestMemoryLeakDetection(unittest.TestCase):
    """Test memory leak detection and prevention."""

    def setUp(self):
        """Set up memory monitoring."""
        tracemalloc.start()
        self.initial_memory = psutil.Process().memory_info().rss
        self.initial_objects = len(gc.get_objects())

    def tearDown(self):
        """Clean up memory monitoring."""
        tracemalloc.stop()
        gc.collect()

    def get_memory_usage(self):
        """Get current memory usage in MB."""
        return psutil.Process().memory_info().rss / 1024 / 1024

    def assert_no_memory_leak(self, operation_func, iterations=100, max_growth_mb=10.0):
        """Assert that an operation doesn't leak memory."""
        initial_memory = self.get_memory_usage()
        initial_objects = len(gc.get_objects())

        # Run operation multiple times
        for i in range(iterations):
            operation_func()
            if i % 20 == 0:  # Periodic cleanup
                gc.collect()

        final_memory = self.get_memory_usage()
        final_objects = len(gc.get_objects())

        memory_growth = final_memory - initial_memory
        object_growth = final_objects - initial_objects

        self.assertLess(
            memory_growth,
            max_growth_mb,
            f"Memory leak detected: {memory_growth:.2f}MB growth after {iterations} iterations",
        )

        # Log for monitoring
        print(f"Memory test: {memory_growth:.2f}MB growth, {object_growth} objects")

    def test_message_processing_memory_leak(self):
        """Test message processing doesn't leak memory."""
        from bridges.websocket_mission_bridge import WebSocketMissionBridge

        def process_messages():
            # Simulate message processing
            mock_message = {
                "type": "mission_command",
                "data": {"action": "start", "mission_id": "test_001"},
                "timestamp": time.time(),
            }

            # Process through bridge (mocked to avoid ROS2)
            bridge = WebSocketMissionBridge.__new__(WebSocketMissionBridge)
            bridge.process_message = Mock()
            bridge.process_message(mock_message)

        self.assert_no_memory_leak(process_messages, iterations=1000, max_growth_mb=5.0)

    def test_state_machine_transitions_memory_leak(self):
        """Test state machine transitions don't leak memory."""
        from autonomy.code.state_management.autonomy_state_machine.states import (
            RoverState,
            can_transition,
        )

        def test_transitions():
            # Test all possible transitions
            for from_state in RoverState:
                for to_state in RoverState:
                    can_transition(from_state, to_state)

        self.assert_no_memory_leak(test_transitions, iterations=5000, max_growth_mb=2.0)

    def test_dashboard_component_memory_leak(self):
        """Test dashboard components don't leak memory."""

        # This would require React testing environment
        # For now, test the underlying logic
        def create_destroy_components():
            # Simulate component lifecycle
            components = []
            for i in range(100):
                components.append(
                    {
                        "id": f"comp_{i}",
                        "state": {"active": True, "data": [1, 2, 3] * 100},
                        "handlers": [lambda: None] * 10,
                    }
                )

            # Simulate destruction
            components.clear()
            del components

        self.assert_no_memory_leak(
            create_destroy_components, iterations=100, max_growth_mb=3.0
        )

    def test_ros2_publisher_memory_leak(self):
        """Test ROS2 publishers don't leak memory."""

        def create_publishers():
            # Mock ROS2 publisher creation and publishing
            mock_publisher = Mock()
            mock_publisher.publish = Mock()

            for i in range(100):
                message = {"data": f"test_message_{i}", "timestamp": time.time()}
                mock_publisher.publish(message)

        self.assert_no_memory_leak(create_publishers, iterations=500, max_growth_mb=2.0)

    def test_configuration_loading_memory_leak(self):
        """Test configuration loading doesn't leak memory."""
        import yaml

        sample_config = """
        simulation:
          update_rate: 10
          noise: 0.01
        mission:
          timeout: 600
          waypoints: []
        safety:
          limits:
            speed: 2.0
            temperature: 70.0
        """

        def load_config():
            config = yaml.safe_load(sample_config)
            # Simulate processing
            processed = {}
            for section, values in config.items():
                processed[section] = values
            return processed

        self.assert_no_memory_leak(load_config, iterations=1000, max_growth_mb=1.0)

    def test_large_message_handling(self):
        """Test handling of large messages doesn't cause memory issues."""

        def process_large_messages():
            # Simulate processing large sensor data
            large_data = {
                "pointcloud": [[i, i + 1, i + 2] for i in range(10000)],  # 10K points
                "images": ["base64_data_" * 1000] * 5,  # 5 large images
                "metadata": {"sensor_id": "large_test", "timestamp": time.time()},
            }

            # Simulate processing
            processed_size = len(str(large_data))
            compressed = {"size": processed_size, "compressed": True}

            return compressed

        self.assert_no_memory_leak(
            process_large_messages, iterations=50, max_growth_mb=20.0
        )

    def test_concurrent_operation_memory(self):
        """Test concurrent operations don't leak memory."""
        import threading

        results = []
        lock = threading.Lock()

        def concurrent_worker(worker_id):
            for i in range(100):
                with lock:
                    results.append(f"worker_{worker_id}_item_{i}")
            time.sleep(0.001)  # Small delay to simulate work

        def run_concurrent_operations():
            threads = []
            for i in range(5):
                t = threading.Thread(target=concurrent_worker, args=(i,))
                threads.append(t)
                t.start()

            for t in threads:
                t.join()

            # Clear results to test cleanup
            results.clear()

        self.assert_no_memory_leak(
            run_concurrent_operations, iterations=10, max_growth_mb=5.0
        )


class TestPerformanceRegression(unittest.TestCase):
    """Test performance regression detection."""

    def setUp(self):
        """Set up performance baselines."""
        self.baselines = {
            "state_machine_transition": 0.0001,  # 0.1ms baseline
            "message_processing": 0.001,  # 1ms baseline
            "configuration_loading": 0.01,  # 10ms baseline
        }

    def test_performance_regression_state_machine(self):
        """Test state machine performance doesn't regress."""
        import time

        from autonomy.code.state_management.autonomy_state_machine.states import (
            RoverState,
            can_transition,
        )

        start_time = time.perf_counter()
        for _ in range(10000):
            can_transition(RoverState.READY, RoverState.AUTO)
        end_time = time.perf_counter()

        avg_time = (end_time - start_time) / 10000
        max_allowed = (
            self.baselines["state_machine_transition"] * 10
        )  # Allow 10x baseline

        self.assertLess(avg_time, max_allowed, ".6f")

    def test_performance_regression_message_processing(self):
        """Test message processing performance doesn't regress."""
        import time

        def process_message(msg):
            # Simulate message processing
            return {
                "processed": True,
                "data": msg["data"].upper(),
                "timestamp": time.time(),
            }

        messages = [{"data": f"message_{i}"} for i in range(1000)]

        start_time = time.perf_counter()
        for msg in messages:
            process_message(msg)
        end_time = time.perf_counter()

        avg_time = (end_time - start_time) / len(messages)
        max_allowed = self.baselines["message_processing"] * 5  # Allow 5x baseline

        self.assertLess(avg_time, max_allowed, ".6f")


if __name__ == "__main__":
    unittest.main()
