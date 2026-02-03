#!/usr/bin/env python3
"""
Communication Stress Testing for URC 2026

Tests communication bridges under high load conditions:
- WebSocket bridge stress testing
- CAN bus performance under load
- ROS2 topic saturation handling
- Message loss and recovery scenarios
- Real-time performance validation

Author: URC 2026 Communication Team
"""

import pytest
import asyncio
import time
import threading
import json
from unittest.mock import Mock, AsyncMock, patch
from typing import Dict, List, Any
import concurrent.futures
import statistics

# Add source paths
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../src"))

pytest.importorskip("src.bridges", reason="src.bridges not available")


class TestCommunicationStress:
    """Test communication systems under stress conditions."""

    @pytest.fixture
    def mock_websocket_bridge(self):
        """Create mock WebSocket bridge for stress testing."""
        from src.bridges.simple_bridge import SimpleBridge

        return SimpleBridge()

    @pytest.fixture
    def performance_metrics(self):
        """Collect performance metrics during tests."""
        return {
            "message_count": 0,
            "latencies": [],
            "dropped_messages": 0,
            "errors": [],
            "start_time": None,
            "end_time": None,
        }

    @pytest.mark.critical
    def test_websocket_high_frequency_commands(
        self, mock_websocket_bridge, performance_metrics
    ):
        """Test WebSocket bridge with high-frequency command stream."""
        message_rate = 100  # Hz
        test_duration = 10.0  # seconds
        total_messages = int(message_rate * test_duration)

        # Register test handler
        results = []

        def test_handler(data):
            results.append(data)
            return {"status": "success", "timestamp": time.time()}

        mock_websocket_bridge.register_command_handler("stress_test", test_handler)

        # Start performance monitoring
        performance_metrics["start_time"] = time.time()

        # Generate high-frequency commands
        with concurrent.futures.ThreadPoolExecutor(max_workers=10) as executor:
            futures = []

            for i in range(total_messages):
                message = {
                    "command": "stress_test",
                    "data": {
                        "sequence": i,
                        "timestamp": time.time(),
                        "payload": "x" * 100,  # 100 byte payload
                    },
                }

                future = executor.submit(
                    asyncio.run, mock_websocket_bridge._process_command(message)
                )
                futures.append(future)

                # Rate limiting
                if i % message_rate == 0:
                    time.sleep(0.01)  # 10ms batches

            # Collect results
            successful_results = 0
            for future in concurrent.futures.as_completed(futures):
                try:
                    result = future.result(timeout=1.0)
                    if result.get("status") == "success":
                        successful_results += 1
                    performance_metrics["message_count"] += 1
                except Exception as e:
                    performance_metrics["errors"].append(str(e))
                    performance_metrics["dropped_messages"] += 1

        performance_metrics["end_time"] = time.time()

        # Calculate metrics
        actual_duration = (
            performance_metrics["end_time"] - performance_metrics["start_time"]
        )
        actual_rate = performance_metrics["message_count"] / actual_duration
        success_rate = successful_results / total_messages

        # Assertions
        assert success_rate >= 0.95, f"Success rate too low: {success_rate:.2%}"
        assert actual_rate >= 50, f"Actual message rate too low: {actual_rate:.1f} Hz"
        assert (
            len(performance_metrics["errors"]) <= total_messages * 0.05
        ), "Too many errors"

        print(f"WebSocket Stress Test Results:")
        print(f"  Messages sent: {total_messages}")
        print(f"  Messages processed: {performance_metrics['message_count']}")
        print(f"  Success rate: {success_rate:.2%}")
        print(f"  Actual rate: {actual_rate:.1f} Hz")
        print(f"  Errors: {len(performance_metrics['errors'])}")

    @pytest.mark.critical
    def test_can_bus_message_saturation(self, performance_metrics):
        """Test CAN bus performance under message saturation."""
        from src.bridges.direct_can_bridge import DirectCANBridge

        can_bridge = DirectCANBridge()

        # Test parameters
        message_rate = 1000  # Hz (high for CAN)
        test_duration = 5.0  # seconds
        total_messages = int(message_rate * test_duration)

        performance_metrics["start_time"] = time.time()

        # Generate CAN messages at high rate
        sent_messages = 0
        failed_messages = 0

        for i in range(total_messages):
            try:
                # Simulate motor command
                motor_id = i % 4  # 4 motors
                speed = (i % 100) / 10.0  # Variable speeds

                success = can_bridge.send_motor_command(motor_id, speed)
                if success:
                    sent_messages += 1
                else:
                    failed_messages += 1

            except Exception as e:
                performance_metrics["errors"].append(str(e))
                failed_messages += 1

            # Rate limiting
            if i % 100 == 0:
                time.sleep(0.001)  # 1ms batches

        performance_metrics["end_time"] = time.time()
        performance_metrics["message_count"] = sent_messages
        performance_metrics["dropped_messages"] = failed_messages

        actual_duration = (
            performance_metrics["end_time"] - performance_metrics["start_time"]
        )
        actual_rate = sent_messages / actual_duration
        success_rate = sent_messages / total_messages

        # CAN bus has lower theoretical maximum than WebSocket
        assert success_rate >= 0.80, f"CAN success rate too low: {success_rate:.2%}"
        assert actual_rate >= 200, f"CAN message rate too low: {actual_rate:.1f} Hz"

        print(f"CAN Bus Saturation Test Results:")
        print(f"  Messages attempted: {total_messages}")
        print(f"  Messages sent: {sent_messages}")
        print(f"  Success rate: {success_rate:.2%}")
        print(f"  Actual rate: {actual_rate:.1f} Hz")
        print(f"  Failed messages: {failed_messages}")

    @pytest.mark.critical
    def test_message_loss_recovery(self, mock_websocket_bridge):
        """Test system recovery from message loss scenarios."""
        recovery_times = []
        messages_before_failure = []
        messages_after_recovery = []

        # Test multiple failure scenarios
        for scenario in range(5):
            # Normal operation baseline
            baseline_results = []

            def baseline_handler(data):
                baseline_results.append(data)
                return {"status": "success"}

            mock_websocket_bridge.register_command_handler("baseline", baseline_handler)

            # Send normal messages
            for i in range(20):
                result = asyncio.run(
                    mock_websocket_bridge._process_command(
                        {"command": "baseline", "data": {"seq": i}}
                    )
                )
                assert result["status"] == "success"

            messages_before_failure.append(len(baseline_results))

            # Simulate message loss (network interruption)
            start_failure = time.time()

            # Handler that fails
            def failing_handler(data):
                raise ConnectionError("Simulated network failure")

            mock_websocket_bridge.register_command_handler("test", failing_handler)

            # Attempt messages during failure
            failure_count = 0
            for i in range(10):
                result = asyncio.run(
                    mock_websocket_bridge._process_command(
                        {"command": "test", "data": {"seq": i}}
                    )
                )
                if result.get("status") == "handler_error":
                    failure_count += 1

            # Recovery handler
            recovery_results = []

            def recovery_handler(data):
                recovery_results.append(data)
                return {"status": "recovered"}

            mock_websocket_bridge.register_command_handler("test", recovery_handler)

            # Test recovery
            recovery_start = time.time()
            for i in range(20):
                result = asyncio.run(
                    mock_websocket_bridge._process_command(
                        {"command": "test", "data": {"seq": i}}
                    )
                )
            recovery_end = time.time()

            recovery_times.append(recovery_end - recovery_start)
            messages_after_recovery.append(len(recovery_results))

        # Validate recovery
        avg_recovery_time = statistics.mean(recovery_times)
        avg_messages_before = statistics.mean(messages_before_failure)
        avg_messages_after = statistics.mean(messages_after_recovery)

        assert avg_recovery_time < 1.0, f"Recovery too slow: {avg_recovery_time:.2f}s"
        assert (
            avg_messages_after >= avg_messages_before * 0.9
        ), "Poor recovery performance"

        print(f"Message Loss Recovery Test Results:")
        print(f"  Average recovery time: {avg_recovery_time:.3f}s")
        print(f"  Average messages before failure: {avg_messages_before:.1f}")
        print(f"  Average messages after recovery: {avg_messages_after:.1f}")

    @pytest.mark.critical
    def test_memory_leak_under_load(self, mock_websocket_bridge):
        """Test for memory leaks during sustained high load."""
        import psutil
        import gc

        process = psutil.Process()
        initial_memory = process.memory_info().rss / 1024 / 1024  # MB

        # Sustained load test
        test_duration = 30.0  # seconds
        message_rate = 50  # Hz
        start_time = time.time()

        memory_samples = []

        while time.time() - start_time < test_duration:
            # Send messages
            for i in range(message_rate):
                result = asyncio.run(
                    mock_websocket_bridge._process_command(
                        {
                            "command": "test",
                            "data": {"payload": "x" * 1000, "timestamp": time.time()},
                        }
                    )
                )

            # Sample memory every second
            current_memory = process.memory_info().rss / 1024 / 1024
            memory_samples.append(current_memory)

            time.sleep(1.0)

        final_memory = process.memory_info().rss / 1024 / 1024
        memory_growth = final_memory - initial_memory
        max_memory = max(memory_samples)
        avg_memory = statistics.mean(memory_samples)

        # Force garbage collection
        gc.collect()
        after_gc_memory = process.memory_info().rss / 1024 / 1024

        # Assert no significant memory leak
        assert memory_growth < 100, f"Memory grew too much: {memory_growth:.1f} MB"
        assert after_gc_memory < final_memory * 1.1, "Memory not properly reclaimed"

        print(f"Memory Leak Test Results:")
        print(f"  Initial memory: {initial_memory:.1f} MB")
        print(f"  Final memory: {final_memory:.1f} MB")
        print(f"  Memory growth: {memory_growth:.1f} MB")
        print(f"  Max memory: {max_memory:.1f} MB")
        print(f"  Avg memory: {avg_memory:.1f} MB")
        print(f"  After GC: {after_gc_memory:.1f} MB")

    @pytest.mark.critical
    def test_concurrent_client_simulation(self, mock_websocket_bridge):
        """Test bridge with multiple concurrent clients."""
        num_clients = 10
        messages_per_client = 50

        # Client simulation function
        def client_simulation(client_id):
            results = []
            errors = []

            for i in range(messages_per_client):
                try:
                    result = asyncio.run(
                        mock_websocket_bridge._process_command(
                            {
                                "command": "client_test",
                                "data": {
                                    "client_id": client_id,
                                    "message_id": i,
                                    "timestamp": time.time(),
                                },
                            }
                        )
                    )
                    results.append(result)
                except Exception as e:
                    errors.append(str(e))

            return {
                "client_id": client_id,
                "results": results,
                "errors": errors,
                "success_count": len(results),
                "error_count": len(errors),
            }

        # Register handler
        handler_results = []

        def client_handler(data):
            handler_results.append(data)
            return {"status": "success", "client": data.get("client_id")}

        mock_websocket_bridge.register_command_handler("client_test", client_handler)

        # Run concurrent clients
        with concurrent.futures.ThreadPoolExecutor(max_workers=num_clients) as executor:
            futures = [
                executor.submit(client_simulation, i) for i in range(num_clients)
            ]

            client_results = []
            for future in concurrent.futures.as_completed(futures):
                result = future.result(timeout=30)
                client_results.append(result)

        # Analyze results
        total_messages = sum(r["success_count"] for r in client_results)
        total_errors = sum(r["error_count"] for r in client_results)
        expected_messages = num_clients * messages_per_client
        success_rate = total_messages / expected_messages

        # Validate concurrent performance
        assert (
            success_rate >= 0.90
        ), f"Concurrent success rate too low: {success_rate:.2%}"
        assert len(handler_results) == total_messages, "Handler results mismatch"

        print(f"Concurrent Client Test Results:")
        print(f"  Clients: {num_clients}")
        print(f"  Messages per client: {messages_per_client}")
        print(f"  Total messages: {total_messages}/{expected_messages}")
        print(f"  Success rate: {success_rate:.2%}")
        print(f"  Total errors: {total_errors}")

    @pytest.mark.critical
    def test_latency_under_load(self, mock_websocket_bridge, performance_metrics):
        """Test message latency under increasing load."""
        load_levels = [10, 25, 50, 100, 200]  # Messages per second
        test_duration_per_level = 5.0

        latency_results = {}

        for load in load_levels:
            latencies = []

            # Register latency tracking handler
            def latency_handler(data):
                receive_time = time.time()
                send_time = data.get("send_time", receive_time)
                latency = (receive_time - send_time) * 1000  # Convert to ms
                latencies.append(latency)
                return {"status": "success", "latency_ms": latency}

            mock_websocket_bridge.register_command_handler(
                "latency_test", latency_handler
            )

            # Send messages at specified load
            start_time = time.time()
            message_count = 0

            while time.time() - start_time < test_duration_per_level:
                # Send burst of messages
                for _ in range(load):
                    message = {
                        "command": "latency_test",
                        "data": {"send_time": time.time(), "message_id": message_count},
                    }

                    asyncio.run(mock_websocket_bridge._process_command(message))
                    message_count += 1

                time.sleep(1.0)  # Rate limiting

            # Calculate statistics
            if latencies:
                avg_latency = statistics.mean(latencies)
                max_latency = max(latencies)
                p95_latency = sorted(latencies)[int(len(latencies) * 0.95)]
            else:
                avg_latency = max_latency = p95_latency = 0

            latency_results[load] = {
                "avg_latency_ms": avg_latency,
                "max_latency_ms": max_latency,
                "p95_latency_ms": p95_latency,
                "message_count": len(latencies),
            }

        # Validate latency requirements
        for load, stats in latency_results.items():
            assert (
                stats["avg_latency_ms"] < 50
            ), f"Avg latency too high at {load} Hz: {stats['avg_latency_ms']:.1f}ms"
            assert (
                stats["p95_latency_ms"] < 100
            ), f"P95 latency too high at {load} Hz: {stats['p95_latency_ms']:.1f}ms"

        print(f"Latency Under Load Test Results:")
        for load, stats in latency_results.items():
            print(
                f"  {load:3d} Hz: Avg={stats['avg_latency_ms']:6.1f}ms, "
                f"Max={stats['max_latency_ms']:6.1f}ms, P95={stats['p95_latency_ms']:6.1f}ms"
            )


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
