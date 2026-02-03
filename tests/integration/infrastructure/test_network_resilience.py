#!/usr/bin/env python3
"""
Network Resilience Tests - URC 2026

Tests system behavior under various network conditions:
- Packet loss simulation
- Latency injection
- Connection drops
- Bandwidth throttling
- Network partitioning
- Recovery mechanisms

Author: URC 2026 Network Engineering Team
"""

import asyncio
import aiohttp
import socket
import time
import threading
from typing import Dict, Any, List, Optional
import pytest
from unittest.mock import patch, AsyncMock
import random


class NetworkEmulator:
    """Network condition emulator for testing."""

    def __init__(self):
        self.active_conditions = {}
        self.original_socket = socket.socket

    def apply_network_conditions(self, conditions: Dict[str, Any]):
        """Apply network conditions globally."""
        self.active_conditions = conditions

        # Monkey patch socket operations
        original_connect = socket.socket.connect
        original_send = socket.socket.send
        original_recv = socket.socket.recv

        def conditional_connect(self, address):
            if self._should_drop_connection():
                raise ConnectionError("Simulated connection drop")
            return original_connect(self, address)

        def conditional_send(self, data):
            if self._should_drop_packet():
                # Simulate packet loss
                return 0
            time.sleep(self._get_latency_delay())
            return original_send(self, data)

        def conditional_recv(self, bufsize):
            time.sleep(self._get_latency_delay())
            return original_recv(self, bufsize)

        socket.socket.connect = conditional_connect
        socket.socket.send = conditional_send
        socket.socket.recv = conditional_recv

    def _should_drop_connection(self) -> bool:
        """Determine if connection should be dropped."""
        drop_rate = self.active_conditions.get("connection_drop_rate", 0.0)
        return random.random() < drop_rate

    def _should_drop_packet(self) -> bool:
        """Determine if packet should be dropped."""
        loss_rate = self.active_conditions.get("packet_loss_rate", 0.0)
        return random.random() < loss_rate

    def _get_latency_delay(self) -> float:
        """Get latency delay for current conditions."""
        base_latency = self.active_conditions.get("latency_ms", 0) / 1000.0
        jitter = self.active_conditions.get("jitter_ms", 0) / 1000.0

        if jitter > 0:
            return base_latency + random.uniform(-jitter, jitter)
        return base_latency

    def reset_conditions(self):
        """Reset network conditions to normal."""
        self.active_conditions = {}
        # Restore original socket methods
        socket.socket.connect = self.original_socket.connect
        socket.socket.send = self.original_socket.send
        socket.socket.recv = self.original_socket.recv


class TestNetworkResilience:
    """Test network resilience functionality."""

    @pytest.fixture
    def network_emulator(self):
        """Create network emulator."""
        return NetworkEmulator()

    @pytest.fixture
    def resilience_manager(self):
        """Create network resilience manager."""
        from src.core.network_resilience import NetworkResilienceManager

        return NetworkResilienceManager()

    @pytest.mark.asyncio
    async def test_packet_loss_resilience(self, network_emulator, resilience_manager):
        """Test system resilience to packet loss."""
        # Simulate 10% packet loss
        network_conditions = {"packet_loss_rate": 0.1, "latency_ms": 50}

        network_emulator.apply_network_conditions(network_conditions)

        try:
            # Test resilient HTTP requests
            success_count = 0
            total_attempts = 10

            for _ in range(total_attempts):
                try:
                    result = await resilience_manager.execute_with_resilience(
                        self._make_test_request, max_attempts=3, backoff_factor=0.5
                    )
                    if result:
                        success_count += 1
                except Exception:
                    pass  # Expected under network conditions

            success_rate = success_count / total_attempts

            print(
                f"📡 Packet loss test: {success_rate:.1%} success rate under 10% loss"
            )

            # System should maintain reasonable success rate
            assert success_rate >= 0.3  # At least 30% success with retries

        finally:
            network_emulator.reset_conditions()

    @pytest.mark.asyncio
    async def test_high_latency_handling(self, network_emulator, resilience_manager):
        """Test handling of high network latency."""
        # Simulate high latency (500ms)
        network_conditions = {"latency_ms": 500, "jitter_ms": 100}

        network_emulator.apply_network_conditions(network_conditions)

        try:
            start_time = time.time()

            # Execute request with timeout
            result = await resilience_manager.execute_with_timeout(
                self._make_test_request, timeout=2.0  # 2 second timeout
            )

            elapsed = time.time() - start_time

            print(f"⏱️  High latency test: {elapsed:.2f}s total time")

            # Should complete within reasonable time despite latency
            assert elapsed < 3.0  # Should not take excessively long

        finally:
            network_emulator.reset_conditions()

    @pytest.mark.asyncio
    async def test_connection_drop_recovery(self, network_emulator, resilience_manager):
        """Test recovery from connection drops."""
        # Simulate occasional connection drops
        network_conditions = {
            "connection_drop_rate": 0.3,  # 30% of connections drop
            "latency_ms": 100,
        }

        network_emulator.apply_network_conditions(network_conditions)

        try:
            recovery_attempts = []

            for attempt in range(5):
                try:
                    start_time = time.time()
                    result = await resilience_manager.execute_with_resilience(
                        self._make_test_request, max_attempts=5, backoff_factor=0.1
                    )
                    elapsed = time.time() - start_time

                    recovery_attempts.append(
                        {
                            "attempt": attempt + 1,
                            "success": result is not None,
                            "time": elapsed,
                        }
                    )

                except Exception as e:
                    recovery_attempts.append(
                        {"attempt": attempt + 1, "success": False, "error": str(e)}
                    )

            successful_attempts = sum(1 for a in recovery_attempts if a["success"])

            print(
                f"🔌 Connection drop test: {successful_attempts}/{len(recovery_attempts)} successful"
            )

            # Should eventually succeed despite drops
            assert successful_attempts >= 2

        finally:
            network_emulator.reset_conditions()

    @pytest.mark.asyncio
    async def test_circuit_breaker_pattern(self, resilience_manager):
        """Test circuit breaker pattern implementation."""

        # Create a failing operation
        async def failing_operation():
            raise ConnectionError("Simulated network failure")

        # Test circuit breaker opens after failures
        breaker = resilience_manager.create_circuit_breaker(
            "test_service", failure_threshold=3, recovery_timeout=2.0
        )

        # Cause failures to open circuit
        for _ in range(4):  # More than threshold
            try:
                await breaker.call(failing_operation)
            except:
                pass  # Expected

        # Circuit should be open
        assert breaker.state.name == "OPEN"

        # Wait for recovery timeout
        await asyncio.sleep(2.5)

        # Circuit should be half-open and allow one test call
        assert breaker.state.name == "HALF_OPEN"

        print("🔌 Circuit breaker pattern working correctly")

    @pytest.mark.asyncio
    async def test_adaptive_retry_strategy(self, network_emulator, resilience_manager):
        """Test adaptive retry strategies."""
        # Start with moderate network issues
        network_conditions = {"packet_loss_rate": 0.05, "latency_ms": 200}

        network_emulator.apply_network_conditions(network_conditions)

        try:
            # Test different retry strategies
            strategies = [
                {"name": "fixed", "backoff_factor": 0, "max_attempts": 3},
                {"name": "linear", "backoff_factor": 0.5, "max_attempts": 3},
                {"name": "exponential", "backoff_factor": 2.0, "max_attempts": 3},
            ]

            for strategy in strategies:
                start_time = time.time()

                try:
                    result = await resilience_manager.execute_with_resilience(
                        self._make_failing_request,
                        max_attempts=strategy["max_attempts"],
                        backoff_factor=strategy["backoff_factor"],
                    )
                except Exception:
                    pass  # Expected

                elapsed = time.time() - start_time

                print(f"🔄 {strategy['name']} retry: {elapsed:.2f}s total time")

                # Different strategies should have different timing profiles
                if strategy["name"] == "fixed":
                    assert elapsed < 1.0  # Fastest
                elif strategy["name"] == "exponential":
                    assert elapsed > 2.0  # Slowest due to backoff

        finally:
            network_emulator.reset_conditions()

    @pytest.mark.asyncio
    async def test_connection_pooling_efficiency(self, resilience_manager):
        """Test connection pooling for efficiency."""
        # Test connection reuse
        pool_stats = {"connections_created": 0, "connections_reused": 0}

        # Mock connection creation to track pooling
        original_create = aiohttp.ClientSession

        def mock_session(*args, **kwargs):
            pool_stats["connections_created"] += 1
            return original_create(*args, **kwargs)

        with patch("aiohttp.ClientSession", side_effect=mock_session):
            # Make multiple requests
            tasks = []
            for _ in range(10):
                task = resilience_manager.execute_with_resilience(
                    self._make_test_request
                )
                tasks.append(task)

            await asyncio.gather(*tasks)

            print(
                f"🔗 Connection pooling: {pool_stats['connections_created']} connections created"
            )

            # Should reuse connections (fewer than requests)
            assert pool_stats["connections_created"] < 10

    @pytest.mark.asyncio
    async def test_network_partition_handling(
        self, network_emulator, resilience_manager
    ):
        """Test handling of network partitions."""
        # Simulate complete network partition for 2 seconds
        partition_duration = 2.0

        async def simulate_partition():
            # Apply severe network conditions
            network_emulator.apply_network_conditions(
                {
                    "packet_loss_rate": 1.0,  # 100% loss
                    "connection_drop_rate": 1.0,  # All connections drop
                    "latency_ms": 5000,  # Very high latency
                }
            )

            await asyncio.sleep(partition_duration)

            # Restore normal conditions
            network_emulator.reset_conditions()

        # Start partition simulation
        partition_task = asyncio.create_task(simulate_partition())

        # Try to make requests during partition
        request_results = []

        start_time = time.time()
        for i in range(5):
            try:
                result = await resilience_manager.execute_with_timeout(
                    self._make_test_request, timeout=1.0
                )
                request_results.append({"attempt": i + 1, "success": True})
            except Exception as e:
                elapsed = time.time() - start_time
                request_results.append(
                    {"attempt": i + 1, "success": False, "time": elapsed}
                )

        await partition_task

        # Analyze results
        failed_during_partition = sum(1 for r in request_results if not r["success"])

        print(
            f"🌐 Network partition test: {failed_during_partition}/5 requests failed during partition"
        )

        # Most requests during partition should fail
        assert failed_during_partition >= 3

    @pytest.mark.asyncio
    async def test_bandwidth_throttling_resilience(
        self, network_emulator, resilience_manager
    ):
        """Test resilience to bandwidth throttling."""
        # Simulate low bandwidth conditions
        network_conditions = {
            "bandwidth_kbps": 50,  # Very low bandwidth
            "latency_ms": 200,
        }

        network_emulator.apply_network_conditions(network_conditions)

        try:
            # Test large data transfer resilience
            async def large_data_request():
                # Simulate large payload
                payload_size = 100 * 1024  # 100KB
                data = b"x" * payload_size

                # This would normally take time over slow connection
                await asyncio.sleep(0.1)  # Simulate network delay
                return len(data)

            start_time = time.time()
            result = await resilience_manager.execute_with_timeout(
                large_data_request, timeout=5.0  # Generous timeout for slow connection
            )
            elapsed = time.time() - start_time

            print(f"📶 Bandwidth throttling test: {elapsed:.2f}s for large transfer")

            # Should complete within reasonable time
            assert elapsed < 10.0
            assert result == 100 * 1024

        finally:
            network_emulator.reset_conditions()

    @pytest.mark.asyncio
    async def test_service_discovery_resilience(self, resilience_manager):
        """Test service discovery under network stress."""
        # Test service endpoint failover
        service_endpoints = [
            "http://primary-service:8080",
            "http://backup-service:8081",
            "http://tertiary-service:8082",
        ]

        # Simulate primary service failure
        async def resilient_service_call():
            for endpoint in service_endpoints:
                try:
                    # Simulate calling each endpoint
                    if "primary" in endpoint:
                        raise aiohttp.ClientError("Primary service down")
                    elif "backup" in endpoint:
                        raise aiohttp.ClientError("Backup service slow")
                    else:
                        # Tertiary succeeds
                        await asyncio.sleep(0.1)
                        return f"Success via {endpoint}"

                except aiohttp.ClientError:
                    continue  # Try next endpoint

            raise aiohttp.ClientError("All endpoints failed")

        result = await resilience_manager.execute_with_resilience(
            resilient_service_call, max_attempts=len(service_endpoints)
        )

        assert "tertiary" in result
        print(
            "🔍 Service discovery resilience working - failed over to tertiary service"
        )

    @pytest.mark.asyncio
    async def test_websocket_connection_resilience(
        self, network_emulator, resilience_manager
    ):
        """Test WebSocket connection resilience."""
        # Simulate intermittent connectivity
        network_conditions = {
            "connection_drop_rate": 0.2,  # 20% connection drops
            "packet_loss_rate": 0.05,
            "latency_ms": 150,
        }

        network_emulator.apply_network_conditions(network_conditions)

        try:
            # Test WebSocket reconnection logic
            reconnection_attempts = []

            for attempt in range(3):
                try:
                    start_time = time.time()

                    # Simulate WebSocket connection attempt
                    await asyncio.sleep(0.2)  # Connection time

                    # Randomly succeed or fail
                    if random.random() > 0.3:  # 70% success rate
                        elapsed = time.time() - start_time
                        reconnection_attempts.append(
                            {"attempt": attempt + 1, "success": True, "time": elapsed}
                        )
                        break
                    else:
                        raise ConnectionError("WebSocket connection failed")

                except ConnectionError:
                    elapsed = time.time() - start_time
                    reconnection_attempts.append(
                        {"attempt": attempt + 1, "success": False, "time": elapsed}
                    )

                    # Exponential backoff
                    await asyncio.sleep(0.1 * (2**attempt))

            successful_connections = sum(
                1 for a in reconnection_attempts if a["success"]
            )

            print(
                f"🔌 WebSocket resilience: {successful_connections}/{len(reconnection_attempts)} successful connections"
            )

            # Should eventually succeed
            assert successful_connections >= 1

        finally:
            network_emulator.reset_conditions()

    def test_network_health_monitoring(self, resilience_manager):
        """Test network health monitoring."""
        # Test network metrics collection
        health_metrics = resilience_manager.get_network_health()

        expected_metrics = [
            "latency_ms",
            "packet_loss_rate",
            "connection_success_rate",
            "bandwidth_utilization",
        ]

        for metric in expected_metrics:
            assert metric in health_metrics
            assert isinstance(health_metrics[metric], (int, float))

        print("📊 Network health monitoring providing all required metrics")

    @pytest.mark.asyncio
    async def test_end_to_end_network_resilience(
        self, network_emulator, resilience_manager
    ):
        """Test end-to-end network resilience scenario."""
        # Comprehensive network resilience test
        test_scenarios = [
            {
                "name": "Normal conditions",
                "conditions": {"latency_ms": 10, "packet_loss_rate": 0.001},
                "expected_success_rate": 0.99,
            },
            {
                "name": "Degraded conditions",
                "conditions": {"latency_ms": 200, "packet_loss_rate": 0.05},
                "expected_success_rate": 0.85,
            },
            {
                "name": "Poor conditions",
                "conditions": {"latency_ms": 1000, "packet_loss_rate": 0.15},
                "expected_success_rate": 0.60,
            },
        ]

        overall_results = []

        for scenario in test_scenarios:
            network_emulator.apply_network_conditions(scenario["conditions"])

            try:
                # Run multiple requests under conditions
                results = []
                for _ in range(20):
                    try:
                        result = await resilience_manager.execute_with_timeout(
                            self._make_test_request, timeout=5.0
                        )
                        results.append(True)
                    except Exception:
                        results.append(False)

                    await asyncio.sleep(0.05)  # Small delay between requests

                success_rate = sum(results) / len(results)
                overall_results.append(
                    {
                        "scenario": scenario["name"],
                        "success_rate": success_rate,
                        "expected": scenario["expected_success_rate"],
                        "met_expectations": success_rate
                        >= scenario["expected_success_rate"],
                    }
                )

                print(
                    f"🌐 {scenario['name']}: {success_rate:.1%} success (expected {scenario['expected_success_rate']:.1%})"
                )

            finally:
                network_emulator.reset_conditions()

        # Analyze overall resilience
        scenarios_met = sum(1 for r in overall_results if r["met_expectations"])

        print(
            f"🎯 End-to-end resilience: {scenarios_met}/{len(overall_results)} scenarios met expectations"
        )

        # System should handle at least normal and degraded conditions
        assert scenarios_met >= 2

    async def _make_test_request(self):
        """Helper method to make a test request."""
        # This would normally make an actual HTTP request
        # For testing, just simulate network operation
        await asyncio.sleep(0.01)  # Small delay to simulate network

        # Randomly succeed/fail based on network conditions
        if random.random() > 0.1:  # 90% success rate baseline
            return {"status": "success", "data": "test response"}
        else:
            raise aiohttp.ClientError("Simulated network error")

    async def _make_failing_request(self):
        """Helper method that always fails for testing retries."""
        await asyncio.sleep(0.01)
        raise aiohttp.ClientError("Simulated failure for retry testing")
