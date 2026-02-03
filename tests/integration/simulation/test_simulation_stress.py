#!/usr/bin/env python3
"""
Simulation Stress Tests

Long-running stress tests for memory leaks, connection stability,
and sustained high load.

Author: URC 2026 Testing Team
"""

import asyncio
import sys
import time
from pathlib import Path

import pytest

# Repo root so simulation is importable
_REPO_ROOT = Path(__file__).resolve().parent.parent.parent.parent
sys.path.insert(0, str(_REPO_ROOT))

pytest.importorskip("simulation.network.websocket_server_simulator")
from simulation.integration.full_stack_simulator import create_full_stack_simulator


@pytest.mark.stress
class TestLongRunningStability:
    """Long-running stability tests."""

    @pytest.mark.asyncio
    @pytest.mark.timeout(120)  # 2 minute timeout
    async def test_sustained_operation(self):
        """Test simulator runs stably for extended period."""
        sim = create_full_stack_simulator("default")
        client_id = await sim.websocket_sim.connect()

        print("\n🔄 Running sustained operation test (60 seconds)...")

        start_time = time.time()
        message_count = 0
        errors = []

        # Run for 60 seconds
        while time.time() - start_time < 60.0:
            try:
                await sim.websocket_sim.receive(
                    "driveCommands", {"linear": 0.5, "angular": 0.1}, client_id
                )
                message_count += 1

                # Moderate rate - 10 msg/sec
                await asyncio.sleep(0.1)

            except Exception as e:
                errors.append(str(e))

        elapsed = time.time() - start_time

        sim.shutdown()

        print(f"  Duration: {elapsed:.1f}s")
        print(f"  Messages: {message_count}")
        print(f"  Rate: {message_count/elapsed:.1f} msg/s")
        print(f"  Errors: {len(errors)}")

        # Should send most messages successfully
        assert message_count > 500, f"Too few messages: {message_count}"
        assert len(errors) < message_count * 0.05, f"Too many errors: {len(errors)}"

    @pytest.mark.asyncio
    async def test_memory_stability(self):
        """Test memory doesn't grow unbounded."""
        import psutil
        import os

        sim = create_full_stack_simulator("perfect")
        client_id = await sim.websocket_sim.connect()

        process = psutil.Process(os.getpid())

        # Get initial memory
        initial_memory = process.memory_info().rss / 1024 / 1024  # MB

        print(f"\n💾 Memory stability test...")
        print(f"  Initial memory: {initial_memory:.1f} MB")

        # Send many messages
        for i in range(1000):
            await sim.websocket_sim.receive(
                "driveCommands", {"linear": 0.5, "angular": 0.0}, client_id
            )

            if i % 100 == 0:
                await asyncio.sleep(0.01)

        # Get final memory
        final_memory = process.memory_info().rss / 1024 / 1024  # MB
        memory_growth = final_memory - initial_memory

        sim.shutdown()

        print(f"  Final memory: {final_memory:.1f} MB")
        print(f"  Growth: {memory_growth:.1f} MB")

        # Memory shouldn't grow more than 50 MB
        assert memory_growth < 50, f"Excessive memory growth: {memory_growth:.1f} MB"


@pytest.mark.stress
class TestHighLoadStress:
    """High load stress tests."""

    @pytest.mark.asyncio
    async def test_rapid_message_burst(self):
        """Test handling rapid message bursts."""
        sim = create_full_stack_simulator("perfect")
        client_id = await sim.websocket_sim.connect()

        print("\n⚡ Rapid message burst test...")

        # Send 100 messages as fast as possible
        start = time.time()

        tasks = []
        for i in range(100):
            task = sim.websocket_sim.receive(
                "driveCommands", {"linear": 0.5, "angular": 0.0}, client_id
            )
            tasks.append(task)

        await asyncio.gather(*tasks)

        duration = time.time() - start
        rate = 100 / duration

        sim.shutdown()

        print(f"  Duration: {duration:.3f}s")
        print(f"  Rate: {rate:.0f} msg/s")

        # Should handle burst quickly
        assert duration < 1.0, f"Burst took too long: {duration:.3f}s"

    @pytest.mark.asyncio
    async def test_sustained_high_rate(self):
        """Test sustained high message rate."""
        sim = create_full_stack_simulator("perfect")
        client_id = await sim.websocket_sim.connect()

        print("\n📈 Sustained high rate test (10 seconds @ 50 msg/s)...")

        start_time = time.time()
        sent = 0

        # 50 msg/s for 10 seconds = 500 messages
        while time.time() - start_time < 10.0:
            await sim.websocket_sim.receive(
                "driveCommands", {"linear": 0.5, "angular": 0.0}, client_id
            )
            sent += 1
            await asyncio.sleep(0.02)  # 50 Hz

        elapsed = time.time() - start_time

        stats = sim.websocket_sim.get_statistics()

        sim.shutdown()

        print(f"  Sent: {sent}")
        print(f"  Duration: {elapsed:.1f}s")
        print(f"  Rate: {sent/elapsed:.1f} msg/s")

        assert sent > 450, f"Too few messages sent: {sent}"

    @pytest.mark.asyncio
    async def test_connection_churn(self):
        """Test rapid connect/disconnect cycles."""
        sim = create_full_stack_simulator("perfect")

        print("\n🔄 Connection churn test (50 cycles)...")

        for i in range(50):
            client_id = await sim.websocket_sim.connect()
            await sim.websocket_sim.receive(
                "driveCommands", {"linear": 0.5, "angular": 0.0}, client_id
            )
            await sim.websocket_sim.disconnect(client_id)

        stats = sim.websocket_sim.get_statistics()

        sim.shutdown()

        print(f"  Connections: {stats['stats']['connections']}")
        print(f"  Disconnections: {stats['stats']['disconnections']}")

        assert stats["stats"]["connections"] >= 50


@pytest.mark.stress
class TestConcurrencyStress:
    """Concurrent operation stress tests."""

    @pytest.mark.asyncio
    async def test_concurrent_clients(self):
        """Test many concurrent clients."""
        sim = create_full_stack_simulator("perfect")

        print("\n👥 Concurrent clients test (10 clients)...")

        # Connect 10 clients
        clients = []
        for i in range(10):
            client_id = await sim.websocket_sim.connect()
            clients.append(client_id)

        # Each client sends messages
        async def client_sends(client_id, count):
            for i in range(count):
                await sim.websocket_sim.receive(
                    "driveCommands", {"linear": 0.5, "angular": 0.0}, client_id
                )
                await asyncio.sleep(0.01)

        # Run all clients concurrently
        tasks = [client_sends(cid, 10) for cid in clients]
        await asyncio.gather(*tasks)

        stats = sim.websocket_sim.get_statistics()

        sim.shutdown()

        print(f"  Total messages: {stats['stats']['messages_received']}")

        # Should receive 100 messages (10 clients × 10 messages)
        assert stats["stats"]["messages_received"] >= 90  # Allow some loss


@pytest.mark.stress
class TestRecoveryStress:
    """Recovery and fault tolerance stress tests."""

    @pytest.mark.asyncio
    async def test_repeated_emergency_stops(self):
        """Test repeated emergency stop cycles."""
        sim = create_full_stack_simulator("perfect")
        client_id = await sim.websocket_sim.connect()

        print("\n🛑 Repeated E-stop test (20 cycles)...")

        for i in range(20):
            # Send velocity
            await sim.websocket_sim.receive(
                "driveCommands", {"linear": 0.5, "angular": 0.0}, client_id
            )

            await asyncio.sleep(0.05)

            # Emergency stop
            await sim.websocket_sim.receive("emergencyStop", {}, client_id)

            await asyncio.sleep(0.05)

            # Clear e-stop
            sim.firmware_sim.clear_emergency_stop()

        print(f"  E-stop cycles: {sim.firmware_sim.stats['emergency_stops']}")

        sim.shutdown()

        assert sim.firmware_sim.stats["emergency_stops"] >= 20

    @pytest.mark.asyncio
    async def test_reconnection_resilience(self):
        """Test repeated reconnection cycles."""
        sim = create_full_stack_simulator("default")

        print("\n🔌 Reconnection resilience test (10 cycles)...")

        for i in range(10):
            # Connect
            client_id = await sim.websocket_sim.connect()

            # Send some messages
            for j in range(5):
                await sim.websocket_sim.receive(
                    "driveCommands", {"linear": 0.5, "angular": 0.0}, client_id
                )

            # Disconnect
            await sim.websocket_sim.disconnect(client_id)

        stats = sim.websocket_sim.get_statistics()

        sim.shutdown()

        print(f"  Reconnections: {stats['stats']['connections']}")
        print(f"  Messages: {stats['stats']['messages_received']}")

        assert stats["stats"]["connections"] >= 10
        assert stats["stats"]["messages_received"] >= 45  # 10 cycles × 5 messages


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s", "-m", "stress"])
