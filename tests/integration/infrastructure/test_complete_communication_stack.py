#!/usr/bin/env python3
"""
Complete Communication Stack Integration Tests

Tests the complete communication stack from frontend to firmware
using the full stack simulator without requiring hardware.

Test Coverage:
1. Frontend → WebSocket → ROS2 → SLCAN → Firmware
2. Firmware → SLCAN → ROS2 → WebSocket → Frontend  
3. Emergency stop propagation (<100ms requirement)
4. Network failure recovery
5. Firmware fault handling
6. High message load handling
7. Connection recovery

Author: URC 2026 Integration Testing Team
"""

import pytest
import asyncio
import time
import sys
from pathlib import Path
from typing import Dict, Any

# Add simulation to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

pytest.importorskip("simulation.network.websocket_server_simulator")
from simulation.integration.full_stack_simulator import (
    FullStackSimulator,
    ScenarioType,
    create_full_stack_simulator,
)


class TestCompleteCommunicationStack:
    """Test suite for complete communication stack."""

    @pytest.fixture
    def simulator(self):
        """Create full stack simulator for testing."""
        sim = create_full_stack_simulator("default")
        yield sim
        sim.shutdown()

    @pytest.fixture
    def perfect_simulator(self):
        """Create perfect (no errors) simulator for testing."""
        sim = create_full_stack_simulator("perfect")
        yield sim
        sim.shutdown()

    async def test_frontend_to_firmware_velocity_command(self, simulator):
        """Test complete path: Frontend gamepad → Firmware motors."""
        print("\n=== Test: Frontend to Firmware Velocity Command ===")

        result = await simulator.run_scenario(ScenarioType.BASIC_VELOCITY)

        assert result.success, f"Scenario failed: {result.errors}"
        assert result.duration_s < 1.0, f"Scenario too slow: {result.duration_s:.3f}s"

        # Verify firmware received command
        motor_status = simulator.firmware_sim.get_motor_status(0)
        assert motor_status is not None
        assert abs(motor_status["velocity_setpoint"]) > 0.0

        print(f"✅ Frontend to firmware test passed in {result.duration_s:.3f}s")

    async def test_firmware_to_frontend_status_feedback(self, simulator):
        """Test feedback: Firmware state → Frontend display."""
        print("\n=== Test: Firmware to Frontend Status Feedback ===")

        # Connect client
        client_id = await simulator.websocket_sim.connect()

        # Send command to activate firmware
        await simulator.websocket_sim.receive(
            "driveCommands", {"linear": 0.5, "angular": 0.2}, client_id
        )

        await asyncio.sleep(0.1)

        # Get firmware status
        firmware_status = simulator.firmware_sim.get_system_status()

        # Emit status to frontend
        status_data = {
            "battery": {"voltage": 24.0, "percentage": 90.0},
            "motors": {
                "active": firmware_status["num_motors"],
                "faulted": firmware_status["motors_faulted"],
            },
            "timestamp": time.time(),
        }

        await simulator.websocket_sim.emit("systemStatus", status_data, client_id)

        # Verify message was sent
        stats = simulator.websocket_sim.get_statistics()
        assert stats["stats"]["messages_sent"] > 0

        print("✅ Firmware to frontend feedback test passed")

    async def test_emergency_stop_propagation(self, perfect_simulator):
        """Test E-stop through all layers within 100ms."""
        print("\n=== Test: Emergency Stop Propagation ===")

        simulator = perfect_simulator

        # Run emergency stop scenario
        result = await simulator.run_scenario(ScenarioType.EMERGENCY_STOP)

        assert result.success, f"Emergency stop failed: {result.errors}"
        assert (
            result.duration_s < 0.1
        ), f"E-stop too slow: {result.duration_s*1000:.1f}ms > 100ms"

        # Verify all motors stopped
        system_status = simulator.firmware_sim.get_system_status()
        assert system_status["emergency_stop"] == True

        for motor_id in range(simulator.firmware_sim.num_motors):
            status = simulator.firmware_sim.get_motor_status(motor_id)
            assert status is not None
            assert (
                abs(status["velocity_actual"]) < 0.01
            ), f"Motor {motor_id} not stopped"

        print(f"✅ Emergency stop propagated in {result.duration_s*1000:.1f}ms")

    async def test_network_disconnect_recovery(self, simulator):
        """Test reconnection and state recovery."""
        print("\n=== Test: Network Disconnect Recovery ===")

        result = await simulator.run_scenario(ScenarioType.RECOVERY)

        assert result.success, f"Recovery failed: {result.errors}"

        # Verify client can reconnect
        stats = simulator.websocket_sim.get_statistics()
        assert stats["stats"]["connections"] >= 2, "Should have reconnected"

        print("✅ Network disconnect recovery test passed")

    async def test_can_bus_overload(self, simulator):
        """Test behavior under high message load."""
        print("\n=== Test: CAN Bus High Load ===")

        result = await simulator.run_scenario(ScenarioType.HIGH_LOAD)

        assert result.success, f"High load test failed: {result.errors}"

        # Check SLCAN statistics
        slcan_stats = simulator.slcan_sim.get_statistics()
        assert slcan_stats["success_rate"] > 0.9, "Success rate too low under load"

        print(
            f"✅ High load test passed with {slcan_stats['success_rate']*100:.1f}% success rate"
        )

    async def test_network_failure_handling(self, simulator):
        """Test network failure and degradation handling."""
        print("\n=== Test: Network Failure Handling ===")

        result = await simulator.run_scenario(ScenarioType.NETWORK_FAILURE)

        assert result.success, f"Network failure handling failed: {result.errors}"

        # Verify some messages got through despite high packet loss
        assert (
            result.messages_received > 0
        ), "No messages received during network failure"

        print("✅ Network failure handling test passed")

    async def test_firmware_fault_injection(self, simulator):
        """Test firmware fault injection and handling."""
        print("\n=== Test: Firmware Fault Injection ===")

        result = await simulator.run_scenario(ScenarioType.FIRMWARE_FAULT)

        assert result.success, f"Firmware fault handling failed: {result.errors}"

        print("✅ Firmware fault injection test passed")

    async def test_complete_roundtrip(self, perfect_simulator):
        """Test complete roundtrip: Frontend → Firmware → Frontend."""
        print("\n=== Test: Complete Roundtrip Communication ===")

        simulator = perfect_simulator

        # Connect client
        client_id = await simulator.websocket_sim.connect()

        # Send velocity command
        test_linear = 0.75
        test_angular = 0.3

        await simulator.websocket_sim.receive(
            "driveCommands", {"linear": test_linear, "angular": test_angular}, client_id
        )

        # Wait for processing
        await asyncio.sleep(0.05)

        # Get firmware state (simulates feedback)
        motor_status = simulator.firmware_sim.get_motor_status(0)
        assert motor_status is not None

        # Verify command propagated
        # Note: Wheel velocities will be different due to differential drive conversion
        assert abs(motor_status["velocity_setpoint"]) > 0.0

        # Send status back to frontend
        status_data = {
            "linear_velocity": test_linear,
            "angular_velocity": test_angular,
            "motors": [
                simulator.firmware_sim.get_motor_status(i)
                for i in range(simulator.firmware_sim.num_motors)
            ],
        }

        await simulator.websocket_sim.emit("systemStatus", status_data, client_id)

        # Verify message sent
        stats = simulator.websocket_sim.get_statistics()
        assert stats["stats"]["messages_sent"] > 0
        assert stats["stats"]["messages_received"] > 0

        print("✅ Complete roundtrip test passed")

    async def test_full_test_suite(self, simulator):
        """Run complete test suite."""
        print("\n=== Test: Full Test Suite ===")

        summary = await simulator.run_test_suite()

        assert summary["total_scenarios"] == 6
        assert summary["passed"] >= 5, f"Too many failures: {summary['failed']}/6"
        assert (
            summary["pass_rate"] >= 0.8
        ), f"Pass rate too low: {summary['pass_rate']*100:.1f}%"

        print(
            f"✅ Full test suite: {summary['passed']}/{summary['total_scenarios']} passed"
        )

    async def test_slcan_encoding_accuracy(self, perfect_simulator):
        """Test SLCAN encoding/decoding accuracy."""
        print("\n=== Test: SLCAN Encoding Accuracy ===")

        simulator = perfect_simulator

        # Test various velocity values
        test_cases = [
            (0.5, 0.0, 0.0),  # Forward only
            (0.0, 0.0, 0.2618),  # Rotate only (15 deg/s)
            (0.5, 0.25, 0.2618),  # Combined
            (-0.3, 0.0, -0.1),  # Backward + rotate
        ]

        for linear_x, linear_y, angular_z in test_cases:
            # Encode
            frame = simulator.slcan_sim.encode_velocity_command(
                linear_x, linear_y, angular_z
            )

            # Decode
            cmd = simulator.slcan_sim.decode_velocity_command(frame)

            assert cmd is not None, f"Failed to decode frame: {frame}"

            # Verify accuracy (allow small rounding error)
            assert (
                abs(cmd.linear_x - linear_x) < 0.001
            ), f"Linear X mismatch: {cmd.linear_x} != {linear_x}"
            assert (
                abs(cmd.linear_y - linear_y) < 0.001
            ), f"Linear Y mismatch: {cmd.linear_y} != {linear_y}"
            assert (
                abs(cmd.angular_z - angular_z) < 0.001
            ), f"Angular Z mismatch: {cmd.angular_z} != {angular_z}"

        print("✅ SLCAN encoding accuracy test passed")

    async def test_concurrent_clients(self, simulator):
        """Test multiple concurrent WebSocket clients."""
        print("\n=== Test: Concurrent Clients ===")

        # Connect multiple clients
        client_ids = []
        for i in range(3):
            client_id = await simulator.websocket_sim.connect()
            client_ids.append(client_id)

        # Each client sends commands
        for i, client_id in enumerate(client_ids):
            await simulator.websocket_sim.receive(
                "driveCommands", {"linear": 0.5 + i * 0.1, "angular": 0.0}, client_id
            )

        await asyncio.sleep(0.1)

        # Verify all clients connected
        stats = simulator.websocket_sim.get_statistics()
        assert stats["connected_clients"] == 3

        # Disconnect all
        for client_id in client_ids:
            await simulator.websocket_sim.disconnect(client_id)

        assert (
            stats["connected_clients"] == 0
            or len(simulator.websocket_sim.connected_clients) == 0
        )

        print("✅ Concurrent clients test passed")


# Main test runner
if __name__ == "__main__":
    import time

    print("=" * 60)
    print("Complete Communication Stack Integration Tests")
    print("=" * 60)

    async def run_all_tests():
        """Run all integration tests."""
        simulator = create_full_stack_simulator("default")
        perfect_sim = create_full_stack_simulator("perfect")

        test = TestCompleteCommunicationStack()

        try:
            # Run all tests
            await test.test_frontend_to_firmware_velocity_command(simulator)
            await test.test_firmware_to_frontend_status_feedback(simulator)
            await test.test_emergency_stop_propagation(perfect_sim)
            await test.test_network_disconnect_recovery(simulator)
            await test.test_can_bus_overload(simulator)
            await test.test_network_failure_handling(simulator)
            await test.test_firmware_fault_injection(simulator)
            await test.test_complete_roundtrip(perfect_sim)
            await test.test_slcan_encoding_accuracy(perfect_sim)
            await test.test_concurrent_clients(simulator)

            print("\n" + "=" * 60)
            print("✅ All tests passed!")
            print("=" * 60)

        finally:
            simulator.shutdown()
            perfect_sim.shutdown()

    asyncio.run(run_all_tests())
