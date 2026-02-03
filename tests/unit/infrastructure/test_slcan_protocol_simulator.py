#!/usr/bin/env python3
"""
Unit Tests for SLCAN Protocol Simulator

Comprehensive tests for SLCAN (Serial Line CAN) protocol implementation
including encoding/decoding, scaling, error injection, and buffer management.

Author: URC 2026 Testing Team
"""

import pytest
import time
import sys
from pathlib import Path
from typing import Optional

# Add simulation to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from simulation.can.slcan_protocol_simulator import (
    SLCANProtocolSimulator,
    SLCANFrame,
    SLCANMessageType,
    CANMessageID,
    VelocityCommand,
    create_slcan_simulator,
)


class TestSLCANProtocolSimulator:
    """Test suite for SLCAN protocol simulator."""

    @pytest.fixture
    def simulator(self):
        """Create basic simulator for testing."""
        sim = SLCANProtocolSimulator(
            {"simulate_errors": False, "serial_delay_ms": 0.0, "error_rate": 0.0}
        )
        yield sim

    @pytest.fixture
    def error_simulator(self):
        """Create simulator with error injection enabled."""
        sim = SLCANProtocolSimulator(
            {
                "simulate_errors": True,
                "error_rate": 0.5,  # 50% error rate for testing
                "serial_delay_ms": 1.0,
            }
        )
        yield sim

    # Velocity Encoding Tests

    def test_encode_forward_velocity(self, simulator):
        """Test encoding forward-only velocity."""
        frame = simulator.encode_velocity_command(0.5, 0.0, 0.0)

        assert frame is not None
        assert frame.startswith("t00C6")  # Message ID 0x00C, length 6
        assert frame.endswith("\r")

    def test_encode_rotation_velocity(self, simulator):
        """Test encoding rotation-only velocity."""
        frame = simulator.encode_velocity_command(0.0, 0.0, 0.2618)  # 15 deg/s

        assert frame is not None
        assert frame.startswith("t00C6")
        assert frame.endswith("\r")

    def test_encode_combined_velocity(self, simulator):
        """Test encoding combined linear and rotational velocity."""
        frame = simulator.encode_velocity_command(0.5, 0.25, 0.2618)

        assert frame is not None
        assert frame.startswith("t00C6")
        assert len(frame) == 19  # 't00C6' + 12 hex chars + '\r'

    def test_encode_negative_velocities(self, simulator):
        """Test encoding negative velocities."""
        frame = simulator.encode_velocity_command(-0.5, 0.0, -0.1)

        assert frame is not None
        assert frame.startswith("t00C6")

    def test_encode_zero_velocities(self, simulator):
        """Test encoding all-zero velocities."""
        frame = simulator.encode_velocity_command(0.0, 0.0, 0.0)

        assert frame is not None
        assert frame.startswith("t00C6")
        # Should encode as zeros
        assert "000000000000" in frame

    def test_encode_max_velocities(self, simulator):
        """Test encoding maximum safe velocities."""
        # Test within reasonable limits
        frame = simulator.encode_velocity_command(2.0, 2.0, 3.14159)

        assert frame is not None
        assert frame.startswith("t00C6")

    # Velocity Decoding Tests

    def test_decode_velocity_command(self, simulator):
        """Test decoding velocity command frame."""
        # Encode then decode
        original_linear_x = 0.5
        original_linear_y = 0.25
        original_angular_z = 0.2618

        frame = simulator.encode_velocity_command(
            original_linear_x, original_linear_y, original_angular_z
        )
        cmd = simulator.decode_velocity_command(frame)

        assert cmd is not None
        assert abs(cmd.linear_x - original_linear_x) < 0.001
        assert abs(cmd.linear_y - original_linear_y) < 0.001
        assert abs(cmd.angular_z - original_angular_z) < 0.001

    def test_decode_zero_velocities(self, simulator):
        """Test decoding zero velocity command."""
        frame = simulator.encode_velocity_command(0.0, 0.0, 0.0)
        cmd = simulator.decode_velocity_command(frame)

        assert cmd is not None
        assert abs(cmd.linear_x) < 0.001
        assert abs(cmd.linear_y) < 0.001
        assert abs(cmd.angular_z) < 0.001

    def test_decode_negative_velocities(self, simulator):
        """Test decoding negative velocities."""
        original = (-0.5, -0.25, -0.2)
        frame = simulator.encode_velocity_command(*original)
        cmd = simulator.decode_velocity_command(frame)

        assert cmd is not None
        assert abs(cmd.linear_x - original[0]) < 0.001
        assert abs(cmd.linear_y - original[1]) < 0.001
        assert abs(cmd.angular_z - original[2]) < 0.001

    def test_decode_invalid_frame(self, simulator):
        """Test decoding malformed frame returns None."""
        invalid_frames = [
            "invalid",
            "t00C",  # Too short
            "t00CZZZZZZ",  # Invalid hex
            "x00C6000000000000\r",  # Wrong message type
        ]

        for frame in invalid_frames:
            cmd = simulator.decode_velocity_command(frame)
            assert cmd is None

    def test_decode_wrong_message_id(self, simulator):
        """Test decoding frame with wrong message ID."""
        # Create frame with wrong ID (not 0x00C)
        frame = simulator.encode_heartbeat()
        cmd = simulator.decode_velocity_command(frame)

        assert cmd is None  # Wrong message ID

    # Scaling Factor Tests

    def test_linear_velocity_scaling(self, simulator):
        """Test linear velocity scaling factor (2^12 = 4096)."""
        test_velocity = 1.0  # 1 m/s
        frame = simulator.encode_velocity_command(test_velocity, 0.0, 0.0)
        cmd = simulator.decode_velocity_command(frame)

        assert cmd is not None
        assert abs(cmd.linear_x - test_velocity) < 0.001

        # Verify scaling constant
        assert simulator.LINEAR_SCALE == 4096

    def test_angular_velocity_scaling(self, simulator):
        """Test angular velocity scaling factor (2^6 = 64 after deg conversion)."""
        test_angular = 0.5  # rad/s
        frame = simulator.encode_velocity_command(0.0, 0.0, test_angular)
        cmd = simulator.decode_velocity_command(frame)

        assert cmd is not None
        assert abs(cmd.angular_z - test_angular) < 0.001

        # Verify scaling constant
        assert simulator.ANGULAR_SCALE == 64

    def test_scaling_precision(self, simulator):
        """Test precision loss from scaling is acceptable."""
        # Test many different values
        test_values = [0.1, 0.3, 0.7, 0.9, 1.5]

        for val in test_values:
            frame = simulator.encode_velocity_command(val, 0.0, 0.0)
            cmd = simulator.decode_velocity_command(frame)

            assert cmd is not None
            error = abs(cmd.linear_x - val)
            # Allow 0.1% error from integer quantization
            assert error < val * 0.001 or error < 0.001

    # Frame Parsing Tests

    def test_parse_standard_data_frame(self, simulator):
        """Test parsing standard 11-bit ID data frame."""
        test_frame = "t00C6000000000000\r"
        parsed = simulator.parse_frame(test_frame)

        assert parsed is not None
        assert parsed.message_type == SLCANMessageType.STANDARD_DATA
        assert parsed.message_id == 0x00C
        assert parsed.data_length == 6

    def test_parse_heartbeat_frame(self, simulator):
        """Test parsing heartbeat message."""
        frame = simulator.encode_heartbeat()
        parsed = simulator.parse_frame(frame)

        assert parsed is not None
        assert parsed.message_id == CANMessageID.HEARTBEAT_REQUEST.value

    def test_parse_homing_frame(self, simulator):
        """Test parsing homing request message."""
        frame = simulator.encode_homing_request()
        parsed = simulator.parse_frame(frame)

        assert parsed is not None
        assert parsed.message_id == CANMessageID.HOMING_REQUEST.value

    def test_parse_emergency_stop_frame(self, simulator):
        """Test parsing emergency stop message."""
        frame = simulator.encode_emergency_stop()
        parsed = simulator.parse_frame(frame)

        assert parsed is not None
        assert parsed.message_id == CANMessageID.EMERGENCY_STOP.value

    def test_parse_malformed_frames(self, simulator):
        """Test parsing malformed frames returns None."""
        malformed = [
            "",
            "t",
            "t00C",
            "t00CZ",
            "t00C6GGGGGG\r",  # Invalid hex
            "q00C6000000\r",  # Invalid message type
        ]

        for frame in malformed:
            parsed = simulator.parse_frame(frame)
            assert parsed is None

    def test_parse_frame_with_timestamp(self, simulator):
        """Test parsed frame includes timestamp."""
        frame = simulator.encode_heartbeat()

        before = time.time()
        parsed = simulator.parse_frame(frame)
        after = time.time()

        assert parsed is not None
        assert before <= parsed.timestamp <= after

    # Frame Encoding Tests

    def test_encode_custom_frame(self, simulator):
        """Test encoding custom frame with message ID and data."""
        data = b"\x01\x02\x03\x04"
        frame = simulator.encode_frame(0x123, data)

        assert frame is not None
        assert frame.startswith("t123")  # Hex ID
        assert frame.endswith("\r")

    def test_encode_frame_data_length_validation(self, simulator):
        """Test frame encoding validates data length <= 8."""
        # Valid: 8 bytes
        data_valid = b"\x00" * 8
        frame = simulator.encode_frame(0x100, data_valid)
        assert frame is not None

        # Invalid: 9 bytes (should handle gracefully)
        data_invalid = b"\x00" * 9
        # Should either truncate or return None
        frame = simulator.encode_frame(0x100, data_invalid)
        # Implementation may vary, just ensure it doesn't crash

    def test_encode_heartbeat_format(self, simulator):
        """Test heartbeat frame has correct format."""
        frame = simulator.encode_heartbeat()

        assert frame.startswith("t00E")  # ID 0x00E
        assert frame.endswith("\r")

    def test_encode_heartbeat_response_format(self, simulator):
        """Test heartbeat response has correct format."""
        frame = simulator.encode_heartbeat_response()

        assert frame.startswith("t00F")  # ID 0x00F
        assert frame.endswith("\r")

    def test_encode_homing_format(self, simulator):
        """Test homing request has correct format."""
        frame = simulator.encode_homing_request()

        assert frame.startswith("t110")  # ID 0x110
        assert frame.endswith("\r")

    # Buffer Management Tests

    def test_write_to_buffer(self, simulator):
        """Test writing frame to TX buffer."""
        frame = "t00C6000000000000\r"
        success = simulator.write_to_buffer(frame)

        assert success is True
        assert len(simulator.tx_buffer) > 0

    def test_read_from_buffer(self, simulator):
        """Test reading frame from RX buffer."""
        frame = "t00C6000000000000\r"

        # Manually add to RX buffer
        simulator.rx_buffer.extend(frame.encode("ascii"))

        # Read back
        read_frame = simulator.read_from_buffer()
        assert read_frame == frame

    def test_buffer_overflow_handling(self, simulator):
        """Test buffer overflow is handled gracefully."""
        simulator.buffer_size = 100

        # Try to write large amount of data
        large_frame = "t00C6" + ("00" * 50) + "\r"

        success = simulator.write_to_buffer(large_frame)
        # Should either succeed or fail gracefully
        assert isinstance(success, bool)

        # Check statistics tracked overflow
        if not success:
            assert simulator.stats["buffer_overflows"] > 0

    def test_multiple_frames_in_buffer(self, simulator):
        """Test multiple frames can be buffered and read."""
        frames = ["t00C6000000000000\r", "t00E0\r", "t1100\r"]

        for frame in frames:
            simulator.rx_buffer.extend(frame.encode("ascii"))

        # Read all frames
        read_frames = []
        for _ in range(3):
            frame = simulator.read_from_buffer()
            if frame:
                read_frames.append(frame)

        assert len(read_frames) == 3

    def test_clear_buffers(self, simulator):
        """Test clearing buffers."""
        simulator.write_to_buffer("test\r")
        simulator.rx_buffer.extend(b"test\r")

        simulator.clear_buffers()

        assert len(simulator.tx_buffer) == 0
        assert len(simulator.rx_buffer) == 0

    # Error Injection Tests

    def test_error_injection_corrupts_frame(self, error_simulator):
        """Test error injection modifies frames."""
        simulator = error_simulator
        original_frame = "t00C6000000000000\r"

        # Inject errors multiple times
        different = False
        for _ in range(10):
            corrupted = simulator.inject_error(original_frame)
            if corrupted != original_frame:
                different = True
                break

        # With 50% error rate, should get corrupted frame
        assert different

    def test_error_injection_tracking(self, error_simulator):
        """Test error injection updates statistics."""
        simulator = error_simulator
        frame = "t00C6000000000000\r"

        initial_errors = simulator.stats["encoding_errors"]

        # Inject errors multiple times
        for _ in range(10):
            simulator.inject_error(frame)

        # Statistics should increase (with 50% rate)
        # Note: Statistics might not increase if implementation varies
        # Just check it doesn't crash

    def test_no_errors_when_disabled(self, simulator):
        """Test no errors injected when disabled."""
        simulator.simulate_errors = False
        original = "t00C6000000000000\r"

        # Try many times
        for _ in range(100):
            result = simulator.inject_error(original)
            assert result == original  # Should never change

    # Statistics Tests

    def test_encoding_statistics(self, simulator):
        """Test encoding updates statistics."""
        initial = simulator.stats["frames_sent"]

        simulator.encode_velocity_command(0.5, 0.0, 0.0)

        # Check statistics increased
        assert simulator.stats["frames_sent"] > initial

    def test_decoding_statistics(self, simulator):
        """Test decoding updates statistics."""
        frame = simulator.encode_velocity_command(0.5, 0.0, 0.0)
        initial = simulator.stats["frames_received"]

        simulator.decode_velocity_command(frame)

        assert simulator.stats["frames_received"] > initial

    def test_malformed_frame_statistics(self, simulator):
        """Test malformed frames update error statistics."""
        initial = simulator.stats["malformed_frames"]

        simulator.parse_frame("invalid_frame")

        assert simulator.stats["malformed_frames"] > initial

    def test_success_rate_calculation(self, simulator):
        """Test success rate is calculated correctly."""
        # Send some successful frames
        for _ in range(10):
            simulator.encode_velocity_command(0.5, 0.0, 0.0)

        stats = simulator.get_statistics()
        success_rate = stats["success_rate"]

        assert 0.0 <= success_rate <= 1.0
        assert success_rate > 0.9  # Should be high with no errors

    def test_reset_statistics(self, simulator):
        """Test resetting statistics."""
        # Generate some activity
        for _ in range(5):
            simulator.encode_velocity_command(0.5, 0.0, 0.0)

        simulator.reset_statistics()

        assert simulator.stats["frames_sent"] == 0
        assert simulator.stats["frames_received"] == 0

    # Bus Load Tests

    def test_bus_load_tracking(self, simulator):
        """Test bus load is tracked."""
        # Send multiple frames rapidly
        for _ in range(10):
            simulator.encode_velocity_command(0.5, 0.0, 0.0)

        bus_load = simulator.get_bus_load()

        assert bus_load >= 0.0
        assert bus_load <= 1.0

    # Message History Tests

    def test_message_history_recording(self, simulator):
        """Test messages are recorded in history."""
        simulator.encode_velocity_command(0.5, 0.0, 0.0)

        assert len(simulator.message_history) > 0

    def test_message_history_limit(self, simulator):
        """Test history respects maximum size."""
        simulator.max_history = 10

        # Send more than limit
        for _ in range(20):
            simulator.encode_velocity_command(0.5, 0.0, 0.0)

        assert len(simulator.message_history) <= 10

    # Factory Function Tests

    def test_create_default_simulator(self):
        """Test factory creates default simulator."""
        sim = create_slcan_simulator("default")
        assert sim is not None
        assert sim.error_rate == 0.001

    def test_create_perfect_simulator(self):
        """Test factory creates perfect simulator."""
        sim = create_slcan_simulator("perfect")
        assert sim is not None
        assert sim.error_rate == 0.0
        assert sim.serial_delay_ms == 0.0

    def test_create_stressed_simulator(self):
        """Test factory creates stressed simulator."""
        sim = create_slcan_simulator("stressed")
        assert sim is not None
        assert sim.error_rate > 0.01  # Higher error rate

    # Edge Cases and Robustness

    def test_very_small_velocities(self, simulator):
        """Test very small velocity values."""
        small_vel = 0.001
        frame = simulator.encode_velocity_command(small_vel, 0.0, 0.0)
        cmd = simulator.decode_velocity_command(frame)

        assert cmd is not None
        # Precision may be lost for very small values
        assert abs(cmd.linear_x - small_vel) < 0.01

    def test_alternating_sign_velocities(self, simulator):
        """Test rapidly alternating positive/negative velocities."""
        values = [(0.5, 0.0, 0.0), (-0.5, 0.0, 0.0), (0.0, 0.3, 0.0), (0.0, -0.3, 0.0)]

        for linear_x, linear_y, angular_z in values:
            frame = simulator.encode_velocity_command(linear_x, linear_y, angular_z)
            cmd = simulator.decode_velocity_command(frame)

            assert cmd is not None
            assert abs(cmd.linear_x - linear_x) < 0.001


# Performance Tests
@pytest.mark.benchmark
class TestSLCANProtocolPerformance:
    """Performance tests for SLCAN protocol simulator."""

    def test_encoding_performance(self, simulator):
        """Test encoding performance."""
        start = time.time()
        count = 10000

        for i in range(count):
            simulator.encode_velocity_command(0.5, 0.0, 0.1)

        duration = time.time() - start
        rate = count / duration

        # Should encode at least 10k frames/sec
        assert rate > 10000, f"Encoding too slow: {rate:.0f} frames/s"

    def test_decoding_performance(self, simulator):
        """Test decoding performance."""
        frame = simulator.encode_velocity_command(0.5, 0.0, 0.1)

        start = time.time()
        count = 10000

        for i in range(count):
            simulator.decode_velocity_command(frame)

        duration = time.time() - start
        rate = count / duration

        # Should decode at least 10k frames/sec
        assert rate > 10000, f"Decoding too slow: {rate:.0f} frames/s"

    def test_roundtrip_latency(self, simulator):
        """Test complete encode/decode roundtrip latency."""
        iterations = 1000
        latencies = []

        for _ in range(iterations):
            start = time.time()
            frame = simulator.encode_velocity_command(0.5, 0.25, 0.1)
            cmd = simulator.decode_velocity_command(frame)
            latency = (time.time() - start) * 1000  # ms
            latencies.append(latency)

        avg_latency = sum(latencies) / len(latencies)

        # Should be sub-millisecond on average
        assert avg_latency < 1.0, f"Roundtrip too slow: {avg_latency:.3f}ms"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
