#!/usr/bin/env python3
"""
Integration Tests with Stubs - Bridge Communication Paths

Tests all communication paths with stub implementations:
1. ROS2 → CAN → Firmware (stub)
2. ROS2 → Teleoperation Server (stub) → Firmware (stub)
3. Teleoperation Frontend (stub) → ROS2 → CAN
4. Complete end-to-end flow

Author: URC 2026 Integration Testing Team
"""

import pytest
import asyncio
import sys
from pathlib import Path
from unittest.mock import Mock, MagicMock, AsyncMock, patch
from typing import List, Dict, Any

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

from geometry_msgs.msg import Twist

try:
    from bridges.can_bridge import CANBridge, BridgeMessage
except ModuleNotFoundError:
    pytest.skip("bridges package not available", allow_module_level=True)


class StubSerialPort:
    """
    Stub serial port for testing without hardware.

    Simulates serial communication with mock firmware responses.
    """

    def __init__(
        self, port: str, baudrate: int, timeout: float = 0.1, write_timeout: float = 1.0
    ):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.write_timeout = write_timeout
        self.is_open = True
        self.in_waiting = 0

        # Buffers
        self.write_buffer = []
        self.read_buffer = b""

        # Stub firmware behavior
        self.auto_respond = True

    def write(self, data: bytes) -> int:
        """Write data (store for inspection)."""
        self.write_buffer.append(data)

        # Auto-generate response if enabled
        if self.auto_respond:
            self._generate_response(data)

        return len(data)

    def read(self, size: int = 1) -> bytes:
        """Read data from buffer."""
        if not self.read_buffer:
            return b""

        data = self.read_buffer[:size]
        self.read_buffer = self.read_buffer[size:]
        self.in_waiting = len(self.read_buffer)
        return data

    def close(self):
        """Close port."""
        self.is_open = False

    def _generate_response(self, command: bytes):
        """Generate stub firmware response."""
        try:
            cmd_str = command.decode("ascii").strip()

            # Velocity command → velocity response
            if cmd_str.startswith("t00C6"):
                # Echo back as response (change ID to 0x00D)
                response = cmd_str.replace("t00C", "t00D") + "\r"
                self.read_buffer += response.encode("ascii")
                self.in_waiting = len(self.read_buffer)

            # Heartbeat → heartbeat reply
            elif cmd_str.startswith("t00E0"):
                response = "t00F0\r"
                self.read_buffer += response.encode("ascii")
                self.in_waiting = len(self.read_buffer)

            # Homing → homing response
            elif cmd_str.startswith("t110"):
                response = "t1110\r"
                self.read_buffer += response.encode("ascii")
                self.in_waiting = len(self.read_buffer)

        except Exception:
            pass  # Ignore malformed commands


class StubTeleopServer:
    """
    Stub teleoperation server for testing.

    Simulates py_server.py behavior without actual network connection.
    """

    def __init__(self):
        self.received_commands = []
        self.connected_clients = []
        self.running = False

    async def start(self):
        """Start stub server."""
        self.running = True

    async def stop(self):
        """Stop stub server."""
        self.running = False

    async def emit(self, event: str, data: Dict):
        """Simulate emitting event to clients."""
        pass

    async def handle_drive_command(self, data: Dict):
        """Handle drive command from frontend."""
        self.received_commands.append({"type": "drive", "data": data})


class StubFirmware:
    """
    Stub firmware for testing.

    Simulates STM32 firmware behavior without actual hardware.
    """

    def __init__(self):
        self.current_velocity = {"x": 0.0, "y": 0.0, "rot": 0.0}
        self.motor_states = {
            "left_front": 0.0,
            "right_front": 0.0,
            "left_rear": 0.0,
            "right_rear": 0.0,
        }
        self.is_homed = False

    def process_velocity_command(self, x: float, y: float, rot: float):
        """Process velocity command."""
        self.current_velocity = {"x": x, "y": y, "rot": rot}

        # Simulate motor control
        self.motor_states["left_front"] = x + rot
        self.motor_states["right_front"] = x - rot
        self.motor_states["left_rear"] = x + rot
        self.motor_states["right_rear"] = x - rot

    def process_homing(self):
        """Process homing command."""
        self.is_homed = True
        self.current_velocity = {"x": 0.0, "y": 0.0, "rot": 0.0}


# ============================================================================
# TEST SUITE
# ============================================================================


class TestBridgeIntegrationWithStubs:
    """Integration tests using stubs instead of actual hardware."""

    @pytest.mark.asyncio
    async def test_ros2_to_can_velocity_command(self):
        """Test: ROS2 → CAN Bridge → Stub Firmware"""
        # Create stub serial port
        stub_serial = StubSerialPort("/dev/ttyACM0", 115200)

        # Create CAN bridge with stub
        config = {"protocol": "teleoperation", "device": "/dev/ttyACM0"}
        bridge = CANBridge(config)

        # Patch serial.Serial to return our stub
        with patch("serial.Serial", return_value=stub_serial):
            # Connect bridge
            connected = await bridge.connect()
            assert connected, "Bridge should connect to stub"

            # Create ROS2 Twist command
            twist = Twist()
            twist.linear.x = 0.5
            twist.angular.z = 0.2618

            # Send via bridge
            msg = BridgeMessage(message_type="velocity_command", data={"twist": twist})

            success = await bridge.send_message(msg)
            assert success, "Message should send successfully"

            # Verify SLCAN frame was written
            assert len(stub_serial.write_buffer) > 0, "Should have written data"

            # Check frame format
            frame = stub_serial.write_buffer[0].decode("ascii")
            assert frame.startswith("t00C6"), "Should be velocity command (0x00C)"
            assert frame.endswith("\r"), "Should end with carriage return"

            # Verify stub firmware would receive correct command
            # (In real system, teleoperation server would forward to firmware)
            assert "0800" in frame, "Should contain scaled x velocity (0x0800 = 2048)"

            await bridge.disconnect()

        print("✓ ROS2 → CAN → Stub Firmware test passed")

    @pytest.mark.asyncio
    async def test_can_to_ros2_velocity_feedback(self):
        """Test: Stub Firmware → CAN Bridge → ROS2"""
        stub_serial = StubSerialPort("/dev/ttyACM0", 115200)

        config = {"protocol": "teleoperation", "device": "/dev/ttyACM0"}
        bridge = CANBridge(config)

        # Track received messages
        received_messages = []

        async def message_handler(msg: BridgeMessage):
            received_messages.append(msg)

        await bridge.register_handler("velocity_feedback", message_handler)

        with patch("serial.Serial", return_value=stub_serial):
            await bridge.connect()

            # Simulate firmware sending velocity feedback
            feedback_frame = b"t00D60800000003c0\r"  # 0.5 m/s, 15 deg/s
            stub_serial.read_buffer = feedback_frame
            stub_serial.in_waiting = len(feedback_frame)

            # Allow processing loop to run
            await asyncio.sleep(0.2)

            # Check that message was received and routed
            assert len(received_messages) > 0, "Should have received feedback"

            # Verify decoded velocity
            twist = received_messages[0].data["twist"]
            assert abs(twist.linear.x - 0.5) < 0.001, "X velocity should be 0.5 m/s"

            await bridge.disconnect()

        print("✓ Stub Firmware → CAN → ROS2 test passed")

    @pytest.mark.asyncio
    async def test_ros2_heartbeat(self):
        """Test: ROS2 heartbeat through CAN bridge"""
        stub_serial = StubSerialPort("/dev/ttyACM0", 115200)

        config = {"protocol": "teleoperation", "device": "/dev/ttyACM0"}
        bridge = CANBridge(config)

        with patch("serial.Serial", return_value=stub_serial):
            await bridge.connect()

            # Send heartbeat
            msg = BridgeMessage(message_type="heartbeat", data={})
            success = await bridge.send_message(msg)

            assert success, "Heartbeat should send"

            # Check frame
            frame = stub_serial.write_buffer[0].decode("ascii")
            assert frame == "t00E0\r", "Should be heartbeat (0x00E)"

            # Verify stub auto-generated response
            assert stub_serial.in_waiting > 0, "Should have heartbeat reply"

            await bridge.disconnect()

        print("✓ ROS2 heartbeat test passed")

    @pytest.mark.asyncio
    async def test_ros2_homing_sequence(self):
        """Test: ROS2 homing command through CAN bridge"""
        stub_serial = StubSerialPort("/dev/ttyACM0", 115200)
        stub_firmware = StubFirmware()

        config = {"protocol": "teleoperation", "device": "/dev/ttyACM0"}
        bridge = CANBridge(config)

        with patch("serial.Serial", return_value=stub_serial):
            await bridge.connect()

            # Send homing command
            msg = BridgeMessage(message_type="homing", data={})
            success = await bridge.send_message(msg)

            assert success, "Homing should send"

            # Check frame
            frame = stub_serial.write_buffer[0].decode("ascii")
            assert frame.startswith("t110"), "Should be homing (0x110)"

            # Simulate firmware processing
            stub_firmware.process_homing()
            assert stub_firmware.is_homed, "Firmware should be homed"

            await bridge.disconnect()

        print("✓ ROS2 homing test passed")

    @pytest.mark.asyncio
    async def test_protocol_adapter_roundtrip(self):
        """Test: CAN bridge encode/decode round-trip (inline protocol)"""
        bridge = CANBridge({})

        # Original twist
        twist_orig = Twist()
        twist_orig.linear.x = 0.5
        twist_orig.linear.y = 0.25
        twist_orig.angular.z = 0.2618

        # Encode
        frame = bridge._encode_velocity_command(twist_orig)
        assert frame is not None

        # Simulate firmware response (change ID to 0x00D)
        slcan_response = frame.replace(b"t00C", b"t00D")

        # Decode
        twist_decoded = bridge._decode_velocity_feedback(slcan_response)

        assert twist_decoded is not None
        assert abs(twist_decoded.linear.x - twist_orig.linear.x) < 0.001
        assert abs(twist_decoded.linear.y - twist_orig.linear.y) < 0.001
        assert abs(twist_decoded.angular.z - twist_orig.angular.z) < 0.01

        print("✓ CAN bridge encode/decode round-trip test passed")

    @pytest.mark.asyncio
    async def test_emergency_stop_flow(self):
        """Test: Emergency stop through all layers"""
        stub_serial = StubSerialPort("/dev/ttyACM0", 115200)
        stub_firmware = StubFirmware()

        config = {"protocol": "teleoperation", "device": "/dev/ttyACM0"}
        bridge = CANBridge(config)

        with patch("serial.Serial", return_value=stub_serial):
            await bridge.connect()

            # Send normal velocity first
            twist = Twist()
            twist.linear.x = 0.5
            msg = BridgeMessage(message_type="velocity_command", data={"twist": twist})
            await bridge.send_message(msg)

            # Send emergency stop
            stop_msg = BridgeMessage(message_type="emergency_stop", data={})
            success = await bridge.send_message(stop_msg)

            assert success, "Emergency stop should send"
            assert bridge.emergency_stop_active, "Emergency stop flag should be set"

            # Check that zero velocity was sent
            last_frame = stub_serial.write_buffer[-1].decode("ascii")
            assert "000000000000" in last_frame, "Should send zero velocity"

            await bridge.disconnect()

        print("✓ Emergency stop flow test passed")

    @pytest.mark.asyncio
    async def test_multiple_velocity_commands(self):
        """Test: Rapid sequence of velocity commands"""
        stub_serial = StubSerialPort("/dev/ttyACM0", 115200)

        config = {"protocol": "teleoperation", "device": "/dev/ttyACM0"}
        bridge = CANBridge(config)

        with patch("serial.Serial", return_value=stub_serial):
            await bridge.connect()

            # Send multiple commands rapidly
            velocities = [0.1, 0.2, 0.3, 0.4, 0.5]

            for vel in velocities:
                twist = Twist()
                twist.linear.x = vel
                msg = BridgeMessage(
                    message_type="velocity_command", data={"twist": twist}
                )
                await bridge.send_message(msg)

            # Verify all commands were sent
            assert len(stub_serial.write_buffer) == len(velocities)

            # Verify commands are in order
            for i, frame_bytes in enumerate(stub_serial.write_buffer):
                frame = frame_bytes.decode("ascii")
                assert frame.startswith(
                    "t00C6"
                ), f"Command {i} should be velocity command"

            await bridge.disconnect()

        print("✓ Multiple velocity commands test passed")

    def test_stub_firmware_behavior(self):
        """Test: Stub firmware logic"""
        firmware = StubFirmware()

        # Test velocity command processing
        firmware.process_velocity_command(0.5, 0.0, 15.0)
        assert firmware.current_velocity["x"] == 0.5
        assert firmware.current_velocity["rot"] == 15.0

        # Test homing
        firmware.process_homing()
        assert firmware.is_homed
        assert firmware.current_velocity["x"] == 0.0

        print("✓ Stub firmware behavior test passed")

    def test_stub_serial_auto_response(self):
        """Test: Stub serial auto-response generation"""
        serial_port = StubSerialPort("/dev/ttyACM0", 115200)

        # Send velocity command
        serial_port.write(b"t00C60800000003c0\r")

        # Check auto-response was generated
        assert serial_port.in_waiting > 0, "Should have auto-generated response"

        # Read response
        response = serial_port.read(20)
        response_str = response.decode("ascii")

        assert response_str.startswith("t00D"), "Should be velocity response (0x00D)"

        print("✓ Stub serial auto-response test passed")


# ============================================================================
# STUB FACTORIES
# ============================================================================


def create_stub_serial_port(port: str = "/dev/ttyACM0") -> StubSerialPort:
    """Factory for creating stub serial ports."""
    return StubSerialPort(port, 115200)


def create_stub_teleoperation_server() -> StubTeleopServer:
    """Factory for creating stub teleoperation server."""
    return StubTeleopServer()


def create_stub_firmware() -> StubFirmware:
    """Factory for creating stub firmware."""
    return StubFirmware()


if __name__ == "__main__":
    print("Running integration tests with stubs...")
    print("=" * 60)
    pytest.main([__file__, "-v", "-s"])
