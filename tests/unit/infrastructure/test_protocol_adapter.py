#!/usr/bin/env python3
"""
Unit Tests for CAN Bridge Protocol (Inline Encoding/Decoding)

Tests encoding/decoding accuracy and edge cases for the CAN bridge's
inline SLCAN/teleoperation protocol. Replaces tests for removed
TeleopCANAdapter and VelocityScaler.

Author: URC 2026 Testing Team
"""

import pytest
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

# CAN bridge and encoding tests require ROS2 geometry_msgs
pytest.importorskip("geometry_msgs")
from geometry_msgs.msg import Twist

from bridges import can_bridge
from bridges.can_bridge import CANBridge


class TestCanBridgeScaling:
    """Test velocity scaling/descaling (inline in can_bridge)."""

    def test_linear_scaling(self):
        scaled = can_bridge._scale_linear(0.5, 4096)
        assert scaled == 2048
        scaled = can_bridge._scale_linear(0.0, 4096)
        assert scaled == 0
        scaled = can_bridge._scale_linear(-0.5, 4096)
        assert scaled == -2048

    def test_linear_descaling(self):
        descaled = can_bridge._descale_linear(2048, 4096)
        assert abs(descaled - 0.5) < 0.001
        descaled = can_bridge._descale_linear(0, 4096)
        assert abs(descaled) < 0.001

    def test_angular_scaling_deg(self):
        scaled = can_bridge._scale_angular_deg(0.2618, 64)
        assert 950 <= scaled <= 970
        scaled = can_bridge._scale_angular_deg(0.0, 64)
        assert scaled == 0

    def test_angular_descaling_deg(self):
        descaled = can_bridge._descale_angular_deg(960, 64)
        assert abs(descaled - 0.2618) < 0.001

    def test_scaling_limits(self):
        scaled = can_bridge._scale_linear(10.0, 4096)
        assert scaled == 32767
        scaled = can_bridge._scale_linear(-10.0, 4096)
        assert scaled == -32768


class TestCanBridgeEncoding:
    """Test CAN bridge encoding/decoding (inline protocol)."""

    def setup_method(self):
        self.bridge = CANBridge({})

    def test_encode_forward_velocity(self):
        twist = Twist()
        twist.linear.x = 0.5
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        frame = self.bridge._encode_velocity_command(twist)
        assert frame is not None
        slcan = frame.decode("ascii")
        assert slcan.startswith("t00C6")
        assert slcan.endswith("\r")

    def test_encode_rotation(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.2618
        frame = self.bridge._encode_velocity_command(twist)
        assert frame is not None

    def test_decode_velocity_response(self):
        slcan_frame = b"t00D60800000003c0\r"
        twist = self.bridge._decode_velocity_feedback(slcan_frame)
        assert twist is not None
        assert abs(twist.linear.x - 0.5) < 0.001
        assert abs(twist.angular.z - 0.2618) < 0.01

    def test_decode_invalid_frame(self):
        assert self.bridge._decode_velocity_feedback(b"t00D6\r") is None
        assert self.bridge._decode_velocity_feedback(b"invalid") is None

    def test_encode_heartbeat(self):
        frame = self.bridge._encode_heartbeat()
        assert frame == b"t00E0\r"

    def test_encode_homing(self):
        frame = self.bridge._encode_homing()
        assert frame is not None
        assert frame.startswith(b"t110")

    def test_encode_sensor_request(self):
        frame = self.bridge._encode_sensor_request("velocity")
        assert frame == b"t1140\r"
        frame = self.bridge._encode_sensor_request("offset")
        assert frame == b"t1120\r"
        assert self.bridge._encode_sensor_request("unknown") is None

    def test_zero_velocity(self):
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        frame = self.bridge._encode_velocity_command(twist)
        assert frame is not None
        assert b"000000000000" in frame

    def test_get_status(self):
        status = self.bridge.get_status()
        assert status.is_connected is False
        assert status.messages_sent == 0
        assert status.messages_received == 0
        assert "device_path" in status.additional_info


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
