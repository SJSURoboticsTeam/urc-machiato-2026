#!/usr/bin/env python3
"""
Unit Tests for Protocol Adapters

Tests encoding/decoding accuracy and edge cases for protocol adapters.

Author: URC 2026 Testing Team
"""

import pytest
import sys
from pathlib import Path

# Add src to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

from geometry_msgs.msg import Twist
from bridges.teleop_can_adapter import TeleopCANAdapter
from bridges.protocol_adapter import VelocityScaler


class TestVelocityScaler:
    """Test velocity scaling/descaling operations."""
    
    def test_linear_scaling(self):
        """Test linear velocity scaling."""
        scaler = VelocityScaler()
        
        # Test positive velocity
        scaled = scaler.scale_linear(0.5, 4096)
        assert scaled == 2048
        
        # Test zero
        scaled = scaler.scale_linear(0.0, 4096)
        assert scaled == 0
        
        # Test negative velocity
        scaled = scaler.scale_linear(-0.5, 4096)
        assert scaled == -2048
    
    def test_linear_descaling(self):
        """Test linear velocity descaling."""
        scaler = VelocityScaler()
        
        # Test positive
        descaled = scaler.descale_linear(2048, 4096)
        assert abs(descaled - 0.5) < 0.001
        
        # Test zero
        descaled = scaler.descale_linear(0, 4096)
        assert abs(descaled) < 0.001
        
        # Test negative
        descaled = scaler.descale_linear(-2048, 4096)
        assert abs(descaled - (-0.5)) < 0.001
    
    def test_angular_scaling_deg(self):
        """Test angular velocity scaling (rad/s → scaled deg/s)."""
        scaler = VelocityScaler()
        
        # Test 15 deg/s = 0.2618 rad/s
        scaled = scaler.scale_angular_deg(0.2618, 64)
        assert 950 <= scaled <= 970  # Allow some rounding error
        
        # Test zero
        scaled = scaler.scale_angular_deg(0.0, 64)
        assert scaled == 0
        
        # Test negative
        scaled = scaler.scale_angular_deg(-0.2618, 64)
        assert -970 <= scaled <= -950
    
    def test_angular_descaling_deg(self):
        """Test angular velocity descaling (scaled deg/s → rad/s)."""
        scaler = VelocityScaler()
        
        # Test 960 scaled = 15 deg/s = 0.2618 rad/s
        descaled = scaler.descale_angular_deg(960, 64)
        assert abs(descaled - 0.2618) < 0.001
        
        # Test zero
        descaled = scaler.descale_angular_deg(0, 64)
        assert abs(descaled) < 0.001
    
    def test_scaling_limits(self):
        """Test clamping to 16-bit signed range."""
        scaler = VelocityScaler()
        
        # Test very large positive value (should clamp to 32767)
        scaled = scaler.scale_linear(10.0, 4096)
        assert scaled == 32767
        
        # Test very large negative value (should clamp to -32768)
        scaled = scaler.scale_linear(-10.0, 4096)
        assert scaled == -32768


class TestTeleopCANAdapter:
    """Test teleoperation CAN protocol adapter."""
    
    def setup_method(self):
        """Setup test adapter."""
        self.adapter = TeleopCANAdapter()
    
    def test_encode_forward_velocity(self):
        """Test encoding forward velocity."""
        twist = Twist()
        twist.linear.x = 0.5  # 0.5 m/s forward
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        
        msg = self.adapter.encode_velocity_command(twist)
        
        assert msg is not None
        assert msg.message_type == "velocity_command"
        
        # Decode the SLCAN frame
        slcan_frame = msg.data.decode('ascii')
        assert slcan_frame.startswith('t00C6')  # Message ID 0x00C, DLC 6
        assert slcan_frame.endswith('\r')
        
        # Check scaled value in metadata
        assert msg.metadata['x_vel_scaled'] == 2048  # 0.5 * 4096
        assert msg.metadata['y_vel_scaled'] == 0
        assert msg.metadata['rot_vel_scaled'] == 0
    
    def test_encode_rotation(self):
        """Test encoding rotation."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.2618  # ~15 deg/s
        
        msg = self.adapter.encode_velocity_command(twist)
        
        assert msg is not None
        
        # Check rotation is scaled
        rot_scaled = msg.metadata['rot_vel_scaled']
        assert 950 <= rot_scaled <= 970  # 15 * 64 ≈ 960
    
    def test_encode_combined_velocity(self):
        """Test encoding combined forward and rotation."""
        twist = Twist()
        twist.linear.x = 0.5
        twist.linear.y = 0.25
        twist.angular.z = 0.2618
        
        msg = self.adapter.encode_velocity_command(twist)
        
        assert msg is not None
        assert msg.metadata['x_vel_scaled'] == 2048
        assert msg.metadata['y_vel_scaled'] == 1024  # 0.25 * 4096
        assert 950 <= msg.metadata['rot_vel_scaled'] <= 970
    
    def test_encode_negative_velocity(self):
        """Test encoding backward (negative) velocity."""
        twist = Twist()
        twist.linear.x = -0.5
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        
        msg = self.adapter.encode_velocity_command(twist)
        
        assert msg is not None
        assert msg.metadata['x_vel_scaled'] == -2048
    
    def test_decode_velocity_response(self):
        """Test decoding velocity response."""
        # Create SLCAN response: 0.5 m/s forward, 15 deg/s rotation
        slcan_frame = b't00D60800000003c0\r'
        
        twist = self.adapter.decode_velocity_feedback(slcan_frame)
        
        assert twist is not None
        assert abs(twist.linear.x - 0.5) < 0.001
        assert abs(twist.linear.y - 0.0) < 0.001
        assert abs(twist.angular.z - 0.2618) < 0.01
    
    def test_decode_estimated_velocities(self):
        """Test decoding estimated velocities (message ID 0x115)."""
        # Same format as response but different message ID
        slcan_frame = b't11560800000003c0\r'
        
        twist = self.adapter.decode_velocity_feedback(slcan_frame)
        
        assert twist is not None
        assert abs(twist.linear.x - 0.5) < 0.001
    
    def test_roundtrip_accuracy(self):
        """Test encoding and decoding round-trip."""
        # Original twist
        twist_orig = Twist()
        twist_orig.linear.x = 0.5
        twist_orig.linear.y = 0.25
        twist_orig.angular.z = 0.2618
        
        # Encode
        msg = self.adapter.encode_velocity_command(twist_orig)
        assert msg is not None
        
        # Modify message ID to response (0x00D)
        slcan_send = msg.data.decode('ascii')
        slcan_response = slcan_send.replace('t00C', 't00D').encode('ascii')
        
        # Decode
        twist_decoded = self.adapter.decode_velocity_feedback(slcan_response)
        
        assert twist_decoded is not None
        assert abs(twist_decoded.linear.x - twist_orig.linear.x) < 0.001
        assert abs(twist_decoded.linear.y - twist_orig.linear.y) < 0.001
        assert abs(twist_decoded.angular.z - twist_orig.angular.z) < 0.01
    
    def test_decode_invalid_frame(self):
        """Test decoding invalid SLCAN frame."""
        # Too short
        twist = self.adapter.decode_velocity_feedback(b't00D6\r')
        assert twist is None
        
        # Invalid format
        twist = self.adapter.decode_velocity_feedback(b'invalid')
        assert twist is None
        
        # Wrong message ID
        twist = self.adapter.decode_velocity_feedback(b't00E60800000003c0\r')
        assert twist is None
    
    def test_encode_heartbeat(self):
        """Test encoding heartbeat message."""
        msg = self.adapter.encode_heartbeat()
        
        assert msg is not None
        assert msg.message_type == "heartbeat"
        
        slcan_frame = msg.data.decode('ascii')
        assert slcan_frame == 't00E0\r'  # Message ID 0x00E, no data
    
    def test_encode_homing_sequence(self):
        """Test encoding homing sequence."""
        msg = self.adapter.encode_homing_sequence()
        
        assert msg is not None
        assert msg.message_type == "homing_sequence"
        
        slcan_frame = msg.data.decode('ascii')
        assert slcan_frame.startswith('t110')  # Message ID 0x110
        assert slcan_frame.endswith('\r')
    
    def test_encode_sensor_request(self):
        """Test encoding sensor requests."""
        # Velocity request
        msg = self.adapter.encode_sensor_request("velocity")
        assert msg is not None
        assert msg.data.decode('ascii') == 't1140\r'  # Message ID 0x114
        
        # Offset request
        msg = self.adapter.encode_sensor_request("offset")
        assert msg is not None
        assert msg.data.decode('ascii') == 't1120\r'  # Message ID 0x112
        
        # Unknown sensor type
        msg = self.adapter.encode_sensor_request("unknown")
        assert msg is None
    
    def test_adapter_statistics(self):
        """Test adapter statistics tracking."""
        twist = Twist()
        twist.linear.x = 0.5
        
        # Encode a few messages
        for _ in range(5):
            self.adapter.encode_velocity_command(twist)
        
        # Decode a few messages
        for _ in range(3):
            self.adapter.decode_velocity_feedback(b't00D60800000003c0\r')
        
        # Cause some errors
        self.adapter.decode_velocity_feedback(b'invalid')
        self.adapter.decode_velocity_feedback(b'invalid')
        
        stats = self.adapter.get_stats()
        
        assert stats['messages_encoded'] == 5
        assert stats['messages_decoded'] == 3
        assert stats['decoding_errors'] == 2
        assert stats['error_rate'] > 0
    
    def test_zero_velocity(self):
        """Test encoding zero velocity (stop command)."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        
        msg = self.adapter.encode_velocity_command(twist)
        
        assert msg is not None
        
        # All scaled values should be zero
        assert msg.metadata['x_vel_scaled'] == 0
        assert msg.metadata['y_vel_scaled'] == 0
        assert msg.metadata['rot_vel_scaled'] == 0
        
        # SLCAN frame should contain all zeros in data
        slcan_frame = msg.data.decode('ascii')
        assert '000000000000' in slcan_frame


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
