#!/usr/bin/env python3
"""
SLCAN Protocol Simulator for CAN Communication Testing

Complete SLCAN (Serial Line CAN) protocol simulation for testing
CAN communication without physical hardware.

Features:
- Message encoding/decoding (ASCII hex format)
- Velocity scaling (×4096 linear, ×64 angular)
- Message ID routing (0x00C, 0x00E, 0x100, etc.)
- Error conditions (malformed frames, timeouts)
- Bus statistics and monitoring

Author: URC 2026 Simulation Team
"""

import logging
import time
import random
import re
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum
import struct

logger = logging.getLogger(__name__)


class SLCANMessageType(Enum):
    """SLCAN message types."""
    STANDARD_DATA = 't'  # Standard 11-bit ID, data frame
    STANDARD_RTR = 'r'   # Standard 11-bit ID, RTR frame
    EXTENDED_DATA = 'T'  # Extended 29-bit ID, data frame
    EXTENDED_RTR = 'R'   # Extended 29-bit ID, RTR frame


class CANMessageID(Enum):
    """CAN message IDs used in the system."""
    SET_CHASSIS_VELOCITIES = 0x00C
    FEEDBACK_CHASSIS_VELOCITIES = 0x00D
    HEARTBEAT_REQUEST = 0x00E
    HEARTBEAT_RESPONSE = 0x00F
    MOTOR_COMMAND = 0x100
    MOTOR_STATUS = 0x101
    HOMING_REQUEST = 0x110
    HOMING_RESPONSE = 0x111
    EMERGENCY_STOP = 0x1FF
    MAST_GIMBAL = 0x301


@dataclass
class SLCANFrame:
    """Represents a parsed SLCAN frame."""
    message_type: SLCANMessageType
    message_id: int
    data_length: int
    data: bytes
    timestamp: float = field(default_factory=time.time)
    raw_frame: str = ""
    
    def __post_init__(self):
        """Validate frame data."""
        if self.data_length > 8:
            raise ValueError(f"CAN data length cannot exceed 8 bytes, got {self.data_length}")
        if len(self.data) != self.data_length:
            raise ValueError(f"Data length mismatch: expected {self.data_length}, got {len(self.data)}")


@dataclass
class VelocityCommand:
    """Parsed velocity command from CAN message."""
    linear_x: float  # m/s
    linear_y: float  # m/s (swerve)
    angular_z: float  # rad/s
    timestamp: float = field(default_factory=time.time)


class SLCANProtocolSimulator:
    """
    Complete SLCAN protocol simulator.
    
    Simulates serial CAN communication including:
    - Frame encoding/decoding
    - Velocity scaling
    - Message routing
    - Error injection
    - Bus statistics
    """
    
    # Velocity scaling factors (from protocol specification)
    LINEAR_SCALE = 4096  # 2^12 for linear velocity (m/s)
    ANGULAR_SCALE = 64   # 2^6 for angular velocity (after deg/s conversion)
    RAD_TO_DEG = 57.2957795131  # Conversion factor
    
    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """Initialize SLCAN protocol simulator.
        
        Args:
            config: Configuration dictionary with error rates, delays, etc.
        """
        self.logger = logging.getLogger(f"{__name__}.SLCANProtocolSimulator")
        
        # Configuration
        config = config or {}
        self.simulate_errors = config.get('simulate_errors', True)
        self.error_rate = config.get('error_rate', 0.001)  # 0.1% error rate
        self.serial_delay_ms = config.get('serial_delay_ms', 5.0)  # 5ms average
        self.buffer_size = config.get('buffer_size', 1024)
        
        # Frame buffer (simulates serial buffer)
        self.rx_buffer = bytearray()
        self.tx_buffer = bytearray()
        
        # Statistics
        self.stats = {
            'frames_sent': 0,
            'frames_received': 0,
            'frames_dropped': 0,
            'encoding_errors': 0,
            'decoding_errors': 0,
            'buffer_overflows': 0,
            'malformed_frames': 0
        }
        
        # Message history
        self.message_history: List[SLCANFrame] = []
        self.max_history = config.get('max_history', 1000)
        
        # Bus load tracking
        self.bus_load_window = []
        self.bus_load_window_size = 100  # Track last 100 messages
        
        self.logger.info("SLCAN protocol simulator initialized")
    
    def encode_velocity_command(self, linear_x: float, linear_y: float, 
                                angular_z: float) -> str:
        """Encode velocity command to SLCAN frame.
        
        Args:
            linear_x: Forward velocity (m/s)
            linear_y: Lateral velocity (m/s)
            angular_z: Rotational velocity (rad/s)
            
        Returns:
            str: SLCAN frame string (e.g., 't00C6...\r')
        """
        try:
            # Scale velocities
            x_scaled = int(linear_x * self.LINEAR_SCALE)
            y_scaled = int(linear_y * self.LINEAR_SCALE)
            
            # Convert angular velocity: rad/s → deg/s → scaled
            angular_deg_s = angular_z * self.RAD_TO_DEG
            rot_scaled = int(angular_deg_s * self.ANGULAR_SCALE)
            
            # Clamp to 16-bit signed integer range
            x_scaled = max(-32768, min(32767, x_scaled))
            y_scaled = max(-32768, min(32767, y_scaled))
            rot_scaled = max(-32768, min(32767, rot_scaled))
            
            # Pack as bytes (big-endian, signed)
            data = struct.pack('>hhh', x_scaled, y_scaled, rot_scaled)
            
            # Create SLCAN frame
            msg_id = CANMessageID.SET_CHASSIS_VELOCITIES.value
            data_len = len(data)
            data_hex = data.hex()
            
            frame = f't{msg_id:03X}{data_len}{data_hex}\r'
            
            self.stats['frames_sent'] += 1
            self._record_frame(frame, 'encoded')
            
            return frame
            
        except Exception as e:
            self.logger.error(f"Encoding error: {e}")
            self.stats['encoding_errors'] += 1
            raise
    
    def decode_velocity_command(self, slcan_frame: str) -> Optional[VelocityCommand]:
        """Decode velocity command from SLCAN frame.
        
        Args:
            slcan_frame: SLCAN frame string
            
        Returns:
            VelocityCommand object or None if invalid
        """
        try:
            # Parse SLCAN frame
            parsed = self.parse_frame(slcan_frame)
            if not parsed:
                return None
            
            # Check if this is a velocity command
            if parsed.message_id != CANMessageID.SET_CHASSIS_VELOCITIES.value:
                self.logger.debug(f"Frame is not velocity command: 0x{parsed.message_id:03X}")
                return None
            
            # Check data length
            if parsed.data_length != 6:
                self.logger.error(f"Invalid data length for velocity command: {parsed.data_length}")
                return None
            
            # Unpack velocity values (big-endian, signed)
            x_scaled, y_scaled, rot_scaled = struct.unpack('>hhh', parsed.data)
            
            # Descale to SI units
            linear_x = x_scaled / self.LINEAR_SCALE
            linear_y = y_scaled / self.LINEAR_SCALE
            angular_deg_s = rot_scaled / self.ANGULAR_SCALE
            angular_z = angular_deg_s / self.RAD_TO_DEG
            
            return VelocityCommand(
                linear_x=linear_x,
                linear_y=linear_y,
                angular_z=angular_z
            )
            
        except Exception as e:
            self.logger.error(f"Decoding error: {e}")
            self.stats['decoding_errors'] += 1
            return None
    
    def parse_frame(self, slcan_frame: str) -> Optional[SLCANFrame]:
        """Parse SLCAN frame string.
        
        Args:
            slcan_frame: SLCAN frame string (e.g., 't00C6...\r')
            
        Returns:
            SLCANFrame object or None if invalid
        """
        try:
            # Remove whitespace and carriage return
            frame = slcan_frame.strip()
            
            # Check minimum length (type + 3 hex ID + length)
            if len(frame) < 5:
                self.logger.error(f"Frame too short: {len(frame)} chars")
                self.stats['malformed_frames'] += 1
                return None
            
            # Parse message type
            msg_type_char = frame[0]
            try:
                msg_type = SLCANMessageType(msg_type_char)
            except ValueError:
                self.logger.error(f"Invalid message type: {msg_type_char}")
                self.stats['malformed_frames'] += 1
                return None
            
            # Parse message ID (3 hex digits for standard, 8 for extended)
            if msg_type in [SLCANMessageType.STANDARD_DATA, SLCANMessageType.STANDARD_RTR]:
                msg_id = int(frame[1:4], 16)
                data_start = 5
            else:
                msg_id = int(frame[1:9], 16)
                data_start = 10
            
            # Parse data length
            data_len = int(frame[data_start - 1])
            if data_len > 8:
                self.logger.error(f"Invalid data length: {data_len}")
                self.stats['malformed_frames'] += 1
                return None
            
            # Parse data bytes
            data_hex = frame[data_start:data_start + data_len * 2]
            if len(data_hex) != data_len * 2:
                self.logger.error(f"Data length mismatch: expected {data_len*2} chars, got {len(data_hex)}")
                self.stats['malformed_frames'] += 1
                return None
            
            data = bytes.fromhex(data_hex)
            
            # Create frame object
            parsed_frame = SLCANFrame(
                message_type=msg_type,
                message_id=msg_id,
                data_length=data_len,
                data=data,
                raw_frame=slcan_frame
            )
            
            self.stats['frames_received'] += 1
            self._record_frame(slcan_frame, 'decoded')
            
            return parsed_frame
            
        except Exception as e:
            self.logger.error(f"Frame parsing error: {e}")
            self.stats['decoding_errors'] += 1
            return None
    
    def encode_frame(self, message_id: int, data: bytes, 
                    extended: bool = False) -> str:
        """Encode generic CAN frame to SLCAN format.
        
        Args:
            message_id: CAN message ID
            data: Data bytes (max 8)
            extended: Use extended 29-bit ID
            
        Returns:
            str: SLCAN frame string
        """
        try:
            if len(data) > 8:
                raise ValueError(f"Data length {len(data)} exceeds maximum 8 bytes")
            
            # Select message type
            msg_type = 'T' if extended else 't'
            
            # Format ID (8 hex digits for extended, 3 for standard)
            if extended:
                id_str = f'{message_id:08X}'
            else:
                id_str = f'{message_id:03X}'
            
            # Format data
            data_len = len(data)
            data_hex = data.hex()
            
            # Create frame
            frame = f'{msg_type}{id_str}{data_len}{data_hex}\r'
            
            self.stats['frames_sent'] += 1
            self._record_frame(frame, 'encoded')
            
            return frame
            
        except Exception as e:
            self.logger.error(f"Frame encoding error: {e}")
            self.stats['encoding_errors'] += 1
            raise
    
    def encode_heartbeat(self) -> str:
        """Encode heartbeat request frame.
        
        Returns:
            str: SLCAN heartbeat frame
        """
        return self.encode_frame(CANMessageID.HEARTBEAT_REQUEST.value, b'')
    
    def encode_heartbeat_response(self) -> str:
        """Encode heartbeat response frame.
        
        Returns:
            str: SLCAN heartbeat response frame
        """
        return self.encode_frame(CANMessageID.HEARTBEAT_RESPONSE.value, b'')
    
    def encode_homing_request(self) -> str:
        """Encode homing sequence request.
        
        Returns:
            str: SLCAN homing request frame
        """
        data = b'\x00' * 8  # 8 bytes of zeros
        return self.encode_frame(CANMessageID.HOMING_REQUEST.value, data)
    
    def encode_homing_response(self) -> str:
        """Encode homing sequence response.
        
        Returns:
            str: SLCAN homing response frame
        """
        return self.encode_frame(CANMessageID.HOMING_RESPONSE.value, b'')
    
    def encode_emergency_stop(self) -> str:
        """Encode emergency stop command.
        
        Returns:
            str: SLCAN emergency stop frame
        """
        data = b'\xFF'  # Stop command
        return self.encode_frame(CANMessageID.EMERGENCY_STOP.value, data)
    
    def simulate_serial_delay(self) -> float:
        """Simulate serial communication delay.
        
        Returns:
            float: Delay time in seconds
        """
        # Add realistic serial delay with jitter
        delay_ms = random.gauss(self.serial_delay_ms, self.serial_delay_ms * 0.2)
        delay_ms = max(1.0, delay_ms)  # Minimum 1ms
        
        delay_s = delay_ms / 1000.0
        time.sleep(delay_s)
        
        return delay_s
    
    def inject_error(self, frame: str) -> str:
        """Inject error into frame for testing.
        
        Args:
            frame: Original SLCAN frame
            
        Returns:
            str: Frame with injected error (or original if no error)
        """
        if not self.simulate_errors:
            return frame
        
        if random.random() > self.error_rate:
            return frame
        
        # Choose random error type
        error_type = random.choice([
            'corrupt_byte',
            'truncate',
            'wrong_checksum',
            'invalid_id'
        ])
        
        if error_type == 'corrupt_byte' and len(frame) > 5:
            # Corrupt random data byte
            pos = random.randint(5, len(frame) - 2)
            frame_list = list(frame)
            frame_list[pos] = random.choice('0123456789ABCDEF')
            frame = ''.join(frame_list)
            
        elif error_type == 'truncate':
            # Truncate frame
            frame = frame[:random.randint(3, len(frame) - 1)]
            
        elif error_type == 'invalid_id':
            # Use invalid message ID
            frame = 't' + 'FFF' + frame[4:]
        
        self.logger.debug(f"Injected error: {error_type}")
        self.stats['frames_dropped'] += 1
        
        return frame
    
    def write_to_buffer(self, frame: str) -> bool:
        """Write frame to TX buffer.
        
        Args:
            frame: SLCAN frame to write
            
        Returns:
            bool: True if successful, False if buffer overflow
        """
        frame_bytes = frame.encode('ascii')
        
        if len(self.tx_buffer) + len(frame_bytes) > self.buffer_size:
            self.logger.warning("TX buffer overflow")
            self.stats['buffer_overflows'] += 1
            return False
        
        self.tx_buffer.extend(frame_bytes)
        return True
    
    def read_from_buffer(self) -> Optional[str]:
        """Read complete frame from RX buffer.
        
        Returns:
            str: Complete SLCAN frame or None
        """
        # Look for frame terminator
        try:
            term_idx = self.rx_buffer.index(ord('\r'))
        except ValueError:
            # No complete frame yet
            return None
        
        # Extract frame
        frame_bytes = bytes(self.rx_buffer[:term_idx + 1])
        frame = frame_bytes.decode('ascii')
        
        # Remove from buffer
        self.rx_buffer = self.rx_buffer[term_idx + 1:]
        
        return frame
    
    def get_statistics(self) -> Dict[str, Any]:
        """Get protocol statistics.
        
        Returns:
            Dict with statistics
        """
        return {
            'frames_sent': self.stats['frames_sent'],
            'frames_received': self.stats['frames_received'],
            'frames_dropped': self.stats['frames_dropped'],
            'encoding_errors': self.stats['encoding_errors'],
            'decoding_errors': self.stats['decoding_errors'],
            'buffer_overflows': self.stats['buffer_overflows'],
            'malformed_frames': self.stats['malformed_frames'],
            'success_rate': self._calculate_success_rate(),
            'buffer_usage': {
                'tx_bytes': len(self.tx_buffer),
                'rx_bytes': len(self.rx_buffer),
                'tx_percent': len(self.tx_buffer) / self.buffer_size * 100,
                'rx_percent': len(self.rx_buffer) / self.buffer_size * 100
            }
        }
    
    def _calculate_success_rate(self) -> float:
        """Calculate message success rate.
        
        Returns:
            float: Success rate (0.0 to 1.0)
        """
        total = self.stats['frames_sent'] + self.stats['frames_received']
        if total == 0:
            return 1.0
        
        errors = (self.stats['frames_dropped'] + 
                 self.stats['encoding_errors'] + 
                 self.stats['decoding_errors'] +
                 self.stats['malformed_frames'])
        
        return max(0.0, 1.0 - (errors / total))
    
    def _record_frame(self, frame: str, direction: str):
        """Record frame in history.
        
        Args:
            frame: SLCAN frame string
            direction: 'encoded' or 'decoded'
        """
        # Parse and store (if not too many)
        if len(self.message_history) < self.max_history:
            try:
                parsed = self.parse_frame(frame) if direction == 'decoded' else None
                if parsed:
                    self.message_history.append(parsed)
            except:
                pass
        
        # Track bus load
        self.bus_load_window.append(time.time())
        if len(self.bus_load_window) > self.bus_load_window_size:
            self.bus_load_window.pop(0)
    
    def get_bus_load(self) -> float:
        """Calculate current bus load (messages per second).
        
        Returns:
            float: Messages per second
        """
        if len(self.bus_load_window) < 2:
            return 0.0
        
        time_span = self.bus_load_window[-1] - self.bus_load_window[0]
        if time_span == 0:
            return 0.0
        
        return len(self.bus_load_window) / time_span
    
    def clear_buffers(self):
        """Clear TX and RX buffers."""
        self.tx_buffer.clear()
        self.rx_buffer.clear()
        self.logger.debug("Buffers cleared")
    
    def reset_statistics(self):
        """Reset all statistics."""
        for key in self.stats:
            self.stats[key] = 0
        self.message_history.clear()
        self.bus_load_window.clear()
        self.logger.info("Statistics reset")


# Convenience functions
def create_slcan_simulator(profile: str = 'default') -> SLCANProtocolSimulator:
    """Create SLCAN simulator with predefined configuration.
    
    Args:
        profile: Configuration profile ('default', 'perfect', 'noisy')
        
    Returns:
        Configured SLCANProtocolSimulator instance
    """
    profiles = {
        'default': {
            'simulate_errors': True,
            'error_rate': 0.001,
            'serial_delay_ms': 5.0
        },
        'perfect': {
            'simulate_errors': False,
            'error_rate': 0.0,
            'serial_delay_ms': 0.0
        },
        'noisy': {
            'simulate_errors': True,
            'error_rate': 0.05,
            'serial_delay_ms': 15.0
        }
    }
    
    config = profiles.get(profile, profiles['default'])
    return SLCANProtocolSimulator(config)
