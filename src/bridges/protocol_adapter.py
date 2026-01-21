#!/usr/bin/env python3
"""
Protocol Adapter - Base Classes for Multi-Protocol Support

Provides abstraction layer for converting between ROS2 messages and various
hardware protocols (SLCAN, Socket.IO, HTTP, etc.).

Author: URC 2026 Bridge Integration Team
"""

from abc import ABC, abstractmethod
from typing import Optional, Dict, Any, List, Tuple
from dataclasses import dataclass
from enum import Enum
import struct
import logging
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, BatteryState, JointState

logger = logging.getLogger(__name__)


class ProtocolType(Enum):
    """Supported protocol types."""
    TELEOPERATION = "teleoperation"  # SLCAN with teleoperation message IDs
    MAIN = "main"  # Native main codebase protocol
    SOCKETIO = "socketio"  # Socket.IO JSON protocol
    HTTP = "http"  # HTTP REST protocol


@dataclass
class ProtocolMessage:
    """Generic protocol message container."""
    message_type: str
    data: bytes
    metadata: Dict[str, Any]
    timestamp: float = None
    
    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()


class ProtocolAdapter(ABC):
    """
    Base class for protocol adapters.
    
    Each adapter converts between ROS2 messages and a specific protocol format.
    """
    
    def __init__(self, config: Dict[str, Any] = None):
        self.config = config or {}
        self.protocol_type: ProtocolType
        self.encoding_errors = 0
        self.decoding_errors = 0
        self.messages_encoded = 0
        self.messages_decoded = 0
    
    @abstractmethod
    def encode_velocity_command(self, twist: Twist) -> Optional[ProtocolMessage]:
        """
        Encode ROS2 Twist to protocol-specific format.
        
        Args:
            twist: ROS2 Twist message
        
        Returns:
            ProtocolMessage or None if encoding fails
        """
        pass
    
    @abstractmethod
    def decode_velocity_feedback(self, data: bytes) -> Optional[Twist]:
        """
        Decode protocol-specific velocity feedback to ROS2 Twist.
        
        Args:
            data: Raw protocol data
        
        Returns:
            Twist or None if decoding fails
        """
        pass
    
    @abstractmethod
    def encode_sensor_request(self, sensor_type: str) -> Optional[ProtocolMessage]:
        """
        Encode sensor data request.
        
        Args:
            sensor_type: Type of sensor ("imu", "battery", "encoders", etc.)
        
        Returns:
            ProtocolMessage or None if encoding fails
        """
        pass
    
    @abstractmethod
    def decode_sensor_data(self, data: bytes, sensor_type: str) -> Optional[Dict[str, Any]]:
        """
        Decode sensor data from protocol format.
        
        Args:
            data: Raw protocol data
            sensor_type: Expected sensor type
        
        Returns:
            Dictionary with sensor data or None if decoding fails
        """
        pass
    
    def get_stats(self) -> Dict[str, Any]:
        """Get adapter statistics."""
        return {
            "protocol_type": self.protocol_type.value,
            "messages_encoded": self.messages_encoded,
            "messages_decoded": self.messages_decoded,
            "encoding_errors": self.encoding_errors,
            "decoding_errors": self.decoding_errors,
            "error_rate": (self.encoding_errors + self.decoding_errors) / 
                         max(1, self.messages_encoded + self.messages_decoded)
        }


class VelocityScaler:
    """
    Utility for velocity scaling/descaling operations.
    
    Different protocols use different scaling factors for fixed-point arithmetic.
    """
    
    @staticmethod
    def scale_linear(velocity_m_per_s: float, scale_factor: int) -> int:
        """
        Scale linear velocity to fixed-point integer.
        
        Args:
            velocity_m_per_s: Velocity in m/s
            scale_factor: Scaling factor (e.g., 4096 for 2^12)
        
        Returns:
            Scaled velocity as 16-bit signed integer
        """
        scaled = int(velocity_m_per_s * scale_factor)
        # Clamp to 16-bit signed range
        return max(-32768, min(32767, scaled))
    
    @staticmethod
    def descale_linear(scaled_value: int, scale_factor: int) -> float:
        """
        Descale fixed-point integer to linear velocity.
        
        Args:
            scaled_value: Scaled integer value
            scale_factor: Scaling factor used
        
        Returns:
            Velocity in m/s
        """
        return float(scaled_value) / float(scale_factor)
    
    @staticmethod
    def scale_angular_deg(velocity_rad_per_s: float, scale_factor: int) -> int:
        """
        Scale angular velocity from rad/s to scaled deg/s.
        
        Args:
            velocity_rad_per_s: Angular velocity in rad/s
            scale_factor: Scaling factor for degrees (e.g., 64 for 2^6)
        
        Returns:
            Scaled angular velocity as 16-bit signed integer
        """
        # Convert rad/s to deg/s
        velocity_deg_per_s = velocity_rad_per_s * 57.2957795131
        scaled = int(velocity_deg_per_s * scale_factor)
        # Clamp to 16-bit signed range
        return max(-32768, min(32767, scaled))
    
    @staticmethod
    def descale_angular_deg(scaled_value: int, scale_factor: int) -> float:
        """
        Descale fixed-point degrees to rad/s.
        
        Args:
            scaled_value: Scaled integer value in degrees
            scale_factor: Scaling factor used
        
        Returns:
            Angular velocity in rad/s
        """
        velocity_deg_per_s = float(scaled_value) / float(scale_factor)
        # Convert deg/s to rad/s
        return velocity_deg_per_s / 57.2957795131


# Export key components
__all__ = [
    'ProtocolAdapter',
    'ProtocolType',
    'ProtocolMessage',
    'VelocityScaler'
]
