#!/usr/bin/env python3
"""
Teleoperation CAN Protocol Adapter

Implements SLCAN protocol with teleoperation message IDs and scaling.
Adapts main codebase to use teleoperation/control-systems protocol.

Protocol Specification:
- Message IDs: 0x00C-0x300 (teleoperation convention)
- Velocity scaling: Linear ×4096 (2^12), Angular ×64 (2^6) 
- Format: SLCAN ('t<ID><DLC><DATA>\\r')

Author: URC 2026 Bridge Integration Team
"""

from typing import Optional, Dict, Any
import logging
import time

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, BatteryState

from .protocol_adapter import (
    ProtocolAdapter, ProtocolType, ProtocolMessage, VelocityScaler
)

logger = logging.getLogger(__name__)


class TeleopCANAdapter(ProtocolAdapter):
    """
    Teleoperation CAN protocol adapter.
    
    Converts between ROS2 messages and SLCAN frames using teleoperation
    message IDs and scaling conventions.
    """
    
    # Teleoperation protocol message IDs (from vendor/teleoperation)
    SEND_IDS = {
        "SET_CHASSIS_VELOCITIES": 0x00C,
        "HEARTBEAT": 0x00E,
        "HOMING_SEQUENCE": 0x110,
        "GET_OFFSET": 0x112,
        "GET_ESTIMATED_VELOCITIES": 0x114,
        "CONFIG": 0x119,
        "SET_MAST_GIMBAL": 0x301,  # Changed from 0x300 to avoid conflict
    }
    
    RECEIVE_IDS = {
        "SET_VELOCITIES_RESPONSE": 0x00D,
        "HEARTBEAT_REPLY": 0x00F,
        "HOMING_SEQUENCE_RESPONSE": 0x111,
        "RETURN_OFFSET": 0x113,
        "RETURN_ESTIMATED_CHASSIS_VELOCITIES": 0x115,
        "CONFIG_ACK": 0x11A,
    }
    
    # Scaling factors (from teleoperation protocol)
    LINEAR_SCALE = 4096  # 2^12
    ANGULAR_SCALE = 64   # 2^6
    
    def __init__(self, config: Dict[str, Any] = None):
        super().__init__(config)
        self.protocol_type = ProtocolType.TELEOPERATION
        self.velocity_scaler = VelocityScaler()
    
    def encode_velocity_command(self, twist: Twist) -> Optional[ProtocolMessage]:
        """
        Encode ROS2 Twist to SLCAN velocity command.
        
        Format: 't00C6<x_vel><y_vel><rot_vel>\\r'
        
        Args:
            twist: ROS2 Twist message (linear.x, linear.y, angular.z)
        
        Returns:
            ProtocolMessage with SLCAN frame or None if encoding fails
        """
        try:
            # Extract velocities
            x_vel = twist.linear.x  # m/s forward
            y_vel = twist.linear.y  # m/s lateral (swerve drive)
            rot_vel = twist.angular.z  # rad/s
            
            # Scale velocities to fixed-point integers
            x_scaled = self.velocity_scaler.scale_linear(x_vel, self.LINEAR_SCALE)
            y_scaled = self.velocity_scaler.scale_linear(y_vel, self.LINEAR_SCALE)
            rot_scaled = self.velocity_scaler.scale_angular_deg(rot_vel, self.ANGULAR_SCALE)
            
            # Convert to bytes (big-endian, signed)
            x_bytes = x_scaled.to_bytes(2, 'big', signed=True)
            y_bytes = y_scaled.to_bytes(2, 'big', signed=True)
            rot_bytes = rot_scaled.to_bytes(2, 'big', signed=True)
            
            # Create SLCAN frame
            msg_id = self.SEND_IDS["SET_CHASSIS_VELOCITIES"]
            data_hex = x_bytes.hex() + y_bytes.hex() + rot_bytes.hex()
            slcan_frame = f't{msg_id:03X}6{data_hex}\r'
            
            self.messages_encoded += 1
            
            logger.debug(
                f"Encoded velocity: x={x_vel:.3f} y={y_vel:.3f} rot={rot_vel:.3f} "
                f"→ SLCAN: {slcan_frame.strip()}"
            )
            
            return ProtocolMessage(
                message_type="velocity_command",
                data=slcan_frame.encode('ascii'),
                metadata={
                    "msg_id": msg_id,
                    "x_vel_scaled": x_scaled,
                    "y_vel_scaled": y_scaled,
                    "rot_vel_scaled": rot_scaled,
                    "original_twist": {
                        "x": x_vel,
                        "y": y_vel,
                        "rot": rot_vel
                    }
                }
            )
            
        except Exception as e:
            logger.error(f"Failed to encode velocity command: {e}")
            self.encoding_errors += 1
            return None
    
    def decode_velocity_feedback(self, data: bytes) -> Optional[Twist]:
        """
        Decode SLCAN velocity response to ROS2 Twist.
        
        Expected format: 't00D6<x_vel><y_vel><rot_vel>\\r'
        
        Args:
            data: SLCAN frame bytes
        
        Returns:
            Twist message or None if decoding fails
        """
        try:
            # Decode to string
            slcan_frame = data.decode('ascii').strip()
            
            # Validate format
            # Minimum length: 't' + 3 hex ID + 1 hex DLC + data + '\r' = depends on DLC
            if not slcan_frame.startswith('t') or len(slcan_frame) < 6:
                logger.warning(f"Invalid SLCAN frame format: {slcan_frame}")
                self.decoding_errors += 1
                return None
            
            # Parse message ID
            msg_id = int(slcan_frame[1:4], 16)
            expected_ids = [
                self.RECEIVE_IDS["SET_VELOCITIES_RESPONSE"],
                self.RECEIVE_IDS["RETURN_ESTIMATED_CHASSIS_VELOCITIES"]
            ]
            
            if msg_id not in expected_ids:
                logger.debug(f"Not a velocity feedback message: 0x{msg_id:03X}")
                return None
            
            # Extract DLC
            dlc = int(slcan_frame[4], 16)
            if dlc != 6:
                logger.warning(f"Unexpected DLC for velocity feedback: {dlc}")
                self.decoding_errors += 1
                return None
            
            # Extract data bytes (6 bytes = 12 hex chars)
            data_hex = slcan_frame[5:17]
            if len(data_hex) != 12:
                logger.warning(f"Invalid data length: {len(data_hex)}")
                self.decoding_errors += 1
                return None
            
            # Parse velocity values
            x_hex = data_hex[0:4]
            y_hex = data_hex[4:8]
            rot_hex = data_hex[8:12]
            
            # Convert to signed integers
            x_scaled = int.from_bytes(bytes.fromhex(x_hex), 'big', signed=True)
            y_scaled = int.from_bytes(bytes.fromhex(y_hex), 'big', signed=True)
            rot_scaled = int.from_bytes(bytes.fromhex(rot_hex), 'big', signed=True)
            
            # Descale to SI units
            x_vel = self.velocity_scaler.descale_linear(x_scaled, self.LINEAR_SCALE)
            y_vel = self.velocity_scaler.descale_linear(y_scaled, self.LINEAR_SCALE)
            rot_vel = self.velocity_scaler.descale_angular_deg(rot_scaled, self.ANGULAR_SCALE)
            
            # Create Twist message
            twist = Twist()
            twist.linear.x = x_vel
            twist.linear.y = y_vel
            twist.angular.z = rot_vel
            
            self.messages_decoded += 1
            
            logger.debug(
                f"Decoded velocity: SLCAN: {slcan_frame} → "
                f"x={x_vel:.3f} y={y_vel:.3f} rot={rot_vel:.3f}"
            )
            
            return twist
            
        except Exception as e:
            logger.error(f"Failed to decode velocity feedback: {e}")
            self.decoding_errors += 1
            return None
    
    def encode_sensor_request(self, sensor_type: str) -> Optional[ProtocolMessage]:
        """
        Encode sensor data request.
        
        Args:
            sensor_type: "velocity", "offset", etc.
        
        Returns:
            ProtocolMessage with SLCAN request frame
        """
        try:
            msg_id = None
            
            if sensor_type == "velocity":
                msg_id = self.SEND_IDS["GET_ESTIMATED_VELOCITIES"]
            elif sensor_type == "offset":
                msg_id = self.SEND_IDS["GET_OFFSET"]
            else:
                logger.warning(f"Unknown sensor type: {sensor_type}")
                return None
            
            # Create request frame (no data)
            slcan_frame = f't{msg_id:03X}0\r'
            
            self.messages_encoded += 1
            
            return ProtocolMessage(
                message_type=f"sensor_request_{sensor_type}",
                data=slcan_frame.encode('ascii'),
                metadata={"msg_id": msg_id, "sensor_type": sensor_type}
            )
            
        except Exception as e:
            logger.error(f"Failed to encode sensor request: {e}")
            self.encoding_errors += 1
            return None
    
    def decode_sensor_data(self, data: bytes, sensor_type: str) -> Optional[Dict[str, Any]]:
        """
        Decode sensor data from SLCAN format.
        
        Args:
            data: SLCAN frame bytes
            sensor_type: Expected sensor type
        
        Returns:
            Dictionary with parsed sensor data
        """
        try:
            slcan_frame = data.decode('ascii').strip()
            
            if not slcan_frame.startswith('t'):
                return None
            
            msg_id = int(slcan_frame[1:4], 16)
            
            # Route to appropriate decoder
            if msg_id == self.RECEIVE_IDS["RETURN_OFFSET"]:
                return self._decode_offset_data(slcan_frame)
            elif msg_id == self.RECEIVE_IDS["RETURN_ESTIMATED_CHASSIS_VELOCITIES"]:
                # Already handled by decode_velocity_feedback
                return None
            
            self.messages_decoded += 1
            return None
            
        except Exception as e:
            logger.error(f"Failed to decode sensor data: {e}")
            self.decoding_errors += 1
            return None
    
    def _decode_offset_data(self, slcan_frame: str) -> Optional[Dict[str, Any]]:
        """Decode encoder offset data."""
        try:
            # Format: 't113<8-byte-offset><1-byte-module-position>\r'
            if len(slcan_frame) < 15:
                return None
            
            offset_hex = slcan_frame[5:13]
            module_pos_hex = slcan_frame[13:15]
            
            angle_offset = int(offset_hex, 16)
            module_position = int(module_pos_hex, 16)
            
            return {
                "type": "encoder_offset",
                "angle_offset": angle_offset,
                "module_position": module_position
            }
            
        except Exception as e:
            logger.error(f"Failed to decode offset data: {e}")
            return None
    
    def encode_heartbeat(self) -> Optional[ProtocolMessage]:
        """Encode heartbeat message."""
        try:
            msg_id = self.SEND_IDS["HEARTBEAT"]
            slcan_frame = f't{msg_id:03X}0\r'
            
            return ProtocolMessage(
                message_type="heartbeat",
                data=slcan_frame.encode('ascii'),
                metadata={"msg_id": msg_id}
            )
            
        except Exception as e:
            logger.error(f"Failed to encode heartbeat: {e}")
            return None
    
    def encode_homing_sequence(self) -> Optional[ProtocolMessage]:
        """Encode homing sequence command."""
        try:
            msg_id = self.SEND_IDS["HOMING_SEQUENCE"]
            # 8 bytes of zeros for homing command
            slcan_frame = f't{msg_id:03X}80000000000000000\r'
            
            return ProtocolMessage(
                message_type="homing_sequence",
                data=slcan_frame.encode('ascii'),
                metadata={"msg_id": msg_id}
            )
            
        except Exception as e:
            logger.error(f"Failed to encode homing sequence: {e}")
            return None


# Export
__all__ = ['TeleopCANAdapter']
