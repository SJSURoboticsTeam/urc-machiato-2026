#!/usr/bin/env python3
"""
CAN Bridge - Unified CAN Bus Communication

Implements the unified bridge interface for CAN bus communication.
Handles motor control, sensor feedback, and system monitoring.

Now supports protocol adapters for flexible protocol integration.

Author: URC 2026 CAN Bridge Team
"""

import asyncio
import serial
import struct
import time
from typing import Dict, List, Any, Optional
from enum import Enum
import logging

from geometry_msgs.msg import Twist

from .unified_bridge_interface import (
    BridgeInterface, BridgeType, BridgeMessage,
    BridgeStatus, MessagePriority, register_bridge_factory
)
from .protocol_adapter import ProtocolAdapter, ProtocolType
from .teleop_can_adapter import TeleopCANAdapter

logger = logging.getLogger(__name__)


class CANMessageType(Enum):
    """CAN message types for motor and sensor communication."""
    MOTOR_COMMAND = 0x100
    MOTOR_FEEDBACK = 0x101
    IMU_DATA = 0x200
    ENCODER_DATA = 0x201
    BATTERY_STATUS = 0x300
    SYSTEM_STATUS = 0x400
    EMERGENCY_STOP = 0xFFF


class CANBridge(BridgeInterface):
    """
    Unified CAN bridge for hardware communication.

    Handles:
    - Motor control commands
    - Sensor data feedback (IMU, encoders, battery)
    - System health monitoring
    - Emergency stop functionality
    """

    def __init__(self, config: Dict[str, Any]):
        # Set bridge_type before calling super().__init__
        self.bridge_type = BridgeType.CAN
        super().__init__(config)

        # Protocol adapter configuration
        protocol_type = config.get('protocol', 'teleoperation')
        if protocol_type == 'teleoperation':
            self.protocol_adapter = TeleopCANAdapter(config)
            logger.info("Using teleoperation protocol adapter")
        else:
            # Default to teleoperation
            self.protocol_adapter = TeleopCANAdapter(config)
            logger.warning(f"Unknown protocol '{protocol_type}', defaulting to teleoperation")

        # Serial/CAN configuration
        self.device_path = config.get('device', '/dev/ttyACM0')
        self.fallback_devices = config.get('fallback_devices', ['/dev/ttyAMA10', '/dev/ttyUSB0'])
        self.baudrate = config.get('baudrate', 115200)
        self.serial_port = None

        # Motor control state
        self.motor_commands = {
            "left_front": {"speed": 0.0, "current": 0.0},
            "right_front": {"speed": 0.0, "current": 0.0},
            "left_rear": {"speed": 0.0, "current": 0.0},
            "right_rear": {"speed": 0.0, "current": 0.0}
        }

        # Sensor data buffers
        self.sensor_data = {
            "imu": {},
            "encoders": {},
            "battery": {},
            "system": {}
        }

        # Emergency stop state
        self.emergency_stop_active = False
        self.last_emergency_check = time.time()

        # Message processing task
        self._message_task: Optional[asyncio.Task] = None
        self._read_buffer = b''

    async def connect(self) -> bool:
        """Establish serial/CAN connection with auto-discovery."""
        # Try primary device first
        devices_to_try = [self.device_path] + self.fallback_devices
        
        for device in devices_to_try:
            try:
                logger.info(f"Attempting to connect to {device}...")
                self.serial_port = serial.Serial(
                    port=device,
                    baudrate=self.baudrate,
                    timeout=0.1,  # Non-blocking read
                    write_timeout=1.0
                )
                
                self.is_connected = True
                self.device_path = device
                logger.info(f"✓ Connected to CAN bridge: {device}")

                # Start message processing
                self._message_task = asyncio.create_task(self._process_messages())

                return True

            except (serial.SerialException, FileNotFoundError) as e:
                logger.debug(f"Failed to connect to {device}: {e}")
                continue
        
        logger.error("Failed to connect to any CAN device")
        return False

    async def disconnect(self) -> None:
        """Cleanly disconnect from serial/CAN."""
        self.is_connected = False

        if self._message_task:
            self._message_task.cancel()
            try:
                await self._message_task
            except asyncio.CancelledError:
                pass

        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
            self.serial_port = None

        logger.info("Disconnected from CAN bridge")

    async def send_message(self, message: BridgeMessage) -> bool:
        """Send message via serial/CAN using protocol adapter."""
        if not self.is_connected or not self.serial_port:
            return False

        try:
            # Route message to appropriate handler
            if message.message_type == "velocity_command":
                return await self._send_velocity_command(message)
            elif message.message_type == "motor_command":
                return await self._send_motor_command(message)
            elif message.message_type == "emergency_stop":
                return await self._send_emergency_stop(message)
            elif message.message_type == "sensor_request":
                return await self._send_sensor_request(message)
            elif message.message_type == "heartbeat":
                return await self._send_heartbeat()
            elif message.message_type == "homing":
                return await self._send_homing()
            else:
                logger.warning(f"Unknown CAN message type: {message.message_type}")
                return False

        except Exception as e:
            logger.error(f"CAN bridge send error: {e}")
            self.stats.errors += 1
            return False
    
    async def _send_velocity_command(self, message: BridgeMessage) -> bool:
        """
        Send velocity command using protocol adapter.
        
        Args:
            message: BridgeMessage with 'twist' in data
        
        Returns:
            True if sent successfully
        """
        try:
            twist = message.data.get('twist')
            if not twist:
                logger.error("No twist in velocity command message")
                return False
            
            # Use protocol adapter to encode
            protocol_msg = self.protocol_adapter.encode_velocity_command(twist)
            if not protocol_msg:
                logger.error("Failed to encode velocity command")
                return False
            
            # Send via serial
            await asyncio.to_thread(self.serial_port.write, protocol_msg.data)
            self.stats.messages_sent += 1
            
            logger.debug(f"Sent velocity: {protocol_msg.data.decode().strip()}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send velocity command: {e}")
            return False

    async def _send_motor_command(self, message: BridgeMessage) -> bool:
        """
        Send motor control command (legacy support).
        
        Converts motor command to velocity command.
        """
        try:
            # Extract motor command data
            data = message.data
            motor_id = data.get("motor_id", "all")
            speed = data.get("speed", 0.0)
            
            # Convert to Twist (simplified - assumes differential drive)
            twist = Twist()
            if motor_id == "all":
                twist.linear.x = speed  # Forward speed
            else:
                # Individual motor control not directly supported
                logger.warning(f"Individual motor control for {motor_id} not supported, using as forward speed")
                twist.linear.x = speed
            
            # Send as velocity command
            velocity_msg = BridgeMessage(
                message_type="velocity_command",
                data={"twist": twist}
            )
            return await self._send_velocity_command(velocity_msg)
            
        except Exception as e:
            logger.error(f"Failed to send motor command: {e}")
            return False

    async def _send_emergency_stop(self, message: BridgeMessage) -> bool:
        """Send emergency stop command (zero velocity)."""
        self.emergency_stop_active = True

        # Send zero velocity
        twist = Twist()  # All zeros
        velocity_msg = BridgeMessage(
            message_type="velocity_command",
            data={"twist": twist}
        )
        
        success = await self._send_velocity_command(velocity_msg)
        
        if success:
            logger.warning("Emergency stop sent via CAN")
        
        return success
    
    async def _send_heartbeat(self) -> bool:
        """Send heartbeat message."""
        try:
            protocol_msg = self.protocol_adapter.encode_heartbeat()
            if not protocol_msg:
                return False
            
            await asyncio.to_thread(self.serial_port.write, protocol_msg.data)
            self.stats.messages_sent += 1
            return True
            
        except Exception as e:
            logger.error(f"Failed to send heartbeat: {e}")
            return False
    
    async def _send_homing(self) -> bool:
        """Send homing sequence command."""
        try:
            protocol_msg = self.protocol_adapter.encode_homing_sequence()
            if not protocol_msg:
                return False
            
            await asyncio.to_thread(self.serial_port.write, protocol_msg.data)
            self.stats.messages_sent += 1
            logger.info("Homing sequence initiated")
            return True
            
        except Exception as e:
            logger.error(f"Failed to send homing sequence: {e}")
            return False

    async def _send_sensor_request(self, message: BridgeMessage) -> bool:
        """Request sensor data using protocol adapter."""
        try:
            sensor_type = message.data.get("sensor_type", "velocity")
            
            protocol_msg = self.protocol_adapter.encode_sensor_request(sensor_type)
            if not protocol_msg:
                return False
            
            await asyncio.to_thread(self.serial_port.write, protocol_msg.data)
            self.stats.messages_sent += 1
            return True
            
        except Exception as e:
            logger.error(f"Failed to send sensor request: {e}")
            return False


    async def _process_messages(self) -> None:
        """Process incoming serial/CAN messages."""
        while self.is_connected and self.serial_port:
            try:
                # Read available data (non-blocking)
                data = await asyncio.to_thread(self.serial_port.read, self.serial_port.in_waiting or 1)
                
                if not data:
                    await asyncio.sleep(0.01)
                    continue
                
                # Add to buffer
                self._read_buffer += data
                
                # Process complete frames (delimited by \r)
                while b'\r' in self._read_buffer:
                    # Extract frame
                    frame_end = self._read_buffer.index(b'\r') + 1
                    frame = self._read_buffer[:frame_end]
                    self._read_buffer = self._read_buffer[frame_end:]
                    
                    # Process frame using protocol adapter
                    await self._process_protocol_frame(frame)
                
                # Prevent buffer overflow
                if len(self._read_buffer) > 1024:
                    logger.warning("Read buffer overflow, clearing")
                    self._read_buffer = b''

            except Exception as e:
                logger.error(f"CAN bridge receive error: {e}")
                self.stats.errors += 1
                await asyncio.sleep(0.1)
    
    async def _process_protocol_frame(self, frame: bytes) -> None:
        """Process a complete protocol frame using the adapter."""
        try:
            # Try to decode as velocity feedback
            twist = self.protocol_adapter.decode_velocity_feedback(frame)
            if twist:
                # Create bridge message for routing
                bridge_msg = BridgeMessage(
                    message_type="velocity_feedback",
                    data={"twist": twist},
                    source_bridge=BridgeType.CAN
                )
                await self._route_message(bridge_msg)
                self.stats.messages_received += 1
                return
            
            # Try to decode as sensor data
            sensor_data = self.protocol_adapter.decode_sensor_data(frame, "all")
            if sensor_data:
                bridge_msg = BridgeMessage(
                    message_type="sensor_data",
                    data=sensor_data,
                    source_bridge=BridgeType.CAN
                )
                await self._route_message(bridge_msg)
                self.stats.messages_received += 1
                return
            
            # Frame not recognized (could be heartbeat reply, etc.)
            logger.debug(f"Unrecognized frame: {frame.decode('ascii', errors='ignore').strip()}")
            
        except Exception as e:
            logger.error(f"Failed to process protocol frame: {e}")


    def get_status(self) -> BridgeStatus:
        """Get CAN bridge status."""
        status = super().get_status()

        # Add CAN-specific information
        status.additional_info.update({
            "device_path": self.device_path,
            "protocol": self.protocol_adapter.protocol_type.value,
            "emergency_stop_active": self.emergency_stop_active,
            "motor_count": len(self.motor_commands),
            "sensor_types": list(self.sensor_data.keys()),
            "protocol_stats": self.protocol_adapter.get_stats()
        })

        return status

    def get_motor_status(self) -> Dict[str, Any]:
        """Get current motor status (legacy compatibility)."""
        return self.motor_commands.copy()

    def get_sensor_data(self, sensor_type: str) -> Dict[str, Any]:
        """Get latest sensor data (legacy compatibility)."""
        return self.sensor_data.get(sensor_type, {}).copy()


# Register CAN bridge factory
def create_can_bridge(config: Dict[str, Any]) -> CANBridge:
    """Factory function for CAN bridge."""
    return CANBridge(config)

register_bridge_factory(BridgeType.CAN, create_can_bridge)

# Export key components
__all__ = [
    'CANBridge',
    'CANMessageType',
    'create_can_bridge'
]