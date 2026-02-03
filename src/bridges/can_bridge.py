#!/usr/bin/env python3
"""
CAN Bridge - Direct CAN Bus Communication.

Simplified implementation with inline SLCAN/teleoperation protocol logic.
No abstraction layers; direct serial communication.

Author: URC 2026 CAN Bridge Team
"""

import asyncio
import serial
import struct
import time
from typing import Dict, List, Any, Optional
from dataclasses import dataclass, field
from enum import Enum
import logging

from geometry_msgs.msg import Twist

logger = logging.getLogger(__name__)

# Teleoperation protocol message IDs (SLCAN)
SEND_IDS = {
    "SET_CHASSIS_VELOCITIES": 0x00C,
    "HEARTBEAT": 0x00E,
    "HOMING_SEQUENCE": 0x110,
    "GET_OFFSET": 0x112,
    "GET_ESTIMATED_VELOCITIES": 0x114,
}
RECEIVE_IDS = {
    "SET_VELOCITIES_RESPONSE": 0x00D,
    "HEARTBEAT_REPLY": 0x00F,
    "RETURN_ESTIMATED_CHASSIS_VELOCITIES": 0x115,
}
LINEAR_SCALE = 4096
ANGULAR_SCALE = 64


def _scale_linear(velocity_m_per_s: float, scale_factor: int) -> int:
    scaled = int(velocity_m_per_s * scale_factor)
    return max(-32768, min(32767, scaled))


def _scale_angular_deg(velocity_rad_per_s: float, scale_factor: int) -> int:
    velocity_deg_per_s = velocity_rad_per_s * 57.2957795131
    scaled = int(velocity_deg_per_s * scale_factor)
    return max(-32768, min(32767, scaled))


def _descale_linear(scaled_value: int, scale_factor: int) -> float:
    return float(scaled_value) / float(scale_factor)


def _descale_angular_deg(scaled_value: int, scale_factor: int) -> float:
    velocity_deg_per_s = float(scaled_value) / float(scale_factor)
    return velocity_deg_per_s / 57.2957795131


@dataclass
class BridgeMessage:
    """Message format for CAN bridge (minimal, for API compatibility)."""

    message_type: str
    data: Dict[str, Any]


@dataclass
class BridgeStatus:
    """Bridge status for monitoring."""

    is_connected: bool
    last_message_time: Optional[float] = None
    messages_sent: int = 0
    messages_received: int = 0
    errors: int = 0
    uptime: float = 0.0
    additional_info: Dict[str, Any] = field(default_factory=dict)
    bridge_type: str = "can"


class CANBridge:
    """
    CAN bridge with inline SLCAN/teleoperation protocol.
    Direct implementation; no BridgeInterface or ProtocolAdapter.
    """

    def __init__(self, config: Dict[str, Any]):
        self.config = config
        self.device_path = config.get("device", "/dev/ttyACM0")
        self.fallback_devices = config.get(
            "fallback_devices", ["/dev/ttyAMA10", "/dev/ttyUSB0"]
        )
        self.baudrate = config.get("baudrate", 115200)
        self.serial_port: Optional[serial.Serial] = None
        self.is_connected = False
        self.start_time = time.time()
        self.emergency_stop_active = False
        self._message_task: Optional[asyncio.Task] = None
        self._read_buffer = b""
        self.motor_commands: Dict[str, Dict[str, float]] = {
            "left_front": {"speed": 0.0, "current": 0.0},
            "right_front": {"speed": 0.0, "current": 0.0},
            "left_rear": {"speed": 0.0, "current": 0.0},
            "right_rear": {"speed": 0.0, "current": 0.0},
        }
        self.sensor_data: Dict[str, Dict] = {
            "imu": {},
            "encoders": {},
            "battery": {},
            "system": {},
        }
        self._stats = {"messages_sent": 0, "messages_received": 0, "errors": 0}

    def _encode_velocity_command(self, twist: Twist) -> Optional[bytes]:
        """Encode ROS2 Twist to SLCAN velocity command."""
        try:
            x_vel = twist.linear.x
            y_vel = twist.linear.y
            rot_vel = twist.angular.z
            x_scaled = _scale_linear(x_vel, LINEAR_SCALE)
            y_scaled = _scale_linear(y_vel, LINEAR_SCALE)
            rot_scaled = _scale_angular_deg(rot_vel, ANGULAR_SCALE)
            x_bytes = x_scaled.to_bytes(2, "big", signed=True)
            y_bytes = y_scaled.to_bytes(2, "big", signed=True)
            rot_bytes = rot_scaled.to_bytes(2, "big", signed=True)
            msg_id = SEND_IDS["SET_CHASSIS_VELOCITIES"]
            data_hex = x_bytes.hex() + y_bytes.hex() + rot_bytes.hex()
            slcan_frame = f"t{msg_id:03X}6{data_hex}\r"
            return slcan_frame.encode("ascii")
        except Exception as e:
            logger.error(f"Failed to encode velocity: {e}")
            self._stats["errors"] += 1
            return None

    def _decode_velocity_feedback(self, frame: bytes) -> Optional[Twist]:
        """Decode SLCAN velocity response to Twist."""
        try:
            slcan_frame = frame.decode("ascii").strip()
            if not slcan_frame.startswith("t") or len(slcan_frame) < 17:
                return None
            msg_id = int(slcan_frame[1:4], 16)
            if msg_id not in (
                RECEIVE_IDS["SET_VELOCITIES_RESPONSE"],
                RECEIVE_IDS["RETURN_ESTIMATED_CHASSIS_VELOCITIES"],
            ):
                return None
            data_hex = slcan_frame[5:17]
            if len(data_hex) != 12:
                return None
            x_scaled = int.from_bytes(bytes.fromhex(data_hex[0:4]), "big", signed=True)
            y_scaled = int.from_bytes(bytes.fromhex(data_hex[4:8]), "big", signed=True)
            rot_scaled = int.from_bytes(
                bytes.fromhex(data_hex[8:12]), "big", signed=True
            )
            twist = Twist()
            twist.linear.x = _descale_linear(x_scaled, LINEAR_SCALE)
            twist.linear.y = _descale_linear(y_scaled, LINEAR_SCALE)
            twist.angular.z = _descale_angular_deg(rot_scaled, ANGULAR_SCALE)
            return twist
        except Exception as e:
            logger.debug(f"Decode velocity failed: {e}")
            return None

    def _encode_heartbeat(self) -> bytes:
        msg_id = SEND_IDS["HEARTBEAT"]
        return f"t{msg_id:03X}0\r".encode("ascii")

    def _encode_homing(self) -> bytes:
        msg_id = SEND_IDS["HOMING_SEQUENCE"]
        return f"t{msg_id:03X}80000000000000000\r".encode("ascii")

    def _encode_sensor_request(self, sensor_type: str) -> Optional[bytes]:
        if sensor_type == "velocity":
            msg_id = SEND_IDS["GET_ESTIMATED_VELOCITIES"]
        elif sensor_type == "offset":
            msg_id = SEND_IDS["GET_OFFSET"]
        else:
            return None
        return f"t{msg_id:03X}0\r".encode("ascii")

    async def connect(self) -> bool:
        """Establish serial/CAN connection."""
        for device in [self.device_path] + self.fallback_devices:
            try:
                logger.info(f"Attempting to connect to {device}...")
                self.serial_port = serial.Serial(
                    port=device,
                    baudrate=self.baudrate,
                    timeout=0.1,
                    write_timeout=1.0,
                )
                self.is_connected = True
                self.device_path = device
                logger.info(f"Connected to CAN bridge: {device}")
                self._message_task = asyncio.create_task(self._process_messages())
                return True
            except (serial.SerialException, FileNotFoundError, OSError) as e:
                logger.debug(f"Failed to connect to {device}: {e}")
                continue
            except Exception as e:
                logger.debug(f"Connection error to {device}: {e}")
                continue
        logger.error("Failed to connect to any CAN device")
        return False

    async def disconnect(self) -> None:
        """Cleanly disconnect."""
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

    def shutdown(self) -> None:
        """Synchronous shutdown for signal handlers."""
        self.is_connected = False
        if self._message_task and not self._message_task.done():
            self._message_task.cancel()
        if self.serial_port and self.serial_port.is_open:
            try:
                self.serial_port.close()
            except Exception as e:
                logger.debug(f"Error closing serial on shutdown: {e}")
            self.serial_port = None
        logger.info("CAN bridge shutdown complete")

    async def send_message(self, message: BridgeMessage) -> bool:
        """Send message via serial/CAN."""
        if not self.is_connected or not self.serial_port:
            return False
        try:
            if message.message_type == "velocity_command":
                twist = message.data.get("twist")
                if not twist:
                    return False
                frame = self._encode_velocity_command(twist)
                if not frame:
                    return False
                await asyncio.to_thread(self.serial_port.write, frame)
                self._stats["messages_sent"] += 1
                return True
            elif message.message_type == "motor_command":
                speed = message.data.get("speed", 0.0)
                twist = Twist()
                twist.linear.x = speed
                return await self.send_message(
                    BridgeMessage(
                        message_type="velocity_command", data={"twist": twist}
                    )
                )
            elif message.message_type == "emergency_stop":
                self.emergency_stop_active = True
                return await self.send_message(
                    BridgeMessage(
                        message_type="velocity_command", data={"twist": Twist()}
                    )
                )
            elif message.message_type == "heartbeat":
                frame = self._encode_heartbeat()
                await asyncio.to_thread(self.serial_port.write, frame)
                self._stats["messages_sent"] += 1
                return True
            elif message.message_type == "homing":
                frame = self._encode_homing()
                await asyncio.to_thread(self.serial_port.write, frame)
                self._stats["messages_sent"] += 1
                return True
            elif message.message_type == "sensor_request":
                sensor_type = message.data.get("sensor_type", "velocity")
                frame = self._encode_sensor_request(sensor_type)
                if not frame:
                    return False
                await asyncio.to_thread(self.serial_port.write, frame)
                self._stats["messages_sent"] += 1
                return True
            return False
        except Exception as e:
            logger.error(f"Send error: {e}")
            self._stats["errors"] += 1
            return False

    async def _process_messages(self) -> None:
        """Process incoming serial frames."""
        while self.is_connected and self.serial_port:
            try:
                data = await asyncio.to_thread(
                    self.serial_port.read, self.serial_port.in_waiting or 1
                )
                if not data:
                    await asyncio.sleep(0.01)
                    continue
                self._read_buffer += data
                while b"\r" in self._read_buffer:
                    frame_end = self._read_buffer.index(b"\r") + 1
                    frame = self._read_buffer[:frame_end]
                    self._read_buffer = self._read_buffer[frame_end:]
                    twist = self._decode_velocity_feedback(frame)
                    if twist:
                        self._stats["messages_received"] += 1
                if len(self._read_buffer) > 1024:
                    self._read_buffer = b""
            except Exception as e:
                logger.error(f"Receive error: {e}")
                self._stats["errors"] += 1
                await asyncio.sleep(0.1)

    def get_status(self) -> BridgeStatus:
        """Get bridge status."""
        return BridgeStatus(
            is_connected=self.is_connected,
            last_message_time=None,
            messages_sent=self._stats["messages_sent"],
            messages_received=self._stats["messages_received"],
            errors=self._stats["errors"],
            uptime=time.time() - self.start_time,
            additional_info={
                "device_path": self.device_path,
                "protocol": "teleoperation",
                "emergency_stop_active": self.emergency_stop_active,
                "motor_count": len(self.motor_commands),
                "sensor_types": list(self.sensor_data.keys()),
            },
        )

    def get_motor_status(self) -> Dict[str, Any]:
        return self.motor_commands.copy()

    def get_sensor_data(self, sensor_type: str) -> Dict[str, Any]:
        return self.sensor_data.get(sensor_type, {}).copy()


__all__ = ["CANBridge", "BridgeMessage", "BridgeStatus"]
