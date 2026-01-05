#!/usr/bin/env python3
"""
Direct CAN Bridge - Hardware to ROS2 without Intermediate Layers

Eliminates translation overhead by directly interfacing CAN hardware with ROS2.
Provides low-latency, high-reliability motor control and sensor feedback.

Features:
- Direct CAN message handling
- Hardware-specific optimizations
- Minimal latency path
- Failsafe motor control

Author: URC 2026 Direct CAN Team
"""

import time
import struct
from typing import Dict, List, Any, Optional, Callable
from enum import Enum
from dataclasses import dataclass


class CANMessageType(Enum):
    """CAN message types."""
    MOTOR_COMMAND = 0x100
    MOTOR_FEEDBACK = 0x101
    IMU_DATA = 0x200
    ENCODER_DATA = 0x201
    BATTERY_STATUS = 0x300
    SYSTEM_STATUS = 0x400
    EMERGENCY_STOP = 0xFFF


@dataclass
class CANMessage:
    """CAN message structure."""
    arbitration_id: int
    data: bytes
    timestamp: float = None
    is_extended: bool = False

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()


class DirectCANBridge:
    """
    Direct CAN bridge for hardware communication.

    Eliminates intermediate translation layers for maximum performance
    and minimum latency in motor control and sensor feedback loops.
    """

    def __init__(self, can_interface: str = "can0"):
        self.can_interface = can_interface
        self.can_socket = None
        self.is_connected = False

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

        # Callbacks
        self.message_callbacks: Dict[CANMessageType, List[Callable]] = {}

    def create_can_message(self, msg_type: CANMessageType, data: bytes) -> CANMessage:
        """Create a CAN message."""
        return CANMessage(
            arbitration_id=msg_type.value,
            data=data,
            is_extended=False
        )

    def create_motor_control_message(self, motor_id: str, speed: float, current: float = 0.0) -> CANMessage:
        """Create a motor control CAN message."""
        # Pack motor control data (speed as float, current as float)
        data = struct.pack('<ff', speed, current)
        arbitration_id = CANMessageType.MOTOR_COMMAND.value + hash(motor_id) % 4
        return CANMessage(arbitration_id=arbitration_id, data=data)

    def connect(self) -> bool:
        """Connect to CAN interface."""
        try:
            # Try to import and connect to CAN
            import socket
            import fcntl
            import struct

            # Create CAN socket
            self.can_socket = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)

            # Bind to interface
            self.can_socket.bind((self.can_interface,))

            # Set non-blocking mode
            fcntl.fcntl(self.can_socket, fcntl.F_SETFL, os.O_NONBLOCK)

            self.is_connected = True
            print(f"🔗 Connected to CAN interface: {self.can_interface}")
            return True

        except Exception as e:
            print(f"❌ CAN connection failed: {e}")
            self.can_socket = None
            return False

    def disconnect(self):
        """Disconnect from CAN interface."""
        if self.can_socket:
            self.can_socket.close()
            self.can_socket = None
        self.is_connected = False
        print("🔌 Disconnected from CAN interface")

    def send_motor_command(self, motor_id: str, speed: float, current_limit: float = 10.0):
        """Send direct motor command via CAN."""
        if not self.is_connected or motor_id not in self.motor_commands:
            return False

        # Update local state
        self.motor_commands[motor_id]["speed"] = speed
        self.motor_commands[motor_id]["current"] = min(current_limit, abs(speed) * 2.0)

        # Create CAN message
        motor_index = ["left_front", "right_front", "left_rear", "right_rear"].index(motor_id)
        arbitration_id = CANMessageType.MOTOR_COMMAND.value + motor_index

        # Pack data: speed (float32), current_limit (float32)
        data = struct.pack('ff', speed, current_limit)

        message = CANMessage(arbitration_id, data)

        return self._send_can_message(message)

    def send_emergency_stop(self):
        """Send emergency stop command to all motors."""
        self.emergency_stop_active = True

        # Stop all motors immediately
        for motor_id in self.motor_commands:
            self.send_motor_command(motor_id, 0.0)

        # Send global emergency stop
        message = CANMessage(CANMessageType.EMERGENCY_STOP.value, b'\x01')
        self._send_can_message(message)

        print("🚨 EMERGENCY STOP SENT")

    def clear_emergency_stop(self):
        """Clear emergency stop state."""
        self.emergency_stop_active = False
        message = CANMessage(CANMessageType.EMERGENCY_STOP.value, b'\x00')
        self._send_can_message(message)
        print("✅ Emergency stop cleared")

    def request_sensor_data(self, sensor_type: str):
        """Request sensor data update."""
        if sensor_type == "imu":
            # Request IMU data
            message = CANMessage(CANMessageType.IMU_DATA.value, b'\x01')
            self._send_can_message(message)
        elif sensor_type == "encoders":
            # Request encoder data
            message = CANMessage(CANMessageType.ENCODER_DATA.value, b'\x01')
            self._send_can_message(message)
        elif sensor_type == "battery":
            # Request battery status
            message = CANMessage(CANMessageType.BATTERY_STATUS.value, b'\x01')
            self._send_can_message(message)

    def process_incoming_messages(self):
        """Process incoming CAN messages."""
        if not self.is_connected:
            return

        try:
            while True:
                # Try to receive message (non-blocking)
                can_frame = self.can_socket.recv(16)

                # Parse CAN frame
                arbitration_id = struct.unpack('I', can_frame[:4])[0] & 0x1FFFFFFF
                data_length = can_frame[4] & 0x0F
                data = can_frame[8:8+data_length]

                message = CANMessage(arbitration_id, data)

                # Process message
                self._process_can_message(message)

        except BlockingIOError:
            # No more messages available
            pass
        except Exception as e:
            print(f"❌ CAN receive error: {e}")

    def _process_can_message(self, message: CANMessage):
        """Process incoming CAN message."""
        message_type = CANMessageType(message.arbitration_id & 0xF00)  # Extract base type

        if message_type == CANMessageType.MOTOR_FEEDBACK:
            self._process_motor_feedback(message)
        elif message_type == CANMessageType.IMU_DATA:
            self._process_imu_data(message)
        elif message_type == CANMessageType.ENCODER_DATA:
            self._process_encoder_data(message)
        elif message_type == CANMessageType.BATTERY_STATUS:
            self._process_battery_data(message)
        elif message_type == CANMessageType.SYSTEM_STATUS:
            self._process_system_data(message)

        # Trigger callbacks
        if message_type in self.message_callbacks:
            for callback in self.message_callbacks[message_type]:
                try:
                    callback(message)
                except Exception as e:
                    print(f"❌ CAN callback error: {e}")

    def _process_motor_feedback(self, message: CANMessage):
        """Process motor feedback data."""
        motor_index = message.arbitration_id & 0x0FF
        motor_names = ["left_front", "right_front", "left_rear", "right_rear"]

        if motor_index < len(motor_names):
            try:
                # Unpack: actual_speed (float32), current_draw (float32), temperature (float32)
                actual_speed, current_draw, temperature = struct.unpack('fff', message.data[:12])

                motor_name = motor_names[motor_index]
                self.motor_commands[motor_name].update({
                    "actual_speed": actual_speed,
                    "current_draw": current_draw,
                    "temperature": temperature
                })

            except struct.error:
                print(f"❌ Motor feedback parse error for motor {motor_index}")

    def _process_imu_data(self, message: CANMessage):
        """Process IMU sensor data."""
        try:
            # Unpack IMU data: accel_x,y,z, gyro_x,y,z
            accel_x, accel_y, accel_z, gyro_x, gyro_y, gyro_z = struct.unpack('ffffff', message.data[:24])

            self.sensor_data["imu"] = {
                "acceleration": {"x": accel_x, "y": accel_y, "z": accel_z},
                "gyroscope": {"x": gyro_x, "y": gyro_y, "z": gyro_z},
                "timestamp": message.timestamp
            }

        except struct.error:
            print("❌ IMU data parse error")

    def _process_encoder_data(self, message: CANMessage):
        """Process encoder data."""
        try:
            # Unpack encoder data: left_front, right_front, left_rear, right_rear positions
            positions = struct.unpack('ffff', message.data[:16])

            motor_names = ["left_front", "right_front", "left_rear", "right_rear"]
            self.sensor_data["encoders"] = {
                name: {"position": pos, "timestamp": message.timestamp}
                for name, pos in zip(motor_names, positions)
            }

        except struct.error:
            print("❌ Encoder data parse error")

    def _process_battery_data(self, message: CANMessage):
        """Process battery status data."""
        try:
            # Unpack battery data: voltage, current, temperature, soc_percent
            voltage, current, temperature, soc_percent = struct.unpack('ffff', message.data[:16])

            self.sensor_data["battery"] = {
                "voltage": voltage,
                "current": current,
                "temperature": temperature,
                "state_of_charge": soc_percent,
                "timestamp": message.timestamp
            }

        except struct.error:
            print("❌ Battery data parse error")

    def _process_system_data(self, message: CANMessage):
        """Process system status data."""
        try:
            # Unpack system data: cpu_temp, board_temp, uptime_seconds, error_flags
            cpu_temp, board_temp, uptime_seconds, error_flags = struct.unpack('fffi', message.data[:16])

            self.sensor_data["system"] = {
                "cpu_temperature": cpu_temp,
                "board_temperature": board_temp,
                "uptime_seconds": uptime_seconds,
                "error_flags": error_flags,
                "timestamp": message.timestamp
            }

        except struct.error:
            print("❌ System data parse error")

    def _send_can_message(self, message: CANMessage) -> bool:
        """Send CAN message."""
        if not self.is_connected or not self.can_socket:
            return False

        try:
            # Construct CAN frame
            can_id = message.arbitration_id | (1 << 31 if message.is_extended else 0)
            data_length = len(message.data)

            # CAN frame format: arbitration_id (4 bytes) + data_length (1 byte) + padding (3 bytes) + data
            frame = struct.pack('I', can_id)
            frame += bytes([data_length, 0, 0, 0])  # DLC + padding
            frame += message.data.ljust(8, b'\x00')  # Data (padded to 8 bytes)

            self.can_socket.send(frame)
            return True

        except Exception as e:
            print(f"❌ CAN send error: {e}")
            return False

    def register_callback(self, message_type: CANMessageType, callback: Callable):
        """Register callback for CAN message type."""
        if message_type not in self.message_callbacks:
            self.message_callbacks[message_type] = []

        self.message_callbacks[message_type].append(callback)

    def get_motor_status(self) -> Dict[str, Any]:
        """Get current motor status."""
        return self.motor_commands.copy()

    def get_sensor_data(self, sensor_type: str) -> Dict[str, Any]:
        """Get latest sensor data."""
        return self.sensor_data.get(sensor_type, {}).copy()

    def get_system_health(self) -> Dict[str, Any]:
        """Get overall system health from CAN data."""
        health = {
            "can_connected": self.is_connected,
            "emergency_stop": self.emergency_stop_active,
            "motors_ok": True,
            "sensors_ok": True,
            "battery_ok": True
        }

        # Check motor health
        for motor_name, motor_data in self.motor_commands.items():
            temp = motor_data.get("temperature", 25)
            current = motor_data.get("current_draw", 0)
            if temp > 80 or current > 15:  # Overheating or overcurrent
                health["motors_ok"] = False
                break

        # Check sensor data freshness (should be updated within 1 second)
        current_time = time.time()
        for sensor_type, data in self.sensor_data.items():
            if data and current_time - data.get("timestamp", 0) > 1.0:
                health["sensors_ok"] = False
                break

        # Check battery health
        battery = self.sensor_data.get("battery", {})
        if battery:
            soc = battery.get("state_of_charge", 100)
            voltage = battery.get("voltage", 12.0)
            if soc < 10 or voltage < 10.0:
                health["battery_ok"] = False

        return health


# Global CAN bridge instance
_can_bridge = None

def get_direct_can_bridge() -> DirectCANBridge:
    """Get global direct CAN bridge instance."""
    global _can_bridge
    if _can_bridge is None:
        _can_bridge = DirectCANBridge()
    return _can_bridge

def connect_can_bridge(interface: str = "can0") -> bool:
    """Connect CAN bridge to interface."""
    bridge = get_direct_can_bridge()
    return bridge.connect()

# Export key components
__all__ = [
    'DirectCANBridge',
    'CANMessageType',
    'CANMessage',
    'get_direct_can_bridge',
    'connect_can_bridge'
]
