#!/usr/bin/env python3
"""
Motor Controller Hardware Interface

Provides hardware abstraction for motor control using:
- PySerial for serial communication with motor controllers
- python-can for CAN bus communication (competition rover)
- Encoder feedback and PID control

Author: URC 2026 Motor Control Team
"""

import time
import struct
import threading
from typing import List, Optional, Dict, Any, Tuple
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class MotorInterface(Enum):
    """Motor controller interface types."""
    SERIAL = "serial"
    CAN = "can"
    MOCK = "mock"


class MotorController:
    """
    Hardware motor controller interface supporting multiple communication protocols.
    """

    def __init__(self, interface: MotorInterface = MotorInterface.MOCK,
                 config: Optional[Dict[str, Any]] = None):
        """
        Initialize motor controller.

        Args:
            interface: Communication interface type
            config: Interface-specific configuration
        """
        self.interface = interface
        self.config = config or {}
        self.logger = logging.getLogger(f"{__name__}.MotorController")

        # Motor state
        self.connected = False
        self.motor_count = 4  # Default for rover with 4 wheels
        self.motor_speeds = [0.0] * self.motor_count
        self.encoder_counts = [0] * self.motor_count
        self.motor_temperatures = [25.0] * self.motor_count
        self.motor_currents = [0.0] * self.motor_count

        # Communication
        self.serial_conn = None
        self.can_bus = None

        # Monitoring
        self.monitoring_active = False
        self.monitor_thread: Optional[threading.Thread] = None
        self.last_update = time.time()

        # PID control parameters
        self.pid_params = {
            'kp': 1.0,
            'ki': 0.1,
            'kd': 0.05
        }

        # Initialize interface
        self._initialize_interface()

    def _initialize_interface(self):
        """Initialize the specified communication interface."""
        try:
            if self.interface == MotorInterface.SERIAL:
                self._initialize_serial()
            elif self.interface == MotorInterface.CAN:
                self._initialize_can()
            elif self.interface == MotorInterface.MOCK:
                self._initialize_mock()
            else:
                raise ValueError(f"Unsupported interface: {self.interface}")
        except Exception as e:
            self.logger.error(f"Failed to initialize {self.interface.value} interface: {e}")
            # Fall back to mock interface
            self.interface = MotorInterface.MOCK
            self._initialize_mock()

    def _initialize_serial(self):
        """Initialize serial communication."""
        try:
            import serial
        except ImportError:
            raise ImportError("PySerial required for serial motor control")

        port = self.config.get('port', '/dev/ttyACM0')
        baudrate = self.config.get('baudrate', 115200)

        self.serial_conn = serial.Serial(
            port=port,
            baudrate=baudrate,
            timeout=1.0
        )

        self.logger.info(f"Serial motor controller initialized on {port}")
        self.connected = True

    def _initialize_can(self):
        """Initialize CAN bus communication."""
        try:
            import can
        except ImportError:
            raise ImportError("python-can required for CAN motor control")

        interface = self.config.get('interface', 'can0')
        bitrate = self.config.get('bitrate', 500000)

        self.can_bus = can.interface.Bus(
            channel=interface,
            bustype='socketcan',
            bitrate=bitrate
        )

        self.logger.info(f"CAN motor controller initialized on {interface}")
        self.connected = True

    def _initialize_mock(self):
        """Initialize mock interface for testing."""
        self.logger.info("Mock motor controller initialized")
        self.connected = True

    def connect(self) -> bool:
        """Establish connection to motor controllers."""
        if self.connected:
            return True

        try:
            if self.interface == MotorInterface.SERIAL and self.serial_conn:
                # Test serial connection
                self.serial_conn.write(b"PING\n")
                response = self.serial_conn.readline()
                if response:
                    self.connected = True
            elif self.interface == MotorInterface.CAN and self.can_bus:
                # Test CAN connection
                test_msg = can.Message(arbitration_id=0x100, data=[0x01])
                self.can_bus.send(test_msg)
                self.connected = True
            elif self.interface == MotorInterface.MOCK:
                self.connected = True

            if self.connected:
                self.logger.info(f"{self.interface.value.upper()} motor controller connected")
                self.start_monitoring()

        except Exception as e:
            self.logger.error(f"Failed to connect to motor controller: {e}")
            self.connected = False

        return self.connected

    def disconnect(self) -> bool:
        """Disconnect from motor controllers."""
        self.stop_monitoring()

        try:
            # Set all motors to zero speed
            self.set_motor_speeds([0.0] * self.motor_count)

            if self.interface == MotorInterface.SERIAL and self.serial_conn:
                self.serial_conn.close()
            elif self.interface == MotorInterface.CAN and self.can_bus:
                self.can_bus.shutdown()

            self.connected = False
            self.logger.info("Motor controller disconnected")
            return True

        except Exception as e:
            self.logger.error(f"Error during disconnect: {e}")
            return False

    def set_motor_speeds(self, speeds: List[float]) -> bool:
        """
        Set speeds for all motors.

        Args:
            speeds: List of motor speeds (rad/s or m/s depending on motor type)

        Returns:
            True if command sent successfully
        """
        if not self.connected or len(speeds) != self.motor_count:
            return False

        try:
            if self.interface == MotorInterface.SERIAL:
                return self._send_serial_speeds(speeds)
            elif self.interface == MotorInterface.CAN:
                return self._send_can_speeds(speeds)
            elif self.interface == MotorInterface.MOCK:
                return self._send_mock_speeds(speeds)

        except Exception as e:
            self.logger.error(f"Failed to set motor speeds: {e}")
            return False

    def _send_serial_speeds(self, speeds: List[float]) -> bool:
        """Send motor speeds via serial."""
        try:
            # Format: "SPEEDS speed1 speed2 speed3 speed4\n"
            speed_str = " ".join(f"{speed:.3f}" for speed in speeds)
            command = f"SPEEDS {speed_str}\n"

            self.serial_conn.write(command.encode())
            response = self.serial_conn.readline().decode().strip()

            if response == "OK":
                self.motor_speeds = speeds.copy()
                return True

        except Exception as e:
            self.logger.error(f"Serial speed command failed: {e}")

        return False

    def _send_can_speeds(self, speeds: List[float]) -> bool:
        """Send motor speeds via CAN bus."""
        try:
            import can

            # Send individual messages for each motor
            for i, speed in enumerate(speeds):
                # Motor ID offset by 0x100
                arbitration_id = 0x100 + i

                # Pack speed as float (4 bytes)
                data = struct.pack('>f', speed)

                msg = can.Message(arbitration_id=arbitration_id, data=data)
                self.can_bus.send(msg)

            self.motor_speeds = speeds.copy()
            return True

        except Exception as e:
            self.logger.error(f"CAN speed command failed: {e}")
            return False

    def _send_mock_speeds(self, speeds: List[float]) -> bool:
        """Mock implementation for testing."""
        self.logger.debug(f"Mock: Setting motor speeds to {speeds}")
        self.motor_speeds = speeds.copy()

        # Simulate some motor response
        for i in range(len(self.encoder_counts)):
            # Simple encoder simulation
            self.encoder_counts[i] += int(speeds[i] * 10)  # Rough encoder ticks

        return True

    def get_encoder_counts(self) -> List[int]:
        """
        Get current encoder counts for all motors.

        Returns:
            List of encoder counts
        """
        if not self.connected:
            return [0] * self.motor_count

        try:
            if self.interface == MotorInterface.SERIAL:
                return self._read_serial_encoders()
            elif self.interface == MotorInterface.CAN:
                return self._read_can_encoders()
            elif self.interface == MotorInterface.MOCK:
                return self.encoder_counts.copy()

        except Exception as e:
            self.logger.error(f"Failed to read encoders: {e}")

        return [0] * self.motor_count

    def _read_serial_encoders(self) -> List[int]:
        """Read encoder counts via serial."""
        try:
            self.serial_conn.write(b"GET_ENCODERS\n")
            response = self.serial_conn.readline().decode().strip()

            # Parse response like "ENC:1234 5678 9012 3456"
            if response.startswith("ENC:"):
                counts = [int(x) for x in response[4:].split()]
                if len(counts) == self.motor_count:
                    self.encoder_counts = counts
                    return counts

        except Exception as e:
            self.logger.error(f"Serial encoder read failed: {e}")

        return self.encoder_counts.copy()

    def _read_can_encoders(self) -> List[int]:
        """Read encoder counts via CAN bus."""
        try:
            # This would require implementing CAN message parsing
            # For now, return cached values
            return self.encoder_counts.copy()

        except Exception as e:
            self.logger.error(f"CAN encoder read failed: {e}")
            return self.encoder_counts.copy()

    def get_motor_status(self) -> Dict[str, Any]:
        """
        Get comprehensive motor status.

        Returns:
            Dictionary with motor status information
        """
        return {
            'connected': self.connected,
            'interface': self.interface.value,
            'motor_count': self.motor_count,
            'speeds': self.motor_speeds.copy(),
            'encoders': self.get_encoder_counts(),
            'temperatures': self.motor_temperatures.copy(),
            'currents': self.motor_currents.copy(),
            'last_update': self.last_update
        }

    def calibrate_motors(self) -> bool:
        """
        Perform motor calibration sequence.

        Returns:
            True if calibration successful
        """
        try:
            self.logger.info("Starting motor calibration...")

            if self.interface == MotorInterface.SERIAL:
                return self._calibrate_serial()
            elif self.interface == MotorInterface.CAN:
                return self._calibrate_can()
            elif self.interface == MotorInterface.MOCK:
                return self._calibrate_mock()

        except Exception as e:
            self.logger.error(f"Motor calibration failed: {e}")
            return False

    def _calibrate_serial(self) -> bool:
        """Calibrate motors via serial."""
        try:
            self.serial_conn.write(b"CALIBRATE\n")
            response = self.serial_conn.readline().decode().strip()

            if response == "CALIBRATED":
                self.logger.info("Motor calibration completed")
                return True

        except Exception as e:
            self.logger.error(f"Serial calibration failed: {e}")

        return False

    def _calibrate_can(self) -> bool:
        """Calibrate motors via CAN."""
        try:
            import can

            # Send calibration command
            msg = can.Message(arbitration_id=0xFFF, data=[0xCA, 0x1B, 0x00, 0x00])
            self.can_bus.send(msg)

            # Wait for calibration response (simplified)
            time.sleep(2.0)

            self.logger.info("CAN motor calibration completed")
            return True

        except Exception as e:
            self.logger.error(f"CAN calibration failed: {e}")
            return False

    def _calibrate_mock(self) -> bool:
        """Mock calibration for testing."""
        self.logger.info("Mock motor calibration completed")
        time.sleep(0.1)  # Simulate calibration time
        return True

    def start_monitoring(self):
        """Start motor status monitoring."""
        if self.monitoring_active:
            return

        self.monitoring_active = True
        self.monitor_thread = threading.Thread(target=self._monitoring_loop, daemon=True)
        self.monitor_thread.start()

        self.logger.debug("Motor monitoring started")

    def stop_monitoring(self):
        """Stop motor status monitoring."""
        self.monitoring_active = False

        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=1.0)

        self.logger.debug("Motor monitoring stopped")

    def _monitoring_loop(self):
        """Motor monitoring loop."""
        while self.monitoring_active:
            try:
                # Update encoder counts
                self.get_encoder_counts()

                # Update timestamp
                self.last_update = time.time()

                # Check for motor faults (simplified)
                self._check_motor_faults()

                time.sleep(0.1)  # 10 Hz monitoring

            except Exception as e:
                self.logger.error(f"Monitoring loop error: {e}")
                time.sleep(1.0)

    def _check_motor_faults(self):
        """Check for motor faults and issues."""
        # Simplified fault checking
        for i, current in enumerate(self.motor_currents):
            if current > 15.0:  # High current threshold
                self.logger.warning(f"Motor {i} high current: {current}A")

        for i, temp in enumerate(self.motor_temperatures):
            if temp > 70.0:  # High temperature threshold
                self.logger.warning(f"Motor {i} high temperature: {temp}°C")


# Factory function for creating motor controllers
def create_motor_controller(interface_type: str = "mock",
                           config: Optional[Dict[str, Any]] = None) -> MotorController:
    """
    Create a motor controller instance.

    Args:
        interface_type: Type of interface ("serial", "can", "mock")
        config: Interface configuration

    Returns:
        MotorController instance
    """
    interface_map = {
        "serial": MotorInterface.SERIAL,
        "can": MotorInterface.CAN,
        "mock": MotorInterface.MOCK
    }

    interface = interface_map.get(interface_type.lower(), MotorInterface.MOCK)

    return MotorController(interface=interface, config=config or {})

