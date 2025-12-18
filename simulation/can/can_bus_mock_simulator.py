#!/usr/bin/env python3
"""
Enhanced CAN Bus Mock Simulator - URC Machiato 2026

MOCK IMPLEMENTATION - Simulates CAN bus communication for rover control systems.
Provides realistic timing, error simulation, and hardware interface mocking.

This is a software simulation of the physical CAN bus hardware and should be
replaced with actual CAN bus hardware interfaces when available.

Author: URC 2026 Autonomy Team
"""

import logging
import random
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)


class CANMessagePriority(Enum):
    """CAN message priority levels (lower number = higher priority)."""

    EMERGENCY_STOP = 0
    SAFETY_CRITICAL = 1
    CONTROL_COMMAND = 2
    STATUS_UPDATE = 3
    DIAGNOSTIC = 4
    DEBUG = 5


class CANMessageType(Enum):
    """Types of CAN messages supported."""

    MOTOR_COMMAND = "motor_command"
    MOTOR_STATUS = "motor_status"
    SENSOR_DATA = "sensor_data"
    SYSTEM_STATUS = "system_status"
    EMERGENCY_STOP = "emergency_stop"
    HEARTBEAT = "heartbeat"
    DIAGNOSTIC = "diagnostic"


@dataclass
class CANMessage:
    """Represents a CAN bus message."""

    message_id: int
    data: bytes
    priority: CANMessagePriority
    message_type: CANMessageType
    timestamp: float
    source_node: str
    destination_node: Optional[str] = None

    def __post_init__(self):
        """Validate message data length."""
        if len(self.data) > 8:
            raise ValueError("CAN message data cannot exceed 8 bytes")


class MotorControllerMock:
    """
    MOCK IMPLEMENTATION - Simulates a motor controller on the CAN bus.

    Provides realistic motor control simulation including:
    - Velocity and position control
    - Current and temperature monitoring
    - Fault simulation and recovery
    - Communication delays and errors
    """

    def __init__(self, node_id: int, motor_name: str):
        self.node_id = node_id
        self.motor_name = motor_name
        self.logger = logging.getLogger(f"{__name__}.{motor_name}")

        # Motor state
        self.velocity_setpoint = 0.0  # rad/s
        self.velocity_actual = 0.0  # rad/s
        self.position = 0.0  # rad
        self.current = 0.0  # A
        self.temperature = 25.0  # °C
        self.voltage = 24.0  # V

        # Motor parameters
        self.max_velocity = 10.0  # rad/s
        self.max_current = 5.0  # A
        self.nominal_voltage = 24.0  # V
        self.gear_ratio = 50.0  # :1

        # Simulation parameters
        self.velocity_ramp_rate = 2.0  # rad/s²
        self.thermal_time_constant = 30.0  # seconds
        self.friction_coefficient = 0.1

        # Fault simulation
        self.fault_active = False
        self.fault_type = None
        self.last_update = time.time()

        self.logger.info(f"🚗 MOCK Motor Controller '{motor_name}' initialized")

    def set_velocity(self, velocity: float) -> bool:
        """
        Set motor velocity setpoint.

        Args:
            velocity: Target velocity in rad/s

        Returns:
            bool: True if command accepted
        """
        if abs(velocity) > self.max_velocity:
            self.logger.warning(f"Velocity {velocity} exceeds max {self.max_velocity}")
            return False

        self.velocity_setpoint = velocity

        # Simulate CAN transmission delay
        time.sleep(random.uniform(0.001, 0.005))

        self.logger.debug(f"Set velocity: {velocity} rad/s")
        return True

    def get_status(self) -> Dict[str, Any]:
        """Get current motor status."""
        # Update simulation
        self._update_simulation()

        return {
            "node_id": self.node_id,
            "motor_name": self.motor_name,
            "velocity_setpoint": self.velocity_setpoint,
            "velocity_actual": self.velocity_actual,
            "position": self.position,
            "current": self.current,
            "temperature": self.temperature,
            "voltage": self.voltage,
            "fault_active": self.fault_active,
            "fault_type": self.fault_type,
            "timestamp": time.time(),
        }

    def emergency_stop(self) -> bool:
        """Execute emergency stop."""
        self.velocity_setpoint = 0.0
        self.fault_active = True
        self.fault_type = "emergency_stop"
        self.logger.warning("🚨 EMERGENCY STOP activated")
        return True

    def clear_faults(self) -> bool:
        """Clear active faults."""
        if self.fault_active:
            self.fault_active = False
            self.fault_type = None
            self.logger.info("Faults cleared")
        return True

    def _update_simulation(self):
        """Update motor simulation state."""
        current_time = time.time()
        dt = current_time - self.last_update
        self.last_update = current_time

        if self.fault_active:
            # Fault state - no movement
            self.velocity_actual = 0.0
            self.current = 0.0
            return

        # Velocity control simulation
        velocity_error = self.velocity_setpoint - self.velocity_actual
        acceleration = min(
            self.velocity_ramp_rate, max(-self.velocity_ramp_rate, velocity_error / 0.1)
        )
        self.velocity_actual += acceleration * dt

        # Position integration
        self.position += self.velocity_actual * dt

        # Current calculation (simplified motor model)
        torque_current = abs(self.velocity_actual) * 0.5  # Base current
        acceleration_current = abs(acceleration) * 0.2  # Acceleration current
        friction_current = self.friction_coefficient  # Friction current
        self.current = torque_current + acceleration_current + friction_current

        # Temperature simulation (thermal model)
        power_loss = self.current * 0.5  # Simplified power loss
        temp_rise = power_loss * dt / self.thermal_time_constant
        temp_fall = (self.temperature - 25.0) * dt / self.thermal_time_constant
        self.temperature += temp_rise - temp_fall

        # Voltage simulation (with some ripple)
        self.voltage = self.nominal_voltage + random.uniform(-0.5, 0.5)

        # Random fault simulation (very low probability)
        if random.random() < 0.0001:  # 0.01% chance per update
            self.fault_active = True
            self.fault_type = random.choice(
                ["overcurrent", "overtemp", "encoder_failure"]
            )
            self.logger.error(f"🚨 SIMULATED FAULT: {self.fault_type}")


class SensorInterfaceMock:
    """
    MOCK IMPLEMENTATION - Simulates sensor interfaces on CAN bus.

    Provides realistic sensor data simulation including:
    - IMU (accelerometer, gyroscope, magnetometer)
    - GPS (position, velocity, satellite data)
    - Encoders (wheel position/speed)
    - Current sensors (motor current monitoring)
    - Temperature sensors
    """

    def __init__(self, node_id: int, sensor_type: str):
        self.node_id = node_id
        self.sensor_type = sensor_type
        self.logger = logging.getLogger(f"{__name__}.{sensor_type}_{node_id}")

        # Sensor state
        self.last_reading: Dict[str, Any] = {}
        self.update_rate = 100.0  # Hz
        self.last_update = time.time()

        # Sensor-specific parameters
        if sensor_type == "imu":
            self.noise_std = {"accel": 0.1, "gyro": 0.01, "mag": 0.5}
        elif sensor_type == "gps":
            self.accuracy = 2.0  # meters CEP
            self.update_rate = 10.0  # GPS updates slower
        elif sensor_type == "encoder":
            self.resolution = 4096  # counts per revolution
            self.noise_std = 1.0  # count noise
        elif sensor_type == "current":
            self.shunt_resistance = 0.01  # ohms
            self.noise_std = 0.01  # A
        elif sensor_type == "temperature":
            self.noise_std = 0.5  # °C

        self.logger.info(f"📡 MOCK Sensor '{sensor_type}' initialized")

    def get_reading(self) -> Dict[str, Any]:
        """Get current sensor reading."""
        current_time = time.time()
        dt = current_time - self.last_update

        # Update at sensor rate
        if dt >= (1.0 / self.update_rate):
            self.last_reading = self._generate_reading()
            self.last_update = current_time

        return {
            "node_id": self.node_id,
            "sensor_type": self.sensor_type,
            "timestamp": current_time,
            **self.last_reading,
        }

    def _generate_reading(self) -> Dict[str, Any]:
        """Generate realistic sensor reading."""
        if self.sensor_type == "imu":
            return self._generate_imu_reading()
        elif self.sensor_type == "gps":
            return self._generate_gps_reading()
        elif self.sensor_type == "encoder":
            return self._generate_encoder_reading()
        elif self.sensor_type == "current":
            return self._generate_current_reading()
        elif self.sensor_type == "temperature":
            return self._generate_temperature_reading()
        else:
            return {"error": f"Unknown sensor type: {self.sensor_type}"}

    def _generate_imu_reading(self) -> Dict[str, Any]:
        """Generate IMU sensor reading."""
        # Gravity vector with noise
        accel_x = random.gauss(0.0, self.noise_std["accel"])
        accel_y = random.gauss(0.0, self.noise_std["accel"])
        accel_z = random.gauss(9.81, self.noise_std["accel"])

        # Angular rates with noise
        gyro_x = random.gauss(0.0, self.noise_std["gyro"])
        gyro_y = random.gauss(0.0, self.noise_std["gyro"])
        gyro_z = random.gauss(0.0, self.noise_std["gyro"])

        # Magnetic field (simplified)
        mag_x = random.gauss(0.0, self.noise_std["mag"])
        mag_y = random.gauss(0.0, self.noise_std["mag"])
        mag_z = random.gauss(50.0, self.noise_std["mag"])  # Earth's field ~50uT

        return {
            "accel_x": accel_x,
            "accel_y": accel_y,
            "accel_z": accel_z,
            "gyro_x": gyro_x,
            "gyro_y": gyro_y,
            "gyro_z": gyro_z,
            "mag_x": mag_x,
            "mag_y": mag_y,
            "mag_z": mag_z,
        }

    def _generate_gps_reading(self) -> Dict[str, Any]:
        """Generate GPS sensor reading."""
        # Simulate position around a base point (Mars-like coordinates)
        base_lat, base_lon = 35.0, -117.0  # Edwards AFB area

        lat_noise = random.gauss(0, self.accuracy / 111000)  # Convert meters to degrees
        lon_noise = random.gauss(0, self.accuracy / 111000)

        return {
            "latitude": base_lat + lat_noise,
            "longitude": base_lon + lon_noise,
            "altitude": random.gauss(100.0, 5.0),
            "hdop": random.uniform(1.0, 3.0),
            "satellites_visible": random.randint(8, 12),
            "fix_quality": random.choice([1, 2, 4]),  # GPS fix types
        }

    def _generate_encoder_reading(self) -> Dict[str, Any]:
        """Generate encoder sensor reading."""
        position = random.gauss(0.0, self.noise_std / self.resolution * 2 * 3.14159)
        velocity = random.gauss(0.0, 0.1)  # rad/s noise

        return {
            "position": position,
            "velocity": velocity,
            "raw_counts": int(position * self.resolution / (2 * 3.14159)),
        }

    def _generate_current_reading(self) -> Dict[str, Any]:
        """Generate current sensor reading."""
        current = random.gauss(2.0, self.noise_std)  # Typical motor current
        voltage = current * self.shunt_resistance

        return {
            "current": current,
            "voltage_drop": voltage,
            "power": current * 24.0,  # Assuming 24V system
        }

    def _generate_temperature_reading(self) -> Dict[str, Any]:
        """Generate temperature sensor reading."""
        temperature = random.gauss(35.0, self.noise_std)  # Typical operating temp

        return {"temperature": temperature, "temperature_f": temperature * 9 / 5 + 32}


class CANBusMockSimulator:
    """
    MOCK IMPLEMENTATION - Complete CAN bus network simulator.

    Simulates the entire CAN bus network with multiple nodes:
    - 6 motor controllers (wheels)
    - 4 arm joint controllers
    - Multiple sensors (IMU, GPS, encoders, current, temperature)
    - Message routing and arbitration
    - Bus loading and timing simulation

    This replaces physical CAN bus hardware for software development and testing.
    """

    def __init__(self):
        self.logger = logging.getLogger(__name__)

        # Initialize motor controllers (6 wheels)
        self.motor_controllers = {}
        wheel_names = [
            "front_left",
            "front_right",
            "middle_left",
            "middle_right",
            "rear_left",
            "rear_right",
        ]

        for i, name in enumerate(wheel_names):
            self.motor_controllers[name] = MotorControllerMock(
                node_id=i + 1, motor_name=name
            )

        # Initialize arm controllers (4 joints)
        self.arm_controllers = {}
        arm_joints = ["shoulder", "elbow", "wrist_pitch", "wrist_roll"]

        for i, joint in enumerate(arm_joints):
            node_id = i + 10  # Different node IDs
            self.arm_controllers[joint] = MotorControllerMock(
                node_id=node_id, motor_name=f"arm_{joint}"
            )

        # Initialize sensors
        self.sensors = {}

        # IMU sensors
        for i in range(2):  # Primary and backup IMU
            self.sensors[f"imu_{i}"] = SensorInterfaceMock(
                node_id=20 + i, sensor_type="imu"
            )

        # GPS sensors
        for i in range(2):  # Primary and backup GPS
            self.sensors[f"gps_{i}"] = SensorInterfaceMock(
                node_id=30 + i, sensor_type="gps"
            )

        # Wheel encoders (one per wheel)
        for i, wheel in enumerate(wheel_names):
            self.sensors[f"encoder_{wheel}"] = SensorInterfaceMock(
                node_id=40 + i, sensor_type="encoder"
            )

        # Current sensors
        for i, motor in enumerate(wheel_names):
            self.sensors[f"current_{motor}"] = SensorInterfaceMock(
                node_id=50 + i, sensor_type="current"
            )

        # Temperature sensors
        for i, motor in enumerate(wheel_names + arm_joints):
            self.sensors[f"temp_{motor}"] = SensorInterfaceMock(
                node_id=60 + i, sensor_type="temperature"
            )

        # CAN bus state
        self.bus_load = 0.0  # Percentage bus utilization
        self.error_count = 0
        self.message_count = 0
        self.last_bus_check = time.time()

        # Message queue for simulation
        self.message_queue = []

        self.logger.info("🚍 MOCK CAN Bus Simulator initialized with full rover network")

    def get_mock_reading(
        self, sensor_type: str, sensor_id: Optional[str] = None
    ) -> Dict[str, Any]:
        """
        Get mock sensor reading.

        Args:
            sensor_type: Type of sensor ("imu", "gps", "motor", etc.)
            sensor_id: Specific sensor identifier

        Returns:
            Dict containing sensor data with mock indicator
        """
        try:
            if sensor_type == "imu":
                sensor_key = sensor_id or "imu_0"
                data = self.sensors[sensor_key].get_reading()
            elif sensor_type == "gps":
                sensor_key = sensor_id or "gps_0"
                data = self.sensors[sensor_key].get_reading()
            elif sensor_type == "motor":
                motor_name = sensor_id or "front_left"
                if motor_name in self.motor_controllers:
                    data = self.motor_controllers[motor_name].get_status()
                else:
                    return {"error": f"Unknown motor: {motor_name}", "mock": True}
            elif sensor_type == "encoder":
                sensor_key = sensor_id or "encoder_front_left"
                data = self.sensors[sensor_key].get_reading()
            elif sensor_type == "current":
                sensor_key = sensor_id or "current_front_left"
                data = self.sensors[sensor_key].get_reading()
            elif sensor_type == "temperature":
                sensor_key = sensor_id or "temp_front_left"
                data = self.sensors[sensor_key].get_reading()
            else:
                return {"error": f"Unknown sensor type: {sensor_type}", "mock": True}

            # Add mock indicator
            data["mock"] = True
            data["simulated"] = True

            # Simulate CAN bus delay
            time.sleep(random.uniform(0.0001, 0.001))  # 0.1-1ms delay

            self.message_count += 1
            return data

        except KeyError as e:
            return {"error": f"Sensor not found: {e}", "mock": True}
        except Exception as e:
            return {"error": f"Simulation error: {e}", "mock": True}

    def set_motor_command(self, motor_name: str, command: Dict[str, Any]) -> bool:
        """
        Send motor command over CAN bus.

        Args:
            motor_name: Name of motor to control
            command: Command dictionary with velocity, position, etc.

        Returns:
            bool: True if command sent successfully
        """
        try:
            if motor_name in self.motor_controllers:
                controller = self.motor_controllers[motor_name]
            elif motor_name in self.arm_controllers:
                controller = self.arm_controllers[motor_name]
            else:
                self.logger.error(f"Unknown motor: {motor_name}")
                return False

            # Extract velocity command
            velocity = command.get("velocity", 0.0)

            # Send command
            success = controller.set_velocity(velocity)

            if success:
                # Simulate CAN bus loading
                self._update_bus_load()

                # Add to message queue for tracking
                self.message_queue.append(
                    {
                        "type": "motor_command",
                        "motor": motor_name,
                        "velocity": velocity,
                        "timestamp": time.time(),
                    }
                )

                # Keep queue size reasonable
                if len(self.message_queue) > 100:
                    self.message_queue = self.message_queue[-50:]

            return success

        except Exception as e:
            self.logger.error(f"Failed to set motor command: {e}")
            self.error_count += 1
            return False

    def emergency_stop_all(self) -> bool:
        """Emergency stop all motors."""
        self.logger.warning("🚨 EMERGENCY STOP ALL MOTORS")

        try:
            for controller in self.motor_controllers.values():
                controller.emergency_stop()

            for controller in self.arm_controllers.values():
                controller.emergency_stop()

            return True
        except Exception as e:
            self.logger.error(f"Emergency stop failed: {e}")
            return False

    def get_bus_status(self) -> Dict[str, Any]:
        """Get CAN bus status."""
        self._update_bus_load()

        return {
            "bus_load": self.bus_load,
            "error_count": self.error_count,
            "message_count": self.message_count,
            "motor_controllers": len(self.motor_controllers),
            "arm_controllers": len(self.arm_controllers),
            "sensors": len(self.sensors),
            "mock": True,
            "simulated": True,
        }

    def _update_bus_load(self):
        """Update bus load simulation."""
        # Simplified bus load calculation based on message frequency
        current_time = time.time()
        time_window = 1.0  # 1 second window

        recent_messages = [
            msg
            for msg in self.message_queue
            if current_time - msg["timestamp"] < time_window
        ]

        # Estimate bus load (rough calculation)
        self.bus_load = min(
            100.0, len(recent_messages) * 2.0
        )  # Max ~50 messages/sec = 100% load
