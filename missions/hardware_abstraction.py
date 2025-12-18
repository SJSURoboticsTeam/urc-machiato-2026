#!/usr/bin/env python3
"""
Hardware Abstraction Layer for Mission System

Provides mock interfaces for testing mission functionality without physical hardware.
"""

from abc import ABC, abstractmethod
from typing import Dict, Any, Optional, List
import time


class HardwareInterface(ABC):
    """Abstract base class for hardware interfaces."""

    @abstractmethod
    def is_connected(self) -> bool:
        """Check if hardware is connected."""
        pass

    @abstractmethod
    def get_status(self) -> Dict[str, Any]:
        """Get hardware status."""
        pass


class MockSensorInterface(HardwareInterface):
    """Mock sensor interface for testing."""

    def __init__(self, sensor_name: str = "mock_sensor"):
        self.sensor_name = sensor_name
        self.connected = True
        self.last_reading = None

    def is_connected(self) -> bool:
        return self.connected

    def get_status(self) -> Dict[str, Any]:
        return {
            "name": self.sensor_name,
            "connected": self.connected,
            "last_reading": self.last_reading,
            "timestamp": time.time()
        }

    def get_reading(self) -> Optional[float]:
        """Get sensor reading."""
        if not self.connected:
            return None
        self.last_reading = time.time() * 100  # Mock value
        return self.last_reading


class MockActuatorInterface(HardwareInterface):
    """Mock actuator interface for testing."""

    def __init__(self, actuator_name: str = "mock_actuator"):
        self.actuator_name = actuator_name
        self.connected = True
        self.last_command = None

    def is_connected(self) -> bool:
        return self.connected

    def get_status(self) -> Dict[str, Any]:
        return {
            "name": self.actuator_name,
            "connected": self.connected,
            "last_command": self.last_command,
            "timestamp": time.time()
        }

    def send_command(self, command: Any) -> bool:
        """Send command to actuator."""
        if not self.connected:
            return False
        self.last_command = command
        return True


class MockGPSInterface(MockSensorInterface):
    """Mock GPS interface."""

    def __init__(self):
        super().__init__("gps")
        self.latitude = 37.7749
        self.longitude = -122.4194
        self.altitude = 0.0

    def get_reading(self) -> Dict[str, float]:
        """Get GPS position."""
        if not self.connected:
            return None
        self.last_reading = {
            "latitude": self.latitude,
            "longitude": self.longitude,
            "altitude": self.altitude,
            "timestamp": time.time()
        }
        return self.last_reading


class MockIMUInterface(MockSensorInterface):
    """Mock IMU interface."""

    def __init__(self):
        super().__init__("imu")
        self.orientation = {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
        self.linear_acceleration = {"x": 0.0, "y": 0.0, "z": 0.0}
        self.angular_velocity = {"x": 0.0, "y": 0.0, "z": 0.0}

    def get_reading(self) -> Dict[str, Dict[str, float]]:
        """Get IMU data."""
        if not self.connected:
            return None
        self.last_reading = {
            "orientation": self.orientation,
            "linear_acceleration": self.linear_acceleration,
            "angular_velocity": self.angular_velocity,
            "timestamp": time.time()
        }
        return self.last_reading


class MockDriveInterface(MockActuatorInterface):
    """Mock drive interface."""

    def __init__(self):
        super().__init__("drive")
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0

    def set_velocity(self, linear: float, angular: float) -> bool:
        """Set drive velocity."""
        if not self.connected:
            return False
        self.linear_velocity = linear
        self.angular_velocity = angular
        return True


class MockManipulatorInterface(MockActuatorInterface):
    """Mock manipulator interface."""

    def __init__(self):
        super().__init__("manipulator")
        self.joint_positions = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  # 6 joints

    def set_joint_positions(self, positions: List[float]) -> bool:
        """Set joint positions."""
        if not self.connected or len(positions) != len(self.joint_positions):
            return False
        self.joint_positions = positions.copy()
        return True


class HardwareAbstractionLayer:
    """Hardware abstraction layer for mission system."""

    def __init__(self):
        self.sensors: Dict[str, HardwareInterface] = {}
        self.actuators: Dict[str, HardwareInterface] = {}

    def add_sensor(self, name: str, sensor: HardwareInterface):
        """Add a sensor."""
        self.sensors[name] = sensor

    def add_actuator(self, name: str, actuator: HardwareInterface):
        """Add an actuator."""
        self.actuators[name] = actuator

    def get_sensor(self, name: str) -> Optional[HardwareInterface]:
        """Get a sensor by name."""
        return self.sensors.get(name)

    def get_actuator(self, name: str) -> Optional[HardwareInterface]:
        """Get an actuator by name."""
        return self.actuators.get(name)

    def get_all_sensors(self) -> Dict[str, HardwareInterface]:
        """Get all sensors."""
        return self.sensors.copy()

    def get_all_actuators(self) -> Dict[str, HardwareInterface]:
        """Get all actuators."""
        return self.actuators.copy()

    def check_hardware_status(self) -> Dict[str, Any]:
        """Check status of all hardware."""
        sensor_status = {name: sensor.get_status() for name, sensor in self.sensors.items()}
        actuator_status = {name: actuator.get_status() for name, actuator in self.actuators.items()}

        return {
            "sensors": sensor_status,
            "actuators": actuator_status,
            "overall_healthy": all(sensor.get_status()["connected"] for sensor in self.sensors.values()) and
                              all(actuator.get_status()["connected"] for actuator in self.actuators.values())
        }


# Convenience functions for testing
def create_mock_hardware_layer() -> HardwareAbstractionLayer:
    """Create a mock hardware layer with all mock interfaces."""
    hal = HardwareAbstractionLayer()

    # Add mock sensors
    hal.add_sensor("gps", MockGPSInterface())
    hal.add_sensor("imu", MockIMUInterface())

    # Add mock actuators
    hal.add_actuator("drive", MockDriveInterface())
    hal.add_actuator("manipulator", MockManipulatorInterface())

    return hal
