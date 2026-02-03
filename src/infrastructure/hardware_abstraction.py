"""
Unified hardware abstraction for motor control and sensor reads.

Defines small Python interfaces so autonomy_core and missions can use
a single API; implement once for CAN/blackboard and once for mock.
Hardware bridges (CAN, blackboard) implement these interfaces.
"""

import logging
import time
from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple

logger = logging.getLogger(__name__)


@dataclass
class SensorReading:
    """Current sensor value with timestamp (unified contract for timeout/fusion)."""

    sensor_id: str
    value: Any  # dict or scalar
    timestamp_ns: int
    valid: bool = True


class IMotorControl(ABC):
    """
    Unified motor control interface.

    Used by autonomy_core and missions; implement for CAN/blackboard and mock.
    """

    @abstractmethod
    def set_velocity(self, axis: str, value: float) -> bool:
        """
        Set velocity for an axis (e.g. "linear_x", "linear_y", "angular_z").
        Returns True if command was accepted/sent.
        """
        pass

    @abstractmethod
    def set_pose(self, arm_joints: Optional[List[float]] = None) -> bool:
        """
        Set arm pose (joint positions) or chassis pose if applicable.
        Returns True if command was accepted/sent.
        """
        pass

    @abstractmethod
    def emergency_stop(self) -> bool:
        """Set all outputs to safe values. Returns True if sent."""
        pass


class ISensorReader(ABC):
    """
    Unified sensor reading pattern.

    Exposes current sensor values with timestamps for timeout and fusion.
    Hardware bridges publish or write to this contract.
    """

    @abstractmethod
    def get_readings(self) -> Dict[str, SensorReading]:
        """Return current sensor readings keyed by sensor_id."""
        pass

    @abstractmethod
    def get_reading(self, sensor_id: str) -> Optional[SensorReading]:
        """Return one sensor reading or None if not available."""
        pass


class MockMotorControl(IMotorControl):
    """Mock motor control for testing; no hardware."""

    def __init__(self) -> None:
        self._last_velocity: Dict[str, float] = {}
        self._last_pose: Optional[List[float]] = None
        self._emergency_stop_called = False

    def set_velocity(self, axis: str, value: float) -> bool:
        self._last_velocity[axis] = value
        logger.debug("MockMotorControl set_velocity %s = %s", axis, value)
        return True

    def set_pose(self, arm_joints: Optional[List[float]] = None) -> bool:
        self._last_pose = arm_joints
        logger.debug("MockMotorControl set_pose %s", arm_joints)
        return True

    def emergency_stop(self) -> bool:
        self._emergency_stop_called = True
        self._last_velocity = {}
        logger.debug("MockMotorControl emergency_stop")
        return True


class MockSensorReader(ISensorReader):
    """Mock sensor reader for testing; returns fixed or configurable readings."""

    def __init__(self) -> None:
        self._readings: Dict[str, SensorReading] = {}
        self._now_ns = lambda: int(time.time() * 1e9)

    def set_reading(self, sensor_id: str, value: Any, valid: bool = True) -> None:
        self._readings[sensor_id] = SensorReading(
            sensor_id=sensor_id,
            value=value,
            timestamp_ns=self._now_ns(),
            valid=valid,
        )

    def get_readings(self) -> Dict[str, SensorReading]:
        return dict(self._readings)

    def get_reading(self, sensor_id: str) -> Optional[SensorReading]:
        return self._readings.get(sensor_id)


def get_motor_control(use_mock: bool = False) -> IMotorControl:
    """Factory: return motor control implementation (mock or real when wired)."""
    if use_mock:
        return MockMotorControl()
    # TODO: when CAN/bridge is wired, return adapter around CAN bridge
    return MockMotorControl()


def get_sensor_reader(use_mock: bool = False) -> ISensorReader:
    """Factory: return sensor reader implementation (mock or real when wired)."""
    if use_mock:
        return MockSensorReader()
    # TODO: when blackboard/bridge is wired, return BlackboardSensorReaderAdapter
    return MockSensorReader()
