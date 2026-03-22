"""Sensor fusion stubs (EKF, complementary filter, manager) for tests and integration."""

from __future__ import annotations

import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, Optional, Union

import numpy as np


class SensorType(Enum):
    IMU = "imu"
    GPS = "gps"
    LIDAR = "lidar"
    CAMERA = "camera"


class SensorStatus(Enum):
    HEALTHY = "healthy"
    FAILED = "failed"
    DEGRADED = "degraded"


@dataclass
class SensorMeasurement:
    sensor_type: SensorType
    timestamp: float
    data: Dict[str, Any]
    confidence: float
    status: SensorStatus


@dataclass
class _FilterState:
    position: np.ndarray
    velocity: np.ndarray
    orientation: np.ndarray


class ExtendedKalmanFilter:
    """Minimal 16-state stub (position, velocity, orientation, biases)."""

    def __init__(self) -> None:
        self.state = np.zeros(16)
        self._cov = np.eye(16) * 0.01

    def predict(self, dt: float) -> None:
        _ = dt
        self.state *= 0.9999  # slight decay for numerical stability

    def update_imu(self, measurement: np.ndarray, R: np.ndarray) -> None:
        _ = R
        if measurement.size >= 6:
            self.state[3:6] = measurement[:3] * 0.1
            self.state[6:9] = measurement[3:6] * 0.1

    def get_state(self) -> _FilterState:
        pos = self.state[0:3].copy()
        vel = self.state[3:6].copy()
        quat = self.state[6:10].copy()
        if np.linalg.norm(quat) < 1e-6:
            quat = np.array([1.0, 0.0, 0.0, 0.0])
        quat /= np.linalg.norm(quat)
        return _FilterState(pos, vel, quat)


class ComplementaryFilter:
    def __init__(self, alpha: float = 0.98) -> None:
        self.alpha = alpha
        self._q = np.array([1.0, 0.0, 0.0, 0.0])

    def update(
        self, gyro: np.ndarray, accel: np.ndarray, dt: float
    ) -> np.ndarray:
        _ = dt
        gnorm = np.linalg.norm(accel)
        if gnorm > 1e-6:
            a = accel / gnorm
            # crude tilt quaternion from gravity
            tilt = np.array(
                [max(0.0, a[2]), a[1] * 0.1, -a[0] * 0.1, 0.0]
            )
            tnorm = np.linalg.norm(tilt)
            if tnorm > 1e-6:
                tilt /= tnorm
            self._q = self.alpha * self._q + (1.0 - self.alpha) * tilt
        spin = np.array([1.0, gyro[0] * 0.001, gyro[1] * 0.001, gyro[2] * 0.001])
        self._q = self.alpha * self._q + (1.0 - self.alpha) * spin
        n = np.linalg.norm(self._q)
        if n > 1e-6:
            self._q /= n
        return self._q.copy()


class FusedState:
    def __init__(self) -> None:
        self.position = np.zeros(3, dtype=float)
        self.velocity = np.zeros(3, dtype=float)
        self.orientation = np.array([1.0, 0.0, 0.0, 0.0], dtype=float)


def _norm_type(t: Union[SensorType, str]) -> str:
    if isinstance(t, SensorType):
        return t.value
    return str(t).lower()


class SensorFusionManager:
    def __init__(self, config: Dict[str, Any]) -> None:
        self.config = config
        self._sensors: Dict[str, Dict[str, Any]] = {}
        self._running: bool = False
        self._fused = FusedState()

    def add_sensor(self, name: str, sensor_type: Union[SensorType, str]) -> None:
        self._sensors[name] = {
            "type": _norm_type(sensor_type),
            "status": "healthy",
            "confidence": 1.0,
        }

    def start_fusion(self) -> None:
        self._running = True

    def stop_fusion(self) -> None:
        self._running = False

    def get_sensor_status(self) -> Dict[str, Dict[str, Any]]:
        return {
            k: {
                "type": v["type"],
                "status": v["status"],
                "confidence": v["confidence"],
            }
            for k, v in self._sensors.items()
        }

    def update_sensor_measurement(
        self, name: str, measurement: SensorMeasurement
    ) -> None:
        if name not in self._sensors:
            self.add_sensor(name, measurement.sensor_type)
        st = self._sensors[name]
        st["status"] = measurement.status.value
        st["confidence"] = measurement.confidence
        if measurement.status == SensorStatus.HEALTHY:
            self._fused.position += 0.001

    def get_fused_state(self) -> FusedState:
        return self._fused


def create_sensor_fusion_manager(
    config: Optional[Dict[str, Any]] = None,
) -> SensorFusionManager:
    return SensorFusionManager(config or {})
