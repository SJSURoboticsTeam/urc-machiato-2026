"""
Per-sensor timeout, staleness, and confidence for perception and safety.

Tracks last-received time per sensor (IMU, GPS, camera, lidar, etc.),
exposes stale/timeout per source, and provides a simple confidence score (0-1)
for fusion and watchdog use.
"""

import logging
import threading
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Callable, Dict, List, Optional, Set

logger = logging.getLogger(__name__)


class SensorId(Enum):
    """Canonical sensor identifiers for health tracking."""

    IMU = "imu"
    GPS = "gps"
    CAMERA = "camera"
    LIDAR = "lidar"
    SLAM_POSE = "slam_pose"
    ODOM = "odom"
    BATTERY = "battery"
    PROXIMITY = "proximity"


@dataclass
class SensorState:
    """Per-sensor state: last received time and optional validity flag."""

    last_received_ns: int = 0
    valid: bool = True  # False if sensor reported invalid/stale internally


class SensorHealthTracker:
    """
    Tracks last-received time per sensor and exposes staleness and confidence.

    Used by watchdog (per-sensor timeout in addition to global sensor check)
    and by fusion nodes (fallback and confidence weighting).
    """

    def __init__(self, time_source_ns: Optional[Callable[[], int]] = None):
        self._lock = threading.Lock()
        self._state: Dict[str, SensorState] = {}
        self._time_ns = time_source_ns or (lambda: int(time.time() * 1e9))
        self._default_timeout_ns: int = int(2.0 * 1e9)  # 2 s default

    def update(self, sensor_id: str, valid: bool = True) -> None:
        """Record that a message was received for this sensor."""
        now_ns = self._time_ns()
        with self._lock:
            self._state[sensor_id] = SensorState(last_received_ns=now_ns, valid=valid)

    def is_stale(self, sensor_id: str, timeout_sec: Optional[float] = None) -> bool:
        """Return True if the sensor has not been updated within timeout_sec."""
        timeout_ns = int((timeout_sec or 2.0) * 1e9)
        with self._lock:
            s = self._state.get(sensor_id)
            if s is None:
                return True
            return (self._time_ns() - s.last_received_ns) > timeout_ns or not s.valid

    def get_timeout_sensors(self, timeout_sec: Optional[float] = None) -> Set[str]:
        """Return set of sensor ids that are currently stale/timeout."""
        timeout_ns = int((timeout_sec or 2.0) * 1e9)
        now_ns = self._time_ns()
        with self._lock:
            return {
                sid
                for sid, s in self._state.items()
                if (now_ns - s.last_received_ns) > timeout_ns or not s.valid
            }

    def get_confidence(
        self, sensor_id: str, timeout_sec: Optional[float] = None
    ) -> float:
        """
        Return confidence score 0.0--1.0 from age and validity.
        1.0 = just received and valid; decays with age; 0.0 if timeout or invalid.
        """
        timeout_ns = int((timeout_sec or 2.0) * 1e9)
        with self._lock:
            s = self._state.get(sensor_id)
            if s is None or not s.valid:
                return 0.0
            age_ns = self._time_ns() - s.last_received_ns
            if age_ns >= timeout_ns:
                return 0.0
            return max(0.0, 1.0 - (age_ns / timeout_ns))

    def get_all_confidences(
        self, timeout_sec: Optional[float] = None
    ) -> Dict[str, float]:
        """Return confidence per sensor id for blackboard or publishing."""
        with self._lock:
            ids = list(self._state.keys())
        return {sid: self.get_confidence(sid, timeout_sec) for sid in ids}


# Optional process-wide singleton for use by multiple nodes
_tracker: Optional[SensorHealthTracker] = None
_tracker_lock = threading.Lock()


def get_sensor_health_tracker() -> SensorHealthTracker:
    """Return the process-wide sensor health tracker (create if needed)."""
    global _tracker
    with _tracker_lock:
        if _tracker is None:
            _tracker = SensorHealthTracker()
        return _tracker
