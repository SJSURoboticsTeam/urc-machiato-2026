"""Perception subsystem: SLAM, vision, sensor health."""

try:
    from .sensor_health import (
        SensorHealthTracker,
        SensorId,
        SensorState,
        get_sensor_health_tracker,
    )

    __all__ = [
        "SensorHealthTracker",
        "SensorId",
        "SensorState",
        "get_sensor_health_tracker",
    ]
except ImportError:
    __all__ = []
