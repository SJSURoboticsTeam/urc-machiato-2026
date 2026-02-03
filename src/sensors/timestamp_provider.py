#!/usr/bin/env python3
"""
Sensor Timestamp Provider - Accurate Measurement Time Tagging

Provides accurate timestamps for sensor data to enable proper sensor fusion.
Accounts for hardware latencies and ensures synchronized measurements.

Critical for sensor fusion algorithms that assume simultaneous measurements.

Features:
- Measurement timestamp tagging (when sensor actually measured)
- Hardware latency compensation (IMU: 5ms, GPS: 100ms, etc.)
- Reception timestamp recording (when we received data)
- Latency monitoring and alerting
- Clock synchronization validation

Author: URC 2026 Sensor Fusion Optimization Team
"""

import time
import threading
from typing import Dict, Any, Optional, List
from dataclasses import dataclass
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class SensorType(Enum):
    """Sensor types with known latency characteristics."""

    IMU = "imu"
    GPS = "gps"
    SLAM = "slam"
    CAMERA = "camera"
    WHEEL_ENCODER = "wheel_encoder"
    BATTERY = "battery"
    TEMPERATURE = "temperature"
    LIDAR = "lidar"


@dataclass
class SensorLatencyProfile:
    """Latency profile for a sensor type."""

    typical_latency_ms: float  # Expected measurement latency
    max_latency_ms: float  # Maximum acceptable latency
    jitter_ms: float  # Expected latency variation
    calibration_required: bool  # Whether latency needs calibration
    health_check_interval_ms: int  # How often to check sensor health


@dataclass
class TimestampedSensorData:
    """Sensor data with accurate timestamps."""

    sensor_type: SensorType
    measurement_timestamp_ns: int  # When sensor measured (corrected for latency)
    reception_timestamp_ns: int  # When we received the data
    latency_ms: float  # Calculated latency
    data: Dict[str, Any]  # Original sensor data
    sequence_number: int  # For loss detection
    quality_score: float  # 0.0-1.0 data quality indicator


class SensorTimestampProvider:
    """
    Provides accurate timestamps for sensor measurements.

    Critical for sensor fusion algorithms that require synchronized data.
    """

    # Sensor latency database (calibrated values)
    SENSOR_LATENCIES = {
        SensorType.IMU: SensorLatencyProfile(
            typical_latency_ms=5.0,
            max_latency_ms=15.0,
            jitter_ms=2.0,
            calibration_required=True,
            health_check_interval_ms=1000,
        ),
        SensorType.GPS: SensorLatencyProfile(
            typical_latency_ms=100.0,
            max_latency_ms=200.0,
            jitter_ms=20.0,
            calibration_required=True,
            health_check_interval_ms=5000,
        ),
        SensorType.SLAM: SensorLatencyProfile(
            typical_latency_ms=50.0,
            max_latency_ms=100.0,
            jitter_ms=10.0,
            calibration_required=True,
            health_check_interval_ms=2000,
        ),
        SensorType.CAMERA: SensorLatencyProfile(
            typical_latency_ms=33.0,
            max_latency_ms=66.0,
            jitter_ms=5.0,  # 30Hz frame rate
            calibration_required=False,
            health_check_interval_ms=1000,
        ),
        SensorType.WHEEL_ENCODER: SensorLatencyProfile(
            typical_latency_ms=1.0,
            max_latency_ms=5.0,
            jitter_ms=0.5,
            calibration_required=False,
            health_check_interval_ms=1000,
        ),
        SensorType.BATTERY: SensorLatencyProfile(
            typical_latency_ms=10.0,
            max_latency_ms=50.0,
            jitter_ms=5.0,
            calibration_required=False,
            health_check_interval_ms=10000,
        ),
        SensorType.TEMPERATURE: SensorLatencyProfile(
            typical_latency_ms=100.0,
            max_latency_ms=500.0,
            jitter_ms=50.0,
            calibration_required=False,
            health_check_interval_ms=30000,
        ),
        SensorType.LIDAR: SensorLatencyProfile(
            typical_latency_ms=20.0,
            max_latency_ms=50.0,
            jitter_ms=5.0,
            calibration_required=True,
            health_check_interval_ms=1000,
        ),
    }

    def __init__(self):
        self._latency_calibration: Dict[SensorType, float] = {}
        self._sensor_health: Dict[SensorType, Dict[str, Any]] = {}
        self._sequence_counters: Dict[SensorType, int] = {}
        self._lock = threading.RLock()

        # Initialize with default latencies
        for sensor_type in SensorType:
            self._latency_calibration[sensor_type] = self.SENSOR_LATENCIES[
                sensor_type
            ].typical_latency_ms
            self._sensor_health[sensor_type] = {
                "last_update_ns": 0,
                "latency_measurements": [],
                "quality_score": 1.0,
                "health_status": "unknown",
            }
            self._sequence_counters[sensor_type] = 0

        logger.info("SensorTimestampProvider initialized with default latency profiles")

    def tag_sensor_data(
        self, sensor_type: SensorType, raw_data: Dict[str, Any]
    ) -> TimestampedSensorData:
        """
        Tag sensor data with accurate timestamps and quality metrics.

        Args:
            sensor_type: Type of sensor
            raw_data: Raw sensor data dictionary

        Returns:
            TimestampedSensorData with accurate measurement timestamps
        """
        with self._lock:
            reception_timestamp_ns = time.time_ns()

            # Get calibrated latency for this sensor
            calibrated_latency_ms = self._latency_calibration.get(
                sensor_type, self.SENSOR_LATENCIES[sensor_type].typical_latency_ms
            )

            # Calculate measurement timestamp (reception - latency)
            measurement_timestamp_ns = reception_timestamp_ns - int(
                calibrated_latency_ms * 1_000_000
            )

            # Calculate actual latency (for monitoring)
            latency_ms = (reception_timestamp_ns - measurement_timestamp_ns) / 1_000_000

            # Generate sequence number
            sequence_number = self._sequence_counters[sensor_type]
            self._sequence_counters[sensor_type] = (
                sequence_number + 1
            ) % 2**32  # Wrap at 4 billion

            # Calculate quality score based on latency
            quality_score = self._calculate_quality_score(sensor_type, latency_ms)

            # Update health monitoring
            self._update_sensor_health(
                sensor_type, latency_ms, quality_score, reception_timestamp_ns
            )

            return TimestampedSensorData(
                sensor_type=sensor_type,
                measurement_timestamp_ns=measurement_timestamp_ns,
                reception_timestamp_ns=reception_timestamp_ns,
                latency_ms=latency_ms,
                data=raw_data,
                sequence_number=sequence_number,
                quality_score=quality_score,
            )

    def _calculate_quality_score(
        self, sensor_type: SensorType, latency_ms: float
    ) -> float:
        """
        Calculate data quality score based on latency and sensor characteristics.

        Returns 0.0-1.0 where 1.0 is perfect quality.
        """
        profile = self.SENSOR_LATENCIES[sensor_type]

        # Perfect quality if within typical latency
        if latency_ms <= profile.typical_latency_ms:
            return 1.0

        # Degraded quality if within max latency
        if latency_ms <= profile.max_latency_ms:
            # Linear degradation from typical to max latency
            degradation_factor = (latency_ms - profile.typical_latency_ms) / (
                profile.max_latency_ms - profile.typical_latency_ms
            )
            return max(0.1, 1.0 - degradation_factor * 0.8)  # Never go below 0.1

        # Poor quality if beyond max latency
        return 0.1

    def _update_sensor_health(
        self,
        sensor_type: SensorType,
        latency_ms: float,
        quality_score: float,
        timestamp_ns: int,
    ):
        """Update sensor health monitoring."""
        health = self._sensor_health[sensor_type]

        # Update last update time
        health["last_update_ns"] = timestamp_ns

        # Track recent latency measurements (keep last 100)
        health["latency_measurements"].append(latency_ms)
        if len(health["latency_measurements"]) > 100:
            health["latency_measurements"].pop(0)

        # Update quality score (exponential moving average)
        alpha = 0.1  # Smoothing factor
        health["quality_score"] = (
            alpha * quality_score + (1 - alpha) * health["quality_score"]
        )

        # Determine health status
        profile = self.SENSOR_LATENCIES[sensor_type]
        time_since_update_ms = (time.time_ns() - health["last_update_ns"]) / 1_000_000

        if time_since_update_ms > profile.health_check_interval_ms * 2:
            health["health_status"] = "stale"
        elif health["quality_score"] < 0.3:
            health["health_status"] = "degraded"
        elif health["quality_score"] < 0.8:
            health["health_status"] = "fair"
        else:
            health["health_status"] = "good"

    def calibrate_sensor_latency(
        self, sensor_type: SensorType, measured_latency_ms: float
    ):
        """
        Calibrate sensor latency based on measurements.

        Args:
            sensor_type: Sensor to calibrate
            measured_latency_ms: Measured latency in milliseconds
        """
        with self._lock:
            # Use exponential moving average for calibration
            alpha = 0.1  # Slow adaptation to prevent oscillations
            current_calibration = self._latency_calibration.get(
                sensor_type, self.SENSOR_LATENCIES[sensor_type].typical_latency_ms
            )

            new_calibration = (
                alpha * measured_latency_ms + (1 - alpha) * current_calibration
            )
            self._latency_calibration[sensor_type] = new_calibration

            logger.info(
                f"Calibrated {sensor_type.value} latency to {new_calibration:.1f}ms"
            )

    def get_sensor_health(self, sensor_type: SensorType) -> Dict[str, Any]:
        """Get health status for a sensor."""
        with self._lock:
            health = self._sensor_health[sensor_type].copy()

            # Add computed metrics
            if health["latency_measurements"]:
                health["avg_latency_ms"] = sum(health["latency_measurements"]) / len(
                    health["latency_measurements"]
                )
                health["max_latency_ms"] = max(health["latency_measurements"])
                health["min_latency_ms"] = min(health["latency_measurements"])
            else:
                health["avg_latency_ms"] = 0.0
                health["max_latency_ms"] = 0.0
                health["min_latency_ms"] = 0.0

            # Add time since last update
            health["time_since_update_ms"] = (
                time.time_ns() - health["last_update_ns"]
            ) / 1_000_000

            return health

    def get_all_sensor_health(self) -> Dict[SensorType, Dict[str, Any]]:
        """Get health status for all sensors."""
        return {
            sensor_type: self.get_sensor_health(sensor_type)
            for sensor_type in SensorType
        }

    def reset_sensor_health(self, sensor_type: SensorType):
        """Reset health monitoring for a sensor."""
        with self._lock:
            self._sensor_health[sensor_type] = {
                "last_update_ns": 0,
                "latency_measurements": [],
                "quality_score": 1.0,
                "health_status": "unknown",
            }

    def detect_stale_sensors(self, max_age_ms: float = 5000) -> List[SensorType]:
        """
        Detect sensors that haven't updated recently.

        Args:
            max_age_ms: Maximum age in milliseconds to consider fresh

        Returns:
            List of stale sensor types
        """
        stale_sensors = []
        current_time_ns = time.time_ns()

        for sensor_type in SensorType:
            health = self._sensor_health[sensor_type]
            age_ms = (current_time_ns - health["last_update_ns"]) / 1_000_000

            if age_ms > max_age_ms:
                stale_sensors.append(sensor_type)

        return stale_sensors

    def validate_timestamp_synchronization(
        self,
        sensor_data_list: List[TimestampedSensorData],
        max_time_diff_ms: float = 50.0,
    ) -> Dict[str, Any]:
        """
        Validate that sensor data is synchronized in time.

        Args:
            sensor_data_list: List of timestamped sensor data
            max_time_diff_ms: Maximum allowed time difference

        Returns:
            Validation results with sync status and outliers
        """
        if len(sensor_data_list) < 2:
            return {"synchronized": True, "max_diff_ms": 0.0, "outliers": []}

        # Find median timestamp as reference
        timestamps = [data.measurement_timestamp_ns for data in sensor_data_list]
        timestamps.sort()
        median_timestamp = timestamps[len(timestamps) // 2]

        # Check each timestamp against median
        max_diff_ms = 0.0
        outliers = []

        for data in sensor_data_list:
            diff_ms = abs(data.measurement_timestamp_ns - median_timestamp) / 1_000_000
            max_diff_ms = max(max_diff_ms, diff_ms)

            if diff_ms > max_time_diff_ms:
                outliers.append(
                    {
                        "sensor_type": data.sensor_type.value,
                        "timestamp_diff_ms": diff_ms,
                        "quality_score": data.quality_score,
                    }
                )

        return {
            "synchronized": len(outliers) == 0,
            "max_diff_ms": max_diff_ms,
            "outliers": outliers,
            "reference_timestamp_ns": median_timestamp,
        }

    def export_latency_calibration(self) -> Dict[str, float]:
        """Export current latency calibration for persistence."""
        return {
            sensor_type.value: latency_ms
            for sensor_type, latency_ms in self._latency_calibration.items()
        }

    def import_latency_calibration(self, calibration: Dict[str, float]):
        """Import latency calibration from persistence."""
        with self._lock:
            for sensor_name, latency_ms in calibration.items():
                try:
                    sensor_type = SensorType(sensor_name)
                    self._latency_calibration[sensor_type] = latency_ms
                except ValueError:
                    logger.warning(f"Unknown sensor type in calibration: {sensor_name}")

    def get_latency_stats(self) -> Dict[str, Dict[str, float]]:
        """Get latency statistics for all sensors."""
        stats = {}

        for sensor_type in SensorType:
            health = self.get_sensor_health(sensor_type)
            profile = self.SENSOR_LATENCIES[sensor_type]

            stats[sensor_type.value] = {
                "calibrated_latency_ms": self._latency_calibration[sensor_type],
                "typical_latency_ms": profile.typical_latency_ms,
                "max_latency_ms": profile.max_latency_ms,
                "current_avg_latency_ms": health["avg_latency_ms"],
                "current_max_latency_ms": health["max_latency_ms"],
                "quality_score": health["quality_score"],
                "health_status": health["health_status"],
            }

        return stats


# Global instance for easy access
_timestamp_provider_instance = None
_timestamp_provider_lock = threading.Lock()


def get_timestamp_provider() -> SensorTimestampProvider:
    """Get global timestamp provider instance."""
    global _timestamp_provider_instance

    if _timestamp_provider_instance is None:
        with _timestamp_provider_lock:
            if _timestamp_provider_instance is None:
                _timestamp_provider_instance = SensorTimestampProvider()

    return _timestamp_provider_instance


# Convenience functions
def tag_sensor_data(
    sensor_type: SensorType, raw_data: Dict[str, Any]
) -> TimestampedSensorData:
    """Convenience function to tag sensor data."""
    provider = get_timestamp_provider()
    return provider.tag_sensor_data(sensor_type, raw_data)


def get_sensor_health(sensor_type: SensorType) -> Dict[str, Any]:
    """Convenience function to get sensor health."""
    provider = get_timestamp_provider()
    return provider.get_sensor_health(sensor_type)


def calibrate_sensor_latency(sensor_type: SensorType, measured_latency_ms: float):
    """Convenience function to calibrate sensor latency."""
    provider = get_timestamp_provider()
    provider.calibrate_sensor_latency(sensor_type, measured_latency_ms)


def detect_stale_sensors(max_age_ms: float = 5000) -> List[SensorType]:
    """Convenience function to detect stale sensors."""
    provider = get_timestamp_provider()
    return provider.detect_stale_sensors(max_age_ms)
