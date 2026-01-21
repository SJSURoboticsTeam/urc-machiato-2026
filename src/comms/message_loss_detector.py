#!/usr/bin/env python3
"""
Message Loss Detector - Sequence Number Tracking and Gap Detection

Detects lost messages in sensor data streams using sequence numbers.
Critical for ensuring sensor fusion reliability and detecting network issues.

Features:
- Sequence number validation for each sensor type
- Gap detection with configurable thresholds
- Loss rate monitoring and alerting
- Recovery strategies for lost data
- Performance impact assessment

Author: URC 2026 Communication Reliability Team
"""

import time
import threading
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class SensorType(Enum):
    """Sensor types for sequence tracking."""
    IMU = "imu"
    GPS = "gps"
    BATTERY = "battery"
    WHEEL_ODOM = "wheel_odom"
    TEMPERATURE = "temperature"
    CAMERA = "camera"
    LIDAR = "lidar"
    SLAM = "slam"


@dataclass
class SequenceGap:
    """Represents a detected gap in sequence numbers."""
    sensor_type: SensorType
    expected_sequence: int
    received_sequence: int
    gap_size: int
    timestamp: float
    consecutive_gaps: int


@dataclass
class SequenceStats:
    """Statistics for sequence number tracking."""
    sensor_type: SensorType
    last_sequence: int
    total_messages: int
    total_gaps: int
    consecutive_gaps: int
    largest_gap: int
    loss_rate_percent: float
    last_update_time: float
    gaps_in_last_minute: int
    gaps_in_last_hour: int


class MessageLossDetector:
    """
    Detects message loss using sequence numbers.

    Maintains separate sequence tracking for each sensor type to detect
    gaps that indicate lost messages or network issues.
    """

    def __init__(self, gap_alert_threshold: int = 5, loss_rate_alert_threshold: float = 1.0):
        """
        Initialize message loss detector.

        Args:
            gap_alert_threshold: Alert when gap size exceeds this
            loss_rate_alert_threshold: Alert when loss rate exceeds this percentage
        """
        self.gap_alert_threshold = gap_alert_threshold
        self.loss_rate_alert_threshold = loss_rate_alert_threshold

        # Sequence tracking per sensor
        self.sequence_stats: Dict[SensorType, SequenceStats] = {}
        self.recent_gaps: List[SequenceGap] = []

        # Initialize stats for all sensor types
        for sensor_type in SensorType:
            self.sequence_stats[sensor_type] = SequenceStats(
                sensor_type=sensor_type,
                last_sequence=-1,  # -1 means no messages received yet
                total_messages=0,
                total_gaps=0,
                consecutive_gaps=0,
                largest_gap=0,
                loss_rate_percent=0.0,
                last_update_time=time.time(),
                gaps_in_last_minute=0,
                gaps_in_last_hour=0
            )

        self.lock = threading.RLock()
        self.alert_callbacks: List[Callable[[SequenceGap], None]] = []

        logger.info("MessageLossDetector initialized")

    def register_alert_callback(self, callback: Callable[[SequenceGap], None]):
        """Register callback for gap alerts."""
        with self.lock:
            self.alert_callbacks.append(callback)

    def track_sequence(self, sensor_type: SensorType, sequence_number: int) -> Dict[str, Any]:
        """
        Track sequence number for a sensor type.

        Args:
            sensor_type: Type of sensor
            sequence_number: Sequence number from message

        Returns:
            Dict with tracking results and any detected gaps
        """
        with self.lock:
            stats = self.sequence_stats[sensor_type]
            current_time = time.time()

            result = {
                'gap_detected': False,
                'gap_size': 0,
                'loss_rate_percent': stats.loss_rate_percent,
                'is_first_message': stats.last_sequence == -1
            }

            # Update message count
            stats.total_messages += 1
            stats.last_update_time = current_time

            # Check for sequence gap
            if stats.last_sequence != -1:
                expected_sequence = (stats.last_sequence + 1) % 2**32  # Handle wraparound

                if sequence_number != expected_sequence:
                    # Gap detected
                    gap_size = (sequence_number - expected_sequence) % 2**32

                    # Create gap record
                    gap = SequenceGap(
                        sensor_type=sensor_type,
                        expected_sequence=expected_sequence,
                        received_sequence=sequence_number,
                        gap_size=gap_size,
                        timestamp=current_time,
                        consecutive_gaps=stats.consecutive_gaps + 1
                    )

                    # Update stats
                    stats.total_gaps += 1
                    stats.consecutive_gaps += 1
                    stats.largest_gap = max(stats.largest_gap, gap_size)

                    # Track recent gaps
                    self.recent_gaps.append(gap)
                    if len(self.recent_gaps) > 1000:  # Keep last 1000 gaps
                        self.recent_gaps.pop(0)

                    # Update time-windowed gap counts
                    self._update_gap_counts(sensor_type, current_time)

                    # Calculate loss rate
                    total_expected = stats.total_messages + stats.total_gaps
                    if total_expected > 0:
                        stats.loss_rate_percent = (stats.total_gaps / total_expected) * 100

                    # Alert if gap is significant
                    if gap_size >= self.gap_alert_threshold or stats.loss_rate_percent >= self.loss_rate_alert_threshold:
                        result['gap_detected'] = True
                        result['gap_size'] = gap_size
                        self._trigger_alert(gap)

                    result['gap_detected'] = True
                    result['gap_size'] = gap_size

                else:
                    # No gap, reset consecutive counter
                    stats.consecutive_gaps = 0
            else:
                # First message
                stats.consecutive_gaps = 0

            # Update last sequence
            stats.last_sequence = sequence_number

            return result

    def _update_gap_counts(self, sensor_type: SensorType, current_time: float):
        """Update time-windowed gap counts."""
        stats = self.sequence_stats[sensor_type]

        # Clean old gaps (older than 1 hour)
        cutoff_time = current_time - 3600
        self.recent_gaps = [g for g in self.recent_gaps if g.timestamp > cutoff_time]

        # Count gaps in last minute and hour
        minute_ago = current_time - 60
        hour_ago = current_time - 3600

        stats.gaps_in_last_minute = sum(1 for g in self.recent_gaps
                                       if g.sensor_type == sensor_type and g.timestamp > minute_ago)
        stats.gaps_in_last_hour = sum(1 for g in self.recent_gaps
                                     if g.sensor_type == sensor_type and g.timestamp > hour_ago)

    def _trigger_alert(self, gap: SequenceGap):
        """Trigger alert callbacks for significant gaps."""
        logger.warning(f"Message gap detected: {gap.sensor_type.value} "
                      f"expected={gap.expected_sequence} received={gap.received_sequence} "
                      f"gap_size={gap.gap_size}")

        for callback in self.alert_callbacks:
            try:
                callback(gap)
            except Exception as e:
                logger.error(f"Alert callback failed: {e}")

    def get_sequence_stats(self, sensor_type: SensorType) -> SequenceStats:
        """Get sequence statistics for a sensor type."""
        with self.lock:
            return self.sequence_stats[sensor_type]

    def get_all_sequence_stats(self) -> Dict[SensorType, SequenceStats]:
        """Get sequence statistics for all sensors."""
        with self.lock:
            return self.sequence_stats.copy()

    def get_recent_gaps(self, sensor_type: Optional[SensorType] = None,
                       time_window_seconds: float = 300) -> List[SequenceGap]:
        """Get recent gaps within time window."""
        with self.lock:
            cutoff_time = time.time() - time_window_seconds
            gaps = [g for g in self.recent_gaps if g.timestamp > cutoff_time]

            if sensor_type:
                gaps = [g for g in gaps if g.sensor_type == sensor_type]

            return gaps

    def get_loss_summary(self) -> Dict[str, Any]:
        """Get overall message loss summary."""
        with self.lock:
            total_messages = sum(stats.total_messages for stats in self.sequence_stats.values())
            total_gaps = sum(stats.total_gaps for stats in self.sequence_stats.values())

            if total_messages == 0:
                overall_loss_rate = 0.0
            else:
                overall_loss_rate = (total_gaps / (total_messages + total_gaps)) * 100

            # Find worst performing sensor
            worst_sensor = None
            worst_loss_rate = 0.0

            for sensor_type, stats in self.sequence_stats.items():
                if stats.total_messages > 0:
                    sensor_expected = stats.total_messages + stats.total_gaps
                    sensor_loss_rate = (stats.total_gaps / sensor_expected) * 100 if sensor_expected > 0 else 0

                    if sensor_loss_rate > worst_loss_rate:
                        worst_loss_rate = sensor_loss_rate
                        worst_sensor = sensor_type

            return {
                'total_messages': total_messages,
                'total_gaps': total_gaps,
                'overall_loss_rate_percent': overall_loss_rate,
                'worst_sensor': worst_sensor.value if worst_sensor else None,
                'worst_sensor_loss_rate_percent': worst_loss_rate,
                'sensors_with_gaps': [s.value for s, stats in self.sequence_stats.items() if stats.total_gaps > 0],
                'timestamp': time.time()
            }

    def reset_stats(self, sensor_type: Optional[SensorType] = None):
        """Reset sequence statistics."""
        with self.lock:
            if sensor_type:
                # Reset specific sensor
                self.sequence_stats[sensor_type] = SequenceStats(
                    sensor_type=sensor_type,
                    last_sequence=-1,
                    total_messages=0,
                    total_gaps=0,
                    consecutive_gaps=0,
                    largest_gap=0,
                    loss_rate_percent=0.0,
                    last_update_time=time.time(),
                    gaps_in_last_minute=0,
                    gaps_in_last_hour=0
                )
            else:
                # Reset all sensors
                for sensor_enum in SensorType:
                    self.reset_stats(sensor_enum)

                self.recent_gaps.clear()

        logger.info(f"Reset sequence stats for {sensor_type.value if sensor_type else 'all sensors'}")

    def validate_sequence_integrity(self, sensor_type: SensorType,
                                   expected_sequence: int,
                                   timeout_seconds: float = 5.0) -> bool:
        """
        Validate that expected sequence number is received within timeout.

        Useful for critical messages that must not be lost.

        Args:
            sensor_type: Sensor type to check
            expected_sequence: Expected sequence number
            timeout_seconds: How long to wait

        Returns:
            True if sequence received within timeout, False otherwise
        """
        start_time = time.time()

        while time.time() - start_time < timeout_seconds:
            stats = self.get_sequence_stats(sensor_type)

            if stats.last_sequence >= expected_sequence:
                return True

            time.sleep(0.01)  # Small sleep to avoid busy waiting

        return False

    def export_stats(self) -> Dict[str, Any]:
        """Export all statistics for persistence/debugging."""
        with self.lock:
            return {
                'sequence_stats': {s.value: {
                    'last_sequence': stats.last_sequence,
                    'total_messages': stats.total_messages,
                    'total_gaps': stats.total_gaps,
                    'consecutive_gaps': stats.consecutive_gaps,
                    'largest_gap': stats.largest_gap,
                    'loss_rate_percent': stats.loss_rate_percent,
                    'gaps_in_last_minute': stats.gaps_in_last_minute,
                    'gaps_in_last_hour': stats.gaps_in_last_hour
                } for s, stats in self.sequence_stats.items()},
                'recent_gaps': [{
                    'sensor_type': g.sensor_type.value,
                    'expected_sequence': g.expected_sequence,
                    'received_sequence': g.received_sequence,
                    'gap_size': g.gap_size,
                    'timestamp': g.timestamp,
                    'consecutive_gaps': g.consecutive_gaps
                } for g in self.recent_gaps[-100:]],  # Last 100 gaps
                'loss_summary': self.get_loss_summary(),
                'export_timestamp': time.time()
            }


# Global instance
_message_loss_detector_instance: Optional[MessageLossDetector] = None
_message_loss_detector_lock = threading.Lock()


def get_message_loss_detector(gap_alert_threshold: int = 5,
                             loss_rate_alert_threshold: float = 1.0) -> MessageLossDetector:
    """Get global message loss detector instance."""
    global _message_loss_detector_instance

    if _message_loss_detector_instance is None:
        with _message_loss_detector_lock:
            if _message_loss_detector_instance is None:
                _message_loss_detector_instance = MessageLossDetector(
                    gap_alert_threshold, loss_rate_alert_threshold
                )

    return _message_loss_detector_instance


# Convenience functions
def track_sensor_sequence(sensor_type: SensorType, sequence_number: int) -> Dict[str, Any]:
    """Track sequence number for sensor."""
    detector = get_message_loss_detector()
    return detector.track_sequence(sensor_type, sequence_number)


def get_sensor_sequence_stats(sensor_type: SensorType) -> SequenceStats:
    """Get sequence stats for sensor."""
    detector = get_message_loss_detector()
    return detector.get_sequence_stats(sensor_type)


def get_message_loss_summary() -> Dict[str, Any]:
    """Get overall message loss summary."""
    detector = get_message_loss_detector()
    return detector.get_loss_summary()
