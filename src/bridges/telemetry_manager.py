#!/usr/bin/env python3
"""
Telemetry Manager - URC Competition Telemetry Data Management

Handles telemetry data collection, processing, publishing, and aggregation
for the URC competition bridge. Provides centralized telemetry management
with proper error handling and data validation.
"""

import json
import time
from collections import deque
from typing import Any, Dict, List, Optional

import rclpy
from constants import (
    ADAPTATION_RATE,
    BANDWIDTH_MEASUREMENT_WINDOW_SEC,
    BANDWIDTH_TARGET_UTILIZATION,
    DEFAULT_TELEMETRY_RATE_HZ,
    HEALTH_HISTORY_SIZE,
    LATENCY_HISTORY_SIZE,
    LATENCY_TARGET_MS,
    MAX_TELEMETRY_RATE_HZ,
    MIN_TELEMETRY_RATE_HZ,
    PACKET_LOSS_HISTORY_SIZE,
)
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.publisher import Publisher
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import String


class TelemetryManager:
    """
    Manages telemetry data collection, processing, and publishing.

    Handles all telemetry-related operations including:
    - Data aggregation from multiple sources
    - Adaptive telemetry rate management
    - Network quality monitoring
    - Safe data publishing with error handling
    """

    def __init__(self, node: Node, logger):
        """
        Initialize the Telemetry Manager.

        Args:
            node: ROS2 node instance for publishing
            logger: Logger instance for telemetry operations
        """
        self.node = node
        self.logger = logger
        self._telemetry_update_errors = 0

        # Telemetry data storage
        self.telemetry_data = {
            "timestamp": 0.0,
            "mission_time": 0.0,
            "competition_start_time": time.time(),
            "system_health": "nominal",
            "battery_level": 0.0,
            "current": 0.0,
            "gps_position": {"lat": 0.0, "lon": 0.0, "alt": 0.0},
            "imu_data": {"accel": [0.0, 0.0, 0.0], "gyro": [0.0, 0.0, 0.0]},
            "velocity": {"linear": [0.0, 0.0, 0.0], "angular": [0.0, 0.0, 0.0]},
            "current_mission": "none",
            "mission_status": "idle",
            "samples_collected": 0,
            "cache_used": 0,
            "autonomous_mode": False,
            "emergency_stop": False,
            "boundary_violation": False,
            "terrain_traversability": 0.0,
            "joint_states": [],
            "motor_temperatures": [],
            "system_errors": [],
        }

        # Adaptive telemetry data structures
        self.adaptive_data = {
            "current_rate": DEFAULT_TELEMETRY_RATE_HZ,
            "target_rate": DEFAULT_TELEMETRY_RATE_HZ,
            "bandwidth_history": deque(
                maxlen=int(BANDWIDTH_MEASUREMENT_WINDOW_SEC * 10)
            ),
            "latency_history": deque(maxlen=LATENCY_HISTORY_SIZE),
            "packet_loss_history": deque(maxlen=PACKET_LOSS_HISTORY_SIZE),
            "last_adaptation": time.time(),
            "adaptation_cooldown": 5.0,
            "bandwidth_estimator": None,
            "network_quality_score": 1.0,
        }

        # Publishers
        self.telemetry_pub: Optional[Publisher] = None
        self.summary_pub: Optional[Publisher] = None

        self._initialize_publishers()

    def _initialize_publishers(self) -> None:
        """Initialize ROS2 publishers for telemetry data."""
        try:
            self.telemetry_pub = self.node.create_publisher(
                String, "/competition/telemetry", 10
            )
            self.summary_pub = self.node.create_publisher(
                String, "/competition/telemetry_summary", 10
            )
            self.logger.info("Telemetry publishers initialized")
        except Exception as e:
            self.logger.error(f"Failed to initialize telemetry publishers: {e}")

    def _safe_telemetry_update(
        self, key: str, value: Any, description: str = ""
    ) -> None:
        """
        Safely update telemetry data with proper error handling.

        Args:
            key: Telemetry data key to update
            value: Value to set
            description: Description for logging (optional)
        """
        try:
            self.telemetry_data[key] = value
        except (KeyError, TypeError, ValueError) as e:
            self._telemetry_update_errors += 1
            self.logger.warning(
                f"Failed to update telemetry {key}: {e}. "
                f"Errors this session: {self._telemetry_update_errors}"
            )

    def _safe_json_parse(
        self, json_str: str, description: str = ""
    ) -> Optional[Dict[str, Any]]:
        """
        Safely parse JSON string with error handling.

        Args:
            json_str: JSON string to parse
            description: Optional description for logging

        Returns:
            Parsed dictionary or None if parsing failed
        """
        try:
            return json.loads(json_str)
        except (json.JSONDecodeError, TypeError) as e:
            self.logger.warning(f"Failed to parse JSON ({description}): {e}")
            return None

    def update_battery_data(self, msg: BatteryState) -> None:
        """Update battery telemetry data."""
        self._safe_telemetry_update("battery_level", msg.percentage, "battery level")
        self._safe_telemetry_update("current", msg.current, "battery current")

    def update_gps_data(self, msg: NavSatFix) -> None:
        """Update GPS telemetry data and verify WGS84 compliance."""
        gps_data = {
            "lat": msg.latitude,
            "lon": msg.longitude,
            "alt": msg.altitude,
        }
        self._safe_telemetry_update("gps_position", gps_data, "GPS position")

    def update_imu_data(self, msg: Imu) -> None:
        """Update IMU telemetry data."""
        imu_data = {
            "accel": [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ],
            "gyro": [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
        }
        self._safe_telemetry_update("imu_data", imu_data, "IMU data")

    def update_velocity_data(self, msg: TwistStamped) -> None:
        """Update velocity telemetry data."""
        velocity_data = {
            "linear": [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            "angular": [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z],
        }
        self._safe_telemetry_update("velocity", velocity_data, "velocity data")

    def update_odometry_data(self, msg: Odometry) -> None:
        """Update odometry data for mission time calculation."""
        mission_time = time.time() - self.telemetry_data["competition_start_time"]
        self._safe_telemetry_update("mission_time", mission_time, "mission time")

    def update_mission_status(self, mission: str, status: str) -> None:
        """Update mission status information."""
        self._safe_telemetry_update("current_mission", mission, "current mission")
        self._safe_telemetry_update("mission_status", status, "mission status")

    def update_system_health(
        self, health: str, errors: Optional[List[str]] = None
    ) -> None:
        """Update system health information."""
        self._safe_telemetry_update("system_health", health, "system health")
        if errors is not None:
            self._safe_telemetry_update("system_errors", errors, "system errors")

    def update_emergency_status(
        self, emergency_stop: bool, boundary_violation: bool = False
    ) -> None:
        """Update emergency status information."""
        self._safe_telemetry_update("emergency_stop", emergency_stop, "emergency stop")
        self._safe_telemetry_update(
            "boundary_violation", boundary_violation, "boundary violation"
        )

    def update_sample_data(self, samples_collected: int, cache_used: int) -> None:
        """Update sample collection data."""
        self._safe_telemetry_update(
            "samples_collected", samples_collected, "samples collected"
        )
        self._safe_telemetry_update("cache_used", cache_used, "cache used")

    def publish_telemetry(self) -> None:
        """Publish current telemetry data."""
        try:
            # Update timestamp
            self.telemetry_data["timestamp"] = time.time()

            # Publish full telemetry
            if self.telemetry_pub:
                telemetry_json = json.dumps(self.telemetry_data)
                msg = String()
                msg.data = telemetry_json
                self.telemetry_pub.publish(msg)

            # Publish summary for monitoring
            if self.summary_pub:
                summary = {
                    "mission": self.telemetry_data["current_mission"],
                    "status": self.telemetry_data["mission_status"],
                    "battery": self.telemetry_data["battery_level"],
                    "samples": self.telemetry_data["samples_collected"],
                    "emergency_stop": self.telemetry_data["emergency_stop"],
                    "boundary_violation": self.telemetry_data["boundary_violation"],
                    "timestamp": self.telemetry_data["timestamp"],
                }
                summary_json = json.dumps(summary)
                msg = String()
                msg.data = summary_json
                self.summary_pub.publish(msg)

        except Exception as e:
            self.logger.error(f"Failed to publish telemetry: {e}")

    def update_adaptive_telemetry(
        self,
        bandwidth_mbps: float,
        latency_ms: float,
        packet_loss: float,
        signal_strength: float = 0.0,
    ) -> None:
        """
        Update adaptive telemetry with current network conditions.

        Args:
            bandwidth_mbps: Current bandwidth in Mbps
            latency_ms: Current latency in milliseconds
            packet_loss: Current packet loss ratio (0.0-1.0)
            signal_strength: Signal strength indicator (optional)
        """
        # Update measurement history
        self.adaptive_data["bandwidth_history"].append(bandwidth_mbps)
        self.adaptive_data["latency_history"].append(latency_ms)
        self.adaptive_data["packet_loss_history"].append(packet_loss)

        # Calculate network quality score
        quality_score = self._calculate_network_quality(
            bandwidth_mbps, latency_ms, packet_loss, signal_strength
        )
        self.adaptive_data["network_quality_score"] = quality_score

        # Check if enough time has passed for adaptation
        current_time = time.time()
        if (
            current_time - self.adaptive_data["last_adaptation"]
            < self.adaptive_data["adaptation_cooldown"]
        ):
            return

        # Apply adaptive rate adjustment
        self._adapt_telemetry_rate(
            bandwidth_mbps, latency_ms, packet_loss, quality_score
        )
        self.adaptive_data["last_adaptation"] = current_time

    def _calculate_network_quality(
        self,
        bandwidth: float,
        latency: float,
        packet_loss: float,
        signal_strength: float,
    ) -> float:
        """Calculate overall network quality score (0.0-1.0)."""
        # Bandwidth quality (relative to URC limits)
        bandwidth_score = min(1.0, bandwidth / 10.0)  # Assume 10 Mbps is excellent

        # Latency quality (lower is better)
        if latency <= LATENCY_TARGET_MS:
            latency_score = 1.0
        elif latency <= LATENCY_TARGET_MS * 2:
            latency_score = 0.7
        else:
            latency_score = max(0.0, 1.0 - (latency - LATENCY_TARGET_MS) / 200.0)

        # Packet loss quality (lower is better)
        packet_loss_score = max(0.0, 1.0 - packet_loss * 5)  # 20% loss = 0.0 score

        # Signal strength quality
        signal_score = min(
            1.0, max(0.0, (signal_strength + 50) / 50)
        )  # -50 to 0 dBm range

        # Weighted average
        quality_score = (
            bandwidth_score * 0.4
            + latency_score * 0.3
            + packet_loss_score * 0.2
            + signal_score * 0.1
        )

        return max(0.0, min(1.0, quality_score))

    def _adapt_telemetry_rate(
        self, bandwidth: float, latency: float, packet_loss: float, quality_score: float
    ) -> None:
        """Apply adaptive telemetry rate adjustment."""
        # Calculate optimal rate based on network conditions
        base_rate = DEFAULT_TELEMETRY_RATE_HZ

        # Reduce rate under poor conditions
        if quality_score < 0.5:
            optimal_rate = max(MIN_TELEMETRY_RATE_HZ, base_rate * quality_score)
        elif quality_score > 0.8:
            optimal_rate = min(MAX_TELEMETRY_RATE_HZ, base_rate * 1.2)
        else:
            optimal_rate = base_rate

        # Apply smoothing
        current_rate = self.adaptive_data["current_rate"]
        smoothed_rate = (
            current_rate * (1 - ADAPTATION_RATE) + optimal_rate * ADAPTATION_RATE
        )

        # Only adapt if change is significant (>10% difference)
        rate_change_ratio = abs(smoothed_rate - current_rate) / current_rate
        if rate_change_ratio > 0.1:
            self._apply_telemetry_rate(smoothed_rate)

    def _apply_telemetry_rate(self, new_rate: float) -> None:
        """Apply new telemetry rate."""
        old_rate = self.adaptive_data["current_rate"]
        self.adaptive_data["target_rate"] = new_rate
        self.adaptive_data["current_rate"] = new_rate

        self.logger.info(
            f"Adaptive telemetry: rate changed from {old_rate}Hz to {new_rate}Hz"
        )

    def get_telemetry_summary(self) -> Dict[str, Any]:
        """Get current telemetry summary for status reporting."""
        return {
            "data_points": len(self.telemetry_data),
            "update_errors": self._telemetry_update_errors,
            "adaptive_rate": self.adaptive_data["current_rate"],
            "network_quality": self.adaptive_data["network_quality_score"],
            "samples_collected": self.telemetry_data["samples_collected"],
            "system_health": self.telemetry_data["system_health"],
            "mission_time": self.telemetry_data["mission_time"],
        }
