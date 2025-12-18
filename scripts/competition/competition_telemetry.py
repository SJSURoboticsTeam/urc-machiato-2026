#!/usr/bin/env python3
"""
Competition Telemetry System
Collects and analyzes telemetry data during competition runs.
"""

import json
import threading
import time
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import Float32, String


class CompetitionTelemetryCollector(Node):
    """
    Competition telemetry data collector.

    Collects and stores telemetry data during competition runs.
    """

    def __init__(self, session_name: str = "competition_session"):
        super().__init__("competition_telemetry_collector")
        self.session_name = session_name
        self.start_time = time.time()
        self.telemetry_data = {
            "session": session_name,
            "start_time": self.start_time,
            "gps_tracks": [],
            "performance_metrics": [],
            "mission_events": [],
            "error_events": [],
            "sensor_quality": {},
            "system_events": [],
        }

    def record_gps_data(
        self, latitude: float, longitude: float, altitude: float, accuracy: float
    ):
        """Record GPS position data."""
        gps_entry = {
            "timestamp": time.time(),
            "latitude": latitude,
            "longitude": longitude,
            "altitude": altitude,
            "accuracy": accuracy,
        }
        self.telemetry_data["gps_tracks"].append(gps_entry)

    def record_performance_metric(self, metric_name: str, value: float):
        """Record a performance metric."""
        metric_entry = {"timestamp": time.time(), "metric": metric_name, "value": value}
        self.telemetry_data["performance_metrics"].append(metric_entry)

    def record_mission_event(self, event_type: str, details: Dict[str, Any]):
        """Record a mission-related event."""
        event_entry = {"timestamp": time.time(), "type": event_type, "details": details}
        self.telemetry_data["mission_events"].append(event_entry)

    def record_error_event(
        self, error_type: str, message: str, severity: str = "warning"
    ):
        """Record an error or warning event."""
        error_entry = {
            "timestamp": time.time(),
            "type": error_type,
            "message": message,
            "severity": severity,
        }
        self.telemetry_data["error_events"].append(error_entry)

    def update_sensor_quality(self, sensor_name: str, quality_score: float):
        """Update sensor quality metrics."""
        self.telemetry_data["sensor_quality"][sensor_name] = {
            "timestamp": time.time(),
            "quality_score": quality_score,
        }

    def record_system_event(self, event_type: str, details: Dict[str, Any]):
        """Record system-level events."""
        event_entry = {"timestamp": time.time(), "type": event_type, "details": details}
        self.telemetry_data["system_events"].append(event_entry)

    def save_telemetry_data(self, filepath: str):
        """Save telemetry data to file."""
        with open(filepath, "w") as f:
            json.dump(self.telemetry_data, f, indent=2)

    def get_session_statistics(self) -> Dict[str, Any]:
        """Get session statistics."""
        end_time = time.time()
        duration = end_time - self.start_time

        return {
            "session_name": self.session_name,
            "duration_seconds": duration,
            "gps_points_collected": len(self.telemetry_data["gps_tracks"]),
            "mission_events_count": len(self.telemetry_data["mission_events"]),
            "error_events_count": len(self.telemetry_data["error_events"]),
            "performance_metrics_count": len(
                self.telemetry_data["performance_metrics"]
            ),
        }


class CompetitionTelemetryAnalysis:
    """
    Competition telemetry data analysis.
    """

    def __init__(self, telemetry_data: Dict[str, Any]):
        self.telemetry_data = telemetry_data

    def analyze_performance(self) -> Dict[str, Any]:
        """Analyze performance metrics."""
        metrics = self.telemetry_data.get("performance_metrics", [])
        if not metrics:
            return {"status": "no_data"}

        # Basic analysis
        metric_names = set(m["metric"] for m in metrics)
        analysis = {}

        for metric_name in metric_names:
            values = [m["value"] for m in metrics if m["metric"] == metric_name]
            analysis[metric_name] = {
                "count": len(values),
                "average": sum(values) / len(values) if values else 0,
                "min": min(values) if values else 0,
                "max": max(values) if values else 0,
            }

        return analysis

    def analyze_mission_events(self) -> Dict[str, Any]:
        """Analyze mission events."""
        events = self.telemetry_data.get("mission_events", [])
        event_types = {}

        for event in events:
            event_type = event.get("type", "unknown")
            if event_type not in event_types:
                event_types[event_type] = 0
            event_types[event_type] += 1

        return {"total_events": len(events), "event_types": event_types}

    def analyze_errors(self) -> Dict[str, Any]:
        """Analyze error events."""
        errors = self.telemetry_data.get("error_events", [])
        error_types = {}
        severity_counts = {"warning": 0, "error": 0, "critical": 0}

        for error in errors:
            error_type = error.get("type", "unknown")
            severity = error.get("severity", "warning")

            if error_type not in error_types:
                error_types[error_type] = 0
            error_types[error_type] += 1

            if severity in severity_counts:
                severity_counts[severity] += 1

        return {
            "total_errors": len(errors),
            "error_types": error_types,
            "severity_breakdown": severity_counts,
        }

    @classmethod
    def load_from_file(cls, filepath: str) -> "CompetitionTelemetryAnalysis":
        """Load telemetry data from file."""
        with open(filepath, "r") as f:
            data = json.load(f)
        return cls(data)

    def generate_comprehensive_report(self) -> Dict[str, Any]:
        """Generate a comprehensive telemetry report."""
        return {
            "session_info": {
                "name": self.telemetry_data.get("session", "unknown"),
                "start_time": self.telemetry_data.get("start_time", 0),
            },
            "performance_analysis": self.analyze_performance(),
            "mission_analysis": self.analyze_mission_events(),
            "error_analysis": self.analyze_errors(),
            "summary": {
                "total_gps_points": len(self.telemetry_data.get("gps_tracks", [])),
                "total_system_events": len(
                    self.telemetry_data.get("system_events", [])
                ),
            },
        }


class CompetitionTelemetry(Node):
    """
    Competition telemetry collection and monitoring system.

    Tracks:
    - Battery levels and consumption
    - GPS positioning and accuracy
    - IMU sensor quality
    - Mission progress and events
    - Error events and recovery
    - Performance metrics
    """

    def __init__(self):
        super().__init__("competition_telemetry")

        # Data storage
        self.telemetry_data = []
        self.session_start_time = time.time()
        self.mission_events = []
        self.error_events = []
        self.battery_history = []
        self.gps_history = []
        self.performance_metrics = []

        # Current state
        self.current_battery_level = 100.0
        self.current_gps_fix = None
        self.mission_active = False
        self.error_count = 0

        # Subscribers
        self.battery_sub = self.create_subscription(
            BatteryState, "/battery_state", self._battery_callback, 10
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self._gps_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, "/imu/data", self._imu_callback, 10
        )
        self.mission_sub = self.create_subscription(
            String, "/mission/status", self._mission_callback, 10
        )

        # Publishers for telemetry data
        self.telemetry_pub = self.create_publisher(String, "/competition/telemetry", 10)

        # Configuration
        self.telemetry_interval = 1.0  # seconds
        self.data_retention_hours = 24

        # Start telemetry collection
        self.timer = self.create_timer(self.telemetry_interval, self._collect_telemetry)

    def _battery_callback(self, msg: BatteryState):
        """Handle battery state updates."""
        self.current_battery_level = msg.percentage * 100.0
        self.battery_history.append(
            {
                "timestamp": time.time(),
                "level": self.current_battery_level,
                "voltage": msg.voltage,
                "current": msg.current,
            }
        )

        # Keep only recent history
        cutoff_time = time.time() - (self.data_retention_hours * 3600)
        self.battery_history = [
            b for b in self.battery_history if b["timestamp"] > cutoff_time
        ]

    def _gps_callback(self, msg: NavSatFix):
        """Handle GPS position updates."""
        self.current_gps_fix = {
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
            "satellites_visible": msg.position_covariance[0]
            if msg.position_covariance
            else 0,
        }

        self.gps_history.append({"timestamp": time.time(), **self.current_gps_fix})

    def _imu_callback(self, msg: Imu):
        """Handle IMU sensor updates."""
        # Store IMU quality metrics
        imu_data = {
            "timestamp": time.time(),
            "angular_velocity": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z,
            },
            "linear_acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z,
            },
        }
        self.performance_metrics.append(imu_data)

    def _mission_callback(self, msg: String):
        """Handle mission status updates."""
        try:
            data = json.loads(msg.data)
            event = {
                "timestamp": time.time(),
                "type": data.get("type", "unknown"),
                "data": data,
            }
            self.mission_events.append(event)

            if data.get("type") == "mission_start":
                self.mission_active = True
            elif data.get("type") == "mission_complete":
                self.mission_active = False

        except json.JSONDecodeError:
            pass

    def _collect_telemetry(self):
        """Collect and publish telemetry data."""
        telemetry = {
            "timestamp": time.time(),
            "session_duration": time.time() - self.session_start_time,
            "battery_level": self.current_battery_level,
            "gps_fix": self.current_gps_fix,
            "mission_active": self.mission_active,
            "error_count": self.error_count,
            "data_points_collected": len(self.telemetry_data),
        }

        self.telemetry_data.append(telemetry)

        # Publish telemetry
        msg = String()
        msg.data = json.dumps(telemetry)
        self.telemetry_pub.publish(msg)

    def record_error_event(
        self, error_type: str, description: str, severity: str = "warning"
    ):
        """Record an error event."""
        event = {
            "timestamp": time.time(),
            "type": error_type,
            "description": description,
            "severity": severity,
        }
        self.error_events.append(event)
        self.error_count += 1

    def record_mission_event(self, event_type: str, data: Dict[str, Any]):
        """Record a mission event."""
        event = {"timestamp": time.time(), "type": event_type, "data": data}
        self.mission_events.append(event)

    def get_session_statistics(self) -> Dict[str, Any]:
        """Get comprehensive session statistics."""
        session_duration = time.time() - self.session_start_time

        return {
            "session_duration_hours": session_duration / 3600,
            "total_telemetry_points": len(self.telemetry_data),
            "total_mission_events": len(self.mission_events),
            "total_error_events": len(self.error_events),
            "average_battery_level": sum(b["level"] for b in self.battery_history[-10:])
            / max(len(self.battery_history[-10:]), 1),
            "mission_completion_rate": len(
                [e for e in self.mission_events if e["type"] == "mission_complete"]
            )
            / max(
                len([e for e in self.mission_events if e["type"] == "mission_start"]), 1
            ),
            "error_rate_per_hour": self.error_count / max(session_duration / 3600, 1),
        }

    def save_session_data(self, filepath: str):
        """Save all session data to file."""
        data = {
            "session_info": self.get_session_statistics(),
            "telemetry_data": self.telemetry_data,
            "mission_events": self.mission_events,
            "error_events": self.error_events,
            "battery_history": self.battery_history,
            "gps_history": self.gps_history,
        }

        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)


class CompetitionTelemetryAnalyzer:
    """
    Analyzes competition telemetry data for post-run insights.
    """

    def __init__(self, data_file: Optional[str] = None):
        self.data = {}
        if data_file and Path(data_file).exists():
            self.load_data(data_file)

    def load_data(self, filepath: str):
        """Load telemetry data from file."""
        with open(filepath, "r") as f:
            self.data = json.load(f)

    def analyze_mission_performance(self) -> Dict[str, Any]:
        """Analyze mission performance metrics."""
        if not self.data:
            return {"error": "No data loaded"}

        mission_events = self.data.get("mission_events", [])

        return {
            "total_missions": len(
                [e for e in mission_events if e["type"] == "mission_start"]
            ),
            "completed_missions": len(
                [e for e in mission_events if e["type"] == "mission_complete"]
            ),
            "failed_missions": len(
                [e for e in mission_events if e["type"] == "mission_failed"]
            ),
            "average_mission_duration": self._calculate_average_mission_duration(),
        }

    def analyze_error_patterns(self) -> Dict[str, Any]:
        """Analyze error patterns and frequencies."""
        if not self.data:
            return {"error": "No data loaded"}

        error_events = self.data.get("error_events", [])

        error_types = {}
        for event in error_events:
            error_type = event.get("type", "unknown")
            error_types[error_type] = error_types.get(error_type, 0) + 1

        return {
            "total_errors": len(error_events),
            "error_types": error_types,
            "most_common_error": max(error_types.keys(), key=lambda k: error_types[k])
            if error_types
            else None,
        }

    def analyze_performance_trends(self) -> Dict[str, Any]:
        """Analyze performance trends over time."""
        if not self.data:
            return {"error": "No data loaded"}

        telemetry = self.data.get("telemetry_data", [])
        battery_history = self.data.get("battery_history", [])

        return {
            "battery_degradation_rate": self._calculate_battery_degradation(
                battery_history
            ),
            "telemetry_frequency": len(telemetry)
            / max(
                self.data.get("session_info", {}).get("session_duration_hours", 1), 1
            ),
            "error_trend": self._analyze_error_trend(),
        }

    def _calculate_average_mission_duration(self) -> float:
        """Calculate average mission duration."""
        # Implementation would analyze mission start/complete timestamps
        return 0.0  # Placeholder

    def _calculate_battery_degradation(self, battery_history: List[Dict]) -> float:
        """Calculate battery degradation rate."""
        if len(battery_history) < 2:
            return 0.0

        initial = battery_history[0]["level"]
        final = battery_history[-1]["level"]
        duration_hours = (
            battery_history[-1]["timestamp"] - battery_history[0]["timestamp"]
        ) / 3600

        return (initial - final) / max(duration_hours, 1)

    def _analyze_error_trend(self) -> str:
        """Analyze if errors are increasing or decreasing."""
        return "stable"  # Placeholder analysis

    def generate_comprehensive_report(self) -> Dict[str, Any]:
        """Generate comprehensive analysis report."""
        return {
            "mission_analysis": self.analyze_mission_performance(),
            "error_analysis": self.analyze_error_patterns(),
            "performance_analysis": self.analyze_performance_trends(),
            "recommendations": self._generate_recommendations(),
        }

    def _generate_recommendations(self) -> List[str]:
        """Generate recommendations based on analysis."""
        recommendations = []

        # Add recommendations based on analysis results
        error_analysis = self.analyze_error_patterns()
        if error_analysis.get("total_errors", 0) > 10:
            recommendations.append(
                "High error rate detected - investigate system stability"
            )

        perf_analysis = self.analyze_performance_trends()
        if perf_analysis.get("battery_degradation_rate", 0) > 20:
            recommendations.append("High battery degradation - check power management")

        return recommendations
