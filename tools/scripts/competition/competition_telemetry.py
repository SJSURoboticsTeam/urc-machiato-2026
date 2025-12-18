#!/usr/bin/env python3
"""
Competition Telemetry System - Competition Ready
Simple telemetry collection and analysis for competition performance.
"""

import json
import statistics
import threading
import time
from datetime import datetime
from pathlib import Path
from typing import Any, Dict, List, Optional

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String


class CompetitionTelemetryCollector(Node):
    """
    Simple competition telemetry collector.

    Collects:
    - GPS tracks and waypoints
    - System performance metrics
    - Mission progress and timing
    - Error events and recovery actions
    - Sensor data quality metrics
    """

    def __init__(self, session_name: str = None):
        super().__init__("competition_telemetry_collector")

        self.session_name = session_name or f"competition_{int(time.time())}"
        self.start_time = time.time()
        self.telemetry_data = {
            "session": self.session_name,
            "start_time": datetime.fromtimestamp(self.start_time).isoformat(),
            "gps_tracks": [],
            "performance_metrics": [],
            "mission_events": [],
            "error_events": [],
            "sensor_quality": [],
            "system_events": [],
        }

        # Subscribers for telemetry data
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self._gps_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, "/imu/data", self._imu_callback, 10
        )
        self.battery_sub = self.create_subscription(
            Float32, "/battery/status", self._battery_callback, 10
        )
        self.mission_sub = self.create_subscription(
            String, "/mission/status", self._mission_callback, 10
        )
        self.state_sub = self.create_subscription(
            String, "/state_machine/current_state", self._state_callback, 10
        )
        self.emergency_sub = self.create_subscription(
            String, "/emergency/status", self._emergency_callback, 10
        )

        # Performance monitoring
        self.performance_timer = self.create_timer(
            5.0, self._collect_performance_metrics
        )

        # Auto-save timer (every 30 seconds)
        self.save_timer = self.create_timer(30.0, self._auto_save)

        self.get_logger().info(
            f"Competition telemetry collection started: {self.session_name}"
        )

    def _gps_callback(self, msg):
        """Collect GPS position data."""
        gps_point = {
            "timestamp": time.time(),
            "elapsed_time": time.time() - self.start_time,
            "latitude": msg.latitude,
            "longitude": msg.longitude,
            "altitude": msg.altitude,
            "quality": {"status": msg.status.status, "service": msg.status.service},
        }

        self.telemetry_data["gps_tracks"].append(gps_point)

    def _imu_callback(self, msg):
        """Collect IMU sensor data."""
        imu_data = {
            "timestamp": time.time(),
            "elapsed_time": time.time() - self.start_time,
            "acceleration": {
                "x": msg.linear_acceleration.x,
                "y": msg.linear_acceleration.y,
                "z": msg.linear_acceleration.z,
            },
            "gyroscope": {
                "x": msg.angular_velocity.x,
                "y": msg.angular_velocity.y,
                "z": msg.angular_velocity.z,
            },
        }

        # Track sensor quality (acceleration magnitude should be ~9.8 when stationary)
        accel_magnitude = (
            msg.linear_acceleration.x**2
            + msg.linear_acceleration.y**2
            + msg.linear_acceleration.z**2
        ) ** 0.5

        quality_point = {
            "timestamp": time.time(),
            "sensor": "imu",
            "accel_magnitude": accel_magnitude,
            "expected_gravity": 9.8,
            "deviation": abs(accel_magnitude - 9.8),
        }

        self.telemetry_data["sensor_quality"].append(quality_point)

    def _battery_callback(self, msg):
        """Collect battery status."""
        battery_data = {
            "timestamp": time.time(),
            "elapsed_time": time.time() - self.start_time,
            "voltage": msg.data,
            "level": self._estimate_battery_level(msg.data),
        }

        # Add to performance metrics
        self.telemetry_data["performance_metrics"].append(
            {
                "timestamp": time.time(),
                "metric": "battery_voltage",
                "value": msg.data,
                "unit": "V",
            }
        )

    def _mission_callback(self, msg):
        """Collect mission status updates."""
        try:
            mission_data = json.loads(msg.data)
            mission_event = {
                "timestamp": time.time(),
                "elapsed_time": time.time() - self.start_time,
                "event_type": "mission_update",
                "data": mission_data,
            }

            self.telemetry_data["mission_events"].append(mission_event)

        except json.JSONDecodeError:
            pass

    def _state_callback(self, msg):
        """Collect state machine transitions."""
        try:
            state_data = json.loads(msg.data)
            state_event = {
                "timestamp": time.time(),
                "elapsed_time": time.time() - self.start_time,
                "event_type": "state_change",
                "data": state_data,
            }

            self.telemetry_data["system_events"].append(state_event)

        except json.JSONDecodeError:
            pass

    def _emergency_callback(self, msg):
        """Collect emergency system events."""
        try:
            emergency_data = json.loads(msg.data)
            emergency_event = {
                "timestamp": time.time(),
                "elapsed_time": time.time() - self.start_time,
                "event_type": "emergency_event",
                "data": emergency_data,
            }

            if emergency_data.get("level") != "normal":
                self.telemetry_data["error_events"].append(emergency_event)

        except json.JSONDecodeError:
            pass

    def _collect_performance_metrics(self):
        """Collect system performance metrics."""
        # Basic performance metrics (would be enhanced with psutil in real implementation)
        perf_data = {
            "timestamp": time.time(),
            "elapsed_time": time.time() - self.start_time,
            "metric": "system_uptime",
            "value": time.time() - self.start_time,
            "unit": "seconds",
        }

        self.telemetry_data["performance_metrics"].append(perf_data)

    def _auto_save(self):
        """Auto-save telemetry data periodically."""
        self.save_telemetry_data()
        self.get_logger().info("Telemetry auto-saved")

    def record_mission_start(self, mission_type: str, mission_id: str):
        """Record mission start event."""
        event = {
            "timestamp": time.time(),
            "elapsed_time": time.time() - self.start_time,
            "event_type": "mission_start",
            "mission_type": mission_type,
            "mission_id": mission_id,
        }

        self.telemetry_data["mission_events"].append(event)

    def record_mission_end(self, mission_id: str, success: bool, duration: float):
        """Record mission completion."""
        event = {
            "timestamp": time.time(),
            "elapsed_time": time.time() - self.start_time,
            "event_type": "mission_end",
            "mission_id": mission_id,
            "success": success,
            "duration": duration,
        }

        self.telemetry_data["mission_events"].append(event)

    def record_error_event(
        self, error_type: str, description: str, severity: str = "warning"
    ):
        """Record an error event."""
        error_event = {
            "timestamp": time.time(),
            "elapsed_time": time.time() - self.start_time,
            "event_type": "error",
            "error_type": error_type,
            "description": description,
            "severity": severity,
        }

        self.telemetry_data["error_events"].append(error_event)

    def save_telemetry_data(self, filename: str = None) -> str:
        """Save telemetry data to file."""
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"competition_telemetry_{self.session_name}_{timestamp}.json"

        # Add session summary
        self.telemetry_data["end_time"] = datetime.now().isoformat()
        self.telemetry_data["duration_seconds"] = time.time() - self.start_time
        self.telemetry_data["summary"] = self._generate_summary()

        with open(filename, "w") as f:
            json.dump(self.telemetry_data, f, indent=2, default=str)

        self.get_logger().info(f"Telemetry saved to: {filename}")
        return filename

    def _generate_summary(self) -> Dict[str, Any]:
        """Generate session summary statistics."""
        summary = {
            "total_duration_s": time.time() - self.start_time,
            "gps_points_collected": len(self.telemetry_data["gps_tracks"]),
            "mission_events": len(self.telemetry_data["mission_events"]),
            "error_events": len(self.telemetry_data["error_events"]),
            "performance_metrics": len(self.telemetry_data["performance_metrics"]),
        }

        # GPS track analysis
        if self.telemetry_data["gps_tracks"]:
            lats = [p["latitude"] for p in self.telemetry_data["gps_tracks"]]
            lons = [p["longitude"] for p in self.telemetry_data["gps_tracks"]]

            summary["gps_bounds"] = {
                "lat_min": min(lats),
                "lat_max": max(lats),
                "lon_min": min(lons),
                "lon_max": max(lons),
            }

        # Mission success analysis
        mission_events = self.telemetry_data["mission_events"]
        if mission_events:
            mission_starts = [
                e for e in mission_events if e["event_type"] == "mission_start"
            ]
            mission_ends = [
                e for e in mission_events if e["event_type"] == "mission_end"
            ]

            summary["missions_attempted"] = len(mission_starts)
            summary["missions_completed"] = len(mission_ends)
            summary["mission_success_rate"] = (
                len(mission_ends) / len(mission_starts) if mission_starts else 0
            )

        # Error analysis
        error_events = self.telemetry_data["error_events"]
        if error_events:
            severity_counts = {}
            for event in error_events:
                severity = event.get("severity", "unknown")
                severity_counts[severity] = severity_counts.get(severity, 0) + 1

            summary["error_breakdown"] = severity_counts

        return summary

    def get_session_stats(self) -> Dict[str, Any]:
        """Get current session statistics."""
        return {
            "session_name": self.session_name,
            "duration_s": time.time() - self.start_time,
            "data_points": {
                "gps_tracks": len(self.telemetry_data["gps_tracks"]),
                "performance_metrics": len(self.telemetry_data["performance_metrics"]),
                "mission_events": len(self.telemetry_data["mission_events"]),
                "error_events": len(self.telemetry_data["error_events"]),
                "sensor_quality": len(self.telemetry_data["sensor_quality"]),
            },
        }

    def _estimate_battery_level(self, voltage: float) -> float:
        """Estimate battery level from voltage (simplified)."""
        # LiPo battery estimation (adjust for your battery specs)
        if voltage >= 12.6:
            return 100.0
        elif voltage >= 11.8:
            return 80.0 + (voltage - 11.8) * 50  # Linear interpolation
        elif voltage >= 11.0:
            return 20.0 + (voltage - 11.0) * 60
        else:
            return max(0.0, (voltage - 10.0) * 100)  # Emergency range


class CompetitionTelemetryAnalyzer:
    """
    Simple telemetry analysis for post-competition review.
    """

    def __init__(self, telemetry_file: str):
        self.telemetry_file = telemetry_file
        self.data = None

    def load_data(self) -> bool:
        """Load telemetry data from file."""
        try:
            with open(self.telemetry_file, "r") as f:
                self.data = json.load(f)
            return True
        except Exception as e:
            print(f"Error loading telemetry data: {e}")
            return False

    def generate_competition_report(self) -> Dict[str, Any]:
        """Generate comprehensive competition report."""
        if not self.data:
            return {"error": "No data loaded"}

        report = {
            "session_info": {
                "name": self.data.get("session"),
                "start_time": self.data.get("start_time"),
                "duration_s": self.data.get("duration_seconds", 0),
                "duration_minutes": self.data.get("duration_seconds", 0) / 60,
            },
            "performance_analysis": self._analyze_performance(),
            "mission_analysis": self._analyze_missions(),
            "error_analysis": self._analyze_errors(),
            "recommendations": self._generate_recommendations(),
        }

        return report

    def _analyze_performance(self) -> Dict[str, Any]:
        """Analyze system performance."""
        perf_metrics = self.data.get("performance_metrics", [])

        analysis = {
            "total_metrics": len(perf_metrics),
            "battery_analysis": {},
            "system_stability": {},
        }

        # Battery analysis
        battery_metrics = [
            m for m in perf_metrics if m.get("metric") == "battery_voltage"
        ]
        if battery_metrics:
            voltages = [m["value"] for m in battery_metrics]
            analysis["battery_analysis"] = {
                "min_voltage": min(voltages),
                "max_voltage": max(voltages),
                "avg_voltage": statistics.mean(voltages),
                "voltage_drop": max(voltages) - min(voltages),
            }

        return analysis

    def _analyze_missions(self) -> Dict[str, Any]:
        """Analyze mission performance."""
        mission_events = self.data.get("mission_events", [])

        analysis = {
            "total_events": len(mission_events),
            "missions_attempted": 0,
            "missions_completed": 0,
            "success_rate": 0.0,
            "avg_mission_duration": 0.0,
        }

        mission_starts = {}
        mission_durations = []

        for event in mission_events:
            if event["event_type"] == "mission_start":
                mission_starts[event.get("mission_id", "unknown")] = event["timestamp"]
                analysis["missions_attempted"] += 1
            elif event["event_type"] == "mission_end":
                mission_id = event.get("mission_id", "unknown")
                if mission_id in mission_starts:
                    duration = event["timestamp"] - mission_starts[mission_id]
                    mission_durations.append(duration)
                    if event.get("success", False):
                        analysis["missions_completed"] += 1

        if analysis["missions_attempted"] > 0:
            analysis["success_rate"] = (
                analysis["missions_completed"] / analysis["missions_attempted"]
            )

        if mission_durations:
            analysis["avg_mission_duration"] = statistics.mean(mission_durations)

        return analysis

    def _analyze_errors(self) -> Dict[str, Any]:
        """Analyze error patterns."""
        error_events = self.data.get("error_events", [])

        analysis = {
            "total_errors": len(error_events),
            "error_types": {},
            "severity_breakdown": {},
            "error_timeline": [],
        }

        for event in error_events:
            # Error types
            error_type = event.get("error_type", "unknown")
            analysis["error_types"][error_type] = (
                analysis["error_types"].get(error_type, 0) + 1
            )

            # Severity breakdown
            severity = event.get("severity", "unknown")
            analysis["severity_breakdown"][severity] = (
                analysis["severity_breakdown"].get(severity, 0) + 1
            )

            # Timeline
            analysis["error_timeline"].append(
                {
                    "time": event["elapsed_time"],
                    "type": error_type,
                    "severity": severity,
                }
            )

        return analysis

    def _generate_recommendations(self) -> List[str]:
        """Generate performance recommendations."""
        recommendations = []

        # Analyze mission success
        mission_analysis = self._analyze_missions()
        if mission_analysis["success_rate"] < 0.8:
            recommendations.append(
                "Improve mission reliability - success rate below 80%"
            )

        # Analyze errors
        error_analysis = self._analyze_errors()
        if error_analysis["total_errors"] > 10:
            recommendations.append("Reduce system errors - high error count detected")

        # Analyze battery
        perf_analysis = self._analyze_performance()
        battery = perf_analysis.get("battery_analysis", {})
        if battery.get("voltage_drop", 0) > 2.0:
            recommendations.append(
                "Monitor battery health - significant voltage drop detected"
            )

        # Default recommendations
        if not recommendations:
            recommendations.append("System performance within acceptable parameters")
            recommendations.append("Continue regular maintenance and testing")

        return recommendations


def run_telemetry_collection(session_name: str = None):
    """Run telemetry collection for a competition session."""
    print("📊 Starting Competition Telemetry Collection...")
    print("=" * 50)

    rclpy.init()
    collector = CompetitionTelemetryCollector(session_name)

    try:
        print("Press Ctrl+C to stop collection and save data...")
        rclpy.spin(collector)
    except KeyboardInterrupt:
        pass
    finally:
        filename = collector.save_telemetry_data()
        print(f"\n💾 Telemetry collection complete: {filename}")
        collector.destroy_node()
        rclpy.shutdown()


def analyze_telemetry_file(telemetry_file: str):
    """Analyze a telemetry file and generate report."""
    print(f"📈 Analyzing telemetry file: {telemetry_file}")
    print("=" * 50)

    analyzer = CompetitionTelemetryAnalyzer(telemetry_file)

    if not analyzer.load_data():
        print("❌ Failed to load telemetry file")
        return

    report = analyzer.generate_competition_report()

    # Print summary report
    session = report["session_info"]
    print(f"Session: {session['name']}")
    print(f"Duration: {session['duration_minutes']:.1f} minutes")

    # Mission analysis
    missions = report["mission_analysis"]
    print(f"\n🏁 Mission Performance:")
    print(f"  Attempted: {missions['missions_attempted']}")
    print(f"  Completed: {missions['missions_completed']}")
    print(f"  Success Rate: {missions['success_rate']:.1f}%")
    # Error analysis
    errors = report["error_analysis"]
    print(f"\n⚠️  Error Analysis:")
    print(f"  Total Errors: {errors['total_errors']}")
    if errors["error_types"]:
        print("  Error Types:")
        for error_type, count in errors["error_types"].items():
            print(f"    {error_type}: {count}")
    else:
        print("  No errors recorded")

    # Recommendations
    print(f"\n💡 Recommendations:")
    for rec in report["recommendations"]:
        print(f"  - {rec}")

    # Save detailed report
    report_file = telemetry_file.replace(".json", "_analysis.json")
    with open(report_file, "w") as f:
        json.dump(report, f, indent=2, default=str)

    print(f"\n📄 Detailed analysis saved to: {report_file}")


def main():
    """Main telemetry function."""
    import argparse

    parser = argparse.ArgumentParser(description="Competition Telemetry System")
    parser.add_argument(
        "--collect", action="store_true", help="Start telemetry collection"
    )
    parser.add_argument("--analyze", type=str, help="Analyze telemetry file")
    parser.add_argument("--session-name", type=str, help="Session name for collection")

    args = parser.parse_args()

    if args.collect:
        run_telemetry_collection(args.session_name)
    elif args.analyze:
        analyze_telemetry_file(args.analyze)
    else:
        parser.print_help()


if __name__ == "__main__":
    main()
