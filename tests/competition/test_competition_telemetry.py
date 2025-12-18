#!/usr/bin/env python3
"""
Competition Telemetry Tests - Competition Critical
Tests the competition telemetry collection and analysis system.
"""

import json
import os
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float32, String


class TestCompetitionTelemetry(unittest.TestCase):
    """Test competition telemetry functionality."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS2 environment."""
        if not rclpy.ok():
            rclpy.init(args=[])
        cls.node = Node("test_telemetry")

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2 environment."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        from scripts.competition.competition_telemetry import (
            CompetitionTelemetryCollector,
        )

        self.collector = CompetitionTelemetryCollector(session_name="test_session")

    def tearDown(self):
        """Clean up test fixtures."""
        import shutil

        shutil.rmtree(self.temp_dir)
        if hasattr(self, "collector"):
            self.collector.destroy_node()

    def test_telemetry_initialization(self):
        """Test telemetry collector initialization."""
        self.assertEqual(self.collector.session_name, "test_session")
        self.assertIsInstance(self.collector.start_time, float)
        self.assertIsInstance(self.collector.telemetry_data, dict)

        # Check required data structures
        required_keys = [
            "session",
            "start_time",
            "gps_tracks",
            "performance_metrics",
            "mission_events",
            "error_events",
            "sensor_quality",
            "system_events",
        ]
        for key in required_keys:
            self.assertIn(key, self.collector.telemetry_data)

    def test_gps_data_collection(self):
        """Test GPS data collection."""
        # Create GPS message
        gps_msg = NavSatFix()
        gps_msg.latitude = 40.7128
        gps_msg.longitude = -74.0060
        gps_msg.altitude = 10.5
        gps_msg.status.status = 0
        gps_msg.status.service = 1

        # Simulate callback
        self.collector._gps_callback(gps_msg)

        # Check data was recorded
        gps_tracks = self.collector.telemetry_data["gps_tracks"]
        self.assertEqual(len(gps_tracks), 1)

        track = gps_tracks[0]
        self.assertEqual(track["latitude"], 40.7128)
        self.assertEqual(track["longitude"], -74.0060)
        self.assertEqual(track["altitude"], 10.5)
        self.assertGreater(track["elapsed_time"], 0)

    def test_imu_sensor_quality_tracking(self):
        """Test IMU data collection and quality tracking."""
        # Create IMU message with reasonable values
        imu_msg = Imu()
        imu_msg.linear_acceleration.x = 0.1
        imu_msg.linear_acceleration.y = 0.2
        imu_msg.linear_acceleration.z = 9.8  # Gravity

        # Simulate callback
        self.collector._imu_callback(imu_msg)

        # Check IMU data recorded
        self.assertEqual(len(self.collector.telemetry_data["sensor_quality"]), 1)

        quality_data = self.collector.telemetry_data["sensor_quality"][0]
        self.assertEqual(quality_data["sensor"], "imu")
        self.assertAlmostEqual(quality_data["accel_magnitude"], 9.8, places=1)
        self.assertAlmostEqual(quality_data["expected_gravity"], 9.8)
        self.assertLess(
            abs(quality_data["deviation"]), 1.0
        )  # Should be close to gravity

    def test_battery_monitoring(self):
        """Test battery level monitoring."""
        # Create battery message
        battery_msg = Float32()
        battery_msg.data = 12.3  # 12.3V

        # Simulate callback
        self.collector._battery_callback(battery_msg)

        # Check battery data recorded in performance metrics
        perf_metrics = self.collector.telemetry_data["performance_metrics"]
        battery_metrics = [
            m for m in perf_metrics if m.get("metric") == "battery_voltage"
        ]

        self.assertEqual(len(battery_metrics), 1)
        self.assertEqual(battery_metrics[0]["value"], 12.3)
        self.assertEqual(battery_metrics[0]["unit"], "V")

    def test_mission_event_tracking(self):
        """Test mission event tracking."""
        # Create mission status message
        mission_msg = String()
        mission_msg.data = json.dumps(
            {"active": True, "current_mission": "waypoint_navigation", "progress": 0.5}
        )

        # Simulate callback
        self.collector._mission_callback(mission_msg)

        # Check mission event recorded
        mission_events = self.collector.telemetry_data["mission_events"]
        self.assertEqual(len(mission_events), 1)

        event = mission_events[0]
        self.assertEqual(event["event_type"], "mission_update")
        self.assertEqual(event["data"]["active"], True)
        self.assertEqual(event["data"]["current_mission"], "waypoint_navigation")

    def test_emergency_event_tracking(self):
        """Test emergency event tracking."""
        # Create emergency status message
        emergency_msg = String()
        emergency_msg.data = json.dumps(
            {"level": "SOFT_STOP", "manual_override": False, "trigger_count": 1}
        )

        # Simulate callback
        self.collector._emergency_callback(emergency_msg)

        # Check emergency event recorded in error events
        error_events = self.collector.telemetry_data["error_events"]
        self.assertEqual(len(error_events), 1)

        event = error_events[0]
        self.assertEqual(event["event_type"], "emergency_event")
        self.assertEqual(event["data"]["level"], "SOFT_STOP")

    def test_mission_lifecycle_tracking(self):
        """Test mission start and end tracking."""
        # Record mission start
        self.collector.record_mission_start("test_mission", "mission_001")

        # Check start event recorded
        mission_events = self.collector.telemetry_data["mission_events"]
        start_events = [e for e in mission_events if e["event_type"] == "mission_start"]
        self.assertEqual(len(start_events), 1)

        # Record mission end
        self.collector.record_mission_end("mission_001", True, 45.2)

        # Check end event recorded
        end_events = [e for e in mission_events if e["event_type"] == "mission_end"]
        self.assertEqual(len(end_events), 1)

        end_event = end_events[0]
        self.assertTrue(end_event["success"])
        self.assertEqual(end_event["duration"], 45.2)

    def test_error_event_logging(self):
        """Test manual error event logging."""
        self.collector.record_error_event(
            "navigation_failure", "GPS lost lock", "warning"
        )

        error_events = self.collector.telemetry_data["error_events"]
        self.assertEqual(len(error_events), 1)

        event = error_events[0]
        self.assertEqual(event["error_type"], "navigation_failure")
        self.assertEqual(event["description"], "GPS lost lock")
        self.assertEqual(event["severity"], "warning")

    def test_telemetry_data_persistence(self):
        """Test telemetry data saving and loading."""
        # Add some test data
        self.collector.record_mission_start("persistence_test", "test_001")
        self.collector.record_error_event("test_error", "Test error message")

        # Save telemetry
        save_path = os.path.join(self.temp_dir, "test_telemetry.json")
        saved_path = self.collector.save_telemetry_data(save_path)

        self.assertEqual(saved_path, save_path)
        self.assertTrue(os.path.exists(save_path))

        # Load and verify data
        with open(save_path, "r") as f:
            loaded_data = json.load(f)

        self.assertEqual(loaded_data["session"], "test_session")
        self.assertIn("start_time", loaded_data)
        self.assertIn("end_time", loaded_data)
        self.assertGreater(loaded_data["duration_seconds"], 0)

    def test_session_statistics(self):
        """Test session statistics generation."""
        # Add test data
        self.collector.record_mission_start("stats_test", "stat_001")
        self.collector.record_mission_end("stat_001", True, 30.0)
        self.collector.record_error_event("test_error", "Test error")

        stats = self.collector.get_session_stats()

        self.assertEqual(stats["session_name"], "test_session")
        self.assertIn("data_points", stats)
        self.assertGreater(stats["duration_s"], 0)
        self.assertGreaterEqual(stats["data_points"]["mission_events"], 2)
        self.assertGreaterEqual(stats["data_points"]["error_events"], 1)

    def test_battery_level_estimation(self):
        """Test battery level estimation from voltage."""
        # Test various voltage levels
        test_cases = [
            (12.6, 100.0),  # Full charge
            (11.8, 80.0),  # 80% charge
            (11.0, 20.0),  # 20% charge
            (10.0, 0.0),  # Empty
            (9.5, 0.0),  # Below minimum
        ]

        for voltage, expected_level in test_cases:
            level = self.collector._estimate_battery_level(voltage)
            self.assertAlmostEqual(level, expected_level, places=0)

    def test_performance_metrics_collection(self):
        """Test automatic performance metrics collection."""
        # Call performance collection
        self.collector._collect_performance_metrics()

        # Should have recorded a performance metric
        perf_metrics = self.collector.telemetry_data["performance_metrics"]
        self.assertGreater(len(perf_metrics), 0)

        # Check latest metric
        latest = perf_metrics[-1]
        self.assertEqual(latest["metric"], "system_uptime")
        self.assertIn("value", latest)
        self.assertEqual(latest["unit"], "seconds")


class TestCompetitionTelemetryAnalysis(unittest.TestCase):
    """Test telemetry analysis functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        from scripts.competition.competition_telemetry import (
            CompetitionTelemetryAnalyzer,
        )

        # Create mock telemetry data
        self.mock_data = {
            "session": "test_analysis",
            "start_time": "2024-01-01T10:00:00",
            "end_time": "2024-01-01T11:00:00",
            "duration_seconds": 3600,
            "gps_tracks": [
                {"latitude": 40.0, "longitude": -74.0, "elapsed_time": 0},
                {"latitude": 40.1, "longitude": -74.1, "elapsed_time": 1800},
            ],
            "mission_events": [
                {
                    "event_type": "mission_start",
                    "mission_id": "m1",
                    "timestamp": 1000000000,
                },
                {
                    "event_type": "mission_end",
                    "mission_id": "m1",
                    "success": True,
                    "duration": 1800,
                    "timestamp": 1000001800,
                },
            ],
            "error_events": [
                {"error_type": "navigation", "severity": "warning"},
                {"error_type": "communication", "severity": "error"},
            ],
            "performance_metrics": [
                {"metric": "battery_voltage", "value": 12.0},
                {"metric": "battery_voltage", "value": 11.5},
            ],
        }

        # Save mock data
        self.telemetry_file = os.path.join(self.temp_dir, "mock_telemetry.json")
        with open(self.telemetry_file, "w") as f:
            json.dump(self.mock_data, f)

        self.analyzer = CompetitionTelemetryAnalyzer(self.telemetry_file)

    def tearDown(self):
        """Clean up test fixtures."""
        import shutil

        shutil.rmtree(self.temp_dir)

    def test_data_loading(self):
        """Test telemetry data loading."""
        success = self.analyzer.load_data()
        self.assertTrue(success)
        self.assertIsNotNone(self.analyzer.data)

    def test_performance_analysis(self):
        """Test performance metrics analysis."""
        self.analyzer.load_data()
        analysis = self.analyzer._analyze_performance()

        self.assertIn("battery_analysis", analysis)
        battery = analysis["battery_analysis"]
        self.assertEqual(battery["min_voltage"], 11.5)
        self.assertEqual(battery["max_voltage"], 12.0)

    def test_mission_analysis(self):
        """Test mission performance analysis."""
        self.analyzer.load_data()
        analysis = self.analyzer._analyze_missions()

        self.assertEqual(analysis["missions_attempted"], 1)
        self.assertEqual(analysis["missions_completed"], 1)
        self.assertEqual(analysis["success_rate"], 1.0)
        self.assertEqual(analysis["avg_mission_duration"], 1800)

    def test_error_analysis(self):
        """Test error pattern analysis."""
        self.analyzer.load_data()
        analysis = self.analyzer._analyze_errors()

        self.assertEqual(analysis["total_errors"], 2)
        self.assertEqual(len(analysis["error_types"]), 2)
        self.assertIn("navigation", analysis["error_types"])
        self.assertIn("communication", analysis["error_types"])

    def test_comprehensive_report_generation(self):
        """Test complete competition report generation."""
        self.analyzer.load_data()
        report = self.analyzer.generate_competition_report()

        self.assertIn("session_info", report)
        self.assertIn("performance_analysis", report)
        self.assertIn("mission_analysis", report)
        self.assertIn("error_analysis", report)
        self.assertIn("recommendations", report)

        # Check session info
        session = report["session_info"]
        self.assertEqual(session["name"], "test_analysis")
        self.assertEqual(session["duration_s"], 3600)

        # Check recommendations exist
        recommendations = report["recommendations"]
        self.assertIsInstance(recommendations, list)
        self.assertGreater(len(recommendations), 0)


if __name__ == "__main__":
    unittest.main()
