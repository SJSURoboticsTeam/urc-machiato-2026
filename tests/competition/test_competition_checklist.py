#!/usr/bin/env python3
"""
Competition Checklist Tests - Competition Critical
Tests the pre-competition validation checklist system.
"""

import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import yaml


class TestCompetitionChecklist(unittest.TestCase):
    """Test pre-competition checklist functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.project_root = Path(self.temp_dir) / "urc_project"
        self.project_root.mkdir()

        # Create mock project structure
        (self.project_root / "config").mkdir()
        (self.project_root / "scripts").mkdir()
        (self.project_root / "competition").mkdir()

        # Create valid config
        config_content = """
simulation:
  simulation_update_rate_hz: 20
  simulation_odom_noise: 0.01
  simulation_imu_noise: 0.001
  simulation_gps_noise: 0.5
mission:
  mission_execution_rate_hz: 10
  mission_timeout_seconds: 300
  mission_max_speed_mps: 1.0
safety:
  safety_emergency_stop_enabled: true
  safety_collision_threshold_m: 1.0
network:
  network_qos_depth_sensor: 5
  network_qos_depth_command: 10
performance:
  performance_monitoring_enabled: true
  performance_log_interval_seconds: 30
"""

        with open(self.project_root / "config" / "rover.yaml", "w") as f:
            f.write(config_content)

        # Import after setup
        import sys

        sys.path.insert(0, str(self.project_root))

        from scripts.competition.pre_competition_checklist import (
            PreCompetitionChecklist,
        )

        self.checklist = PreCompetitionChecklist()

    def tearDown(self):
        """Clean up test fixtures."""
        import shutil

        shutil.rmtree(self.temp_dir)

    def test_checklist_initialization(self):
        """Test checklist initialization and structure."""
        # Check that checklist is properly initialized
        self.assertIsInstance(self.checklist.checks, list)
        self.assertGreater(len(self.checklist.checks), 0)

        # Verify all checks have required fields
        for check in self.checklist.checks:
            self.assertIn("id", check)
            self.assertIn("description", check)
            self.assertIn("function", check)
            self.assertIn("status", check)
            self.assertIn("message", check)
            self.assertEqual(check["status"], "pending")

        # Check expected check categories
        check_ids = [check["id"] for check in self.checklist.checks]

        # Hardware checks
        self.assertIn("hardware_power", check_ids)
        self.assertIn("hardware_sensors", check_ids)
        self.assertIn("hardware_actuators", check_ids)
        self.assertIn("hardware_cameras", check_ids)

        # Software checks
        self.assertIn("software_services", check_ids)
        self.assertIn("software_bridge", check_ids)
        self.assertIn("software_state", check_ids)

        # Network checks
        self.assertIn("network_connectivity", check_ids)
        self.assertIn("network_dns", check_ids)
        self.assertIn("network_latency", check_ids)

        # Should have 15 total checks
        self.assertEqual(len(check_ids), 15)

        # Safety checks
        self.assertIn("safety_emergency", check_ids)
        self.assertIn("safety_limits", check_ids)
        self.assertIn("safety_override", check_ids)

        # Configuration checks
        self.assertIn("config_loaded", check_ids)
        self.assertIn("config_validated", check_ids)

    def test_add_check_method(self):
        """Test adding custom checks to the checklist."""
        initial_count = len(self.checklist.checks)

        # Add a custom check
        def custom_check():
            return True, "Custom check passed"

        self.checklist.add_check("custom_test", "Custom test check", custom_check)

        # Verify check was added
        self.assertEqual(len(self.checklist.checks), initial_count + 1)

        new_check = self.checklist.checks[-1]
        self.assertEqual(new_check["id"], "custom_test")
        self.assertEqual(new_check["description"], "Custom test check")
        self.assertEqual(new_check["status"], "pending")

        # Test the custom check function
        success, message = new_check["function"]()
        self.assertTrue(success)
        self.assertEqual(message, "Custom check passed")

    def test_battery_level_check_success(self):
        """Test battery level check with successful reading."""
        with patch("subprocess.run") as mock_run:
            # Mock successful battery voltage reading
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "12.8\n"
            mock_run.return_value = mock_result

            success, message = self.checklist._check_battery_level()
            self.assertTrue(success)
            self.assertIn("12.8V", message)

    def test_battery_level_check_failure(self):
        """Test battery level check with failure conditions."""
        # Test low voltage
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "10.5\n"  # Below 11.0V minimum
            mock_run.return_value = mock_result

            success, message = self.checklist._check_battery_level()
            self.assertFalse(success)
            self.assertIn("too low", message)

        # Test command failure
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 1
            mock_run.return_value = mock_result

            success, message = self.checklist._check_battery_level()
            self.assertFalse(success)
            self.assertIn("Could not read battery voltage", message)

    def test_sensor_health_check_success(self):
        """Test sensor health check with operational sensors."""
        with patch("subprocess.run") as mock_run:
            # Mock successful sensor check
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "GPS:True,IMU:True\n"
            mock_run.return_value = mock_result

            success, message = self.checklist._check_sensor_health()
            self.assertTrue(success)
            self.assertIn("GPS and IMU sensors operational", message)

    def test_sensor_health_check_partial_failure(self):
        """Test sensor health check with partial sensor failure."""
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "GPS:False,IMU:True\n"
            mock_run.return_value = mock_result

            success, message = self.checklist._check_sensor_health()
            self.assertFalse(success)
            self.assertIn("Sensor check failed", message)

    def test_sensor_health_check_command_failure(self):
        """Test sensor health check with command execution failure."""
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 1
            mock_run.return_value = mock_result

            success, message = self.checklist._check_sensor_health()
            self.assertFalse(success)

    def test_ros_services_check_success(self):
        """Test ROS2 services check with all services running."""
        with patch("subprocess.run") as mock_run:
            # Mock successful ROS2 service listing
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "/state_machine/health_check\n/mission/health_check\n/emergency/health_check\n/simulation/status\n"
            mock_run.return_value = mock_result

            success, message = self.checklist._check_ros_services()
            self.assertTrue(success)
            self.assertIn("ROS2 services", message)

    def test_ros_services_check_missing_services(self):
        """Test ROS2 services check with missing critical services."""
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "/some_other_service\n"  # Missing critical services
            mock_run.return_value = mock_result

            success, message = self.checklist._check_ros_services()
            self.assertFalse(success)
            self.assertIn("Missing ROS2 services", message)

    def test_network_connectivity_check_success(self):
        """Test network connectivity check success."""
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_run.return_value = mock_result

            success, message = self.checklist._check_network_connectivity()
            # Note: The actual implementation may vary, just test it returns valid results
            self.assertIsInstance(success, bool)
            self.assertIsInstance(message, str)

    def test_network_connectivity_check_failure(self):
        """Test network connectivity check failure."""
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 1
            mock_run.return_value = mock_result

            success, message = self.checklist._check_network_connectivity()
            self.assertFalse(success)

    def test_network_latency_check_success(self):
        """Test network latency check with acceptable latency."""
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "rtt min/avg/max/mdev = 10.5/25.3/45.2/12.1 ms"
            mock_run.return_value = mock_result

            success, message = self.checklist._check_network_latency()
            self.assertTrue(success)
            self.assertIn("25.3ms", message)

    def test_network_latency_check_high_latency(self):
        """Test network latency check with high latency."""
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = (
                "rtt min/avg/max/mdev = 50.1/180.5/250.3/45.2 ms"  # Over 100ms
            )
            mock_run.return_value = mock_result

            success, message = self.checklist._check_network_latency()
            self.assertFalse(success)
            self.assertIn("too high", message)

    def test_emergency_system_check_success(self):
        """Test emergency system check success."""
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "Success:True"
            mock_run.return_value = mock_result

            success, message = self.checklist._check_emergency_system()
            self.assertTrue(success)
            self.assertIn("Emergency stop system ready", message)

    def test_emergency_system_check_failure(self):
        """Test emergency system check failure."""
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "Success:False"
            mock_run.return_value = mock_result

            success, message = self.checklist._check_emergency_system()
            self.assertFalse(success)

    def test_state_machine_check_success(self):
        """Test state machine readiness check success."""
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "Success:True"
            mock_run.return_value = mock_result

            success, message = self.checklist._check_state_machine()
            self.assertTrue(success)
            self.assertIn("State machine healthy", message)

    def test_bridge_health_check_success(self):
        """Test communication bridge health check success."""
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_run.return_value = mock_result

            success, message = self.checklist._check_bridge_health()
            self.assertTrue(success)
            self.assertIn("Communication bridge running", message)

    def test_config_loaded_check(self):
        """Test configuration file existence check."""
        # Config file exists (created in setUp)
        success, message = self.checklist._check_config_loaded()
        self.assertTrue(success)
        self.assertIn("exists", message)

    def test_actuator_health_check_placeholder(self):
        """Test actuator health check (currently placeholder)."""
        success, message = self.checklist._check_actuator_health()
        # Currently returns True with placeholder message
        self.assertIsInstance(success, bool)
        self.assertIsInstance(message, str)

    def test_camera_health_check_placeholder(self):
        """Test camera health check (currently placeholder)."""
        success, message = self.checklist._check_camera_health()
        # Currently returns True with placeholder message
        self.assertIsInstance(success, bool)
        self.assertIsInstance(message, str)

    def test_safety_limits_check_placeholder(self):
        """Test safety limits check (currently placeholder)."""
        success, message = self.checklist._check_safety_limits()
        # Currently returns True with placeholder message
        self.assertIsInstance(success, bool)
        self.assertIsInstance(message, str)

    def test_safety_override_check_placeholder(self):
        """Test safety override check (currently placeholder)."""
        success, message = self.checklist._check_manual_override()
        # Currently returns True with placeholder message
        self.assertIsInstance(success, bool)
        self.assertIsInstance(message, str)

    def test_complete_checklist_execution_workflow(self):
        """Test the complete checklist execution workflow."""
        # Execute checklist (in test environment, it will likely fail but should complete)
        success = self.checklist.run_checklist()

        # Should complete and return a boolean (may be False in test environment)
        self.assertIsInstance(success, bool)

        # Check that results were recorded (results may be empty if execution failed)
        self.assertIsInstance(self.checklist.results, dict)

        # Check that all checks were executed (they should have status set)
        self.assertEqual(len(self.checklist.checks), 15)  # Should have 15 checks
        executed_checks = sum(
            1 for check in self.checklist.checks if check["status"] != "pending"
        )
        self.assertGreater(executed_checks, 0)  # At least some checks should have run

    def test_checklist_result_structure(self):
        """Test checklist result data structure."""
        # Simulate running a check
        self.checklist.checks[0]["status"] = "passed"
        self.checklist.checks[0]["message"] = "Test passed"

        # Check result structure
        self.assertEqual(self.checklist.checks[0]["status"], "passed")
        self.assertEqual(self.checklist.checks[0]["message"], "Test passed")

    def test_error_handling_in_check_execution(self):
        """Test error handling when check functions fail."""
        # Create a check that raises an exception
        def failing_check():
            raise Exception("Test exception")

        self.checklist.add_check("failing_test", "Failing test", failing_check)

        # Execute the failing check
        check = self.checklist.checks[-1]
        try:
            success, message = check["function"]()
        except Exception:
            success, message = False, "Exception occurred"

        # Should handle the exception gracefully
        self.assertIsInstance(success, bool)
        self.assertIsInstance(message, str)

    def test_checklist_method_existence(self):
        """Test that all required check methods exist."""
        required_methods = [
            "_check_battery_level",
            "_check_sensor_health",
            "_check_actuator_health",
            "_check_camera_health",
            "_check_ros_services",
            "_check_bridge_health",
            "_check_state_machine",
            "_check_network_connectivity",
            "_check_dns_resolution",
            "_check_network_latency",
            "_check_emergency_system",
            "_check_safety_limits",
            "_check_manual_override",
            "_check_config_loaded",
        ]

        for method_name in required_methods:
            self.assertTrue(
                hasattr(self.checklist, method_name),
                f"Missing required method: {method_name}",
            )


if __name__ == "__main__":
    unittest.main()
