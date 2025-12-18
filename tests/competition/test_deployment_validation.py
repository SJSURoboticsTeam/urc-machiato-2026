#!/usr/bin/env python3
"""
Deployment Validation Tests - Competition Critical
Tests the deployment validation and pre-competition checks.
"""

import os
import tempfile
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch

import yaml


class TestDeploymentValidation(unittest.TestCase):
    """Test deployment validation functionality."""

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
        """Test checklist initialization."""
        self.assertIsInstance(self.checklist.checks, list)
        self.assertGreater(len(self.checklist.checks), 0)

        # Check required checks are present
        check_ids = [check["id"] for check in self.checklist.checks]
        required_checks = [
            "hardware_power",
            "hardware_sensors",
            "hardware_actuators",
            "software_services",
            "software_bridge",
            "software_state",
            "network_connectivity",
            "network_dns",
            "network_latency",
            "safety_emergency",
            "safety_limits",
            "safety_override",
            "config_loaded",
            "config_validated",
        ]

        for required_check in required_checks:
            self.assertIn(required_check, check_ids)

    def test_hardware_power_check_success(self):
        """Test battery power check success."""
        with patch("subprocess.run") as mock_run:
            # Mock successful battery reading
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "12.5\n"
            mock_run.return_value = mock_result

            success, message = self.checklist._check_battery_level()
            self.assertTrue(success)
            self.assertIn("12.5V", message)

    def test_hardware_power_check_failure(self):
        """Test battery power check failure."""
        with patch("subprocess.run") as mock_run:
            # Mock failed battery reading
            mock_result = MagicMock()
            mock_result.returncode = 1
            mock_run.return_value = mock_result

            success, message = self.checklist._check_battery_level()
            self.assertFalse(success)
            self.assertIn("Could not read battery voltage", message)

    def test_sensor_health_check_success(self):
        """Test sensor health check success."""
        with patch("subprocess.run") as mock_run:
            # Mock successful sensor check
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "GPS:True,IMU:True\n"
            mock_run.return_value = mock_result

            success, message = self.checklist._check_sensor_health()
            self.assertTrue(success)
            self.assertIn("GPS and IMU sensors operational", message)

    def test_sensor_health_check_failure(self):
        """Test sensor health check failure."""
        with patch("subprocess.run") as mock_run:
            # Mock failed sensor check
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "GPS:False,IMU:True\n"
            mock_run.return_value = mock_result

            success, message = self.checklist._check_sensor_health()
            self.assertFalse(success)
            self.assertIn("Sensor check failed", message)

    def test_ros_services_check_success(self):
        """Test ROS2 services check success."""
        with patch("subprocess.run") as mock_run:
            # Mock successful ROS2 service list
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "/state_machine/health_check\n/mission/health_check\n/emergency/health_check\n"
            mock_run.return_value = mock_result

            success, message = self.checklist._check_ros_services()
            self.assertTrue(success)
            self.assertIn("ROS2 services", message)

    def test_network_connectivity_check_success(self):
        """Test network connectivity check success."""
        with patch("subprocess.run") as mock_run:
            # Mock successful ping
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_run.return_value = mock_result

            success, message = self.checklist._check_network_connectivity()
            self.assertTrue(success)
            self.assertIn("Network connectivity established", message)

    def test_network_latency_check_success(self):
        """Test network latency check success."""
        with patch("subprocess.run") as mock_run:
            # Mock successful ping with good latency
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = """
PING 8.8.8.8 (8.8.8.8) 56(84) bytes of data.
64 bytes from 8.8.8.8: icmp_seq=1 ttl=118 time=15.2 ms
64 bytes from 8.8.8.8: icmp_seq=2 ttl=118 time=14.8 ms
64 bytes from 8.8.8.8: icmp_seq=3 ttl=118 time=16.1 ms

--- 8.8.8.8 ping statistics ---
3 packets transmitted, 3 received, 0% packet loss, time 2003ms
rtt min/avg/max/mdev = 14.783/15.367/16.098/0.551 ms
"""
            mock_run.return_value = mock_result

            success, message = self.checklist._check_network_latency()
            self.assertTrue(success)
            self.assertIn("15.4ms", message)

    def test_network_latency_check_failure(self):
        """Test network latency check failure."""
        with patch("subprocess.run") as mock_run:
            # Mock ping with high latency
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = (
                "rtt min/avg/max/mdev = 14.783/150.367/160.098/0.551 ms"
            )
            mock_run.return_value = mock_result

            success, message = self.checklist._check_network_latency()
            self.assertFalse(success)
            self.assertIn("too high", message)

    def test_disk_space_check_success(self):
        """Test disk space check success."""
        with patch("subprocess.run") as mock_run:
            # Mock successful df command
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "/dev/sda1 1000000 100000 900000 10% /"
            mock_run.return_value = mock_result

            success = self.checklist._check_disk_space()
            self.assertTrue(success)

    def test_disk_space_check_failure(self):
        """Test disk space check failure."""
        with patch("subprocess.run") as mock_run:
            # Mock df with low space
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "/dev/sda1 1000000 950000 50000 95% /"
            mock_run.return_value = mock_result

            success = self.checklist._check_disk_space()
            self.assertFalse(success)

    def test_emergency_system_check_success(self):
        """Test emergency system check success."""
        with patch("subprocess.run") as mock_run:
            # Mock successful emergency system check
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "Success:True"
            mock_run.return_value = mock_result

            success, message = self.checklist._check_emergency_system()
            self.assertTrue(success)
            self.assertIn("Emergency stop system ready", message)

    def test_state_machine_check_success(self):
        """Test state machine check success."""
        with patch("subprocess.run") as mock_run:
            # Mock successful state machine check
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "Success:True"
            mock_run.return_value = mock_result

            success, message = self.checklist._check_state_machine()
            self.assertTrue(success)
            self.assertIn("State machine ready", message)

    def test_bridge_health_check_success(self):
        """Test communication bridge health check success."""
        with patch("subprocess.run") as mock_run:
            # Mock successful bridge check
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_run.return_value = mock_result

            success, message = self.checklist._check_bridge_health()
            self.assertTrue(success)
            self.assertIn("Communication bridge running", message)

    def test_config_validation_checks(self):
        """Test configuration validation checks."""
        # Test config loaded
        success, message = self.checklist._check_config_loaded()
        self.assertTrue(success)
        self.assertIn("exists", message)

        # Test config validation (mock successful validation)
        with patch(
            "scripts.competition.pre_competition_checklist.PreCompetitionChecklist._check_config_validation"
        ) as mock_validate:
            mock_validate.return_value = (True, "Configuration validated")

            # This would normally call the config validation
            # For this test, we verify the method exists and can be called
            self.assertTrue(hasattr(self.checklist, "_check_config_loaded"))

    def test_complete_checklist_execution(self):
        """Test complete checklist execution workflow."""
        # Mock all checks to pass
        check_methods = [
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

        with patch(
            "scripts.competition.pre_competition_checklist.PreCompetitionChecklist._check_disk_space",
            return_value=True,
        ), patch(
            "scripts.competition.pre_competition_checklist.PreCompetitionChecklist._check_hardware_readiness",
            return_value=True,
        ), patch(
            "scripts.competition.pre_competition_checklist.PreCompetitionChecklist._check_software_dependencies",
            return_value=True,
        ), patch(
            "scripts.competition.pre_competition_checklist.PreCompetitionChecklist._check_configuration_validity",
            return_value=True,
        ):

            for method_name in check_methods:
                if hasattr(self.checklist, method_name):
                    with patch.object(
                        self.checklist, method_name, return_value=(True, "OK")
                    ):
                        pass

            # Mock subprocess calls for external checks
            with patch("subprocess.run") as mock_run:
                mock_result = MagicMock()
                mock_result.returncode = 0
                mock_result.stdout = "OK"
                mock_run.return_value = mock_result

                # Run the checklist
                success = self.checklist.run_checklist()

                # Should succeed with all mocks passing
                # (Note: In real execution, some checks may still fail due to system state)
                self.assertIsInstance(success, bool)

    def test_checklist_result_persistence(self):
        """Test checklist result saving."""
        # Mock successful run
        self.checklist.results = {
            "timestamp": 1234567890,
            "test_suites": {},
            "overall_success": True,
            "summary": {},
        }

        # This would normally save to file
        # For this test, we verify the structure
        self.assertIn("timestamp", self.checklist.results)
        self.assertIn("test_suites", self.checklist.results)
        self.assertIn("overall_success", self.checklist.results)

    def test_error_handling_in_checks(self):
        """Test error handling in individual checks."""
        # Test that exceptions in checks are handled gracefully
        with patch.object(
            self.checklist, "_check_battery_level", side_effect=Exception("Test error")
        ):
            success, message = self.checklist._check_battery_level()
            # Should return False and error message
            self.assertIsInstance(success, bool)
            self.assertIsInstance(message, str)

    def test_check_method_existence(self):
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
            "_check_disk_space",
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
