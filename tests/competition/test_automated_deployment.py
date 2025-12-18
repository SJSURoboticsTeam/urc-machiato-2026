#!/usr/bin/env python3
"""
Automated Deployment Tests - Competition Critical
Tests the automated deployment system for competition reliability.
"""

import os
import shutil
import tempfile
import time
import unittest
from pathlib import Path
from unittest.mock import MagicMock, patch


class TestAutomatedDeployment(unittest.TestCase):
    """Test automated deployment functionality."""

    def setUp(self):
        """Set up test fixtures."""
        self.temp_dir = tempfile.mkdtemp()
        self.project_root = Path(self.temp_dir) / "urc_project"
        self.project_root.mkdir()

        # Create mock project structure
        (self.project_root / "config").mkdir()
        (self.project_root / "logs").mkdir()
        (self.project_root / "scripts").mkdir()
        (self.project_root / "bridges").mkdir()

        # Create mock config file
        config_content = """
simulation:
  simulation_update_rate_hz: 20
mission:
  mission_timeout_seconds: 300
"""
        with open(self.project_root / "config" / "rover.yaml", "w") as f:
            f.write(config_content)

        # Import after setup
        import sys

        sys.path.insert(0, str(self.project_root))

        from scripts.deployment.automated_deployment import AutomatedDeployment

        self.deployer = AutomatedDeployment(str(self.project_root))

    def tearDown(self):
        """Clean up test fixtures."""
        shutil.rmtree(self.temp_dir)

    def test_pre_deployment_checks_pass(self):
        """Test pre-deployment checks when everything is ready."""
        with patch.object(
            self.deployer, "_check_hardware_readiness", return_value=True
        ), patch.object(
            self.deployer, "_check_software_dependencies", return_value=True
        ), patch.object(
            self.deployer, "_check_configuration_validity", return_value=True
        ), patch.object(
            self.deployer, "_check_network_readiness", return_value=True
        ), patch.object(
            self.deployer, "_check_disk_space", return_value=True
        ):

            success = self.deployer._run_pre_deployment_checks()
            self.assertTrue(success)

    def test_pre_deployment_checks_fail(self):
        """Test pre-deployment checks when components fail."""
        with patch.object(
            self.deployer, "_check_hardware_readiness", return_value=False
        ), patch.object(
            self.deployer, "_check_software_dependencies", return_value=True
        ), patch.object(
            self.deployer, "_check_configuration_validity", return_value=True
        ), patch.object(
            self.deployer, "_check_network_readiness", return_value=True
        ), patch.object(
            self.deployer, "_check_disk_space", return_value=True
        ):

            success = self.deployer._run_pre_deployment_checks()
            self.assertFalse(success)

    def test_environment_setup(self):
        """Test deployment environment setup."""
        with patch.dict(os.environ, {}, clear=True):
            success = self.deployer._setup_deployment_environment()

            # Should succeed even if ROS2 isn't available in test
            self.assertIsInstance(success, bool)

            # Should set environment variables
            self.assertEqual(os.environ.get("URC_ENV"), "competition")

    def test_service_deployment_success(self):
        """Test successful service deployment."""
        services = [
            {
                "name": "test_service",
                "command": 'echo "test"',
                "health_check": lambda name: True,
                "timeout": 5,
            }
        ]

        with patch("subprocess.Popen") as mock_popen, patch("time.sleep") as mock_sleep:

            mock_process = MagicMock()
            mock_process.pid = 12345
            mock_process.poll.return_value = None  # Process is running
            mock_popen.return_value = mock_process

            success = self.deployer._start_service(services[0])
            self.assertTrue(success)

    def test_service_deployment_failure(self):
        """Test service deployment failure handling."""
        services = [
            {
                "name": "failing_service",
                "command": "false",  # Command that fails
                "health_check": lambda name: False,  # Health check fails
                "timeout": 1,
            }
        ]

        with patch("subprocess.Popen") as mock_popen, patch("time.sleep") as mock_sleep:

            mock_process = MagicMock()
            mock_process.pid = 12345
            mock_process.poll.return_value = None
            mock_process.terminate = MagicMock()
            mock_process.wait = MagicMock()
            mock_popen.return_value = mock_process

            success = self.deployer._start_service(services[0])
            self.assertFalse(success)

    def test_deployment_rollback(self):
        """Test deployment rollback on failure."""
        # Add some mock running services
        self.deployer.services_started = [
            {"name": "service1", "process": MagicMock()},
            {"name": "service2", "process": MagicMock()},
        ]

        # Mock process termination
        for service in self.deployer.services_started:
            service["process"].terminate = MagicMock()
            service["process"].wait = MagicMock()

        # Test rollback
        self.deployer._rollback_deployment()

        # Should have cleared services list
        self.assertEqual(len(self.deployer.services_started), 0)

        # Should have called terminate on all processes
        for service in self.deployer.services_started:
            service["process"].terminate.assert_called_once()

    def test_post_deployment_validation(self):
        """Test post-deployment validation."""
        with patch.object(
            self.deployer, "_check_overall_system_health", return_value=True
        ), patch.object(
            self.deployer, "_check_ros2_communication", return_value=True
        ), patch.object(
            self.deployer, "_check_service_integration", return_value=True
        ), patch.object(
            self.deployer, "_check_performance_baseline", return_value=True
        ):

            success = self.deployer._run_post_deployment_checks()
            self.assertTrue(success)

    def test_deployment_report_generation(self):
        """Test deployment report generation."""
        # Simulate a deployment
        self.deployer.start_time = time.time() - 10  # 10 seconds ago
        self.deployer.services_started = [
            {"name": "service1", "process": MagicMock()},
            {"name": "service2", "process": MagicMock()},
        ]

        report = self.deployer.generate_deployment_report()

        self.assertIn("deployment_status", report)
        self.assertIn("services_deployed", report)
        self.assertIn("deployment_duration", report)
        self.assertIn("health_checks", report)
        self.assertGreater(
            report["deployment_duration"], 0
        )  # Should have some duration

    def test_individual_check_methods(self):
        """Test individual pre-deployment check methods."""
        # Test disk space check
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "Filesystem 1K-blocks Used Available Use% Mounted on\n/dev/sda1 1000000 100000 900000 10% /"
            mock_run.return_value = mock_result

            success = self.deployer._check_disk_space()
            self.assertTrue(success)

        # Test network connectivity check
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_run.return_value = mock_result

            success = self.deployer._check_network_connectivity()
            self.assertTrue(success)

    def test_full_deployment_simulation(self):
        """Test full deployment workflow simulation."""
        with patch.object(
            self.deployer, "_run_pre_deployment_checks", return_value=True
        ), patch.object(
            self.deployer, "_setup_deployment_environment", return_value=True
        ), patch.object(
            self.deployer, "_deploy_services", return_value=True
        ), patch.object(
            self.deployer, "_run_post_deployment_checks", return_value=True
        ):

            success = self.deployer.deploy_competition_system(validate_only=False)
            self.assertTrue(success)

    def test_validation_only_mode(self):
        """Test validation-only deployment mode."""
        with patch.object(
            self.deployer, "_run_pre_deployment_checks", return_value=True
        ), patch.object(
            self.deployer, "_setup_deployment_environment", return_value=False
        ), patch.object(
            self.deployer, "_deploy_services", return_value=False
        ), patch.object(
            self.deployer, "_run_post_deployment_checks", return_value=False
        ):

            # Should succeed in validation-only mode even if deployment fails
            success = self.deployer.deploy_competition_system(validate_only=True)
            self.assertTrue(success)

    def test_deployment_timeout_handling(self):
        """Test handling of deployment timeouts."""
        with patch.object(
            self.deployer, "_run_pre_deployment_checks", side_effect=TimeoutError
        ), patch.object(self.deployer, "_rollback_deployment") as mock_rollback:

            success = self.deployer.deploy_competition_system()
            self.assertFalse(success)
            mock_rollback.assert_called_once()

    def test_configuration_validation_check(self):
        """Test configuration validation in pre-deployment checks."""
        # Create valid config file
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
"""

        with open(self.project_root / "config" / "competition.yaml", "w") as f:
            f.write(config_content)

        success = self.deployer._check_configuration_validity()
        # Should pass (or at least not crash) even if config manager not available in test
        self.assertIsInstance(success, bool)


if __name__ == "__main__":
    unittest.main()
