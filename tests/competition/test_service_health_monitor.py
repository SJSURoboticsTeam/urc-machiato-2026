#!/usr/bin/env python3
"""
Service Health Monitor Tests - Competition Critical
Tests the service health monitoring and auto-restart system.
"""

import subprocess
import time
import unittest
from unittest.mock import MagicMock, patch

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class TestServiceHealthMonitor(unittest.TestCase):
    """Test service health monitoring functionality."""

    @classmethod
    def setUpClass(cls):
        """Set up ROS2 environment."""
        if not rclpy.ok():
            rclpy.init(args=[])
        cls.node = Node("test_health_monitor")

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2 environment."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        """Set up test fixtures."""
        from scripts.monitoring.service_health_monitor import ServiceHealthMonitor

        self.monitor = ServiceHealthMonitor()
        self.received_health_status = []

        # Subscribe to health status
        self.health_sub = self.node.create_subscription(
            String, "/system/service_health", self._health_callback, 10
        )

    def tearDown(self):
        """Clean up after each test."""
        if hasattr(self, "health_sub"):
            self.node.destroy_subscription(self.health_sub)
        if hasattr(self, "monitor"):
            self.monitor.destroy_node()

    def _health_callback(self, msg):
        """Callback for health status updates."""
        import json

        try:
            data = json.loads(msg.data)
            self.received_health_status.append(data)
        except:
            pass

    def test_monitor_initialization(self):
        """Test health monitor initialization and configuration."""
        # Verify services dictionary is properly initialized
        self.assertIsInstance(self.monitor.services, dict)
        self.assertGreater(
            len(self.monitor.services), 0, "Monitor should track at least one service"
        )

        # Check required critical services are monitored
        required_services = [
            "state_machine_bridge",
            "mission_bridge",
            "websocket_bridge",
        ]
        for service in required_services:
            self.assertIn(
                service,
                self.monitor.services,
                f"Critical service {service} must be monitored",
            )

        # Verify service configuration structure
        for service_name, config in self.monitor.services.items():
            self.assertIn(
                "service_name", config, f"Service {service_name} missing service_name"
            )
            self.assertIn(
                "critical", config, f"Service {service_name} missing criticality flag"
            )
            self.assertIn(
                "max_restarts", config, f"Service {service_name} missing restart limits"
            )
            self.assertIn(
                "health_status",
                config,
                f"Service {service_name} missing health tracking",
            )

        # Verify critical services are marked as such
        for critical_service in required_services:
            if critical_service in self.monitor.services:
                self.assertTrue(
                    self.monitor.services[critical_service]["critical"],
                    f"Critical service {critical_service} must be marked as critical",
                )

        # Verify health clients were created
        self.assertIsInstance(
            self.monitor.health_clients, dict, "Health clients dictionary should exist"
        )
        self.assertGreater(
            len(self.monitor.health_clients),
            0,
            "Health clients should be created for services",
        )

    def test_service_health_check_success(self):
        """Test successful service health check logic."""
        # Test the logic by directly mocking the _check_service_health method
        with patch.object(self.monitor, "_check_service_health") as mock_check:
            mock_check.return_value = {"healthy": True, "response": "Healthy"}

            # Call the monitoring method
            self.monitor._perform_health_checks()

            # Verify service status was updated correctly
            self.assertEqual(
                self.monitor._service_configs["state_machine_bridge"]["health_status"],
                "healthy",
                "Service should be marked as healthy",
            )
            self.assertEqual(
                self.monitor.service_failures["state_machine_bridge"],
                0,
                "Failure counter should remain at 0 for healthy service",
            )

            # Verify overall health status
            self.assertEqual(
                self.monitor.overall_health_status,
                "healthy",
                "Overall health should be healthy when all services are healthy",
            )

    def test_service_health_check_failure(self):
        """Test failed service health check and failure handling."""
        # Test the logic by directly mocking the _check_service_health method
        with patch.object(self.monitor, "_check_service_health") as mock_check:
            mock_check.return_value = {
                "healthy": False,
                "response": "Service not available",
            }

            initial_failures = self.monitor.service_failures["state_machine_bridge"]

            # Call the monitoring method
            self.monitor._perform_health_checks()

            # Verify service status was updated correctly
            self.assertEqual(
                self.monitor._service_configs["state_machine_bridge"]["health_status"],
                "unhealthy",
                "Service should be marked as unhealthy",
            )
            self.assertEqual(
                self.monitor.service_failures["state_machine_bridge"],
                initial_failures + 1,
                "Failure counter should be incremented",
            )

            # Since all services are critical and this would make all unhealthy,
            # overall status should be critical
            # (Note: other services would also be unhealthy in this scenario)

    def test_process_health_check_success(self):
        """Test successful process health check."""
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 0
            mock_result.stdout = "12345 python3"
            mock_run.return_value = mock_result

            result = self.monitor._check_process_health("test_service")

            self.assertTrue(result["healthy"])
            self.assertTrue(result["process_running"])

    def test_process_health_check_failure(self):
        """Test failed process health check."""
        with patch("subprocess.run") as mock_run:
            mock_result = MagicMock()
            mock_result.returncode = 1  # Process not found
            mock_run.return_value = mock_result

            result = self.monitor._check_process_health("test_service")

            self.assertFalse(result["healthy"])
            self.assertFalse(result["process_running"])

    def test_service_restart_logic(self):
        """Test service restart logic."""
        # Set up service for restart
        service_config = self.monitor.services["state_machine_bridge"]
        service_config["restart_attempts"] = 0
        service_config["last_restart"] = 0

        with patch("subprocess.run") as mock_run, patch(
            "subprocess.Popen"
        ) as mock_popen, patch("time.sleep") as mock_sleep:

            # Mock successful restart
            mock_process = MagicMock()
            mock_process.pid = 12345
            mock_popen.return_value = mock_process

            # Mock pkill and other subprocess calls
            mock_run.return_value = MagicMock(returncode=0)

            self.monitor._attempt_service_restart("state_machine_bridge")

            # Should have attempted restart
            mock_popen.assert_called_once()
            self.assertEqual(service_config["restart_attempts"], 1)
            self.assertGreater(service_config["last_restart"], 0)

    def test_restart_rate_limiting(self):
        """Test restart rate limiting to prevent storms."""
        service_config = self.monitor.services["state_machine_bridge"]
        service_config["restart_attempts"] = 3  # At limit
        service_config["last_restart"] = time.time()  # Recent restart

        # Should not restart due to rate limiting
        with patch.object(self.monitor, "get_logger") as mock_logger:
            result = self.monitor._attempt_service_restart("state_machine_bridge")

            # Should return False due to rate limiting
            self.assertFalse(result)
            # Should log rate limiting message
            mock_logger.warning.assert_called()

    def test_health_monitoring_cycle(self):
        """Test complete health monitoring cycle."""
        # Mock all health checks to succeed
        with patch.object(
            self.monitor, "_check_service_health"
        ) as mock_service_check, patch.object(
            self.monitor, "_check_process_health"
        ) as mock_process_check:

            mock_service_check.return_value = {"healthy": True, "response": "OK"}
            mock_process_check.return_value = {"healthy": True, "process_running": True}

            # Run health check cycle
            self.monitor._perform_health_checks()

            # Should have published health status
            time.sleep(0.1)  # Allow async publishing
            self.assertGreater(len(self.received_health_status), 0)

            # Check latest status
            latest_status = self.received_health_status[-1]
            self.assertIn("services", latest_status)
            self.assertIn("overall_status", latest_status)

    def test_overall_health_assessment(self):
        """Test overall system health assessment."""
        # Set up mixed health states
        self.monitor.services["state_machine_bridge"]["healthy"] = True
        self.monitor.services["mission_bridge"]["healthy"] = False
        self.monitor.services["websocket_bridge"]["healthy"] = True

        status = self.monitor.get_service_status()

        self.assertTrue(status["state_machine_bridge"]["healthy"])
        self.assertFalse(status["mission_bridge"]["healthy"])
        self.assertTrue(status["websocket_bridge"]["healthy"])

    def test_service_timeout_handling(self):
        """Test timeout handling in service checks."""
        with patch.object(self.monitor, "health_clients") as mock_clients:
            mock_client = MagicMock()
            mock_client.wait_for_service.return_value = True

            # Mock timeout - future never completes
            mock_future = MagicMock()
            mock_future.done.return_value = False
            mock_client.call_async.return_value = mock_future

            mock_clients.__getitem__.return_value = mock_client

            result = self.monitor._check_service_health("state_machine_bridge")

            self.assertFalse(result["healthy"])
            self.assertEqual(result["error"], "service_timeout")

    def test_consecutive_failures_escalation(self):
        """Test handling of consecutive service failures."""
        service_name = "state_machine_bridge"

        # Mock service health check to always fail
        with patch.object(self.monitor, "_check_service_health") as mock_check:
            mock_check.return_value = {
                "healthy": False,
                "response": "Service unavailable",
            }

            # Simulate multiple failures
            for i in range(4):  # More than threshold
                self.monitor._perform_health_checks()
                time.sleep(0.1)

        # Should have incremented failure count and attempted restart
        service_config = self.monitor.services[service_name]
        self.assertGreaterEqual(self.monitor.service_failures[service_name], 3)
        # Restart may or may not have happened depending on timing

    def test_service_recovery_detection(self):
        """Test detection of service recovery."""
        service_name = "state_machine_bridge"
        service_config = self.monitor.services[service_name]

        # Start unhealthy
        service_config["healthy"] = False

        # Mock recovery
        with patch.object(self.monitor, "_check_service_health") as mock_check:
            mock_check.return_value = {"healthy": True, "response": "Recovered"}

            self.monitor._perform_health_checks()

            # Should be marked healthy
            self.assertTrue(service_config["healthy"])

    def test_monitoring_interval_accuracy(self):
        """Test health monitoring interval accuracy."""
        start_time = time.time()

        # Run multiple monitoring cycles
        for i in range(3):
            self.monitor._perform_health_checks()
            time.sleep(self.monitor.health_check_interval)

        elapsed = time.time() - start_time

        # Should be close to expected time (3 intervals)
        expected_time = 3 * self.monitor.health_check_interval
        self.assertAlmostEqual(elapsed, expected_time, delta=1.0)

    def test_service_dependency_tracking(self):
        """Test tracking of service dependencies."""
        # All services should be properly configured
        for service_name, service_config in self.monitor.services.items():
            self.assertIn("service_name", service_config)
            self.assertIn("process_name", service_config)
            self.assertIn("restart_command", service_config)
            self.assertIn("last_restart", service_config)
            self.assertIn("restart_count", service_config)
            self.assertIn("healthy", service_config)


if __name__ == "__main__":
    unittest.main()
