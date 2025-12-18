#!/usr/bin/env python3
"""
Service Health Monitor
Monitors critical ROS2 services and automatically restarts failed services.
"""

import subprocess
import threading
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger


class ServiceHealthMonitor(Node):
    """
    Monitors health of critical ROS2 services and handles automatic restart.

    Critical Services Monitored:
    - State machine bridge
    - Mission execution system
    - Communication bridges
    - Vision processing
    - Navigation stack
    """

    def __init__(self):
        super().__init__("service_health_monitor")

        # Service configuration (using _service_configs to avoid Node._services internal attribute)
        self._service_configs = {
            "state_machine_bridge": {
                "service_name": "/state_machine/health",
                "restart_command": "ros2 launch urc_bringup state_machine_bridge.launch.py",
                "critical": True,
                "restart_attempts": 0,
                "max_restarts": 3,
                "last_restart": 0,
                "health_status": "unknown",
            },
            "mission_bridge": {
                "service_name": "/mission/health",
                "restart_command": "ros2 launch urc_bringup mission_bridge.launch.py",
                "critical": True,
                "restart_attempts": 0,
                "max_restarts": 3,
                "last_restart": 0,
                "health_status": "unknown",
            },
            "websocket_bridge": {
                "service_name": "/websocket/health",
                "restart_command": "ros2 launch urc_bridges websocket_bridge.launch.py",
                "critical": True,
                "restart_attempts": 0,
                "max_restarts": 5,
                "last_restart": 0,
                "health_status": "unknown",
            },
            "vision_processing": {
                "service_name": "/vision/health",
                "restart_command": "ros2 launch urc_vision vision_processing.launch.py",
                "critical": False,
                "restart_attempts": 0,
                "max_restarts": 2,
                "last_restart": 0,
                "health_status": "unknown",
            },
            "navigation": {
                "service_name": "/navigation/health",
                "restart_command": "ros2 launch urc_navigation navigation.launch.py",
                "critical": False,
                "restart_attempts": 0,
                "max_restarts": 2,
                "last_restart": 0,
                "health_status": "unknown",
            },
        }

        # Monitoring configuration
        self.monitoring_interval = 5.0  # seconds
        self.health_check_interval = self.monitoring_interval  # Alias for tests
        self.consecutive_failures_threshold = 3
        self.restart_cooldown = 30.0  # seconds between restart attempts
        self.overall_health_status = "healthy"

        # Health tracking
        self.service_failures = {name: 0 for name in self._service_configs.keys()}
        self.last_health_check = time.time()
        self.health_history = []

        # ROS2 clients for health checks
        self.health_clients = {}
        self._create_health_clients()

        # Publishers
        self.health_pub = self.create_publisher(String, "/system/service_health", 10)
        self.alert_pub = self.create_publisher(String, "/system/health_alerts", 10)

        # Start monitoring thread
        self.monitoring_active = True
        self.monitor_thread = threading.Thread(target=self._monitoring_loop)
        self.monitor_thread.daemon = True
        self.monitor_thread.start()

        self.get_logger().info("Service Health Monitor initialized")

    @property
    def services(self):
        """Get services configuration (readonly access to avoid Node.services conflict)."""
        return self._service_configs

    def _create_health_clients(self):
        """Create ROS2 service clients for health checks."""
        for service_name, config in self._service_configs.items():
            try:
                client = self.create_client(Trigger, config["service_name"])
                self.health_clients[service_name] = client
            except Exception as e:
                self.get_logger().warning(
                    f"Failed to create health client for {service_name}: {e}"
                )

    def _monitoring_loop(self):
        """Main monitoring loop."""
        while self.monitoring_active:
            try:
                self._perform_health_checks()
                self._publish_health_status()
                time.sleep(self.monitoring_interval)
            except Exception as e:
                self.get_logger().error(f"Error in monitoring loop: {e}")
                time.sleep(1.0)

    def _perform_health_checks(self):
        """Perform health checks on all monitored services."""
        current_time = time.time()
        self.last_health_check = current_time

        healthy_count = 0
        critical_failures = 0

        for service_name, config in self._service_configs.items():
            health_result = self._check_service_health(service_name)

            if health_result["healthy"]:
                healthy_count += 1
                self.service_failures[service_name] = 0
                config["health_status"] = "healthy"
            else:
                self.service_failures[service_name] += 1
                config["health_status"] = "unhealthy"

                # Check if restart is needed
                if self._should_restart_service(service_name):
                    self._restart_service(service_name)
                    if config["critical"]:
                        critical_failures += 1

        # Update overall health status
        if critical_failures > 0:
            self.overall_health_status = "critical"
        elif healthy_count < len(self._service_configs):
            self.overall_health_status = "degraded"
        else:
            self.overall_health_status = "healthy"

    def _check_service_health(self, service_name: str) -> Dict[str, Any]:
        """Check health of a specific service."""
        config = self._service_configs[service_name]
        client = self.health_clients.get(service_name)

        if not client:
            return {"healthy": False, "response": "No health client available"}

        try:
            # Wait for service to be available
            if not client.wait_for_service(timeout_sec=1.0):
                return {"healthy": False, "response": "Service not available"}

            # Call health check service
            future = client.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)

            if future.done() and future.result():
                response = future.result()
                return {"healthy": response.success, "response": response.message}
            else:
                return {"healthy": False, "response": "Health check timeout"}

        except Exception as e:
            return {"healthy": False, "response": f"Health check error: {e}"}

    def _check_process_health(self, service_name: str) -> Dict[str, Any]:
        """Check if a service process is running."""
        try:
            # Use pgrep to check if process is running
            result = subprocess.run(
                ["pgrep", "-f", service_name], capture_output=True, text=True, timeout=5
            )

            process_running = result.returncode == 0
            return {"healthy": process_running, "process_running": process_running}
        except subprocess.TimeoutExpired:
            return {"healthy": False, "process_running": False}
        except Exception as e:
            self.get_logger().error(
                f"Process health check error for {service_name}: {e}"
            )
            return {"healthy": False, "process_running": False}

    def _should_restart_service(self, service_name: str) -> bool:
        """Determine if a service should be restarted."""
        config = self._service_configs[service_name]
        failures = self.service_failures[service_name]
        current_time = time.time()

        # Check failure threshold
        if failures < self.consecutive_failures_threshold:
            return False

        # Check restart cooldown
        time_since_restart = current_time - config["last_restart"]
        if time_since_restart < self.restart_cooldown:
            return False

        # Check restart attempt limit
        if config["restart_attempts"] >= config["max_restarts"]:
            return False

        return True

    def _restart_service(self, service_name: str):
        """Restart a failed service."""
        config = self._service_configs[service_name]

        try:
            self.get_logger().warning(f"Restarting service: {service_name}")

            # Kill existing process (simplified - would need better process management)
            subprocess.run(["pkill", "-f", service_name], check=False, timeout=5)

            # Wait a moment
            time.sleep(2)

            # Start new process
            process = subprocess.Popen(
                config["restart_command"].split(),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

            # Update tracking
            config["restart_attempts"] += 1
            config["last_restart"] = time.time()

            # Publish alert
            alert_msg = String()
            alert_msg.data = f'{{"type": "service_restart", "service": "{service_name}", "attempt": {config["restart_attempts"]}}}'
            self.alert_pub.publish(alert_msg)

            self.get_logger().info(
                f"Service {service_name} restart initiated (attempt {config['restart_attempts']})"
            )

        except Exception as e:
            self.get_logger().error(f"Failed to restart service {service_name}: {e}")

    def _attempt_service_restart(self, service_name: str) -> bool:
        """Attempt to restart a service with error handling."""
        try:
            self._restart_service(service_name)
            return True
        except Exception as e:
            self.get_logger().error(f"Failed to restart service {service_name}: {e}")
            return False

    def _publish_health_status(self):
        """Publish overall health status."""
        health_data = {
            "timestamp": time.time(),
            "overall_status": self.overall_health_status,
            "services": {},
        }

        for service_name, config in self._service_configs.items():
            health_data["services"][service_name] = {
                "status": config["health_status"],
                "failures": self.service_failures[service_name],
                "restarts": config["restart_attempts"],
            }

        # Publish health status
        msg = String()
        msg.data = str(health_data).replace("'", '"')  # Simple JSON conversion
        self.health_pub.publish(msg)

    def get_overall_health_assessment(self) -> Dict[str, Any]:
        """Get comprehensive health assessment."""
        return {
            "overall_status": self.overall_health_status,
            "healthy_services": sum(
                1
                for s in self._service_configs.values()
                if s["health_status"] == "healthy"
            ),
            "total_services": len(self._service_configs),
            "critical_failures": sum(
                1
                for s in self._service_configs.values()
                if s["critical"] and s["health_status"] != "healthy"
            ),
            "total_restarts": sum(
                s["restart_attempts"] for s in self._service_configs.values()
            ),
            "last_check": self.last_health_check,
        }

    def destroy_node(self):
        """Clean shutdown."""
        self.monitoring_active = False
        if hasattr(self, "monitor_thread"):
            self.monitor_thread.join(timeout=5.0)
        # Clean up health clients
        if hasattr(self, "health_clients"):
            for client in self.health_clients.values():
                if client:
                    self.destroy_client(client)
        super().destroy_node()
