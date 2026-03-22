#!/usr/bin/env python3
"""
Unified Service Health Monitor - URC 2026 Robotics

Consolidated service health monitoring that combines the best features
from both scripts/monitoring/service_health_monitor.py and tools/scripts/monitoring/service_health_monitor.py

Features:
- Monitors critical ROS2 services and processes
- Automatic service restart with rate limiting
- Health status publishing
- Process-level monitoring for non-ROS services
- Integration with unified observability system

Services Monitored:
- State Machine Bridge (/state_machine/health_check)
- Mission Bridge (/mission/health_check)  
- WebSocket Simulation Bridge (process monitoring)

Author: URC 2026 Monitoring System Team
"""

import json
import os
import subprocess
import threading
import time
from typing import Dict, Any

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String


class UnifiedServiceHealthMonitor(Node):
    """Unified service health monitor with auto-restart capabilities."""

    def __init__(self):
        super().__init__("unified_service_health_monitor")

        # Configuration
        self.health_check_interval = 10.0  # seconds
        self.restart_timeout = 30.0  # seconds
        self.max_restart_attempts = 3  # per service per hour

        # Service configurations
        self.services = {
            "state_machine": {
                "service_name": "/state_machine/health_check",
                "process_name": "ros2_state_machine_bridge.py",
                "restart_command": "python3 bridges/ros2_state_machine_bridge.py",
                "restart_count": 0,
                "healthy": True,
                "last_restart": 0,
            },
            "mission": {
                "service_name": "/mission/health_check",
                "process_name": "ros2_mission_bridge.py",
                "restart_command": "python3 bridges/ros2_mission_bridge.py",
                "restart_count": 0,
                "healthy": True,
                "last_restart": 0,
            },
            "websocket": {
                "service_name": None,  # Process monitoring only
                "process_name": "dashboard_simulation_bridge.py",
                "restart_command": "python3 bridges/dashboard_simulation_bridge.py",
                "restart_count": 0,
                "healthy": True,
                "last_restart": 0,
            },
        }

        # Health clients for ROS2 services
        self.health_clients = {}
        self.callback_group = ReentrantCallbackGroup()

        # Initialize ROS2 clients
        self._setup_ros_clients()

        # Publishers
        self.health_status_pub = self.create_publisher(
            String, "/system/service_health", 10
        )

        # Health check timer
        self.health_timer = self.create_timer(
            self.health_check_interval, self._perform_health_checks
        )

        # Process monitoring thread
        self.process_monitor_thread = threading.Thread(
            target=self._monitor_processes, daemon=True
        )
        self.process_monitor_thread.start()

        self.get_logger().info(f"Monitoring {len(self.services)} services")

    def _setup_ros_clients(self):
        """Create ROS2 service clients for health checks."""
        from std_srvs.srv import Trigger

        for service_id, service_config in self.services.items():
            if service_config["service_name"]:
                self.health_clients[service_id] = self.create_client(
                    Trigger,
                    service_config["service_name"],
                    callback_group=self.callback_group,
                )

    def _perform_health_checks(self):
        """Perform comprehensive health checks for all services."""
        current_time = time.time()

        health_status = {
            "timestamp": current_time,
            "services": {},
            "overall_status": "healthy",
        }

        for service_id, service_config in self.services.items():
            if (
                service_config["service_name"]
                and service_config["service_name"] in self.health_clients
            ):
                health_status["services"][service_id] = self._check_service_health(
                    service_id
                )
            else:
                # For services without ROS services, check process status
                health_status["services"][service_id] = self._check_process_health(
                    service_id
                )

            # Update overall status
            if not health_status["services"][service_id]["healthy"]:
                health_status["overall_status"] = "degraded"

        # Publish health status
        msg = String()
        msg.data = json.dumps(health_status)
        self.health_status_pub.publish(msg)

        # Log summary
        healthy_count = sum(
            1 for s in health_status["services"].values() if s["healthy"]
        )
        total_count = len(health_status["services"])
        self.get_logger().info(
            f"Health check: {healthy_count}/{total_count} services healthy"
        )

        # Auto-restart failed services
        for service_id, status in health_status["services"].items():
            if not status["healthy"]:
                self._attempt_service_restart(service_id)

    def _check_service_health(self, service_id: str) -> Dict[str, Any]:
        """Check health of a ROS2 service."""
        client = self.health_clients[service_id]
        service_config = self.services[service_id]

        if not client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warning(
                f"Service {service_config['service_name']} not available"
            )
            service_config["healthy"] = False
            return {
                "healthy": False,
                "error": "service_unavailable",
                "last_check": time.time(),
            }

        # Call health check service
        from std_srvs.srv import TriggerRequest

        request = TriggerRequest()
        future = client.call_async(request)

        # Wait for response with timeout
        timeout_time = time.time() + 5.0
        while not future.done() and time.time() < timeout_time:
            time.sleep(0.1)

        if future.done():
            try:
                response = future.result()
                healthy = response.success
                service_config["healthy"] = healthy

                return {
                    "healthy": healthy,
                    "response": response.message if healthy else str(response.message),
                    "last_check": time.time(),
                }
            except Exception as e:
                service_config["healthy"] = False
                return {
                    "healthy": False,
                    "error": f"service_call_failed: {str(e)}",
                    "last_check": time.time(),
                }
        else:
            service_config["healthy"] = False
            return {
                "healthy": False,
                "error": "service_timeout",
                "last_check": time.time(),
            }

    def _check_process_health(self, service_id: str) -> Dict[str, Any]:
        """Check health of a process by monitoring if it's running."""
        service_config = self.services[service_id]
        process_name = service_config["process_name"]

        try:
            # Check if process is running
            result = subprocess.run(
                ["pgrep", "-f", process_name],
                capture_output=True,
                text=True,
                timeout=5.0,
            )

            is_running = result.returncode == 0
            service_config["healthy"] = is_running

            return {
                "healthy": is_running,
                "process_running": is_running,
                "last_check": time.time(),
            }

        except subprocess.TimeoutExpired:
            service_config["healthy"] = False
            return {
                "healthy": False,
                "error": "process_check_timeout",
                "last_check": time.time(),
            }

        except Exception as e:
            service_config["healthy"] = False
            return {
                "healthy": False,
                "error": f"process_check_failed: {str(e)}",
                "last_check": time.time(),
            }

    def _attempt_service_restart(self, service_id: str):
        """Attempt to restart a failed service."""
        service_config = self.services[service_id]
        current_time = time.time()

        # Check restart rate limiting (max 3 per hour)
        if (
            service_config["restart_count"] >= self.max_restart_attempts
            and current_time - service_config["last_restart"] < 3600
        ):  # 1 hour
            self.get_logger().warning(
                f"Rate limited restart for {service_id} (max {self.max_restart_attempts}/hour)"
            )
            return

        self.get_logger().info(f"Attempting to restart service: {service_id}")

        try:
            # Kill existing process if running
            process_name = service_config["process_name"]
            subprocess.run(["pkill", "-f", process_name], timeout=5.0)

            # Wait a moment for cleanup
            time.sleep(2.0)

            # Start new process
            restart_command = service_config["restart_command"]
            env = os.environ.copy()
            env["PYTHONPATH"] = f"{os.getcwd()}:{os.getcwd()}/autonomy/code"

            subprocess.Popen(
                restart_command.split(),
                env=env,
                cwd=os.getcwd(),
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )

            # Update restart tracking
            service_config["last_restart"] = current_time
            service_config["restart_count"] += 1

            self.get_logger().info(f"Restarted service: {service_id}")

        except Exception as e:
            self.get_logger().error(f"Failed to restart service {service_id}: {e}")

    def _monitor_processes(self):
        """Background thread to monitor process health."""
        while rclpy.ok():
            try:
                # Additional monitoring can be added here
                time.sleep(30.0)  # Check every 30 seconds
            except Exception as e:
                self.get_logger().error(f"Process monitoring error: {e}")
                time.sleep(10.0)

    def get_service_status(self) -> Dict[str, Any]:
        """Get current status of all monitored services."""
        return {
            service_id: {
                "healthy": config["healthy"],
                "last_restart": config["last_restart"],
                "restart_count": config["restart_count"],
            }
            for service_id, config in self.services.items()
        }


def main():
    """Main entry point."""
    rclpy.init()
    monitor = UnifiedServiceHealthMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
