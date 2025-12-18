#!/usr/bin/env python3
"""
Service Health Monitor and Auto-Restart
Monitors critical ROS2 services and automatically restarts them if they fail.

This provides basic redundancy by ensuring services are restarted automatically
rather than requiring manual intervention during competition.
"""

import os
import subprocess
import threading
import time
from typing import Any, Dict, List, Optional

import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node


class ServiceHealthMonitor(Node):
    """
    Monitors critical ROS2 services and automatically restarts failed ones.

    Monitors:
    - ROS2 State Machine Bridge (/state_machine/health_check)
    - ROS2 Mission Bridge (/mission/health_check)
    - WebSocket Simulation Bridge (process monitoring)

    Actions:
    - Health check every 10 seconds
    - Auto-restart failed services
    - Log failures and recovery attempts
    - Publish health status for dashboard
    """

    def __init__(self):
        super().__init__("service_health_monitor")

        # Configuration
        self.health_check_interval = 10.0  # seconds
        self.restart_timeout = 30.0  # seconds to wait before declaring restart failed
        self.max_restart_attempts = 3  # per service per hour

        # Service configurations
        self.services = {
            "state_machine_bridge": {
                "service_name": "/state_machine/health_check",
                "process_name": "ros2_state_machine_bridge.py",
                "restart_command": "python3 bridges/ros2_state_machine_bridge.py",
                "last_restart": 0,
                "restart_count": 0,
                "healthy": True,
            },
            "mission_bridge": {
                "service_name": "/mission/health_check",
                "process_name": "ros2_mission_bridge.py",
                "restart_command": "python3 bridges/ros2_mission_bridge.py",
                "last_restart": 0,
                "restart_count": 0,
                "healthy": True,
            },
            "websocket_bridge": {
                "service_name": None,  # Process monitoring only
                "process_name": "dashboard_simulation_bridge.py",
                "restart_command": "python3 bridges/dashboard_simulation_bridge.py",
                "last_restart": 0,
                "restart_count": 0,
                "healthy": True,
            },
        }

        # ROS2 clients for health checks
        from std_srvs.srv import Trigger

        self.health_clients = {}
        self.callback_group = ReentrantCallbackGroup()

        for service_id, service_config in self.services.items():
            if service_config["service_name"]:
                self.health_clients[service_id] = self.create_client(
                    Trigger,
                    service_config["service_name"],
                    callback_group=self.callback_group,
                )

        # Publishers
        from std_msgs.msg import String

        self.health_status_pub = self.create_publisher(
            String, "/system/service_health", 10
        )

        # Health check timer
        self.health_timer = self.create_timer(
            self.health_check_interval, self._perform_health_checks
        )

        # Process monitoring
        self.process_monitor_thread = threading.Thread(
            target=self._monitor_processes, daemon=True
        )
        self.process_monitor_thread.start()

        self.get_logger().info("Service Health Monitor initialized")
        self.get_logger().info(f"Monitoring {len(self.services)} services")

    def _perform_health_checks(self):
        """Perform health checks on all monitored services."""
        health_status = {
            "timestamp": time.time(),
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
        import json

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

        # Check if process is running
        try:
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

        # Check restart rate limiting (max 3 per hour)
        current_time = time.time()
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
                for service_id, service_config in self.services.items():
                    # Additional process-level monitoring
                    if not service_config["healthy"]:
                        # Could implement more sophisticated monitoring here
                        pass

                time.sleep(5.0)  # Check every 5 seconds
            except Exception as e:
                self.get_logger().error(f"Process monitoring error: {e}")
                time.sleep(10.0)  # Back off on errors

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
    monitor = ServiceHealthMonitor()

    try:
        rclpy.spin(monitor)
    except KeyboardInterrupt:
        pass
    finally:
        monitor.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
