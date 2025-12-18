#!/usr/bin/env python3
"""
ROS2 Mission Control Bridge

Connects the mission control system to ROS2 topics for dashboard integration.
Provides real-time mission status, progress updates, and command handling.

Features:
- Mission status publishing
- Command handling from dashboard
- Progress updates and telemetry
- Mission history and logging

Usage: python3 bridges/ros2_mission_bridge.py
"""

import json
import os
import sys
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

sys.path.append(os.path.join(os.path.dirname(__file__), ".."))
from missions.mission_executor import MissionExecutor


class ROS2MissionBridge(Node):
    """
    ROS2 bridge for mission control communication.

    Connects the real mission executor to ROS2 topics for dashboard integration.
    Publishes mission status and accepts mission commands.
    """

    def __init__(self):
        super().__init__("ros2_mission_bridge")

        # Initialize mission executor
        self.mission_executor = MissionExecutor()

        # Publishers for mission data
        self.mission_status_pub = self.create_publisher(String, "/mission/status", 10)
        self.mission_progress_pub = self.create_publisher(
            String, "/mission/progress", 10
        )
        self.mission_telemetry_pub = self.create_publisher(
            String, "/mission/telemetry", 10
        )

        # Subscribers for mission commands
        self.mission_command_sub = self.create_subscription(
            String, "/mission/commands", self.handle_mission_command, 10
        )

        # Health check service for monitoring
        self.health_check_srv = self.create_service(
            Trigger, "/mission/health_check", self.handle_health_check
        )

        # Timer for periodic status updates
        self.status_timer = self.create_timer(1.0, self.publish_mission_status)

        # Mission state tracking
        self.current_mission = None
        self.mission_start_time = None

        self.get_logger().info("ROS2 Mission Bridge initialized")

    def publish_mission_status(self):
        """Publish current mission status."""
        try:
            status_data = {
                "active": self.mission_executor.mission_active,
                "current_mission": self.mission_executor.current_mission,
                "timestamp": self.get_clock().now().nanoseconds / 1e9,
                "status": "active" if self.mission_executor.mission_active else "idle",
            }

            msg = String()
            msg.data = json.dumps(status_data)
            self.mission_status_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing mission status: {e}")

    def publish_mission_progress(self, mission_data):
        """Publish mission progress updates."""
        try:
            progress_data = {
                "mission_id": mission_data.get("id", "unknown"),
                "name": mission_data.get("name", "Unknown Mission"),
                "progress": mission_data.get("progress", 0),
                "current_task": mission_data.get("currentTask", "Initializing"),
                "next_task": mission_data.get("nextTask", "Pending"),
                "eta": mission_data.get("eta", "Unknown"),
                "waypoints": mission_data.get("waypoints", "0/0"),
                "samples": mission_data.get("samples", "0/0"),
                "analysis": mission_data.get("analysis", "0/0"),
                "timestamp": self.get_clock().now().nanoseconds / 1e9,
            }

            msg = String()
            msg.data = json.dumps(progress_data)
            self.mission_progress_pub.publish(msg)

            self.get_logger().info(
                f'Mission progress: {progress_data["name"]} - {progress_data["progress"]}%'
            )

        except Exception as e:
            self.get_logger().error(f"Error publishing mission progress: {e}")

    def publish_mission_telemetry(self, telemetry_data):
        """Publish mission telemetry data."""
        try:
            telemetry = {
                "position": telemetry_data.get("position", [0, 0, 0]),
                "orientation": telemetry_data.get("orientation", [0, 0, 0, 1]),
                "velocity": telemetry_data.get("velocity", [0, 0, 0]),
                "battery_level": telemetry_data.get("battery_level", 100),
                "temperature": telemetry_data.get("temperature", 25),
                "timestamp": self.get_clock().now().nanoseconds / 1e9,
            }

            msg = String()
            msg.data = json.dumps(telemetry)
            self.mission_telemetry_pub.publish(msg)

        except Exception as e:
            self.get_logger().error(f"Error publishing mission telemetry: {e}")

    def handle_mission_command(self, msg):
        """Handle mission commands from dashboard."""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get("command", "").lower()

            self.get_logger().info(f"Received mission command: {command}")

            if command == "start":
                self.handle_start_mission(command_data)
            elif command == "stop":
                self.handle_stop_mission()
            elif command == "pause":
                self.handle_pause_mission()
            elif command == "resume":
                self.handle_resume_mission()
            elif command.startswith("waypoint:"):
                self.handle_waypoint_command(command_data)
            elif command.startswith("sample:"):
                self.handle_sample_command(command_data)
            else:
                self.get_logger().warn(f"Unknown mission command: {command}")

        except json.JSONDecodeError as e:
            self.get_logger().error(f"Invalid mission command JSON: {e}")
        except Exception as e:
            self.get_logger().error(f"Error handling mission command: {e}")

    def handle_start_mission(self, command_data):
        """Handle mission start command."""
        try:
            mission_config = command_data.get("config", {})

            # Create mission data structure
            mission_data = {
                "id": f"mission_{int(self.get_clock().now().nanoseconds / 1e9)}",
                "name": mission_config.get("name", "Dashboard Mission"),
                "type": mission_config.get("type", "navigation"),
                "progress": 0,
                "currentTask": "Starting mission...",
                "nextTask": "Waypoint navigation",
                "eta": "5m 00s",
                "waypoints": "0/3",
                "samples": "0/3",
                "analysis": "0/3",
            }

            self.current_mission = mission_data
            self.mission_start_time = self.get_clock().now().nanoseconds / 1e9

            # Start the mission executor
            self.mission_executor.start_mission()

            # Publish initial progress
            self.publish_mission_progress(mission_data)

            # Simulate mission progress over time
            self.start_mission_progress_simulation(mission_data)

            self.get_logger().info(f'Started mission: {mission_data["name"]}')

        except Exception as e:
            self.get_logger().error(f"Error starting mission: {e}")

    def handle_health_check(self, request, response):
        """Health check service for monitoring mission system status."""
        try:
            # Check if mission executor is responsive
            mission_active = self.mission_executor.mission_active
            current_mission = self.mission_executor.current_mission

            response.success = True
            response.message = json.dumps(
                {
                    "component": "mission_bridge",
                    "status": "healthy",
                    "mission_active": mission_active,
                    "current_mission": current_mission,
                    "timestamp": self.get_clock().now().nanoseconds / 1e9,
                    "uptime": time.time() - getattr(self, "_start_time", time.time()),
                }
            )
        except Exception as e:
            response.success = False
            response.message = f"Health check failed: {str(e)}"

        return response

    def handle_stop_mission(self):
        """Handle mission stop command."""
        self.mission_executor.stop_mission()
        self.current_mission = None
        self.mission_start_time = None
        self.get_logger().info("Mission stopped")

    def handle_pause_mission(self):
        """Handle mission pause command."""
        self.mission_executor.pause_mission()
        if self.current_mission:
            self.current_mission["currentTask"] = "Mission Paused"
            self.publish_mission_progress(self.current_mission)
        self.get_logger().info("Mission paused")

    def handle_resume_mission(self):
        """Handle mission resume command."""
        if self.current_mission:
            self.current_mission["currentTask"] = "Resuming mission..."
            self.publish_mission_progress(self.current_mission)
        self.get_logger().info("Mission resumed")

    def handle_waypoint_command(self, command_data):
        """Handle waypoint navigation commands."""
        if self.current_mission:
            waypoint_id = command_data.get("waypoint_id", 0)
            total_waypoints = command_data.get("total_waypoints", 3)

            self.current_mission["waypoints"] = f"{waypoint_id}/{total_waypoints}"
            self.current_mission[
                "currentTask"
            ] = f"Navigating to waypoint {waypoint_id}"
            self.publish_mission_progress(self.current_mission)

    def handle_sample_command(self, command_data):
        """Handle sample collection commands."""
        if self.current_mission:
            sample_id = command_data.get("sample_id", 0)
            total_samples = command_data.get("total_samples", 3)

            self.current_mission["samples"] = f"{sample_id}/{total_samples}"
            self.current_mission["currentTask"] = f"Collecting sample {sample_id}"
            self.publish_mission_progress(self.current_mission)

    def start_mission_progress_simulation(self, mission_data):
        """Simulate mission progress over time."""
        import threading

        def simulate_progress():
            progress_steps = [
                {"progress": 10, "task": "Initializing systems...", "eta": "4m 30s"},
                {"progress": 25, "task": "Navigating to waypoint 1", "eta": "4m 00s"},
                {"progress": 40, "task": "Collecting sample 1", "eta": "3m 00s"},
                {"progress": 55, "task": "Navigating to waypoint 2", "eta": "2m 30s"},
                {"progress": 70, "task": "Collecting sample 2", "eta": "2m 00s"},
                {"progress": 85, "task": "Navigating to waypoint 3", "eta": "1m 00s"},
                {"progress": 95, "task": "Collecting sample 3", "eta": "0m 30s"},
                {"progress": 100, "task": "Mission complete", "eta": "0m 00s"},
            ]

            for step in progress_steps:
                if not self.mission_executor.mission_active:
                    break

                self.current_mission.update(step)
                self.publish_mission_progress(self.current_mission)

                # Simulate telemetry during mission
                telemetry = {
                    "position": [step["progress"] * 0.1, step["progress"] * 0.05, 0],
                    "orientation": [0, 0, 0, 1],
                    "velocity": [0.5, 0.3, 0],
                    "battery_level": max(20, 100 - step["progress"]),
                    "temperature": 25 + (step["progress"] * 0.1),
                }
                self.publish_mission_telemetry(telemetry)

                self.create_timer(3.0, lambda: None)  # Wait 3 seconds
                import time

                time.sleep(3)

            if self.mission_executor.mission_active:
                self.mission_executor.stop_mission()
                self.current_mission = None

        # Start simulation in background thread
        progress_thread = threading.Thread(target=simulate_progress, daemon=True)
        progress_thread.start()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        bridge = ROS2MissionBridge()
        self.get_logger().info("[START] ROS2 Mission Control Bridge started")
        self.get_logger().info("Publishing mission data to ROS2 topics...")
        self.get_logger().info("- /mission/status")
        self.get_logger().info("- /mission/progress")
        self.get_logger().info("- /mission/telemetry")
        self.get_logger().info("Listening for commands on /mission/commands")
        rclpy.spin(bridge)

    except KeyboardInterrupt:
        self.get_logger().info("\n[STOP] Received interrupt signal...")
    except Exception as e:
        self.get_logger().info(f"[ERROR] Bridge error: {e}")
    finally:
        rclpy.shutdown()
        self.get_logger().info("👋 ROS2 Mission Control Bridge shut down.")


if __name__ == "__main__":
    main()
