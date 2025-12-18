#!/usr/bin/env python3
"""
Simple Mission Executor - Essential mission control functionality

Handles basic mission start/stop/pause commands via ROS2 topics.
"""

from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MissionState(Enum):
    """Mission execution states."""

    IDLE = "idle"
    ACTIVE = "active"
    PAUSED = "paused"
    COMPLETED = "completed"
    FAILED = "failed"


class MissionExecutor(Node):
    """Simple mission executor for basic start/stop/pause operations."""

    def __init__(self) -> None:
        """Initialize with essential ROS2 interfaces."""
        super().__init__("mission_executor")

        # ROS2 interfaces - simple and clear
        self.cmd_sub = self.create_subscription(
            String, "/mission/commands", self._command_callback, 10
        )
        self.status_pub = self.create_publisher(String, "/mission/status", 10)

        # Mission state
        self.current_mission = None
        self.mission_active = False

        self.get_logger().info("Mission Executor ready")

    def _command_callback(self, msg: String) -> None:
        """Handle incoming mission commands."""
        try:
            command = msg.data.lower()
            if command == "start":
                self.start_mission()
            elif command == "stop":
                self.stop_mission()
            elif command == "pause":
                self.pause_mission()
            else:
                self.get_logger().warn(f"Unknown command: {command}")
        except Exception as e:
            self.get_logger().error(f"Command processing failed: {e}")

    def start_mission(self) -> None:
        """Start a mission."""
        if not self.mission_active:
            self.mission_active = True
            self._publish_status("Mission started")
            self.get_logger().info("Mission started")

    def stop_mission(self) -> None:
        """Stop the current mission."""
        self.mission_active = False
        self._publish_status("Mission stopped")
        self.get_logger().info("Mission stopped")

    def pause_mission(self) -> None:
        """Pause the current mission."""
        if self.mission_active:
            self._publish_status("Mission paused")
            self.get_logger().info("Mission paused")

    def _publish_status(self, status: str) -> None:
        """Publish mission status."""
        msg = String()
        msg.data = status
        self.status_pub.publish(msg)


def main():
    """Main entry point."""
    rclpy.init()
    node = MissionExecutor()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    """Setup ROS QoS profiles for different message types."""
    self.sensor_qos = QoSProfile(
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=DurabilityPolicy.VOLATILE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
    )

    self.command_qos = QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
    )
