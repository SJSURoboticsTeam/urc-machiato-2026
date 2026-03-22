#!/usr/bin/env python3
"""
Mission Executor - Mission control and blackboard sync

Handles mission start/stop/pause commands, forwards to mission nodes,
and syncs mission_active and current_mission_phase with the blackboard.
"""

from enum import Enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger

# Unified blackboard (optional)
try:
    from core.unified_blackboard_client import UnifiedBlackboardClient
    from core.blackboard_keys import BlackboardKeys

    _BLACKBOARD_AVAILABLE = True
except ImportError:
    _BLACKBOARD_AVAILABLE = False
    BlackboardKeys = None


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

        # Forward commands to mission-specific topics/services
        self.sample_cmd_pub = self.create_publisher(
            String, "/mission/sample_collection_command", 10
        )
        self.delivery_start_client = self.create_client(
            Trigger, "/mission/delivery/start"
        )
        self.delivery_stop_client = self.create_client(
            Trigger, "/mission/delivery/stop"
        )
        self.waypoint_start_client = self.create_client(
            Trigger, "/mission/waypoint/start"
        )
        self.waypoint_stop_client = self.create_client(
            Trigger, "/mission/waypoint/stop"
        )

        # Unified blackboard for mission_active and current_mission_phase
        self._blackboard = None
        if _BLACKBOARD_AVAILABLE:
            try:
                self._blackboard = UnifiedBlackboardClient(self)
            except Exception:
                self._blackboard = None

        self.get_logger().info("Mission Executor ready")

    def _command_callback(self, msg: String) -> None:
        """Handle incoming mission commands and forward to mission nodes."""
        try:
            command = msg.data.lower().strip()
            if command == "start":
                self.start_mission()
            elif command == "stop":
                self.stop_mission()
            elif command == "pause":
                self.pause_mission()
            elif command == "start_sample_collection":
                self._start_sample_collection()
            elif command == "stop_sample_collection":
                self._stop_sample_collection()
            elif command == "start_delivery":
                self._start_delivery()
            elif command == "stop_delivery":
                self._stop_delivery()
            elif command == "start_waypoint":
                self._start_waypoint()
            elif command == "stop_waypoint":
                self._stop_waypoint()
            else:
                self.get_logger().warn(f"Unknown command: {command}")
        except Exception as e:
            self.get_logger().error(f"Command processing failed: {e}")

    def _sync_blackboard(
        self, mission_active: bool, current_mission_phase: str = ""
    ) -> None:
        """Write mission_active and current_mission_phase to blackboard."""
        if self._blackboard and BlackboardKeys:
            try:
                self._blackboard.set(BlackboardKeys.MISSION_ACTIVE, mission_active)
                if current_mission_phase:
                    self._blackboard.set(
                        BlackboardKeys.CURRENT_MISSION_PHASE, current_mission_phase
                    )
            except Exception:
                pass

    def _start_sample_collection(self) -> None:
        self.sample_cmd_pub.publish(String(data="start"))
        self.mission_active = True
        self.current_mission = "sample_collection"
        self._sync_blackboard(True, "sample_collection")
        self.get_logger().info("Started sample collection mission")

    def _stop_sample_collection(self) -> None:
        self.sample_cmd_pub.publish(String(data="stop"))
        self.mission_active = False
        self.current_mission = None
        self._sync_blackboard(False, "idle")
        self.get_logger().info("Stopped sample collection mission")

    def _start_delivery(self) -> None:
        if self.delivery_start_client.wait_for_service(timeout_sec=2.0):
            self.delivery_start_client.call_async(Trigger.Request())
            self.mission_active = True
            self.current_mission = "delivery"
            self._sync_blackboard(True, "delivery")
            self.get_logger().info("Started delivery mission")
        else:
            self.get_logger().warn("Delivery start service not available")

    def _stop_delivery(self) -> None:
        if self.delivery_stop_client.wait_for_service(timeout_sec=2.0):
            self.delivery_stop_client.call_async(Trigger.Request())
        self.mission_active = False
        self.current_mission = None
        self._sync_blackboard(False, "idle")
        self.get_logger().info("Stopped delivery mission")

    def _start_waypoint(self) -> None:
        if self.waypoint_start_client.wait_for_service(timeout_sec=2.0):
            self.waypoint_start_client.call_async(Trigger.Request())
            self.mission_active = True
            self.current_mission = "waypoint_navigation"
            self._sync_blackboard(True, "waypoint_navigation")
            self.get_logger().info("Started waypoint navigation mission")
        else:
            self.get_logger().warn("Waypoint start service not available")

    def _stop_waypoint(self) -> None:
        if self.waypoint_stop_client.wait_for_service(timeout_sec=2.0):
            self.waypoint_stop_client.call_async(Trigger.Request())
        self.mission_active = False
        self.current_mission = None
        self._sync_blackboard(False, "idle")
        self.get_logger().info("Stopped waypoint navigation mission")

    def start_mission(self) -> None:
        """Start a mission (generic)."""
        if not self.mission_active:
            self.mission_active = True
            self._sync_blackboard(True, "active")
            self._publish_status("Mission started")
            self.get_logger().info("Mission started")

    def stop_mission(self) -> None:
        """Stop the current mission."""
        self.mission_active = False
        self.current_mission = None
        self._sync_blackboard(False, "idle")
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
