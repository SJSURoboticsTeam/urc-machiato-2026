#!/usr/bin/env python3
"""
Competition Communication Bridge - URC Telemetry & Control

Handles competition-specific communication protocols and telemetry requirements.
Provides real-time data streaming, mission status, and operator control interfaces.

URC Requirements:
- Real-time telemetry streaming
- Competition status monitoring
- Operator command handling
- Data logging for judging
- Emergency stop coordination
"""

import time
from typing import Any, Dict, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, NavSatFix, Imu
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

from constants import (
    DEFAULT_COMPETITION_LOG_FILE,
    DEFAULT_DDS_DOMAIN_ID,
    DEFAULT_TELEMETRY_RATE_HZ,
    DEFAULT_WEBSOCKET_PORT,
)

# Import extracted manager classes
from .telemetry_manager import TelemetryManager
from .websocket_manager import WebSocketManager
from .mission_orchestrator import MissionOrchestrator
from .parameter_manager import ParameterManager
from .emergency_communicator import EmergencyCommunicator
from .spectrum_monitor import SpectrumMonitor


class CompetitionBridge(Node):
    """
    Competition Communication Bridge

    Manages all competition-specific communication using specialized manager classes:
    - TelemetryManager: Data collection and publishing
    - WebSocketManager: Client connections and messaging
    - MissionOrchestrator: URC mission coordination
    - ParameterManager: Configuration validation
    - EmergencyCommunicator: Emergency protocols
    - SpectrumMonitor: FCC compliance monitoring
    """

    def __init__(self):
        super().__init__("competition_bridge")

        # Initialize manager classes
        self.logger = self.get_logger()
        self.parameter_manager = ParameterManager(self, self.logger)
        self.telemetry_manager = TelemetryManager(self, self.logger)
        self.mission_orchestrator = MissionOrchestrator(self, self.logger)
        self.emergency_communicator = EmergencyCommunicator(self.logger, self.telemetry_manager.telemetry_data)
        self.spectrum_monitor = SpectrumMonitor(self.logger)

        # WebSocket manager (initialized after redundancy setup)
        self.websocket_manager = None

        # Get key parameters for backward compatibility
        self.websocket_port = self.parameter_manager.get_parameter("websocket_port", DEFAULT_WEBSOCKET_PORT)
        self.telemetry_rate = self.parameter_manager.get_parameter("telemetry_rate_hz", DEFAULT_TELEMETRY_RATE_HZ)
        self.max_clients = self.parameter_manager.get_parameter("max_websocket_clients", 10)
        self.redundancy_role = self.parameter_manager.get_parameter("redundancy_role", "primary")
        self.primary_domain_id = self.parameter_manager.get_parameter("primary_domain_id", DEFAULT_DDS_DOMAIN_ID)
        self.log_file = self.parameter_manager.get_parameter("competition_log_file", DEFAULT_COMPETITION_LOG_FILE)
        self.enable_logging = self.parameter_manager.get_parameter("enable_data_logging", True)

        # ROS2 subscriptions for telemetry data
        telemetry_qos = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=1)

        self.battery_sub = self.create_subscription(
            BatteryState, "/hardware/battery_state", self._battery_callback, telemetry_qos
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, "/hardware/gps", self._gps_callback, telemetry_qos
        )
        self.imu_sub = self.create_subscription(
            Imu, "/hardware/imu", self._imu_callback, telemetry_qos
        )
        self.velocity_sub = self.create_subscription(
            TwistStamped, "/hardware/chassis_velocity", self._velocity_callback, telemetry_qos
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self._odom_callback, telemetry_qos
        )

        # Mission status subscriptions
        self.mission_status_sub = self.create_subscription(
            String, "/mission/status", self._mission_status_callback, telemetry_qos
        )
        self.keyboard_status_sub = self.create_subscription(
            String, "/mission/keyboard_status", self._keyboard_status_callback, telemetry_qos
        )
        self.sample_status_sub = self.create_subscription(
            String, "/mission/sample_collection_status", self._sample_status_callback, telemetry_qos
        )
        self.system_status_sub = self.create_subscription(
            String, "/hardware/system_status", self._system_status_callback, telemetry_qos
        )

        # Emergency and safety subscriptions
        self.emergency_stop_sub = self.create_subscription(
            Bool, "/emergency_stop", self._emergency_stop_callback, telemetry_qos
        )
        self.boundary_sub = self.create_subscription(
            Bool, "/safety/boundary_violation", self._boundary_callback, telemetry_qos
        )

        # Publishers for commands
        self.mission_cmd_pub = self.create_publisher(String, "/mission/commands", 10)
        self.emergency_stop_pub = self.create_publisher(Bool, "/emergency_stop", 10)

        # Mission command publishers
        self.mission_cmd_pub = self.create_publisher(String, "/mission/commands", 10)

        # LED status publisher (URC requirement)
        self.led_pub = self.create_publisher(LedCommand, "/led/command", 10)  # type: ignore

        # Object highlighting for C2 display (URC autonomous navigation)
        self.object_highlight_pub = self.create_publisher(
            VisionDetection, "/c2/object_highlight", 10  # type: ignore
        )

        # Telemetry timer
        self.telemetry_timer = self.create_timer(1.0 / self.telemetry_rate, self._publish_telemetry)

        # Initialize WebSocket manager
        self._initialize_websocket_manager()

        self.logger.info("Competition Bridge initialized with extracted manager classes")

    def _initialize_websocket_manager(self) -> None:
        """Initialize WebSocket manager with command handlers."""
        self.websocket_manager = WebSocketManager(self.logger, self.telemetry_manager.telemetry_data)

        # Register command handlers
        self.websocket_manager.register_command_handler("start_mission", self._handle_start_mission)
        self.websocket_manager.register_command_handler("stop_mission", self._handle_stop_mission)
        self.websocket_manager.register_command_handler("emergency_stop", self._handle_emergency_stop)
        self.websocket_manager.register_command_handler("teleoperation_enable", self._handle_teleoperation_enable)
        self.websocket_manager.register_command_handler("teleoperation_disable", self._handle_teleoperation_disable)
        self.websocket_manager.register_command_handler("start_autonomous_navigation", self._handle_start_autonomous_navigation)
        self.websocket_manager.register_command_handler("target_reached", self._handle_target_reached)
        self.websocket_manager.register_command_handler("abort_to_previous", self._handle_abort_to_previous)
        self.websocket_manager.register_command_handler("highlight_object", self._handle_highlight_object)
        self.websocket_manager.register_command_handler("equipment_servicing_command", self._handle_equipment_command)
        self.websocket_manager.register_command_handler("get_compliance_status", self._handle_get_compliance_status)

        # Start WebSocket server
        if self.websocket_manager.start_server():
            self.logger.info(f"WebSocket server started on port {self.websocket_port}")

    # Telemetry callback methods
    def _battery_callback(self, msg: BatteryState) -> None:
        """Update battery telemetry."""
        self.telemetry_manager.update_battery_data(msg)

    def _gps_callback(self, msg: NavSatFix) -> None:
        """Update GPS telemetry."""
        self.telemetry_manager.update_gps_data(msg)
        self.mission_orchestrator.verify_gnss_compliance(msg)

    def _imu_callback(self, msg: Imu) -> None:
        """Update IMU telemetry."""
        self.telemetry_manager.update_imu_data(msg)

    def _velocity_callback(self, msg: TwistStamped) -> None:
        """Update velocity telemetry."""
        self.telemetry_manager.update_velocity_data(msg)

    def _odom_callback(self, msg: Odometry) -> None:
        """Update odometry telemetry."""
        self.telemetry_manager.update_odometry_data(msg)

    # Mission callback methods
    def _mission_status_callback(self, msg: String) -> None:
        """Update general mission status."""
        status = self.telemetry_manager._safe_json_parse(msg.data, "mission status")
        if status:
            mission = status.get("mission", "unknown")
            status_str = status.get("status", "unknown")
            self.telemetry_manager.update_mission_status(mission, status_str)

    def _keyboard_status_callback(self, msg: String) -> None:
        """Update keyboard mission status."""
        status = self.telemetry_manager._safe_json_parse(msg.data, "keyboard status")
        if status and status.get("mission") == "autonomous_keyboard":
            status_str = status.get("state", "unknown")
            self.telemetry_manager.update_mission_status("keyboard_typing", status_str)

    def _sample_status_callback(self, msg: String) -> None:
        """Update sample collection status."""
        status = self.telemetry_manager._safe_json_parse(msg.data, "sample status")
        if status and status.get("mission") == "sample_collection":
            samples = status.get("samples_collected", 0)
            cache = status.get("cache_used", 0)
            self.telemetry_manager.update_sample_data(samples, cache)

            # Update mission status if not in keyboard mode
            if self.telemetry_manager.telemetry_data.get("current_mission") != "keyboard_typing":
                status_str = status.get("state", "unknown")
                self.telemetry_manager.update_mission_status("sample_collection", status_str)

    def _system_status_callback(self, msg: String) -> None:
        """Update system health status."""
        status = self.telemetry_manager._safe_json_parse(msg.data, "system status")
        if status:
            health = status.get("system_healthy", False)
            errors = status.get("errors") if "errors" in status else None
            self.telemetry_manager.update_system_health(health, errors)

    def _emergency_stop_callback(self, msg: Bool) -> None:
        """Update emergency stop status."""
        self.telemetry_manager.update_emergency_status(emergency_stop=msg.data)

    def _boundary_callback(self, msg: Bool) -> None:
        """Update boundary violation status."""
        self.telemetry_manager.update_emergency_status(boundary_violation=msg.data)

    # WebSocket command handlers
    async def _handle_start_mission(self, websocket, mission_type: str) -> None:
        """Handle start mission command."""
        self.send_mission_command(f"start_{mission_type}")
        self.logger.info(f"Started mission: {mission_type}")

    async def _handle_stop_mission(self, websocket) -> None:
        """Handle stop mission command."""
        self.send_mission_command("stop")
        self.logger.info("Stopped mission")

    async def _handle_emergency_stop(self, websocket) -> None:
        """Handle emergency stop command."""
        self.emergency_communicator.trigger_emergency_stop()
        self.logger.critical("EMERGENCY STOP activated via WebSocket")

    async def _handle_teleoperation_enable(self, websocket) -> None:
        """Handle teleoperation enable command."""
        self.send_mission_command("enable_teleoperation")

    async def _handle_teleoperation_disable(self, websocket) -> None:
        """Handle teleoperation disable command."""
        self.send_mission_command("disable_teleoperation")

    async def _handle_start_autonomous_navigation(self, websocket, targets: Dict[str, Any]) -> None:
        """Handle start autonomous navigation command."""
        self.mission_orchestrator.start_autonomous_navigation(targets)
        self.logger.info("Autonomous navigation started via WebSocket")

    async def _handle_target_reached(self, websocket, target_type: str, target_index: int) -> None:
        """Handle target reached command."""
        self.mission_orchestrator.target_reached(target_type, target_index)

    async def _handle_abort_to_previous(self, websocket) -> Optional[Dict[str, Any]]:
        """Handle abort to previous target command."""
        return self.mission_orchestrator.abort_to_previous_target()

    async def _handle_highlight_object(self, websocket, object_name: str, confidence: float, bounding_box: List[float]) -> None:
        """Handle highlight object command."""
        self.mission_orchestrator.highlight_object(object_name, confidence, bounding_box)

    async def _handle_equipment_command(self, websocket, subcommand: str, **kwargs) -> None:
        """Handle equipment servicing command."""
        if subcommand == "complete_task":
            task_name = kwargs.get("task_name", "")
            self.mission_orchestrator.complete_equipment_task(task_name)
        elif subcommand == "set_launch_key":
            launch_key = kwargs.get("launch_key", "")
            self.mission_orchestrator.set_launch_key(launch_key)

    async def _handle_get_compliance_status(self, websocket) -> Dict[str, Any]:
        """Handle get compliance status command."""
        return self.mission_orchestrator.get_competition_status()

    # Utility methods
    def send_mission_command(self, command: str) -> None:
        """Send a mission command."""
        msg = String()
        msg.data = command
        self.mission_cmd_pub.publish(msg)

    def _publish_telemetry(self) -> None:
        """Publish telemetry data."""
        self.telemetry_manager.publish_telemetry()

        # Monitor spectrum compliance
        self.spectrum_monitor.monitor_spectrum_compliance()

    def get_competition_status(self) -> Dict[str, Any]:
        """Get comprehensive competition status."""
        return {
            "telemetry": self.telemetry_manager.get_telemetry_summary(),
            "mission": self.mission_orchestrator.get_mission_status(),
            "websocket": self.websocket_manager.get_connection_status() if self.websocket_manager else {},
            "emergency": self.emergency_communicator.get_emergency_status(),
            "spectrum": self.spectrum_monitor.get_spectrum_status(),
            "parameters": self.parameter_manager.get_parameter_summary(),
        }

    def destroy_node(self) -> None:
        """Clean shutdown of all managers."""
        if self.websocket_manager:
            self.websocket_manager.stop_server()

        self.emergency_communicator.deactivate_emergency_mode()

        super().destroy_node()
        self.logger.info("Competition Bridge shut down cleanly")


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = CompetitionBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
