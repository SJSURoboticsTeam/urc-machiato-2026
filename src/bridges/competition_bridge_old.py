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

import asyncio
import json
import logging
import math
import threading
import time
from collections import deque
from typing import Any, Dict, List, Optional, Tuple, Callable, Awaitable

import rclpy
import websocket
try:
    import websockets
    from websockets.server import WebSocketServerProtocol
    WEBSOCKET_AVAILABLE = True
except ImportError:
    WEBSOCKET_AVAILABLE = False
    WebSocketServerProtocol = Any  # type: ignore
from autonomy_interfaces.action import NavigateToPose, PerformTyping
from autonomy_interfaces.msg import LedCommand, VisionDetection
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion, TwistStamped
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import Bool, Float32, Float32MultiArray, Header, String
from std_srvs.srv import Trigger

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

    Manages all competition-specific communication:
    - WebSocket telemetry streaming for judges/operators
    - Mission status aggregation and reporting
    - Operator command processing
    - Competition data logging
    - Emergency stop coordination across systems
    """

    def __init__(self):
        super().__init__("competition_bridge")

        # Declare parameters
        self.declare_parameter("websocket_port", DEFAULT_WEBSOCKET_PORT)
        self.declare_parameter(
            "telemetry_rate_hz", DEFAULT_TELEMETRY_RATE_HZ
        )  # 5Hz telemetry
        self.declare_parameter("enable_websocket_redundancy", True)
        self.declare_parameter(
            "redundancy_role", "primary"
        )  # primary, secondary, tertiary, emergency
        self.declare_parameter("enable_state_sync", True)
        self.declare_parameter("enable_dds_redundancy", True)
        self.declare_parameter("enable_dynamic_config", True)
        self.declare_parameter("enable_recovery_coordinator", True)
        self.declare_parameter("primary_domain_id", DEFAULT_DDS_DOMAIN_ID)
        self.declare_parameter("max_websocket_clients", DEFAULT_MAX_WEBSOCKET_CLIENTS)
        self.declare_parameter("competition_log_file", DEFAULT_COMPETITION_LOG_FILE)
        self.declare_parameter("enable_data_logging", True)

        # Adaptive telemetry parameters
        self.declare_parameter("adaptive_telemetry_enabled", True)
        self.declare_parameter("min_telemetry_rate", MIN_TELEMETRY_RATE_HZ)
        self.declare_parameter("max_telemetry_rate", MAX_TELEMETRY_RATE_HZ)
        self.declare_parameter(
            "bandwidth_target_utilization", BANDWIDTH_TARGET_UTILIZATION
        )  # Target 70% of max bandwidth
        self.declare_parameter("latency_target_ms", LATENCY_TARGET_MS)
        self.declare_parameter(
            "adaptation_rate", ADAPTATION_RATE
        )  # How aggressively to adapt
        self.declare_parameter(
            "bandwidth_measurement_window", BANDWIDTH_MEASUREMENT_WINDOW_SEC
        )  # Seconds for averaging

        # Get parameters using safe helper
        self.websocket_port = self._get_parameter_value("websocket_port", DEFAULT_WEBSOCKET_PORT)
        self.telemetry_rate = self._get_parameter_value("telemetry_rate_hz", DEFAULT_TELEMETRY_RATE_HZ)
        self.max_clients = self._get_parameter_value("max_websocket_clients", DEFAULT_MAX_WEBSOCKET_CLIENTS)
        self.redundancy_role = self._get_parameter_value("redundancy_role", "primary")
        self.state_sync_enabled = self._get_parameter_value("enable_state_sync", True)
        self.dds_redundancy_enabled = self._get_parameter_value("enable_dds_redundancy", True)
        self.dynamic_config_enabled = self._get_parameter_value("enable_dynamic_config", True)
        self.recovery_enabled = self._get_parameter_value("enable_recovery_coordinator", True)
        self.primary_domain_id = self._get_parameter_value("primary_domain_id", DEFAULT_DDS_DOMAIN_ID)
        self.log_file = self._get_parameter_value("competition_log_file", DEFAULT_COMPETITION_LOG_FILE)
        self.enable_logging = self._get_parameter_value("enable_data_logging", True)

        # Adaptive telemetry parameters
        self.adaptive_enabled = self._get_parameter_value("adaptive_telemetry_enabled", True)
        self.min_telemetry_rate = self._get_parameter_value("min_telemetry_rate", MIN_TELEMETRY_RATE_HZ)
        self.max_telemetry_rate = self._get_parameter_value("max_telemetry_rate", MAX_TELEMETRY_RATE_HZ)
        self.bandwidth_target = self._get_parameter_value("bandwidth_target_utilization", BANDWIDTH_TARGET_UTILIZATION)
        self.latency_target = self._get_parameter_value("latency_target_ms", LATENCY_TARGET_MS)
        self.adaptation_rate = self._get_parameter_value("adaptation_rate", ADAPTATION_RATE)
        self.bandwidth_window = self._get_parameter_value("bandwidth_measurement_window", BANDWIDTH_MEASUREMENT_WINDOW_SEC)

        # WebSocket server components
        self.websocket_server = None
        self.websocket_clients = set()
        self.websocket_thread = None

        # Advanced System Managers
        from core.dds_domain_redundancy_manager import get_dds_redundancy_manager
        from core.dynamic_config_manager import get_dynamic_config_manager
        from core.state_synchronization_manager import get_state_manager
        from websocket_redundancy_manager import (
            EndpointPriority,
            WebSocketEndpoint,
            get_redundancy_manager,
        )

        # WebSocket Redundancy
        self.redundancy_manager = get_redundancy_manager()
        self.endpoint_name = "competition_bridge"
        self.redundancy_enabled = self.get_parameter(
            "enable_websocket_redundancy"
        ).value

        # State Synchronization
        self.state_manager = get_state_manager(self.get_name())
        self.state_sync_enabled = self.get_parameter("enable_state_sync").value

        # DDS Domain Redundancy
        self.dds_manager = get_dds_redundancy_manager()
        self.dds_redundancy_enabled = self.get_parameter("enable_dds_redundancy").value

        # Dynamic Configuration
        self.config_manager = get_dynamic_config_manager()
        self.dynamic_config_enabled = self.get_parameter("enable_dynamic_config").value

        # Recovery Coordinator
        from core.recovery_coordinator import get_recovery_coordinator

        self.recovery_coordinator = get_recovery_coordinator()
        self.recovery_enabled = self.get_parameter("enable_recovery_coordinator").value

        # Safe telemetry update helper
        self._telemetry_update_errors = 0

    def _get_parameter_value(self, param_name: str, default_value: Optional[Any] = None) -> Any:
        """
        Safely get a ROS2 parameter value with optional default.

        Args:
            param_name: Name of the parameter to retrieve
            default_value: Default value if parameter retrieval fails

        Returns:
            Parameter value or default_value
        """
        try:
            return self.get_parameter(param_name).value
        except Exception as e:
            if default_value is not None:
                self.get_logger().warning(
                    f"Failed to get parameter '{param_name}', using default: {e}"
                )
                return default_value
            else:
                self.get_logger().error(f"Failed to get parameter '{param_name}': {e}")
                raise

    async def _send_websocket_message(self, websocket, message: Dict[str, Any], description: str = "") -> None:
        """
        Safely send a message via WebSocket with error handling.

        Args:
            websocket: WebSocket connection to send to
            message: Dictionary message to send
            description: Optional description for logging
        """
        try:
            await websocket.send(json.dumps(message))
            if description:
                self.get_logger().debug(f"Sent WebSocket message: {description}")
        except Exception as e:
            self.get_logger().warning(f"Failed to send WebSocket message ({description}): {e}")

    def _safe_json_parse(self, json_str: str, description: str = "") -> Optional[Dict[str, Any]]:
        """
        Safely parse JSON string with error handling.

        Args:
            json_str: JSON string to parse
            description: Optional description for logging

        Returns:
            Parsed dictionary or None if parsing failed
        """
        try:
            return json.loads(json_str)
        except (json.JSONDecodeError, TypeError) as e:
            self.get_logger().warning(f"Failed to parse JSON ({description}): {e}")
            return None

    def _safe_telemetry_update(
        self, key: str, value: Any, description: str = ""
    ) -> None:
        """
        Safely update telemetry data with proper error handling.

        Args:
            key: Telemetry data key to update
            value: Value to set
            description: Description for logging (optional)
        """
        try:
            self.telemetry_data[key] = value
        except (KeyError, TypeError, ValueError) as e:
            self._telemetry_update_errors += 1
            self.get_logger().warning(
                f"Failed to update telemetry {key}: {e}. "
                f"Errors this session: {self._telemetry_update_errors}"
            )

    def _safe_json_parse_telemetry(
        self, json_data: str, key: str, description: str = ""
    ) -> None:
        """
        Safely parse JSON and update telemetry with proper error handling.

        Args:
            json_data: JSON string to parse
            key: Telemetry data key to update
            description: Description for logging (optional)
        """
        try:
            data = json.loads(json_data)
            self.telemetry_data[key] = data
        except (json.JSONDecodeError, KeyError, TypeError) as e:
            self._telemetry_update_errors += 1
            self.get_logger().warning(
                f"Failed to parse telemetry JSON for {key}: {e}. "
                f"Errors this session: {self._telemetry_update_errors}"
            )

        # URC Band Configuration (competition restrictions)
        self.urc_band_config = {
            "900mhz": {
                "max_bandwidth_mhz": URC_900MHZ_MAX_BANDWIDTH,  # 8 MHz max for entire band
                "sub_bands": {
                    "low": {"range": URC_900MHZ_LOW_RANGE, "active": False},
                    "mid": {"range": URC_900MHZ_MID_RANGE, "active": False},
                    "high": {"range": URC_900MHZ_HIGH_RANGE, "active": False},
                },
                "current_subband": None,
                "frequency_hopping": True,  # Required for interference tolerance
                "channel_selection": True,  # Automatic channel selection
            },
            "2_4ghz": {
                "max_bandwidth_mhz": None,  # No FCC restriction, but interference-prone
                "interference_tolerant": True,
                "frequency_hopping": True,
                "channel_selection": True,
            },
            "current_band": "unknown",
            "subband_switching_enabled": True,
        }

        # Adaptive telemetry data structures
        self.adaptive_data = {
            "current_rate": self.telemetry_rate,
            "target_rate": self.telemetry_rate,
            "bandwidth_history": deque(
                maxlen=int(self.bandwidth_window * 10)
            ),  # 10Hz samples
            "latency_history": deque(maxlen=LATENCY_HISTORY_SIZE),
            "packet_loss_history": deque(maxlen=PACKET_LOSS_HISTORY_SIZE),
            "last_adaptation": time.time(),
            "adaptation_cooldown": 5.0,  # Minimum seconds between adaptations
            "bandwidth_estimator": None,  # Will be initialized if adaptive enabled
            "network_quality_score": 1.0,  # 0.0 (poor) to 1.0 (excellent)
        }

        # Data logging
        self.log_file_handle = None
        if self.enable_logging:
            try:
                self.log_file_handle = open(self.log_file, "a")
                self.get_logger().info(
                    f"Competition telemetry logging to {self.log_file}"
                )
            except Exception as e:
                self.get_logger().error(f"Failed to open log file: {e}")

        # Telemetry data storage
        self.telemetry_data = {
            "timestamp": 0.0,
            "mission_time": 0.0,
            "competition_start_time": time.time(),
            "system_health": "nominal",
            "battery_level": 0.0,
            "current": 0.0,
            "gps_position": {"lat": 0.0, "lon": 0.0, "alt": 0.0},
            "imu_data": {"accel": [0.0, 0.0, 0.0], "gyro": [0.0, 0.0, 0.0]},
            "velocity": {"linear": [0.0, 0.0, 0.0], "angular": [0.0, 0.0, 0.0]},
            "current_mission": "none",
            "mission_status": "idle",
            "samples_collected": 0,
            "cache_used": 0,
            "autonomous_mode": False,
            "emergency_stop": False,
            "boundary_violation": False,
            "terrain_traversability": 0.0,
            "joint_states": [],
            "motor_temperatures": [],
            "system_errors": [],
        }

        # URC Mission Orchestrator Data
        self.mission_orchestrator = {
            "autonomous_navigation": {
                "active": False,
                "current_target_index": 0,
                "targets": {
                    "gnss_locations": [],  # 2 precise GNSS points (<3m tolerance)
                    "ar_tags": [],  # 2 AR-tagged posts (2-20m tolerance)
                    "objects": [],  # 3 objects to detect and approach
                },
                "completed_targets": [],  # Stack for abort/return functionality
                "current_mode": "idle",  # idle, autonomous, teleoperation, arrived
                "last_target_reached": False,
            },
            "equipment_servicing": {
                "active": False,
                "tasks_completed": {
                    "sample_pickup": False,
                    "cache_container": False,
                    "drawer_open": False,
                    "panel_latch": False,
                    "autonomous_typing": False,
                    "usb_connection": False,
                    "hose_connection": False,
                    "valve_turn": False,
                    "button_push": False,
                    "switch_flip": False,
                    "knob_turn": False,
                },
                "current_task": None,
                "launch_key": None,  # For autonomous typing
            },
        }

        # GNSS Compliance Tracking
        self.gnss_compliance = {
            "wgs84_verified": False,
            "coordinate_format": "decimal_degrees",  # decimal_degrees, dms, etc.
            "last_coordinates": {"lat": 0.0, "lon": 0.0},
            "compliance_log": [],
        }

        # Spectrum Compliance Monitoring
        self.spectrum_compliance = {
            "fcc_compliant": True,
            "current_band": "unknown",
            "current_subband": None,
            "bandwidth_usage_mhz": 0.0,
            "interference_detected": False,
            "compliance_violations": [],
            "monitoring_active": True,
        }

        # Initialize 2.4 GHz Channel Safety Manager
        self.__init_channel_safety_manager()

        # Initialize Communication Redundancy Manager
        self.__init_communication_redundancy_manager()

        # 2.4 GHz Emergency Communication Protocols
        self.emergency_communication = {
            "active": False,
            "message_queue": [],
            "low_bandwidth_mode": False,
            "heartbeat_interval": 5.0,  # Send heartbeat every 5 seconds
            "last_heartbeat": 0,
            "reduced_telemetry_rate": 1.0,  # 1 Hz in emergency mode
        }

        # Defensive Network Resource Maximization (Optional)
        self.declare_parameter("defensive_maximization_enabled", False)
        self.declare_parameter("aggressive_channel_switching", False)
        self.declare_parameter(
            "bandwidth_reservation_ratio", 0.7
        )  # 70% bandwidth reservation
        self.declare_parameter("transmission_power_maximization", False)

        self.defensive_maximization_enabled = self.get_parameter(
            "defensive_maximization_enabled"
        ).value
        self.aggressive_channel_switching = self.get_parameter(
            "aggressive_channel_switching"
        ).value
        self.bandwidth_reservation_ratio = self.get_parameter(
            "bandwidth_reservation_ratio"
        ).value
        self.transmission_power_maximization = self.get_parameter(
            "transmission_power_maximization"
        ).value

        # QoS profiles
        telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1,
        )

        # Subscribers for telemetry data aggregation
        self.battery_sub = self.create_subscription(
            BatteryState,
            "/hardware/battery_state",
            self.battery_callback,
            telemetry_qos,
        )

        self.gps_sub = self.create_subscription(
            NavSatFix, "/hardware/gps", self.gps_callback, telemetry_qos
        )

        self.imu_sub = self.create_subscription(
            Imu, "/hardware/imu", self.imu_callback, telemetry_qos
        )

        self.velocity_sub = self.create_subscription(
            TwistStamped,
            "/hardware/chassis_velocity",
            self.velocity_callback,
            telemetry_qos,
        )

        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, telemetry_qos
        )

        self.mission_status_sub = self.create_subscription(
            String, "/mission/status", self.mission_status_callback, telemetry_qos
        )

        self.keyboard_status_sub = self.create_subscription(
            String,
            "/mission/keyboard_status",
            self.keyboard_status_callback,
            telemetry_qos,
        )

        self.sample_status_sub = self.create_subscription(
            String,
            "/mission/sample_collection_status",
            self.sample_status_callback,
            telemetry_qos,
        )

        self.system_status_sub = self.create_subscription(
            String,
            "/hardware/system_status",
            self.system_status_callback,
            telemetry_qos,
        )

        self.emergency_stop_sub = self.create_subscription(
            Bool, "/emergency_stop", self.emergency_stop_callback, telemetry_qos
        )

        self.boundary_sub = self.create_subscription(
            Bool, "/safety/boundary_violation", self.boundary_callback, telemetry_qos
        )

        # Publishers for operator commands
        self.mission_cmd_pub = self.create_publisher(String, "/mission/commands", 10)

        self.emergency_stop_pub = self.create_publisher(Bool, "/emergency_stop", 10)

        # LED status indicator publisher (URC requirement)
        self.led_pub = self.create_publisher(LedCommand, "/led/command", 10)

        # Object highlighting for C2 display (URC autonomous navigation)
        self.object_highlight_pub = self.create_publisher(
            VisionDetection, "/c2/object_highlight", 10
        )

        # Advanced system services
        if self.redundancy_enabled:
            self.redundancy_status_srv = self.create_service(
                Trigger,
                "/websocket_redundancy/status",
                self.handle_redundancy_status_request,
            )

        if self.state_sync_enabled:
            self.state_sync_status_srv = self.create_service(
                Trigger, "/state_sync/status", self.handle_state_sync_status_request
            )

        if self.dds_redundancy_enabled:
            self.dds_status_srv = self.create_service(
                Trigger, "/dds_redundancy/status", self.handle_dds_status_request
            )

        if self.dynamic_config_enabled:
            self.config_status_srv = self.create_service(
                Trigger, "/dynamic_config/status", self.handle_config_status_request
            )

        if self.recovery_enabled:
            self.recovery_status_srv = self.create_service(
                Trigger, "/recovery/status", self.handle_recovery_status_request
            )
            self.recovery_initiate_srv = self.create_service(
                Trigger, "/recovery/initiate", self.handle_recovery_initiate_request
            )

        # Telemetry publishing timer
        self.create_timer(1.0 / self.telemetry_rate, self.publish_telemetry)

        # Start WebSocket server
        self.start_websocket_server()

        self.get_logger().info("Competition Bridge initialized")
        self.get_logger().info(f"WebSocket server on port {self.websocket_port}")

    def start_websocket_server(self):
        """Start the WebSocket server for real-time telemetry."""
        try:
            import websockets

            # Initialize redundancy if enabled
            if self.redundancy_enabled:
                from .websocket_redundancy_manager import EndpointPriority

                # Determine endpoint priority based on role
                priority_map = {
                    "primary": EndpointPriority.PRIMARY,
                    "secondary": EndpointPriority.SECONDARY,
                    "tertiary": EndpointPriority.TERTIARY,
                    "emergency": EndpointPriority.EMERGENCY,
                }
                priority = priority_map.get(
                    self.redundancy_role, EndpointPriority.PRIMARY
                )

                # Configure telemetry scope based on role
                scope_map = {
                    "primary": [
                        "full_telemetry",
                        "commands",
                        "state",
                        "mission",
                        "safety",
                        "sensors",
                    ],
                    "secondary": ["state", "mission", "emergency", "commands"],
                    "tertiary": ["safety", "emergency", "location", "health"],
                    "emergency": ["emergency", "location", "health", "battery"],
                }
                telemetry_scope = scope_map.get(
                    self.redundancy_role, ["full_telemetry"]
                )

                # Register with redundancy manager
                endpoint = WebSocketEndpoint(
                    name=self.endpoint_name,
                    port=self.websocket_port,
                    priority=priority,
                    telemetry_scope=telemetry_scope,
                    max_clients=self.max_clients,
                )
                self.redundancy_manager.add_endpoint(endpoint)
                self.redundancy_manager.start_redundancy_system()

            self.get_logger().info(
                f"WebSocket redundancy enabled - role: {self.redundancy_role}, priority: {priority.value}"
            )

            # Initialize State Synchronization
            if self.state_sync_enabled:
                self.state_manager.start()
                self.state_manager.add_state_callback(self._on_state_change)
                self.get_logger().info("State synchronization enabled")

            # Initialize DDS Domain Redundancy
            if self.dds_redundancy_enabled:
                self.dds_manager.start()
                self.dds_manager.register_node(
                    self.get_name(),
                    f"python3 {__file__}",
                    domain_id=self.primary_domain_id,
                )
                self.dds_manager.add_failover_callback(self._on_dds_failover)
                self.get_logger().info("DDS domain redundancy enabled")

            # Initialize Dynamic Configuration
            if self.dynamic_config_enabled:
                self.config_manager.register_node(
                    self.get_name(),
                    {
                        "telemetry_rate_hz": self.telemetry_rate,
                        "websocket_port": self.websocket_port,
                        "max_clients": self.max_clients,
                    },
                )
                self.config_manager.add_update_callback(self._on_config_update)
                self.get_logger().info("Dynamic configuration enabled")

            # Initialize Recovery Coordinator
            if self.recovery_enabled:
                # Register all system managers for coordinated recovery
                if self.state_sync_enabled:
                    self.recovery_coordinator.register_system_manager(
                        "state", self.state_manager
                    )
                if self.dds_redundancy_enabled:
                    self.recovery_coordinator.register_system_manager(
                        "dds", self.dds_manager
                    )
                if self.dynamic_config_enabled:
                    self.recovery_coordinator.register_system_manager(
                        "config", self.config_manager
                    )
                if self.redundancy_enabled:
                    self.recovery_coordinator.register_system_manager(
                        "websocket", self.redundancy_manager
                    )

                # Add recovery progress callbacks
                self.recovery_coordinator.add_progress_callback(
                    self._on_recovery_progress
                )
                self.recovery_coordinator.add_completion_callback(
                    self._on_recovery_complete
                )

                self.get_logger().info(
                    "Recovery coordinator enabled with {} systems".format(
                        len(self.recovery_coordinator.system_managers)
                    )
                )

            self.websocket_thread = threading.Thread(target=self.websocket_server_loop)
            self.websocket_thread.daemon = True
            self.websocket_thread.start()
        except ImportError:
            self.get_logger().error(
                "websockets library not available - WebSocket server disabled"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to start WebSocket server: {e}")

    async def websocket_server_loop(self):
        """WebSocket server main loop."""
        try:
            import websockets

            async def handler(websocket: WebSocketServerProtocol, path: str) -> None:
                client_id = f"competition_{int(time.time() * 1000)}_{hash(websocket)}"

                # Handle through redundancy manager if enabled
                if self.redundancy_enabled:
                    await self.redundancy_manager.handle_client_connection(
                        websocket, self.endpoint_name, client_id
                    )
                else:
                    # Legacy handling without redundancy
                    self.websocket_clients.add(websocket)
                    self.get_logger().info(
                        f"WebSocket client connected ({len(self.websocket_clients)} total)"
                    )

                    try:
                        # Send initial telemetry
                        await self._send_websocket_message(websocket, self.telemetry_data, "initial telemetry")

                        # Handle incoming messages
                        async for message in websocket:
                            try:
                                await self.handle_websocket_message(websocket, message)
                            except Exception as e:
                                self.get_logger().error(
                                    f"WebSocket message handling error: {e}"
                                )

                    except websockets.exceptions.ConnectionClosed:
                        pass
                    finally:
                        self.websocket_clients.discard(websocket)
                        self.get_logger().info("WebSocket client disconnected")

            # Start the server (optimized for low latency)
            server = await websockets.serve(
                handler,
                "0.0.0.0",
                self.websocket_port,
                max_size=1048576,  # 1MB max message size
                max_queue=128,  # Increased queue for high throughput
                ping_interval=WEBSOCKET_PING_INTERVAL,  # Optimized ping frequency
                close_timeout=WEBSOCKET_CLOSE_TIMEOUT,  # Faster connection cleanup
                compression=None,  # Disable compression for speed
            )

            self.get_logger().info(
                f"WebSocket server started on ws://0.0.0.0:{self.websocket_port}"
            )
            await server.wait_closed()

        except Exception as e:
            self.get_logger().error(f"WebSocket server error: {e}")

    async def handle_websocket_message(self, websocket, message: str):
        """Handle incoming WebSocket messages from operators/judges."""
        data = self._safe_json_parse(message, "WebSocket message")
        if not data:
            return

        try:
            command_type = data.get("type", "")

            if command_type == "start_mission":
                mission_type = data.get("mission", "autonomous")
                self.send_mission_command(f"start_{mission_type}")

            elif command_type == "stop_mission":
                self.send_mission_command("stop")

            elif command_type == "emergency_stop":
                self.emergency_stop_pub.publish(Bool(data=True))
                self.get_logger().error("EMERGENCY STOP activated via WebSocket")

            elif command_type == "teleoperation_enable":
                self.send_mission_command("enable_teleoperation")

            elif command_type == "teleoperation_disable":
                self.send_mission_command("disable_teleoperation")

            elif command_type == "request_telemetry":
                # Send current telemetry
                await self._send_websocket_message(websocket, self.telemetry_data, "current telemetry")

            elif command_type == "system_command":
                # Handle system-level commands
                cmd = data.get("command", "")
                if cmd == "reboot":
                    self.handle_system_reboot()
                elif cmd == "shutdown":
                    self.handle_system_shutdown()

            elif command_type == "start_autonomous_navigation":
                # Start autonomous navigation mission
                targets = data.get("targets", {})
                self.start_autonomous_navigation(targets)
                self.get_logger().info("Autonomous navigation started via WebSocket")

            elif command_type == "target_reached":
                # Mark target as reached
                target_type = data.get("target_type", "")
                target_index = data.get("target_index", 0)
                self.target_reached(target_type, target_index)

            elif command_type == "abort_to_previous":
                # Abort and return to previous target
                previous_target = self.abort_to_previous_target()
                if previous_target:
                    await websocket.send(
                        json.dumps(
                            {
                                "type": "abort_acknowledged",
                                "previous_target": previous_target,
                            }
                        )
                    )

            elif command_type == "highlight_object":
                # Highlight object on C2 display
                object_name = data.get("object_name", "")
                confidence = data.get("confidence", 0.0)
                bounding_box = data.get("bounding_box", [])
                self.highlight_object(object_name, confidence, bounding_box)

            elif command_type == "equipment_servicing_command":
                # Handle equipment servicing commands
                subcommand = data.get("subcommand", "")
                if subcommand == "complete_task":
                    task_name = data.get("task_name", "")
                    self.complete_equipment_task(task_name)
                elif subcommand == "set_launch_key":
                    launch_key = data.get("launch_key", "")
                    self.set_launch_key(launch_key)

            elif command_type == "get_compliance_status":
                # Send compliance status to judges
                status = self.get_competition_status()
                await websocket.send(
                    json.dumps({"type": "compliance_status", "data": status})
                )

        except json.JSONDecodeError:
            self.get_logger().error(f"Invalid WebSocket message format: {message}")

    def send_mission_command(self, command: str):
        """Send mission command to mission executor."""
        cmd_msg = String()
        cmd_msg.data = command
        self.mission_cmd_pub.publish(cmd_msg)
        self.get_logger().info(f"Sent mission command: {command}")

    def handle_system_reboot(self):
        """Handle system reboot command."""
        self.get_logger().warning("System reboot requested via WebSocket")
        # In a real system, this would trigger a safe reboot
        # For now, just log it

    def handle_system_shutdown(self):
        """Handle system shutdown command."""
        self.get_logger().warning("System shutdown requested via WebSocket")
        # In a real system, this would trigger a safe shutdown

    def handle_redundancy_status_request(self, request, response):
        """Handle requests for WebSocket redundancy status."""
        try:
            if not self.redundancy_enabled:
                response.success = False
                response.message = "WebSocket redundancy not enabled"
                return response

            status = self.redundancy_manager.get_system_status()

            # Format status as JSON string
            status_json = json.dumps(status, indent=2, default=str)
            response.success = True
            response.message = status_json

        except Exception as e:
            response.success = False
            response.message = f"Error getting redundancy status: {str(e)}"

        return response

    def handle_state_sync_status_request(self, request, response):
        """Handle requests for state synchronization status."""
        try:
            if not self.state_sync_enabled:
                response.success = False
                response.message = "State synchronization not enabled"
                return response

            status = self.state_manager.get_system_status()

            # Format status as JSON string
            status_json = json.dumps(status, indent=2, default=str)
            response.success = True
            response.message = status_json

        except Exception as e:
            response.success = False
            response.message = f"Error getting state sync status: {str(e)}"

        return response

    def handle_dds_status_request(self, request, response):
        """Handle requests for DDS domain redundancy status."""
        try:
            if not self.dds_redundancy_enabled:
                response.success = False
                response.message = "DDS domain redundancy not enabled"
                return response

            status = self.dds_manager.get_system_status()

            # Format status as JSON string
            status_json = json.dumps(status, indent=2, default=str)
            response.success = True
            response.message = status_json

        except Exception as e:
            response.success = False
            response.message = f"Error getting DDS status: {str(e)}"

        return response

    def handle_config_status_request(self, request, response):
        """Handle requests for dynamic configuration status."""
        try:
            if not self.dynamic_config_enabled:
                response.success = False
                response.message = "Dynamic configuration not enabled"
                return response

            status = self.config_manager.get_system_status()

            # Format status as JSON string
            status_json = json.dumps(status, indent=2, default=str)
            response.success = True
            response.message = status_json

        except Exception as e:
            response.success = False
            response.message = f"Error getting config status: {str(e)}"

        return response

    def handle_recovery_status_request(self, request, response):
        """Handle requests for recovery coordinator status."""
        try:
            if not self.recovery_enabled:
                response.success = False
                response.message = "Recovery coordinator not enabled"
                return response

            status = self.recovery_coordinator.get_recovery_status()

            # Format status as JSON string
            status_json = json.dumps(status, indent=2, default=str)
            response.success = True
            response.message = status_json

        except Exception as e:
            response.success = False
            response.message = f"Error getting recovery status: {str(e)}"

        return response

    def handle_recovery_initiate_request(self, request, response):
        """Handle requests to initiate coordinated recovery."""
        try:
            if not self.recovery_enabled:
                response.success = False
                response.message = "Recovery coordinator not enabled"
                return response

            # Initiate recovery
            success = self.recovery_coordinator.initiate_recovery(
                "Manual recovery initiation"
            )

            if success:
                response.success = True
                response.message = "Coordinated recovery initiated"
            else:
                response.success = False
                response.message = "Recovery already in progress"

        except Exception as e:
            response.success = False
            response.message = f"Error initiating recovery: {str(e)}"

        return response

    # Telemetry data callbacks
    def battery_callback(self, msg: BatteryState) -> None:
        """Update battery telemetry."""
        self.telemetry_data["battery_level"] = msg.percentage
        self.telemetry_data["current"] = msg.current

    def gps_callback(self, msg: NavSatFix) -> None:
        """Update GPS telemetry and verify WGS84 compliance."""
        self.telemetry_data["gps_position"] = {
            "lat": msg.latitude,
            "lon": msg.longitude,
            "alt": msg.altitude,
        }

        # Verify GNSS compliance (URC requirement)
        compliance_ok = self.verify_gnss_compliance(msg)
        if not compliance_ok:
            self.get_logger().warning("GNSS compliance check failed")

    def imu_callback(self, msg: Imu) -> None:
        """Update IMU telemetry."""
        self.telemetry_data["imu_data"] = {
            "accel": [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z,
            ],
            "gyro": [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z,
            ],
        }

    def velocity_callback(self, msg: TwistStamped) -> None:
        """Update velocity telemetry."""
        self.telemetry_data["velocity"] = {
            "linear": [msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z],
            "angular": [msg.twist.angular.x, msg.twist.angular.y, msg.twist.angular.z],
        }

    def odom_callback(self, msg: Odometry):
        """Update odometry data for mission time calculation."""
        self.telemetry_data["mission_time"] = (
            time.time() - self.telemetry_data["competition_start_time"]
        )

    def mission_status_callback(self, msg: String):
        """Update general mission status."""
        status = self._safe_json_parse(msg.data, "mission status")
        if status:
            self._safe_telemetry_update(
                "current_mission", status.get("mission", "unknown"), "mission type"
            )
            self._safe_telemetry_update(
                "mission_status", status.get("status", "unknown"), "mission status"
            )

    def keyboard_status_callback(self, msg: String):
        """Update keyboard mission status."""
        status = self._safe_json_parse(msg.data, "keyboard status")
        if status and status.get("mission") == "autonomous_keyboard":
            self._safe_telemetry_update(
                "current_mission", "keyboard_typing", "keyboard mission"
            )
            self._safe_telemetry_update(
                "mission_status", status.get("state", "unknown"), "keyboard status"
            )

    def sample_status_callback(self, msg: String):
        """Update sample collection status."""
        status = self._safe_json_parse(msg.data, "sample status")
        if status and status.get("mission") == "sample_collection":
            self._safe_telemetry_update(
                "samples_collected",
                status.get("samples_collected", 0),
                "samples collected",
            )
            self._safe_telemetry_update(
                "cache_used", status.get("cache_used", 0), "cache usage"
            )
            if self.telemetry_data.get("current_mission") != "keyboard_typing":
                self._safe_telemetry_update(
                    "mission_status",
                    status.get("state", "unknown"),
                    "sample mission status",
                )

    def system_status_callback(self, msg: String):
        """Update system health status."""
        status = self._safe_json_parse(msg.data, "system status")
        if status:
            self._safe_telemetry_update(
                "system_health", status.get("system_healthy", False), "system health"
            )
            if "errors" in status:
                self._safe_telemetry_update(
                    "system_errors", status["errors"], "system errors"
                )

    def emergency_stop_callback(self, msg: Bool):
        """Update emergency stop status."""
        self.telemetry_data["emergency_stop"] = msg.data

    def boundary_callback(self, msg: Bool):
        """Update boundary violation status."""
        self.telemetry_data["boundary_violation"] = msg.data

    # LED Status Indicator System (URC Autonomous Navigation Requirement)
    def update_led_status(self, mission_type: str, status: str):
        """
        Update LED indicators per URC requirements.

        Args:
            mission_type: 'autonomous_navigation' or 'equipment_servicing'
            status: Status code for LED pattern
        """
        led_cmd = LedCommand()
        led_cmd.header.stamp = self.get_clock().now().to_msg()

        if mission_type == "autonomous_navigation":
            if status == "autonomous_mode":
                # Red: Autonomous operation
                led_cmd.status_code = 1
                led_cmd.red, led_cmd.green, led_cmd.blue = 1.0, 0.0, 0.0
                led_cmd.pattern = "solid"
                led_cmd.priority = 1
            elif status == "teleoperation":
                # Blue: Teleoperation (Manually driving)
                led_cmd.status_code = 2
                led_cmd.red, led_cmd.green, led_cmd.blue = 0.0, 0.0, 1.0
                led_cmd.pattern = "solid"
                led_cmd.priority = 1
            elif status == "target_reached":
                # Flashing Green: Successful arrival at target
                led_cmd.status_code = 3
                led_cmd.red, led_cmd.green, led_cmd.blue = 0.0, 1.0, 0.0
                led_cmd.pattern = "blinking"
                led_cmd.frequency = 2.0
                led_cmd.priority = 2  # Higher priority
                led_cmd.duration = 5.0  # Flash for 5 seconds
        elif mission_type == "equipment_servicing":
            if status == "task_completed":
                # Solid Green: Task completed
                led_cmd.status_code = 4
                led_cmd.red, led_cmd.green, led_cmd.blue = 0.0, 1.0, 0.0
                led_cmd.pattern = "solid"
                led_cmd.priority = 1
            elif status == "task_failed":
                # Solid Red: Task failed
                led_cmd.status_code = 5
                led_cmd.red, led_cmd.green, led_cmd.blue = 1.0, 0.0, 0.0
                led_cmd.pattern = "solid"
                led_cmd.priority = 2

        led_cmd.override = True  # Override lower priority commands
        self.led_pub.publish(led_cmd)

        self.get_logger().info(f"LED status updated: {mission_type} -> {status}")

    # Mission Orchestrator Methods
    def start_autonomous_navigation(self, targets: Dict[str, List[Dict[str, Any]]]):
        """
        Start autonomous navigation mission per URC requirements.

        Args:
            targets: Dict containing gnss_locations, ar_tags, objects
        """
        self.mission_orchestrator["autonomous_navigation"]["active"] = True
        self.mission_orchestrator["autonomous_navigation"]["targets"] = targets
        self.mission_orchestrator["autonomous_navigation"]["current_target_index"] = 0
        self.mission_orchestrator["autonomous_navigation"]["completed_targets"] = []
        self.mission_orchestrator["autonomous_navigation"][
            "current_mode"
        ] = "autonomous"

        self.update_led_status("autonomous_navigation", "autonomous_mode")
        self.get_logger().info("Autonomous Navigation mission started")

    def target_reached(self, target_type: str, target_index: int):
        """
        Mark target as reached and update LED status.

        Args:
            target_type: 'gnss_locations', 'ar_tags', or 'objects'
            target_index: Index of reached target
        """
        # Add to completed targets stack for abort/return
        completed_target = {
            "type": target_type,
            "index": target_index,
            "timestamp": time.time(),
        }
        self.mission_orchestrator["autonomous_navigation"]["completed_targets"].append(
            completed_target
        )

        # Update LED to flashing green
        self.update_led_status("autonomous_navigation", "target_reached")

        # Move to next target
        self.mission_orchestrator["autonomous_navigation"]["current_target_index"] += 1
        self.mission_orchestrator["autonomous_navigation"]["last_target_reached"] = True

        self.get_logger().info(f"Target reached: {target_type}[{target_index}]")

    def abort_to_previous_target(self) -> Optional[Dict[str, Any]]:
        """
        Abort current attempt and return to previous target (URC requirement).

        Returns:
            Previous target info or None if no previous targets
        """
        completed_targets = self.mission_orchestrator["autonomous_navigation"][
            "completed_targets"
        ]

        if not completed_targets:
            self.get_logger().warning("No previous targets to return to")
            return None

        # Get most recent completed target
        previous_target = completed_targets[-1]

        # Switch to teleoperation mode
        self.mission_orchestrator["autonomous_navigation"][
            "current_mode"
        ] = "teleoperation"
        self.update_led_status("autonomous_navigation", "teleoperation")

        self.get_logger().info(f"Aborted to previous target: {previous_target}")
        return previous_target

    def highlight_object(
        self, object_name: str, confidence: float, bounding_box: List[float]
    ):
        """
        Highlight detected object on C2 display (URC autonomous navigation requirement).

        Args:
            object_name: Name of detected object (mallet, pick, bottle)
            confidence: Detection confidence (0.0-1.0)
            bounding_box: [x, y, width, height] in image coordinates
        """
        detection = VisionDetection()
        detection.header.stamp = self.get_clock().now().to_msg()
        detection.object_class = object_name
        detection.confidence = confidence
        detection.bounding_box = bounding_box
        detection.highlight = True  # C2 display highlight flag

        self.object_highlight_pub.publish(detection)
        self.get_logger().info(
            f"Object highlighted: {object_name} (conf: {confidence:.2f})"
        )

    def complete_equipment_task(self, task_name: str):
        """
        Mark equipment servicing task as completed.

        Args:
            task_name: Name of completed task
        """
        if (
            task_name
            in self.mission_orchestrator["equipment_servicing"]["tasks_completed"]
        ):
            self.mission_orchestrator["equipment_servicing"]["tasks_completed"][
                task_name
            ] = True
            self.update_led_status("equipment_servicing", "task_completed")
            self.get_logger().info(f"Equipment task completed: {task_name}")
        else:
            self.get_logger().warning(f"Unknown equipment task: {task_name}")

    def set_launch_key(self, launch_key: str):
        """
        Set launch key for autonomous typing task.

        Args:
            launch_key: Launch key string for typing task
        """
        self.mission_orchestrator["equipment_servicing"]["launch_key"] = launch_key
        self.get_logger().info(f"Launch key set for autonomous typing: {launch_key}")

    # GNSS Compliance Verification
    def verify_gnss_compliance(self, navsat_msg: NavSatFix) -> bool:
        """
        Verify GNSS data uses WGS84 datum as required by URC.

        Args:
            navsat_msg: NavSatFix message from GPS

        Returns:
            True if compliant, False otherwise
        """
        # WGS84 coordinate validation
        if not (-90.0 <= navsat_msg.latitude <= 90.0):
            self.get_logger().error("Invalid latitude range for WGS84")
            self.gnss_compliance["wgs84_verified"] = False
            return False

        if not (-180.0 <= navsat_msg.longitude <= 180.0):
            self.get_logger().error("Invalid longitude range for WGS84")
            self.gnss_compliance["wgs84_verified"] = False
            return False

        # Update compliance tracking
        self.gnss_compliance["wgs84_verified"] = True
        self.gnss_compliance["last_coordinates"] = {
            "lat": navsat_msg.latitude,
            "lon": navsat_msg.longitude,
        }

        # Log compliance for judging
        compliance_entry = {
            "timestamp": time.time(),
            "latitude": navsat_msg.latitude,
            "longitude": navsat_msg.longitude,
            "altitude": navsat_msg.altitude,
            "datum": "WGS84",
            "compliant": True,
        }
        self.gnss_compliance["compliance_log"].append(compliance_entry)

        return True

    # Spectrum Compliance Monitoring
    def monitor_spectrum_compliance(self):
        """Monitor spectrum usage and ensure FCC compliance per URC requirements."""
        current_band = self.urc_band_config["current_band"]

        # Check bandwidth limits
        max_bandwidth = self._get_current_band_limit()
        current_usage = self._measure_current_bandwidth()

        if max_bandwidth and current_usage > max_bandwidth:
            violation = {
                "timestamp": time.time(),
                "type": "bandwidth_exceeded",
                "current_usage_mhz": current_usage,
                "limit_mhz": max_bandwidth,
                "band": current_band,
            }
            self.spectrum_compliance["compliance_violations"].append(violation)
            self.spectrum_compliance["fcc_compliant"] = False
            self.get_logger().error(
                f"FCC bandwidth violation: {current_usage:.1f}MHz > {max_bandwidth:.1f}MHz"
            )

        # Check sub-band compliance for 900MHz
        if current_band == "900mhz":
            current_subband = self.urc_band_config["900mhz"]["current_subband"]
            if not current_subband:
                violation = {
                    "timestamp": time.time(),
                    "type": "no_subband_selected",
                    "band": current_band,
                }
                self.spectrum_compliance["compliance_violations"].append(violation)
                self.spectrum_compliance["fcc_compliant"] = False

        # Check for interference
        if self._detect_interference():
            self.spectrum_compliance["interference_detected"] = True
            violation = {
                "timestamp": time.time(),
                "type": "interference_detected",
                "band": current_band,
            }
            self.spectrum_compliance["compliance_violations"].append(violation)

        self.spectrum_compliance["current_band"] = current_band
        self.spectrum_compliance["bandwidth_usage_mhz"] = current_usage

    def _measure_current_bandwidth(self) -> float:
        """Measure current bandwidth usage in MHz."""
        # Simplified bandwidth measurement - in real implementation,
        # this would analyze actual network traffic
        if self.adaptive_enabled:
            return self.adaptive_data["current_rate"] * 0.001  # Rough estimate
        return 1.0  # Conservative default

    def _detect_interference(self) -> bool:
        """Detect potential interference from other teams."""
        # Simplified interference detection - in real implementation,
        # this would analyze signal quality metrics
        current_band = self.urc_band_config["current_band"]
        if current_band == "2.4ghz":
            # 2.4GHz is interference-prone
            return True  # Assume potential interference
        return False

    # Enhanced 2.4 GHz Safety Mechanisms

    def detect_2_4ghz_specific_interference(self) -> Dict[str, float]:
        """
        Detect 2.4 GHz specific interference patterns.

        Returns:
            Dict of interference sources and their severity levels
        """
        if self.urc_band_config["current_band"] != "2.4ghz":
            return {}

        interference_sources = {
            "wifi_congestion": self._detect_wifi_congestion(),
            "team_interference": self._detect_competing_teams(),
            "environmental_noise": self._detect_environmental_noise(),
            "channel_overlap": self._detect_channel_overlap(),
        }

        total_interference = sum(interference_sources.values())

        # Trigger safety response if interference is critical
        if total_interference > 0.8:  # 80% interference threshold
            self._trigger_2_4ghz_safety_protocol(
                "critical_interference", interference_sources
            )

        return interference_sources

    def _detect_wifi_congestion(self) -> float:
        """Detect WiFi congestion from nearby devices."""
        # In real implementation, this would scan for WiFi networks
        # and measure channel utilization
        return 0.3  # Mock value - 30% congestion

    def _detect_competing_teams(self) -> float:
        """Detect interference from other URC teams."""
        # Look for URC-specific communication patterns
        # or known team communication signatures
        return 0.2  # Mock value - low interference from teams

    def _detect_environmental_noise(self) -> float:
        """Detect environmental interference (microwaves, cordless phones, etc.)."""
        # Environmental noise detection
        return 0.1  # Mock value - minimal environmental noise

    def _detect_channel_overlap(self) -> float:
        """Detect overlapping WiFi channels."""
        current_channel = getattr(self, "current_wifi_channel", 6)
        # Check if current channel overlaps with adjacent channels
        overlap_penalty = 0.0
        if current_channel in [1, 6, 11]:  # Non-overlapping channels
            overlap_penalty = 0.0
        else:
            overlap_penalty = 0.4  # High overlap penalty
        return overlap_penalty

    def _trigger_2_4ghz_safety_protocol(self, trigger_type: str, data: Dict = None):
        """
        Trigger 2.4 GHz specific safety protocols.

        Args:
            trigger_type: Type of safety trigger
            data: Additional data for the trigger
        """
        self.get_logger().warning(f"2.4 GHz safety protocol triggered: {trigger_type}")

        if trigger_type == "critical_interference":
            # Immediate channel switch
            self.attempt_safe_channel_switch()
        elif trigger_type == "communication_loss":
            # Activate emergency communication mode
            self.activate_emergency_communication_mode()
        elif trigger_type == "bandwidth_degradation":
            # Reduce communication load
            self.reduce_communication_load()

    def monitor_2_4ghz_link_quality(self) -> Dict[str, float]:
        """
        Monitor 2.4 GHz link quality beyond basic connectivity.

        Returns:
            Dict of link quality metrics
        """
        if self.urc_band_config["current_band"] != "2.4ghz":
            return {}

        metrics = {
            "packet_loss_rate": self._measure_packet_loss(),
            "latency_variation": self._measure_jitter(),
            "signal_strength": self._measure_rssi(),
            "throughput_degradation": self._measure_bandwidth_drop(),
        }

        # Check thresholds and trigger safety protocols
        if metrics["packet_loss_rate"] > 0.05:  # 5% loss threshold
            self._trigger_2_4ghz_safety_protocol("packet_loss")
        elif metrics["latency_variation"] > 100:  # 100ms jitter
            self._trigger_2_4ghz_safety_protocol("jitter")
        elif metrics["signal_strength"] < -70:  # Weak signal
            self._trigger_2_4ghz_safety_protocol("weak_signal")

        return metrics

    def _measure_packet_loss(self) -> float:
        """Measure packet loss rate."""
        # In real implementation, track sent vs received packets
        return 0.02  # Mock value - 2% loss

    def _measure_jitter(self) -> float:
        """Measure latency variation (jitter)."""
        # Calculate jitter from recent latency samples
        return 25.0  # Mock value - 25ms jitter

    def _measure_rssi(self) -> float:
        """Measure Received Signal Strength Indicator."""
        # Get RSSI from WiFi interface
        return -45.0  # Mock value - good signal strength

    def _measure_bandwidth_drop(self) -> float:
        """Measure throughput degradation."""
        # Compare current vs expected bandwidth
        return 0.1  # Mock value - 10% degradation

    # Channel Safety Manager

    def __init_channel_safety_manager(self):
        """Initialize channel safety manager."""
        self.channel_safety_manager = {
            "available_channels": [1, 6, 11],  # Non-overlapping WiFi channels
            "current_channel": 6,
            "channel_quality_history": {},  # Track channel reliability
            "last_channel_switch": 0,
            "channel_switch_cooldown": 30.0,  # 30 seconds between switches
            "validation_timeout": 5.0,  # 5 seconds to validate new channel
        }

        # Initialize quality history for each channel
        for channel in self.channel_safety_manager["available_channels"]:
            self.channel_safety_manager["channel_quality_history"][channel] = []

    def attempt_safe_channel_switch(self) -> bool:
        """
        Attempt to safely switch to a better WiFi channel.

        Returns:
            True if switch successful, False otherwise
        """
        if (
            time.time() - self.channel_safety_manager["last_channel_switch"]
            < self.channel_safety_manager["channel_switch_cooldown"]
        ):
            self.get_logger().info("Channel switch on cooldown")
            return False

        # Find best available channel
        best_channel = self._find_best_available_channel()
        if best_channel == self.channel_safety_manager["current_channel"]:
            self.get_logger().info("Current channel is already optimal")
            return True

        # Attempt safe channel switch
        return self.safe_channel_switch(best_channel)

    def safe_channel_switch(self, target_channel: int) -> bool:
        """
        Safely switch channels with validation and rollback capability.

        Args:
            target_channel: Target WiFi channel (1, 6, 11)

        Returns:
            True if switch successful, False otherwise
        """
        if target_channel not in self.channel_safety_manager["available_channels"]:
            self.get_logger().error(f"Invalid channel: {target_channel}")
            return False

        # Pre-switch validation
        if not self._validate_channel_safety(target_channel):
            self.get_logger().warning(
                f"Channel {target_channel} failed safety validation"
            )
            return False

        # Execute switch with rollback capability
        old_channel = self.channel_safety_manager["current_channel"]
        self.get_logger().info(
            f"Switching from channel {old_channel} to {target_channel}"
        )

        if not self._execute_channel_switch(target_channel):
            self.get_logger().error(
                f"Failed to execute channel switch to {target_channel}"
            )
            return False

        # Post-switch validation
        if not self._verify_channel_stability(
            target_channel, self.channel_safety_manager["validation_timeout"]
        ):
            self.get_logger().warning(
                f"Channel {target_channel} failed stability verification, rolling back"
            )
            return self._emergency_channel_rollback(old_channel)

        # Switch successful
        self.channel_safety_manager["current_channel"] = target_channel
        self.channel_safety_manager["last_channel_switch"] = time.time()

        self.get_logger().info(f"Successfully switched to channel {target_channel}")
        return True

    def _find_best_available_channel(self) -> int:
        """Find the best available channel based on quality history."""
        channel_scores = {}

        for channel in self.channel_safety_manager["available_channels"]:
            quality_history = self.channel_safety_manager["channel_quality_history"][
                channel
            ]
            if quality_history:
                # Calculate average quality score (higher is better)
                avg_quality = sum(quality_history[-10:]) / len(
                    quality_history[-10:]
                )  # Last 10 measurements
                channel_scores[channel] = avg_quality
            else:
                channel_scores[channel] = 0.5  # Neutral score for untested channels

        # Return channel with highest score
        return max(channel_scores, key=channel_scores.get)

    def _validate_channel_safety(self, channel: int) -> bool:
        """Validate that a channel is safe to switch to."""
        # Check if channel is in allowed range
        if channel not in [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11]:
            return False

        # Check recent quality history
        quality_history = self.channel_safety_manager["channel_quality_history"].get(
            channel, []
        )
        if quality_history and len(quality_history) >= 3:
            recent_avg = sum(quality_history[-3:]) / 3
            if recent_avg < 0.3:  # Poor quality threshold
                return False

        return True

    def _execute_channel_switch(self, channel: int) -> bool:
        """Execute the actual channel switch command."""
        try:
            # In real implementation, this would send command to WiFi hardware
            # For now, simulate the switch
            import subprocess

            # Example: iwconfig wlan0 channel {channel}
            # subprocess.run(['iwconfig', 'wlan0', 'channel', str(channel)], check=True)
            self.get_logger().info(f"Simulated channel switch to {channel}")
            return True
        except Exception as e:
            self.get_logger().error(f"Channel switch execution failed: {e}")
            return False

    def _verify_channel_stability(self, channel: int, timeout: float) -> bool:
        """Verify that the new channel is stable."""
        start_time = time.time()
        stable_readings = 0
        required_stable_readings = 3

        while time.time() - start_time < timeout:
            # Take quality measurement
            quality = self._measure_channel_quality(channel)
            if quality > 0.6:  # Stability threshold
                stable_readings += 1
                if stable_readings >= required_stable_readings:
                    return True
            else:
                stable_readings = 0  # Reset on poor reading

            time.sleep(0.5)  # Check every 500ms

        return False

    def _emergency_channel_rollback(self, target_channel: int) -> bool:
        """Emergency rollback to a known good channel."""
        try:
            self._execute_channel_switch(target_channel)
            self.channel_safety_manager["current_channel"] = target_channel
            self.get_logger().warning(f"Emergency rollback to channel {target_channel}")
            return True
        except Exception as e:
            self.get_logger().error(f"Emergency rollback failed: {e}")
            return False

    def _measure_channel_quality(self, channel: int) -> float:
        """Measure quality of a specific channel."""
        # In real implementation, this would measure actual channel quality
        # For simulation, return mock values
        base_quality = 0.8  # Good baseline
        noise_factor = (channel % 3) * 0.1  # Some channels have more noise
        return max(0.0, min(1.0, base_quality - noise_factor))

    def update_channel_quality_history(self, channel: int, quality: float):
        """Update quality history for a channel."""
        if channel not in self.channel_safety_manager["channel_quality_history"]:
            self.channel_safety_manager["channel_quality_history"][channel] = []

        history = self.channel_safety_manager["channel_quality_history"][channel]
        history.append(quality)

        # Keep only recent history
        if len(history) > 50:
            history.pop(0)

    # Emergency Communication Protocols

    def activate_emergency_communication_mode(self):
        """Activate emergency communication mode for degraded 2.4 GHz links."""
        if self.emergency_communication["active"]:
            return  # Already in emergency mode

        self.get_logger().warning("Activating 2.4 GHz emergency communication mode")

        self.emergency_communication["active"] = True
        self.emergency_communication["low_bandwidth_mode"] = True

        # Reduce message frequency and size
        self._reduce_telemetry_rate()
        self._prioritize_emergency_messages()
        self._switch_to_robust_modulation()

        # Start emergency heartbeat
        self._start_emergency_heartbeat()

    def _reduce_telemetry_rate(self):
        """Reduce telemetry rate for emergency communication."""
        original_rate = self.telemetry_rate
        self.telemetry_rate = self.emergency_communication["reduced_telemetry_rate"]
        self.get_logger().info(
            f"Reduced telemetry rate: {original_rate}Hz -> {self.telemetry_rate}Hz"
        )

    def _prioritize_emergency_messages(self):
        """Prioritize critical messages in emergency mode."""
        # Clear non-critical message queues
        # In real implementation, this would filter WebSocket messages
        self.get_logger().info("Prioritized emergency messages")

    def _switch_to_robust_modulation(self):
        """Switch to more robust modulation scheme."""
        # In real implementation, this would change WiFi modulation
        # from high-throughput to robust modes
        self.get_logger().info("Switched to robust modulation scheme")

    def _start_emergency_heartbeat(self):
        """Start sending emergency heartbeat signals."""
        self.emergency_communication["last_heartbeat"] = time.time()
        self.get_logger().info("Started emergency heartbeat transmission")

    def send_emergency_heartbeat(self):
        """Send minimal emergency heartbeat signal."""
        if not self.emergency_communication["active"]:
            return

        current_time = time.time()
        if (
            current_time - self.emergency_communication["last_heartbeat"]
            < self.emergency_communication["heartbeat_interval"]
        ):
            return

        emergency_packet = {
            "type": "emergency_heartbeat",
            "timestamp": current_time,
            "rover_status": "alive_but_impaired",
            "communication_mode": "emergency",
            "last_position": self.telemetry_data.get("gps_position", {}),
            "critical_systems": {
                "battery_level": self.telemetry_data.get("battery_level", 0),
                "emergency_stop": self.telemetry_data.get("emergency_stop", False),
                "autonomous_mode": self.telemetry_data.get("autonomous_mode", False),
            },
        }

        # Send minimal packet (would use WebSocket in real implementation)
        self._send_minimal_packet(emergency_packet)
        self.emergency_communication["last_heartbeat"] = current_time

    def _send_minimal_packet(self, packet: Dict):
        """Send a minimal emergency packet."""
        # In real implementation, this would send via the most reliable method available
        self.get_logger().info(f"Sent emergency packet: {packet['type']}")

    def deactivate_emergency_mode(self):
        """Deactivate emergency communication mode."""
        if not self.emergency_communication["active"]:
            return

        self.get_logger().info("Deactivating emergency communication mode")

        self.emergency_communication["active"] = False
        self.emergency_communication["low_bandwidth_mode"] = False

        # Restore normal communication parameters
        self.telemetry_rate = self.get_parameter("telemetry_rate_hz").value
        self.get_logger().info(
            f"Restored normal telemetry rate: {self.telemetry_rate}Hz"
        )

    # Communication Redundancy and Failover Manager

    def __init_communication_redundancy_manager(self):
        """Initialize communication redundancy manager."""
        self.redundancy_manager = {
            "primary_channel": 6,  # 2.4 GHz channel 6
            "backup_channels": [1, 11],  # Alternative 2.4 GHz channels
            "emergency_band": "900mhz",  # Fallback to 900 MHz
            "failover_active": False,
            "current_failover_level": 0,  # 0=normal, 1=channel_switch, 2=band_switch, 3=emergency
            "last_failover_attempt": 0,
            "failover_cooldown": 60.0,  # 60 seconds between failover attempts
            "communication_health_score": 1.0,  # 0.0 to 1.0
        }

    def execute_emergency_failover(self) -> bool:
        """
        Execute emergency failover when 2.4 GHz becomes unusable.

        Returns:
            True if failover successful, False otherwise
        """
        if (
            time.time() - self.redundancy_manager["last_failover_attempt"]
            < self.redundancy_manager["failover_cooldown"]
        ):
            self.get_logger().info("Failover on cooldown")
            return False

        self.redundancy_manager["last_failover_attempt"] = time.time()
        self.get_logger().warning("Executing emergency communication failover")

        # Step 1: Try alternative 2.4 GHz channels
        for channel in self.redundancy_manager["backup_channels"]:
            if self._test_channel_viability(channel):
                success = self.safe_channel_switch(channel)
                if success:
                    self.redundancy_manager["failover_active"] = True
                    self.redundancy_manager["current_failover_level"] = 1
                    self.get_logger().info(
                        f"Failover successful: switched to channel {channel}"
                    )
                    return True

        # Step 2: Failover to 900 MHz band
        if self._switch_to_900mhz_band():
            self.redundancy_manager["failover_active"] = True
            self.redundancy_manager["current_failover_level"] = 2
            self.get_logger().info("Failover successful: switched to 900 MHz band")
            return True

        # Step 3: Enter degraded emergency mode
        self._enter_degraded_communication_mode()
        self.redundancy_manager["failover_active"] = True
        self.redundancy_manager["current_failover_level"] = 3
        self.get_logger().warning("Entered degraded emergency communication mode")
        return True

    def _test_channel_viability(self, channel: int) -> bool:
        """Test if a channel is viable for communication."""
        try:
            # Quick channel quality test
            quality = self._measure_channel_quality(channel)
            return quality > 0.5  # Minimum viability threshold
        except Exception as e:
            self.get_logger().error(f"Channel viability test failed for {channel}: {e}")
            return False

    def _switch_to_900mhz_band(self) -> bool:
        """Switch to 900 MHz band as emergency fallback."""
        try:
            # Use existing band switching mechanism
            self.set_urc_band("900mhz", "low")
            return True
        except Exception as e:
            self.get_logger().error(f"900 MHz band switch failed: {e}")
            return False

    def _enter_degraded_communication_mode(self):
        """Enter degraded emergency communication mode."""
        self.activate_emergency_communication_mode()

        # Additional degraded mode settings
        self.redundancy_manager["communication_health_score"] = 0.2  # Very degraded

        # Notify all systems of degraded communication
        self._broadcast_communication_degradation()

    def _broadcast_communication_degradation(self):
        """Broadcast communication degradation to all systems."""
        degradation_msg = {
            "type": "communication_degraded",
            "level": self.redundancy_manager["current_failover_level"],
            "health_score": self.redundancy_manager["communication_health_score"],
            "timestamp": time.time(),
        }

        # Would broadcast to ROS2 topics and WebSocket in real implementation
        self.get_logger().warning(f"Communication degraded: {degradation_msg}")

    def attempt_failover_recovery(self) -> bool:
        """
        Attempt to recover from failover state.

        Returns:
            True if recovery successful, False otherwise
        """
        if not self.redundancy_manager["failover_active"]:
            return True  # Already in normal mode

        current_level = self.redundancy_manager["current_failover_level"]

        # Try to recover step by step
        if current_level >= 3:
            # Try to exit emergency mode
            if self._test_primary_channel_health():
                self.deactivate_emergency_mode()
                self.redundancy_manager["current_failover_level"] = 2
                current_level = 2

        if current_level >= 2:
            # Try to switch back to 2.4 GHz
            if self.safe_channel_switch(self.redundancy_manager["primary_channel"]):
                self.set_urc_band("2.4ghz")
                self.redundancy_manager["current_failover_level"] = 1
                current_level = 1

        if current_level >= 1:
            # Try to return to normal operation
            self.redundancy_manager["failover_active"] = False
            self.redundancy_manager["current_failover_level"] = 0
            self.redundancy_manager["communication_health_score"] = 1.0
            self.get_logger().info("Communication failover recovery successful")
            return True

        return False

    def _test_primary_channel_health(self) -> bool:
        """Test if primary communication channel is healthy."""
        primary_channel = self.redundancy_manager["primary_channel"]
        quality = self._measure_channel_quality(primary_channel)
        return quality > 0.7  # Good health threshold

    def reduce_communication_load(self):
        """Reduce communication load during high interference."""
        # Temporarily reduce telemetry rate
        if not self.emergency_communication["active"]:
            original_rate = self.telemetry_rate
            self.telemetry_rate = max(1.0, self.telemetry_rate * 0.5)  # Reduce by half
            self.get_logger().info(
                f"Temporarily reduced telemetry rate: {original_rate}Hz -> {self.telemetry_rate}Hz"
            )

            # Schedule rate restoration
            # In real implementation, this would use a timer

    def _monitor_2_4ghz_safety(self):
        """Monitor and manage 2.4 GHz safety mechanisms."""
        current_time = time.time()

        # Update channel quality history
        current_channel = self.channel_safety_manager["current_channel"]
        quality = self._measure_channel_quality(current_channel)
        self.update_channel_quality_history(current_channel, quality)

        # Check for interference patterns
        interference = self.detect_2_4ghz_specific_interference()

        # Monitor link quality
        link_metrics = self.monitor_2_4ghz_link_quality()

        # Send emergency heartbeat if in emergency mode
        self.send_emergency_heartbeat()

        # Attempt failover recovery if in degraded state
        if self.redundancy_manager["failover_active"]:
            self.attempt_failover_recovery()

        # Defensive resource maximization (if enabled)
        if (
            hasattr(self, "defensive_maximization_enabled")
            and self.defensive_maximization_enabled
        ):
            self._execute_defensive_resource_maximization()

    def _execute_defensive_resource_maximization(self):
        """Execute defensive network resource maximization strategies."""
        # Only maximize if communication is healthy
        if self.redundancy_manager["communication_health_score"] > 0.7:
            # Attempt to improve channel position
            if (
                time.time() - self.channel_safety_manager["last_channel_switch"] > 300
            ):  # Every 5 minutes
                self.attempt_safe_channel_switch()

            # Optimize transmission parameters
            self._optimize_transmission_parameters()

            # Maintain bandwidth reservation
            self._maintain_bandwidth_reservation()

    def _optimize_transmission_parameters(self):
        """Optimize transmission parameters for maximum reliability."""
        if not self.defensive_maximization_enabled:
            return

        # Adjust transmission power (if supported by hardware)
        if self.transmission_power_maximization:
            self._maximize_transmission_power()

        # Optimize packet size for channel conditions
        self._optimize_packet_size()

        # Adjust retry limits for critical packets
        self._optimize_retry_parameters()

    def _maximize_transmission_power(self):
        """Maximize transmission power within regulatory limits."""
        # In real implementation, this would adjust WiFi transmission power
        # while staying within FCC limits for the band
        max_power_dbm = 20  # Maximum allowed power for 2.4 GHz
        current_power = self._get_current_transmission_power()

        if current_power < max_power_dbm:
            self._set_transmission_power(max_power_dbm)
            self.get_logger().info(
                f"Maximized transmission power to {max_power_dbm} dBm"
            )

    def _optimize_packet_size(self):
        """Optimize packet size based on channel conditions."""
        # Measure channel conditions
        packet_loss = self._measure_packet_loss()
        jitter = self._measure_jitter()

        # Adjust packet size for optimal throughput
        if packet_loss < 0.01 and jitter < 10:  # Excellent conditions
            optimal_size = 1500  # Maximum Ethernet frame
        elif packet_loss < 0.05:  # Good conditions
            optimal_size = 1000
        else:  # Poor conditions
            optimal_size = 500  # Smaller packets for reliability

        self._set_optimal_packet_size(optimal_size)

    def _optimize_retry_parameters(self):
        """Optimize retry parameters for critical communications."""
        # Increase retry count for critical packets
        critical_retry_count = 7  # IEEE 802.11 maximum
        normal_retry_count = 3

        # Set different retry counts for different traffic types
        self._set_retry_parameters(
            {
                "critical": critical_retry_count,
                "normal": normal_retry_count,
                "low_priority": 1,
            }
        )

    def _maintain_bandwidth_reservation(self):
        """Maintain bandwidth reservation for critical traffic."""
        if not self.defensive_maximization_enabled:
            return

        reservation_ratio = self.bandwidth_reservation_ratio

        # Reserve bandwidth for critical traffic types
        reservations = {
            "emergency": reservation_ratio * 0.3,  # 30% of reserved bandwidth
            "telemetry": reservation_ratio * 0.4,  # 40% of reserved bandwidth
            "control": reservation_ratio * 0.3,  # 30% of reserved bandwidth
        }

        # Apply bandwidth reservations
        for traffic_type, ratio in reservations.items():
            self._reserve_bandwidth(traffic_type, ratio)

    def _reserve_bandwidth(self, traffic_type: str, ratio: float):
        """Reserve bandwidth for specific traffic type."""
        # In real implementation, this would configure QoS parameters
        # or traffic shaping rules to guarantee bandwidth
        self.get_logger().debug(f"Reserved {ratio:.1%} bandwidth for {traffic_type}")

    def _get_current_transmission_power(self) -> float:
        """Get current transmission power in dBm."""
        # Mock implementation
        return 15.0  # 15 dBm typical

    def _set_transmission_power(self, power_dbm: float):
        """Set transmission power."""
        # Mock implementation - would interface with WiFi hardware
        self.get_logger().info(f"Setting transmission power to {power_dbm} dBm")

    def _set_optimal_packet_size(self, size_bytes: int):
        """Set optimal packet size."""
        # Mock implementation - would configure network stack
        self.get_logger().debug(f"Setting optimal packet size to {size_bytes} bytes")

    def _set_retry_parameters(self, retry_config: Dict[str, int]):
        """Set retry parameters for different traffic types."""
        # Mock implementation - would configure WiFi retry parameters
        self.get_logger().debug(f"Setting retry parameters: {retry_config}")

    def enable_defensive_maximization(self):
        """Enable defensive network resource maximization."""
        self.defensive_maximization_enabled = True
        self.get_logger().info("Enabled defensive network resource maximization")

    def disable_defensive_maximization(self):
        """Disable defensive network resource maximization."""
        self.defensive_maximization_enabled = False
        self.get_logger().info("Disabled defensive network resource maximization")

    def get_defensive_maximization_status(self) -> Dict[str, Any]:
        """Get status of defensive resource maximization."""
        return {
            "enabled": self.defensive_maximization_enabled,
            "aggressive_channel_switching": self.aggressive_channel_switching,
            "bandwidth_reservation_ratio": self.bandwidth_reservation_ratio,
            "transmission_power_maximization": self.transmission_power_maximization,
            "current_channel": self.channel_safety_manager["current_channel"],
            "communication_health_score": self.redundancy_manager[
                "communication_health_score"
            ],
            "failover_active": self.redundancy_manager["failover_active"],
        }

    def publish_telemetry(self):
        """Publish telemetry data via WebSocket and log to file."""
        # Update timestamp
        self.telemetry_data["timestamp"] = time.time()

        # Synchronize critical state across bridges
        if self.state_sync_enabled:
            try:
                # Sync telemetry rate
                self.state_manager.update_state(
                    "telemetry_rate_hz", self.telemetry_rate
                )

                # Sync system health
                system_health = "nominal"
                if self.telemetry_data.get("emergency_stop"):
                    system_health = "emergency"
                elif len(self.telemetry_data.get("system_errors", [])) > 0:
                    system_health = "degraded"

                self.state_manager.update_state("system_health", system_health)
                self.state_manager.update_state(
                    "battery_level", self.telemetry_data.get("battery_level", 0.0)
                )

            except Exception as e:
                self.get_logger().error(f"State synchronization error: {e}")

        # Monitor spectrum compliance (URC requirement)
        if self.spectrum_compliance["monitoring_active"]:
            self.monitor_spectrum_compliance()

        # Monitor 2.4 GHz safety mechanisms
        if self.urc_band_config["current_band"] == "2.4ghz":
            self._monitor_2_4ghz_safety()

        # Convert to JSON
        telemetry_json = json.dumps(self.telemetry_data)

        # Send to WebSocket clients
        if self.websocket_clients:
            # Create a copy of clients to avoid modification during iteration
            clients = self.websocket_clients.copy()
            for client in clients:
                try:
                    # In a real implementation, this would be async
                    # For now, we'll just log that we'd send it
                    pass
                except Exception as e:
                    self.get_logger().error(f"WebSocket send error: {e}")
                    self.websocket_clients.discard(client)

        # Log to file
        if self.log_file_handle:
            try:
                self.log_file_handle.write(telemetry_json + "\n")
                self.log_file_handle.flush()
            except Exception as e:
                self.get_logger().error(f"Telemetry logging error: {e}")

        # Publish summary for ROS monitoring
        summary = {
            "mission": self.telemetry_data["current_mission"],
            "status": self.telemetry_data["mission_status"],
            "battery": self.telemetry_data["battery_level"],
            "samples": self.telemetry_data["samples_collected"],
            "emergency_stop": self.telemetry_data["emergency_stop"],
            "boundary_violation": self.telemetry_data["boundary_violation"],
            "timestamp": self.telemetry_data["timestamp"],
        }

        summary_msg = String()
        summary_msg.data = json.dumps(summary)

        # Publish to ROS topic for internal monitoring
        # (Would need another publisher for this)

    def get_competition_status(self) -> Dict[str, Any]:
        """Get current competition status summary."""
        return {
            "mission_time": self.telemetry_data["mission_time"],
            "current_mission": self.telemetry_data["current_mission"],
            "mission_status": self.telemetry_data["mission_status"],
            "samples_collected": self.telemetry_data["samples_collected"],
            "system_health": self.telemetry_data["system_health"],
            "emergency_stop": self.telemetry_data["emergency_stop"],
            "boundary_violation": self.telemetry_data["boundary_violation"],
            "websocket_clients": len(self.websocket_clients),
            "adaptive_telemetry": {
                "enabled": self.adaptive_enabled,
                "current_rate": self.adaptive_data["current_rate"],
                "target_rate": self.adaptive_data["target_rate"],
                "network_quality": self.adaptive_data["network_quality_score"],
                "current_band": self.urc_band_config["current_band"],
            },
            "mission_orchestrator": {
                "autonomous_navigation": {
                    "active": self.mission_orchestrator["autonomous_navigation"][
                        "active"
                    ],
                    "current_target_index": self.mission_orchestrator[
                        "autonomous_navigation"
                    ]["current_target_index"],
                    "current_mode": self.mission_orchestrator["autonomous_navigation"][
                        "current_mode"
                    ],
                    "targets_completed": len(
                        self.mission_orchestrator["autonomous_navigation"][
                            "completed_targets"
                        ]
                    ),
                },
                "equipment_servicing": {
                    "active": self.mission_orchestrator["equipment_servicing"][
                        "active"
                    ],
                    "tasks_completed": sum(
                        self.mission_orchestrator["equipment_servicing"][
                            "tasks_completed"
                        ].values()
                    ),
                    "total_tasks": len(
                        self.mission_orchestrator["equipment_servicing"][
                            "tasks_completed"
                        ]
                    ),
                },
            },
            "gnss_compliance": {
                "wgs84_verified": self.gnss_compliance["wgs84_verified"],
                "coordinate_format": self.gnss_compliance["coordinate_format"],
                "last_coordinates": self.gnss_compliance["last_coordinates"],
                "compliance_entries": len(self.gnss_compliance["compliance_log"]),
            },
            "spectrum_compliance": {
                "fcc_compliant": self.spectrum_compliance["fcc_compliant"],
                "current_band": self.spectrum_compliance["current_band"],
                "bandwidth_usage_mhz": self.spectrum_compliance["bandwidth_usage_mhz"],
                "interference_detected": self.spectrum_compliance[
                    "interference_detected"
                ],
                "violations": len(self.spectrum_compliance["compliance_violations"]),
            },
        }

    def _on_state_change(self, key: str, entry):
        """Handle state synchronization updates."""
        try:
            # Update local state based on synchronized changes
            if key == "telemetry_rate_hz":
                self.telemetry_rate = entry.value
                self.get_logger().info(
                    f"Telemetry rate updated via state sync: {entry.value}Hz"
                )
            elif key == "system_mode":
                self.get_logger().info(
                    f"System mode changed via state sync: {entry.value}"
                )
        except Exception as e:
            self.get_logger().error(f"State sync callback error: {e}")

    def _on_dds_failover(self, old_domain: int, new_domain: int):
        """Handle DDS domain failover events."""
        self.get_logger().warning(f"DDS domain failover: {old_domain} → {new_domain}")

        # Update our domain tracking
        self.primary_domain_id = new_domain

        # The DDS manager will handle restarting nodes in the new domain
        # We just need to be aware of the change

    def _on_config_update(self, snapshot):
        """Handle dynamic configuration updates."""
        try:
            for change in snapshot.changes:
                if change.node_name == self.get_name():
                    if change.parameter_name == "telemetry_rate_hz":
                        self.telemetry_rate = change.new_value
                        # Update the timer
                        if hasattr(self, "telemetry_timer"):
                            self.telemetry_timer.cancel()
                        self.create_timer(
                            1.0 / self.telemetry_rate, self.publish_telemetry
                        )
                        self.get_logger().info(
                            f"Telemetry rate updated via dynamic config: {change.new_value}Hz"
                        )

                    elif change.parameter_name == "max_clients":
                        self.max_clients = change.new_value
                        self.get_logger().info(
                            f"Max clients updated via dynamic config: {change.new_value}"
                        )

        except Exception as e:
            self.get_logger().error(f"Config update callback error: {e}")

    def _on_recovery_progress(self, phase, message):
        """Handle recovery progress updates."""
        self.get_logger().info(f"[UPDATE] Recovery progress: {phase.value} - {message}")

    def _on_recovery_complete(self, success, error_message):
        """Handle recovery completion."""
        if success:
            self.get_logger().info("[SUCCESS] Coordinated recovery completed successfully")
        else:
            self.get_logger().error(f"[ERROR] Coordinated recovery failed: {error_message}")

            # Trigger emergency protocols if recovery fails
            self._handle_emergency_recovery_failure()

    def _handle_emergency_recovery_failure(self):
        """Handle emergency situation when coordinated recovery fails."""
        self.get_logger().critical(
            "[ALERT] EMERGENCY: Coordinated recovery failed - entering emergency mode"
        )

        try:
            # Emergency actions:
            # 1. Force DDS domain to emergency (domain 44)
            if self.dds_redundancy_enabled:
                self.get_logger().warning("Forcing DDS to emergency domain")
                self.dds_manager.trigger_domain_failover(target_domain_id=44)

            # 2. Set system to emergency state
            if self.state_sync_enabled:
                self.get_logger().warning("Setting system state to emergency")
                self.state_manager.update_state("system_mode", "emergency")
                self.state_manager.update_state("emergency_stop", True)

            # 3. Enable minimal WebSocket connectivity
            if self.redundancy_enabled:
                self.get_logger().warning("Ensuring minimal WebSocket connectivity")
                # This would ensure at least one endpoint is available

            self.get_logger().critical(
                "System in emergency mode - manual intervention required"
            )

        except Exception as e:
            self.get_logger().critical(f"Emergency protocol failed: {e}")

    def update_adaptive_telemetry(
        self,
        bandwidth_mbps: float,
        latency_ms: float,
        packet_loss: float,
        signal_strength: float = 0.0,
    ):
        """
        Update adaptive telemetry with current network conditions.

        Args:
            bandwidth_mbps: Current bandwidth usage in Mbps
            latency_ms: Current latency in milliseconds
            packet_loss: Current packet loss rate (0.0-1.0)
            signal_strength: Signal strength indicator (0.0-1.0)
        """
        if not self.adaptive_enabled:
            return

        current_time = time.time()

        # Update measurement history
        self.adaptive_data["bandwidth_history"].append(bandwidth_mbps)
        self.adaptive_data["latency_history"].append(latency_ms)
        self.adaptive_data["packet_loss_history"].append(packet_loss)

        # Calculate network quality score (0.0 = poor, 1.0 = excellent)
        quality_score = self._calculate_network_quality(
            bandwidth_mbps, latency_ms, packet_loss, signal_strength
        )
        self.adaptive_data["network_quality_score"] = quality_score

        # Check if enough time has passed for adaptation
        if (
            current_time - self.adaptive_data["last_adaptation"]
            < self.adaptive_data["adaptation_cooldown"]
        ):
            return

        # Calculate new target telemetry rate
        new_target_rate = self._calculate_optimal_telemetry_rate(
            bandwidth_mbps, latency_ms, packet_loss, quality_score
        )

        # Apply rate limiting and bounds
        new_target_rate = max(
            self.min_telemetry_rate, min(self.max_telemetry_rate, new_target_rate)
        )

        # Smooth the adaptation
        adaptation_factor = self.adaptation_rate
        smoothed_rate = (
            self.adaptive_data["target_rate"] * (1 - adaptation_factor)
            + new_target_rate * adaptation_factor
        )

        # Only adapt if change is significant (>10% difference)
        rate_change_ratio = (
            abs(smoothed_rate - self.adaptive_data["current_rate"])
            / self.adaptive_data["current_rate"]
        )
        if rate_change_ratio > 0.1:
            self._adapt_telemetry_rate(smoothed_rate)
            self.adaptive_data["last_adaptation"] = current_time

    def _calculate_network_quality(
        self,
        bandwidth: float,
        latency: float,
        packet_loss: float,
        signal_strength: float,
    ) -> float:
        """Calculate overall network quality score (0.0-1.0)."""
        # Bandwidth quality (relative to URC limits)
        max_bandwidth = self._get_current_band_limit()
        bandwidth_score = min(1.0, bandwidth / max_bandwidth) if max_bandwidth else 0.8

        # Latency quality (inverse relationship)
        latency_score = max(0.0, 1.0 - (latency / self.latency_target))

        # Packet loss quality (inverse relationship)
        packet_loss_score = max(0.0, 1.0 - packet_loss * 10)  # 10% loss = 0 score

        # Signal strength quality
        signal_score = signal_strength

        # Weighted average (bandwidth most important for telemetry)
        quality_score = (
            bandwidth_score * 0.4
            + latency_score * 0.3
            + packet_loss_score * 0.2
            + signal_score * 0.1
        )

        return max(0.0, min(1.0, quality_score))

    def _calculate_optimal_telemetry_rate(
        self, bandwidth: float, latency: float, packet_loss: float, quality_score: float
    ) -> float:
        """Calculate optimal telemetry rate based on network conditions."""
        base_rate = self.telemetry_rate

        # Bandwidth-based adjustment
        max_bandwidth = self._get_current_band_limit()
        if max_bandwidth:
            bandwidth_ratio = bandwidth / max_bandwidth
            if bandwidth_ratio > self.bandwidth_target:
                # Reduce rate if over-utilizing bandwidth
                bandwidth_factor = self.bandwidth_target / bandwidth_ratio
                base_rate *= bandwidth_factor

        # Latency-based adjustment
        if latency > self.latency_target:
            latency_factor = self.latency_target / latency
            base_rate *= latency_factor

        # Packet loss adjustment
        if packet_loss > 0.05:  # More than 5% loss
            loss_factor = max(0.5, 1.0 - packet_loss * 5)  # Reduce rate with high loss
            base_rate *= loss_factor

        # Quality score adjustment
        quality_factor = 0.5 + (quality_score * 0.5)  # 0.5-1.0 range
        base_rate *= quality_factor

        return base_rate

    def _adapt_telemetry_rate(self, new_rate: float):
        """Apply new telemetry rate adaptation."""
        old_rate = self.adaptive_data["current_rate"]

        self.get_logger().info(
            f"Adapting telemetry rate: {old_rate:.1f}Hz -> {new_rate:.1f}Hz "
            f"(quality: {self.adaptive_data['network_quality_score']:.2f})"
        )

        self.adaptive_data["target_rate"] = new_rate
        self.adaptive_data["current_rate"] = new_rate

        # Update the actual telemetry timer rate
        # This would require modifying the timer callback rate
        # For now, we'll update the rate parameter
        self.telemetry_rate = new_rate

        # Log adaptation for analysis
        if self.log_file_handle:
            adaptation_record = {
                "timestamp": time.time(),
                "event": "telemetry_rate_adaptation",
                "old_rate": old_rate,
                "new_rate": new_rate,
                "network_quality": self.adaptive_data["network_quality_score"],
                "current_band": self.urc_band_config["current_band"],
            }
            self.log_file_handle.write(json.dumps(adaptation_record) + "\n")
            self.log_file_handle.flush()

    def _get_current_band_limit(self) -> Optional[float]:
        """Get current band bandwidth limit in Mbps."""
        current_band = self.urc_band_config["current_band"]

        if current_band == "900mhz":
            return self.urc_band_config["900mhz"]["max_bandwidth_mhz"]
        elif current_band == "2.4ghz":
            return None  # No limit, but interference-prone
        else:
            return 10.0  # Conservative default

    def set_urc_band(self, band: str, subband: Optional[str] = None):
        """
        Set current URC band configuration.

        Args:
            band: '900mhz' or '2.4ghz'
            subband: For 900MHz, 'low', 'mid', or 'high'
        """
        if band not in ["900mhz", "2.4ghz"]:
            self.get_logger().error(f"Invalid band: {band}")
            return

        self.urc_band_config["current_band"] = band

        if band == "900mhz" and subband:
            if subband in self.urc_band_config["900mhz"]["sub_bands"]:
                # Deactivate all subbands
                for sb in self.urc_band_config["900mhz"]["sub_bands"].values():
                    sb["active"] = False

                # Activate selected subband
                self.urc_band_config["900mhz"]["sub_bands"][subband]["active"] = True
                self.urc_band_config["900mhz"]["current_subband"] = subband

                self.get_logger().info(
                    f"Switched to URC 900MHz {subband} subband ({self.urc_band_config['900mhz']['sub_bands'][subband]['range']} MHz)"
                )
            else:
                self.get_logger().error(f"Invalid 900MHz subband: {subband}")

        elif band == "2.4ghz":
            self.get_logger().info(
                "Switched to 2.4GHz band (interference monitoring active)"
            )

        # Trigger immediate telemetry adaptation for new band constraints
        if self.adaptive_enabled:
            self.adaptive_data["last_adaptation"] = 0  # Force immediate adaptation

    def get_adaptive_telemetry_status(self) -> Dict[str, Any]:
        """Get current adaptive telemetry status."""
        return {
            "enabled": self.adaptive_enabled,
            "current_rate": self.adaptive_data["current_rate"],
            "target_rate": self.adaptive_data["target_rate"],
            "network_quality": self.adaptive_data["network_quality_score"],
            "current_band": self.urc_band_config["current_band"],
            "band_limits": {
                "900mhz_max_mhz": self.urc_band_config["900mhz"]["max_bandwidth_mhz"],
                "current_limit_mhz": self._get_current_band_limit(),
            },
            "history": {
                "bandwidth_samples": len(self.adaptive_data["bandwidth_history"]),
                "latency_samples": len(self.adaptive_data["latency_history"]),
                "packet_loss_samples": len(self.adaptive_data["packet_loss_history"]),
            },
        }

    def destroy_node(self):
        """Clean shutdown."""
        if self.log_file_handle:
            self.log_file_handle.close()

        # Close WebSocket connections
        if self.websocket_clients:
            for client in self.websocket_clients.copy():
                try:
                    # In async context, this would close properly
                    pass
                except:
                    pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    bridge = CompetitionBridge()

    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
