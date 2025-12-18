#!/usr/bin/env python3
"""
Mission Orchestrator - URC Competition Mission Coordination

Handles coordination and state management for URC competition missions
including autonomous navigation and equipment servicing tasks.
"""

import time
from typing import Any, Dict, List, Optional

import rclpy
from autonomy_interfaces.msg import LedCommand, VisionDetection
from constants import DEFAULT_WEBSOCKET_PORT
from rclpy.node import Node
from rclpy.publisher import Publisher


class MissionOrchestrator:
    """
    Orchestrates URC competition missions and tasks.

    Manages mission state, coordinates between different mission types,
    and provides LED status indicators per URC requirements.
    """

    def __init__(self, node: Node, logger):
        """
        Initialize the Mission Orchestrator.

        Args:
            node: ROS2 node instance for publishing
            logger: Logger instance for mission operations
        """
        self.node = node
        self.logger = logger

        # Mission state tracking
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

        # LED status publisher
        self.led_pub: Optional[Publisher] = None
        self.object_highlight_pub: Optional[Publisher] = None

        self._initialize_publishers()

    def _initialize_publishers(self) -> None:
        """Initialize ROS2 publishers for mission coordination."""
        try:
            self.led_pub = self.node.create_publisher(LedCommand, "/led/command", 10)
            self.object_highlight_pub = self.node.create_publisher(
                VisionDetection, "/c2/object_highlight", 10
            )
            self.logger.info("Mission orchestrator publishers initialized")
        except Exception as e:
            self.logger.error(f"Failed to initialize mission publishers: {e}")

    def start_autonomous_navigation(
        self, targets: Dict[str, List[Dict[str, Any]]]
    ) -> None:
        """
        Start autonomous navigation mission (URC requirement).

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
        self.logger.info("Autonomous navigation mission started")

    def target_reached(self, target_type: str, target_index: int) -> None:
        """
        Mark target as reached and move to next (URC requirement).

        Args:
            target_type: Type of target reached ('gnss_locations', 'ar_tags', 'objects')
            target_index: Index of target in mission list
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

        self.logger.info(f"Target reached: {target_type}[{target_index}]")

    def abort_to_previous_target(self) -> Optional[Dict[str, Any]]:
        """
        Abort current target and return to previous target.

        Returns:
            Previous target info or None if no previous targets
        """
        completed_targets = self.mission_orchestrator["autonomous_navigation"][
            "completed_targets"
        ]

        if not completed_targets:
            self.logger.warning("No previous targets to abort to")
            return None

        # Get the last completed target
        previous_target = completed_targets.pop()

        # Switch to teleoperation mode
        self.mission_orchestrator["autonomous_navigation"][
            "current_mode"
        ] = "teleoperation"
        self.update_led_status("autonomous_navigation", "teleoperation")

        self.logger.info(f"Aborted to previous target: {previous_target}")
        return previous_target

    def highlight_object(
        self, object_name: str, confidence: float, bounding_box: List[float]
    ) -> None:
        """
        Highlight detected object on C2 display (URC autonomous navigation requirement).

        Args:
            object_name: Name of the detected object
            confidence: Detection confidence (0.0-1.0)
            bounding_box: Object bounding box coordinates
        """
        if not self.object_highlight_pub:
            self.logger.warning("Object highlight publisher not available")
            return

        # Create vision detection message
        detection = VisionDetection()
        detection.header.stamp = self.node.get_clock().now().to_msg()
        detection.object_name = object_name
        detection.confidence = confidence
        detection.bounding_box = bounding_box

        self.object_highlight_pub.publish(detection)
        self.logger.info(f"Object highlighted: {object_name} (conf: {confidence:.2f})")

    def complete_equipment_task(self, task_name: str) -> None:
        """
        Mark an equipment servicing task as completed.

        Args:
            task_name: Name of the completed task
        """
        if (
            task_name
            in self.mission_orchestrator["equipment_servicing"]["tasks_completed"]
        ):
            self.mission_orchestrator["equipment_servicing"]["tasks_completed"][
                task_name
            ] = True
            self.update_led_status("equipment_servicing", "task_completed")
            self.logger.info(f"Equipment task completed: {task_name}")
        else:
            self.logger.warning(f"Unknown equipment task: {task_name}")

    def set_launch_key(self, launch_key: str) -> None:
        """
        Set launch key for autonomous typing task.

        Args:
            launch_key: Launch key string for typing task
        """
        self.mission_orchestrator["equipment_servicing"]["launch_key"] = launch_key
        self.logger.info(f"Launch key set for autonomous typing: {launch_key}")

    def update_led_status(self, mission_type: str, status: str) -> None:
        """
        Update LED indicators per URC requirements.

        Args:
            mission_type: 'autonomous_navigation' or 'equipment_servicing'
            status: Status code for LED pattern
        """
        if not self.led_pub:
            self.logger.warning("LED publisher not available")
            return

        led_cmd = LedCommand()
        led_cmd.header.stamp = self.node.get_clock().now().to_msg()

        if mission_type == "autonomous_navigation":
            if status == "autonomous_mode":
                # Red: Autonomous operation
                led_cmd.status_code = 1
            elif status == "target_reached":
                # Flashing green: Target reached
                led_cmd.status_code = 2
            elif status == "teleoperation":
                # Blue: Teleoperation mode
                led_cmd.status_code = 3
            else:
                led_cmd.status_code = 0  # Default/off

        elif mission_type == "equipment_servicing":
            if status == "task_completed":
                # Yellow: Task completed
                led_cmd.status_code = 4
            elif status == "error":
                # Flashing red: Error condition
                led_cmd.status_code = 5
            else:
                led_cmd.status_code = 0  # Default/off

        self.led_pub.publish(led_cmd)

    def verify_gnss_compliance(self, navsat_msg) -> bool:
        """
        Verify GNSS data compliance with WGS84 standards.

        Args:
            navsat_msg: NavSatFix message to verify

        Returns:
            True if compliant, False otherwise
        """
        # Check latitude range
        if not (-90.0 <= navsat_msg.latitude <= 90.0):
            self.logger.error("Invalid latitude range for WGS84")
            self.gnss_compliance["wgs84_verified"] = False
            return False

        # Check longitude range
        if not (-180.0 <= navsat_msg.longitude <= 180.0):
            self.logger.error("Invalid longitude range for WGS84")
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

    def get_mission_status(self) -> Dict[str, Any]:
        """Get current mission status summary."""
        return {
            "autonomous_navigation": {
                "active": self.mission_orchestrator["autonomous_navigation"]["active"],
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
                "active": self.mission_orchestrator["equipment_servicing"]["active"],
                "tasks_completed": sum(
                    self.mission_orchestrator["equipment_servicing"][
                        "tasks_completed"
                    ].values()
                ),
                "total_tasks": len(
                    self.mission_orchestrator["equipment_servicing"]["tasks_completed"]
                ),
                "current_task": self.mission_orchestrator["equipment_servicing"][
                    "current_task"
                ],
            },
            "gnss_compliance": {
                "wgs84_verified": self.gnss_compliance["wgs84_verified"],
                "coordinate_format": self.gnss_compliance["coordinate_format"],
                "last_coordinates": self.gnss_compliance["last_coordinates"],
                "compliance_entries": len(self.gnss_compliance["compliance_log"]),
            },
        }

    def get_competition_status(self) -> Dict[str, Any]:
        """Get competition compliance status for judges."""
        return {
            "gnss_compliance": self.gnss_compliance,
            "mission_orchestrator": self.get_mission_status(),
        }
