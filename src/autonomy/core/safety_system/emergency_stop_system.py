#!/usr/bin/env python3
"""
Emergency Stop System - Competition Ready
Simple, hierarchical emergency stop system with clear escalation levels.
"""

import threading
import time
from enum import Enum
from typing import Callable, Dict, Optional

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from std_srvs.srv import SetBool, Trigger


class EmergencyStopLevel(Enum):
    """Emergency stop hierarchy - from least to most severe."""

    SOFT_STOP = "soft_stop"  # Immediate halt, maintain power
    HARD_STOP = "hard_stop"  # Immediate halt, cut actuator power
    EMERGENCY_SHUTDOWN = "shutdown"  # Full system shutdown


class EmergencyStopSystem(Node):
    """
    Simple, reliable emergency stop system for competition use.

    Features:
    - Hierarchical stop levels (soft -> hard -> shutdown)
    - Automatic escalation on repeated triggers
    - Recovery procedures for each level
    - ROS2 integration with health monitoring
    - Manual override capability
    """

    def __init__(self):
        super().__init__("emergency_stop_system")

        # Current state
        self.current_level = None
        self.last_trigger_time = 0
        self.trigger_count = 0
        self.manual_override = False

        # Configuration
        self.escalation_threshold = 3  # Triggers within window to escalate
        self.escalation_window = 10.0  # Seconds
        self.recovery_timeout = 30.0  # Recovery attempt timeout

        # Publishers
        self.emergency_pub = self.create_publisher(String, "/emergency/status", 10)
        self.system_state_pub = self.create_publisher(String, "/system/state", 10)

        # Services
        self.manual_stop_srv = self.create_service(
            SetBool, "/emergency/manual_stop", self._handle_manual_stop
        )
        self.reset_srv = self.create_service(
            Trigger, "/emergency/reset", self._handle_reset
        )
        self.health_srv = self.create_service(
            Trigger, "/emergency/health_check", self._handle_health_check
        )

        # Subscribers for emergency triggers
        self.collision_sub = self.create_subscription(
            Bool, "/safety/collision_detected", self._handle_collision, 10
        )
        self.motor_sub = self.create_subscription(
            String, "/motor/fault", self._handle_motor_fault, 10
        )
        self.comm_sub = self.create_subscription(
            Bool, "/communication/lost", self._handle_comm_loss, 10
        )

        # Recovery monitoring
        self.recovery_timer = None
        self.recovery_attempts = 0

        # Stop procedures - simple function mapping
        self.stop_procedures = {
            EmergencyStopLevel.SOFT_STOP: self._execute_soft_stop,
            EmergencyStopLevel.HARD_STOP: self._execute_hard_stop,
            EmergencyStopLevel.EMERGENCY_SHUTDOWN: self._execute_emergency_shutdown,
        }

        # Recovery procedures
        self.recovery_procedures = {
            EmergencyStopLevel.SOFT_STOP: self._recover_soft_stop,
            EmergencyStopLevel.HARD_STOP: self._recover_hard_stop,
            EmergencyStopLevel.EMERGENCY_SHUTDOWN: self._recover_emergency_shutdown,
        }

        self.get_logger().info("Emergency Stop System initialized")
        self._publish_status()

    def trigger_emergency_stop(self, level: EmergencyStopLevel, reason: str):
        """Trigger emergency stop at specified level."""
        if self.manual_override:
            self.get_logger().warn(
                f"Emergency stop blocked by manual override: {reason}"
            )
            return

        # Check for escalation
        current_time = time.time()
        if (current_time - self.last_trigger_time) < self.escalation_window:
            self.trigger_count += 1
            if self.trigger_count >= self.escalation_threshold:
                level = EmergencyStopLevel.EMERGENCY_SHUTDOWN
                self.get_logger().error(
                    f"ESCALATED to EMERGENCY SHUTDOWN due to repeated triggers"
                )
        else:
            self.trigger_count = 1

        self.last_trigger_time = current_time
        self.current_level = level

        self.get_logger().error(f"EMERGENCY STOP TRIGGERED: {level.value} - {reason}")
        self._execute_stop_procedure(level, reason)

    def _execute_stop_procedure(self, level: EmergencyStopLevel, reason: str):
        """Execute the appropriate stop procedure."""
        if level in self.stop_procedures:
            try:
                self.stop_procedures[level](reason)
                self._publish_status()
            except Exception as e:
                self.get_logger().error(
                    f"Failed to execute {level.value} procedure: {e}"
                )
                # Escalate to shutdown on procedure failure
                if level != EmergencyStopLevel.EMERGENCY_SHUTDOWN:
                    self.trigger_emergency_stop(
                        EmergencyStopLevel.EMERGENCY_SHUTDOWN,
                        f"Stop procedure failure: {e}",
                    )

    def _execute_soft_stop(self, reason: str):
        """Soft stop: Halt motion but maintain system power."""
        self.get_logger().warn("Executing SOFT STOP")

        # Publish stop command to motion control
        stop_msg = String()
        stop_msg.data = f"SOFT_STOP:{reason}"
        self.system_state_pub.publish(stop_msg)

        # Allow recovery after short delay
        self._schedule_recovery(EmergencyStopLevel.SOFT_STOP, 5.0)

    def _execute_hard_stop(self, reason: str):
        """Hard stop: Halt motion and cut actuator power."""
        self.get_logger().error("Executing HARD STOP")

        # Publish hard stop command
        stop_msg = String()
        stop_msg.data = f"HARD_STOP:{reason}"
        self.system_state_pub.publish(stop_msg)

        # Require manual recovery
        self._schedule_recovery(EmergencyStopLevel.HARD_STOP, self.recovery_timeout)

    def _execute_emergency_shutdown(self, reason: str):
        """Emergency shutdown: Full system power down."""
        self.get_logger().fatal("Executing EMERGENCY SHUTDOWN")

        # Publish shutdown command
        stop_msg = String()
        stop_msg.data = "EMERGENCY_SHUTDOWN"
        self.system_state_pub.publish(stop_msg)

        # Shutdown requires manual intervention
        self.get_logger().fatal(
            "System requires manual restart after emergency shutdown"
        )

    def _schedule_recovery(self, level: EmergencyStopLevel, delay: float):
        """Schedule recovery attempt after delay."""
        if self.recovery_timer:
            self.recovery_timer.cancel()

        self.recovery_timer = threading.Timer(
            delay, self._attempt_recovery, args=[level]
        )
        self.recovery_timer.start()

    def _attempt_recovery(self, level: EmergencyStopLevel):
        """Attempt to recover from emergency stop."""
        if level in self.recovery_procedures:
            try:
                self.get_logger().info(f"Attempting recovery from {level.value}")
                success = self.recovery_procedures[level]()
                if success:
                    self.current_level = None
                    self._publish_status()
                    self.get_logger().info(f"Successfully recovered from {level.value}")
                else:
                    self.get_logger().warn(
                        f"Recovery from {level.value} failed, manual intervention required"
                    )
            except Exception as e:
                self.get_logger().error(f"Recovery procedure failed: {e}")

    def _recover_soft_stop(self) -> bool:
        """Recover from soft stop - check if safe to resume."""
        # Simple recovery: just clear the stop
        resume_msg = String()
        resume_msg.data = "RESUME_NORMAL_OPERATION"
        self.system_state_pub.publish(resume_msg)
        return True

    def _recover_hard_stop(self) -> bool:
        """Recover from hard stop - requires system checks."""
        # In competition, this might require operator confirmation
        # For now, just log that manual recovery is needed
        self.get_logger().warn("Hard stop recovery requires manual confirmation")
        return False  # Require manual intervention

    def _recover_emergency_shutdown(self) -> bool:
        """Recovery from emergency shutdown - not automatic."""
        self.get_logger().fatal("Emergency shutdown requires complete system restart")
        return False

    def _handle_collision(self, msg):
        """Handle collision detection."""
        if msg.data:
            self.trigger_emergency_stop(
                EmergencyStopLevel.SOFT_STOP, "Collision detected"
            )

    def _handle_motor_fault(self, msg):
        """Handle motor fault reports."""
        if "CRITICAL" in msg.data.upper():
            self.trigger_emergency_stop(
                EmergencyStopLevel.HARD_STOP, f"Motor fault: {msg.data}"
            )
        else:
            self.trigger_emergency_stop(
                EmergencyStopLevel.SOFT_STOP, f"Motor warning: {msg.data}"
            )

    def _handle_comm_loss(self, msg):
        """Handle communication loss."""
        if msg.data:  # True = communication lost
            self.trigger_emergency_stop(
                EmergencyStopLevel.SOFT_STOP, "Communication lost"
            )

    def _handle_manual_stop(self, request, response):
        """Handle manual emergency stop requests."""
        if request.data:
            self.manual_override = True
            self.trigger_emergency_stop(
                EmergencyStopLevel.HARD_STOP, "Manual emergency stop"
            )
            response.success = True
            response.message = "Manual emergency stop activated"
        else:
            self.manual_override = False
            response.success = True
            response.message = "Manual override cleared"

        return response

    def _handle_reset(self, request, response):
        """Handle emergency system reset."""
        if self.current_level == EmergencyStopLevel.EMERGENCY_SHUTDOWN:
            response.success = False
            response.message = (
                "Cannot reset from emergency shutdown - manual restart required"
            )
        else:
            self.current_level = None
            self.trigger_count = 0
            self.manual_override = False
            if self.recovery_timer:
                self.recovery_timer.cancel()
                self.recovery_timer = None
            self._publish_status()
            response.success = True
            response.message = "Emergency system reset"

        return response

    def _handle_health_check(self, request, response):
        """Health check for emergency system."""
        response.success = True
        response.message = f"Emergency system healthy - Level: {self.current_level.value if self.current_level else 'NORMAL'}"
        return response

    def _publish_status(self):
        """Publish current emergency status."""
        status_data = {
            "level": self.current_level.value if self.current_level else "normal",
            "manual_override": self.manual_override,
            "trigger_count": self.trigger_count,
            "last_trigger": self.last_trigger_time,
            "timestamp": time.time(),
        }

        import json

        status_msg = String()
        status_msg.data = json.dumps(status_data)  # Proper JSON format
        self.emergency_pub.publish(status_msg)


def main():
    rclpy.init()
    emergency_system = EmergencyStopSystem()

    try:
        rclpy.spin(emergency_system)
    except KeyboardInterrupt:
        pass
    finally:
        emergency_system.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
