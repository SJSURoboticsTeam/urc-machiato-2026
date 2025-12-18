#!/usr/bin/env python3
"""
Direct CAN Safety Interface - Hardware-Level Emergency Control

Bypasses ROS2 middleware for critical safety operations.
"""

import serial
import time
import threading
from typing import Optional, Dict, Any
from enum import Enum
import logging


class SafetyCommand(Enum):
    """Safety command types."""
    EMERGENCY_STOP = "EMERGENCY_STOP"
    BOUNDARY_VIOLATION_STOP = "BOUNDARY_VIOLATION_STOP"
    MOTOR_BRAKE = "MOTOR_BRAKE"


class DirectCANSafety:
    """Direct CAN Safety Interface for immediate hardware response."""

    def __init__(self, can_port: str = '/dev/ttyACM0'):
        self.can_port = can_port
        self.can_serial = None
        self.connected = False
        self.command_lock = threading.Lock()
        self.logger = logging.getLogger('direct_can_safety')
        self.logger.setLevel(logging.INFO)
        self.connect()

    def connect(self) -> bool:
        """Connect to CAN serial interface."""
        try:
            import sys
            import os
            sys.path.append(os.path.join(
                os.path.dirname(__file__), '..', '..', '..', 
                'vendor', 'teleoperation', 'server'))
            from can_serial import CanSerial  # type: ignore

            self.can_serial = CanSerial(self.can_port)
            self.connected = True
            self.logger.info(f"Direct CAN safety connected on {self.can_port}")
            return True
        except Exception as e:
            self.connected = False
            self.logger.error(f"Failed to connect direct CAN safety: {e}")
            return False

    def is_connected(self) -> bool:
        """Check if CAN is connected."""
        return self.connected

    def emergency_stop(self, reason: str = "UNKNOWN") -> bool:
        """Execute emergency stop via direct CAN."""
        return self.send_safety_command(SafetyCommand.EMERGENCY_STOP, reason)

    def boundary_violation_stop(self) -> bool:
        """Stop due to boundary violation."""
        return self.send_safety_command(SafetyCommand.BOUNDARY_VIOLATION_STOP, "BOUNDARY_VIOLATION")

    def activate_motor_brake(self) -> bool:
        """Activate motor brakes."""
        return self.send_safety_command(SafetyCommand.MOTOR_BRAKE, "MOTOR_BRAKE")

    def send_safety_command(self, command: SafetyCommand, reason: str = "") -> bool:
        """Send safety command with highest priority."""
        if not self.connected or not self.can_serial:
            return False

        with self.command_lock:
            try:
                can_message = f"PRIORITY:{command.value}:{reason}\r"
                self.can_serial.write(can_message.encode())
                self.can_serial.flush()
                return True
            except Exception as e:
                self.logger.error(f"Failed to send safety command: {e}")
                return False
