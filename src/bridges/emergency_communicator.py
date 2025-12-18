#!/usr/bin/env python3
"""
Emergency Communicator - URC Competition Emergency Communication

Handles emergency communication protocols, degraded communication modes,
and critical system state broadcasting during communication failures.
"""

import time
from typing import Any, Dict, List, Optional

import rclpy
from constants import DEFAULT_TELEMETRY_RATE_HZ, WEBSOCKET_PING_INTERVAL
from rclpy.node import Node
from rclpy.publisher import Publisher
from std_msgs.msg import Bool


class EmergencyCommunicator:
    """
    Manages emergency communication protocols and degraded communication modes.

    Handles:
    - Emergency heartbeat transmission
    - Degraded communication mode activation
    - Critical system state broadcasting
    - Communication load reduction strategies
    """

    def __init__(self, node: Node, logger, telemetry_data: Dict[str, Any]):
        """
        Initialize the Emergency Communicator.

        Args:
            node: ROS2 node instance for publishing
            logger: Logger instance for emergency operations
            telemetry_data: Reference to telemetry data dictionary
        """
        self.node = node
        self.logger = logger
        self.telemetry_data = telemetry_data

        # Emergency communication state
        self.emergency_communication = {
            "active": False,
            "message_queue": [],
            "low_bandwidth_mode": False,
            "heartbeat_interval": WEBSOCKET_PING_INTERVAL,  # Send heartbeat every 30 seconds
            "last_heartbeat": 0,
            "reduced_telemetry_rate": 1.0,  # 1 Hz in emergency mode
        }

        # Communication redundancy state
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

        # Channel safety management
        self.channel_safety_manager = {
            "available_channels": [1, 6, 11],  # Non-overlapping WiFi channels
            "current_channel": 6,
            "channel_quality_history": {},  # Track channel reliability
            "last_channel_switch": 0,
            "channel_switch_cooldown": 30.0,  # 30 seconds between switches
            "validation_timeout": 5.0,  # 5 seconds to validate new channel
        }

        # Publishers for emergency signals
        self.emergency_stop_pub: Optional[Publisher] = None
        self._initialize_publishers()

        # Initialize channel quality history
        for channel in self.channel_safety_manager["available_channels"]:
            self.channel_safety_manager["channel_quality_history"][channel] = []

    def _initialize_publishers(self) -> None:
        """Initialize ROS2 publishers for emergency signals."""
        try:
            self.emergency_stop_pub = self.node.create_publisher(
                Bool, "/emergency_stop", 10
            )
            self.logger.info("Emergency communicator publishers initialized")
        except Exception as e:
            self.logger.error(f"Failed to initialize emergency publishers: {e}")

    def activate_emergency_mode(self) -> None:
        """
        Activate emergency communication mode for degraded communication links.

        This reduces communication load and switches to minimal protocols.
        """
        if self.emergency_communication["active"]:
            return  # Already in emergency mode

        self.logger.warning("Activating 2.4 GHz emergency communication mode")

        self.emergency_communication["active"] = True
        self.emergency_communication["low_bandwidth_mode"] = True

        # Reduce telemetry rate
        original_rate = getattr(self, "telemetry_rate", DEFAULT_TELEMETRY_RATE_HZ)
        self.emergency_communication["original_telemetry_rate"] = original_rate
        self._reduce_telemetry_rate()

        # Start emergency heartbeat
        self._start_emergency_heartbeat()

        # Notify all systems of degraded communication
        self._broadcast_communication_degradation()

        self.logger.warning("Emergency communication mode activated")

    def deactivate_emergency_mode(self) -> None:
        """
        Deactivate emergency communication mode and restore normal operation.
        """
        if not self.emergency_communication["active"]:
            return

        self.logger.info("Deactivating emergency communication mode")

        self.emergency_communication["active"] = False
        self.emergency_communication["low_bandwidth_mode"] = False

        # Restore normal telemetry rate
        if "original_telemetry_rate" in self.emergency_communication:
            original_rate = self.emergency_communication["original_telemetry_rate"]
            setattr(self, "telemetry_rate", original_rate)
            self.logger.info(f"Restored normal telemetry rate: {original_rate}Hz")

        self.logger.info("Emergency communication mode deactivated")

    def _reduce_telemetry_rate(self) -> None:
        """Reduce telemetry rate for emergency communication."""
        reduced_rate = self.emergency_communication["reduced_telemetry_rate"]

        # Store original rate if not already stored
        if "original_telemetry_rate" not in self.emergency_communication:
            current_rate = getattr(self, "telemetry_rate", DEFAULT_TELEMETRY_RATE_HZ)
            self.emergency_communication["original_telemetry_rate"] = current_rate

        setattr(self, "telemetry_rate", reduced_rate)
        self.logger.info(
            f"Reduced telemetry rate to {reduced_rate}Hz for emergency mode"
        )

    def _start_emergency_heartbeat(self) -> None:
        """Start sending emergency heartbeat signals."""
        self.emergency_communication["last_heartbeat"] = time.time()
        self.logger.info("Started emergency heartbeat transmission")

    def send_emergency_heartbeat(self) -> None:
        """
        Send minimal emergency heartbeat signal.

        Contains critical system status for remote monitoring during communication issues.
        """
        if not self.emergency_communication["active"]:
            return

        current_time = time.time()
        time_since_last = current_time - self.emergency_communication["last_heartbeat"]

        if time_since_last < self.emergency_communication["heartbeat_interval"]:
            return

        # Create emergency heartbeat packet
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

    def _send_minimal_packet(self, packet: Dict[str, Any]) -> None:
        """
        Send a minimal emergency packet.

        Args:
            packet: Emergency packet to send
        """
        # In a real implementation, this would send via available communication channels
        # For now, just log the emergency packet
        self.logger.warning(f"Emergency packet: {packet}")

    def attempt_communication_failover(self) -> bool:
        """
        Attempt emergency communication failover.

        Returns:
            True if failover successful, False otherwise
        """
        current_time = time.time()
        time_since_last_attempt = (
            current_time - self.redundancy_manager["last_failover_attempt"]
        )

        if time_since_last_attempt < self.redundancy_manager["failover_cooldown"]:
            self.logger.info("Failover on cooldown")
            return False

        self.redundancy_manager["last_failover_attempt"] = current_time
        self.logger.warning("Executing emergency communication failover")

        # Step 1: Try alternative 2.4 GHz channels
        for channel in self.redundancy_manager["backup_channels"]:
            if self._test_channel_viability(channel):
                success = self.safe_channel_switch(channel)
                if success:
                    self.redundancy_manager["failover_active"] = True
                    self.redundancy_manager["current_failover_level"] = 1
                    self.logger.info(
                        f"Failover successful: switched to channel {channel}"
                    )
                    return True

        # Step 2: Failover to 900 MHz band
        if self._switch_to_900mhz_band():
            self.redundancy_manager["failover_active"] = True
            self.redundancy_manager["current_failover_level"] = 2
            self.logger.info("Failover successful: switched to 900 MHz band")
            return True

        # Step 3: Enter degraded emergency mode
        self._enter_degraded_communication_mode()
        self.redundancy_manager["failover_active"] = True
        self.redundancy_manager["current_failover_level"] = 3
        self.logger.warning("Entered degraded emergency communication mode")
        return True

    def attempt_failover_recovery(self) -> bool:
        """
        Attempt to recover from communication failover.

        Returns:
            True if recovery successful, False otherwise
        """
        if not self.redundancy_manager["failover_active"]:
            return True  # Already in normal mode

        current_level = self.redundancy_manager["current_failover_level"]

        # Try to recover step by step
        if current_level >= 3:
            # Try to exit degraded mode
            if self._test_primary_channel_health():
                self.deactivate_emergency_mode()
                self.redundancy_manager["current_failover_level"] = 2
                current_level = 2

        if current_level >= 2:
            # Try to switch back to 2.4 GHz
            if self.safe_channel_switch(self.redundancy_manager["primary_channel"]):
                # Switch back to 2.4 GHz band
                setattr(self, "current_urc_band", "2.4ghz")
                self.redundancy_manager["current_failover_level"] = 1
                current_level = 1

        if current_level >= 1:
            # Try to return to normal operation
            self.redundancy_manager["failover_active"] = False
            self.redundancy_manager["current_failover_level"] = 0
            self.redundancy_manager["communication_health_score"] = 1.0
            self.logger.info("Communication failover recovery successful")
            return True

        return False

    def safe_channel_switch(self, target_channel: int) -> bool:
        """
        Safely switch to a different WiFi channel.

        Args:
            target_channel: Target channel number

        Returns:
            True if switch successful, False otherwise
        """
        # Check cooldown
        current_time = time.time()
        time_since_last_switch = (
            current_time - self.channel_safety_manager["last_channel_switch"]
        )

        if (
            time_since_last_switch
            < self.channel_safety_manager["channel_switch_cooldown"]
        ):
            self.logger.info("Channel switch on cooldown")
            return False

        # Validate channel
        if target_channel not in self.channel_safety_manager["available_channels"]:
            self.logger.error(f"Invalid channel: {target_channel}")
            return False

        # Pre-switch validation
        if not self._validate_channel_safety(target_channel):
            self.logger.warning(f"Channel {target_channel} failed safety validation")
            return False

        # Execute switch
        old_channel = self.channel_safety_manager["current_channel"]
        self.logger.info(f"Switching from channel {old_channel} to {target_channel}")

        if not self._execute_channel_switch(target_channel):
            self.logger.error(f"Failed to execute channel switch to {target_channel}")
            return False

        # Post-switch validation
        validation_timeout = self.channel_safety_manager["validation_timeout"]
        if not self._verify_channel_stability(target_channel, validation_timeout):
            self.logger.warning(
                f"Channel {target_channel} failed stability verification, rolling back"
            )
            return self._emergency_channel_rollback(old_channel)

        # Switch successful
        self.channel_safety_manager["current_channel"] = target_channel
        self.channel_safety_manager["last_channel_switch"] = current_time

        self.logger.info(f"Successfully switched to channel {target_channel}")
        return True

    def _validate_channel_safety(self, channel: int) -> bool:
        """
        Validate that a channel is safe to switch to.

        Args:
            channel: Channel number to validate

        Returns:
            True if channel is safe, False otherwise
        """
        # Check recent quality history
        quality_history = self.channel_safety_manager["channel_quality_history"].get(
            channel, []
        )
        if quality_history and len(quality_history) >= 3:
            recent_avg = sum(quality_history[-3:]) / 3
            return recent_avg > 0.5  # Minimum quality threshold

        return True  # Allow switch if no history (optimistic)

    def _execute_channel_switch(self, channel: int) -> bool:
        """
        Execute the actual channel switch.

        Args:
            channel: Channel number to switch to

        Returns:
            True if switch executed, False otherwise
        """
        try:
            # In a real implementation, this would execute system commands
            # to switch WiFi channels (iwconfig, etc.)
            self.logger.info(f"Simulated channel switch to {channel}")
            return True
        except Exception as e:
            self.logger.error(f"Channel switch execution failed: {e}")
            return False

    def _verify_channel_stability(self, channel: int, timeout: float) -> bool:
        """
        Verify that the switched channel is stable.

        Args:
            channel: Channel number to verify
            timeout: Time to wait for verification

        Returns:
            True if channel is stable, False otherwise
        """
        # Simulate stability check
        time.sleep(min(timeout, 1.0))  # Brief wait

        # Check if we can still communicate (simulated)
        quality = self._measure_channel_quality(channel)
        return quality > 0.6  # Stability threshold

    def _emergency_channel_rollback(self, original_channel: int) -> bool:
        """
        Emergency rollback to original channel.

        Args:
            original_channel: Channel to rollback to

        Returns:
            True if rollback successful, False otherwise
        """
        try:
            self._execute_channel_switch(original_channel)
            self.channel_safety_manager["current_channel"] = original_channel
            self.logger.warning(f"Emergency rollback to channel {original_channel}")
            return True
        except Exception as e:
            self.logger.error(f"Emergency rollback failed: {e}")
            return False

    def update_channel_quality_history(self, channel: int, quality: float) -> None:
        """
        Update quality history for a channel.

        Args:
            channel: Channel number
            quality: Quality score (0.0-1.0)
        """
        if channel not in self.channel_safety_manager["channel_quality_history"]:
            self.channel_safety_manager["channel_quality_history"][channel] = []

        history = self.channel_safety_manager["channel_quality_history"][channel]
        history.append(quality)

        # Keep only recent history
        if len(history) > 50:
            history.pop(0)

    def _measure_channel_quality(self, channel: int) -> float:
        """
        Measure the quality of a WiFi channel.

        Args:
            channel: Channel number to measure

        Returns:
            Quality score (0.0-1.0)
        """
        # In a real implementation, this would measure signal strength,
        # interference, packet loss, etc.
        # For simulation, return a mock quality score
        base_quality = 0.8  # Assume good baseline
        noise_factor = (channel % 3) * 0.1  # Some channels have more interference
        return max(0.0, min(1.0, base_quality - noise_factor))

    def _test_channel_viability(self, channel: int) -> bool:
        """
        Test if a channel is viable for communication.

        Args:
            channel: Channel number to test

        Returns:
            True if channel is viable, False otherwise
        """
        quality = self._measure_channel_quality(channel)
        return quality > 0.4  # Minimum viability threshold

    def _switch_to_900mhz_band(self) -> bool:
        """
        Switch to 900 MHz emergency band.

        Returns:
            True if switch successful, False otherwise
        """
        try:
            # Switch URC band configuration
            setattr(self, "current_urc_band", "900mhz")
            setattr(self, "current_urc_subband", "low")
            self.logger.info("Switched to 900 MHz emergency band")
            return True
        except Exception as e:
            self.logger.error(f"900 MHz band switch failed: {e}")
            return False

    def _enter_degraded_communication_mode(self) -> None:
        """Enter degraded communication mode."""
        self.activate_emergency_mode()

        # Additional degraded mode settings
        self.redundancy_manager["communication_health_score"] = 0.2  # Very degraded

        # Notify all systems of degraded communication
        self._broadcast_communication_degradation()

    def _broadcast_communication_degradation(self) -> None:
        """Broadcast communication degradation to all systems."""
        degradation_msg = {
            "type": "communication_degraded",
            "level": self.redundancy_manager["current_failover_level"],
            "health_score": self.redundancy_manager["communication_health_score"],
            "timestamp": time.time(),
        }

        # In a real implementation, this would broadcast to all connected systems
        self.logger.warning(f"Communication degradation broadcast: {degradation_msg}")

    def _test_primary_channel_health(self) -> bool:
        """
        Test if primary communication channel is healthy.

        Returns:
            True if primary channel is healthy, False otherwise
        """
        primary_channel = self.redundancy_manager["primary_channel"]
        quality = self._measure_channel_quality(primary_channel)
        return quality > 0.7  # Good health threshold

    def reduce_communication_load(self) -> None:
        """Reduce communication load during high interference."""
        # Temporarily reduce telemetry rate
        if not self.emergency_communication["active"]:
            current_rate = getattr(self, "telemetry_rate", DEFAULT_TELEMETRY_RATE_HZ)
            reduced_rate = max(1.0, current_rate * 0.5)  # Reduce by half

            # Store original rate
            if "original_telemetry_rate" not in self.emergency_communication:
                self.emergency_communication["original_telemetry_rate"] = current_rate

            setattr(self, "telemetry_rate", reduced_rate)
            self.logger.info(
                f"Temporarily reduced telemetry rate: {current_rate}Hz -> {reduced_rate}Hz"
            )

    def trigger_emergency_stop(self) -> None:
        """Trigger emergency stop via ROS2."""
        if self.emergency_stop_pub:
            msg = Bool()
            msg.data = True
            self.emergency_stop_pub.publish(msg)
            self.logger.critical("EMERGENCY STOP triggered via emergency communicator")

    def get_emergency_status(self) -> Dict[str, Any]:
        """
        Get current emergency communication status.

        Returns:
            Dictionary with emergency status information
        """
        return {
            "emergency_mode_active": self.emergency_communication["active"],
            "low_bandwidth_mode": self.emergency_communication["low_bandwidth_mode"],
            "failover_active": self.redundancy_manager["failover_active"],
            "current_failover_level": self.redundancy_manager["current_failover_level"],
            "communication_health_score": self.redundancy_manager[
                "communication_health_score"
            ],
            "current_channel": self.channel_safety_manager["current_channel"],
            "last_heartbeat": self.emergency_communication["last_heartbeat"],
        }
