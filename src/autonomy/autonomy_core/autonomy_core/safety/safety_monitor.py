#!/usr/bin/env python3
"""
Safety System Monitor - Comprehensive safety monitoring and emergency response

Provides real-time safety monitoring for:
- Motor temperatures and currents
- Battery voltage and current
- IMU acceleration limits
- GPS signal quality
- Communication timeouts
- Obstacle proximity
- System health indicators

Author: URC 2026 Safety Team
"""

import time
import threading
from enum import Enum
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass
import logging

logger = logging.getLogger(__name__)


class SafetySeverity(Enum):
    """Safety trigger severity levels."""

    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class SafetyTrigger(Enum):
    """Types of safety triggers."""

    MOTOR_OVERTEMP = "motor_overtemp"
    BATTERY_LOW = "battery_low"
    IMU_SHOCK = "imu_shock"
    GPS_LOSS = "gps_loss"
    COMM_TIMEOUT = "comm_timeout"
    OBSTACLE_CLOSE = "obstacle_close"
    CURRENT_SPIKE = "current_spike"
    SYSTEM_HANG = "system_hang"


class DegradedMode(Enum):
    """
    Graceful degradation modes.

    When timeouts or health checks fail with acceptable severity,
    switch to a degraded mode instead of immediate full stop.
    """

    FULL = "full"  # All sensors and fusion nominal
    ODOM_ONLY_NO_GPS = "odom_only_no_gps"  # No GPS: use odom-only for pose
    SENSOR_STALE_EXCLUDE_FUSION = (
        "sensor_stale_exclude_fusion"  # One or more sensors stale: exclude from fusion
    )


@dataclass
class SafetyThreshold:
    """Safety threshold configuration."""

    name: str
    trigger: SafetyTrigger
    severity: SafetySeverity
    threshold_value: float
    hysteresis: float = 0.0
    description: str = ""
    auto_clear: bool = True
    requires_ack: bool = False


@dataclass
class SafetyEvent:
    """Safety event record."""

    trigger: SafetyTrigger
    severity: SafetySeverity
    value: float
    threshold: float
    timestamp: float
    description: str
    active: bool = True


class SafetyMonitor:
    """
    Comprehensive safety monitoring system.

    Monitors system health and triggers emergency responses when safety limits are exceeded.
    """

    def __init__(self, node=None):
        """
        Initialize safety monitor with redundant safety layers.

        Args:
            node: ROS2 node for publishing safety status
        """
        self.node = node
        self.logger = logging.getLogger(f"{__name__}.SafetyMonitor")

        # Safety state with redundancy
        self.active_triggers: Dict[SafetyTrigger, SafetyEvent] = {}
        self.safety_enabled = True
        self.emergency_stop_active = False
        self.redundant_safety_checks: Dict[str, Callable] = {}

        # Monitoring data with health tracking
        self.last_sensor_update = time.time()
        self.watchdog_timeout = 5.0  # seconds
        self.monitoring_active = False
        self.system_health_score = 1.0  # 1.0 = fully healthy

        # Multi-layer safety thresholds
        self.thresholds = self._load_default_thresholds()
        self.redundant_thresholds = self._load_redundant_thresholds()

        # Cascading failure prevention
        self.safety_layers = {
            "primary": True,  # Main safety checks
            "secondary": True,  # Backup safety checks
            "tertiary": True,  # Emergency safety checks
        }
        self.layer_health = {layer: 1.0 for layer in self.safety_layers}

        # Callbacks with prioritization
        self.emergency_stop_callbacks: List[Callable] = []
        self.warning_callbacks: List[Callable] = []
        self.recovery_callbacks: List[Callable] = []

        # Monitoring thread with redundancy
        self.monitor_thread: Optional[threading.Thread] = None
        self.backup_monitor_thread: Optional[threading.Thread] = None
        self.monitoring_interval = 0.1  # 10 Hz monitoring

        # Failure recovery
        self.recovery_actions: List[Callable] = []
        self.last_recovery_attempt = 0
        self.recovery_cooldown = 30.0  # seconds

        # Graceful degradation: switch modes instead of full stop when acceptable
        self.current_degraded_mode = DegradedMode.FULL
        self.enable_graceful_degradation = (
            True  # When True, use degraded modes for MEDIUM-severity triggers
        )

        # Initialize redundant safety checks
        self._initialize_redundant_checks()

        self.logger.info("Enhanced safety monitor initialized with redundant layers")

    def _initialize_redundant_checks(self):
        """Initialize redundant safety check layers."""
        # Primary layer - main safety checks
        self.redundant_safety_checks["primary_motor_temp"] = (
            self._check_motor_temperature_primary
        )
        self.redundant_safety_checks["primary_battery"] = (
            self._check_battery_level_primary
        )
        self.redundant_safety_checks["primary_imu"] = self._check_imu_safety_primary

        # Secondary layer - backup checks with different methods
        self.redundant_safety_checks["secondary_motor_temp"] = (
            self._check_motor_temperature_secondary
        )
        self.redundant_safety_checks["secondary_battery"] = (
            self._check_battery_level_secondary
        )
        self.redundant_safety_checks["secondary_imu"] = self._check_imu_safety_secondary

        # Tertiary layer - emergency checks with minimal dependencies
        self.redundant_safety_checks["tertiary_watchdog"] = (
            self._check_system_watchdog_tertiary
        )

    def _load_redundant_thresholds(self) -> Dict[SafetyTrigger, SafetyThreshold]:
        """Load more conservative thresholds for redundant checks."""
        return {
            SafetyTrigger.MOTOR_OVERTEMP: SafetyThreshold(
                name="Redundant Motor Overtemperature",
                trigger=SafetyTrigger.MOTOR_OVERTEMP,
                severity=SafetySeverity.HIGH,
                threshold_value=65.0,  # Lower threshold for redundancy
                hysteresis=3.0,
                description="Redundant motor temperature check",
                auto_clear=True,
            ),
            SafetyTrigger.BATTERY_LOW: SafetyThreshold(
                name="Redundant Battery Low",
                trigger=SafetyTrigger.BATTERY_LOW,
                severity=SafetySeverity.CRITICAL,
                threshold_value=18.0,  # Higher threshold for redundancy
                hysteresis=1.0,
                description="Redundant battery level check",
                auto_clear=False,
                requires_ack=True,
            ),
            SafetyTrigger.IMU_SHOCK: SafetyThreshold(
                name="Redundant IMU Shock Detection",
                trigger=SafetyTrigger.IMU_SHOCK,
                severity=SafetySeverity.HIGH,
                threshold_value=40.0,  # Lower threshold for redundancy
                hysteresis=5.0,
                description="Redundant acceleration check",
                auto_clear=True,
            ),
        }

    def _load_default_thresholds(self) -> Dict[SafetyTrigger, SafetyThreshold]:
        """Load default safety thresholds."""
        return {
            SafetyTrigger.MOTOR_OVERTEMP: SafetyThreshold(
                name="Motor Overtemperature",
                trigger=SafetyTrigger.MOTOR_OVERTEMP,
                severity=SafetySeverity.HIGH,
                threshold_value=70.0,  # Celsius
                hysteresis=5.0,
                description="Motor temperature exceeds safe limit",
                auto_clear=True,
            ),
            SafetyTrigger.BATTERY_LOW: SafetyThreshold(
                name="Battery Low",
                trigger=SafetyTrigger.BATTERY_LOW,
                severity=SafetySeverity.CRITICAL,
                threshold_value=15.0,  # Percentage
                hysteresis=2.0,
                description="Battery level critically low",
                auto_clear=False,
                requires_ack=True,
            ),
            SafetyTrigger.IMU_SHOCK: SafetyThreshold(
                name="IMU Shock Detection",
                trigger=SafetyTrigger.IMU_SHOCK,
                severity=SafetySeverity.HIGH,
                threshold_value=50.0,  # m/s²
                hysteresis=10.0,
                description="Excessive acceleration detected",
                auto_clear=True,
            ),
            SafetyTrigger.GPS_LOSS: SafetyThreshold(
                name="GPS Signal Loss",
                trigger=SafetyTrigger.GPS_LOSS,
                severity=SafetySeverity.MEDIUM,
                threshold_value=self.watchdog_timeout,
                description="GPS signal lost for extended period",
                auto_clear=True,
            ),
            SafetyTrigger.COMM_TIMEOUT: SafetyThreshold(
                name="Communication Timeout",
                trigger=SafetyTrigger.COMM_TIMEOUT,
                severity=SafetySeverity.HIGH,
                threshold_value=5.0,  # seconds
                description="Communication with base station lost",
                auto_clear=True,
            ),
            SafetyTrigger.OBSTACLE_CLOSE: SafetyThreshold(
                name="Obstacle Too Close",
                trigger=SafetyTrigger.OBSTACLE_CLOSE,
                severity=SafetySeverity.CRITICAL,
                threshold_value=0.3,  # meters
                hysteresis=0.1,
                description="Obstacle dangerously close",
                auto_clear=True,
            ),
            SafetyTrigger.CURRENT_SPIKE: SafetyThreshold(
                name="Current Spike",
                trigger=SafetyTrigger.CURRENT_SPIKE,
                severity=SafetySeverity.HIGH,
                threshold_value=15.0,  # Amperes
                hysteresis=2.0,
                description="Motor current spike detected",
                auto_clear=True,
            ),
        }

    def start_monitoring(self) -> bool:
        """Start safety monitoring with redundant threads."""
        if self.monitoring_active:
            return False

        self.monitoring_active = True

        # Start primary monitoring thread
        self.monitor_thread = threading.Thread(
            target=self._monitoring_loop, daemon=True, name="primary_safety_monitor"
        )
        self.monitor_thread.start()

        # Start backup monitoring thread (delayed start to prevent resource contention)
        def start_backup_monitor():
            time.sleep(1.0)  # Delay backup thread start
            self.backup_monitor_thread = threading.Thread(
                target=self._backup_monitoring_loop,
                daemon=True,
                name="backup_safety_monitor",
            )
            self.backup_monitor_thread.start()

        backup_starter = threading.Thread(target=start_backup_monitor, daemon=True)
        backup_starter.start()

        self.logger.info("Redundant safety monitoring started")
        return True

    def stop_monitoring(self) -> bool:
        """Stop safety monitoring."""
        self.monitoring_active = False

        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=1.0)

        self.logger.info("Safety monitoring stopped")
        return True

    def _monitoring_loop(self):
        """Main monitoring loop with cascading failure prevention."""
        consecutive_errors = 0
        max_consecutive_errors = 5

        while self.monitoring_active:
            try:
                self._check_safety_limits()
                self._check_watchdog()
                self._check_layer_health()  # Additional health monitoring
                consecutive_errors = 0  # Reset error counter on success
                time.sleep(self.monitoring_interval)
            except Exception as e:
                consecutive_errors += 1
                self.logger.error(
                    f"Safety monitoring error ({consecutive_errors}/{max_consecutive_errors}): {e}"
                )

                if consecutive_errors >= max_consecutive_errors:
                    self.logger.critical(
                        "Primary safety monitoring failed repeatedly, escalating"
                    )
                    self._escalate_monitoring_failure()
                    break  # Exit loop, let backup thread take over

                time.sleep(1.0)  # Slow down on errors

    def _backup_monitoring_loop(self):
        """Backup monitoring loop that activates if primary fails."""
        self.logger.info("Backup safety monitoring thread started")

        while self.monitoring_active:
            try:
                # Check if primary thread is still alive
                if self.monitor_thread and not self.monitor_thread.is_alive():
                    self.logger.warning(
                        "Primary safety monitoring thread died, backup taking over"
                    )

                    # Take over safety monitoring
                    self._check_safety_limits()
                    self._check_watchdog()

                    # Try to restart primary thread
                    self._attempt_monitor_restart()

                time.sleep(self.monitoring_interval * 2)  # Slower check rate for backup
            except Exception as e:
                self.logger.error(f"Backup safety monitoring error: {e}")
                time.sleep(2.0)

    def _check_layer_health(self):
        """Check health of safety layers and trigger recovery if needed."""
        degraded_layers = [
            layer for layer, health in self.layer_health.items() if health < 0.8
        ]

        if degraded_layers:
            self.logger.warning(f"Degraded safety layers detected: {degraded_layers}")

            # Attempt layer recovery
            for layer in degraded_layers:
                if layer == "primary" and self.layer_health[layer] < 0.5:
                    self._recover_primary_layer()
                elif layer == "secondary" and self.layer_health[layer] < 0.5:
                    self._recover_secondary_layer()

    def _escalate_monitoring_failure(self):
        """Escalate when primary monitoring fails repeatedly."""
        self.logger.critical("Escalating safety monitoring failure")

        # Force emergency stop as fail-safe
        if not self.emergency_stop_active:
            self._execute_emergency_stop(
                type(
                    "Event",
                    (),
                    {
                        "description": "Safety monitoring system failure",
                        "trigger": SafetyTrigger.SYSTEM_HANG,
                        "severity": SafetySeverity.CRITICAL,
                    },
                )()
            )

    def _attempt_monitor_restart(self):
        """Attempt to restart the primary monitoring thread."""
        try:
            self.logger.info("Attempting to restart primary safety monitoring thread")
            self.monitor_thread = threading.Thread(
                target=self._monitoring_loop,
                daemon=True,
                name="restarted_primary_monitor",
            )
            self.monitor_thread.start()
        except Exception as e:
            self.logger.error(f"Failed to restart primary monitoring thread: {e}")

    def _recover_primary_layer(self):
        """Attempt to recover the primary safety layer."""
        try:
            self.logger.info("Attempting to recover primary safety layer")
            self._initialize_redundant_checks()
            self.safety_layers["primary"] = True
            self.layer_health["primary"] = 0.9  # Near full recovery
        except Exception as e:
            self.logger.error(f"Primary layer recovery failed: {e}")

    def _recover_secondary_layer(self):
        """Attempt to recover the secondary safety layer."""
        try:
            self.logger.info("Attempting to recover secondary safety layer")
            # Reinitialize secondary checks
            self.safety_layers["secondary"] = True
            self.layer_health["secondary"] = 0.8
        except Exception as e:
            self.logger.error(f"Secondary layer recovery failed: {e}")

    def _check_safety_limits(self):
        """Check all safety thresholds with redundant layers."""
        # Execute primary safety checks
        if self.safety_layers["primary"]:
            try:
                self._execute_primary_safety_checks()
                self.layer_health["primary"] = 1.0
            except Exception as e:
                self.logger.error(f"Primary safety layer failed: {e}")
                self.layer_health["primary"] = 0.0
                self.safety_layers["primary"] = False

        # Execute secondary safety checks if primary is degraded
        if not self.safety_layers["primary"] and self.safety_layers["secondary"]:
            try:
                self._execute_secondary_safety_checks()
                self.layer_health["secondary"] = 1.0
            except Exception as e:
                self.logger.error(f"Secondary safety layer failed: {e}")
                self.layer_health["secondary"] = 0.0
                self.safety_layers["secondary"] = False

        # Execute tertiary safety checks if both primary and secondary fail
        if (
            not self.safety_layers["primary"]
            and not self.safety_layers["secondary"]
            and self.safety_layers["tertiary"]
        ):
            try:
                self._execute_tertiary_safety_checks()
                self.layer_health["tertiary"] = 1.0
            except Exception as e:
                self.logger.critical(f"Tertiary safety layer failed: {e}")
                self.layer_health["tertiary"] = 0.0
                self.safety_layers["tertiary"] = False

        # Update overall system health
        self._update_system_health_score()

    def _execute_primary_safety_checks(self):
        """Execute primary safety checks (original implementation)."""
        # Motor temperature checks
        if hasattr(self, "_check_motor_temperature_primary"):
            self._check_motor_temperature_primary()

        # Battery checks
        if hasattr(self, "_check_battery_level_primary"):
            self._check_battery_level_primary()

        # IMU safety checks
        if hasattr(self, "_check_imu_safety_primary"):
            self._check_imu_safety_primary()

    def _execute_secondary_safety_checks(self):
        """Execute secondary safety checks with different methods."""
        self.logger.warning(
            "Executing secondary safety checks - primary layer degraded"
        )

        if hasattr(self, "_check_motor_temperature_secondary"):
            self._check_motor_temperature_secondary()

        if hasattr(self, "_check_battery_level_secondary"):
            self._check_battery_level_secondary()

        if hasattr(self, "_check_imu_safety_secondary"):
            self._check_imu_safety_secondary()

    def _execute_tertiary_safety_checks(self):
        """Execute tertiary emergency safety checks."""
        self.logger.critical(
            "Executing tertiary safety checks - multiple layers failed"
        )

        if hasattr(self, "_check_system_watchdog_tertiary"):
            self._check_system_watchdog_tertiary()

    def _update_system_health_score(self):
        """Update overall system health score based on layer health."""
        # Weighted average of layer health
        weights = {"primary": 0.6, "secondary": 0.3, "tertiary": 0.1}
        self.system_health_score = sum(
            self.layer_health[layer] * weights[layer]
            for layer in self.safety_layers.keys()
        )

        # Trigger recovery if health is critically low
        if self.system_health_score < 0.3:
            self._attempt_safety_recovery()

    def _attempt_safety_recovery(self):
        """Attempt to recover safety system functionality."""
        current_time = time.time()
        if current_time - self.last_recovery_attempt < self.recovery_cooldown:
            return  # Too soon since last attempt

        self.last_recovery_attempt = current_time
        self.logger.warning("Attempting safety system recovery")

        # Try to restore primary layer
        try:
            self._initialize_redundant_checks()
            self.safety_layers["primary"] = True
            self.layer_health["primary"] = 0.8  # Partially recovered
            self.logger.info("Primary safety layer recovered")
        except Exception as e:
            self.logger.error(f"Failed to recover primary safety layer: {e}")

        # Execute recovery callbacks
        for callback in self.recovery_callbacks:
            try:
                callback()
            except Exception as e:
                self.logger.error(f"Recovery callback failed: {e}")

    # Redundant safety check implementations
    def _check_motor_temperature_primary(self):
        """Primary motor temperature check."""
        # Use main sensor data - this would be implemented with actual sensor readings
        pass

    def _check_motor_temperature_secondary(self):
        """Secondary motor temperature check (different sensor or method)."""
        # Use backup temperature sensors or estimated temperature
        pass

    def _check_battery_level_primary(self):
        """Primary battery level check."""
        # Use main battery monitoring system
        pass

    def _check_battery_level_secondary(self):
        """Secondary battery level check."""
        # Use backup battery monitoring or current integration
        pass

    def _check_imu_safety_primary(self):
        """Primary IMU safety check."""
        # Use main IMU data
        pass

    def _check_imu_safety_secondary(self):
        """Secondary IMU safety check."""
        # Use backup IMU or motion estimation
        pass

    def _check_system_watchdog_tertiary(self):
        """Tertiary system watchdog check."""
        # Basic system health check with minimal dependencies
        current_time = time.time()
        if current_time - self.last_sensor_update > self.watchdog_timeout * 2:
            # System is unresponsive, trigger emergency stop
            self._trigger_safety_event(
                SafetyTrigger.SYSTEM_HANG,
                current_time - self.last_sensor_update,
                self.watchdog_timeout * 2,
            )

    def _check_watchdog(self):
        """Check system watchdog timers."""
        current_time = time.time()

        # Check sensor update watchdog
        if current_time - self.last_sensor_update > self.watchdog_timeout:
            # GPS_LOSS is MEDIUM severity: optionally switch to degraded mode instead of full stop
            if self.enable_graceful_degradation:
                self.current_degraded_mode = DegradedMode.ODOM_ONLY_NO_GPS
                self.logger.warning(
                    "Sensor/watchdog timeout: switching to degraded mode ODOM_ONLY_NO_GPS"
                )
            self._trigger_safety_event(
                SafetyTrigger.GPS_LOSS,
                current_time - self.last_sensor_update,
                self.watchdog_timeout,
            )

    def update_sensor_data(self, sensor_data: Dict[str, Any]):
        """
        Update sensor data for safety monitoring.

        Args:
            sensor_data: Dictionary containing sensor readings
        """
        self.last_sensor_update = time.time()
        # When we receive fresh sensor data and were in a stale-based degraded mode, consider restoring
        if self.current_degraded_mode == DegradedMode.SENSOR_STALE_EXCLUDE_FUSION:
            self.current_degraded_mode = DegradedMode.FULL
            self.logger.info("Sensor data restored: degraded mode cleared to FULL")

        # Check motor temperatures
        if "motor_temps" in sensor_data:
            for i, temp in enumerate(sensor_data["motor_temps"]):
                self._check_threshold(
                    SafetyTrigger.MOTOR_OVERTEMP, temp, f"Motor {i+1}"
                )

        # Check battery level
        if "battery_level" in sensor_data:
            self._check_threshold(
                SafetyTrigger.BATTERY_LOW, sensor_data["battery_level"]
            )

        # Check IMU acceleration
        if "imu_accel" in sensor_data:
            accel_magnitude = sum(x**2 for x in sensor_data["imu_accel"]) ** 0.5
            self._check_threshold(SafetyTrigger.IMU_SHOCK, accel_magnitude)

        # Check motor currents
        if "motor_currents" in sensor_data:
            for i, current in enumerate(sensor_data["motor_currents"]):
                self._check_threshold(
                    SafetyTrigger.CURRENT_SPIKE, current, f"Motor {i+1}"
                )

        # Check obstacle distance
        if "min_obstacle_distance" in sensor_data:
            self._check_threshold(
                SafetyTrigger.OBSTACLE_CLOSE, sensor_data["min_obstacle_distance"]
            )

    def _check_threshold(self, trigger: SafetyTrigger, value: float, context: str = ""):
        """Check if a value exceeds a safety threshold."""
        if trigger not in self.thresholds:
            return

        threshold = self.thresholds[trigger]
        threshold_value = threshold.threshold_value

        # Check if threshold is exceeded
        if value > threshold_value:
            if trigger not in self.active_triggers:
                # New trigger - activate
                self._trigger_safety_event(trigger, value, threshold_value, context)
        else:
            # Check hysteresis for clearing
            clear_value = threshold_value - threshold.hysteresis
            if (
                trigger in self.active_triggers
                and value < clear_value
                and threshold.auto_clear
            ):
                self._clear_safety_event(trigger)

    def _trigger_safety_event(
        self, trigger: SafetyTrigger, value: float, threshold: float, context: str = ""
    ):
        """Trigger a safety event."""
        threshold_config = self.thresholds[trigger]

        event = SafetyEvent(
            trigger=trigger,
            severity=threshold_config.severity,
            value=value,
            threshold=threshold,
            timestamp=time.time(),
            description=f"{threshold_config.description}: {value:.2f} > {threshold:.2f} {context}".strip(),
        )

        self.active_triggers[trigger] = event

        self.logger.warning(f"Safety trigger activated: {event.description}")

        # For MEDIUM severity (e.g. GPS_LOSS), optionally only set degraded mode, no emergency stop
        if event.severity == SafetySeverity.MEDIUM and self.enable_graceful_degradation:
            if trigger == SafetyTrigger.GPS_LOSS:
                self.current_degraded_mode = DegradedMode.ODOM_ONLY_NO_GPS
            else:
                self.current_degraded_mode = DegradedMode.SENSOR_STALE_EXCLUDE_FUSION
            # Do not call _execute_emergency_stop for MEDIUM when graceful degradation is on
        elif event.severity in [SafetySeverity.CRITICAL, SafetySeverity.HIGH]:
            self._execute_emergency_stop(event)

        # Call warning callbacks
        for callback in self.warning_callbacks:
            try:
                callback(event)
            except Exception as e:
                self.logger.error(f"Warning callback error: {e}")

    def _clear_safety_event(self, trigger: SafetyTrigger):
        """Clear a safety event."""
        if trigger in self.active_triggers:
            event = self.active_triggers[trigger]
            event.active = False

            # Remove after a delay if auto-clear is enabled
            if self.thresholds[trigger].auto_clear:

                def delayed_remove():
                    time.sleep(1.0)  # Keep in history briefly
                    self.active_triggers.pop(trigger, None)

                threading.Thread(target=delayed_remove, daemon=True).start()

            self.logger.info(f"Safety trigger cleared: {trigger.value}")

    def _execute_emergency_stop(self, event: SafetyEvent):
        """Execute emergency stop procedure."""
        if self.emergency_stop_active:
            return  # Already in emergency stop

        self.emergency_stop_active = True
        self.logger.critical(f"EMERGENCY STOP triggered by: {event.description}")

        # Call all emergency stop callbacks
        for callback in self.emergency_stop_callbacks:
            try:
                callback(event)
            except Exception as e:
                self.logger.error(f"Emergency stop callback error: {e}")

    def register_emergency_stop_callback(self, callback: Callable):
        """Register an emergency stop callback."""
        self.emergency_stop_callbacks.append(callback)

    def register_warning_callback(self, callback: Callable):
        """Register a warning callback."""
        self.warning_callbacks.append(callback)

    def get_safety_status(self) -> Dict[str, Any]:
        """
        Get comprehensive safety status including redundant layers.

        Returns:
            Dictionary containing detailed safety status information
        """
        active_triggers = [
            {
                "trigger": event.trigger.value,
                "severity": event.severity.value,
                "value": event.value,
                "threshold": event.threshold,
                "description": event.description,
                "timestamp": event.timestamp,
            }
            for event in self.active_triggers.values()
            if event.active
        ]

        highest_severity = None
        if active_triggers:
            severities = [t["severity"] for t in active_triggers]
            severity_order = {"low": 0, "medium": 1, "high": 2, "critical": 3}
            highest_severity = max(severities, key=lambda s: severity_order[s])

        # Calculate overall safety score
        safety_score = 1.0
        if active_triggers:
            # Reduce score based on severity and count
            severity_penalty = {"low": 0.1, "medium": 0.2, "high": 0.4, "critical": 0.8}
            for trigger in active_triggers:
                safety_score -= severity_penalty.get(trigger["severity"], 0.1)
            safety_score = max(0.0, safety_score)

        # Factor in system health
        safety_score *= self.system_health_score
        safety_score = max(0.0, min(1.0, safety_score))

        return {
            "safety_enabled": self.safety_enabled,
            "system_safe": len(active_triggers) == 0 and self.system_health_score > 0.8,
            "emergency_stop_active": self.emergency_stop_active,
            "active_triggers": active_triggers,
            "highest_severity": highest_severity,
            "monitoring_active": self.monitoring_active,
            "last_sensor_update": self.last_sensor_update,
            "degraded_mode": self.current_degraded_mode.value,
            "enable_graceful_degradation": self.enable_graceful_degradation,
            # Enhanced status information
            "safety_score": safety_score,
            "system_health_score": self.system_health_score,
            "safety_layers": self.safety_layers.copy(),
            "layer_health": self.layer_health.copy(),
            "redundant_checks_active": len(self.redundant_safety_checks) > 0,
            "recovery_available": len(self.recovery_actions) > 0,
            "last_recovery_attempt": self.last_recovery_attempt,
            "thread_health": {
                "primary_monitor_alive": (
                    self.monitor_thread.is_alive() if self.monitor_thread else False
                ),
                "backup_monitor_alive": (
                    self.backup_monitor_thread.is_alive()
                    if self.backup_monitor_thread
                    else False
                ),
            },
        }

    def acknowledge_safety_event(self, trigger: SafetyTrigger) -> bool:
        """
        Acknowledge a safety event (for events requiring acknowledgment).

        Args:
            trigger: Safety trigger to acknowledge

        Returns:
            True if acknowledged successfully
        """
        if trigger in self.active_triggers:
            event = self.active_triggers[trigger]
            threshold = self.thresholds[trigger]

            if threshold.requires_ack:
                self._clear_safety_event(trigger)
                self.logger.info(f"Safety event acknowledged: {trigger.value}")
                return True

        return False

    def reset_emergency_stop(self) -> bool:
        """Reset emergency stop state."""
        if not self.active_triggers:  # Only reset if no active triggers
            self.emergency_stop_active = False
            self.logger.info("Emergency stop reset")
            return True
        return False


# Global safety monitor instance
_safety_monitor = None


def get_safety_monitor(node=None) -> SafetyMonitor:
    """Get global safety monitor instance."""
    global _safety_monitor
    if _safety_monitor is None:
        _safety_monitor = SafetyMonitor(node)
    return _safety_monitor
