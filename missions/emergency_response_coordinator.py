"""
Emergency Response Coordinator for URC 2026 Mission System

Handles emergency situations, coordinates responses, and manages system safety
during critical events.
"""

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional


class EmergencyType(Enum):
    """Types of emergency situations."""
    THERMAL_OVERLOAD = "thermal_overload"
    BATTERY_CRITICAL = "battery_critical"
    COMMUNICATION_LOSS = "communication_loss"
    MOTOR_FAILURE = "motor_failure"
    SENSOR_FAILURE = "sensor_failure"
    OBSTACLE_COLLISION = "obstacle_collision"
    SYSTEM_FREEZE = "system_freeze"


class EmergencySeverity(Enum):
    """Emergency severity levels."""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


@dataclass
class EmergencyRecord:
    """Record of an emergency event."""
    emergency_type: EmergencyType
    severity: EmergencySeverity
    description: str
    timestamp: float
    source: str
    system_state: Dict[str, Any]
    response_actions: List[str]
    resolved: bool = False
    resolution_time: Optional[float] = None


class EmergencyResponseCoordinator:
    """Coordinates emergency responses across the system."""

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        self.config = config or {}
        self.emergency_callbacks: Dict[EmergencyType, List[Callable]] = {}
        self.active_emergencies: List[EmergencyRecord] = []
        self.emergency_history: List[EmergencyRecord] = []
        self.lock = threading.Lock()
        self.max_history_size = 100

        # Default emergency responses
        self._setup_default_responses()

    def _setup_default_responses(self):
        """Setup default emergency response procedures."""

        # Thermal overload response
        self.register_emergency_callback(
            EmergencyType.THERMAL_OVERLOAD,
            self._handle_thermal_emergency
        )

        # Battery critical response
        self.register_emergency_callback(
            EmergencyType.BATTERY_CRITICAL,
            self._handle_battery_emergency
        )

        # Communication loss response
        self.register_emergency_callback(
            EmergencyType.COMMUNICATION_LOSS,
            self._handle_communication_loss
        )

        # Motor failure response
        self.register_emergency_callback(
            EmergencyType.MOTOR_FAILURE,
            self._handle_motor_failure
        )

    def register_emergency_callback(
        self,
        emergency_type: EmergencyType,
        callback: Callable[[EmergencyRecord], None]
    ):
        """Register a callback for a specific emergency type."""
        if emergency_type not in self.emergency_callbacks:
            self.emergency_callbacks[emergency_type] = []
        self.emergency_callbacks[emergency_type].append(callback)

    def trigger_emergency(
        self,
        emergency_type: EmergencyType,
        severity: EmergencySeverity,
        description: str,
        source: str,
        system_state: Dict[str, Any]
    ) -> EmergencyRecord:
        """Trigger an emergency response."""

        # Create emergency record
        emergency = EmergencyRecord(
            emergency_type=emergency_type,
            severity=severity,
            description=description,
            timestamp=time.time(),
            source=source,
            system_state=system_state.copy(),
            response_actions=[]
        )

        with self.lock:
            self.active_emergencies.append(emergency)

            # Trigger callbacks
            if emergency_type in self.emergency_callbacks:
                for callback in self.emergency_callbacks[emergency_type]:
                    try:
                        callback(emergency)
                    except Exception as e:
                        # Log callback error but continue
                        print(f"Emergency callback error: {e}")

        return emergency

    def resolve_emergency(self, emergency: EmergencyRecord, resolution_note: str = ""):
        """Resolve an emergency situation."""
        with self.lock:
            emergency.resolved = True
            emergency.resolution_time = time.time()

            if emergency in self.active_emergencies:
                self.active_emergencies.remove(emergency)

            emergency.response_actions.append(f"RESOLVED: {resolution_note}")

            # Move to history
            self.emergency_history.append(emergency)

            # Maintain history size
            if len(self.emergency_history) > self.max_history_size:
                self.emergency_history.pop(0)

    def get_active_emergencies(self) -> List[EmergencyRecord]:
        """Get list of active emergencies."""
        with self.lock:
            return self.active_emergencies.copy()

    def get_emergency_history(self, limit: int = 50) -> List[EmergencyRecord]:
        """Get emergency history."""
        with self.lock:
            return self.emergency_history[-limit:].copy()

    def get_emergency_stats(self) -> Dict[str, Any]:
        """Get emergency statistics."""
        with self.lock:
            total_emergencies = len(self.emergency_history)
            resolved_emergencies = len([
                e for e in self.emergency_history if e.resolved
            ])
            active_emergencies = len(self.active_emergencies)

            # Count by type
            type_counts: Dict[str, int] = {}
            for emergency in self.emergency_history:
                emergency_type = emergency.emergency_type.value
                type_counts[emergency_type] = type_counts.get(emergency_type, 0) + 1

            # Count by severity
            severity_counts: Dict[str, int] = {}
            for emergency in self.emergency_history:
                severity = emergency.severity.value
                severity_counts[severity] = severity_counts.get(severity, 0) + 1

            return {
                'total_emergencies': total_emergencies,
                'active_emergencies': active_emergencies,
                'resolved_emergencies': resolved_emergencies,
                'resolution_rate': (
                    resolved_emergencies / total_emergencies * 100
                    if total_emergencies > 0 else 0
                ),
                'by_type': type_counts,
                'by_severity': severity_counts
            }

    def _handle_thermal_emergency(self, emergency: EmergencyRecord):
        """Handle thermal overload emergency."""
        emergency.response_actions.extend([
            "Reduce system load",
            "Increase cooling if available",
            "Monitor temperature sensors",
            "Prepare for graceful shutdown if temperature continues rising"
        ])

        # In a real system, this would trigger:
        # - CPU frequency reduction
        # - Non-critical process suspension
        # - Cooling system activation
        # - Temperature monitoring escalation

    def _handle_battery_emergency(self, emergency: EmergencyRecord):
        """Handle battery critical emergency."""
        emergency.response_actions.extend([
            "Reduce power consumption",
            "Return to base if possible",
            "Disable non-essential systems",
            "Monitor battery voltage and current"
        ])

        # In a real system, this would trigger:
        # - Motor power reduction
        # - Sensor sampling rate reduction
        # - Navigation to nearest charging station
        # - Low-power mode activation

    def _handle_communication_loss(self, emergency: EmergencyRecord):
        """Handle communication loss emergency."""
        emergency.response_actions.extend([
            "Switch to autonomous mode",
            "Attempt communication recovery",
            "Use cached mission data",
            "Prepare for manual override if communication restored"
        ])

        # In a real system, this would trigger:
        # - Autonomous operation mode
        # - Communication retry attempts
        # - Mission state preservation
        # - Emergency beacon activation

    def _handle_motor_failure(self, emergency: EmergencyRecord):
        """Handle motor failure emergency."""
        emergency.response_actions.extend([
            "Stop affected motor(s)",
            "Redistribute power to remaining motors",
            "Attempt motor restart",
            "Switch to reduced mobility mode"
        ])

        # In a real system, this would trigger:
        # - Motor controller reset
        # - Power redistribution
        # - Alternative locomotion mode
        # - Diagnostic data collection

    def check_system_health(self) -> List[EmergencyRecord]:
        """Check system health and trigger emergencies if needed."""
        # Placeholder for system health monitoring
        # In a real implementation, this would monitor:
        # - Temperature sensors
        # - Battery voltage/current
        # - Motor status
        # - Communication links
        # - CPU/memory usage

        emergencies_triggered = []

        # Example health checks (placeholder values)
        system_metrics = {
            'temperature_celsius': 65.0,  # Normal
            'battery_voltage': 11.5,      # Normal
            'cpu_usage': 45.0,           # Normal
            'communication_ok': True     # Normal
        }

        # Check temperature
        if system_metrics['temperature_celsius'] > 80.0:
            emergency = self.trigger_emergency(
                EmergencyType.THERMAL_OVERLOAD,
                EmergencySeverity.HIGH,
                f"Temperature too high: {system_metrics['temperature_celsius']}°C",
                "system_monitor",
                system_metrics
            )
            emergencies_triggered.append(emergency)

        # Check battery
        if system_metrics['battery_voltage'] < 10.0:
            emergency = self.trigger_emergency(
                EmergencyType.BATTERY_CRITICAL,
                EmergencySeverity.CRITICAL,
                f"Battery voltage critical: {system_metrics['battery_voltage']}V",
                "system_monitor",
                system_metrics
            )
            emergencies_triggered.append(emergency)

        # Check communication
        if not system_metrics['communication_ok']:
            emergency = self.trigger_emergency(
                EmergencyType.COMMUNICATION_LOSS,
                EmergencySeverity.HIGH,
                "Communication link lost",
                "system_monitor",
                system_metrics
            )
            emergencies_triggered.append(emergency)

        return emergencies_triggered
