"""
Safety Subsystem - Autonomy Core

Handles emergency stop, safety monitoring, proximity detection,
recovery procedures, and graceful degradation.
"""

from .safety_watchdog import SafetyWatchdog
from .safety_monitor import SafetyMonitor, DegradedMode
from .emergency_response_coordinator import EmergencyResponseCoordinator
from .proximity_monitor import ProximityMonitor
from .recovery_procedures import (
    clear_and_recheck,
    restart_perception,
    run_recovery_procedures,
)

__all__ = [
    "SafetyWatchdog",
    "SafetyMonitor",
    "DegradedMode",
    "EmergencyResponseCoordinator",
    "ProximityMonitor",
    "clear_and_recheck",
    "restart_perception",
    "run_recovery_procedures",
]
