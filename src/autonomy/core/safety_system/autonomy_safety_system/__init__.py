"""
Autonomy Safety System Package.

Single clear safety path for URC 2026 Mars Rover:
- safety_watchdog: software safety monitor (heartbeat, violations, thresholds)
- emergency_response_coordinator: ties hardware E-stop, topic, and watchdog
- safety_dashboard: ops dashboard for safety status
- safety_integration_tester: CI/testing tool for safety integration
"""

__version__ = "0.1.0"
__author__ = "URC Machiato Safety Team"

from .emergency_response_coordinator import EmergencyResponseCoordinator
from .safety_dashboard import SafetyDashboard
from .safety_integration_tester import SafetyIntegrationTester
from .safety_watchdog import SafetyWatchdog

__all__ = [
    "SafetyWatchdog",
    "EmergencyResponseCoordinator",
    "SafetyDashboard",
    "SafetyIntegrationTester",
]
