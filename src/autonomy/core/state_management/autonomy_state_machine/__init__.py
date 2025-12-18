"""
Autonomy State Machine Package.

This package provides a hierarchical, event-driven state machine for the URC 2026
rover that coordinates all subsystems and provides state management for frontend
integration.
"""

__version__ = "1.0.0"
__author__ = "URC Machiato Team"

from .config import DEFAULT_CONFIG, get_config
from .error_handling import (
    ContextError,
    MonitoringError,
    PolicyError,
    StateMachineError,
    TransitionError,
)

# Only export working components
from .safety_manager import SafetyManager, SafetySeverity, SafetyTriggerType
from .states import SystemState  # Simple state system
from .transition_manager import TransitionManager

__all__ = [
    "SystemState",
    "SafetyManager",
    "SafetyTriggerType",
    "SafetySeverity",
    "TransitionManager",
    "StateMachineError",
    "TransitionError",
    "ContextError",
    "PolicyError",
    "MonitoringError",
    "DEFAULT_CONFIG",
    "get_config",
]
