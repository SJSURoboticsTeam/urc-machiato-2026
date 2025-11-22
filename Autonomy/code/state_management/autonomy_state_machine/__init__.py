"""
Autonomy State Machine Package.

This package provides a hierarchical, event-driven state machine for the URC 2026
rover that coordinates all subsystems and provides state management for frontend
integration.
"""

__version__ = "1.0.0"
__author__ = "URC Machiato Team"

from .safety_manager import SafetyManager, SafetySeverity, SafetyTriggerType
from .state_machine_director import StateMachineDirector
from .states import AutonomousMode, StateMetadata, SystemState
from .transition_validator import TransitionValidator

__all__ = [
    "SystemState",
    "AutonomousMode",
    "StateMetadata",
    "StateMachineDirector",
    "SafetyManager",
    "SafetyTriggerType",
    "SafetySeverity",
    "TransitionValidator",
]
