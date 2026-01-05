"""
Simple Rover State Machine - Essential states only

7 states that cover 95% of rover operations:
- BOOT: Starting up
- READY: Ready for commands
- TELEOP: Manual control
- AUTO: Autonomous mission
- PAUSED: Temporary stop
- ESTOP: Emergency stop
- SHUTDOWN: Powering down

Use can_transition(current, target) to check valid state changes.
"""

from enum import Enum


class RoverState(Enum):
    """7 essential rover states - covers all common use cases"""

    BOOT = "BOOT"  # Starting up, initializing systems
    CALIBRATION = "CALIBRATION"  # Sensor calibration
    IDLE = "IDLE"  # Systems ready, waiting for commands
    TELEOP = "TELEOPERATION"  # Manual remote control
    AUTO = "AUTONOMOUS"  # Autonomous mission execution
    SAFETY = "SAFETY"  # Safety/Emergency stop (requires reset)
    SHUTDOWN = "SHUTDOWN"  # Powering down

    def __str__(self) -> str:
        return self.value


# Valid state transitions - consolidated for competition
VALID_TRANSITIONS = {
    RoverState.BOOT: [RoverState.CALIBRATION, RoverState.IDLE],
    RoverState.CALIBRATION: [RoverState.IDLE],
    RoverState.IDLE: [RoverState.TELEOP, RoverState.AUTO, RoverState.SHUTDOWN],
    RoverState.TELEOP: [RoverState.IDLE, RoverState.SAFETY],
    RoverState.AUTO: [RoverState.IDLE, RoverState.SAFETY],
    RoverState.SAFETY: [RoverState.IDLE],  # Reset to IDLE
    RoverState.SHUTDOWN: [],  # Terminal state
}


def can_transition(from_state: RoverState, to_state: RoverState) -> bool:
    """Check if a state transition is valid."""
    return to_state in VALID_TRANSITIONS.get(from_state, [])


# For existing code that uses old state names
SystemState = RoverState
AutonomousSubstate = type("AutonomousSubstate", (), {})  # Empty compatibility class
