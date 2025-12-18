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

    BOOT = "boot"  # Starting up, initializing systems
    READY = "ready"  # Systems ready, waiting for commands
    TELEOP = "teleop"  # Manual remote control
    AUTO = "auto"  # Autonomous mission execution
    PAUSED = "paused"  # Temporary stop (can resume)
    ESTOP = "estop"  # Emergency stop (requires reset)
    SHUTDOWN = "shutdown"  # Powering down

    def __str__(self) -> str:
        return self.value


# Valid state transitions - simple and clear
VALID_TRANSITIONS = {
    RoverState.BOOT: [RoverState.READY],
    RoverState.READY: [RoverState.TELEOP, RoverState.AUTO, RoverState.SHUTDOWN],
    RoverState.TELEOP: [RoverState.READY, RoverState.PAUSED, RoverState.ESTOP],
    RoverState.AUTO: [RoverState.READY, RoverState.PAUSED, RoverState.ESTOP],
    RoverState.PAUSED: [RoverState.TELEOP, RoverState.AUTO, RoverState.READY],
    RoverState.ESTOP: [RoverState.READY],  # Only way out is manual reset
    RoverState.SHUTDOWN: [],  # Terminal state
}


def can_transition(from_state: RoverState, to_state: RoverState) -> bool:
    """Check if a state transition is valid."""
    return to_state in VALID_TRANSITIONS.get(from_state, [])


# For existing code that uses old state names
SystemState = RoverState
AutonomousSubstate = type("AutonomousSubstate", (), {})  # Empty compatibility class
