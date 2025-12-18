#!/usr/bin/env python3
"""
Unit Tests for Simple State Machine Implementation

Tests the core state definitions and transition logic without ROS2 dependencies.
"""

import os
import sys
import unittest

sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..", "..", "..", "autonomy", "code")
)

from state_management.autonomy_state_machine.states import (
    VALID_TRANSITIONS,
    RoverState,
    can_transition,
)


class TestSimpleStateMachine(unittest.TestCase):
    """Test the simple 7-state rover state machine."""

    def test_all_states_defined(self):
        """Test that all expected states are defined."""
        expected_states = [
            "boot",
            "ready",
            "teleop",
            "auto",
            "paused",
            "estop",
            "shutdown",
        ]
        actual_states = [state.value for state in RoverState]

        self.assertEqual(set(expected_states), set(actual_states))

    def test_state_enum_values(self):
        """Test that state enum values are correct."""
        self.assertEqual(RoverState.BOOT.value, "boot")
        self.assertEqual(RoverState.READY.value, "ready")
        self.assertEqual(RoverState.TELEOP.value, "teleop")
        self.assertEqual(RoverState.AUTO.value, "auto")
        self.assertEqual(RoverState.PAUSED.value, "paused")
        self.assertEqual(RoverState.ESTOP.value, "estop")
        self.assertEqual(RoverState.SHUTDOWN.value, "shutdown")

    def test_valid_transitions_structure(self):
        """Test that VALID_TRANSITIONS has correct structure."""
        # All states should be keys
        for state in RoverState:
            self.assertIn(state, VALID_TRANSITIONS)

        # All values should be lists of RoverState
        for transitions in VALID_TRANSITIONS.values():
            for target_state in transitions:
                self.assertIsInstance(target_state, RoverState)

    def test_can_transition_function(self):
        """Test the can_transition function."""
        # Valid transitions
        self.assertTrue(can_transition(RoverState.BOOT, RoverState.READY))
        self.assertTrue(can_transition(RoverState.READY, RoverState.TELEOP))
        self.assertTrue(can_transition(RoverState.READY, RoverState.AUTO))
        self.assertTrue(can_transition(RoverState.TELEOP, RoverState.READY))
        self.assertTrue(can_transition(RoverState.AUTO, RoverState.READY))

        # Invalid transitions
        self.assertFalse(can_transition(RoverState.BOOT, RoverState.TELEOP))
        self.assertFalse(can_transition(RoverState.READY, RoverState.BOOT))
        self.assertFalse(can_transition(RoverState.ESTOP, RoverState.TELEOP))
        self.assertFalse(can_transition(RoverState.SHUTDOWN, RoverState.READY))

    def test_boot_transitions(self):
        """Test transitions from BOOT state."""
        valid_targets = VALID_TRANSITIONS[RoverState.BOOT]
        self.assertEqual(len(valid_targets), 1)
        self.assertIn(RoverState.READY, valid_targets)

    def test_ready_transitions(self):
        """Test transitions from READY state."""
        valid_targets = VALID_TRANSITIONS[RoverState.READY]
        self.assertEqual(len(valid_targets), 3)
        self.assertIn(RoverState.TELEOP, valid_targets)
        self.assertIn(RoverState.AUTO, valid_targets)
        self.assertIn(RoverState.SHUTDOWN, valid_targets)

    def test_teleop_transitions(self):
        """Test transitions from TELEOP state."""
        valid_targets = VALID_TRANSITIONS[RoverState.TELEOP]
        self.assertEqual(len(valid_targets), 3)
        self.assertIn(RoverState.READY, valid_targets)
        self.assertIn(RoverState.PAUSED, valid_targets)
        self.assertIn(RoverState.ESTOP, valid_targets)

    def test_auto_transitions(self):
        """Test transitions from AUTO state."""
        valid_targets = VALID_TRANSITIONS[RoverState.AUTO]
        self.assertEqual(len(valid_targets), 3)
        self.assertIn(RoverState.READY, valid_targets)
        self.assertIn(RoverState.PAUSED, valid_targets)
        self.assertIn(RoverState.ESTOP, valid_targets)

    def test_paused_transitions(self):
        """Test transitions from PAUSED state."""
        valid_targets = VALID_TRANSITIONS[RoverState.PAUSED]
        self.assertEqual(len(valid_targets), 3)
        self.assertIn(RoverState.TELEOP, valid_targets)
        self.assertIn(RoverState.AUTO, valid_targets)
        self.assertIn(RoverState.READY, valid_targets)

    def test_estop_transitions(self):
        """Test transitions from ESTOP state."""
        valid_targets = VALID_TRANSITIONS[RoverState.ESTOP]
        self.assertEqual(len(valid_targets), 1)
        self.assertIn(RoverState.READY, valid_targets)

    def test_shutdown_transitions(self):
        """Test transitions from SHUTDOWN state."""
        valid_targets = VALID_TRANSITIONS[RoverState.SHUTDOWN]
        self.assertEqual(len(valid_targets), 0)

    def test_state_string_representation(self):
        """Test that states have proper string representation."""
        self.assertEqual(str(RoverState.BOOT), "boot")
        self.assertEqual(str(RoverState.READY), "ready")
        self.assertEqual(str(RoverState.TELEOP), "teleop")
        self.assertEqual(str(RoverState.AUTO), "auto")
        self.assertEqual(str(RoverState.PAUSED), "paused")
        self.assertEqual(str(RoverState.ESTOP), "estop")
        self.assertEqual(str(RoverState.SHUTDOWN), "shutdown")


if __name__ == "__main__":
    unittest.main()
