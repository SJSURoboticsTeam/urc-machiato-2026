#!/usr/bin/env python3
"""
Tests for Transition Manager

Tests the transition validation, execution, and blocking logic.
"""

from unittest.mock import MagicMock, Mock

import pytest
from autonomy_state_machine.adaptive_policy_engine import (
    AdaptiveAction,
    AdaptiveActionType,
)
from autonomy_state_machine.config import PolicyConfig
from autonomy_state_machine.states import RoverState
from autonomy_state_machine.transition_manager import TransitionManager


class TestTransitionManager:
    """Test cases for TransitionManager."""

    @pytest.fixture
    def mock_logger(self):
        """Create a mock logger."""
        return Mock()

    @pytest.fixture
    def transition_manager(self, mock_logger):
        """Create a TransitionManager instance."""
        return TransitionManager(mock_logger)

    def test_validate_valid_transition(self, transition_manager):
        """Test validation of a valid transition."""
        success, message = transition_manager.validate_and_prepare_transition(
            RoverState.READY, RoverState.AUTO, "test transition"
        )

        assert success is True
        assert "is valid" in message

    def test_validate_invalid_transition(self, transition_manager):
        """Test validation of an invalid transition."""
        success, message = transition_manager.validate_and_prepare_transition(
            RoverState.BOOT, RoverState.ESTOP, "invalid transition"
        )

        assert success is False
        assert "Invalid transition" in message

    def test_blocking_with_emergency_action(self, transition_manager):
        """Test that emergency actions block transitions."""
        # Register an emergency return action
        emergency_action = AdaptiveAction(
            action_type=AdaptiveActionType.EMERGENCY_RETURN,
            priority=PolicyConfig.EMERGENCY_RETURN_PRIORITY,
            parameters={},
            success_criteria="",
            expected_duration=0.0,
            trigger_context=Mock(),
        )

        transition_manager.register_adaptive_action(emergency_action)

        # Try to transition to AUTO (should be blocked)
        success, message = transition_manager.validate_and_prepare_transition(
            RoverState.READY, RoverState.AUTO, "blocked transition"
        )

        assert success is False
        assert "blocked by high-priority adaptive action" in message

    def test_allow_safe_transition_with_emergency(self, transition_manager):
        """Test that emergency actions allow transitions to safe states."""
        # Register an emergency return action
        emergency_action = AdaptiveAction(
            action_type=AdaptiveActionType.EMERGENCY_RETURN,
            priority=PolicyConfig.EMERGENCY_RETURN_PRIORITY,
            parameters={},
            success_criteria="",
            expected_duration=0.0,
            trigger_context=Mock(),
        )

        transition_manager.register_adaptive_action(emergency_action)

        # Try to transition to READY (should be allowed)
        success, message = transition_manager.validate_and_prepare_transition(
            RoverState.AUTO, RoverState.READY, "safe transition"
        )

        assert success is True

    def test_execute_transition_success(self, transition_manager):
        """Test successful transition execution."""
        initial_state = RoverState.READY
        target_state = RoverState.AUTO

        success, message = transition_manager.execute_transition(
            initial_state, target_state, "test execution"
        )

        assert success is True
        assert "Successfully transitioned" in message
        # Note: The transition manager modifies the local variable, not the caller's variable
        # This is by design - the caller should update their state based on the return value

    def test_get_blocking_actions(self, transition_manager):
        """Test retrieving blocking actions for a transition."""
        # Register a blocking action
        blocking_action = AdaptiveAction(
            action_type=AdaptiveActionType.COMMUNICATION_SAFE_MODE,
            priority=PolicyConfig.COMMUNICATION_SAFE_MODE_PRIORITY,
            parameters={},
            success_criteria="",
            expected_duration=0.0,
            trigger_context=Mock(),
        )

        transition_manager.register_adaptive_action(blocking_action)

        # Get blocking actions for TELEOP transition
        blocking_actions = transition_manager.get_blocking_actions(RoverState.TELEOP)

        assert len(blocking_actions) == 1
        assert (
            blocking_actions[0].action_type
            == AdaptiveActionType.COMMUNICATION_SAFE_MODE
        )

    def test_unregister_adaptive_action(self, transition_manager):
        """Test unregistering adaptive actions."""
        # Register an action
        test_action = AdaptiveAction(
            action_type=AdaptiveActionType.SYSTEM_THROTTLE,
            priority=PolicyConfig.SYSTEM_THROTTLE_PRIORITY,
            parameters={},
            success_criteria="",
            expected_duration=0.0,
            trigger_context=Mock(),
        )

        transition_manager.register_adaptive_action(test_action)

        # Verify it's registered
        assert (
            AdaptiveActionType.SYSTEM_THROTTLE
            in transition_manager.active_adaptive_actions
        )

        # Unregister it
        transition_manager.unregister_adaptive_action(
            AdaptiveActionType.SYSTEM_THROTTLE
        )

        # Verify it's unregistered
        assert (
            AdaptiveActionType.SYSTEM_THROTTLE
            not in transition_manager.active_adaptive_actions
        )

    def test_can_force_transition_to_safe_states(self, transition_manager):
        """Test forcing transitions to safe states."""
        assert transition_manager.can_force_transition(RoverState.READY)
        assert transition_manager.can_force_transition(RoverState.ESTOP)
        assert transition_manager.can_force_transition(RoverState.SHUTDOWN)
        assert not transition_manager.can_force_transition(RoverState.AUTO)
        assert not transition_manager.can_force_transition(RoverState.TELEOP)
