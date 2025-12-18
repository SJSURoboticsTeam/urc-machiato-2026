#!/usr/bin/env python3
"""
Transition Manager - Handles state transitions and validation

Manages state transitions, validation, blocking logic, and execution
for the adaptive state machine.
"""

from typing import Optional, List, Tuple, Dict, Any
import time

from .states import RoverState, can_transition as basic_can_transition
from .adaptive_policy_engine import AdaptiveAction, AdaptiveActionType
from .error_handling import TransitionError, handle_service_error, validate_transition_request
from .config import PolicyConfig


class TransitionManager:
    """
    Manages state transitions with validation and blocking logic.
    """

    def __init__(self, node_logger):
        """Initialize the transition manager."""
        self.logger = node_logger
        self.active_adaptive_actions: Dict[AdaptiveActionType, AdaptiveAction] = {}

    def validate_and_prepare_transition(
        self,
        current_state: RoverState,
        target_state: RoverState,
        reason: str = "",
        force: bool = False
    ) -> Tuple[bool, str]:
        """
        Validate a transition request and check for blocking conditions.

        Args:
            current_state: Current rover state
            target_state: Target rover state
            reason: Reason for transition
            force: Whether to bypass validation

        Returns:
            Tuple of (can_transition, reason)
        """
        try:
            # Validate input parameters
            validate_transition_request(current_state, target_state, reason)

            # Skip validation if forced
            if force:
                return True, f"Forced transition from {current_state.value} to {target_state.value}"

            # Check basic state machine rules
            if not basic_can_transition(current_state, target_state):
                return False, f"Invalid transition from {current_state.value} to {target_state.value}"

            # Check for adaptive action blocking
            blocking_reason = self._check_adaptive_blocking(current_state, target_state)
            if blocking_reason:
                return False, blocking_reason

            return True, f"Transition from {current_state.value} to {target_state.value} is valid"

        except Exception as e:
            handle_service_error(self.logger, e, "transition validation", "TransitionManager")
            return False, f"Transition validation failed: {str(e)}"

    def execute_transition(
        self,
        current_state: RoverState,
        target_state: RoverState,
        reason: str = "",
        pre_transition_actions: Optional[List[callable]] = None,
        post_transition_actions: Optional[List[callable]] = None
    ) -> Tuple[bool, str]:
        """
        Execute a state transition with optional hooks.

        Args:
            current_state: Current state (will be updated)
            target_state: Target state
            reason: Transition reason
            pre_transition_actions: Callbacks to run before transition
            post_transition_actions: Callbacks to run after transition

        Returns:
            Tuple of (success, message)
        """
        try:
            start_time = time.time()

            # Validate transition first
            can_transition, validation_msg = self.validate_and_prepare_transition(
                current_state, target_state, reason
            )

            if not can_transition:
                return False, validation_msg

            # Run pre-transition actions
            if pre_transition_actions:
                for action in pre_transition_actions:
                    try:
                        action()
                    except Exception as e:
                        self.logger.warning(f"Pre-transition action failed: {e}")

            # Execute the transition
            old_state = current_state
            current_state = target_state

            # Run post-transition actions
            if post_transition_actions:
                for action in post_transition_actions:
                    try:
                        action()
                    except Exception as e:
                        self.logger.warning(f"Post-transition action failed: {e}")

            duration = time.time() - start_time
            self.logger.info(
                f"Transition completed: {old_state.value} -> {current_state.value} "
                ".3f"
            )

            return True, f"Successfully transitioned to {current_state.value}"

        except Exception as e:
            handle_service_error(self.logger, e, "transition execution", "TransitionManager")
            return False, f"Transition execution failed: {str(e)}"

    def _check_adaptive_blocking(
        self,
        current_state: RoverState,
        target_state: RoverState
    ) -> Optional[str]:
        """
        Check if any active adaptive actions should block this transition.

        Args:
            current_state: Current state
            target_state: Target state

        Returns:
            Blocking reason if transition should be blocked, None otherwise
        """
        for action in self.active_adaptive_actions.values():
            if action.priority >= PolicyConfig.EMERGENCY_RETURN_PRIORITY:
                # High priority actions can block transitions
                if self._should_block_for_action(action, target_state):
                    return (
                        f"Transition blocked by high-priority adaptive action: "
                        f"{action.action_type.value} (priority: {action.priority})"
                    )

            elif action.priority >= PolicyConfig.COMMUNICATION_SAFE_MODE_PRIORITY:
                # Medium priority actions block specific dangerous transitions
                if target_state == RoverState.TELEOP and action.action_type == AdaptiveActionType.COMMUNICATION_SAFE_MODE:
                    return "Teleop blocked during communication safe mode"

        return None

    def _should_block_for_action(
        self,
        action: AdaptiveAction,
        target_state: RoverState
    ) -> bool:
        """
        Determine if a specific action should block transition to target state.

        Args:
            action: The adaptive action
            target_state: Target state

        Returns:
            True if transition should be blocked
        """
        # Emergency return blocks non-essential transitions
        if action.action_type == AdaptiveActionType.EMERGENCY_RETURN:
            essential_states = {RoverState.READY, RoverState.ESTOP, RoverState.SHUTDOWN}
            return target_state not in essential_states

        # Mission abort blocks autonomous operations
        if action.action_type == AdaptiveActionType.MISSION_ABORT:
            return target_state in {RoverState.AUTO, RoverState.TELEOP}

        # Communication safe mode blocks teleop
        if action.action_type == AdaptiveActionType.COMMUNICATION_SAFE_MODE:
            return target_state == RoverState.TELEOP

        return False

    def register_adaptive_action(self, action: AdaptiveAction) -> None:
        """
        Register an active adaptive action that may affect transitions.

        Args:
            action: The adaptive action to register
        """
        self.active_adaptive_actions[action.action_type] = action
        self.logger.debug(f"Registered adaptive action: {action.action_type.value}")

    def unregister_adaptive_action(self, action_type: AdaptiveActionType) -> None:
        """
        Unregister an adaptive action.

        Args:
            action_type: Type of action to unregister
        """
        if action_type in self.active_adaptive_actions:
            del self.active_adaptive_actions[action_type]
            self.logger.debug(f"Unregistered adaptive action: {action_type.value}")

    def get_blocking_actions(self, target_state: RoverState) -> List[AdaptiveAction]:
        """
        Get all actions that would block transition to target state.

        Args:
            target_state: Target state

        Returns:
            List of blocking adaptive actions
        """
        blocking_actions = []
        for action in self.active_adaptive_actions.values():
            if self._should_block_for_action(action, target_state):
                blocking_actions.append(action)

        return blocking_actions

    def can_force_transition(self, target_state: RoverState) -> bool:
        """
        Check if a transition can be forced (bypassing normal validation).

        Args:
            target_state: Target state

        Returns:
            True if transition can be forced
        """
        # Only allow forcing to safe states
        safe_states = {RoverState.READY, RoverState.ESTOP, RoverState.SHUTDOWN}
        return target_state in safe_states


