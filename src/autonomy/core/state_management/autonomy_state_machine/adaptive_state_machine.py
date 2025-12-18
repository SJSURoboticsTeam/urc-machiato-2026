#!/usr/bin/env python3
"""
Adaptive State Machine - Enhanced rover state management with context awareness

Extends the simple 7-state machine with adaptive capabilities that respond
intelligently to changing conditions like battery levels, obstacles, and
communication status.
"""

import time
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from autonomy_interfaces.msg import AdaptiveAction as AdaptiveActionMsg
from autonomy_interfaces.msg import ContextState, ContextUpdate, SystemState
from autonomy_interfaces.srv import ChangeState, GetAdaptationHistory, GetContext
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy

from .adaptive_policy_engine import (
    AdaptiveAction,
    AdaptiveActionType,
    AdaptivePolicyEngine,
)
from .config import QoSConfig, Timing
from .context_evaluator import ContextEvaluator
from .error_handling import error_boundary, handle_service_error
from .states import RoverState
from .transition_manager import TransitionManager


class AdaptiveStateMachine(Node):
    """
    Enhanced state machine with adaptive capabilities.

    Combines the proven 7-state architecture with intelligent context-aware
    decision making for improved autonomy and safety.
    """

    def __init__(self):
        """Initialize the adaptive state machine."""
        super().__init__("adaptive_state_machine")

        # Core state machine components
        self.current_state = RoverState.BOOT
        self.previous_state = None
        self.state_entry_time = self.get_clock().now()

        # Adaptive components
        self.context_evaluator = ContextEvaluator(self)
        self.policy_engine = AdaptivePolicyEngine(self)
        self.transition_manager = TransitionManager(self.get_logger())

        # State tracking
        self.transition_history: List[Dict[str, Any]] = []
        self.context_history: List[Dict[str, Any]] = []
        self.active_adaptations: Dict[str, Any] = {}

        # Configuration
        self.declare_parameters(
            namespace="",
            parameters=[
                ("context_update_rate", 1.0),  # Hz
                ("adaptation_check_rate", 0.5),  # Hz
                ("enable_adaptive_transitions", True),
                ("max_transition_history", 100),
                ("adaptive_action_timeout", 300.0),  # seconds
            ],
        )

        # Get parameters
        self.context_update_rate = self.get_parameter("context_update_rate").value
        self.adaptation_check_rate = self.get_parameter("adaptation_check_rate").value
        self.enable_adaptive = self.get_parameter("enable_adaptive_transitions").value

        # ROS2 interfaces
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_services()
        self._setup_timers()

        self.get_logger().info("Adaptive State Machine initialized")
        self.get_logger().info(f"Current state: {self.current_state.value}")
        if self.enable_adaptive:
            self.get_logger().info("Adaptive transitions: ENABLED")
        else:
            self.get_logger().info(
                "Adaptive transitions: DISABLED (using basic transitions only)"
            )

    def _setup_publishers(self):
        """Set up ROS2 publishers."""
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Core state information
        self.state_pub = self.create_publisher(
            SystemState, "/state_machine/current_state", qos_profile
        )

        # Context and adaptation information
        self.context_pub = self.create_publisher(
            ContextState, "/state_machine/context", qos_profile
        )

        self.adaptation_pub = self.create_publisher(
            AdaptiveActionMsg, "/state_machine/adaptation", qos_profile
        )

        self.dashboard_pub = self.create_publisher(
            ContextUpdate, "/dashboard/context_update", qos_profile
        )

    def _setup_subscribers(self):
        """Set up ROS2 subscribers."""
        # Mission control commands
        self.mission_cmd_sub = self.create_subscription(
            SystemState, "/mission/commands", self._mission_command_callback, 10
        )

        # Emergency stop commands
        self.estop_sub = self.create_subscription(
            SystemState, "/emergency_stop", self._emergency_stop_callback, 10
        )

    def _setup_services(self):
        """Set up ROS2 services."""
        # State transition service
        self.transition_srv = self.create_service(
            ChangeState, "/state_machine/change_state", self._change_state_callback
        )

        # Context query service
        self.context_srv = self.create_service(
            GetContext, "/state_machine/get_context", self._get_context_callback
        )

        # Adaptation history service
        self.history_srv = self.create_service(
            GetAdaptationHistory,
            "/state_machine/get_adaptation_history",
            self._get_adaptation_history_callback,
        )

    def _setup_timers(self):
        """Set up periodic timers."""
        # Context monitoring timer
        self.context_timer = self.create_timer(
            1.0 / self.context_update_rate, self._context_monitoring_callback
        )

        # Adaptation checking timer
        self.adaptation_timer = self.create_timer(
            1.0 / self.adaptation_check_rate, self._adaptation_check_callback
        )

        # State publishing timer
        self.state_publish_timer = self.create_timer(
            0.1, self._publish_state_callback  # 10 Hz
        )

    def transition_to_state(
        self, target_state: RoverState, reason: str = "", force: bool = False
    ) -> Tuple[bool, str]:
        """
        Attempt a state transition with adaptive intelligence.

        Args:
            target_state: Desired state to transition to
            reason: Reason for the transition
            force: Whether to bypass validation (emergency use only)

        Returns:
            Tuple of (success: bool, message: str)
        """
        with error_boundary(
            self.get_logger(),
            "AdaptiveStateMachine",
            f"transition to {target_state.value}",
        ):
            # Register any active adaptive actions with the transition manager
            for action in self.active_adaptations.values():
                self.transition_manager.register_adaptive_action(action)

            # Use transition manager for validation and execution
            success, message = self.transition_manager.validate_and_prepare_transition(
                self.current_state, target_state, reason, force
            )

            if not success:
                return False, message

            # Execute pre-transition actions (context evaluation, etc.)
            pre_actions = [
                lambda: self._update_transition_history(target_state, reason),
                lambda: self._evaluate_adaptive_actions(target_state),
            ]

            # Execute post-transition actions
            post_actions = [
                lambda: self._publish_state_update(),
                lambda: self._log_transition_complete(target_state, reason),
            ]

            success, exec_message = self.transition_manager.execute_transition(
                self.current_state, target_state, reason, pre_actions, post_actions
            )

            if success:
                self.previous_state = self.current_state
                self.current_state = target_state
                self.state_entry_time = self.get_clock().now()

            return success, exec_message

    def _update_transition_history(self, target_state: RoverState, reason: str) -> None:
        """Update transition history with new transition."""
        transition_record = {
            "from_state": self.current_state.value,
            "to_state": target_state.value,
            "reason": reason,
            "timestamp": self.get_clock().now().to_msg(),
            "adaptive": getattr(self, "enable_adaptive", True),
        }

        self.transition_history.append(transition_record)
        # Keep history size reasonable
        max_history = getattr(self, "max_transition_history", 100)
        if len(self.transition_history) > max_history:
            self.transition_history.pop(0)

    def _evaluate_adaptive_actions(self, target_state: RoverState) -> None:
        """Evaluate adaptive actions for the transition."""
        if hasattr(self, "enable_adaptive") and self.enable_adaptive:
            context = self.context_evaluator.evaluate_system_context()
            adaptive_actions = self.policy_engine.evaluate_policies(context)

            for action in adaptive_actions:
                self._execute_adaptive_action(action)

    def _publish_state_update(self) -> None:
        """Publish state update after transition."""
        if hasattr(self, "_publish_current_state"):
            self._publish_current_state()

    def _log_transition_complete(self, target_state: RoverState, reason: str) -> None:
        """Log successful transition completion."""
        self.get_logger().info(
            f"Transition completed: {self.previous_state.value if self.previous_state else 'None'} -> {target_state.value} "
            f"(reason: {reason})"
        )

    def _execute_adaptive_action(self, action: AdaptiveAction):
        """Execute an adaptive action."""
        action_id = f"{action.action_type.value}_{int(time.time())}"
        self.active_adaptations[action_id] = action

        # Publish the action
        action_msg = action.to_msg()
        self.adaptation_pub.publish(action_msg)

        self.get_logger().info(
            f"Executing adaptive action: {action.action_type.value} "
            f"(priority: {action.priority})"
        )

        # Schedule action completion check
        timeout = self.get_parameter("adaptive_action_timeout").value
        self.create_timer(timeout, lambda: self._check_action_completion(action_id))

    def _check_action_completion(self, action_id: str):
        """Check if an adaptive action has completed."""
        if action_id in self.active_adaptations:
            action = self.active_adaptations[action_id]
            # In a real implementation, this would check actual completion criteria
            # For now, we'll assume actions complete after timeout
            self.policy_engine.record_action_result(action, success=True)
            del self.active_adaptations[action_id]

            self.get_logger().info(
                f"Adaptive action completed: {action.action_type.value}"
            )

    # Timer Callbacks
    def _context_monitoring_callback(self):
        """Periodic context monitoring."""
        try:
            context = self.context_evaluator.evaluate_system_context()
            self.context_history.append(context)

            # Keep history bounded
            max_history = 1000
            if len(self.context_history) > max_history:
                self.context_history.pop(0)

            # Publish context
            self.context_pub.publish(context)

            # Publish dashboard update
            dashboard_msg = self._create_dashboard_update(context)
            self.dashboard_pub.publish(dashboard_msg)

        except Exception as e:
            self.get_logger().error(f"Context monitoring failed: {e}")

    def _adaptation_check_callback(self):
        """Periodic adaptation evaluation."""
        if not self.enable_adaptive:
            return

        try:
            context = self.context_evaluator.evaluate_system_context()
            actions = self.policy_engine.evaluate_policies(context)

            # Execute high-priority actions
            for action in actions:
                if action.priority >= 70:  # High priority threshold
                    self._execute_adaptive_action(action)

        except Exception as e:
            self.get_logger().error(f"Adaptation check failed: {e}")

    def _publish_state_callback(self):
        """Publish current state information."""
        try:
            state_msg = SystemState()
            state_msg.current_state = self.current_state.value
            state_msg.timestamp = self.get_clock().now().to_msg()

            # Add additional state information
            if self.previous_state:
                state_msg.previous_state = self.previous_state.value

            state_msg.adaptive_enabled = self.enable_adaptive
            state_msg.active_adaptations = list(self.active_adaptations.keys())

            self.state_pub.publish(state_msg)

        except Exception as e:
            self.get_logger().error(f"State publishing failed: {e}")

    def _create_dashboard_update(self, context: ContextState) -> ContextUpdate:
        """Create a simplified dashboard update from full context."""
        update = ContextUpdate()

        # Core status
        update.battery_level = context.battery_level
        update.mission_status = context.mission_status
        update.mission_progress = context.mission_progress
        update.communication_active = context.communication_active
        update.safety_active = context.safety_active

        # Active adaptations
        update.active_adaptations = [
            action.action_type.value for action in self.active_adaptations.values()
        ]

        # Alert level
        if context.safety_active or context.battery_critical:
            update.alert_level = "CRITICAL"
        elif (
            context.battery_warning
            or not context.communication_active
            or context.obstacle_detected
        ):
            update.alert_level = "WARNING"
        else:
            update.alert_level = "NONE"

        # Available actions
        update.available_actions = self._get_available_actions(context)

        update.timestamp = context.timestamp

        return update

    def _get_available_actions(self, context: ContextState) -> List[str]:
        """Get list of available actions based on current context."""
        actions = []

        # Always available
        actions.append("emergency_stop")

        # State-dependent actions
        if self.current_state == RoverState.READY:
            actions.extend(["start_mission", "enter_teleop"])
        elif self.current_state == RoverState.AUTO:
            actions.extend(["pause_mission", "abort_mission"])
        elif self.current_state == RoverState.TELEOP:
            actions.extend(["pause_control", "exit_teleop"])
        elif self.current_state == RoverState.PAUSED:
            actions.extend(["resume_mission", "resume_teleop", "abort_all"])

        # Context-dependent actions
        if context.battery_critical:
            actions.append("emergency_return")
        if context.obstacle_detected:
            actions.append("request_help")
        if not context.communication_active:
            actions.append("safe_mode")

        return actions

    # Service Callbacks
    def _change_state_callback(self, request, response):
        """Handle state change requests."""
        try:
            target_state = RoverState(request.desired_state)
            success = self.transition_to_state(
                target_state,
                reason=request.reason,
                initiator=request.operator_id or "service_call",
            )

            response.success = success
            response.current_state = self.current_state.value
            response.message = (
                "Transition successful" if success else "Transition failed"
            )

        except Exception as e:
            self.get_logger().error(f"State change failed: {e}")
            response.success = False
            response.current_state = self.current_state.value
            response.message = f"Error: {str(e)}"

        return response

    def _get_context_callback(self, request, response):
        """Handle context query requests."""
        try:
            context = self.context_evaluator.evaluate_system_context()
            response.context = context
        except Exception as e:
            self.get_logger().error(f"Context query failed: {e}")
            # Return empty context on error

        return response

    def _get_adaptation_history_callback(self, request, response):
        """Handle adaptation history requests."""
        try:
            # Get recent actions
            recent_actions = list(self.active_adaptations.values())[-request.limit :]

            response.actions = [action.to_msg() for action in recent_actions]

            if request.include_context:
                # Include context for each action
                response.contexts = [
                    action.trigger_context for action in recent_actions
                ]

        except Exception as e:
            self.get_logger().error(f"Adaptation history query failed: {e}")

        return response

    # Subscriber Callbacks
    def _mission_command_callback(self, msg):
        """Handle mission command messages."""
        try:
            command = msg.data.lower()
            if command == "start":
                self.transition_to_state(
                    RoverState.AUTO, "Mission start command", "mission_control"
                )
            elif command == "stop":
                self.transition_to_state(
                    RoverState.READY, "Mission stop command", "mission_control"
                )
            elif command == "pause":
                self.transition_to_state(
                    RoverState.PAUSED, "Mission pause command", "mission_control"
                )
        except Exception as e:
            self.get_logger().error(f"Mission command failed: {e}")

    def _emergency_stop_callback(self, msg):
        """Handle emergency stop commands."""
        self.get_logger().warning("Emergency stop triggered")
        self.transition_to_state(
            RoverState.ESTOP, "Emergency stop command", "emergency_system"
        )

    # Utility Methods
    def get_current_state(self) -> RoverState:
        """Get current state."""
        return self.current_state

    def get_state_duration(self) -> float:
        """Get duration in current state (seconds)."""
        current_time = self.get_clock().now()
        duration = current_time - self.state_entry_time
        return duration.nanoseconds / 1e9

    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive system status."""
        return {
            "current_state": self.current_state.value,
            "state_duration": self.get_state_duration(),
            "adaptive_enabled": self.enable_adaptive,
            "active_adaptations": len(self.active_adaptations),
            "transition_count": len(self.transition_history),
            "context_evaluations": len(self.context_history),
        }


def main():
    """Main entry point."""
    rclpy.init()
    node = AdaptiveStateMachine()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
