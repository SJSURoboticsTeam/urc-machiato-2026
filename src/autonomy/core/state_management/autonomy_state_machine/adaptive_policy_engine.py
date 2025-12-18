#!/usr/bin/env python3
"""
Adaptive Policy Engine - Makes intelligent decisions based on context

Implements adaptive policies for battery management, obstacle avoidance,
communication loss, mission timeouts, and system overload conditions.
"""

import rclpy
from rclpy.node import Node
from typing import Dict, Any, Optional, List
from enum import Enum
import time
from dataclasses import dataclass

from autonomy_interfaces.msg import ContextState, AdaptiveAction


class AdaptiveActionType(Enum):
    """Types of adaptive actions the system can take."""
    EMERGENCY_RETURN = "emergency_return"
    REDUCE_POWER = "reduce_power"
    OBSTACLE_AVOIDANCE = "obstacle_avoidance"
    COMMUNICATION_SAFE_MODE = "communication_safe_mode"
    MISSION_ABORT = "mission_abort"
    MISSION_PAUSE = "mission_pause"
    SYSTEM_THROTTLE = "system_throttle"
    REQUEST_HUMAN_INTERVENTION = "request_human_intervention"
    COMPLETE_AND_RETURN = "complete_and_return"
    AUTO_RETURN = "auto_return"


@dataclass
class AdaptiveAction:
    """Represents an adaptive action with its parameters."""
    action_type: AdaptiveActionType
    priority: int  # 0-100, higher = more urgent
    parameters: Dict[str, Any]
    success_criteria: str
    expected_duration: float
    trigger_context: ContextState

    def to_msg(self) -> 'AdaptiveAction':
        """Convert to ROS2 message."""
        msg = AdaptiveAction()
        msg.action_type = self.action_type.value
        msg.parameters = [f"{k}={v}" for k, v in self.parameters.items()]
        msg.trigger_context = self.trigger_context
        msg.priority = self.priority
        msg.success_criteria = self.success_criteria
        msg.expected_duration = self.expected_duration
        msg.timestamp = self.trigger_context.timestamp
        return msg


class AdaptivePolicyEngine:
    """
    Intelligent policy engine that makes adaptive decisions based on system context.

    Implements policies for:
    - Battery management and power conservation
    - Obstacle detection and avoidance
    - Communication loss handling
    - Mission timeout and failure recovery
    - System performance optimization
    """

    def __init__(self, node: Node):
        """Initialize adaptive policy engine."""
        self.node = node
        self.logger = node.get_logger()

        # Policy configurations
        self.policies = {
            'battery_critical': self._policy_battery_critical,
            'battery_warning': self._policy_battery_warning,
            'obstacle_critical': self._policy_obstacle_critical,
            'obstacle_warning': self._policy_obstacle_warning,
            'communication_loss': self._policy_communication_loss,
            'mission_timeout': self._policy_mission_timeout,
            'system_overload': self._policy_system_overload,
            'terrain_difficult': self._policy_terrain_difficult
        }

        # Policy state tracking
        self.active_actions = {}
        self.action_history = []
        self.policy_cooldowns = {}  # Prevent action spam

        # Policy parameters
        self.parameters = {
            'battery_return_threshold': 15.0,  # Return when battery below this
            'obstacle_detour_timeout': 30.0,   # Max time to attempt detour
            'comm_safe_mode_timeout': 60.0,    # Communication loss before safe mode
            'mission_abort_progress_threshold': 0.8,  # Complete mission if >80% done
            'action_cooldown': 5.0,            # Minimum time between similar actions
        }

        self.logger.info("Adaptive Policy Engine initialized")

    def evaluate_policies(self, context: ContextState) -> List[AdaptiveAction]:
        """
        Evaluate all policies against current context.

        Args:
            context: Current system context

        Returns:
            List of recommended adaptive actions
        """
        actions = []

        try:
            # Evaluate each policy
            for policy_name, policy_func in self.policies.items():
                if self._should_evaluate_policy(policy_name):
                    action = policy_func(context)
                    if action:
                        actions.append(action)

            # Sort by priority (highest first)
            actions.sort(key=lambda x: x.priority, reverse=True)

            # Filter out conflicting actions and apply cooldowns
            actions = self._filter_actions(actions)

            # Log significant actions
            for action in actions:
                if action.priority >= 50:  # High priority actions
                    self.logger.info(f"High-priority adaptive action: {action.action_type.value} "
                                   f"(priority: {action.priority})")

        except Exception as e:
            self.logger.error(f"Policy evaluation failed: {e}")
            actions = []

        return actions

    def _policy_battery_critical(self, context: ContextState) -> Optional[AdaptiveAction]:
        """Handle critical battery levels."""
        if not context.battery_critical:
            return None

        mission_progress = context.mission_progress

        if mission_progress > self.parameters['mission_abort_progress_threshold']:
            # Mission nearly complete - finish it
            return AdaptiveAction(
                action_type=AdaptiveActionType.COMPLETE_AND_RETURN,
                priority=95,
                parameters={
                    'reason': 'battery_critical_mission_completion',
                    'estimated_completion_time': 300.0  # 5 minutes
                },
                success_criteria="Mission completed and returned to base",
                expected_duration=300.0,
                trigger_context=context
            )
        else:
            # Emergency return immediately
            return AdaptiveAction(
                action_type=AdaptiveActionType.EMERGENCY_RETURN,
                priority=100,
                parameters={
                    'reason': 'battery_critical_emergency',
                    'immediate_action': True
                },
                success_criteria="Returned to base safely",
                expected_duration=180.0,
                trigger_context=context
            )

    def _policy_battery_warning(self, context: ContextState) -> Optional[AdaptiveAction]:
        """Handle battery warning levels."""
        if not context.battery_warning or context.battery_critical:
            return None

        mission_progress = context.mission_progress

        if mission_progress > 0.5:
            # Continue but prepare to return
            return AdaptiveAction(
                action_type=AdaptiveActionType.AUTO_RETURN,
                priority=60,
                parameters={
                    'reason': 'battery_warning_prepare_return',
                    'return_when': context.battery_level - 5.0  # Return at 5% lower
                },
                success_criteria="Mission completed before battery depletion",
                expected_duration=600.0,  # 10 minutes
                trigger_context=context
            )
        else:
            # Reduce power consumption
            return AdaptiveAction(
                action_type=AdaptiveActionType.REDUCE_POWER,
                priority=70,
                parameters={
                    'reason': 'battery_warning_conserve_power',
                    'cpu_limit': 50.0,  # Reduce CPU usage
                    'disable_nonessential': True
                },
                success_criteria="Power consumption reduced while maintaining mission",
                expected_duration=0.0,  # Ongoing
                trigger_context=context
            )

    def _policy_obstacle_critical(self, context: ContextState) -> Optional[AdaptiveAction]:
        """Handle critical obstacle situations."""
        if not context.obstacle_detected or context.obstacle_distance > self.parameters.get('obstacle_critical', 0.3):
            return None

        return AdaptiveAction(
            action_type=AdaptiveActionType.REQUEST_HUMAN_INTERVENTION,
            priority=90,
            parameters={
                'reason': 'critical_obstacle_detected',
                'obstacle_distance': context.obstacle_distance,
                'requires_manual_clearance': True
            },
            success_criteria="Obstacle cleared or safe path found",
            expected_duration=120.0,  # 2 minutes for human intervention
            trigger_context=context
        )

    def _policy_obstacle_warning(self, context: ContextState) -> Optional[AdaptiveAction]:
        """Handle obstacle warnings with automatic avoidance."""
        if not context.obstacle_detected or context.obstacle_distance > self.parameters.get('obstacle_warning', 1.0):
            return None

        return AdaptiveAction(
            action_type=AdaptiveActionType.OBSTACLE_AVOIDANCE,
            priority=75,
            parameters={
                'reason': 'obstacle_warning_auto_avoid',
                'obstacle_distance': context.obstacle_distance,
                'timeout': self.parameters['obstacle_detour_timeout']
            },
            success_criteria="Obstacle avoided or detour found",
            expected_duration=self.parameters['obstacle_detour_timeout'],
            trigger_context=context
        )

    def _policy_communication_loss(self, context: ContextState) -> Optional[AdaptiveAction]:
        """Handle communication loss situations."""
        if context.communication_active:
            return None

        comm_loss_duration = self._get_communication_loss_duration()

        if comm_loss_duration > self.parameters['comm_safe_mode_timeout']:
            return AdaptiveAction(
                action_type=AdaptiveActionType.COMMUNICATION_SAFE_MODE,
                priority=85,
                parameters={
                    'reason': 'communication_loss_extended',
                    'loss_duration': comm_loss_duration,
                    'autonomous_only': True
                },
                success_criteria="Communication restored or safe operation maintained",
                expected_duration=0.0,  # Until communication restored
                trigger_context=context
            )

        return None

    def _policy_mission_timeout(self, context: ContextState) -> Optional[AdaptiveAction]:
        """Handle mission timeout situations."""
        if context.mission_time_remaining > 60.0:  # More than 1 minute left
            return None

        mission_progress = context.mission_progress

        if mission_progress > self.parameters['mission_abort_progress_threshold']:
            # Let it complete
            return AdaptiveAction(
                action_type=AdaptiveActionType.COMPLETE_AND_RETURN,
                priority=65,
                parameters={
                    'reason': 'mission_timeout_near_completion',
                    'time_remaining': context.mission_time_remaining
                },
                success_criteria="Mission completed despite timeout",
                expected_duration=context.mission_time_remaining,
                trigger_context=context
            )
        else:
            # Abort and return
            return AdaptiveAction(
                action_type=AdaptiveActionType.MISSION_ABORT,
                priority=80,
                parameters={
                    'reason': 'mission_timeout_abort',
                    'progress_at_abort': mission_progress
                },
                success_criteria="Safely aborted and returned to base",
                expected_duration=180.0,
                trigger_context=context
            )

    def _policy_system_overload(self, context: ContextState) -> Optional[AdaptiveAction]:
        """Handle system overload situations."""
        cpu_high = context.cpu_usage > 80.0
        memory_high = context.memory_usage > 90.0
        temp_high = context.temperature > self.parameters.get('temperature_warning', 70.0)

        if not (cpu_high or memory_high or temp_high):
            return None

        return AdaptiveAction(
            action_type=AdaptiveActionType.SYSTEM_THROTTLE,
            priority=70,
            parameters={
                'reason': 'system_overload_throttle',
                'cpu_high': cpu_high,
                'memory_high': memory_high,
                'temp_high': temp_high,
                'throttle_level': 0.5  # Reduce processing by 50%
            },
            success_criteria="System temperatures and resource usage normalized",
            expected_duration=60.0,  # 1 minute
            trigger_context=context
        )

    def _policy_terrain_difficult(self, context: ContextState) -> Optional[AdaptiveAction]:
        """Handle difficult terrain conditions."""
        if context.terrain_difficulty < 0.7:  # Not difficult enough
            return None

        return AdaptiveAction(
            action_type=AdaptiveActionType.REDUCE_POWER,
            priority=55,
            parameters={
                'reason': 'terrain_difficult_conserve_energy',
                'terrain_difficulty': context.terrain_difficulty,
                'reduce_speed': True,
                'increase_safety_margins': True
            },
            success_criteria="Terrain traversed safely with energy conservation",
            expected_duration=0.0,  # Ongoing
            trigger_context=context
        )

    def _should_evaluate_policy(self, policy_name: str) -> bool:
        """Check if a policy should be evaluated (cooldown check)."""
        if policy_name in self.policy_cooldowns:
            cooldown_end = self.policy_cooldowns[policy_name]
            if time.time() < cooldown_end:
                return False

        return True

    def _filter_actions(self, actions: List[AdaptiveAction]) -> List[AdaptiveAction]:
        """Filter out conflicting actions and apply cooldowns."""
        if not actions:
            return []

        # Sort by priority (highest first)
        sorted_actions = sorted(actions, key=lambda x: x.priority, reverse=True)

        # Remove conflicting actions (keep highest priority for each type)
        filtered_actions = []
        action_types_seen = set()

        for action in sorted_actions:
            if action.action_type not in action_types_seen:
                filtered_actions.append(action)
                action_types_seen.add(action.action_type)

                # Apply cooldown
                self.policy_cooldowns[action.action_type.value] = (
                    time.time() + self.parameters['action_cooldown']
                )

        return filtered_actions

    def _get_communication_loss_duration(self) -> float:
        """Get duration of current communication loss."""
        # This would track communication state over time
        # Placeholder implementation
        return 15.0  # 15 seconds

    def get_active_actions(self) -> List[AdaptiveAction]:
        """Get currently active adaptive actions."""
        return list(self.active_actions.values())

    def record_action_result(self, action: AdaptiveAction, success: bool):
        """Record the result of an adaptive action for learning."""
        result = {
            'action': action,
            'success': success,
            'timestamp': time.time(),
            'context': action.trigger_context
        }

        self.action_history.append(result)

        # Log for analysis
        status = "SUCCESS" if success else "FAILED"
        self.logger.info(f"Adaptive action {action.action_type.value} {status}")

    def get_policy_effectiveness(self) -> Dict[str, float]:
        """Calculate effectiveness of different policies."""
        if not self.action_history:
            return {}

        # Analyze success rates by policy type
        policy_stats = {}

        for result in self.action_history[-100:]:  # Last 100 actions
            action_type = result['action'].action_type.value
            success = result['success']

            if action_type not in policy_stats:
                policy_stats[action_type] = {'total': 0, 'success': 0}

            policy_stats[action_type]['total'] += 1
            if success:
                policy_stats[action_type]['success'] += 1

        # Calculate success rates
        effectiveness = {}
        for policy, stats in policy_stats.items():
            effectiveness[policy] = stats['success'] / stats['total'] if stats['total'] > 0 else 0.0

        return effectiveness
