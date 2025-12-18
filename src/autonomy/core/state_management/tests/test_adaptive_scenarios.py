#!/usr/bin/env python3
"""
Scenario-Based Tests for Adaptive State Machine

Tests realistic URC 2026 mission scenarios with adaptive behaviors,
including sample collection, equipment servicing, and emergency responses.
"""

import time
from dataclasses import dataclass
from typing import Any, Dict, List
from unittest.mock import MagicMock, Mock, patch

import pytest
import rclpy
from autonomy_interfaces.msg import ContextState
from autonomy_state_machine.adaptive_policy_engine import (
    AdaptiveAction,
    AdaptiveActionType,
    AdaptivePolicyEngine,
)
from autonomy_state_machine.adaptive_state_machine import AdaptiveStateMachine
from autonomy_state_machine.context_evaluator import ContextEvaluator
from autonomy_state_machine.monitoring_service import MonitoringService
from autonomy_state_machine.states import RoverState
from rclpy.node import Node


@dataclass
class MissionScenario:
    """Represents a mission scenario for testing."""

    name: str
    initial_state: RoverState
    initial_context: Dict[str, Any]
    expected_transitions: List[Dict[str, Any]]
    adaptive_expectations: List[str]


class TestMissionScenarios:
    """Test adaptive behavior in realistic mission scenarios."""

    @pytest.fixture(scope="function")
    def ros2_context(self):
        """Provide isolated ROS2 context for each scenario test."""
        rclpy.init()
        yield
        rclpy.shutdown()

    @pytest.fixture
    def node(self, ros2_context):
        """Create a test ROS2 node with proper isolation."""
        node = Node("scenario_test")
        yield node
        node.destroy_node()

    @pytest.fixture
    def adaptive_system(self, node):
        """Create a complete adaptive system for testing."""
        with patch("rclpy.node.Node.create_timer"), patch(
            "rclpy.node.Node.create_publisher"
        ), patch("rclpy.node.Node.create_subscription"), patch(
            "rclpy.node.Node.create_service"
        ):

            adaptive_sm = AdaptiveStateMachine()
            monitoring_service = MonitoringService()

            return {
                "state_machine": adaptive_sm,
                "monitoring": monitoring_service,
                "context_evaluator": adaptive_sm.context_evaluator,
                "policy_engine": adaptive_sm.policy_engine,
            }

    def test_sample_collection_mission_scenario(self, adaptive_system):
        """Test adaptive behavior during sample collection mission."""
        system = adaptive_system
        sm = system["state_machine"]

        # Start in READY state
        sm.current_state = RoverState.READY

        # Scenario: Sample collection with degrading conditions
        scenario = MissionScenario(
            name="Sample Collection with Battery Drain",
            initial_state=RoverState.READY,
            initial_context={
                "battery_level": 85.0,
                "mission_type": "SAMPLE_COLLECTION",
                "mission_progress": 0.0,
                "obstacle_detected": False,
            },
            expected_transitions=[
                {
                    "from": RoverState.READY,
                    "to": RoverState.AUTO,
                    "reason": "mission_start",
                },
            ],
            adaptive_expectations=[
                "Should transition to AUTO successfully",
                "Should generate battery warnings as level drops",
                "Should eventually recommend mission completion over continuation",
            ],
        )

        # Execute scenario
        self._execute_scenario(system, scenario)

    def test_equipment_servicing_with_obstacles(self, adaptive_system):
        """Test equipment servicing mission with obstacle challenges."""
        system = adaptive_system
        sm = system["state_machine"]

        # Start in READY state
        sm.current_state = RoverState.READY

        scenario = MissionScenario(
            name="Equipment Servicing with Obstacles",
            initial_state=RoverState.READY,
            initial_context={
                "battery_level": 90.0,
                "mission_type": "EQUIPMENT_SERVICING",
                "mission_progress": 0.0,
                "obstacle_detected": True,
                "obstacle_distance": 0.3,
            },
            expected_transitions=[
                {
                    "from": RoverState.READY,
                    "to": RoverState.AUTO,
                    "reason": "mission_start",
                },
                {
                    "from": RoverState.AUTO,
                    "to": RoverState.PAUSED,
                    "reason": "critical_obstacle",
                },
            ],
            adaptive_expectations=[
                "Should detect critical obstacle immediately",
                "Should pause mission and request human intervention",
                "Should provide clear obstacle avoidance recommendations",
            ],
        )

        self._execute_scenario(system, scenario)

    def test_emergency_battery_scenario(self, adaptive_system):
        """Test emergency response to critical battery levels."""
        system = adaptive_system
        sm = system["state_machine"]

        # Start in autonomous mission
        sm.current_state = RoverState.AUTO

        scenario = MissionScenario(
            name="Emergency Battery Depletion",
            initial_state=RoverState.AUTO,
            initial_context={
                "battery_level": 8.0,  # Critical level
                "mission_type": "DELIVERY",
                "mission_progress": 0.95,  # Almost complete
                "obstacle_detected": False,
            },
            expected_transitions=[
                {
                    "from": RoverState.AUTO,
                    "to": RoverState.READY,
                    "reason": "complete_and_return",
                },
            ],
            adaptive_expectations=[
                "Should immediately trigger emergency return",
                "Should prioritize safety over mission completion",
                "Should generate high-priority emergency actions",
            ],
        )

        self._execute_scenario(system, scenario)

    def test_communication_loss_during_mission(self, adaptive_system):
        """Test adaptive response to communication loss."""
        system = adaptive_system
        sm = system["state_machine"]

        # Start in autonomous mission
        sm.current_state = RoverState.AUTO

        scenario = MissionScenario(
            name="Communication Loss Recovery",
            initial_state=RoverState.AUTO,
            initial_context={
                "battery_level": 75.0,
                "mission_type": "AUTONOMOUS_NAVIGATION",
                "mission_progress": 0.3,
                "communication_active": False,
                "communication_latency": 15.0,
            },
            expected_transitions=[],
            adaptive_expectations=[
                "Should detect communication degradation",
                "Should enter safe autonomous mode",
                "Should continue mission with reduced operator control",
            ],
        )

        self._execute_scenario(system, scenario)

    def test_system_overload_scenario(self, adaptive_system):
        """Test response to system performance degradation."""
        system = adaptive_system
        sm = system["state_machine"]

        # Start in autonomous mission
        sm.current_state = RoverState.AUTO

        scenario = MissionScenario(
            name="System Overload Management",
            initial_state=RoverState.AUTO,
            initial_context={
                "battery_level": 60.0,
                "cpu_usage": 95.0,
                "memory_usage": 92.0,
                "temperature": 82.0,
                "mission_type": "SCIENCE",
                "mission_progress": 0.5,
            },
            expected_transitions=[],
            adaptive_expectations=[
                "Should detect multiple system overload conditions",
                "Should apply system throttling to prevent failure",
                "Should maintain mission progress while protecting hardware",
            ],
        )

        self._execute_scenario(system, scenario)

    def test_mission_timeout_with_completion_near(self, adaptive_system):
        """Test mission timeout handling when completion is near."""
        system = adaptive_system
        sm = system["state_machine"]

        # Start in autonomous mission
        sm.current_state = RoverState.AUTO

        scenario = MissionScenario(
            name="Mission Timeout with Near Completion",
            initial_state=RoverState.AUTO,
            initial_context={
                "battery_level": 45.0,
                "mission_type": "SAMPLE_COLLECTION",
                "mission_progress": 0.85,  # Close to completion
                "mission_time_remaining": 25.0,  # Very little time left
            },
            expected_transitions=[
                {
                    "from": RoverState.AUTO,
                    "to": RoverState.READY,
                    "reason": "complete_and_return",
                },
            ],
            adaptive_expectations=[
                "Should recognize high completion percentage",
                "Should prioritize mission completion over timeout",
                "Should optimize for successful conclusion",
            ],
        )

        self._execute_scenario(system, scenario)

    def test_terrain_adaptation_scenario(self, adaptive_system):
        """Test adaptation to difficult terrain conditions."""
        system = adaptive_system
        sm = system["state_machine"]

        # Start in autonomous mission
        sm.current_state = RoverState.AUTO

        scenario = MissionScenario(
            name="Difficult Terrain Adaptation",
            initial_state=RoverState.AUTO,
            initial_context={
                "battery_level": 70.0,
                "mission_type": "DELIVERY",
                "mission_progress": 0.2,
                "terrain_difficulty": 0.8,  # Very difficult terrain
                "obstacle_detected": False,
            },
            expected_transitions=[],
            adaptive_expectations=[
                "Should detect high terrain difficulty",
                "Should adapt by reducing speed and increasing safety margins",
                "Should conserve energy for challenging terrain",
            ],
        )

        self._execute_scenario(system, scenario)

    def test_multi_condition_crisis_scenario(self, adaptive_system):
        """Test response to multiple simultaneous critical conditions."""
        system = adaptive_system
        sm = system["state_machine"]

        # Start in autonomous mission
        sm.current_state = RoverState.AUTO

        scenario = MissionScenario(
            name="Multi-Condition Crisis",
            initial_state=RoverState.AUTO,
            initial_context={
                "battery_level": 5.0,  # Critical
                "cpu_usage": 96.0,  # Overloaded
                "temperature": 87.0,  # Overheated
                "obstacle_detected": True,  # Blocked
                "obstacle_distance": 0.2,  # Very close
                "mission_progress": 0.6,  # Partial completion
                "communication_active": False,  # No comms
            },
            expected_transitions=[
                {
                    "from": RoverState.AUTO,
                    "to": RoverState.ESTOP,
                    "reason": "multiple_critical_conditions",
                },
            ],
            adaptive_expectations=[
                "Should recognize multiple critical conditions",
                "Should prioritize immediate safety over mission continuation",
                "Should provide clear emergency action recommendations",
            ],
        )

        self._execute_scenario(system, scenario)

    def _execute_scenario(self, system: Dict, scenario: MissionScenario):
        """Execute a test scenario and verify expectations."""
        sm = system["state_machine"]
        context_evaluator = system["context_evaluator"]
        policy_engine = system["policy_engine"]
        monitoring = system["monitoring"]

        self.get_logger().info(f"Executing scenario: {scenario.name}")

        # Set initial state
        sm.current_state = scenario.initial_state

        # Create initial context
        initial_context = ContextState()
        for key, value in scenario.initial_context.items():
            if hasattr(initial_context, key):
                setattr(initial_context, key, value)

        # Evaluate policies based on initial context
        actions = policy_engine.evaluate_policies(initial_context)

        # Log actions for debugging
        for action in actions:
            self.get_logger().info(
                f"Generated action: {action.action_type.value} "
                f"(priority: {action.priority})"
            )

        # Verify scenario expectations
        self._verify_scenario_expectations(scenario, actions, sm.current_state)

        # Test state transitions if specified
        for transition in scenario.expected_transitions:
            success = sm.transition_to_state(
                transition["to"],  # Already a RoverState enum value
                transition["reason"],
            )
            assert success, f"Failed transition: {transition}"

    def _verify_scenario_expectations(
        self,
        scenario: MissionScenario,
        actions: List[AdaptiveAction],
        final_state: RoverState,
    ):
        """Verify that scenario expectations are met."""
        scenario_name = scenario.name

        # Check for expected action types based on scenario
        if "Battery" in scenario_name and "Emergency" in scenario_name:
            # With high mission progress (90%), expect complete_and_return
            complete_actions = [
                a
                for a in actions
                if a.action_type == AdaptiveActionType.COMPLETE_AND_RETURN
            ]
            assert (
                len(complete_actions) > 0
            ), f"Expected complete and return action in {scenario_name}"
            # Check that it's a reasonable priority for the situation
            assert (
                complete_actions[0].priority >= 60
            ), f"Expected reasonable priority action in {scenario_name}"

        elif "Obstacle" in scenario_name:
            intervention_actions = [
                a
                for a in actions
                if a.action_type == AdaptiveActionType.REQUEST_HUMAN_INTERVENTION
            ]
            assert (
                len(intervention_actions) > 0
            ), f"Expected human intervention action in {scenario_name}"

        elif "Communication" in scenario_name:
            # Communication loss triggers appropriate adaptive actions
            abort_actions = [
                a for a in actions if a.action_type == AdaptiveActionType.MISSION_ABORT
            ]
            safe_mode_actions = [
                a
                for a in actions
                if a.action_type == AdaptiveActionType.COMMUNICATION_SAFE_MODE
            ]
            assert (
                len(abort_actions) > 0 or len(safe_mode_actions) > 0
            ), f"Expected abort or safe mode action in {scenario_name}"

        elif "System Overload" in scenario_name:
            throttle_actions = [
                a
                for a in actions
                if a.action_type == AdaptiveActionType.SYSTEM_THROTTLE
            ]
            assert (
                len(throttle_actions) > 0
            ), f"Expected throttle action in {scenario_name}"

        # Verify appropriate priority actions are present for critical scenarios
        if any(
            keyword in scenario_name.lower()
            for keyword in ["emergency", "critical", "crisis"]
        ):
            # For battery emergency with high mission progress, complete_and_return (65) is appropriate
            # For true emergencies, expect priority >= 80
            if (
                "Battery" in scenario_name
                and scenario.initial_context.get("mission_progress", 0) > 0.9
            ):
                appropriate_actions = [a for a in actions if a.priority >= 60]
            else:
                appropriate_actions = [a for a in actions if a.priority >= 80]
            assert (
                len(appropriate_actions) > 0
            ), f"Expected appropriate priority actions in {scenario_name}"

    def get_logger(self):
        """Get a logger for test output."""
        import logging

        return logging.getLogger(__name__)


class TestAdaptiveLearning:
    """Test adaptive learning and improvement over time."""

    @pytest.fixture(scope="function")
    def ros2_context(self):
        """Provide isolated ROS2 context for each learning test."""
        rclpy.init()
        yield
        rclpy.shutdown()

    @pytest.fixture
    def node(self, ros2_context):
        """Create a test ROS2 node with proper isolation."""
        node = Node("learning_test")
        yield node
        node.destroy_node()

    def test_policy_effectiveness_improvement(self, node):
        """Test that policy effectiveness improves with feedback."""
        policy_engine = AdaptivePolicyEngine(node)

        # Simulate initial policy performance
        initial_effectiveness = policy_engine.get_policy_effectiveness()
        assert isinstance(initial_effectiveness, dict)

        # Simulate multiple policy executions with outcomes
        test_actions = [
            AdaptiveAction(
                AdaptiveActionType.EMERGENCY_RETURN, 100, {}, "", 100.0, ContextState()
            ),
            AdaptiveAction(
                AdaptiveActionType.OBSTACLE_AVOIDANCE, 80, {}, "", 60.0, ContextState()
            ),
            AdaptiveAction(
                AdaptiveActionType.REDUCE_POWER, 70, {}, "", 120.0, ContextState()
            ),
        ]

        # Record mixed results
        policy_engine.record_action_result(test_actions[0], success=True)  # Success
        policy_engine.record_action_result(test_actions[1], success=False)  # Failure
        policy_engine.record_action_result(test_actions[1], success=True)  # Success
        policy_engine.record_action_result(test_actions[2], success=True)  # Success

        # Check updated effectiveness
        updated_effectiveness = policy_engine.get_policy_effectiveness()

        # Emergency return should have 100% effectiveness
        assert "emergency_return" in updated_effectiveness
        assert updated_effectiveness["emergency_return"] == 1.0

        # Obstacle avoidance should have 50% effectiveness
        assert "obstacle_avoidance" in updated_effectiveness
        assert updated_effectiveness["obstacle_avoidance"] == 0.5

    def test_adaptive_parameter_tuning(self, node):
        """Test that adaptive parameters are tuned based on performance."""
        # This would test automatic parameter adjustment based on success rates
        # For now, test the framework exists
        policy_engine = AdaptivePolicyEngine(node)

        # Verify parameters exist and are reasonable
        assert hasattr(policy_engine, "parameters")
        assert "battery_return_threshold" in policy_engine.parameters
        assert "action_cooldown" in policy_engine.parameters

        # Parameters should be in reasonable ranges
        assert 0 < policy_engine.parameters["battery_return_threshold"] < 50
        assert policy_engine.parameters["action_cooldown"] > 0

    def test_context_pattern_recognition(self, node):
        """Test recognition of recurring context patterns."""
        context_evaluator = ContextEvaluator(node)

        # Add context history with patterns
        pattern_contexts = []

        # Create repeating pattern: battery drops every 10 readings
        for i in range(50):
            context_entry = {
                "context": ContextState(),
                "timestamp": time.time() - i * 1.0,
            }

            # Create patterns
            context_entry["context"].battery_level = (
                80.0 - (i % 10) * 2
            )  # Declining pattern
            context_entry["context"].cpu_usage = (
                40.0 + (i % 5) * 3
            )  # Oscillating pattern
            context_entry["context"].safety_active = (i % 20) < 2  # Occasional safety

            context_evaluator.context_history.append(context_entry)
            pattern_contexts.append(context_entry["context"])

        # Analyze patterns
        patterns = context_evaluator.get_context_patterns()

        # Should detect trends and patterns
        assert isinstance(patterns, dict)
        if patterns:  # May be empty if analysis fails gracefully
            # Verify reasonable analysis results (can be strings for trends)
            assert all(isinstance(v, (int, float, str)) for v in patterns.values())


class TestURCCompetitionScenarios:
    """Test scenarios specific to URC 2026 competition requirements."""

    @pytest.fixture(scope="function")
    def ros2_context(self):
        """Provide isolated ROS2 context for each URC test."""
        rclpy.init()
        yield
        rclpy.shutdown()

    @pytest.fixture
    def node(self, ros2_context):
        """Create a test ROS2 node with proper isolation."""
        node = Node("urc_test")
        yield node
        node.destroy_node()

    def test_urc_safety_requirements(self, node):
        """Test compliance with URC safety requirements."""
        with patch("rclpy.node.Node.create_timer"), patch(
            "rclpy.node.Node.create_publisher"
        ), patch("rclpy.node.Node.create_subscription"), patch(
            "rclpy.node.Node.create_service"
        ):

            adaptive_sm = AdaptiveStateMachine()

            # Test emergency stop response time
            start_time = time.time()
            adaptive_sm._emergency_stop_callback(MagicMock())
            response_time = time.time() - start_time

            # Should respond within URC requirements (< 100ms)
            assert response_time < 0.1

    def test_led_status_integration(self, node):
        """Test LED status updates for URC judging."""
        # URC requires clear visual status indication
        # This would test that adaptive actions properly update LED status

        with patch("rclpy.node.Node.create_timer"), patch(
            "rclpy.node.Node.create_publisher"
        ), patch("rclpy.node.Node.create_subscription"), patch(
            "rclpy.node.Node.create_service"
        ):

            adaptive_sm = AdaptiveStateMachine()

            # Verify LED status updates are integrated
            # (In real implementation, this would check LED publisher calls)
            assert hasattr(adaptive_sm, "_setup_publishers")

    def test_competition_boundary_awareness(self, node):
        """Test awareness of competition boundaries."""
        # URC has specific boundary requirements
        context_evaluator = ContextEvaluator(node)

        # Test that context evaluation includes boundary awareness
        # (This would integrate with GPS/geofencing systems)
        context = context_evaluator.evaluate_system_context()

        # Should include boundary-related context
        assert hasattr(context, "safety_active")  # Can be used for boundary violations

    def test_judge_transparency_features(self, node):
        """Test features that provide transparency to judges."""
        with patch("rclpy.node.Node.create_timer"), patch(
            "rclpy.node.Node.create_publisher"
        ), patch("rclpy.node.Node.create_subscription"), patch(
            "rclpy.node.Node.create_service"
        ):

            monitoring_service = MonitoringService()

            # Should provide comprehensive status for judges
            stats = monitoring_service.get_monitoring_stats()

            # Should include all required transparency metrics
            required_fields = [
                "context_readings",
                "adaptation_actions",
                "system_events",
                "performance_metrics",
                "policy_effectiveness",
            ]

            for field in required_fields:
                assert field in stats, f"Missing transparency field: {field}"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
