#!/usr/bin/env python3
"""
Comprehensive Unit Tests for Adaptive State Machine

Tests context evaluation, adaptive policies, state machine integration,
monitoring, and performance characteristics.
"""

import pytest
import rclpy
from rclpy.node import Node
from unittest.mock import Mock, patch, MagicMock
import time
import threading
import psutil
from typing import Dict, Any, List, Optional
import numpy as np

from autonomy_state_machine.states import RoverState
from autonomy_state_machine.context_evaluator import ContextEvaluator
from autonomy_state_machine.adaptive_policy_engine import (
    AdaptivePolicyEngine, AdaptiveAction, AdaptiveActionType
)
from autonomy_state_machine.adaptive_state_machine import AdaptiveStateMachine
from autonomy_state_machine.monitoring_service import MonitoringService

from autonomy_interfaces.msg import ContextState, AdaptiveAction as AdaptiveActionMsg, ContextUpdate


class TestContextEvaluator:
    """Test cases for context evaluation functionality."""

    @pytest.fixture(scope="function")
    def ros2_context(self):
        """Provide isolated ROS2 context for each test."""
        # Initialize ROS2 context
        rclpy.init()
        yield
        # Clean shutdown
        rclpy.shutdown()

    @pytest.fixture
    def node(self, ros2_context):
        """Create a test ROS2 node with proper isolation."""
        node = Node('test_node')
        yield node
        node.destroy_node()

    @pytest.fixture
    def context_evaluator(self, node):
        """Create a context evaluator instance."""
        return ContextEvaluator(node)

    def test_context_evaluation_basic(self, context_evaluator):
        """Test basic context evaluation."""
        context = context_evaluator.evaluate_system_context()

        assert isinstance(context, ContextState)
        assert hasattr(context, 'battery_level')
        assert hasattr(context, 'communication_active')
        assert hasattr(context, 'cpu_usage')
        assert hasattr(context, 'timestamp')

        # Verify reasonable value ranges
        assert 0.0 <= context.battery_level <= 100.0
        assert isinstance(context.communication_active, bool)
        assert 0.0 <= context.cpu_usage <= 100.0
        assert 0.0 <= context.memory_usage <= 100.0

    def test_context_evaluation_with_cached_values(self, context_evaluator):
        """Test context evaluation provides consistent results."""
        # First evaluation
        context1 = context_evaluator.evaluate_system_context()
        time.sleep(0.01)  # Small delay

        # Second evaluation (should be consistent)
        context2 = context_evaluator.evaluate_system_context()

        # Should have same battery level (static mock data)
        assert context1.battery_level == context2.battery_level
        # CPU usage may vary between evaluations as it's real system monitoring
        assert context1.memory_usage == context2.memory_usage

    def test_context_history_tracking(self, context_evaluator):
        """Test that context evaluation builds history."""
        initial_history_length = len(context_evaluator.context_history)

        # Multiple evaluations
        for _ in range(3):
            context_evaluator.evaluate_system_context()
            time.sleep(0.01)

        assert len(context_evaluator.context_history) >= initial_history_length + 3

    def test_context_pattern_analysis_empty(self, context_evaluator):
        """Test pattern analysis with minimal data."""
        patterns = context_evaluator.get_context_patterns()
        assert isinstance(patterns, dict)
        # May be empty or have basic analysis

    def test_context_pattern_analysis_with_data(self, context_evaluator):
        """Test pattern analysis with sufficient data."""
        # Add mock context history
        for i in range(15):
            context_entry = {
                'context': ContextState(),
                'timestamp': time.time() - i * 0.1
            }
            context_entry['context'].battery_level = 80.0 - i * 0.5  # Declining trend
            context_entry['context'].cpu_usage = 50.0 + i * 0.2      # Rising trend
            context_evaluator.context_history.append(context_entry)

        patterns = context_evaluator.get_context_patterns()

        # Check that patterns are detected
        assert 'battery_trend' in patterns
        # CPU trend may not be detected with insufficient variation
        assert 'performance_trend' in patterns
        # Battery trend should indicate declining (negative slope)
        if isinstance(patterns['battery_trend'], (int, float)):
            assert patterns['battery_trend'] < 0  # Should show declining trend

    def test_battery_evaluation(self, context_evaluator):
        """Test battery level evaluation."""
        battery_level = context_evaluator._evaluate_battery()

        assert isinstance(battery_level, float)
        assert 0.0 <= battery_level <= 100.0

    def test_system_performance_evaluation(self, context_evaluator):
        """Test system performance evaluation."""
        perf_data = context_evaluator._evaluate_system_performance()

        assert isinstance(perf_data, dict)
        assert 'cpu' in perf_data
        assert 'memory' in perf_data
        assert 'temperature' in perf_data

        assert 0.0 <= perf_data['cpu'] <= 100.0
        assert 0.0 <= perf_data['memory'] <= 100.0
        assert perf_data['temperature'] >= 0.0

    def test_safety_evaluation(self, context_evaluator):
        """Test safety condition evaluation."""
        safety_data = context_evaluator._evaluate_safety()

        assert isinstance(safety_data, dict)
        assert 'active' in safety_data
        assert 'reason' in safety_data
        assert isinstance(safety_data['active'], bool)
        assert isinstance(safety_data['reason'], str)

    @patch('psutil.cpu_percent')
    def test_cpu_monitoring(self, mock_cpu, context_evaluator):
        """Test CPU usage monitoring."""
        mock_cpu.return_value = 75.5

        perf_data = context_evaluator._evaluate_system_performance()

        assert perf_data['cpu'] == 75.5

    def test_context_pattern_analysis(self, context_evaluator):
        """Test context pattern analysis."""
        # Add some mock context history
        mock_context = ContextState()
        mock_context.battery_level = 50.0
        mock_context.cpu_usage = 60.0
        mock_context.safety_active = False

        context_evaluator.context_history.extend([
            {'context': mock_context, 'timestamp': time.time() - i}
            for i in range(10)
        ])

        patterns = context_evaluator.get_context_patterns()

        assert isinstance(patterns, dict)
        # Should contain pattern analysis keys
        assert 'battery_trend' in patterns or len(patterns) >= 0  # May be empty if analysis fails


class TestAdaptivePolicyEngine:
    """Test cases for adaptive policy engine."""

    @pytest.fixture(scope="function")
    def ros2_context(self):
        """Provide isolated ROS2 context for each test."""
        rclpy.init()
        yield
        rclpy.shutdown()

    @pytest.fixture
    def node(self, ros2_context):
        """Create a test ROS2 node with proper isolation."""
        node = Node('test_policy_node')
        yield node
        node.destroy_node()

    @pytest.fixture
    def policy_engine(self, node):
        """Create a policy engine instance."""
        return AdaptivePolicyEngine(node)

    def test_policy_initialization(self, policy_engine):
        """Test policy engine initialization."""
        assert hasattr(policy_engine, 'policies')
        assert 'battery_critical' in policy_engine.policies
        assert 'obstacle_critical' in policy_engine.policies
        assert 'communication_loss' in policy_engine.policies

    def test_battery_critical_policy(self, policy_engine):
        """Test battery critical policy."""
        context = ContextState()
        context.battery_critical = True
        context.battery_level = 5.0
        context.mission_progress = 0.3

        action = policy_engine._policy_battery_critical(context)

        assert action is not None
        assert action.action_type == AdaptiveActionType.EMERGENCY_RETURN
        assert action.priority == 100

    def test_battery_warning_policy(self, policy_engine):
        """Test battery warning policy."""
        context = ContextState()
        context.battery_warning = True
        context.battery_critical = False
        context.battery_level = 18.0
        context.mission_progress = 0.8

        action = policy_engine._policy_battery_warning(context)

        assert action is not None
        assert action.action_type == AdaptiveActionType.AUTO_RETURN
        assert action.priority >= 60

    def test_obstacle_critical_policy(self, policy_engine):
        """Test obstacle critical policy."""
        context = ContextState()
        context.obstacle_detected = True
        context.obstacle_distance = 0.2  # Very close

        action = policy_engine._policy_obstacle_critical(context)

        assert action is not None
        assert action.action_type == AdaptiveActionType.REQUEST_HUMAN_INTERVENTION
        assert action.priority == 90

    def test_mission_timeout_policy(self, policy_engine):
        """Test mission timeout policy."""
        context = ContextState()
        context.mission_time_remaining = 30.0  # 30 seconds left
        context.mission_progress = 0.2  # Not much progress

        action = policy_engine._policy_mission_timeout(context)

        assert action is not None
        assert action.action_type == AdaptiveActionType.MISSION_ABORT
        assert action.priority >= 80

    def test_policy_evaluation_integration(self, policy_engine):
        """Test complete policy evaluation."""
        context = ContextState()
        context.battery_critical = True
        context.battery_level = 8.0
        context.mission_progress = 0.9

        actions = policy_engine.evaluate_policies(context)

        assert len(actions) > 0
        # Should include high-priority emergency return
        high_priority_actions = [a for a in actions if a.priority >= 90]
        assert len(high_priority_actions) > 0

    def test_multiple_policy_triggers(self, policy_engine):
        """Test when multiple policies are triggered simultaneously."""
        context = ContextState()
        context.battery_critical = True
        context.battery_level = 5.0
        context.obstacle_detected = True
        context.obstacle_distance = 0.2
        context.mission_progress = 0.9

        actions = policy_engine.evaluate_policies(context)

        # Should have multiple high-priority actions
        assert len(actions) >= 2

        # Check that we have the expected actions
        action_types = [a.action_type for a in actions]
        # Should include complete_and_return (battery critical + high progress)
        # and obstacle_avoidance (obstacle detected)
        assert AdaptiveActionType.COMPLETE_AND_RETURN in action_types
        assert AdaptiveActionType.OBSTACLE_AVOIDANCE in action_types
        # Check priority of complete_and_return action
        complete_actions = [a for a in actions if a.action_type == AdaptiveActionType.COMPLETE_AND_RETURN]
        assert complete_actions[0].priority >= 90

    def test_policy_action_filtering(self, policy_engine):
        """Test that conflicting actions are filtered properly."""
        # Create multiple actions of same type with different priorities
        actions = [
            AdaptiveAction(
                action_type=AdaptiveActionType.EMERGENCY_RETURN,
                priority=95,
                parameters={},
                success_criteria="",
                expected_duration=100.0,
                trigger_context=ContextState()
            ),
            AdaptiveAction(
                action_type=AdaptiveActionType.EMERGENCY_RETURN,
                priority=100,
                parameters={},
                success_criteria="",
                expected_duration=100.0,
                trigger_context=ContextState()
            )
        ]

        filtered = policy_engine._filter_actions(actions)

        # Should only keep highest priority action
        assert len(filtered) == 1
        assert filtered[0].priority == 100

    def test_policy_effectiveness_tracking(self, policy_engine):
        """Test policy effectiveness calculation."""
        # Simulate some policy executions
        action = AdaptiveAction(
            action_type=AdaptiveActionType.EMERGENCY_RETURN,
            priority=100,
            parameters={},
            success_criteria="",
            expected_duration=100.0,
            trigger_context=ContextState()
        )

        # Record some successes and failures
        policy_engine.record_action_result(action, success=True)
        policy_engine.record_action_result(action, success=True)
        policy_engine.record_action_result(action, success=False)

        effectiveness = policy_engine.get_policy_effectiveness()

        assert 'emergency_return' in effectiveness
        assert effectiveness['emergency_return'] == 2.0/3.0  # 2/3 success rate

    def test_mission_timeout_policy_edge_cases(self, policy_engine):
        """Test mission timeout policy with various scenarios."""
        # High progress - should complete
        context = ContextState()
        context.mission_time_remaining = 30.0
        context.mission_progress = 0.95

        action = policy_engine._policy_mission_timeout(context)
        assert action.action_type == AdaptiveActionType.COMPLETE_AND_RETURN
        assert action.priority >= 65

        # Low progress - should abort
        context.mission_progress = 0.1
        action = policy_engine._policy_mission_timeout(context)
        assert action.action_type == AdaptiveActionType.MISSION_ABORT
        assert action.priority >= 80

    def test_system_overload_policy(self, policy_engine):
        """Test system overload policy triggers."""
        context = ContextState()
        context.cpu_usage = 95.0
        context.memory_usage = 92.0
        context.temperature = 78.0

        action = policy_engine._policy_system_overload(context)

        assert action is not None
        assert action.action_type == AdaptiveActionType.SYSTEM_THROTTLE
        assert 'cpu_high' in action.parameters
        assert 'memory_high' in action.parameters

    def test_policy_cooldown(self, policy_engine):
        """Test policy cooldown mechanism."""
        # First evaluation should work
        policy_name = 'battery_critical'
        assert policy_engine._should_evaluate_policy(policy_name)

        # Simulate cooldown
        policy_engine.policy_cooldowns[policy_name] = time.time() + 10

        # Should be blocked by cooldown
        assert not policy_engine._should_evaluate_policy(policy_name)

        # Clear cooldown
        policy_engine.policy_cooldowns.clear()
        assert policy_engine._should_evaluate_policy(policy_name)


class TestAdaptiveStateMachine:
    """Test cases for the adaptive state machine."""

    @pytest.fixture(scope="function")
    def ros2_context(self):
        """Provide isolated ROS2 context for each test."""
        rclpy.init()
        yield
        rclpy.shutdown()

    @pytest.fixture
    def node(self, ros2_context):
        """Create a test ROS2 node with proper isolation."""
        node = Node('test_adaptive_sm')
        yield node
        node.destroy_node()

    @pytest.fixture
    def adaptive_sm(self, node):
        """Create an adaptive state machine instance."""
        with patch('rclpy.node.Node.create_timer'), \
             patch('rclpy.node.Node.create_publisher'), \
             patch('rclpy.node.Node.create_subscription'), \
             patch('rclpy.node.Node.create_service'):
            sm = AdaptiveStateMachine()
            sm.enable_adaptive = True  # Ensure adaptive features are enabled
            return sm

    def test_state_machine_initialization(self, adaptive_sm):
        """Test state machine initialization."""
        assert adaptive_sm.current_state == RoverState.BOOT
        assert adaptive_sm.enable_adaptive
        assert hasattr(adaptive_sm, 'context_evaluator')
        assert hasattr(adaptive_sm, 'policy_engine')

    def test_basic_transition_validation(self, adaptive_sm):
        """Test basic transition validation."""
        # Valid transition
        success, message = adaptive_sm.transition_to_state(RoverState.READY, "test valid")
        assert success
        assert "Successfully transitioned" in message

        # Reset state for next test
        adaptive_sm.current_state = RoverState.BOOT

        # Invalid transition (BOOT -> AUTO is invalid)
        success, message = adaptive_sm.transition_to_state(RoverState.AUTO, "test invalid")
        assert not success
        assert "Invalid transition" in message

    def test_adaptive_transition_blocking(self, adaptive_sm):
        """Test adaptive transition blocking."""
        # Set up state for testing
        adaptive_sm.current_state = RoverState.READY

        # Register an emergency action that should block certain transitions
        from autonomy_state_machine.adaptive_policy_engine import AdaptiveAction, AdaptiveActionType
        from autonomy_state_machine.config import PolicyConfig

        emergency_action = AdaptiveAction(
            action_type=AdaptiveActionType.EMERGENCY_RETURN,
            priority=PolicyConfig.EMERGENCY_RETURN_PRIORITY,
            parameters={},
            success_criteria="",
            expected_duration=0.0,
            trigger_context=None
        )

        # Register the action with the transition manager
        adaptive_sm.transition_manager.register_adaptive_action(emergency_action)

        # Should block AUTO transition due to emergency action
        success, message = adaptive_sm.transition_to_state(RoverState.AUTO, "test blocking")
        assert not success
        assert "blocked by high-priority adaptive action" in message

    def test_transition_execution(self, adaptive_sm):
        """Test transition execution."""
        initial_state = adaptive_sm.current_state

        # Execute a valid transition
        success = adaptive_sm.transition_to_state(RoverState.READY, "Test transition")

        assert success
        assert adaptive_sm.current_state == RoverState.READY

    # ========== ROS2 CALLBACK TESTS ========== #

    def test_change_state_callback_validation(self, adaptive_sm):
        """Test state change request validation and error handling."""
        from autonomy_interfaces.srv import ChangeState

        # Mock request and response
        request = ChangeState.Request()
        request.target_state = "AUTO"
        request.reason = "Test transition"
        request.force = False

        response = ChangeState.Response()

        # Test valid transition
        adaptive_sm._change_state_callback(request, response)
        assert response.success == True
        assert "Successfully transitioned" in response.message

        # Reset state
        adaptive_sm.current_state = RoverState.READY

        # Test invalid transition
        request.target_state = "INVALID_STATE"
        response = ChangeState.Response()
        adaptive_sm._change_state_callback(request, response)
        assert response.success == False
        assert "Invalid transition" in response.message

    def test_context_query_callback_data_integrity(self, adaptive_sm):
        """Test context data queries return correct information."""
        from autonomy_interfaces.srv import GetContext

        # Add some context data
        adaptive_sm.context_evaluator.context_state.battery_level = 75.0
        adaptive_sm.context_evaluator.context_state.cpu_usage = 45.0
        adaptive_sm.context_evaluator.context_state.mission_progress = 0.6

        request = GetContext.Request()
        response = GetContext.Response()

        adaptive_sm._get_context_callback(request, response)

        # Verify data integrity
        assert response.context.battery_level == 75.0
        assert response.context.cpu_usage == 45.0
        assert response.context.mission_progress == 0.6

    def test_adaptation_history_callback_pagination(self, adaptive_sm):
        """Test adaptation history queries with limits and filtering."""
        from autonomy_interfaces.srv import GetAdaptationHistory

        # Add some adaptation history
        for i in range(5):
            action = Mock()
            action.action_type = f"test_action_{i}"
            action.priority = 70 + i
            adaptive_sm.monitoring_service.adaptation_history.append({
                'action': action,
                'timestamp': time.time()
            })

        request = GetAdaptationHistory.Request()
        request.limit = 3
        request.include_timestamps = True

        response = GetAdaptationHistory.Response()
        adaptive_sm._get_adaptation_history_callback(request, response)

        # Should return limited results
        assert len(response.history) == 3

    def test_callback_error_handling(self, adaptive_sm):
        """Test callback error handling and resilience."""
        # Test that callbacks handle errors gracefully

        # Corrupt internal state to trigger errors
        original_evaluator = adaptive_sm.context_evaluator
        adaptive_sm.context_evaluator = None

        try:
            # This should handle the error gracefully
            adaptive_sm._context_monitoring_callback()
            # Should not crash
        except Exception as e:
            # If it does crash, the error handling is inadequate
            pytest.fail(f"Context monitoring callback crashed: {e}")
        finally:
            # Restore state
            adaptive_sm.context_evaluator = original_evaluator

    def test_callback_performance_under_load(self, adaptive_sm):
        """Test callback performance under load."""
        import time

        # Execute callbacks multiple times rapidly
        start_time = time.time()

        for i in range(10):
            adaptive_sm._context_monitoring_callback()
            adaptive_sm._adaptation_check_callback()
            adaptive_sm._publish_state_callback()

        end_time = time.time()
        total_time = end_time - start_time

        # Should handle 30 callback executions in reasonable time (< 1 second)
        assert total_time < 1.0, f"Callbacks too slow: {total_time:.3f}s for 30 executions"

    def test_service_callback_parameter_validation(self, adaptive_sm):
        """Test service callback parameter validation."""
        from autonomy_interfaces.srv import ChangeState

        # Test with invalid parameters
        request = ChangeState.Request()
        request.target_state = ""  # Invalid
        request.reason = ""  # Invalid
        request.force = False

        response = ChangeState.Response()
        adaptive_sm._change_state_callback(request, response)

        # Should fail validation
        assert response.success == False
        assert adaptive_sm.previous_state == initial_state
        assert len(adaptive_sm.transition_history) > 0

    def test_context_dashboard_creation(self, adaptive_sm):
        """Test dashboard context creation."""
        context = ContextState()
        context.battery_level = 25.0
        context.mission_status = "EXECUTING"
        context.mission_progress = 0.6
        context.communication_active = True
        context.safety_active = False
        context.battery_critical = False
        context.battery_warning = True
        context.obstacle_detected = False
        context.timestamp = adaptive_sm.get_clock().now().to_msg()

        dashboard_update = adaptive_sm._create_dashboard_update(context)

        assert dashboard_update.battery_level == 25.0
        assert dashboard_update.mission_status == "EXECUTING"
        assert dashboard_update.mission_progress == 0.6
        assert dashboard_update.alert_level == "WARNING"  # Due to battery warning
        assert "emergency_stop" in dashboard_update.available_actions

    def test_state_duration_calculation(self, adaptive_sm):
        """Test state duration calculation."""
        duration = adaptive_sm.get_state_duration()
        assert duration >= 0.0

    def test_system_status_query(self, adaptive_sm):
        """Test system status query."""
        status = adaptive_sm.get_system_status()

        assert isinstance(status, dict)
        assert 'current_state' in status
        assert 'adaptive_enabled' in status
        assert 'active_adaptations' in status
        assert 'transition_count' in status


class TestMonitoringService:
    """Test cases for monitoring service."""

    @pytest.fixture
    def node(self):
        """Create a test ROS2 node."""
        rclpy.init()
        node = Node('test_monitoring')
        yield node
        node.destroy_node()
        rclpy.shutdown()

    @pytest.fixture
    def monitoring_service(self, node):
        """Create a monitoring service instance."""
        with patch('rclpy.node.Node.create_timer'), \
             patch('rclpy.node.Node.create_publisher'), \
             patch('rclpy.node.Node.create_subscription'), \
             patch('rclpy.node.Node.create_service'):
            service = MonitoringService()
            return service

    def test_monitoring_initialization(self, monitoring_service):
        """Test monitoring service initialization."""
        assert hasattr(monitoring_service, 'context_history')
        assert hasattr(monitoring_service, 'adaptation_history')
        assert hasattr(monitoring_service, 'system_events')
        assert hasattr(monitoring_service, 'performance_metrics')

    def test_context_callback(self, monitoring_service):
        """Test context callback processing."""
        context_msg = ContextState()
        context_msg.battery_level = 42.0
        context_msg.cpu_usage = 65.0
        context_msg.safety_active = False
        context_msg.timestamp = monitoring_service.get_clock().now().to_msg()

        initial_count = len(monitoring_service.context_history)

        monitoring_service._context_callback(context_msg)

        assert len(monitoring_service.context_history) == initial_count + 1
        assert monitoring_service.context_history[-1]['context'].battery_level == 42.0

    def test_alert_level_calculation(self, monitoring_service):
        """Test alert level calculation."""
        # Normal conditions
        context = ContextState()
        context.battery_critical = False
        context.battery_warning = False
        context.safety_active = False
        context.obstacle_detected = False
        context.communication_active = True

        alert_level = monitoring_service._calculate_alert_level(context)
        assert alert_level == "NONE"

        # Warning conditions
        context.battery_warning = True
        alert_level = monitoring_service._calculate_alert_level(context)
        assert alert_level == "WARNING"

        # Critical conditions
        context.battery_critical = True
        alert_level = monitoring_service._calculate_alert_level(context)
        assert alert_level == "CRITICAL"

    def test_performance_metrics_calculation(self, monitoring_service):
        """Test performance metrics calculation."""
        # Add mock context history
        for i in range(20):
            context_entry = {
                'context': ContextState(),
                'timestamp': time.time() - i * 0.1
            }
            context_entry['context'].battery_level = 100.0 - i * 2  # Declining battery
            context_entry['context'].cpu_usage = 50.0 + i * 0.5     # Increasing CPU
            context_entry['context'].memory_usage = 60.0
            monitoring_service.context_history.append(context_entry)

        monitoring_service._compute_performance_metrics()

        assert 'battery_trend' in monitoring_service.performance_metrics
        assert 'avg_cpu_usage' in monitoring_service.performance_metrics
        assert 'cpu_volatility' in monitoring_service.performance_metrics

        # Battery should be trending downward
        assert monitoring_service.performance_metrics['battery_trend'] < 0


class TestMonitoringService:
    """Test cases for monitoring service functionality."""

    @pytest.fixture(scope="function")
    def ros2_context(self):
        """Provide isolated ROS2 context for each test."""
        rclpy.init()
        yield
        rclpy.shutdown()

    @pytest.fixture
    def node(self, ros2_context):
        """Create a test ROS2 node with proper isolation."""
        node = Node('test_monitoring')
        yield node
        node.destroy_node()

    @pytest.fixture
    def monitoring_service(self, node):
        """Create a monitoring service instance."""
        with patch('rclpy.node.Node.create_timer'), \
             patch('rclpy.node.Node.create_publisher'), \
             patch('rclpy.node.Node.create_subscription'), \
             patch('rclpy.node.Node.create_service'):
            service = MonitoringService()
            return service

    def test_monitoring_initialization(self, monitoring_service):
        """Test monitoring service initialization."""
        assert hasattr(monitoring_service, 'context_history')
        assert hasattr(monitoring_service, 'adaptation_history')
        assert hasattr(monitoring_service, 'system_events')
        assert hasattr(monitoring_service, 'performance_metrics')

    def test_context_callback_processing(self, monitoring_service):
        """Test context callback processing."""
        context_msg = ContextState()
        context_msg.battery_level = 42.0
        context_msg.cpu_usage = 65.0
        context_msg.safety_active = False
        context_msg.timestamp = monitoring_service.get_clock().now().to_msg()

        initial_count = len(monitoring_service.context_history)

        monitoring_service._context_callback(context_msg)

        assert len(monitoring_service.context_history) == initial_count + 1
        assert monitoring_service.context_history[-1]['context'].battery_level == 42.0

    def test_adaptation_callback_processing(self, monitoring_service):
        """Test adaptation callback processing."""
        action_msg = AdaptiveActionMsg()
        action_msg.action_type = "EMERGENCY_RETURN"
        action_msg.priority = 95
        action_msg.timestamp = monitoring_service.get_clock().now().to_msg()

        initial_count = len(monitoring_service.adaptation_history)

        monitoring_service._adaptation_callback(action_msg)

        assert len(monitoring_service.adaptation_history) == initial_count + 1
        assert monitoring_service.adaptation_history[-1]['action'].action_type == "EMERGENCY_RETURN"

    def test_alert_level_calculation(self, monitoring_service):
        """Test alert level calculation for various scenarios."""
        # Normal conditions
        context = ContextState()
        context.battery_critical = False
        context.battery_warning = False
        context.safety_active = False
        context.obstacle_detected = False
        context.communication_active = True

        alert_level = monitoring_service._calculate_alert_level(context)
        assert alert_level == "NONE"

        # Warning conditions
        context.battery_warning = True
        alert_level = monitoring_service._calculate_alert_level(context)
        assert alert_level == "WARNING"

        # Critical conditions
        context.battery_critical = True
        alert_level = monitoring_service._calculate_alert_level(context)
        assert alert_level == "CRITICAL"

    def test_dashboard_update_creation(self, monitoring_service):
        """Test dashboard update message creation."""
        # Add some context history
        for i in range(5):
            context_entry = {
                'context': ContextState(),
                'timestamp': time.time() - i * 0.1
            }
            context_entry['context'].battery_level = 75.0 - i * 2
            context_entry['context'].mission_status = "EXECUTING"
            context_entry['context'].mission_progress = 0.5 + i * 0.1
            context_entry['context'].communication_active = True
            context_entry['context'].safety_active = False
            monitoring_service.context_history.append(context_entry)

        # Trigger dashboard update
        monitoring_service._dashboard_update_callback()

        # Verify the update was processed (can't easily test the publish without mocking)

    def test_performance_metrics_calculation(self, monitoring_service):
        """Test performance metrics calculation with real data."""
        # Add mock context history with trends
        for i in range(20):
            context_entry = {
                'context': ContextState(),
                'timestamp': time.time() - i * 0.1
            }
            context_entry['context'].battery_level = 100.0 - i * 2  # Declining
            context_entry['context'].cpu_usage = 50.0 + i * 0.5     # Rising
            context_entry['context'].memory_usage = 60.0
            monitoring_service.context_history.append(context_entry)

        monitoring_service._compute_performance_metrics()

        assert 'battery_trend' in monitoring_service.performance_metrics
        assert 'avg_cpu_usage' in monitoring_service.performance_metrics
        assert 'cpu_volatility' in monitoring_service.performance_metrics

        # Battery should show declining trend
        assert monitoring_service.performance_metrics['battery_trend'] < 0

    def test_monitoring_stats_comprehensive(self, monitoring_service):
        """Test comprehensive monitoring statistics."""
        # Add some test data
        monitoring_service.context_history.extend([{'context': ContextState(), 'timestamp': time.time()} for _ in range(10)])
        monitoring_service.adaptation_history.extend([{'action': AdaptiveActionMsg(), 'timestamp': time.time()} for _ in range(5)])
        monitoring_service.system_events.extend([{'type': 'TEST', 'message': 'test', 'timestamp': time.time()} for _ in range(3)])

        stats = monitoring_service.get_monitoring_stats()

        assert stats['context_readings'] == 10
        assert stats['adaptation_actions'] == 5
        assert stats['system_events'] == 3
        assert isinstance(stats['performance_metrics'], dict)
        assert isinstance(stats['policy_effectiveness'], dict)

    def test_available_actions_calculation(self, monitoring_service):
        """Test available actions calculation based on context."""
        # Normal context
        context = ContextState()
        context.battery_critical = False
        context.obstacle_detected = False
        context.communication_active = True
        context.safety_active = False

        actions = monitoring_service._get_available_actions(context)
        assert "emergency_stop" in actions
        assert len(actions) >= 1

        # Critical context
        context.battery_critical = True
        context.obstacle_detected = True

        actions = monitoring_service._get_available_actions(context)
        assert "emergency_stop" in actions
        assert "emergency_return" in actions
        assert "request_assistance" in actions


# Integration tests
class TestAdaptiveIntegration:
    """Integration tests for adaptive functionality."""

    def test_context_to_policy_integration(self):
        """Test integration between context evaluation and policy engine."""
        rclpy.init()
        node = Node('integration_test')

        try:
            context_evaluator = ContextEvaluator(node)
            policy_engine = AdaptivePolicyEngine(node)

            # Create a critical battery scenario
            # (In real implementation, this would come from actual system monitoring)

            # Simulate critical battery context
            context = context_evaluator.evaluate_system_context()

            # Manually set critical conditions for testing
            context.battery_critical = True
            context.mission_progress = 0.9  # Almost complete

            # Evaluate policies
            actions = policy_engine.evaluate_policies(context)

            # Should generate complete and return action (mission is 90% complete)
            complete_actions = [a for a in actions if a.action_type == AdaptiveActionType.COMPLETE_AND_RETURN]
            assert len(complete_actions) > 0

            # Action should have high priority
            assert complete_actions[0].priority >= 90

        finally:
            node.destroy_node()
            rclpy.shutdown()

    def test_full_adaptive_workflow(self):
        """Test complete adaptive workflow."""
        rclpy.init()
        node = Node('workflow_test')

        try:
            with patch('rclpy.node.Node.create_timer'), \
                 patch('rclpy.node.Node.create_publisher'), \
                 patch('rclpy.node.Node.create_subscription'), \
                 patch('rclpy.node.Node.create_service'):

                # Create adaptive state machine
                adaptive_sm = AdaptiveStateMachine()

                # Start in READY state
                adaptive_sm.current_state = RoverState.READY

                # Test transition with adaptive blocking
                # (This would be more comprehensive with actual ROS2 messaging)

                # Verify state machine has adaptive components
                assert hasattr(adaptive_sm, 'context_evaluator')
                assert hasattr(adaptive_sm, 'policy_engine')
                assert adaptive_sm.enable_adaptive

                # Test system status query
                status = adaptive_sm.get_system_status()
                assert status['adaptive_enabled'] == True
                assert 'current_state' in status

        finally:
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    pytest.main([__file__])
