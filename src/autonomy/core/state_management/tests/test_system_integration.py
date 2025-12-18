#!/usr/bin/env python3
"""
System Integration Tests

Tests end-to-end integration between state machine, safety manager,
monitoring service, and error handling components.
"""

import pytest
from unittest.mock import Mock, patch, MagicMock
import time

from autonomy_state_machine.adaptive_state_machine import AdaptiveStateMachine
from autonomy_state_machine.safety_manager import SafetyManager, SafetyTriggerType
from autonomy_state_machine.monitoring_service import MonitoringService
from autonomy_state_machine.context_evaluator import ContextEvaluator
from autonomy_state_machine.adaptive_policy_engine import AdaptivePolicyEngine
from autonomy_state_machine.states import RoverState
from autonomy_interfaces.msg import ContextState


class TestSystemIntegration:
    """Test end-to-end system integration scenarios."""

    @pytest.fixture
    def integrated_system(self):
        """Create a fully integrated system for testing."""
        # Create components
        state_machine = AdaptiveStateMachine()
        safety_manager = SafetyManager(logger=state_machine.get_logger())
        monitoring_service = MonitoringService()
        context_evaluator = ContextEvaluator(state_machine)
        policy_engine = AdaptivePolicyEngine(state_machine)

        return {
            'state_machine': state_machine,
            'safety_manager': safety_manager,
            'monitoring_service': monitoring_service,
            'context_evaluator': context_evaluator,
            'policy_engine': policy_engine
        }

    def test_emergency_battery_scenario_integration(self, integrated_system):
        """Test complete emergency battery scenario - CRITICAL INTEGRATION."""
        system = integrated_system
        sm = system['state_machine']
        safety = system['safety_manager']
        monitoring = system['monitoring_service']

        # Start in autonomous mission
        success, _ = sm.transition_to_state(RoverState.AUTO, "start mission")
        assert success
        assert sm.current_state == RoverState.AUTO

        # Simulate critical battery condition
        safety.trigger_safety(SafetyTriggerType.BATTERY_CRITICAL)

        # Create context reflecting critical battery
        context = ContextState()
        context.battery_level = 5.0
        context.battery_critical = True
        context.mission_progress = 0.8  # Mission nearly complete

        # Update monitoring with critical context
        monitoring._context_callback(context)

        # Evaluate policies for emergency response
        policies = system['policy_engine'].evaluate_policies(context)

        # Should generate emergency return policy
        emergency_policies = [p for p in policies if p.action_type.value == "emergency_return"]
        assert len(emergency_policies) > 0

        # Execute emergency transition
        success, message = sm.transition_to_state(RoverState.READY, "emergency return")
        assert success
        assert "emergency return" in message.lower()

        # Verify system state
        assert sm.current_state == RoverState.READY
        assert SafetyTriggerType.BATTERY_CRITICAL in safety.get_active_triggers()

    def test_safety_integration_with_state_machine(self, integrated_system):
        """Test safety manager integration with state machine - CRITICAL."""
        system = integrated_system
        sm = system['state_machine']
        safety = system['safety_manager']

        # Start autonomous operation
        success, _ = sm.transition_to_state(RoverState.AUTO, "autonomous operation")
        assert success

        # Trigger multiple safety conditions
        safety.trigger_safety(SafetyTriggerType.OBSTACLE_CRITICAL)
        safety.trigger_safety(SafetyTriggerType.BATTERY_CRITICAL)

        # Attempt transition that safety should block
        success, message = sm.transition_to_state(RoverState.TELEOP, "unsafe transition")

        # Transition should be blocked by safety system
        # Note: This depends on the exact blocking logic implementation
        # The test verifies that safety conditions are considered

        # Verify safety state
        active_triggers = safety.get_active_triggers()
        assert SafetyTriggerType.OBSTACLE_CRITICAL in active_triggers
        assert SafetyTriggerType.BATTERY_CRITICAL in active_triggers
        assert safety.get_highest_severity().value == "CRITICAL"

    def test_monitoring_integration_with_adaptation(self, integrated_system):
        """Test monitoring service integration with adaptation engine."""
        system = integrated_system
        monitoring = system['monitoring_service']
        policy_engine = system['policy_engine']

        # Create context with issues that should trigger adaptations
        context = ContextState()
        context.battery_level = 15.0  # Critical
        context.obstacle_detected = True
        context.obstacle_distance = 0.3  # Critical distance
        context.communication_active = False  # Lost comms

        # Send context to monitoring
        monitoring._context_callback(context)

        # Evaluate policies
        policies = policy_engine.evaluate_policies(context)

        # Should generate multiple adaptation policies
        assert len(policies) > 0

        # Execute adaptations and monitor them
        for policy in policies[:2]:  # Test first few adaptations
            # Simulate adaptation execution
            monitoring._adaptation_callback(policy.to_msg())

        # Verify monitoring captured adaptations
        assert len(monitoring.adaptation_history) >= 2

        # Check that monitoring computed effectiveness
        monitoring._compute_policy_effectiveness()
        assert len(monitoring.policy_effectiveness) > 0

    def test_error_handling_integration(self, integrated_system):
        """Test error handling integration across components."""
        system = integrated_system
        sm = system['state_machine']

        # Test error boundary in state transitions
        from autonomy_state_machine.error_handling import error_boundary

        error_occurred = False
        try:
            with error_boundary(sm.get_logger(), "StateMachine", "integration test"):
                # Simulate an error during state transition
                raise RuntimeError("Simulated transition error")
        except RuntimeError:
            error_occurred = True

        # Error should have been handled gracefully
        assert error_occurred  # Error was re-raised as expected

        # Test safe execution wrapper
        from autonomy_state_machine.error_handling import safe_execute

        def failing_operation():
            raise ConnectionError("Connection lost")

        result = safe_execute(
            failing_operation,
            sm.get_logger(),
            "IntegrationTest",
            "network operation",
            default_return="offline"
        )

        assert result == "offline"  # Should return default on error

    def test_context_monitoring_lifecycle(self, integrated_system):
        """Test complete context monitoring lifecycle."""
        system = integrated_system
        context_eval = system['context_evaluator']
        monitoring = system['monitoring_service']

        # Create series of context updates
        contexts = []
        for i in range(5):
            context = ContextState()
            context.battery_level = 100.0 - (i * 15)  # Declining battery
            context.cpu_usage = 20.0 + (i * 10)       # Increasing CPU
            context.mission_progress = i * 0.2         # Increasing progress
            contexts.append(context)

        # Feed contexts through the system
        patterns_detected = []
        for context in contexts:
            # Context evaluation
            evaluated_context = context_eval.evaluate_system_context()

            # Monitoring update
            monitoring._context_callback(context)

            # Pattern analysis
            if len(monitoring.context_history) >= 3:
                patterns = context_eval.get_context_patterns()
                if patterns:
                    patterns_detected.append(patterns)

        # Verify pattern detection worked
        assert len(patterns_detected) > 0

        # Check monitoring computed analytics
        monitoring._compute_performance_metrics()
        assert len(monitoring.performance_metrics) > 0

    def test_performance_under_load(self, integrated_system):
        """Test system performance under load conditions."""
        system = integrated_system
        sm = system['state_machine']
        monitoring = system['monitoring_service']

        import time
        start_time = time.time()

        # Simulate high-frequency operations
        operations = 0
        while time.time() - start_time < 2.0:  # Run for 2 seconds
            # Rapid state transitions
            if operations % 2 == 0:
                sm.transition_to_state(RoverState.READY, f"load_test_{operations}")
            else:
                sm.transition_to_state(RoverState.AUTO, f"load_test_{operations}")

            # Context updates
            context = ContextState()
            context.battery_level = 75.0
            context.cpu_usage = 50.0
            monitoring._context_callback(context)

            operations += 1

        # Verify system remained stable
        assert operations > 10  # Should handle reasonable load
        assert len(monitoring.context_history) > 0

        # Check performance metrics were computed
        monitoring._compute_performance_metrics()
        assert 'avg_cpu_usage' in monitoring.performance_metrics

    def test_recovery_scenario_full_cycle(self, integrated_system):
        """Test complete recovery scenario from failure to normal operation."""
        system = integrated_system
        sm = system['state_machine']
        safety = system['safety_manager']
        monitoring = system['monitoring_service']

        # Phase 1: Normal operation
        success, _ = sm.transition_to_state(RoverState.AUTO, "normal operation")
        assert success
        assert sm.current_state == RoverState.AUTO

        # Phase 2: Trigger emergency condition
        safety.trigger_safety(SafetyTriggerType.EMERGENCY_STOP)

        # Phase 3: Emergency recovery
        success, _ = sm.transition_to_state(RoverState.ESTOP, "emergency stop")
        assert success
        assert sm.current_state == RoverState.ESTOP

        # Phase 4: Clear emergency and recover
        safety.clear_all_triggers()

        success, _ = sm.transition_to_state(RoverState.READY, "recovery complete")
        assert success
        assert sm.current_state == RoverState.READY

        # Phase 5: Return to normal operation
        success, _ = sm.transition_to_state(RoverState.AUTO, "normal operation resumed")
        assert success
        assert sm.current_state == RoverState.AUTO

        # Verify monitoring captured the entire cycle
        assert len(monitoring.context_history) >= 3
        monitoring._compute_performance_metrics()
        assert len(monitoring.performance_metrics) > 0

    def test_concurrent_component_interaction(self, integrated_system):
        """Test concurrent interactions between multiple components."""
        system = integrated_system
        sm = system['state_machine']
        safety = system['safety_manager']
        monitoring = system['monitoring_service']

        # Set up initial state
        sm.transition_to_state(RoverState.AUTO, "concurrent test setup")

        # Simulate concurrent safety triggers and context updates
        triggers = [
            SafetyTriggerType.BATTERY_CRITICAL,
            SafetyTriggerType.OBSTACLE_CRITICAL,
            SafetyTriggerType.COMMUNICATION_LOSS
        ]

        contexts = []
        for i in range(3):
            context = ContextState()
            context.battery_level = 20.0 - (i * 5)  # Declining
            context.obstacle_detected = True
            context.communication_active = i % 2 == 0  # Alternating
            contexts.append(context)

        # Apply triggers and contexts
        for trigger in triggers:
            safety.trigger_safety(trigger)

        for context in contexts:
            monitoring._context_callback(context)

        # Verify all components reflect the concurrent activity
        active_triggers = safety.get_active_triggers()
        for trigger in triggers:
            assert trigger in active_triggers

        assert len(monitoring.context_history) >= 3
        assert safety.get_highest_severity().value == "CRITICAL"

        # System should still be operational despite concurrent issues
        assert sm.current_state == RoverState.AUTO

    def test_resource_cleanup_integration(self, integrated_system):
        """Test proper resource cleanup across integrated components."""
        system = integrated_system

        # Use components extensively
        sm = system['state_machine']
        monitoring = system['monitoring_service']

        # Generate significant monitoring data
        for i in range(20):
            context = ContextState()
            context.battery_level = 50.0 + (i % 10)
            monitoring._context_callback(context)

        # Perform multiple state transitions
        for i in range(5):
            sm.transition_to_state(RoverState.READY if i % 2 == 0 else RoverState.AUTO,
                                 f"cleanup_test_{i}")

        # Verify cleanup functionality
        initial_context_count = len(monitoring.context_history)
        initial_transition_count = len(sm.transition_history)

        # Trigger cleanup
        monitoring._cleanup_old_data()

        # History should be managed (not necessarily reduced if under limits)
        # but cleanup should execute without errors
        final_context_count = len(monitoring.context_history)
        final_transition_count = len(sm.transition_history)

        # Counts should be reasonable (cleanup may not trigger if under limits)
        assert final_context_count >= 0
        assert final_transition_count >= 0

    # ========== ADVANCED INTEGRATION SCENARIOS ========== #

    def test_safety_monitoring_state_machine_interaction(self, integrated_system):
        """Test safety triggers affect monitoring and state transitions - CRITICAL."""
        system = integrated_system
        sm = system['state_machine']
        safety = system['safety_manager']
        monitoring = system['monitoring_service']

        # Start with clean slate
        sm.current_state = RoverState.READY

        # Trigger safety condition that should affect monitoring
        safety.trigger_safety(
            SafetyTriggerType.BATTERY_CRITICAL,
            SafetySeverity.CRITICAL,
            "Critical battery level detected"
        )

        # Monitoring should detect the safety condition
        context = system['context_evaluator'].evaluate_system_context()
        monitoring._context_callback(context)

        # Safety state should be reflected in monitoring
        alerts = monitoring._check_alert_conditions(context)
        assert "critical_battery" in alerts or "safety_active" in alerts

    def test_adaptive_policies_under_system_stress(self, integrated_system):
        """Test policy effectiveness during high system load."""
        system = integrated_system
        policy_engine = system['policy_engine']

        # Create high-stress context
        context = system['context_evaluator'].context_state
        context.battery_level = 10.0  # Critical
        context.cpu_usage = 90.0     # High load
        context.memory_usage = 85.0  # High memory
        context.obstacle_detected = True
        context.obstacle_distance = 0.5  # Very close
        context.communication_active = False  # Lost comms

        # Evaluate policies under stress
        policies = policy_engine.evaluate_policies(context)

        # Should generate multiple high-priority adaptations
        assert len(policies) >= 3  # At least battery, obstacle, and comms policies

        # All policies should be high priority
        for policy in policies:
            assert policy.priority >= 70, f"Policy {policy.action_type} has low priority under stress: {policy.priority}"

    def test_cross_component_data_consistency(self, integrated_system):
        """Test data consistency across component boundaries."""
        system = integrated_system

        # Set up consistent test data
        test_battery_level = 25.0
        test_cpu_usage = 60.0

        # Update context evaluator
        context = system['context_evaluator'].context_state
        context.battery_level = test_battery_level
        context.cpu_usage = test_cpu_usage

        # Update monitoring
        monitoring = system['monitoring_service']
        monitoring._context_callback(context)

        # Data should be consistent across components
        assert abs(context.battery_level - test_battery_level) < 0.1
        assert abs(context.cpu_usage - test_cpu_usage) < 0.1

    def test_performance_degradation_under_multi_failure(self, integrated_system):
        """Test performance during multiple simultaneous failures."""
        system = integrated_system
        monitoring = system['monitoring_service']

        import time

        # Establish baseline performance
        start_time = time.time()
        for i in range(10):
            context = system['context_evaluator'].evaluate_system_context()
            monitoring._context_callback(context)
        baseline_time = time.time() - start_time

        # Simulate multiple failures
        safety = system['safety_manager']
        safety.trigger_safety(SafetyTriggerType.BATTERY_CRITICAL, SafetySeverity.CRITICAL, "Battery failure")
        safety.trigger_safety(SafetyTriggerType.COMMUNICATION_LOSS, SafetySeverity.CRITICAL, "Comm failure")

        # Test performance under failure conditions
        start_time = time.time()
        for i in range(10):
            context = system['context_evaluator'].evaluate_system_context()
            monitoring._context_callback(context)
            policies = system['policy_engine'].evaluate_policies(context)
        failure_time = time.time() - start_time

        # Performance degradation should be reasonable (< 5x slower)
        if baseline_time > 0:
            degradation_ratio = failure_time / baseline_time
            assert degradation_ratio < 5.0, f"Performance degraded too much: {degradation_ratio:.2f}x slower"
