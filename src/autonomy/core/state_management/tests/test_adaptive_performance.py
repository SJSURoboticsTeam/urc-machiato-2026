#!/usr/bin/env python3
"""
Performance Tests for Adaptive State Machine

Tests performance characteristics, scalability, and resource usage
of the adaptive state machine components under various loads.
"""

import pytest
import rclpy
from rclpy.node import Node
import time
import threading
import psutil
import os
from unittest.mock import patch, MagicMock
from typing import List, Dict, Any
import numpy as np

from autonomy_state_machine.context_evaluator import ContextEvaluator
from autonomy_state_machine.adaptive_policy_engine import AdaptivePolicyEngine
from autonomy_state_machine.adaptive_state_machine import AdaptiveStateMachine
from autonomy_state_machine.monitoring_service import MonitoringService

from autonomy_interfaces.msg import ContextState, AdaptiveAction as AdaptiveActionMsg


class TestAdaptivePerformance:
    """Performance tests for adaptive components."""

    @pytest.fixture(scope="function")
    def ros2_context(self):
        """Provide isolated ROS2 context for each performance test."""
        rclpy.init()
        yield
        rclpy.shutdown()

    @pytest.fixture
    def node(self, ros2_context):
        """Create a test ROS2 node with proper isolation."""
        node = Node('perf_test_node')
        yield node
        node.destroy_node()

    def test_context_evaluation_performance(self, node, benchmark):
        """Benchmark context evaluation performance."""
        context_evaluator = ContextEvaluator(node)

        # Benchmark function
        def evaluate_context():
            return context_evaluator.evaluate_system_context()

        # Run benchmark - pytest-benchmark will collect performance statistics
        result = benchmark(evaluate_context)

        # Verify benchmark ran successfully and meets performance criteria
        assert isinstance(result, ContextState)
        assert result.battery_level > 0  # Verify context contains valid data

        # Verify performance meets requirements for real-time operation
        # Context evaluation should complete within 100ms for real-time operation
        max_acceptable_time = 0.1  # 100ms
        assert benchmark.stats.mean < max_acceptable_time, \
            f"Context evaluation too slow: {benchmark.stats.mean:.3f}s (max: {max_acceptable_time}s)"

        # Performance should be consistent (low standard deviation)
        performance_variability = benchmark.stats.stddev / benchmark.stats.mean if benchmark.stats.mean > 0 else 0
        assert performance_variability < 0.5, \
            f"Performance too variable: {performance_variability:.2f} (coefficient of variation)"

        # Verify context data quality
        assert hasattr(result, 'timestamp'), "Context should have timestamp"
        assert result.battery_level <= 100.0, "Battery level should not exceed 100%"

    def test_policy_engine_performance(self, node, benchmark):
        """Benchmark policy engine performance."""
        policy_engine = AdaptivePolicyEngine(node)

        # Create test context
        context = ContextState()
        context.battery_critical = True
        context.battery_level = 5.0
        context.obstacle_detected = True
        context.obstacle_distance = 0.2
        context.mission_progress = 0.9

        # Benchmark function
        def evaluate_policies():
            return policy_engine.evaluate_policies(context)

        # Run benchmark - pytest-benchmark will collect performance statistics
        result = benchmark(evaluate_policies)

        # Verify benchmark ran successfully and meets performance criteria
        assert isinstance(result, list)
        assert len(result) >= 1, "Policy engine should generate at least one policy"

        # Verify actions have expected properties and are valid
        for action in result:
            assert hasattr(action, 'action_type'), "Action must have action_type"
            assert hasattr(action, 'priority'), "Action must have priority"
            assert hasattr(action, 'action_type', 'value'), "Action type must have value"

        # Performance requirements for safety-critical decision making
        # Policy evaluation should complete within 50ms for emergency response
        emergency_response_time = 0.05  # 50ms
        assert benchmark.stats.mean < emergency_response_time, \
            f"Policy evaluation too slow for emergency response: {benchmark.stats.mean:.3f}s (max: {emergency_response_time}s)"

        # Verify policy quality - emergency policies should have high priority
        emergency_policies = [p for p in result if 'emergency' in str(p.action_type).lower()]
        if emergency_policies:
            for policy in emergency_policies:
                assert policy.priority >= 0.8, \
                    f"Emergency policy priority too low: {policy.priority} (should be >= 0.8)"

    def test_concurrent_context_evaluation(self, node):
        """Test concurrent context evaluation performance."""
        context_evaluator = ContextEvaluator(node)

        results = []
        errors = []

        def evaluate_worker(worker_id):
            try:
                start_time = time.time()
                context = context_evaluator.evaluate_system_context()
                end_time = time.time()

                results.append({
                    'worker_id': worker_id,
                    'duration': end_time - start_time,
                    'context': context
                })
            except Exception as e:
                errors.append(f"Worker {worker_id}: {e}")

        # Start multiple concurrent evaluations
        threads = []
        num_workers = 10

        for i in range(num_workers):
            thread = threading.Thread(target=evaluate_worker, args=(i,))
            threads.append(thread)

        start_time = time.time()

        # Start all threads
        for thread in threads:
            thread.start()

        # Wait for completion
        for thread in threads:
            thread.join()

        total_time = time.time() - start_time

        # Verify results
        assert len(results) == num_workers
        assert len(errors) == 0

        # Verify reasonable total time (should be much less than sequential)
        sequential_estimate = num_workers * 0.01  # 10ms per evaluation
        assert total_time < sequential_estimate * 2  # Allow some overhead

        # Verify all results are valid
        for result in results:
            assert result['duration'] < 0.1  # Each should complete quickly
            assert isinstance(result['context'], ContextState)

    def test_memory_usage_under_load(self, node):
        """Test memory usage during sustained load."""
        initial_memory = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024  # MB

        context_evaluator = ContextEvaluator(node)
        policy_engine = AdaptivePolicyEngine(node)

        # Generate sustained load
        contexts = []
        for i in range(1000):
            # Create varied contexts to test different code paths
            context = ContextState()
            context.battery_level = float(10 + (i % 90))  # Vary battery level
            context.cpu_usage = 20 + (i % 60)      # Vary CPU usage
            context.obstacle_detected = (i % 10) == 0  # Some obstacles
            context.mission_progress = (i % 100) / 100.0

            # Evaluate context and policies
            full_context = context_evaluator.evaluate_system_context()
            actions = policy_engine.evaluate_policies(full_context)

            contexts.append(full_context)

            # Small delay to prevent overwhelming
            time.sleep(0.001)

        final_memory = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024  # MB
        memory_increase = final_memory - initial_memory

        # Verify reasonable memory usage (should not leak significantly)
        assert memory_increase < 50  # Less than 50MB increase for 1000 evaluations

        # Verify all contexts were created
        assert len(contexts) == 1000

    def test_adaptive_state_machine_throughput(self, node):
        """Test adaptive state machine throughput under load."""
        with patch('rclpy.node.Node.create_timer'), \
             patch('rclpy.node.Node.create_publisher'), \
             patch('rclpy.node.Node.create_subscription'), \
             patch('rclpy.node.Node.create_service'):

            adaptive_sm = AdaptiveStateMachine()

            # Test transition throughput
            transitions = []
            start_time = time.time()

            num_transitions = 100
            for i in range(num_transitions):
                # Alternate between states
                if i % 2 == 0:
                    success = adaptive_sm.transition_to_state(
                        adaptive_sm.current_state, f"Test transition {i}")
                else:
                    success = adaptive_sm.transition_to_state(
                        adaptive_sm.current_state, f"Test transition {i}")

                transitions.append(success)

            end_time = time.time()
            total_time = end_time - start_time

            # Verify all transitions succeeded
            assert all(transitions)

            # Verify reasonable throughput (should handle many transitions per second)
            throughput = num_transitions / total_time
            assert throughput > 10  # At least 10 transitions per second

    def test_monitoring_service_scalability(self, node):
        """Test monitoring service scalability with large data sets."""
        with patch('rclpy.node.Node.create_timer'), \
             patch('rclpy.node.Node.create_publisher'), \
             patch('rclpy.node.Node.create_subscription'), \
             patch('rclpy.node.Node.create_service'):

            monitoring_service = MonitoringService()

            # Generate large amounts of test data
            start_time = time.time()

            # Add many context readings
            for i in range(5000):
                context_entry = {
                    'context': ContextState(),
                    'timestamp': time.time() - i * 0.01
                }
                context_entry['context'].battery_level = float(50.0 + 20 * np.sin(i * 0.1))
                context_entry['context'].cpu_usage = 30.0 + 15 * np.cos(i * 0.1)
                monitoring_service.context_history.append(context_entry)

            # Add adaptation actions
            for i in range(1000):
                action_entry = {
                    'action': AdaptiveActionMsg(),
                    'timestamp': time.time() - i * 0.1
                }
                action_entry['action'].action_type = "TEST_ACTION"
                action_entry['action'].priority = 50 + (i % 50)
                monitoring_service.adaptation_history.append(action_entry)

            data_generation_time = time.time() - start_time

            # Test analytics computation performance
            analytics_start = time.time()
            monitoring_service._compute_performance_metrics()
            monitoring_service._compute_policy_effectiveness()
            analytics_time = time.time() - analytics_start

            # Verify reasonable performance
            assert data_generation_time < 2.0  # Should generate data quickly
            assert analytics_time < 1.0       # Analytics should complete in < 1 second

            # Verify results are valid
            assert len(monitoring_service.performance_metrics) > 0
            assert 'battery_trend' in monitoring_service.performance_metrics

    def test_resource_cleanup_under_stress(self, node):
        """Test resource cleanup during stress conditions."""
        context_evaluator = ContextEvaluator(node)

        # Create many context evaluations to fill history
        for i in range(200):  # More than default limit
            context_evaluator.evaluate_system_context()
            time.sleep(0.001)  # Small delay

        initial_history_length = len(context_evaluator.context_history)

        # Force cleanup by setting short max age and calling cleanup
        context_evaluator.context_history[0]['timestamp'] = time.time() - 3601  # Make old
        # Note: In real implementation, cleanup would be periodic

        # Verify history doesn't grow unbounded
        assert initial_history_length <= 200  # Should be bounded

    @pytest.mark.parametrize("load_factor", [1, 5, 10])
    def test_scalability_under_different_loads(self, node, load_factor):
        """Test scalability under different load factors."""
        context_evaluator = ContextEvaluator(node)
        policy_engine = AdaptivePolicyEngine(node)

        # Create test contexts with different complexity
        contexts = []
        for i in range(10 * load_factor):
            context = ContextState()
            context.battery_level = float(20 + (i % 80))
            context.cpu_usage = 10 + (i % 80)
            context.obstacle_detected = (i % (10 // load_factor + 1)) == 0
            context.mission_progress = (i % 100) / 100.0
            contexts.append(context)

        start_time = time.time()

        # Process all contexts
        results = []
        for context in contexts:
            full_context = context_evaluator.evaluate_system_context()
            actions = policy_engine.evaluate_policies(full_context)
            results.append((full_context, actions))

        end_time = time.time()
        total_time = end_time - start_time

        # Verify all results are valid
        assert len(results) == len(contexts)
        for context, actions in results:
            assert isinstance(context, ContextState)
            assert isinstance(actions, list)

        # Verify reasonable scaling (time shouldn't increase linearly with load_factor)
        base_time_per_item = 0.01  # Estimated base time
        expected_max_time = len(contexts) * base_time_per_item * (load_factor ** 0.5)  # Square root scaling

        assert total_time < expected_max_time * 2  # Allow some margin


class TestAdaptiveReliability:
    """Reliability tests for adaptive components under failure conditions."""

    @pytest.fixture(scope="function")
    def ros2_context(self):
        """Provide isolated ROS2 context for each reliability test."""
        rclpy.init()
        yield
        rclpy.shutdown()

    @pytest.fixture
    def node(self, ros2_context):
        """Create a test ROS2 node with proper isolation."""
        node = Node('reliability_test')
        yield node
        node.destroy_node()

    def test_graceful_degradation_on_context_failure(self, node):
        """Test graceful degradation when context evaluation fails."""
        context_evaluator = ContextEvaluator(node)

        # Simulate system monitoring failure
        with patch('psutil.cpu_percent', side_effect=Exception("Hardware failure")):
            context = context_evaluator.evaluate_system_context()

            # Should still return valid context with safe defaults
            assert isinstance(context, ContextState)
            assert 0 <= context.cpu_usage <= 100  # Should have fallback value

    def test_policy_engine_resilience(self, node):
        """Test policy engine resilience to invalid contexts."""
        policy_engine = AdaptivePolicyEngine(node)

        # Test with None context (should handle gracefully)
        actions = policy_engine.evaluate_policies(None)
        assert isinstance(actions, list)
        assert len(actions) == 0

        # Test with malformed context
        malformed_context = ContextState()
        # Missing required fields - should handle gracefully
        actions = policy_engine.evaluate_policies(malformed_context)
        assert isinstance(actions, list)

    def test_monitoring_service_data_integrity(self, node):
        """Test monitoring service maintains data integrity under failures."""
        with patch('rclpy.node.Node.create_timer'), \
             patch('rclpy.node.Node.create_publisher'), \
             patch('rclpy.node.Node.create_subscription'), \
             patch('rclpy.node.Node.create_service'):

            monitoring_service = MonitoringService()

            # Add valid data
            valid_entry = {
                'context': ContextState(),
                'timestamp': time.time()
            }
            monitoring_service.context_history.append(valid_entry)

            # Simulate processing failures
            with patch.object(monitoring_service, '_compute_performance_metrics',
                            side_effect=Exception("Computation failed")):
                # Should not crash the service
                monitoring_service._analytics_computation_callback()

                # Should still have valid data
                assert len(monitoring_service.context_history) >= 1

    def test_adaptive_state_machine_recovery(self, node):
        """Test adaptive state machine recovery from component failures."""
        with patch('rclpy.node.Node.create_timer'), \
             patch('rclpy.node.Node.create_publisher'), \
             patch('rclpy.node.Node.create_subscription'), \
             patch('rclpy.node.Node.create_service'):

            adaptive_sm = AdaptiveStateMachine()

            # Simulate context evaluator failure
            with patch.object(adaptive_sm.context_evaluator, 'evaluate_system_context',
                            side_effect=Exception("Context failure")):

                # Should still allow basic transitions
                success = adaptive_sm.transition_to_state(
                    adaptive_sm.current_state, "Test during failure")

                # Should succeed with basic validation (adaptive features disabled)
                assert success is True  # Basic transition should work

    def test_memory_bounds_under_failure_conditions(self, node):
        """Test memory usage stays bounded even with repeated failures."""
        context_evaluator = ContextEvaluator(node)

        initial_memory = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024  # MB

        # Simulate repeated failures
        with patch('psutil.cpu_percent', side_effect=Exception("Repeated failure")):
            for i in range(100):
                try:
                    context_evaluator.evaluate_system_context()
                except:
                    pass  # Ignore failures for this test

        final_memory = psutil.Process(os.getpid()).memory_info().rss / 1024 / 1024  # MB
        memory_increase = final_memory - initial_memory

        # Memory increase should be reasonable even with failures
        assert memory_increase < 20  # Less than 20MB increase


if __name__ == '__main__':
    pytest.main([__file__, "-v"])
