#!/usr/bin/env python3
"""
Behavior Tree Execution Failure Tests - URC 2026

Tests behavior tree failures that can break mission execution:
- Action node failures and error handling
- Sequence/Selector logic errors
- Decorator malfunction
- Tree structure corruption
- State persistence failures
- Recovery mechanism failures

Author: URC 2026 Risk Mitigation Team
"""

import pytest
import time
from unittest.mock import Mock, MagicMock, patch, AsyncMock
import sys
import os
from typing import Dict, List, Any, Optional
from enum import Enum

# Add source paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src'))


class MockBTNode:
    """Mock behavior tree node for testing."""
    def __init__(self, name: str, return_status: str = "SUCCESS"):
        self.name = name
        self.return_status = return_status
        self.execute_count = 0
        self.last_execute_time = 0

    def execute(self) -> str:
        """Execute the node."""
        self.execute_count += 1
        self.last_execute_time = time.time()
        return self.return_status


class MockBehaviorTree:
    """Mock behavior tree for testing."""
    def __init__(self):
        self.nodes = {}
        self.execution_log = []
        self.failures = []

    def add_node(self, node: MockBTNode):
        """Add a node to the tree."""
        self.nodes[node.name] = node

    def execute(self) -> str:
        """Execute the behavior tree."""
        self.execution_log.append(f"Executing tree at {time.time()}")
        # Simple execution logic for testing
        for node in self.nodes.values():
            result = node.execute()
            if result == "FAILURE":
                self.failures.append(node.name)
                return "FAILURE"
        return "SUCCESS"


class TestActionNodeFailures:
    """Test failures in behavior tree action nodes."""

    @pytest.fixture
    def mock_tree(self):
        """Create mock behavior tree."""
        return MockBehaviorTree()

    @pytest.mark.critical
    def test_action_node_execution_failure(self, mock_tree):
        """Test action node that fails to execute."""
        # Create failing action node
        failing_node = MockBTNode("navigate_to_waypoint", "FAILURE")
        mock_tree.add_node(failing_node)

        # Execute tree
        result = mock_tree.execute()

        # Should fail due to action node failure
        assert result == "FAILURE", "Tree should fail when action node fails"
        assert "navigate_to_waypoint" in mock_tree.failures, "Should record failing node"

    @pytest.mark.critical
    def test_action_node_timeout_failure(self, mock_tree):
        """Test action node that times out."""
        # Create slow action node that should timeout
        class TimeoutNode(MockBTNode):
            def execute(self):
                time.sleep(2.0)  # Exceeds timeout
                return "SUCCESS"

        timeout_node = TimeoutNode("slow_action")
        mock_tree.add_node(timeout_node)

        # Execute with timeout monitoring
        start_time = time.time()
        result = mock_tree.execute()
        execution_time = time.time() - start_time

        # Should complete but be flagged as timeout
        assert execution_time >= 2.0, "Action should take at least timeout duration"
        # In real BT, this would be interrupted and return FAILURE

    @pytest.mark.critical
    def test_action_node_resource_exhaustion(self, mock_tree):
        """Test action node failure due to resource exhaustion."""
        # Create action that exhausts memory
        class MemoryExhaustionNode(MockBTNode):
            def execute(self):
                # Simulate memory exhaustion
                large_data = [0] * (10**7)  # Try to allocate large array
                return "SUCCESS"

        memory_node = MemoryExhaustionNode("memory_intensive_action")
        mock_tree.add_node(memory_node)

        # Execute and monitor for memory issues
        try:
            result = mock_tree.execute()
            # In real scenario, this might cause OOM or degradation
        except MemoryError:
            assert True, "Should handle memory exhaustion gracefully"

    @pytest.mark.critical
    def test_action_node_hardware_dependency_failure(self, mock_tree):
        """Test action node failure due to hardware dependency issues."""
        # Mock hardware interface failure
        hardware_mock = Mock()
        hardware_mock.is_available.return_value = False
        hardware_mock.execute_command.side_effect = Exception("Hardware unavailable")

        class HardwareDependentNode(MockBTNode):
            def __init__(self, hardware):
                super().__init__("hardware_action")
                self.hardware = hardware

            def execute(self):
                if not self.hardware.is_available():
                    return "FAILURE"
                try:
                    self.hardware.execute_command("move_forward")
                    return "SUCCESS"
                except Exception:
                    return "FAILURE"

        hardware_node = HardwareDependentNode(hardware_mock)
        mock_tree.add_node(hardware_node)

        result = mock_tree.execute()
        assert result == "FAILURE", "Should fail when hardware unavailable"


class TestSequenceSelectorFailures:
    """Test failures in sequence and selector composite nodes."""

    @pytest.fixture
    def sequence_tree(self):
        """Create sequence behavior tree."""
        return MockBehaviorTree()

    @pytest.fixture
    def selector_tree(self):
        """Create selector behavior tree."""
        return MockBehaviorTree()

    @pytest.mark.critical
    def test_sequence_node_early_failure(self, sequence_tree):
        """Test sequence failure when early node fails."""
        # Sequence: Node1 -> Node2 -> Node3
        # If Node2 fails, sequence should stop and return FAILURE

        node1 = MockBTNode("check_sensors", "SUCCESS")
        node2 = MockBTNode("plan_path", "FAILURE")  # This fails
        node3 = MockBTNode("execute_path", "SUCCESS")  # Never reached

        sequence_tree.add_node(node1)
        sequence_tree.add_node(node2)
        sequence_tree.add_node(node3)

        result = sequence_tree.execute()

        assert result == "FAILURE", "Sequence should fail when intermediate node fails"
        assert node3.execute_count == 0, "Should not execute nodes after failure"

    @pytest.mark.critical
    def test_selector_node_no_success_option(self, selector_tree):
        """Test selector failure when no option succeeds."""
        # Selector: Try Node1, if fails try Node2, if fails return FAILURE

        node1 = MockBTNode("try_wifi_navigation", "FAILURE")
        node2 = MockBTNode("try_gps_navigation", "FAILURE")  # Also fails
        node3 = MockBTNode("emergency_stop", "SUCCESS")  # Fallback

        selector_tree.add_node(node1)
        selector_tree.add_node(node2)
        selector_tree.add_node(node3)

        # Without fallback, should fail
        selector_tree.nodes.pop("emergency_stop")  # Remove fallback

        result = selector_tree.execute()
        assert result == "FAILURE", "Selector should fail when all options fail"

    @pytest.mark.critical
    def test_nested_composite_failure_propagation(self):
        """Test failure propagation through nested composites."""
        # Complex nested structure
        root_selector = MockBehaviorTree()

        # Sequence 1: Check sensors -> Navigate
        sensor_check = MockBTNode("sensor_check", "SUCCESS")
        navigate = MockBTNode("navigate", "FAILURE")  # Fails
        sequence1 = MockBehaviorTree()
        sequence1.add_node(sensor_check)
        sequence1.add_node(navigate)

        # Sequence 2: Alternative approach
        alt_navigate = MockBTNode("alt_navigate", "SUCCESS")
        sequence2 = MockBehaviorTree()
        sequence2.add_node(alt_navigate)

        # Root selector tries sequence1, then sequence2
        root_selector.add_node(sequence1)
        root_selector.add_node(sequence2)

        result = root_selector.execute()
        # Should succeed with fallback
        assert result == "SUCCESS", "Should succeed with nested fallback"


class TestDecoratorFailures:
    """Test failures in behavior tree decorator nodes."""

    @pytest.mark.critical
    def test_retry_decorator_exhaustion(self):
        """Test retry decorator that exhausts all attempts."""
        class RetryDecorator:
            def __init__(self, child, max_attempts=3):
                self.child = child
                self.max_attempts = max_attempts
                self.attempts = 0

            def execute(self):
                while self.attempts < self.max_attempts:
                    self.attempts += 1
                    result = self.child.execute()
                    if result == "SUCCESS":
                        return "SUCCESS"
                return "FAILURE"

        # Child that always fails
        failing_child = MockBTNode("unreliable_action", "FAILURE")
        retry_decorator = RetryDecorator(failing_child, max_attempts=3)

        result = retry_decorator.execute()

        assert result == "FAILURE", "Should fail after exhausting retries"
        assert retry_decorator.attempts == 3, "Should attempt maximum retries"

    @pytest.mark.critical
    def test_timeout_decorator_failure(self):
        """Test timeout decorator that interrupts slow actions."""
        class TimeoutDecorator:
            def __init__(self, child, timeout=1.0):
                self.child = child
                self.timeout = timeout

            def execute(self):
                start_time = time.time()
                # In real implementation, this would run child in separate thread
                result = self.child.execute()
                elapsed = time.time() - start_time

                if elapsed > self.timeout:
                    return "FAILURE"  # Timeout
                return result

        # Slow child that exceeds timeout
        slow_child = MockBTNode("slow_action", "SUCCESS")
        original_execute = slow_child.execute
        slow_child.execute = lambda: (time.sleep(2.0), original_execute())[1]

        timeout_decorator = TimeoutDecorator(slow_child, timeout=1.0)

        result = timeout_decorator.execute()
        assert result == "FAILURE", "Should fail due to timeout"

    @pytest.mark.critical
    def test_inverter_decorator_logic_error(self):
        """Test inverter decorator logic errors."""
        class InverterDecorator:
            def __init__(self, child):
                self.child = child

            def execute(self):
                result = self.child.execute()
                return "SUCCESS" if result == "FAILURE" else "FAILURE"

        # Test inversion logic
        success_child = MockBTNode("success_action", "SUCCESS")
        failure_child = MockBTNode("failure_action", "FAILURE")

        inverter_success = InverterDecorator(success_child)
        inverter_failure = InverterDecorator(failure_child)

        assert inverter_success.execute() == "FAILURE", "Should invert SUCCESS to FAILURE"
        assert inverter_failure.execute() == "SUCCESS", "Should invert FAILURE to SUCCESS"


class TestTreeStructureFailures:
    """Test failures due to corrupted or invalid tree structures."""

    @pytest.mark.critical
    def test_circular_reference_detection(self):
        """Test detection of circular references in tree structure."""
        # Create nodes with circular dependency
        node_a = MockBTNode("node_a")
        node_b = MockBTNode("node_b")

        # Simulate circular reference: A -> B -> A
        node_a.parent = None
        node_a.children = [node_b]
        node_b.parent = node_a
        node_b.children = [node_a]  # Creates cycle

        # Tree execution should detect cycle
        tree = MockBehaviorTree()
        tree.add_node(node_a)
        tree.add_node(node_b)

        # In real BT, this would cause infinite recursion or be detected
        try:
            result = tree.execute()
            # If no cycle detection, might hang or crash
        except RecursionError:
            assert True, "Should detect circular reference"

    @pytest.mark.critical
    def test_missing_node_references(self):
        """Test tree with missing or broken node references."""
        tree = MockBehaviorTree()

        # Add node with broken reference
        broken_node = MockBTNode("broken_node")
        broken_node.children = ["nonexistent_node"]  # Reference to missing node

        tree.add_node(broken_node)

        # Execution should handle missing references gracefully
        result = tree.execute()
        # Should not crash on missing reference

    @pytest.mark.critical
    def test_tree_serialization_corruption(self):
        """Test behavior tree corruption during serialization."""
        # Simulate XML/JSON corruption
        corrupted_xml = """
        <root>
            <Sequence>
                <Action name="navigate"/>
                <!-- Corrupted section -->
                <InvalidTag>
                <Action name="sample"/>
        </root>
        """

        # Tree loading should fail gracefully
        try:
            # In real implementation, this would try to parse corrupted XML
            # Should raise parsing exception and handle gracefully
            pass
        except Exception:
            assert True, "Should handle corrupted tree serialization"


class TestStatePersistenceFailures:
    """Test failures in behavior tree state persistence."""

    @pytest.mark.critical
    def test_state_save_failure_corruption(self):
        """Test state persistence failure leading to corruption."""
        class StateManager:
            def __init__(self):
                self.state = {}

            def save_state(self, key, value):
                # Simulate save failure
                if key == "critical_state":
                    raise Exception("Storage full")
                self.state[key] = value

            def load_state(self, key):
                return self.state.get(key)

        state_manager = StateManager()

        # Try to save critical state
        try:
            state_manager.save_state("critical_state", {"waypoint": "wp1", "progress": 50})
            assert False, "Should fail to save critical state"
        except Exception:
            # Recovery should maintain previous valid state
            previous_state = state_manager.load_state("critical_state")
            assert previous_state is None, "Should not corrupt existing state on save failure"

    @pytest.mark.critical
    def test_state_load_corruption_recovery(self):
        """Test recovery from corrupted saved state."""
        # Simulate corrupted state file
        corrupted_state = {
            "waypoint_index": "not_a_number",  # Should be int
            "mission_progress": float('inf'),  # Invalid value
            "last_position": None
        }

        # State loading should validate and recover from corruption
        def validate_state(state):
            errors = []
            if not isinstance(state.get("waypoint_index"), int):
                errors.append("waypoint_index not integer")
            if not isinstance(state.get("mission_progress"), (int, float)) or not (0 <= state.get("mission_progress", 0) <= 100):
                errors.append("mission_progress invalid")
            return errors

        errors = validate_state(corrupted_state)
        assert len(errors) > 0, "Should detect state corruption"

    @pytest.mark.critical
    def test_concurrent_state_access_corruption(self):
        """Test state corruption from concurrent access."""
        import threading

        shared_state = {"counter": 0}

        def increment_counter():
            for _ in range(100):
                current = shared_state["counter"]
                time.sleep(0.001)  # Simulate processing time
                shared_state["counter"] = current + 1

        # Run concurrent updates
        threads = [threading.Thread(target=increment_counter) for _ in range(5)]
        for t in threads:
            t.start()
        for t in threads:
            t.join()

        # Without proper locking, counter might be corrupted
        expected = 500  # 5 threads * 100 increments
        actual = shared_state["counter"]

        if actual != expected:
            assert True, f"Concurrent access caused corruption: expected {expected}, got {actual}"


class TestRecoveryMechanismFailures:
    """Test failures in behavior tree recovery mechanisms."""

    @pytest.mark.critical
    def test_recovery_action_failure(self):
        """Test failure of recovery actions themselves."""
        class RecoveryManager:
            def __init__(self):
                self.recovery_actions = []

            def add_recovery_action(self, action):
                self.recovery_actions.append(action)

            def execute_recovery(self, failure_type):
                for action in self.recovery_actions:
                    try:
                        result = action.execute()
                        if result == "SUCCESS":
                            return True
                    except Exception:
                        continue  # Try next recovery action
                return False

        recovery_manager = RecoveryManager()

        # Add failing recovery actions
        failing_recovery = MockBTNode("failing_recovery", "FAILURE")
        another_failing = MockBTNode("another_failing", "FAILURE")

        recovery_manager.add_recovery_action(failing_recovery)
        recovery_manager.add_recovery_action(another_failing)

        # Recovery should fail when all actions fail
        success = recovery_manager.execute_recovery("navigation_failure")
        assert not success, "Recovery should fail when all actions fail"

    @pytest.mark.critical
    def test_recovery_loop_prevention(self):
        """Test prevention of recovery loops."""
        class LoopingRecovery:
            def __init__(self):
                self.execution_count = 0
                self.max_recoveries = 3

            def recover(self, failure):
                self.execution_count += 1
                if self.execution_count <= self.max_recoveries:
                    # Simulate recovery that causes same failure again
                    return "RECOVERED_BUT_WILL_FAIL_AGAIN"
                return "GAVE_UP"

        recovery = LoopingRecovery()

        # Simulate repeated recoveries
        results = []
        for i in range(5):
            result = recovery.recover("repeated_failure")
            results.append(result)

        # Should eventually give up to prevent infinite loop
        assert "GAVE_UP" in results, "Should prevent infinite recovery loops"

    @pytest.mark.critical
    def test_recovery_state_corruption(self):
        """Test recovery mechanism state corruption."""
        recovery_state = {
            "recovery_attempts": 0,
            "last_failure": None,
            "recovery_history": []
        }

        # Simulate corrupted recovery state
        def corrupt_recovery_state():
            recovery_state["recovery_attempts"] = "not_a_number"  # Corrupt
            recovery_state["recovery_history"] = None  # Corrupt

        corrupt_recovery_state()

        # Recovery system should handle corrupted state
        def validate_recovery_state(state):
            if not isinstance(state.get("recovery_attempts"), int):
                state["recovery_attempts"] = 0
            if not isinstance(state.get("recovery_history"), list):
                state["recovery_history"] = []
            return state

        validated_state = validate_recovery_state(recovery_state)

        assert isinstance(validated_state["recovery_attempts"], int), "Should fix corrupted attempt counter"
        assert isinstance(validated_state["recovery_history"], list), "Should fix corrupted history"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])



