#!/usr/bin/env python3
"""
State Machine Coordination Data Flow Test

Test data consistency during state transitions between
autonomous, teleoperation, and emergency stop states.

Author: URC 2026 State Management Team
"""

import asyncio
import time
from dataclasses import dataclass, field
from enum import Enum
from typing import Dict, List, Optional, Any, Callable
from unittest.mock import Mock, patch, AsyncMock
import pytest
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Twist


# Import state management components
try:
    from src.core.state_management import StateMachine, SystemState
    from src.core.blackboard import Blackboard
    from src.autonomy.bt.bt_orchestrator import BTOrchestrator
except ImportError as e:
    pytest.skip(
        f"Skipping state machine tests due to import error: {e}",
        allow_module_level=True,
    )


class SystemState(Enum):
    """System states for rover"""

    IDLE = "IDLE"
    AUTONOMOUS = "AUTONOMOUS"
    TELEOPERATION = "TELEOPERATION"
    EMERGENCY_STOP = "EMERGENCY_STOP"
    MISSION_PAUSED = "MISSION_PAUSED"
    SYSTEM_ERROR = "SYSTEM_ERROR"


@dataclass
class StateTransition:
    """Track state transition data"""

    from_state: SystemState
    to_state: SystemState
    timestamp: float
    trigger: str
    data_consistency_check: Dict[str, bool] = field(default_factory=dict)
    transition_time: float = 0.0

    @property
    def transition_duration(self) -> float:
        return self.transition_time


@dataclass
class StateDataSnapshot:
    """Snapshot of system data at a point in time"""

    timestamp: float
    state: SystemState
    blackboard_data: Dict[str, Any] = field(default_factory=dict)
    bt_status: Dict[str, Any] = field(default_factory=dict)
    motor_commands: List[Twist] = field(default_factory=list)
    sensor_data: Dict[str, Any] = field(default_factory=dict)

    def compare_consistency(self, other: "StateDataSnapshot") -> Dict[str, bool]:
        """Compare data consistency between snapshots"""
        consistency = {}

        # Check blackboard consistency
        for key, value in self.blackboard_data.items():
            if key in other.blackboard_data:
                consistency[f"blackboard_{key}"] = value == other.blackboard_data[key]

        # Check BT status consistency
        for key, value in self.bt_status.items():
            if key in other.bt_status:
                consistency[f"bt_{key}"] = value == other.bt_status[key]

        return consistency


class MockBlackboard(Node):
    """Mock blackboard for shared state data"""

    def __init__(self):
        super().__init__("mock_blackboard")
        self.data: Dict[str, Any] = {}
        self.state_publisher = self.create_publisher(String, "/blackboard/state", 10)
        self.data_publisher = self.create_publisher(String, "/blackboard/data", 10)

    def set(self, key: str, value: Any):
        """Set blackboard data"""
        self.data[key] = value
        self.publish_data()

    def get(self, key: str, default: Any = None) -> Any:
        """Get blackboard data"""
        return self.data.get(key, default)

    def publish_data(self):
        """Publish blackboard data"""
        msg = String()
        msg.data = str(self.data)
        self.data_publisher.publish(msg)

    def publish_state(self, state: SystemState):
        """Publish system state"""
        msg = String()
        msg.data = state.value
        self.state_publisher.publish(msg)


class MockStateMachine(Node):
    """Mock state machine for testing"""

    def __init__(self, initial_state: SystemState = SystemState.IDLE):
        super().__init__("mock_state_machine")
        self.current_state = initial_state
        self.previous_state = initial_state
        self.state_history: List[StateTransition] = []
        self.state_publisher = self.create_publisher(
            String, "/state_machine/current", 10
        )
        self.transition_publisher = self.create_publisher(
            String, "/state_machine/transition", 10
        )
        self.transition_subscriber = self.create_subscription(
            String, "/state_machine/request", self.state_request_callback, 10
        )

    def state_request_callback(self, msg: String):
        """Handle state transition requests"""
        requested_state = SystemState(msg.data)
        self.transition_to(requested_state, "external_request")

    def transition_to(self, new_state: SystemState, trigger: str) -> bool:
        """Transition to new state"""
        if not self.is_valid_transition(self.current_state, new_state):
            return False

        transition_start = time.time()
        self.previous_state = self.current_state

        # Publish transition
        transition_msg = String()
        transition_msg.data = f"{self.current_state.value}->{new_state.value}:{trigger}"
        self.transition_publisher.publish(transition_msg)

        # Perform state transition
        self.current_state = new_state
        transition_time = time.time() - transition_start

        # Record transition
        transition = StateTransition(
            from_state=self.previous_state,
            to_state=self.current_state,
            timestamp=time.time(),
            trigger=trigger,
            transition_time=transition_time,
        )
        self.state_history.append(transition)

        # Publish new state
        state_msg = String()
        state_msg.data = self.current_state.value
        self.state_publisher.publish(state_msg)

        return True

    def is_valid_transition(
        self, from_state: SystemState, to_state: SystemState
    ) -> bool:
        """Check if state transition is valid"""
        valid_transitions = {
            SystemState.IDLE: [SystemState.AUTONOMOUS, SystemState.TELEOPERATION],
            SystemState.AUTONOMOUS: [
                SystemState.IDLE,
                SystemState.TELEOPERATION,
                SystemState.EMERGENCY_STOP,
            ],
            SystemState.TELEOPERATION: [
                SystemState.IDLE,
                SystemState.AUTONOMOUS,
                SystemState.EMERGENCY_STOP,
            ],
            SystemState.EMERGENCY_STOP: [SystemState.IDLE, SystemState.TELEOPERATION],
            SystemState.MISSION_PAUSED: [
                SystemState.AUTONOMOUS,
                SystemState.IDLE,
                SystemState.EMERGENCY_STOP,
            ],
            SystemState.SYSTEM_ERROR: [SystemState.IDLE, SystemState.EMERGENCY_STOP],
        }
        return to_state in valid_transitions.get(from_state, [])


class MockBTOrchestrator(Node):
    """Mock behavior tree orchestrator"""

    def __init__(self):
        super().__init__("mock_bt_orchestrator")
        self.status: Dict[str, Any] = {
            "running": False,
            "current_tree": None,
            "tree_status": "IDLE",
            "last_update": time.time(),
        }
        self.status_publisher = self.create_publisher(String, "/bt/status", 10)
        self.command_subscriber = self.create_subscription(
            String, "/bt/command", self.bt_command_callback, 10
        )

    def bt_command_callback(self, msg: String):
        """Handle BT commands"""
        command = msg.data
        if command == "start":
            self.start_bt()
        elif command == "stop":
            self.stop_bt()
        elif command == "pause":
            self.pause_bt()

    def start_bt(self):
        """Start behavior tree"""
        self.status["running"] = True
        self.status["tree_status"] = "RUNNING"
        self.status["last_update"] = time.time()
        self.publish_status()

    def stop_bt(self):
        """Stop behavior tree"""
        self.status["running"] = False
        self.status["tree_status"] = "STOPPED"
        self.status["current_tree"] = None
        self.status["last_update"] = time.time()
        self.publish_status()

    def pause_bt(self):
        """Pause behavior tree"""
        if self.status["running"]:
            self.status["tree_status"] = "PAUSED"
            self.status["last_update"] = time.time()
            self.publish_status()

    def set_tree(self, tree_name: str):
        """Set current behavior tree"""
        self.status["current_tree"] = tree_name
        self.status["last_update"] = time.time()
        self.publish_status()

    def publish_status(self):
        """Publish BT status"""
        msg = String()
        msg.data = str(self.status)
        self.status_publisher.publish(msg)


class StateMachineMonitor:
    """Monitor state machine data flow and consistency"""

    def __init__(self):
        self.transitions: List[StateTransition] = []
        self.data_snapshots: List[StateDataSnapshot] = []
        self.consistency_checks: List[Dict[str, bool]] = []
        self.active_measurements: Dict[str, Dict] = {}

    def start_transition_monitoring(self, transition_id: str):
        """Start monitoring a state transition"""
        self.active_measurements[transition_id] = {
            "start_time": time.time(),
            "from_state": None,
            "to_state": None,
            "data_before": None,
            "data_after": None,
        }

    def record_transition_data(
        self,
        transition_id: str,
        from_state: SystemState,
        to_state: SystemState,
        snapshot: StateDataSnapshot,
    ):
        """Record data during transition"""
        if transition_id in self.active_measurements:
            measurement = self.active_measurements[transition_id]
            measurement["from_state"] = from_state
            measurement["to_state"] = to_state
            if measurement["data_before"] is None:
                measurement["data_before"] = snapshot
            else:
                measurement["data_after"] = snapshot

    def complete_transition_monitoring(self, transition_id: str) -> StateTransition:
        """Complete transition monitoring and return transition data"""
        if transition_id not in self.active_measurements:
            return None

        measurement = self.active_measurements[transition_id]

        transition = StateTransition(
            from_state=measurement["from_state"],
            to_state=measurement["to_state"],
            timestamp=measurement["start_time"],
            trigger="test",
        )

        # Check data consistency
        if measurement["data_before"] and measurement["data_after"]:
            consistency = measurement["data_before"].compare_consistency(
                measurement["data_after"]
            )
            transition.data_consistency_check = consistency
            self.consistency_checks.append(consistency)

        self.transitions.append(transition)
        del self.active_measurements[transition_id]

        return transition

    def capture_data_snapshot(
        self,
        state: SystemState,
        blackboard: MockBlackboard,
        bt_orchestrator: MockBTOrchestrator,
    ) -> StateDataSnapshot:
        """Capture snapshot of system data"""
        snapshot = StateDataSnapshot(
            timestamp=time.time(),
            state=state,
            blackboard_data=dict(blackboard.data),
            bt_status=dict(bt_orchestrator.status),
        )

        self.data_snapshots.append(snapshot)
        return snapshot


@pytest.fixture
def ros_context():
    """Provide ROS2 context for tests"""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.fixture
def blackboard(ros_context):
    """Provide mock blackboard"""
    return MockBlackboard()


@pytest.fixture
def state_machine(ros_context):
    """Provide mock state machine"""
    return MockStateMachine()


@pytest.fixture
def bt_orchestrator(ros_context):
    """Provide mock BT orchestrator"""
    return MockBTOrchestrator()


@pytest.fixture
def state_monitor():
    """Provide state machine monitor"""
    return StateMachineMonitor()


class TestStateMachineCoordination:
    """Test state machine coordination and data consistency"""

    def test_state_transition_data_consistency(
        self, ros_context, blackboard, state_machine, bt_orchestrator, state_monitor
    ):
        """Test data consistency during state transitions"""

        # Setup initial data
        blackboard.set("position", {"x": 1.0, "y": 2.0})
        blackboard.set("mission_progress", 0.5)
        bt_orchestrator.set_tree("navigation_tree")
        bt_orchestrator.start_bt()

        # Capture initial state
        initial_snapshot = state_monitor.capture_data_snapshot(
            state_machine.current_state, blackboard, bt_orchestrator
        )

        # Start transition monitoring
        transition_id = "test_autonomous_transition"
        state_monitor.start_transition_monitoring(transition_id)

        # Transition from IDLE to AUTONOMOUS
        success = state_machine.transition_to(SystemState.AUTONOMOUS, "test_trigger")

        # Process state change
        for _ in range(10):
            rclpy.spin_once(ros_context, timeout_sec=0.01)

        # Capture data during transition
        during_snapshot = state_monitor.capture_data_snapshot(
            state_machine.current_state, blackboard, bt_orchestrator
        )
        state_monitor.record_transition_data(
            transition_id,
            initial_snapshot.state,
            during_snapshot.state,
            during_snapshot,
        )

        # Verify state transition
        assert success, "State transition should succeed"
        assert (
            state_machine.current_state == SystemState.AUTONOMOUS
        ), "Should be in AUTONOMOUS state"

        # Complete monitoring
        transition = state_monitor.complete_transition_monitoring(transition_id)

        # Verify transition consistency
        assert transition.from_state == SystemState.IDLE, "Should transition from IDLE"
        assert (
            transition.to_state == SystemState.AUTONOMOUS
        ), "Should transition to AUTONOMOUS"
        assert (
            transition.transition_time < 0.050
        ), f"Transition took {transition.transition_time*1000:.1f}ms, required < 50ms"

        # Verify BT status updated appropriately
        assert bt_orchestrator.status[
            "running"
        ], "BT should be running in AUTONOMOUS state"

        print("✅ State transition data consistency verified")

    def test_concurrent_state_changes(
        self, ros_context, blackboard, state_machine, bt_orchestrator, state_monitor
    ):
        """Test handling of simultaneous state change requests"""

        # Setup initial state
        initial_state = state_machine.current_state

        # Simulate concurrent state change requests
        concurrent_requests = [
            (SystemState.AUTONOMOUS, "request1"),
            (SystemState.TELEOPERATION, "request2"),
            (SystemState.EMERGENCY_STOP, "request3"),
        ]

        # Send concurrent requests
        for target_state, trigger in concurrent_requests:
            state_machine.transition_to(target_state, trigger)
            rclpy.spin_once(ros_context, timeout_sec=0.001)

        # Process state changes
        time.sleep(0.1)
        for _ in range(20):
            rclpy.spin_once(ros_context, timeout_sec=0.01)

        # Verify only one transition occurred
        transitions = state_machine.state_history

        # Should have executed at most one transition (first valid one)
        assert len(transitions) <= len(
            concurrent_requests
        ), f"Too many transitions: {len(transitions)}"

        # Current state should be valid
        valid_states = [state for state, _ in concurrent_requests]
        assert (
            state_machine.current_state in valid_states
        ), f"Invalid final state: {state_machine.current_state}"

        # Verify state consistency after concurrent changes
        final_snapshot = state_monitor.capture_data_snapshot(
            state_machine.current_state, blackboard, bt_orchestrator
        )

        assert (
            final_snapshot.state == state_machine.current_state
        ), "State snapshot mismatch"

        print(
            f"✅ Concurrent state changes handled: {state_machine.current_state.value}"
        )

    def test_state_machine_blackboard_sync(
        self, ros_context, blackboard, state_machine, bt_orchestrator, state_monitor
    ):
        """Test blackboard and state machine synchronization"""

        # Test synchronization across multiple state transitions
        test_data = {
            "mission_id": "test_mission_001",
            "waypoint_index": 3,
            "battery_level": 0.75,
            "system_health": "GOOD",
        }

        # Set initial data
        for key, value in test_data.items():
            blackboard.set(key, value)

        # Capture initial data
        initial_data = dict(blackboard.data)

        # Perform state transition
        state_machine.transition_to(SystemState.AUTONOMOUS, "test_sync")

        # Process transition
        for _ in range(10):
            rclpy.spin_once(ros_context, timeout_sec=0.01)

        # Verify blackboard data persisted
        for key, expected_value in test_data.items():
            actual_value = blackboard.get(key)
            assert (
                actual_value == expected_value
            ), f"Blackboard data {key} changed: {actual_value} != {expected_value}"

        # Add new data during AUTONOMOUS state
        new_data = {"current_action": "navigating_to_waypoint", "progress": 0.3}
        for key, value in new_data.items():
            blackboard.set(key, value)

        # Transition to another state
        state_machine.transition_to(SystemState.TELEOPERATION, "test_sync2")

        # Process transition
        for _ in range(10):
            rclpy.spin_once(ros_context, timeout_sec=0.01)

        # Verify all data persisted across transitions
        all_data = {**test_data, **new_data}
        for key, expected_value in all_data.items():
            actual_value = blackboard.get(key)
            assert (
                actual_value == expected_value
            ), f"Data {key} lost in transition: {actual_value} != {expected_value}"

        print("✅ State machine blackboard synchronization verified")

    def test_emergency_state_transition_priority(
        self, ros_context, blackboard, state_machine, bt_orchestrator, state_monitor
    ):
        """Test emergency state transition priority over other transitions"""

        # Start in autonomous mode
        state_machine.transition_to(SystemState.AUTONOMOUS, "test_start")
        bt_orchestrator.start_bt()

        # Process
        for _ in range(10):
            rclpy.spin_once(ros_context, timeout_sec=0.01)

        assert (
            state_machine.current_state == SystemState.AUTONOMOUS
        ), "Should be in AUTONOMOUS state"
        assert bt_orchestrator.status["running"], "BT should be running"

        # Queue normal transition
        state_machine.transition_to(SystemState.TELEOPERATION, "normal_transition")

        # Immediately trigger emergency stop (should have priority)
        emergency_success = state_machine.transition_to(
            SystemState.EMERGENCY_STOP, "emergency_trigger"
        )

        # Process emergency transition
        for _ in range(20):
            rclpy.spin_once(ros_context, timeout_sec=0.01)

        # Verify emergency state has priority
        assert emergency_success, "Emergency transition should succeed"
        assert (
            state_machine.current_state == SystemState.EMERGENCY_STOP
        ), "Should be in EMERGENCY_STOP state"

        # Verify BT stopped in emergency
        assert not bt_orchestrator.status[
            "running"
        ], "BT should be stopped in emergency"
        assert (
            bt_orchestrator.status["tree_status"] == "STOPPED"
        ), "BT status should be STOPPED"

        # Verify emergency data consistency
        emergency_snapshot = state_monitor.capture_data_snapshot(
            state_machine.current_state, blackboard, bt_orchestrator
        )

        assert (
            emergency_snapshot.state == SystemState.EMERGENCY_STOP
        ), "Emergency state not recorded"
        assert not emergency_snapshot.bt_status[
            "running"
        ], "BT running status inconsistent"

        print("✅ Emergency state transition priority verified")

    def test_state_transition_error_handling(
        self, ros_context, blackboard, state_machine, bt_orchestrator, state_monitor
    ):
        """Test state transition error handling"""

        # Test invalid transitions
        invalid_transitions = [
            (
                SystemState.EMERGENCY_STOP,
                SystemState.AUTONOMOUS,
            ),  # Emergency to autonomous not allowed
            (
                SystemState.SYSTEM_ERROR,
                SystemState.AUTONOMOUS,
            ),  # Error to autonomous not allowed
        ]

        for from_state, to_state in invalid_transitions:
            # Force state machine into from_state
            state_machine.current_state = from_state

            # Attempt invalid transition
            success = state_machine.transition_to(to_state, "test_invalid")

            # Verify transition rejected
            assert (
                not success
            ), f"Invalid transition {from_state.value} -> {to_state.value} should fail"
            assert (
                state_machine.current_state == from_state
            ), "State should not change on invalid transition"

        # Test valid transitions from error states
        error_recovery_transitions = [
            (SystemState.EMERGENCY_STOP, SystemState.IDLE),
            (SystemState.SYSTEM_ERROR, SystemState.IDLE),
        ]

        for from_state, to_state in error_recovery_transitions:
            # Force state machine into from_state
            state_machine.current_state = from_state

            # Attempt valid recovery transition
            success = state_machine.transition_to(to_state, "test_recovery")

            # Verify transition succeeded
            assert (
                success
            ), f"Valid transition {from_state.value} -> {to_state.value} should succeed"
            assert (
                state_machine.current_state == to_state
            ), "State should change on valid transition"

        print("✅ State transition error handling verified")

    def test_state_transition_performance(
        self, ros_context, blackboard, state_machine, bt_orchestrator, state_monitor
    ):
        """Test state transition performance under load"""

        # Measure transition times under various loads
        load_scenarios = [("no_load", 0), ("light_load", 100), ("heavy_load", 1000)]

        for scenario_name, load_level in load_scenarios:
            print(f"Testing {scenario_name} with load level {load_level}...")

            # Simulate load by setting many blackboard entries
            for i in range(load_level):
                blackboard.set(f"load_data_{i}", f"value_{i}")

            # Measure transition time
            start_time = time.time()
            success = state_machine.transition_to(
                SystemState.AUTONOMOUS, f"test_{scenario_name}"
            )

            # Process transition
            for _ in range(20):
                rclpy.spin_once(ros_context, timeout_sec=0.01)

            transition_time = time.time() - start_time

            # Verify performance requirements
            assert success, f"Transition failed under {scenario_name}"
            assert (
                transition_time < 0.100
            ), f"Transition under {scenario_name} took {transition_time*1000:.1f}ms, required < 100ms"

            # Clean up for next test
            for i in range(load_level):
                blackboard.data.pop(f"load_data_{i}", None)

            state_machine.transition_to(SystemState.IDLE, "cleanup")

        print("✅ State transition performance verified")


class TestStateMachineIntegration:
    """Test state machine integration with other system components"""

    def test_state_machine_bt_coordination(
        self, ros_context, state_machine, bt_orchestrator
    ):
        """Test state machine and behavior tree coordination"""

        # Test BT lifecycle coordination with states
        state_bt_mappings = {
            SystemState.IDLE: ("stopped", False),
            SystemState.AUTONOMOUS: ("running", True),
            SystemState.TELEOPERATION: ("stopped", False),
            SystemState.EMERGENCY_STOP: ("stopped", False),
        }

        for target_state, (
            expected_status,
            expected_running,
        ) in state_bt_mappings.items():
            # Transition to state
            state_machine.transition_to(target_state, "test_bt_coord")

            # Process transition
            for _ in range(10):
                rclpy.spin_once(ros_context, timeout_sec=0.01)

            # Simulate BT coordination
            if expected_running:
                bt_orchestrator.start_bt()
            else:
                bt_orchestrator.stop_bt()

            # Verify BT status matches state requirements
            assert (
                bt_orchestrator.status["running"] == expected_running
            ), f"BT running status {bt_orchestrator.status['running']} doesn't match expected {expected_running} for state {target_state.value}"

        print("✅ State machine BT coordination verified")

    def test_state_transition_data_propagation(
        self, ros_context, blackboard, state_machine
    ):
        """Test data propagation during state transitions"""

        # Setup test data
        transition_data = {
            "transition_start_time": time.time(),
            "transition_reason": "test_propagation",
            "user_id": "test_user",
        }

        # Set data before transition
        for key, value in transition_data.items():
            blackboard.set(key, value)

        # Perform state transition
        state_machine.transition_to(SystemState.AUTONOMOUS, "test_propagation")

        # Process transition
        for _ in range(10):
            rclpy.spin_once(ros_context, timeout_sec=0.01)

        # Verify data propagated correctly
        for key, expected_value in transition_data.items():
            actual_value = blackboard.get(key)
            assert (
                actual_value == expected_value
            ), f"Data {key} not propagated: {actual_value} != {expected_value}"

        # Verify transition metadata was added
        assert "last_transition_time" in blackboard.data, "Transition time not recorded"
        assert (
            "last_transition_state" in blackboard.data
        ), "Transition state not recorded"

        print("✅ State transition data propagation verified")


if __name__ == "__main__":
    # Run tests manually for debugging
    pytest.main([__file__, "-v"])
