#!/usr/bin/env python3
"""
State Machine Concurrent Update Tests

Tests state machine behavior under concurrent state change requests,
race conditions, and thread safety.
"""

import os
import sys
import threading
import time

import pytest
import rclpy
from autonomy_interfaces.msg import SystemState as SystemStateMsg
from autonomy_interfaces.srv import ChangeState
from rclpy.node import Node

# Add project paths
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
STATE_MGMT_ROOT = os.path.join(PROJECT_ROOT, "Autonomy", "code", "state_management")
sys.path.insert(0, STATE_MGMT_ROOT)
sys.path.insert(0, os.path.join(STATE_MGMT_ROOT, "autonomy_state_machine"))


@pytest.fixture
def ros_context():
    """Initialize and cleanup ROS context."""
    rclpy.init()
    yield
    rclpy.shutdown()


@pytest.mark.integration
@pytest.mark.ros2
class TestStateMachineConcurrent:
    """Test state machine concurrent update handling."""

    def test_concurrent_state_requests_threading(self, ros_context):
        """Test concurrent state requests from multiple threads."""
        from autonomy_state_machine.state_machine_director import StateMachineDirector

        director = StateMachineDirector()

        try:
            states_received = []
            lock = threading.Lock()
            errors = []

            class StateListener(Node):
                def __init__(self):
                    super().__init__("concurrent_listener")
                    self.states = []
                    self.subscription = self.create_subscription(
                        SystemStateMsg,
                        "/state_machine/current_state",
                        self.state_callback,
                        10,
                    )

                def state_callback(self, msg):
                    with lock:
                        self.states.append((msg.current_state, time.time()))

            listener = StateListener()

            # Spin to receive initial state
            for _ in range(5):
                rclpy.spin_once(director, timeout_sec=0.1)
                rclpy.spin_once(listener, timeout_sec=0.1)
                time.sleep(0.1)

            def request_state_change(target_state: str, thread_id: int):
                """Request state change from a thread."""
                try:
                    client = director.create_client(ChangeState, "/state_machine/change_state")
                    if client.wait_for_service(timeout_sec=5.0):
                        request = ChangeState.Request()
                        request.target_state = target_state
                        future = client.call_async(request)

                        # Wait for response
                        start_time = time.time()
                        while time.time() - start_time < 5.0:
                            rclpy.spin_once(director, timeout_sec=0.1)
                            if future.done():
                                break
                            time.sleep(0.1)
                except Exception as e:
                    with lock:
                        errors.append(f"Thread {thread_id}: {str(e)}")

            # Create multiple threads requesting different states
            threads = []
            target_states = ["IDLE", "AUTONOMOUS", "TELEOPERATION", "SAFETY"]

            for i, target_state in enumerate(target_states):
                thread = threading.Thread(
                    target=request_state_change,
                    args=(target_state, i),
                )
                threads.append(thread)
                thread.start()

            # Monitor state changes
            start_time = time.time()
            while time.time() - start_time < 10.0:
                rclpy.spin_once(director, timeout_sec=0.1)
                rclpy.spin_once(listener, timeout_sec=0.1)
                time.sleep(0.1)

            # Wait for all threads
            for thread in threads:
                thread.join(timeout=2.0)

            # System should handle concurrent requests without crashing
            assert len(errors) == 0, f"Should not have errors: {errors}"
            # Should receive state updates
            with lock:
                assert len(listener.states) > 0, "Should receive state updates"

            director.destroy_node()
            listener.destroy_node()

        finally:
            pass

    def test_state_machine_thread_safety(self, ros_context):
        """Test state machine thread safety."""
        from autonomy_state_machine.state_machine_director import StateMachineDirector

        director = StateMachineDirector()

        try:
            state_accesses = []
            lock = threading.Lock()
            num_threads = 5
            accesses_per_thread = 10

            def access_state(thread_id: int):
                """Access state machine from thread."""
                for _ in range(accesses_per_thread):
                    try:
                        # Create subscriber to access state
                        class QuickListener(Node):
                            def __init__(self):
                                super().__init__(f"thread_{thread_id}_listener")
                                self.received = False
                                self.subscription = self.create_subscription(
                                    SystemStateMsg,
                                    "/state_machine/current_state",
                                    self.callback,
                                    10,
                                )

                            def callback(self, msg):
                                self.received = True

                        listener = QuickListener()

                        # Spin briefly
                        for _ in range(3):
                            rclpy.spin_once(director, timeout_sec=0.05)
                            rclpy.spin_once(listener, timeout_sec=0.05)

                        with lock:
                            state_accesses.append((thread_id, listener.received))

                        listener.destroy_node()
                        time.sleep(0.01)

                    except Exception as e:
                        with lock:
                            state_accesses.append((thread_id, f"Error: {str(e)}"))

            # Start concurrent access threads
            threads = []
            for i in range(num_threads):
                thread = threading.Thread(target=access_state, args=(i,))
                threads.append(thread)
                thread.start()

            # Wait for all threads
            for thread in threads:
                thread.join(timeout=10.0)

            # All threads should successfully access state
            successful_accesses = sum(1 for _, result in state_accesses if result is True)
            total_accesses = len(state_accesses)

            assert total_accesses > 0, "Should have state accesses"
            # Most accesses should succeed
            assert successful_accesses > total_accesses * 0.8, "Most accesses should succeed"

            director.destroy_node()

        finally:
            pass


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
