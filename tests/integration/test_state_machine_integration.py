"""
Integration tests for state machine.

Tests state machine integration with ROS2, subsystem coordination,
and frontend communication.
"""

import os
import sys
import time

import pytest
import rclpy
from autonomy_interfaces.msg import SystemState as SystemStateMsg
from autonomy_interfaces.srv import ChangeState
from rclpy.node import Node

# Add the state machine to path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
STATE_MGMT_ROOT = os.path.join(PROJECT_ROOT, "Autonomy", "code", "state_management")
sys.path.insert(0, STATE_MGMT_ROOT)
sys.path.insert(0, os.path.join(STATE_MGMT_ROOT, "autonomy_state_machine"))


# Removed ROS2 context fixtures - tests now check ROS2 services/topics directly


@pytest.mark.integration
class TestStateMachineIntegration:
    """Integration tests for state machine director node."""

    def test_state_machine_node_creation(self, ros_context):
        """Test that state machine node can be created."""
        from autonomy_state_machine.state_machine_director import StateMachineDirector

        node = StateMachineDirector()
        assert node is not None
        assert node.get_name() == "state_machine_director"
        node.destroy_node()

    def test_service_availability(self):
        """Test that state machine services are available."""
        import os
        import subprocess

        # Check if ROS2 service is available
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = '42'

        result = subprocess.run(
            ['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 service list'],
            env=env, capture_output=True, text=True, timeout=5
        )

        services = result.stdout.strip().split('\n')
        assert "/state_machine/change_state" in services, f"Service not found in: {services}"
        assert "/state_machine/execute_mission" in services, f"Service not found in: {services}"

    def test_state_publishing(self):
        """Test that state machine publishes state updates."""
        import os
        import subprocess

        # Check if ROS2 topic is available and has messages
        env = os.environ.copy()
        env['ROS_DOMAIN_ID'] = '42'

        result = subprocess.run(
            ['bash', '-c', 'source /opt/ros/humble/setup.bash && ros2 topic list'],
            env=env, capture_output=True, text=True, timeout=5
        )

        topics = result.stdout.strip().split('\n')
        assert "/state_machine/current_state" in topics, f"Topic not found in: {topics}"

        # Try to echo the topic to see if it has data
        echo_result = subprocess.run(
            ['bash', '-c', 'source /opt/ros/humble/setup.bash && timeout 3 ros2 topic echo /state_machine/current_state --once'],
            env=env, capture_output=True, text=True, timeout=5
        )

        # Should get some output if the topic is publishing
        assert echo_result.returncode == 0 or "data:" in echo_result.stdout, "Topic not publishing data"


@pytest.mark.integration
class TestFrontendIntegration:
    """Integration tests for frontend interface."""

    def test_frontend_interface_creation(self, ros_context):
        """Test that frontend interface can be created."""
        from autonomy_state_machine.frontend_interface import FrontendInterface

        node = Node("test_frontend")
        interface = FrontendInterface(node)
        assert interface is not None

        node.destroy_node()

    def test_state_callback_registration(self, ros_context):
        """Test that state callbacks can be registered."""
        from autonomy_state_machine.frontend_interface import FrontendInterface

        node = Node("test_frontend")
        interface = FrontendInterface(node)

        callback_called = [False]

        def test_callback(msg):
            callback_called[0] = True

        interface.register_state_callback(test_callback)

        # Simulate state update
        from autonomy_interfaces.msg import SystemState as SystemStateMsg

        msg = SystemStateMsg()
        msg.current_state = "IDLE"
        interface._state_callback(msg)

        assert callback_called[0] is True

        node.destroy_node()


@pytest.mark.integration
class TestSubsystemCoordination:
    """Integration tests for subsystem coordination."""

    def test_subsystem_coordinator_creation(self, ros_context):
        """Test that subsystem coordinator can be created."""
        from autonomy_state_machine.subsystem_coordinator import SubsystemCoordinator

        node = Node("test_node")
        coordinator = SubsystemCoordinator(node)
        assert coordinator is not None

        node.destroy_node()

    def test_subsystem_initialization(self, ros_context):
        """Test subsystem initialization."""
        from autonomy_state_machine.subsystem_coordinator import SubsystemCoordinator

        node = Node("test_node")
        coordinator = SubsystemCoordinator(node)

        coordinator.initialize_subsystems()
        active = coordinator.get_active_subsystems()
        assert len(active) > 0

        node.destroy_node()

    def test_subsystem_activation(self, ros_context):
        """Test subsystem activation for different states."""
        from autonomy_state_machine.states import AutonomousSubstate
        from autonomy_state_machine.subsystem_coordinator import SubsystemCoordinator

        node = Node("test_node")
        coordinator = SubsystemCoordinator(node)

        # Test autonomous mode activation
        coordinator.enable_autonomous(AutonomousSubstate.AUTONOMOUS_NAVIGATION)
        active = coordinator.get_active_subsystems()
        assert "navigation" in active
        assert "computer_vision" in active

        node.destroy_node()


@pytest.mark.integration
class TestSafetyIntegration:
    """Integration tests for safety handling."""

    def test_safety_status_publishing(self, ros_context):
        """Test that safety status is published."""
        from autonomy_interfaces.msg import SafetyStatus as SafetyStatusMsg
        from autonomy_state_machine.state_machine_director import StateMachineDirector

        director = StateMachineDirector()

        # Create subscriber for safety status
        class SafetyListener(Node):
            def __init__(self):
                super().__init__("test_safety_listener")
                self.received_status = []
                self.subscription = self.create_subscription(
                    SafetyStatusMsg,
                    "/state_machine/safety_status",
                    self.safety_callback,
                    10,
                )

            def safety_callback(self, msg):
                self.received_status.append(msg)

        listener = SafetyListener()

        # Spin both nodes
        for _ in range(10):
            rclpy.spin_once(director, timeout_sec=0.1)
            rclpy.spin_once(listener, timeout_sec=0.1)
            time.sleep(0.1)

        # Should have received safety status
        assert len(listener.received_status) > 0

        director.destroy_node()
        listener.destroy_node()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
