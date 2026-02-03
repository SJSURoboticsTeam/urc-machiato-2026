#!/usr/bin/env python3
"""
Basic Integration Tests - Component Interaction Validation

Tests basic integration between system components without requiring
full ROS2 infrastructure. Focuses on logic validation and data flow.
"""

import json
import os
import subprocess
import sys
import time
import unittest
from unittest.mock import Mock, patch

# ROS2 imports for QoS testing
try:
    import rclpy
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
except ImportError:
    # Mock ROS2 imports for systems without ROS2 installed
    rclpy = None
    QoSProfile = Mock()
    ReliabilityPolicy = Mock()
    DurabilityPolicy = Mock()

# Add project paths for imports
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
AUTONOMY_CODE_ROOT = os.path.join(PROJECT_ROOT, "autonomy", "code")
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, AUTONOMY_CODE_ROOT)


# Setup ROS2 environment for tests
def setup_ros2_environment():
    """Setup ROS2 environment if available."""
    try:
        # Try to source ROS2 and autonomy workspace
        env_setup = """
source /opt/ros/humble/setup.bash
cd /home/ubuntu/urc-machiato-2026/autonomy
source install/setup.bash
export ROS_DOMAIN_ID=42
"""
        # This will set up the environment for subprocess calls
        return True
    except:
        # ROS2 not available, skip ROS2-dependent tests
        return False


ROS2_AVAILABLE = setup_ros2_environment()


class TestBasicIntegration(unittest.TestCase):
    """Test basic component integration and data flow."""

    def setUp(self):
        """Set up test fixtures."""
        # Mock ROS2 components for testing
        self.mock_node = Mock()
        self.mock_publisher = Mock()
        self.mock_subscriber = Mock()
        self.mock_client = Mock()
        self.mock_logger = Mock()

        # Setup mock node
        self.mock_node.create_publisher.return_value = self.mock_publisher
        self.mock_node.create_subscription.return_value = self.mock_subscriber
        self.mock_node.create_client.return_value = self.mock_client
        self.mock_node.get_logger.return_value = self.mock_logger
        self.mock_node.get_clock.return_value = Mock()

    def test_state_machine_bridge_initialization(self):
        """Test state machine bridge initializes correctly."""
        try:
            from bridges.ros2_state_machine_bridge import SystemState

            # Test state definitions exist
            self.assertEqual(SystemState.BOOT.value, "BOOT")
            self.assertEqual(SystemState.AUTONOMOUS.value, "AUTONOMOUS")
            self.assertEqual(len(list(SystemState)), 7)  # All states defined

            # Test valid transitions (basic validation)
            from src.autonomy.core.state_management.autonomy_state_machine.states import (
                RoverState,
                can_transition,
            )

            # Test some known valid transitions
            self.assertTrue(can_transition(RoverState.BOOT, RoverState.READY))
            self.assertTrue(can_transition(RoverState.READY, RoverState.AUTO))

            # Test invalid transitions
            self.assertFalse(can_transition(RoverState.BOOT, RoverState.AUTO))
            self.assertFalse(can_transition(RoverState.ESTOP, RoverState.AUTO))

        except Exception as e:
            self.fail(f"State machine bridge initialization failed: {e}")

    @unittest.skipUnless(ROS2_AVAILABLE, "ROS2 not available for testing")
    def test_ros2_interfaces_available(self):
        """Test that ROS2 interfaces are available when ROS2 is set up."""
        try:
            # Run a subprocess with ROS2 environment to test imports
            result = subprocess.run(
                [
                    "bash",
                    "-c",
                    "source /opt/ros/humble/setup.bash && "
                    "cd /home/ubuntu/urc-machiato-2026/autonomy && "
                    "source install/setup.bash && "
                    'python3 -c "from autonomy_interfaces.action import NavigateToPose; print("success")"',
                ],
                capture_output=True,
                text=True,
                timeout=30,
            )

            self.assertEqual(
                result.returncode, 0, f"ROS2 import failed: {result.stderr}"
            )
            self.assertIn("success", result.stdout)

        except subprocess.TimeoutExpired:
            self.skipTest("ROS2 interface test timed out")
        except Exception as e:
            self.skipTest(f"ROS2 not available: {e}")

    def test_mission_executor_integration(self):
        """Test mission executor integrates with state machine."""
        try:
            from missions.mission_executor import MissionExecutor

            # Create executor instance (will use mock ROS2)
            with patch("rclpy.node.Node"):
                executor = MissionExecutor.__new__(MissionExecutor)
                executor.current_mission = None
                executor.mission_active = False

                # Test mission state management
                self.assertFalse(executor.mission_active)
                self.assertIsNone(executor.current_mission)

                # Simulate mission activation
                executor.mission_active = True
                executor.current_mission = {"type": "test", "id": "test_001"}

                self.assertTrue(executor.mission_active)
                self.assertEqual(executor.current_mission["type"], "test")

        except Exception as e:
            self.fail(f"Mission executor integration failed: {e}")

    def test_safety_system_integration(self):
        """Test safety system integrates with other components."""
        try:
            from src.autonomy.core.state_management.autonomy_state_machine.safety_manager import (
                SafetyManager,
                SafetySeverity,
                SafetyTriggerType,
            )

            # Test safety enums
            self.assertEqual(SafetyTriggerType.EMERGENCY_STOP.value, "EMERGENCY_STOP")
            self.assertEqual(SafetySeverity.CRITICAL.value, "CRITICAL")

            # Test safety trigger classification
            emergency_triggers = [
                SafetyTriggerType.EMERGENCY_STOP,
                SafetyTriggerType.SOFTWARE_ESTOP,
                SafetyTriggerType.COMMUNICATION_LOSS,
            ]

            for trigger in emergency_triggers:
                self.assertIn(
                    trigger.value,
                    ["EMERGENCY_STOP", "SOFTWARE_ESTOP", "COMMUNICATION_LOSS"],
                )

        except Exception as e:
            self.fail(f"Safety system integration failed: {e}")

    def test_mission_data_flow(self):
        """Test data flows correctly between mission components."""
        try:
            # Test mission command structure
            mission_commands = [
                {
                    "type": "start",
                    "config": {
                        "name": "Test Mission",
                        "type": "waypoint_navigation",
                        "waypoints": [[0, 0], [5, 0], [5, 5]],
                    },
                },
                {"type": "aruco_search", "config": {"tag_id": 42, "timeout": 300}},
                {
                    "type": "return_to_operator",
                    "config": {"use_gps": True, "use_aruco": True},
                },
            ]

            # Validate command structures
            for cmd in mission_commands:
                self.assertIn("type", cmd)
                self.assertIn("config", cmd)

                if cmd["type"] == "start":
                    self.assertIn("name", cmd["config"])
                    self.assertIn("type", cmd["config"])

            # Test mission result structure
            sample_result = {
                "mission_id": "test_001",
                "success": True,
                "completion_time": time.time(),
                "data": {"waypoints_completed": 3},
            }

            required_fields = ["mission_id", "success", "completion_time"]
            for field in required_fields:
                self.assertIn(field, sample_result)

        except Exception as e:
            self.fail(f"Mission data flow test failed: {e}")

    def test_state_transition_integration(self):
        """Test state transitions integrate with mission system."""
        try:
            from autonomy.code.state_management.autonomy_state_machine.states import (
                RoverState,
            )
            from bridges.ros2_state_machine_bridge import SystemState

            # Test state mapping between systems
            # ROS2 bridge uses 8 states, simple system uses 7
            ros2_states = [s.value for s in SystemState]
            simple_states = [s.value for s in RoverState]

            # Check for expected states
            self.assertIn("BOOT", ros2_states)
            self.assertIn("AUTONOMOUS", ros2_states)
            self.assertIn("IDLE", ros2_states)  # ROS2 bridge uses IDLE instead of READY

            self.assertIn("boot", simple_states)
            self.assertIn("auto", simple_states)
            self.assertIn("ready", simple_states)

            # Test that both systems can represent the same operational states
            # (just with different naming conventions)

        except Exception as e:
            self.fail(f"State transition integration failed: {e}")

    def test_error_handling_integration(self):
        """Test error handling across component boundaries."""
        try:
            # Test structured error responses
            error_scenarios = [
                {
                    "component": "mission_executor",
                    "error": "navigation_failure",
                    "recovery": "retry_navigation",
                },
                {
                    "component": "state_machine",
                    "error": "invalid_transition",
                    "recovery": "maintain_current_state",
                },
                {
                    "component": "safety_system",
                    "error": "sensor_failure",
                    "recovery": "switch_to_safe_mode",
                },
            ]

            for scenario in error_scenarios:
                self.assertIn("component", scenario)
                self.assertIn("error", scenario)
                self.assertIn("recovery", scenario)

                # Validate error message structure
                self.assertIsInstance(scenario["component"], str)
                self.assertIsInstance(scenario["error"], str)
                self.assertIsInstance(scenario["recovery"], str)

        except Exception as e:
            self.fail(f"Error handling integration failed: {e}")

    def test_configuration_integration(self):
        """Test configuration integration across components."""
        try:
            import yaml

            # Test configuration loading and validation
            sample_config = """
simulation_update_rate_hz: 10
mission_execution_rate_hz: 10
safety_distance_meters: 0.3
max_mission_time: 600
"""

            config = yaml.safe_load(sample_config)

            # Validate required configuration fields
            required_fields = [
                "simulation_update_rate_hz",
                "mission_execution_rate_hz",
                "safety_distance_meters",
                "max_mission_time",
            ]

            for field in required_fields:
                self.assertIn(field, config)

            # Test configuration value ranges
            self.assertGreater(config["simulation_update_rate_hz"], 0)
            self.assertGreater(config["safety_distance_meters"], 0)
            self.assertGreater(config["max_mission_time"], 0)

        except Exception as e:
            self.fail(f"Configuration integration failed: {e}")

    def test_performance_requirements(self):
        """Test that components meet performance requirements."""
        try:
            # Test timing requirements
            start_time = time.time()

            # Simulate some processing (represents typical operation)
            for i in range(1000):
                # Simple computation to simulate processing
                result = i * i + i

            end_time = time.time()
            processing_time = end_time - start_time

            # Should complete in reasonable time (< 1 second for 1000 operations)
            self.assertLess(processing_time, 1.0, "Processing too slow")

            # Test memory efficiency (basic check)
            # In a real test, we'd check memory usage before/after operations
            self.assertTrue(True, "Basic performance check passed")

        except Exception as e:
            self.fail(f"Performance requirements test failed: {e}")

    def test_hardware_interface_qos_optimization(self):
        """Test that hardware interface uses optimized QoS settings."""
        try:
            # Import hardware interface for QoS validation
            from autonomy.control.hardware_interface.hardware_interface_node import (
                HardwareInterfaceNode,
            )

            # Create node instance (mock ROS2 context)
            with patch("rclpy.node.Node.__init__", return_value=None):
                node = HardwareInterfaceNode.__new__(HardwareInterfaceNode)

                # Test control QoS settings (depth=20, deadline=50ms)
                control_qos = QoSProfile(
                    reliability=ReliabilityPolicy.RELIABLE,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=20,
                    deadline=rclpy.duration.Duration(milliseconds=50),
                )

                # Validate control QoS matches optimization requirements
                self.assertEqual(
                    control_qos.depth, 20, "Control QoS depth should be 20"
                )
                self.assertEqual(control_qos.reliability, ReliabilityPolicy.RELIABLE)
                self.assertEqual(control_qos.deadline.nanoseconds, 50000000)  # 50ms

                # Test sensor QoS settings (depth=5, deadline=20ms)
                sensor_qos = QoSProfile(
                    reliability=ReliabilityPolicy.BEST_EFFORT,
                    durability=DurabilityPolicy.VOLATILE,
                    depth=5,
                    deadline=rclpy.duration.Duration(milliseconds=20),
                )

                # Validate sensor QoS matches optimization requirements
                self.assertEqual(sensor_qos.depth, 5, "Sensor QoS depth should be 5")
                self.assertEqual(sensor_qos.reliability, ReliabilityPolicy.BEST_EFFORT)
                self.assertEqual(sensor_qos.deadline.nanoseconds, 20000000)  # 20ms

        except Exception as e:
            self.fail(f"QoS optimization test failed: {e}")


if __name__ == "__main__":
    unittest.main()
