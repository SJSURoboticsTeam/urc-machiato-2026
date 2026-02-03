#!/usr/bin/env python3
"""
Launch File Integration Test - ROS2 Jazzy Best Practices

This test demonstrates the recommended ROS2 Jazzy approach using launch_testing
for testing launch files and system integration.

Key Benefits:
- Automatic process management
- Built-in ROS2 lifecycle testing
- Integration with colcon test
- Better launch file validation

Usage:
    # Run with pytest
    pytest tests/integration/test_launch_file_integration.py -v
    
    # Run with colcon (recommended for ROS2 packages)
    colcon test --packages-select autonomy_bt
    colcon test-result --verbose
"""

import pytest
import os
import sys
from pathlib import Path

# Add project root to path
PROJECT_ROOT = Path(__file__).parent.parent.parent
sys.path.insert(0, str(PROJECT_ROOT))

# Try to import launch_testing - skip tests if not available
try:
    import launch
    import launch_ros
    from launch_testing import LaunchTestService
    from launch_testing.actions import ReadyToTest
    from launch_testing.asserts import assertSequentialStdout
    from launch_testing.util import KeepAliveProc

    LAUNCH_TESTING_AVAILABLE = True
except ImportError:
    LAUNCH_TESTING_AVAILABLE = False
    pytestmark = pytest.mark.skip(
        reason="launch_testing not available - install: sudo apt install ros-jazzy-launch-testing ros-jazzy-launch-testing-ros"
    )


@pytest.mark.launch_testing
@pytest.mark.ros2
@pytest.mark.integration
class TestLaunchFileIntegration:
    """
    Integration tests using launch_testing framework.

    These tests validate that launch files start nodes correctly,
    that nodes communicate properly, and that the system behaves
    as expected when launched.
    """

    def test_bt_orchestrator_launch(self, launch_service):
        """
        Test that BT Orchestrator launches successfully.

        This demonstrates the launch_testing pattern:
        1. Launch the system
        2. Wait for ready signal
        3. Verify nodes are running
        4. Test communication
        """
        # Get launch file path
        launch_file_path = (
            PROJECT_ROOT
            / "tools"
            / "scripts"
            / "launch"
            / "integrated_system.launch.py"
        )

        if not launch_file_path.exists():
            pytest.skip(f"Launch file not found: {launch_file_path}")

        # Create launch description
        launch_description = launch.LaunchDescription(
            [
                launch.actions.IncludeLaunchDescription(
                    launch.launch_description_sources.PythonLaunchDescriptionSource(
                        str(launch_file_path)
                    )
                ),
                ReadyToTest(),  # Signal that we're ready to test
            ]
        )

        # Launch and test
        with launch_service.launch(launch_description):
            # Wait for nodes to be ready
            # In a real test, you would:
            # 1. Check that nodes are running: ros2 node list
            # 2. Verify topics are available: ros2 topic list
            # 3. Test communication: publish/subscribe
            # 4. Verify expected behavior

            # Example: Check that BT orchestrator node exists
            # This would require ROS2 CLI integration or node introspection
            assert True  # Placeholder - implement actual checks

    def test_node_lifecycle(self, launch_service):
        """
        Test node lifecycle transitions.

        Demonstrates testing lifecycle-managed nodes using launch_testing.
        """
        # Example lifecycle test
        # This would test that nodes transition through:
        # unconfigured -> inactive -> active -> shutdown

        # For now, this is a template
        assert True  # Placeholder


@pytest.fixture
def launch_service():
    """
    Provide a LaunchTestService for tests.

    This fixture manages the launch service lifecycle,
    ensuring proper cleanup after tests.
    """
    if not LAUNCH_TESTING_AVAILABLE:
        pytest.skip("launch_testing not available")

    service = LaunchTestService()
    yield service
    # Cleanup handled by LaunchTestService


# Alternative: Direct pytest test without launch_testing
# This shows the hybrid approach - use pytest for logic, launch_testing for launch files
@pytest.mark.ros2
@pytest.mark.integration
def test_ros2_node_communication():
    """
    Example of testing ROS2 nodes directly with pytest.

    This approach is fine for unit/integration tests that don't
    need to test launch files specifically.
    """
    try:
        import rclpy
        from rclpy.node import Node

        if not rclpy.ok():
            rclpy.init()

        test_node = Node("test_node")

        # Test node functionality
        assert test_node.get_name() == "test_node"

        test_node.destroy_node()
        rclpy.shutdown()

    except ImportError:
        pytest.skip("ROS2 not available")


# Example of using both approaches together
@pytest.mark.launch_testing
@pytest.mark.ros2
@pytest.mark.integration
def test_hybrid_approach():
    """
    Demonstrates using both pytest and launch_testing together.

    Best Practice:
    - Use pytest for: unit tests, logic tests, fast tests
    - Use launch_testing for: launch file tests, system integration, lifecycle tests
    """
    # This test would combine both:
    # 1. Launch system with launch_testing
    # 2. Use pytest for assertions and test logic
    # 3. Use ROS2 introspection for verification

    assert True  # Placeholder
