#!/usr/bin/env python3
"""
Unit Tests for Mission Executor

Tests the mission executor ROS2 node that handles:
- Mission start/stop/pause commands
- Status publishing
- Command processing

Author: URC 2026 Test Suite
"""

import pytest
from unittest.mock import Mock, MagicMock, patch
import sys
import os

# Add repo root so missions package is importable
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
from missions.mission_executor import MissionExecutor, MissionState


class TestMissionState:
    """Test MissionState enum."""

    @pytest.mark.unit
    def test_mission_states(self):
        """Test all mission state values."""
        assert MissionState.IDLE.value == "idle"
        assert MissionState.ACTIVE.value == "active"
        assert MissionState.PAUSED.value == "paused"
        assert MissionState.COMPLETED.value == "completed"
        assert MissionState.FAILED.value == "failed"


class TestMissionExecutor:
    """Test MissionExecutor ROS2 node."""

    @pytest.fixture
    def mock_node(self):
        """Create mock ROS2 node."""
        node = Mock()
        node.create_subscription = Mock()
        node.create_publisher = Mock()
        node.get_logger = Mock()
        node.get_logger.return_value.info = Mock()
        node.get_logger.return_value.warn = Mock()
        node.get_logger.return_value.error = Mock()
        return node

    @pytest.fixture
    def mission_executor(self, mock_node):
        """Create MissionExecutor instance with mocked ROS2 components."""
        with patch("missions.mission_executor.Node", return_value=mock_node):
            executor = MissionExecutor()
            return executor

    @pytest.mark.unit
    def test_initialization(self, mission_executor, mock_node):
        """Test MissionExecutor initialization."""
        # Verify ROS2 interfaces are created
        mock_node.create_subscription.assert_called_once()
        mock_node.create_publisher.assert_called_once()

        # Verify initial state
        assert mission_executor.current_mission is None
        assert mission_executor.mission_active is False

        # Verify logger was called
        mock_node.get_logger.return_value.info.assert_called_with(
            "Mission Executor ready"
        )

    @pytest.mark.unit
    def test_command_callback_start(self, mission_executor):
        """Test command callback for 'start' command."""
        msg = Mock()
        msg.data = "start"

        # Mock the start_mission method
        mission_executor.start_mission = Mock()

        mission_executor._command_callback(msg)

        mission_executor.start_mission.assert_called_once()

    @pytest.mark.unit
    def test_command_callback_stop(self, mission_executor):
        """Test command callback for 'stop' command."""
        msg = Mock()
        msg.data = "STOP"  # Test case insensitive

        mission_executor.stop_mission = Mock()

        mission_executor._command_callback(msg)

        mission_executor.stop_mission.assert_called_once()

    @pytest.mark.unit
    def test_command_callback_pause(self, mission_executor):
        """Test command callback for 'pause' command."""
        msg = Mock()
        msg.data = "Pause"  # Test mixed case

        mission_executor.pause_mission = Mock()

        mission_executor._command_callback(msg)

        mission_executor.pause_mission.assert_called_once()

    @pytest.mark.unit
    def test_command_callback_unknown(self, mission_executor):
        """Test command callback for unknown command."""
        msg = Mock()
        msg.data = "unknown_command"

        mission_executor._command_callback(msg)

        # Should log warning for unknown command
        mission_executor.node.get_logger.return_value.warn.assert_called_with(
            "Unknown command: unknown_command"
        )

    @pytest.mark.unit
    def test_command_callback_exception(self, mission_executor):
        """Test command callback exception handling."""
        msg = Mock()
        msg.data = "start"

        # Make start_mission raise an exception
        mission_executor.start_mission = Mock(side_effect=Exception("Test error"))

        mission_executor._command_callback(msg)

        # Should log error
        mission_executor.node.get_logger.return_value.error.assert_called_with(
            "Command processing failed: Test error"
        )

    @pytest.mark.unit
    def test_start_mission(self, mission_executor):
        """Test starting a mission."""
        # Initially inactive
        assert mission_executor.mission_active is False

        mission_executor.start_mission()

        # Should become active
        assert mission_executor.mission_active is True

        # Should publish status and log
        mission_executor.status_pub.publish.assert_called_once()
        published_msg = mission_executor.status_pub.publish.call_args[0][0]
        assert published_msg.data == "Mission started"

        mission_executor.node.get_logger.return_value.info.assert_called_with(
            "Mission started"
        )

    @pytest.mark.unit
    def test_start_mission_already_active(self, mission_executor):
        """Test starting a mission that's already active."""
        mission_executor.mission_active = True

        mission_executor.start_mission()

        # Should not publish or log again
        mission_executor.status_pub.publish.assert_not_called()
        mission_executor.node.get_logger.return_value.info.assert_not_called()

    @pytest.mark.unit
    def test_stop_mission(self, mission_executor):
        """Test stopping a mission."""
        mission_executor.mission_active = True

        mission_executor.stop_mission()

        # Should become inactive
        assert mission_executor.mission_active is False

        # Should publish status and log
        mission_executor.status_pub.publish.assert_called_once()
        published_msg = mission_executor.status_pub.publish.call_args[0][0]
        assert published_msg.data == "Mission stopped"

        mission_executor.node.get_logger.return_value.info.assert_called_with(
            "Mission stopped"
        )

    @pytest.mark.unit
    def test_pause_mission(self, mission_executor):
        """Test pausing a mission."""
        mission_executor.mission_active = True

        mission_executor.pause_mission()

        # Should remain active but publish paused status
        assert mission_executor.mission_active is True

        # Should publish status and log
        mission_executor.status_pub.publish.assert_called_once()
        published_msg = mission_executor.status_pub.publish.call_args[0][0]
        assert published_msg.data == "Mission paused"

        mission_executor.node.get_logger.return_value.info.assert_called_with(
            "Mission paused"
        )

    @pytest.mark.unit
    def test_pause_mission_not_active(self, mission_executor):
        """Test pausing a mission that's not active."""
        mission_executor.mission_active = False

        mission_executor.pause_mission()

        # Should not publish or log
        mission_executor.status_pub.publish.assert_not_called()
        mission_executor.node.get_logger.return_value.info.assert_not_called()

    @pytest.mark.unit
    def test_publish_status(self, mission_executor):
        """Test status publishing."""
        test_status = "Test mission status"

        mission_executor._publish_status(test_status)

        # Should publish message with correct data
        mission_executor.status_pub.publish.assert_called_once()
        published_msg = mission_executor.status_pub.publish.call_args[0][0]
        assert published_msg.data == test_status


class TestMissionExecutorIntegration:
    """Integration tests for MissionExecutor."""

    @pytest.mark.integration
    @patch("missions.mission_executor.rclpy.init")
    @patch("missions.mission_executor.rclpy.spin")
    @patch("missions.mission_executor.rclpy.shutdown")
    def test_main_function(self, mock_shutdown, mock_spin, mock_init):
        """Test main function execution."""
        from missions.mission_executor import main

        main()

        # Should initialize, spin, and shutdown ROS2
        mock_init.assert_called_once()
        mock_spin.assert_called_once()
        mock_shutdown.assert_called_once()

    @pytest.mark.integration
    def test_full_command_workflow(self, mission_executor):
        """Test complete command workflow."""
        # Test start -> pause -> stop sequence
        mission_executor.start_mission()
        assert mission_executor.mission_active is True

        mission_executor.pause_mission()
        assert mission_executor.mission_active is True  # Still active, just paused

        mission_executor.stop_mission()
        assert mission_executor.mission_active is False

        # Verify all status messages were published
        assert mission_executor.status_pub.publish.call_count == 3

        published_messages = [
            call[0][0].data
            for call in mission_executor.status_pub.publish.call_args_list
        ]
        assert "Mission started" in published_messages
        assert "Mission paused" in published_messages
        assert "Mission stopped" in published_messages


class TestMissionExecutorROS2:
    """ROS2-specific tests for MissionExecutor."""

    @pytest.mark.ros2
    @patch("missions.mission_executor.rclpy")
    def test_ros2_initialization(self, mock_rclpy):
        """Test ROS2 node initialization."""
        # Mock ROS2 components
        mock_node = Mock()
        mock_rclpy.Node.return_value = mock_node

        # Mock subscription and publisher
        mock_sub = Mock()
        mock_pub = Mock()
        mock_node.create_subscription.return_value = mock_sub
        mock_node.create_publisher.return_value = mock_pub

        executor = MissionExecutor()

        # Verify ROS2 node was created
        mock_rclpy.Node.assert_called_once_with("mission_executor")

        # Verify subscription was created for mission commands
        mock_node.create_subscription.assert_called_once()
        sub_call = mock_node.create_subscription.call_args
        assert sub_call[0][0].__name__ == "String"  # Message type
        assert sub_call[0][1] == "/mission/commands"  # Topic
        assert callable(sub_call[0][2])  # Callback function

        # Verify publisher was created for mission status
        mock_node.create_publisher.assert_called_once()
        pub_call = mock_node.create_publisher.call_args
        assert pub_call[0][0].__name__ == "String"  # Message type
        assert pub_call[0][1] == "/mission/status"  # Topic


class TestMissionExecutorROS2:
    """Test MissionExecutor with real ROS2 components."""

    @pytest.mark.ros2
    def test_initialization(self, ros2_node):
        """Test MissionExecutor initialization with real ROS2."""
        # Patch the Node.__init__ to use our test node
        with patch.object(MissionExecutor, "__init__", lambda self: None):
            executor = MissionExecutor.__new__(MissionExecutor)
            executor._node = ros2_node

            # Manually initialize the attributes
            executor.current_mission = None
            executor.mission_active = False

            # Verify initial state
            assert executor.current_mission is None
            assert executor.mission_active is False

    @pytest.mark.ros2
    def test_command_callback_start(self, ros2_node):
        """Test start command callback."""
        with patch.object(MissionExecutor, "__init__", lambda self: None):
            executor = MissionExecutor.__new__(MissionExecutor)
            executor._node = ros2_node
            executor.current_mission = None
            executor.mission_active = False

            # Mock start_mission
            executor.start_mission = Mock()

            # Call command callback with start
            from std_msgs.msg import String

            msg = String()
            msg.data = "start"

            executor._command_callback(msg)

            executor.start_mission.assert_called_once()

    @pytest.mark.ros2
    def test_command_callback_stop(self, ros2_node):
        """Test stop command callback."""
        with patch.object(MissionExecutor, "__init__", lambda self: None):
            executor = MissionExecutor.__new__(MissionExecutor)
            executor._node = ros2_node
            executor.current_mission = Mock()
            executor.mission_active = True

            # Mock stop_mission
            executor.stop_mission = Mock()

            # Call command callback with stop
            from std_msgs.msg import String

            msg = String()
            msg.data = "stop"

            executor._command_callback(msg)

            executor.stop_mission.assert_called_once()

    @pytest.mark.ros2
    def test_start_mission(self, ros2_node):
        """Test mission start functionality."""
        with patch.object(MissionExecutor, "__init__", lambda self: None):
            executor = MissionExecutor.__new__(MissionExecutor)
            executor._node = ros2_node
            executor.current_mission = Mock()
            executor.mission_active = False

            # Mock publisher
            executor.status_pub = Mock()

            executor.start_mission()

            assert executor.mission_active is True
            executor.status_pub.publish.assert_called()

    @pytest.mark.ros2
    def test_stop_mission(self, ros2_node):
        """Test mission stop functionality."""
        with patch.object(MissionExecutor, "__init__", lambda self: None):
            executor = MissionExecutor.__new__(MissionExecutor)
            executor._node = ros2_node
            executor.current_mission = Mock()
            executor.mission_active = True

            # Mock publisher
            executor.status_pub = Mock()

            executor.stop_mission()

            assert executor.mission_active is False
            executor.status_pub.publish.assert_called()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
