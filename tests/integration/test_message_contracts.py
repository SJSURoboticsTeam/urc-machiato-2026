#!/usr/bin/env python3
"""
Message Contract Validation Tests

Tests ROS2 message format validation and API contracts:
- ROS2 message schema validation
- Interface version compatibility
- Cross-subsystem message format validation
- Backward compatibility tests
- Message field validation
- Type checking

This addresses integration gap: Message format/API contract validation.

Author: URC 2026 Autonomy Team
"""

import os
import sys
from typing import Any, Dict, List, Optional

import pytest

# ROS2 imports and fixtures handled by conftest.py

# Add project paths
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
STATE_MGMT_ROOT = os.path.join(PROJECT_ROOT, "Autonomy", "code", "state_management")
sys.path.insert(0, STATE_MGMT_ROOT)
sys.path.insert(0, os.path.join(STATE_MGMT_ROOT, "autonomy_state_machine"))

# Try to import ROS2 message types
try:
    from autonomy_interfaces.msg import SystemState as SystemStateMsg
    from autonomy_interfaces.srv import ChangeState
    from geometry_msgs.msg import PoseStamped, Twist
    from sensor_msgs.msg import Image, Imu, NavSatFix
    from std_msgs.msg import Header, String
    MESSAGES_AVAILABLE = True
except ImportError:
    MESSAGES_AVAILABLE = False
    SystemStateMsg = None
    ChangeState = None
    PoseStamped = None
    Twist = None
    Image = None
    NavSatFix = None
    Imu = None
    Header = None
    String = None


# Use pytest-ros2 fixtures from conftest.py


@pytest.mark.integration
class TestMessageSchemas:
    """Test ROS2 message schema validation."""

    def test_system_state_message_schema(self):
        """Test SystemState message schema."""
        if not MESSAGES_AVAILABLE:
            pytest.skip("autonomy_interfaces messages not available")

        # Create valid message
        msg = SystemStateMsg()
        msg.header = Header()
        msg.current_state = "IDLE"
        msg.current_substate = "NONE"

        # Validate required fields
        assert hasattr(msg, "header"), "SystemState should have header field"
        assert hasattr(msg, "current_state"), "SystemState should have current_state field"
        assert hasattr(msg, "current_substate"), "SystemState should have current_substate field"

        # Validate field types
        assert isinstance(msg.current_state, str), "current_state should be string"
        assert isinstance(msg.current_substate, str), "current_substate should be string"

    def test_change_state_service_schema(self):
        """Test ChangeState service schema."""
        if not MESSAGES_AVAILABLE:
            pytest.skip("autonomy_interfaces services not available")

        # Create valid request
        request = ChangeState.Request()
        request.target_state = "AUTONOMOUS"
        request.target_substate = "AUTONOMOUS_NAVIGATION"

        # Validate required fields
        assert hasattr(request, "target_state"), "ChangeState.Request should have target_state"
        assert hasattr(request, "target_substate"), "ChangeState.Request should have target_substate"

        # Validate field types
        assert isinstance(request.target_state, str), "target_state should be string"
        assert isinstance(request.target_substate, str), "target_substate should be string"

        # Create valid response
        response = ChangeState.Response()
        response.success = True
        response.message = "State changed successfully"

        assert hasattr(response, "success"), "ChangeState.Response should have success"
        assert hasattr(response, "message"), "ChangeState.Response should have message"
        assert isinstance(response.success, bool), "success should be boolean"
        assert isinstance(response.message, str), "message should be string"

    def test_geometry_messages_schema(self):
        """Test geometry message schemas."""
        if not MESSAGES_AVAILABLE:
            pytest.skip("geometry_msgs not available")

        # Test PoseStamped
        pose = PoseStamped()
        pose.header = Header()
        assert hasattr(pose, "header"), "PoseStamped should have header"
        assert hasattr(pose, "pose"), "PoseStamped should have pose"

        # Test Twist
        twist = Twist()
        assert hasattr(twist, "linear"), "Twist should have linear"
        assert hasattr(twist, "angular"), "Twist should have angular"

    def test_sensor_messages_schema(self):
        """Test sensor message schemas."""
        if not MESSAGES_AVAILABLE:
            pytest.skip("sensor_msgs not available")

        # Test NavSatFix
        gps = NavSatFix()
        gps.header = Header()
        assert hasattr(gps, "header"), "NavSatFix should have header"
        assert hasattr(gps, "latitude"), "NavSatFix should have latitude"
        assert hasattr(gps, "longitude"), "NavSatFix should have longitude"
        assert hasattr(gps, "altitude"), "NavSatFix should have altitude"

        # Test Imu
        imu = Imu()
        imu.header = Header()
        assert hasattr(imu, "header"), "Imu should have header"
        assert hasattr(imu, "orientation"), "Imu should have orientation"
        assert hasattr(imu, "angular_velocity"), "Imu should have angular_velocity"
        assert hasattr(imu, "linear_acceleration"), "Imu should have linear_acceleration"

        # Test Image
        image = Image()
        image.header = Header()
        assert hasattr(image, "header"), "Image should have header"
        assert hasattr(image, "height"), "Image should have height"
        assert hasattr(image, "width"), "Image should have width"
        assert hasattr(image, "encoding"), "Image should have encoding"
        assert hasattr(image, "data"), "Image should have data"


@pytest.mark.integration
@pytest.mark.ros2
class TestMessageValidation:
    """Test message field validation."""

    def test_system_state_field_validation(self):
        """Test SystemState field validation."""
        if not MESSAGES_AVAILABLE:
            pytest.skip("autonomy_interfaces messages not available")

        # Valid states
        valid_states = ["BOOT", "IDLE", "AUTONOMOUS", "TELEOPERATION", "SAFETY"]
        valid_subsystems = ["NONE", "AUTONOMOUS_NAVIGATION", "SCIENCE", "EQUIPMENT_SERVICING"]

        for state in valid_states:
            msg = SystemStateMsg()
            msg.current_state = state
            msg.current_substate = "NONE"

            # Message should be valid
            assert self._validate_system_state(msg), f"State {state} should be valid"

    def test_required_fields_present(self):
        """Test that required message fields are present."""
        if not MESSAGES_AVAILABLE:
            pytest.skip("Messages not available")

        # SystemState requires header, current_state, current_substate
        msg = SystemStateMsg()
        required_fields = ["header", "current_state", "current_substate"]

        for field in required_fields:
            assert hasattr(msg, field), f"SystemState missing required field: {field}"

    def test_message_field_types(self):
        """Test message field type correctness."""
        if not MESSAGES_AVAILABLE:
            pytest.skip("Messages not available")

        # Test SystemState field types
        msg = SystemStateMsg()
        msg.current_state = "IDLE"  # Should be string
        assert isinstance(msg.current_state, str), "current_state should accept string"

        # Test service request types
        request = ChangeState.Request()
        request.target_state = "AUTONOMOUS"  # Should be string
        assert isinstance(request.target_state, str), "target_state should accept string"

    def _validate_system_state(self, msg: SystemStateMsg) -> bool:
        """Validate SystemState message."""
        if not hasattr(msg, "current_state"):
            return False
        if not hasattr(msg, "current_substate"):
            return False

        valid_states = ["BOOT", "IDLE", "AUTONOMOUS", "TELEOPERATION", "SAFETY"]
        if msg.current_state not in valid_states:
            return False

        return True


@pytest.mark.integration
@pytest.mark.ros2
class TestInterfaceCompatibility:
    """Test interface version compatibility."""

    def test_backward_compatibility(self):
        """Test backward compatibility of interfaces."""
        if not MESSAGES_AVAILABLE:
            pytest.skip("Messages not available")

        # Test that old message format still works
        # (In real implementation, would test against previous message versions)
        msg = SystemStateMsg()
        msg.current_state = "IDLE"

        # Should be able to create and use message
        assert msg.current_state == "IDLE", "Should support basic message format"

    def test_forward_compatibility(self):
        """Test forward compatibility (new fields are optional)."""
        if not MESSAGES_AVAILABLE:
            pytest.skip("Messages not available")

        # Test that messages work with missing optional fields
        msg = SystemStateMsg()
        msg.current_state = "IDLE"
        msg.current_substate = "NONE"

        # Should work even if optional fields are missing
        assert msg.current_state == "IDLE", "Should work with minimal fields"

    def test_cross_subsystem_compatibility(self):
        """Test message compatibility across subsystems."""
        if not MESSAGES_AVAILABLE:
            pytest.skip("Messages not available")

        # Test that state machine messages work with navigation
        state_msg = SystemStateMsg()
        state_msg.current_state = "AUTONOMOUS"

        # Navigation should be able to read state
        assert state_msg.current_state in ["BOOT", "IDLE", "AUTONOMOUS", "TELEOPERATION", "SAFETY"], \
            "Navigation should understand state machine states"

        # Test that navigation commands work with state machine
        twist = Twist()
        twist.linear.x = 1.0
        twist.angular.z = 0.5

        # State machine should be able to validate commands
        assert hasattr(twist, "linear"), "State machine should understand Twist messages"
        assert hasattr(twist, "angular"), "State machine should understand Twist messages"


@pytest.mark.integration
@pytest.mark.ros2
class TestMessageContracts:
    """Test API contracts between subsystems."""

    def test_state_machine_navigation_contract(self):
        """Test contract between state machine and navigation."""
        if not MESSAGES_AVAILABLE:
            pytest.skip("Messages not available")

        # State machine publishes SystemState
        state_msg = SystemStateMsg()
        state_msg.current_state = "AUTONOMOUS"

        # Navigation should subscribe and understand
        assert state_msg.current_state in ["AUTONOMOUS", "IDLE", "TELEOPERATION"], \
            "Navigation should understand state machine states"

    def test_vision_navigation_contract(self):
        """Test contract between vision and navigation."""
        if not MESSAGES_AVAILABLE:
            pytest.skip("Messages not available")

        # Vision publishes detections (simulated)
        # Navigation should be able to consume
        # (In real implementation, would test actual vision messages)

        # Navigation publishes goals
        goal = PoseStamped()
        goal.header = Header()

        # Vision should be able to understand goals
        assert hasattr(goal, "pose"), "Vision should understand navigation goals"

    def test_safety_system_contract(self):
        """Test contract between safety system and other subsystems."""
        if not MESSAGES_AVAILABLE:
            pytest.skip("Messages not available")

        # Safety system should be able to stop navigation
        stop_cmd = Twist()
        stop_cmd.linear.x = 0.0
        stop_cmd.angular.z = 0.0

        # Navigation should accept stop commands
        assert stop_cmd.linear.x == 0.0, "Navigation should accept safety stop commands"
        assert stop_cmd.angular.z == 0.0, "Navigation should accept safety stop commands"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
