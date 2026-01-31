#!/usr/bin/env python3
"""
Comprehensive pytest tests for BT.CPP orchestrator implementation.

Tests verify:
- LifecycleNode implementation
- QoS profile usage
- Blackboard functionality
- ROS2 service integration
- Action server functionality
"""

import pytest
import os
from typing import Dict, Any
import time

# ROS2 imports (optional - tests work without them)
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.lifecycle import LifecycleState
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    rclpy = None


class TestBTOrchestratorImplementation:
    """Test BT.CPP orchestrator implementation quality."""
    
    @pytest.fixture(scope="class")
    def ros2_context(self):
        """Initialize ROS2 context for tests (if available)."""
        if ROS2_AVAILABLE and not rclpy.ok():
            rclpy.init()
        yield
        if ROS2_AVAILABLE and rclpy.ok():
            rclpy.shutdown()
    
    def test_uses_lifecycle_node(self):
        """Test that BT orchestrator uses LifecycleNode."""
        # Check source code for LifecycleNode inheritance
        bt_file = "src/autonomy/bt/src/bt_orchestrator.cpp"
        
        assert os.path.exists(bt_file), "BT orchestrator file should exist"
        
        with open(bt_file, 'r') as f:
            content = f.read()
        
        # Check for LifecycleNode
        assert "rclcpp_lifecycle::LifecycleNode" in content, \
            "BT orchestrator should inherit from LifecycleNode"
        assert "on_configure" in content, \
            "Should implement on_configure lifecycle callback"
        assert "on_activate" in content, \
            "Should implement on_activate lifecycle callback"
        assert "on_deactivate" in content, \
            "Should implement on_deactivate lifecycle callback"
    
    def test_uses_qos_profiles(self):
        """Test that BT orchestrator uses QoS profiles."""
        bt_file = "src/autonomy/bt/src/bt_orchestrator.cpp"
        
        with open(bt_file, 'r') as f:
            content = f.read()
        
        # Check for QoS usage
        assert "get_autonomy_status_qos" in content or "QoS" in content, \
            "Should use QoS profiles for subscriptions"
        assert "create_subscription" in content, \
            "Should create subscriptions with QoS"
    
    def test_uses_callback_groups(self):
        """Test that BT orchestrator uses callback groups."""
        bt_file = "src/autonomy/bt/src/bt_orchestrator.cpp"
        
        with open(bt_file, 'r') as f:
            content = f.read()
        
        # Check for callback groups
        assert "CallbackGroup" in content or "callback_group" in content, \
            "Should use callback groups for priority scheduling"
    
    def test_blackboard_initialization(self):
        """Test blackboard initialization."""
        bt_file = "src/autonomy/bt/src/bt_orchestrator.cpp"
        
        with open(bt_file, 'r') as f:
            content = f.read()
        
        # Check for blackboard initialization
        assert "Blackboard::create()" in content, \
            "Should create blackboard instance"
        assert "set_blackboard_value" in content or 'blackboard_->set' in content, \
            "Should initialize blackboard values"
        
        # Check for required blackboard variables
        required_vars = ["mission_active", "robot_x", "robot_y", "samples_collected"]
        for var in required_vars:
            assert f'"{var}"' in content or f"'{var}'" in content, \
                f"Should initialize {var} in blackboard"
    
    def test_ros2_service_integration(self):
        """Test ROS2 service integration."""
        bt_file = "src/autonomy/bt/src/bt_orchestrator.cpp"
        
        with open(bt_file, 'r') as f:
            content = f.read()
        
        # Check for service client usage
        assert "create_client" in content, \
            "Should create ROS2 service clients"
        assert "wait_for_service" in content, \
            "Should wait for service availability"
        assert "async_send_request" in content, \
            "Should use async service calls"
    
    def test_action_server_implementation(self):
        """Test action server implementation."""
        bt_file = "src/autonomy/bt/src/bt_orchestrator.cpp"
        
        with open(bt_file, 'r') as f:
            content = f.read()
        
        # Check for action server
        assert "create_server" in content or "rclcpp_action::create_server" in content, \
            "Should create action servers"
        assert "NavigateToPose" in content, \
            "Should implement NavigateToPose action"
        assert "ExecuteMission" in content, \
            "Should implement ExecuteMission action"
    
    def test_events_executor_usage(self, ros2_context):
        """Test Events Executor usage (Jazzy feature)."""
        bt_file = "src/autonomy/bt/src/bt_orchestrator.cpp"
        
        with open(bt_file, 'r') as f:
            content = f.read()
        
        # Check for Events Executor (with fallback)
        assert "EventsExecutor" in content or "events_executor" in content, \
            "Should use Events Executor for deterministic scheduling"
    
    def test_file_structure(self):
        """Test that required files exist."""
        required_files = [
            "src/autonomy/bt/src/bt_orchestrator.cpp",
            "src/autonomy/bt/CMakeLists.txt",
            "src/autonomy/bt/package.xml"
        ]
        
        for file_path in required_files:
            assert os.path.exists(file_path), f"Required file missing: {file_path}"
    
    def test_cmake_configuration(self, ros2_context):
        """Test CMakeLists.txt configuration."""
        cmake_file = "src/autonomy/bt/CMakeLists.txt"
        
        with open(cmake_file, 'r') as f:
            content = f.read()
        
        # Check for required dependencies
        assert "behaviortree_cpp" in content, \
            "Should link against behaviortree_cpp"
        assert "rclcpp_lifecycle" in content, \
            "Should link against rclcpp_lifecycle"
        assert "rclcpp" in content, \
            "Should link against rclcpp"


class TestPyTreesImplementation:
    """Test PyTrees implementation quality."""
    
    def test_uses_py_trees_blackboard(self):
        """Test that PyTrees uses proper Blackboard."""
        bt_file = "missions/robust_behavior_tree.py"
        
        assert os.path.exists(bt_file), "PyTrees file should exist"
        
        with open(bt_file, 'r') as f:
            content = f.read()
        
        # Check for proper blackboard import
        assert "from py_trees.blackboard import Blackboard" in content, \
            "Should import py_trees Blackboard"
        assert "Blackboard()" in content, \
            "Should create Blackboard instances"
    
    def test_ros2_integration_available(self):
        """Test that ROS2 integration is available."""
        bt_file = "missions/robust_behavior_tree.py"
        
        with open(bt_file, 'r') as f:
            content = f.read()
        
        # Check for py_trees_ros integration
        assert "py_trees_ros" in content, \
            "Should import py_trees_ros for ROS2 integration"
        assert "ActionClient" in content or "ROS2ActionNode" in content, \
            "Should have ROS2 action client support"
    
    def test_fallback_implementation(self):
        """Test fallback implementation when py_trees not available."""
        bt_file = "missions/robust_behavior_tree.py"
        
        with open(bt_file, 'r') as f:
            content = f.read()
        
        # Check for fallback
        assert "PY_TREES_AVAILABLE" in content, \
            "Should check for py_trees availability"
        assert "fallback" in content.lower() or "dict" in content, \
            "Should have fallback implementation"
    
    def test_circuit_breaker_integration(self):
        """Test circuit breaker integration."""
        bt_file = "missions/robust_behavior_tree.py"
        
        with open(bt_file, 'r') as f:
            content = f.read()
        
        # Check for circuit breaker
        assert "circuitbreaker" in content or "CircuitBreaker" in content, \
            "Should integrate circuit breaker for robustness"
    
    def test_monitoring_metrics(self):
        """Test monitoring and metrics."""
        bt_file = "missions/robust_behavior_tree.py"
        
        with open(bt_file, 'r') as f:
            content = f.read()
        
        # Check for monitoring
        assert "execution_count" in content or "performance_metrics" in content, \
            "Should track execution metrics"
        assert "success_count" in content or "failure_count" in content, \
            "Should track success/failure counts"


class TestBTIntegration:
    """Test BT integration with ROS2 system."""
    
    def test_state_machine_service_call(self):
        """Test that BT calls state machine service."""
        bt_file = "src/autonomy/bt/src/bt_orchestrator.cpp"
        
        with open(bt_file, 'r') as f:
            content = f.read()
        
        # Check for state machine service call
        assert "/adaptive_state_machine/get_state" in content, \
            "Should call state machine service"
        assert "GetSystemState" in content, \
            "Should use GetSystemState service type"
    
    def test_blackboard_updates_from_topics(self, ros2_context):
        """Test blackboard updates from ROS2 topics."""
        bt_file = "src/autonomy/bt/src/bt_orchestrator.cpp"
        
        with open(bt_file, 'r') as f:
            content = f.read()
        
        # Check for topic subscriptions
        assert "/odom" in content, \
            "Should subscribe to /odom for position updates"
        assert "/slam/pose" in content, \
            "Should subscribe to /slam/pose for SLAM updates"
        assert "odom_callback" in content, \
            "Should have odometry callback"
        assert "slam_pose_callback" in content, \
            "Should have SLAM pose callback"
    
    def test_telemetry_publishing(self):
        """Test telemetry publishing."""
        bt_file = "src/autonomy/bt/src/bt_orchestrator.cpp"
        
        with open(bt_file, 'r') as f:
            content = f.read()
        
        # Check for telemetry
        assert "/bt/telemetry" in content, \
            "Should publish BT telemetry"
        assert "telemetry" in content.lower(), \
            "Should have telemetry functionality"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
