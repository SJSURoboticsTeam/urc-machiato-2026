#!/usr/bin/env python3
"""
Integration tests for PyTrees ROS2 integration.

Tests verify:
- Proper blackboard usage
- ROS2 node integration
- py_trees_ros functionality
- State machine communication
"""

import pytest
import sys
import os
from typing import Dict, Any

# Add project paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

# ROS2 imports (optional)
try:
    import rclpy
    from rclpy.node import Node
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    rclpy = None

# Try to import PyTrees components
try:
    from missions.robust_behavior_tree import (
        EnhancedBTNode, BehaviorTree, BTExecutionContext,
        ROS2ActionNode, PY_TREES_AVAILABLE, PY_TREES_ROS_AVAILABLE
    )
    PY_TREES_IMPORTED = True
except ImportError as e:
    PY_TREES_IMPORTED = False
    PY_TREES_AVAILABLE = False
    PY_TREES_ROS_AVAILABLE = False
    print(f"Warning: Could not import PyTrees components: {e}")


class TestPyTreesBlackboard:
    """Test PyTrees blackboard implementation."""
    
    @pytest.mark.skipif(not PY_TREES_IMPORTED, reason="PyTrees not available")
    def test_blackboard_creation(self):
        """Test that blackboard is created properly."""
        from missions.robust_behavior_tree import BTExecutionContext
        
        context = BTExecutionContext()
        
        # Check blackboard type
        if PY_TREES_AVAILABLE:
            from py_trees.blackboard import Blackboard
            assert isinstance(context.blackboard, Blackboard) or isinstance(context.blackboard, dict), \
                "Blackboard should be py_trees Blackboard or dict fallback"
        else:
            assert isinstance(context.blackboard, dict), \
                "Blackboard should be dict when py_trees not available"
    
    @pytest.mark.skipif(not PY_TREES_IMPORTED, reason="PyTrees not available")
    def test_blackboard_set_get(self):
        """Test blackboard set/get operations."""
        from missions.robust_behavior_tree import BTExecutionContext
        
        context = BTExecutionContext()
        
        # Test setting and getting values
        if hasattr(context.blackboard, 'set'):
            # py_trees Blackboard
            context.blackboard.set("test_key", "test_value")
            assert context.blackboard.get("test_key") == "test_value"
        else:
            # dict fallback
            context.blackboard["test_key"] = "test_value"
            assert context.blackboard["test_key"] == "test_value"
    
    @pytest.mark.skipif(not PY_TREES_IMPORTED, reason="PyTrees not available")
    def test_blackboard_initialization(self):
        """Test blackboard initialization with values."""
        from missions.robust_behavior_tree import BehaviorTree, EnhancedActionNode
        
        # Create a simple action node
        def test_action():
            return True
        
        action = EnhancedActionNode("test_action", test_action)
        
        # Create tree with blackboard
        tree = BehaviorTree("test_tree", action)
        
        # Execute with blackboard
        result = tree.execute()
        
        assert result is not None, "Tree execution should return result"
        assert hasattr(tree, 'execution_context'), \
            "Tree should have execution context"
        assert tree.execution_context is not None, \
            "Execution context should be created"


class TestPyTreesROS2Integration:
    """Test PyTrees ROS2 integration."""
    
    @pytest.mark.skipif(not PY_TREES_IMPORTED, reason="PyTrees not available")
    @pytest.mark.skipif(not PY_TREES_ROS_AVAILABLE, reason="py_trees_ros not available")
    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_ros2_action_node_creation(self):
        """Test ROS2 action node creation."""
        if not ROS2_AVAILABLE:
            pytest.skip("ROS2 not available")
        
        from rclpy.node import Node
        
        if not rclpy.ok():
            rclpy.init()
        
        try:
            node = Node("test_node")
            
            # Try to create ROS2 action node
            # Note: This requires actual action type, so we'll test the structure
            assert ROS2ActionNode is not None, \
                "ROS2ActionNode should be available"
            
            # Check that it requires node parameter
            with pytest.raises((ValueError, TypeError)):
                # Should fail without node
                ROS2ActionNode("test", None, "/test_action")
            
            node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()
    
    @pytest.mark.skipif(not PY_TREES_IMPORTED, reason="PyTrees not available")
    def test_ros2_integration_availability(self):
        """Test that ROS2 integration is properly available."""
        from missions.robust_behavior_tree import PY_TREES_ROS_AVAILABLE
        
        # Check that availability flag exists
        assert isinstance(PY_TREES_ROS_AVAILABLE, bool), \
            "PY_TREES_ROS_AVAILABLE should be boolean"
        
        if PY_TREES_ROS_AVAILABLE:
            # If available, ROS2ActionNode should be proper py_trees_ros class
            assert ROS2ActionNode is not None, \
                "ROS2ActionNode should be defined when py_trees_ros available"


class TestPyTreesBehaviorTree:
    """Test PyTrees behavior tree execution."""
    
    @pytest.mark.skipif(not PY_TREES_IMPORTED, reason="PyTrees not available")
    def test_tree_creation(self):
        """Test behavior tree creation."""
        from missions.robust_behavior_tree import (
            BehaviorTree, EnhancedActionNode
        )
        
        # Create simple action
        def success_action():
            return True
        
        action = EnhancedActionNode("test_action", success_action)
        tree = BehaviorTree("test_tree", action)
        
        assert tree.name == "test_tree", \
            "Tree should have correct name"
        assert tree.root is not None, \
            "Tree should have root node"
    
    @pytest.mark.skipif(not PY_TREES_IMPORTED, reason="PyTrees not available")
    def test_tree_execution(self):
        """Test tree execution."""
        from missions.robust_behavior_tree import (
            BehaviorTree, EnhancedActionNode
        )
        
        # Create success action
        def success_action():
            return True
        
        action = EnhancedActionNode("success", success_action)
        tree = BehaviorTree("test", action)
        
        result = tree.execute()
        
        assert result is not None, \
            "Execution should return result"
        assert hasattr(result, 'status') or hasattr(result, 'success'), \
            "Result should have status or success attribute"
    
    @pytest.mark.skipif(not PY_TREES_IMPORTED, reason="PyTrees not available")
    def test_circuit_breaker_integration(self):
        """Test circuit breaker integration."""
        from missions.robust_behavior_tree import EnhancedBTNode, BTNodeType
        
        node = EnhancedBTNode("test_node", BTNodeType.ACTION)
        
        assert hasattr(node, 'circuit_breaker'), \
            "Node should have circuit breaker"
        assert hasattr(node, 'circuit_breaker_enabled'), \
            "Node should have circuit breaker enabled flag"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
