"""
Unit tests for blackboard callback functionality.
"""
import os
import sys
import time

import pytest
from unittest.mock import Mock, patch

# Add source paths
_here = os.path.dirname(os.path.abspath(__file__))
_src = os.path.abspath(os.path.join(_here, "..", "..", "..", "shared"))
if _src not in sys.path:
    sys.path.insert(0, _src)

# Import the module directly
sys.path.insert(0, os.path.join(_src, "core"))
from unified_blackboard_client import UnifiedBlackboardClient


class TestBlackboardCallbacks:
    """Test blackboard callback functionality."""

    def setup_method(self):
        """Set up test fixtures."""
        # Mock rclpy and ROS2 services
        with patch.dict('sys.modules', {
            'rclpy': Mock(),
            'rclpy.node': Mock(),
            'autonomy_interfaces.srv': Mock()
        }):
            # Mock the ROS2 node and service clients
            mock_node = Mock()
            mock_get_client = Mock()
            mock_set_client = Mock()
            mock_node.create_client.return_value = mock_get_client
            mock_node.create_client.side_effect = [mock_get_client, mock_set_client]
            
            with patch('rclpy.node.Node', return_value=mock_node):
                self.client = UnifiedBlackboardClient(mock_node)

    def test_subscribe_exact_key(self):
        """Test subscribing to exact key changes."""
        callback = Mock()
        unsubscribe = self.client.subscribe("test.key", callback)
        
        # Simulate a value change
        self.client.set("test.key", "new_value")
        
        # Callback should be called
        callback.assert_called_once_with("test.key", "new_value")
        
        # Unsubscribe should work
        unsubscribe()
        callback.reset_mock()
        self.client.set("test.key", "another_value")
        callback.assert_not_called()

    def test_subscribe_prefix(self):
        """Test subscribing to key prefix changes."""
        callback = Mock()
        unsubscribe = self.client.subscribe_prefix("test.", callback)
        
        # Simulate value changes for matching keys
        self.client.set("test.key1", "value1")
        self.client.set("test.key2", "value2")
        self.client.set("other.key", "value3")  # Should not match
        
        # Callback should be called twice
        assert callback.call_count == 2
        callback.assert_any_call("test.key1", "value1")
        callback.assert_any_call("test.key2", "value2")
        
        # Unsubscribe should work
        unsubscribe()
        callback.reset_mock()
        self.client.set("test.key3", "value4")
        callback.assert_not_called()

    def test_multiple_subscribers(self):
        """Test multiple subscribers to the same key."""
        callback1 = Mock()
        callback2 = Mock()
        
        unsubscribe1 = self.client.subscribe("test.key", callback1)
        unsubscribe2 = self.client.subscribe("test.key", callback2)
        
        # Simulate a value change
        self.client.set("test.key", "new_value")
        
        # Both callbacks should be called
        callback1.assert_called_once_with("test.key", "new_value")
        callback2.assert_called_once_with("test.key", "new_value")
        
        # Unsubscribe one should leave the other
        unsubscribe1()
        callback1.reset_mock()
        self.client.set("test.key", "another_value")
        callback1.assert_not_called()
        callback2.assert_called_once_with("test.key", "another_value")

    def test_no_callback_on_same_value(self):
        """Test that callbacks are not triggered when value doesn't change."""
        callback = Mock()
        unsubscribe = self.client.subscribe("test.key", callback)
        
        # Set initial value
        self.client.set("test.key", "initial_value")
        callback.assert_called_once_with("test.key", "initial_value")
        
        # Set same value again - should not trigger callback
        callback.reset_mock()
        self.client.set("test.key", "initial_value")
        callback.assert_not_called()
        
        # Set different value - should trigger callback
        self.client.set("test.key", "new_value")
        callback.assert_called_once_with("test.key", "new_value")
        
        unsubscribe()

    def test_callback_error_handling(self):
        """Test that errors in callbacks don't break the system."""
        def faulty_callback(key, value):
            raise ValueError("Callback error")
        
        good_callback = Mock()
        
        self.client.subscribe("test.key", faulty_callback)
        self.client.subscribe("test.key", good_callback)
        
        # Despite the faulty callback, the good one should still be called
        self.client.set("test.key", "new_value")
        
        good_callback.assert_called_once_with("test.key", "new_value")

    def test_get_methods_still_work(self):
        """Test that existing get methods still work after adding callbacks."""
        # Test all convenience methods
        self.client.set("bool_key", True)
        self.client.set("int_key", 42)
        self.client.set("double_key", 3.14)
        self.client.set("string_key", "test")
        
        assert self.client.get_bool("bool_key") is True
        assert self.client.get_int("int_key") == 42
        assert abs(self.client.get_double("double_key") - 3.14) < 0.001
        assert self.client.get_string("string_key") == "test"


if __name__ == "__main__":
    pytest.main([__file__, "-v"])