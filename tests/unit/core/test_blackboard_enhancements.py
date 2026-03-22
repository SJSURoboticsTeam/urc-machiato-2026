"""
Unit tests for blackboard enhancements: callbacks, schema validation, metrics, auditing, batch operations.
"""
import sys
import os
import time
from unittest.mock import Mock

# Add source paths
_here = os.path.dirname(os.path.abspath(__file__))
_src = os.path.abspath(os.path.join(_here, "..", "..", "..", "shared"))
if _src not in sys.path:
    sys.path.insert(0, _src)

# Import the module directly - we'll mock the ROS2 dependencies in the test
sys.path.insert(0, os.path.join(_src, "core"))

# Mock the ROS2 dependencies before importing our module
# Create mocks for ROS2
class MockRCLPY:
    class Node:
        def __init__(self, *args, **kwargs):
            pass
        def create_client(self, srv_type, srv_name):
            # Return mock service clients
            if srv_type.__name__ == 'GetBlackboardValue':
                return mock_get_client
            elif srv_type.__name__ == 'SetBlackboardValue':
                return mock_set_client
            else:
                return Mock()

# We need to mock spin_until_future_complete to do nothing but allow the code to proceed
def mock_spin_until_future_complete(node, future, timeout_sec):
    # Simulate the future completing immediately with a success response
    if hasattr(future, '_set_result'):
        future._set_result(Mock(success=True, value="test_value", value_type="string", error_message=""))
    else:
        future._result = Mock(success=True, value="test_value", value_type="string", error_message="")
        future._done = True

# Mock the service types by directly modifying the sys.modules
class MockGetBlackboardValue:
    class Request:
        def __init__(self):
            self.key = ""
            self.value_type = ""
    class Response:
        def __init__(self):
            self.success = True
            self.value = "test_value"
            self.value_type = "string"
            self.error_message = ""

class MockSetBlackboardValue:
    class Request:
        def __init__(self):
            self.key = ""
            self.value_type = ""
            self.value = ""
    class Response:
        def __init__(self):
            self.success = True

# Mock the service request and response instances
mock_get_request = MockGetBlackboardValue.Request()
mock_get_response = MockGetBlackboardValue.Response()
mock_set_request = MockSetBlackboardValue.Request()
mock_set_response = MockSetBlackboardValue.Response()

# Configure the responses to simulate success by default
mock_get_response.success = True
mock_get_response.value = "test_value"
mock_get_response.value_type = "string"
mock_get_response.error_message = ""

mock_set_response.success = True

# Mock the service clients
mock_get_client = Mock()
mock_set_client = Mock()

# Configure the service clients' call_async to return a mock future
class MockFuture:
    def __init__(self, response=None):
        self._response = response or Mock(success=True, value="test_value", value_type="string", error_message="")
        self._done = True
    
    def done(self):
        return self._done
    
    def result(self):
        return self._response

mock_get_client.call_async = Mock(return_value=MockFuture())
mock_set_client.call_async = Mock(return_value=MockFuture())

# Mock the node's create_client method to return our mock clients
mock_node_instance = Mock()
mock_node_instance.create_client = Mock(side_effect=[mock_get_client, mock_set_client])

# Apply the mocks for rclpy
sys.modules['rclpy'] = MockRCLPY()
sys.modules['rclpy.node'] = MockRCLPY()
sys.modules['rclpy.spin_until_future_complete'] = Mock(side_effect=mock_spin_until_future_complete)

# Create a mock for autonomy_interfaces and its srv submodule
# We need to create the full module hierarchy
class MockAutonomyInterfaces:
    pass

class MockAutonomyInterfacesSrv:
    pass

# Set up the mock hierarchy
mock_autonomy_interfaces = MockAutonomyInterfaces()
mock_autonomy_interfaces_srv = MockAutonomyInterfacesSrv()

# Attach the service types to the srv submodule
mock_autonomy_interfaces_srv.GetBlackboardValue = MockGetBlackboardValue
mock_autonomy_interfaces_srv.SetBlackboardValue = MockSetBlackboardValue

# Attach the srv submodule to the main module
mock_autonomy_interfaces.srv = mock_autonomy_interfaces_srv

# Apply the mocks
sys.modules['autonomy_interfaces'] = mock_autonomy_interfaces
sys.modules['autonomy_interfaces.srv'] = mock_autonomy_interfaces_srv

# Now we can import our module
from unified_blackboard_client import UnifiedBlackboardClient

class TestBlackboardEnhancements:
    """Test blackboard enhancement functionality."""

    def setup_method(self):
        """Set up test fixtures."""
        self.mock_node = MockRCLPY.Node()
        self.client = UnifiedBlackboardClient(self.mock_node)

    def test_callbacks(self):
        """Test callback functionality."""
        callback = Mock()
        unsubscribe = self.client.subscribe("test.key", callback)
        
        # Set a value
        self.client.set("test.key", "new_value")
        
        # Callback should be called
        assert callback.called, "Callback should have been called"
        callback.assert_called_once_with("test.key", "new_value")
        
        # Unsubscribe and test again
        callback.reset_mock()
        unsubscribe()
        self.client.set("test.key", "another_value")
        assert not callback.called, "Callback should not have been called after unsubscribe"

    def test_prefix_callbacks(self):
        """Test prefix-based callbacks."""
        callback = Mock()
        unsubscribe = self.client.subscribe_prefix("sensor.", callback)
        
        # Set values
        self.client.set("sensor.temperature", 25.0)
        self.client.set("sensor.humidity", 60.0)
        self.client.set("system.status", "ok")  # Should not match
        
        # Should have been called twice
        assert callback.call_count == 2, f"Expected 2 calls, got {callback.call_count}"
        callback.assert_any_call("sensor.temperature", 25.0)
        callback.assert_any_call("sensor.humidity", 60.0)
        
        # Unsubscribe test
        callback.reset_mock()
        unsubscribe()
        self.client.set("sensor.pressure", 101.3)
        assert not callback.called, "Callback should not have been called after unsubscribe"

    def test_schema_validation(self):
        """Test schema validation."""
        # Define schema for battery level (0-100)
        self.client.define_schema("battery_level", "double", 
                                validator=lambda x: 0 <= x <= 100,
                                error_msg="Battery level must be between 0 and 100")
        
        # Test valid value
        result = self.client.set("battery_level", 85.0)
        assert result, "Setting valid battery level should succeed"
        
        # Test invalid value (too high)
        result = self.client.set("battery_level", 150.0)
        assert not result, "Setting invalid battery level (too high) should fail"
        
        # Test invalid value (too low)
        result = self.client.set("battery_level", -10.0)
        assert not result, "Setting invalid battery level (too low) should fail"
        
        # Test invalid type
        result = self.client.set("battery_level", "invalid")
        assert not result, "Setting battery level to string should fail"

    def test_get_methods(self):
        """Test typed getter methods."""
        # Set values of different types
        self.client.set("bool_key", True)
        self.client.set("int_key", 42)
        self.client.set("double_key", 3.14)
        self.client.set("string_key", "hello world")
        
        # Test getters
        assert self.client.get_bool("bool_key") is True, "Boolean getter failed"
        assert self.client.get_int("int_key") == 42, "Integer getter failed"
        assert abs(self.client.get_double("double_key") - 3.14) < 0.001, "Double getter failed"
        assert self.client.get_string("string_key") == "hello world", "String getter failed"

    def test_metrics(self):
        """Test metrics functionality."""
        # Perform some operations
        self.client.set("metrics.test1", "value1")
        self.client.set("metrics.test2", 123)
        self.client.get("metrics.test1", "default")
        self.client.get("metrics.test2", 0)
        self.client.get("nonexistent.key", "default")  # Will go to service
        
        # Get metrics
        metrics = self.client.get_metrics()
        
        # Verify structure
        assert "access_count" in metrics
        assert "error_count" in metrics
        assert "latency_stats" in metrics
        assert "batch_operations" in metrics
        
        # Check that we have some access counts
        total_accesses = sum(metrics["access_count"].values())
        assert total_accesses > 0, "Should have recorded some accesses"

    def test_auditing(self):
        """Test auditing functionality."""
        # Enable auditing
        self.client.enable_auditing(True)
        
        # Perform some operations
        self.client.set("audit.test", "value")
        self.client.get("audit.test", "default")
        self.client.get("nonexistent.key", "default")  # This will fail but still be audited
        
        # Get audit log
        audit_log = self.client.get_audit_log()
        
        # Should have at least 3 entries
        assert len(audit_log) >= 3, f"Expected at least 3 audit entries, got {len(audit_log)}"
        
        # Check that the entries have the expected structure
        for entry in audit_log:
            assert 'timestamp' in entry
            assert 'operation' in entry
            assert 'key' in entry
            assert 'success' in entry
        
        # Clear audit log
        self.client.clear_audit_log()
        audit_log = self.client.get_audit_log()
        assert len(audit_log) == 0, "Audit log should be empty after clearing"

    def test_callback_error_handling(self):
        """Test that errors in callbacks don't break the system."""
        def faulty_callback(key, value):
            raise ValueError("Intentional error in callback")
        
        good_callback = Mock()
        
        self.client.subscribe("test.key", faulty_callback)
        self.client.subscribe("test.key", good_callback)
        
        # Despite the faulty callback, the good one should still be called
        self.client.set("test.key", "error_test_value")
        
        assert good_callback.called, "Good callback should have been called despite faulty callback"
        good_callback.assert_called_once_with("test.key", "error_test_value")

    def test_batch_operations(self):
        """Test batch operation functionality."""
        # Enable batch operations
        self.client.enable_batch_operations(True)
        
        # Queue some sets
        self.client.set("batch.key1", "value1")
        self.client.set("batch.key2", 42)
        self.client.set("batch.key3", 3.14)
        
        # Check that they're pending
        metrics = self.client.get_metrics()
        assert metrics["batch_operations"]["pending_sets"] == 3
        
        # Flush the batch
        flushed = self.client.flush_batch()
        assert flushed == 3, f"Expected 3 flushed operations, got {flushed}"
        
        # Check that they're no longer pending
        metrics = self.client.get_metrics()
        assert metrics["batch_operations"]["pending_sets"] == 0
        
        # Disable batch operations and test immediate mode
        self.client.enable_batch_operations(False)
        self.client.set("immediate.key", "immediate_value")
        
        # Should have been sent immediately (we can't easily test this without mocking the service call,
        # but we can at least verify it doesn't break)
        assert True  # If we got here without exception, it worked

if __name__ == "__main__":
    import pytest
    pytest.main([__file__, "-v"])