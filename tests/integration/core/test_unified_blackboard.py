#!/usr/bin/env python3
"""
Integration tests for unified blackboard system.

Tests the unified blackboard where BT.CPP is the single source of truth,
accessed by Python code via ROS2 services.

Author: URC 2026 Test Suite
"""
import sys
import os
import importlib.util

# Resolve project src and ensure it is first so core/ and autonomy_interfaces resolve
_here = os.path.dirname(os.path.abspath(__file__))
_src = os.path.abspath(os.path.join(_here, "..", "..", "..", "src"))
if _src not in sys.path:
    sys.path.insert(0, _src)

import pytest
import time

try:
    import rclpy
    from rclpy.node import Node
    from autonomy_interfaces.srv import GetBlackboardValue, SetBlackboardValue

    _BLACKBOARD_DEPS_AVAILABLE = True
except ImportError as e:
    _BLACKBOARD_DEPS_AVAILABLE = False
    _BLACKBOARD_IMPORT_ERROR = e

if not _BLACKBOARD_DEPS_AVAILABLE:
    pytest.skip(
        f"autonomy_interfaces/rclpy not available: {_BLACKBOARD_IMPORT_ERROR}",
        allow_module_level=True,
    )

# Load UnifiedBlackboardClient from project src (avoids shadowing by other "core" packages)
_client_path = os.path.join(_src, "core", "unified_blackboard_client.py")
_spec = importlib.util.spec_from_file_location(
    "unified_blackboard_client", _client_path
)
_client_mod = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_client_mod)
UnifiedBlackboardClient = _client_mod.UnifiedBlackboardClient


@pytest.mark.live_blackboard
class TestUnifiedBlackboard:
    """Test unified blackboard functionality (requires bt_orchestrator running)."""

    @pytest.fixture(scope="class")
    def ros2_context(self):
        """Initialize ROS2 context for tests (skip if conftest already inited)."""
        already_ok = rclpy.utilities.ok()
        if not already_ok:
            rclpy.init()
        yield
        if not already_ok and rclpy.utilities.ok():
            rclpy.shutdown()

    @pytest.fixture
    def test_node(self, ros2_context):
        """Create test node."""
        node = Node("test_blackboard_node")
        yield node
        node.destroy_node()

    @pytest.fixture
    def blackboard_client(self, test_node):
        """Create unified blackboard client with longer get timeout for live service."""
        client = UnifiedBlackboardClient(
            test_node,
            cache_ttl=0.01,  # Short TTL for testing
            get_timeout_sec=3.0,  # Allow time for cross-process service round-trip
        )
        # Ensure discovery is complete before first request (avoids first request being lost)
        assert client.get_client.wait_for_service(
            timeout_sec=5.0
        ), "get_value service not available"
        assert client.set_client.wait_for_service(
            timeout_sec=5.0
        ), "set_value service not available"
        time.sleep(0.3)  # Brief settle after discovery so first get/set reaches server
        return client

    @pytest.mark.integration
    def test_blackboard_service_available(self, test_node):
        """Test that blackboard services are available."""
        get_client = test_node.create_client(
            GetBlackboardValue, "/blackboard/get_value"
        )
        set_client = test_node.create_client(
            SetBlackboardValue, "/blackboard/set_value"
        )
        # bt_orchestrator creates these services only after lifecycle configure
        lifecycle_hint = (
            " Ensure bt_orchestrator is running and lifecycle is configured: "
            "ros2 lifecycle set /bt_orchestrator configure && "
            "ros2 lifecycle set /bt_orchestrator activate"
        )
        assert get_client.wait_for_service(timeout_sec=3.0), (
            "GetBlackboardValue service not available." + lifecycle_hint
        )
        assert set_client.wait_for_service(timeout_sec=3.0), (
            "SetBlackboardValue service not available." + lifecycle_hint
        )

    @pytest.mark.integration
    def test_blackboard_get_default_values(self, blackboard_client):
        """Test getting default blackboard values."""
        # These should be initialized by BT orchestrator
        robot_x = blackboard_client.get_double("robot_x", 0.0)
        assert isinstance(robot_x, float)

        mission_active = blackboard_client.get_bool("mission_active", False)
        assert isinstance(mission_active, bool)

        sensors_ok = blackboard_client.get_bool("sensors_ok", True)
        assert isinstance(sensors_ok, bool)

    @pytest.mark.integration
    def test_blackboard_set_get_bool(self, blackboard_client):
        """Test setting and getting boolean values."""
        # Set value
        success = blackboard_client.set("test_bool", True)
        assert success, "Failed to set boolean value"
        time.sleep(0.1)  # Allow server to process fire-and-forget set
        # Get value
        value = blackboard_client.get_bool("test_bool", False)
        assert value is True, "Failed to get boolean value"
        # Test false
        blackboard_client.set("test_bool", False)
        time.sleep(0.1)
        value = blackboard_client.get_bool("test_bool", True)
        assert value is False

    @pytest.mark.integration
    def test_blackboard_set_get_int(self, blackboard_client):
        """Test setting and getting integer values."""
        # Set value
        success = blackboard_client.set("test_int", 42)
        assert success, "Failed to set integer value"
        time.sleep(0.1)  # Allow server to process fire-and-forget set
        # Get value
        value = blackboard_client.get_int("test_int", 0)
        assert value == 42, "Failed to get integer value"

    @pytest.mark.integration
    def test_blackboard_set_get_double(self, blackboard_client):
        """Test setting and getting double values."""
        # Set value
        success = blackboard_client.set("test_double", 3.14159)
        assert success, "Failed to set double value"
        time.sleep(0.1)  # Allow server to process fire-and-forget set
        # Get value
        value = blackboard_client.get_double("test_double", 0.0)
        assert abs(value - 3.14159) < 0.0001, "Failed to get double value"

    @pytest.mark.integration
    def test_blackboard_set_get_string(self, blackboard_client):
        """Test setting and getting string values."""
        # Set value
        test_string = "test_value_123"
        success = blackboard_client.set("test_string", test_string)
        assert success, "Failed to set string value"
        time.sleep(0.1)  # Allow server to process fire-and-forget set
        # Get value
        value = blackboard_client.get_string("test_string", "")
        assert value == test_string, "Failed to get string value"

    @pytest.mark.integration
    def test_blackboard_navigation_keys(self, blackboard_client):
        """Test navigation-related blackboard keys."""
        # Test perception_confidence
        success = blackboard_client.set("perception_confidence", 0.85)
        assert success, "Failed to set perception_confidence"
        time.sleep(0.1)  # Allow server to process fire-and-forget set
        confidence = blackboard_client.get_double("perception_confidence", 0.0)
        assert abs(confidence - 0.85) < 0.01, "Failed to get perception_confidence"
        # Test map_quality
        success = blackboard_client.set("map_quality", 0.9)
        assert success, "Failed to set map_quality"
        time.sleep(0.1)
        quality = blackboard_client.get_double("map_quality", 0.0)
        assert abs(quality - 0.9) < 0.01, "Failed to get map_quality"
        # Test closest_obstacle_distance
        success = blackboard_client.set("closest_obstacle_distance", 2.5)
        assert success, "Failed to set closest_obstacle_distance"
        time.sleep(0.1)
        distance = blackboard_client.get_double("closest_obstacle_distance", 999.0)
        assert abs(distance - 2.5) < 0.01, "Failed to get closest_obstacle_distance"

    @pytest.mark.integration
    def test_blackboard_cache(self, blackboard_client):
        """Test that caching works correctly."""
        # Set value
        blackboard_client.set("cache_test", 42)
        time.sleep(0.1)  # Allow server to process fire-and-forget set
        # First get should hit service
        value1 = blackboard_client.get_int("cache_test", 0)
        assert value1 == 42
        # Second get within TTL should use cache
        value2 = blackboard_client.get_int("cache_test", 0)
        assert value2 == 42
        # Clear cache and get again
        blackboard_client.clear_cache()
        value3 = blackboard_client.get_int("cache_test", 0)
        assert value3 == 42

    @pytest.mark.integration
    def test_blackboard_default_values(self, blackboard_client):
        """Test that default values are returned for missing keys."""
        # Get non-existent key
        value = blackboard_client.get_int("non_existent_key", 999)
        assert value == 999, "Default value not returned"

        # Get with None default
        value = blackboard_client.get_string("non_existent_key2", "default")
        assert value == "default", "String default value not returned"

    @pytest.mark.integration
    def test_blackboard_mission_state(self, blackboard_client):
        """Test mission state blackboard keys."""
        # Test samples_collected
        success = blackboard_client.set("samples_collected", 5)
        assert success
        time.sleep(0.1)  # Allow server to process fire-and-forget set
        samples = blackboard_client.get_int("samples_collected", 0)
        assert samples == 5
        # Test waypoints_completed
        success = blackboard_client.set("waypoints_completed", 3)
        assert success
        time.sleep(0.1)
        waypoints = blackboard_client.get_int("waypoints_completed", 0)
        assert waypoints == 3
        # Test mission_active
        success = blackboard_client.set("mission_active", True)
        assert success
        time.sleep(0.1)
        active = blackboard_client.get_bool("mission_active", False)
        assert active is True


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
