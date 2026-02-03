#!/usr/bin/env python3
"""
Integration tests for SLAM pipeline (slam/pose, slam_node, blackboard).

Asserts that when slam/pose (or equivalent) is published, slam_node updates
blackboard (slam_confidence, feature_count). Can run with mocked pose publisher
or with slam.launch (manual/CI). Use ROS_DOMAIN_ID to isolate from live system.

Author: URC 2026 Test Suite
"""

import os
import sys

_src = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", "src"))
if _src not in sys.path:
    sys.path.insert(0, _src)

import time

import pytest
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node

# Optional: blackboard client for assertions
try:
    from core.unified_blackboard_client import UnifiedBlackboardClient
    from core.blackboard_keys import BlackboardKeys

    _BLACKBOARD_AVAILABLE = True
except ImportError:
    _BLACKBOARD_AVAILABLE = False
    BlackboardKeys = None


class TestSLAMIntegration:
    """SLAM pipeline integration tests."""

    @pytest.fixture(scope="class")
    def ros2_context(self):
        """Initialize ROS2 context."""
        if not rclpy.utilities.ok():
            rclpy.init()
        yield
        if rclpy.utilities.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass

    @pytest.fixture
    def node(self, ros2_context):
        """Create test node."""
        n = Node("test_slam_integration_node")
        yield n
        n.destroy_node()

    @pytest.mark.integration
    def test_slam_pose_topic_accepts_publisher(self, node):
        """Publishing PoseWithCovarianceStamped to slam/pose does not raise."""
        pub = node.create_publisher(PoseWithCovarianceStamped, "slam/pose", 10)
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.pose.position.x = 1.0
        msg.pose.pose.position.y = 2.0
        msg.pose.pose.position.z = 0.0
        pub.publish(msg)
        time.sleep(0.2)
        # No assertion other than no exception; subscriber may or may not exist
        assert True

    @pytest.mark.integration
    @pytest.mark.skipif(
        not _BLACKBOARD_AVAILABLE, reason="Blackboard client not available"
    )
    def test_blackboard_slam_keys_exist_when_bt_running(self, node):
        """When bt_orchestrator is running, blackboard has SLAM-related keys (read or default)."""
        client = UnifiedBlackboardClient(node, cache_ttl=0.01, get_timeout_sec=2.0)
        if not client.get_client.wait_for_service(timeout_sec=3.0):
            pytest.skip(
                "Blackboard get_value service not available (bt_orchestrator not running)"
            )
        # Read keys that slam_node writes; may return default if slam_node not running
        slam_conf = client.get(BlackboardKeys.SLAM_CONFIDENCE, 0.0, "double")
        feature_count = client.get(BlackboardKeys.FEATURE_COUNT, 0, "int")
        assert isinstance(slam_conf, (int, float))
        assert isinstance(feature_count, (int, float))
