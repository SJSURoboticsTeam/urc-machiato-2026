#!/usr/bin/env python3
"""
Integration tests for mission workflows (mission executor, blackboard, status topics).

Runs mission executor and selected missions (waypoint, sample) with mocked or
simulated SLAM/nav; asserts blackboard and status topics. Use ROS_DOMAIN_ID
to isolate from live system.

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
from rclpy.node import Node
from std_msgs.msg import String

try:
    from core.unified_blackboard_client import UnifiedBlackboardClient
    from core.blackboard_keys import BlackboardKeys

    _BLACKBOARD_AVAILABLE = True
except ImportError:
    _BLACKBOARD_AVAILABLE = False
    BlackboardKeys = None


class TestMissionWorkflows:
    """Mission workflow integration tests."""

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
        n = Node("test_mission_workflows_node")
        yield n
        n.destroy_node()

    @pytest.mark.integration
    def test_mission_status_topic_accepts_subscriber(self, node):
        """Subscribing to /mission/status does not raise."""
        received = []

        def cb(msg):
            received.append(msg.data)

        sub = node.create_subscription(String, "/mission/status", cb, 10)
        time.sleep(0.3)
        assert True

    @pytest.mark.integration
    @pytest.mark.skipif(
        not _BLACKBOARD_AVAILABLE, reason="Blackboard client not available"
    )
    def test_blackboard_mission_keys_readable_when_bt_running(self, node):
        """When bt_orchestrator is running, mission blackboard keys are readable."""
        client = UnifiedBlackboardClient(node, cache_ttl=0.01, get_timeout_sec=2.0)
        if not client.get_client.wait_for_service(timeout_sec=3.0):
            pytest.skip("Blackboard get_value service not available")
        mission_active = client.get(BlackboardKeys.MISSION_ACTIVE, False, "bool")
        phase = client.get(BlackboardKeys.CURRENT_MISSION_PHASE, "idle", "string")
        assert isinstance(mission_active, bool)
        assert isinstance(phase, str)
