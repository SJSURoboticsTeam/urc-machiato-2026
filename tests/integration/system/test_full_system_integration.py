#!/usr/bin/env python3
# type: ignore
"""
Full System Integration Test - Real ROS2 Nodes

Tests complete system with:
- BT Orchestrator
- State Machine
- WebSocket Bridge
- Full connectivity and data flow verification
"""

import pytest
import subprocess
import time
import os
import sys
import json
from typing import Optional, Dict, Any, List

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))

try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped, Point, Quaternion

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    rclpy = None


class FullSystemIntegrationTest:
    """Full system integration tests with real ROS2 nodes."""

    def __init__(self):
        self.processes: List[subprocess.Popen] = []
        self.test_node: Optional[Node] = None
        self.executor = None
        self.test_results: Dict[str, Any] = {
            "connectivity": {},
            "data_availability": {},
            "integration": {},
        }

    def setup_method(self):
        """Set up test environment."""
        if not ROS2_AVAILABLE:
            pytest.skip("ROS2 not available")

        if not rclpy.ok():
            rclpy.shutdown()
        rclpy.init()

        self.test_node = Node("full_system_test")
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.test_node)

    def teardown_method(self):
        """Clean up."""
        for proc in self.processes:
            try:
                proc.terminate()
                proc.wait(timeout=5)
            except:
                proc.kill()
        self.processes.clear()

        if self.test_node:
            self.test_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    def start_node(self, command: List[str], name: str, wait_time: float = 3.0) -> bool:
        """Start a ROS2 node."""
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = "42"

        try:
            proc = subprocess.Popen(
                command,
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True,
            )
            self.processes.append(proc)
            time.sleep(wait_time)

            # Verify node is running
            result = subprocess.run(
                ["ros2", "node", "list"], capture_output=True, text=True, timeout=5
            )

            if name in result.stdout:
                self.test_node.get_logger().info(f"{name} started successfully")
                return True
            else:
                self.test_node.get_logger().warn(f"{name} not found in node list")
                return False
        except Exception as e:
            self.test_node.get_logger().error(f"Failed to start {name}: {e}")
            return False

    def check_topic_exists(self, topic: str) -> bool:
        """Check if a topic exists."""
        result = subprocess.run(
            ["ros2", "topic", "list"], capture_output=True, text=True, timeout=5
        )
        return topic in result.stdout

    def check_service_exists(self, service: str) -> bool:
        """Check if a service exists."""
        result = subprocess.run(
            ["ros2", "service", "list"], capture_output=True, text=True, timeout=5
        )
        return service in result.stdout

    def check_node_exists(self, node_name: str) -> bool:
        """Check if a node exists."""
        result = subprocess.run(
            ["ros2", "node", "list"], capture_output=True, text=True, timeout=5
        )
        return node_name in result.stdout

    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_full_system_startup(self):
        """Test full system startup with all nodes."""
        # Start BT orchestrator
        bt_started = self.start_node(
            ["ros2", "run", "autonomy_bt", "bt_orchestrator"],
            "bt_orchestrator",
            wait_time=5,
        )
        assert bt_started, "BT orchestrator failed to start"

        # Start state machine (try Python script first)
        state_machine_script = os.path.join(
            os.path.dirname(__file__), "../../src/core/adaptive_state_machine.py"
        )
        state_started = False
        if os.path.exists(state_machine_script):
            state_started = self.start_node(
                ["python3", state_machine_script], "adaptive_state_machine", wait_time=3
            )

        # Verify nodes are running
        assert self.check_node_exists("/bt_orchestrator"), "BT orchestrator not found"
        if state_started:
            assert (
                self.check_node_exists("/adaptive_state_machine")
                or "state_machine"
                in subprocess.run(
                    ["ros2", "node", "list"], capture_output=True, text=True
                ).stdout.lower()
            ), "State machine not found"

        self.test_results["integration"]["startup"] = True

    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_connectivity_bt_state_machine(self):
        """Test connectivity between BT and State Machine."""
        assert self.start_node(
            ["ros2", "run", "autonomy_bt", "bt_orchestrator"], "bt_orchestrator"
        ), "BT orchestrator failed to start"

        time.sleep(2)

        # Check if state machine service exists
        service_exists = self.check_service_exists("/adaptive_state_machine/get_state")

        self.test_results["connectivity"]["bt_to_state_machine"] = service_exists
        assert service_exists, "State machine service not available"

    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_connectivity_state_machine_websocket(self):
        """Test connectivity between State Machine and WebSocket Bridge."""
        # Check if state machine publishes state topic
        state_machine_script = os.path.join(
            os.path.dirname(__file__), "../../src/core/adaptive_state_machine.py"
        )
        if os.path.exists(state_machine_script):
            self.start_node(["python3", state_machine_script], "adaptive_state_machine")
            time.sleep(2)

        # Check if state topic exists
        topic_exists = self.check_topic_exists("/adaptive_state_machine/state")

        self.test_results["connectivity"]["state_machine_to_websocket"] = topic_exists
        assert topic_exists, "State machine state topic not found"

    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_data_availability_blackboard(self):
        """Test data availability for blackboard updates."""
        assert self.start_node(
            ["ros2", "run", "autonomy_bt", "bt_orchestrator"], "bt_orchestrator"
        ), "BT orchestrator failed to start"

        time.sleep(2)

        # Check if required topics exist for blackboard updates
        odom_exists = self.check_topic_exists("/odom")
        slam_exists = self.check_topic_exists("/slam/pose")

        self.test_results["data_availability"]["odom"] = odom_exists
        self.test_results["data_availability"]["slam"] = slam_exists

        # At least one should exist
        assert odom_exists or slam_exists, "No blackboard data sources available"

    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_data_availability_state_machine(self):
        """Test data availability from state machine."""
        state_machine_script = os.path.join(
            os.path.dirname(__file__), "../../src/core/adaptive_state_machine.py"
        )
        if os.path.exists(state_machine_script):
            self.start_node(["python3", state_machine_script], "adaptive_state_machine")
            time.sleep(2)

        # Check if state topic is publishing
        result = subprocess.run(
            ["ros2", "topic", "info", "/adaptive_state_machine/state"],
            capture_output=True,
            text=True,
            timeout=5,
        )

        topic_available = result.returncode == 0
        self.test_results["data_availability"]["state_topic"] = topic_available

        # Try to echo once to see if data is flowing
        if topic_available:
            echo_result = subprocess.run(
                [
                    "timeout",
                    "2",
                    "ros2",
                    "topic",
                    "echo",
                    "/adaptive_state_machine/state",
                    "--once",
                ],
                capture_output=True,
                text=True,
                timeout=3,
            )
            has_data = echo_result.returncode == 0 and len(echo_result.stdout) > 0
            self.test_results["data_availability"]["state_data"] = has_data

    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_consistency_check(self):
        """Comprehensive consistency check for connectivity and data."""
        # Start all nodes
        bt_started = self.start_node(
            ["ros2", "run", "autonomy_bt", "bt_orchestrator"],
            "bt_orchestrator",
            wait_time=5,
        )

        state_machine_script = os.path.join(
            os.path.dirname(__file__), "../../src/core/adaptive_state_machine.py"
        )
        state_started = False
        if os.path.exists(state_machine_script):
            state_started = self.start_node(
                ["python3", state_machine_script], "adaptive_state_machine", wait_time=3
            )

        time.sleep(3)

        consistency_report = {
            "nodes": {},
            "topics": {},
            "services": {},
            "data_flow": {},
        }

        # Check nodes
        consistency_report["nodes"]["bt_orchestrator"] = self.check_node_exists(
            "/bt_orchestrator"
        )
        if state_started:
            consistency_report["nodes"]["state_machine"] = (
                self.check_node_exists("/adaptive_state_machine")
                or "state_machine"
                in subprocess.run(
                    ["ros2", "node", "list"], capture_output=True, text=True
                ).stdout.lower()
            )

        # Check critical topics
        consistency_report["topics"]["/bt/telemetry"] = self.check_topic_exists(
            "/bt/telemetry"
        )
        consistency_report["topics"]["/adaptive_state_machine/state"] = (
            self.check_topic_exists("/adaptive_state_machine/state")
        )
        consistency_report["topics"]["/odom"] = self.check_topic_exists("/odom")
        consistency_report["topics"]["/slam/pose"] = self.check_topic_exists(
            "/slam/pose"
        )

        # Check critical services
        consistency_report["services"]["/bt_orchestrator/get_state"] = (
            self.check_service_exists("/bt_orchestrator/get_state")
        )
        consistency_report["services"]["/adaptive_state_machine/get_state"] = (
            self.check_service_exists("/adaptive_state_machine/get_state")
        )

        # Check data flow
        if consistency_report["topics"]["/adaptive_state_machine/state"]:
            result = subprocess.run(
                [
                    "timeout",
                    "2",
                    "ros2",
                    "topic",
                    "echo",
                    "/adaptive_state_machine/state",
                    "--once",
                ],
                capture_output=True,
                text=True,
                timeout=3,
            )
            consistency_report["data_flow"]["state_machine_publishing"] = (
                result.returncode == 0 and len(result.stdout) > 0
            )

        self.test_results["consistency"] = consistency_report

        # Print report
        self.test_node.get_logger().info(
            f"Consistency Report: {json.dumps(consistency_report, indent=2)}"
        )

        # Assertions
        assert consistency_report["nodes"][
            "bt_orchestrator"
        ], "BT orchestrator not running"
        assert consistency_report["topics"][
            "/bt/telemetry"
        ], "BT telemetry topic not found"
        assert consistency_report["services"][
            "/bt_orchestrator/get_state"
        ], "BT lifecycle service not found"

        if state_started:
            assert consistency_report["topics"][
                "/adaptive_state_machine/state"
            ], "State machine state topic not found"
            assert consistency_report["services"][
                "/adaptive_state_machine/get_state"
            ], "State machine service not found"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short", "-s"])
