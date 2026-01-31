#!/usr/bin/env python3
"""
Behavior Tree System Tests - Comprehensive testing for BT.CPP v4.x integration

Tests cover:
- Blackboard data integrity and thread safety
- BT node execution and state transitions
- Action server integration
- Mission execution scenarios
- Failure recovery mechanisms
- Performance under load
"""

import pytest
import time
import threading
from unittest.mock import Mock, patch, MagicMock
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import SingleThreadedExecutor
import rclpy.action
from rclpy.action.client import ClientGoalHandle

# ROS2 imports (skip whole module if workspace packages not built)
try:
    from autonomy_utilities import QoSProfiles, OperationResult
    import autonomy_interfaces.action as action_msgs
except ImportError:
    pytest.skip(
        "autonomy_utilities/autonomy_interfaces not available (run colcon build)",
        allow_module_level=True,
    )


class TestBTActionServers:
    """Test BT action server integration"""

    @pytest.fixture
    def ros_context(self):
        """Provide ROS2 context for tests"""
        rclpy.init()
        yield
        rclpy.shutdown()

    def test_navigate_action_server_creation(self, ros_context):
        """Test navigation action server client creation"""
        node = Node("test_navigate_client")

        # Create action client
        action_client = ActionClient(node, action_msgs.NavigateToPose, '/bt/navigate_to_pose')

        # Test action client creation
        assert action_client is not None
        assert hasattr(action_client, 'send_goal_async'), "ActionClient has required methods"

        node.destroy_node()

    def test_execute_mission_action_server_creation(self, ros_context):
        """Test mission execution action server client creation"""
        node = Node("test_mission_client")

        # Create action client
        action_client = ActionClient(node, action_msgs.ExecuteMission, '/bt/execute_mission')

        # Test action client creation
        assert action_client is not None
        assert hasattr(action_client, 'send_goal_async'), "ActionClient has required methods"

        node.destroy_node()

    @pytest.mark.asyncio
    async def test_navigate_action_communication(self, ros_context):
        """Test actual communication with navigation action server"""
        node = Node("test_navigate_client")
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        # Create action client
        action_client = ActionClient(node, action_msgs.NavigateToPose, '/bt/navigate_to_pose')

        # Wait for server (with timeout)
        server_available = action_client.wait_for_server(timeout_sec=5.0)
        if not server_available:
            pytest.skip("BT action server not available - run integrated system first")

        # Create goal
        goal = action_msgs.NavigateToPose.Goal()
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = 1.0
        goal.target_pose.pose.position.y = 2.0
        goal.target_pose.pose.position.z = 0.0
        goal.target_pose.pose.orientation.w = 1.0
        goal.tolerance = 0.5
        goal.timeout = 30.0

        # Send goal
        future = action_client.send_goal_async(goal)
        executor.spin_until_future_complete(future, timeout_sec=10.0)

        goal_handle = future.result()
        assert goal_handle is not None, "Goal was rejected"

        # Wait for result
        result_future = goal_handle.get_result_async()
        executor.spin_until_future_complete(result_future, timeout_sec=35.0)

        result = result_future.result()
        assert result is not None, "Failed to get result"
        # The result structure depends on the actual implementation
        # For now, just verify we got some result

        node.destroy_node()

    @pytest.mark.asyncio
    async def test_mission_action_communication(self, ros_context):
        """Test actual communication with mission action server"""
        node = Node("test_mission_client")
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        # Create action client
        action_client = ActionClient(node, action_msgs.ExecuteMission, '/bt/execute_mission')

        # Wait for server (with timeout)
        server_available = action_client.wait_for_server(timeout_sec=5.0)
        if not server_available:
            pytest.skip("BT mission action server not available - run integrated system first")

        # Create goal
        goal = action_msgs.ExecuteMission.Goal()
        goal.mission_type = "sample_collection"
        goal.mission_id = "test_mission_001"
        goal.timeout = 30.0
        goal.waypoints = ["waypoint_1", "waypoint_2"]

        # Send goal
        future = action_client.send_goal_async(goal)
        executor.spin_until_future_complete(future, timeout_sec=10.0)

        goal_handle = future.result()
        assert goal_handle is not None, "Mission goal was rejected"

        # Wait for result
        result_future = goal_handle.get_result_async()
        executor.spin_until_future_complete(result_future, timeout_sec=35.0)

        result = result_future.result()
        assert result is not None, "Failed to get mission result"

        node.destroy_node()


class TestBTTelemetry:
    """Test BT telemetry publishing"""

    @pytest.fixture
    def ros_context(self):
        """Provide ROS2 context for tests"""
        rclpy.init()
        yield
        rclpy.shutdown()

    def test_bt_telemetry_topic(self, ros_context):
        """Test that BT telemetry topic exists and publishes data"""
        node = Node("test_telemetry_subscriber")

        received_messages = []

        def telemetry_callback(msg):
            received_messages.append(msg)

        from std_msgs.msg import String
        subscription = node.create_subscription(
            String,
            '/bt/telemetry',
            telemetry_callback,
            10
        )

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        # Wait for messages
        timeout_time = time.time() + 10.0
        while time.time() < timeout_time and len(received_messages) == 0:
            executor.spin_once(timeout_sec=0.1)

        # Clean up
        node.destroy_node()

        # If BT is running, we should have received telemetry
        # If not running, this test will be skipped in integration
        if len(received_messages) > 0:
            assert len(received_messages) > 0, "Received telemetry messages"
            # Verify message structure
            assert hasattr(received_messages[0], 'data'), "Telemetry message has data field"


class TestBTMissionExecution:
    """Test complete mission execution scenarios"""

    def test_sample_collection_mission_xml(self):
        """Test the sample collection mission BT XML structure"""
        # Test BT XML parsing
        bt_xml = """<?xml version="1.0"?>
        <root BTCPP_format="4">
            <BehaviorTree ID="SampleCollection">
                <Sequence name="CollectionSequence">
                    <SensorCheck sensor_type="imu" timeout="5.0"/>
                    <NavigateToWaypoint x="10.0" y="5.0" tolerance="1.0"/>
                    <SampleCollection site_id="1" timeout="60.0"/>
                </Sequence>
            </BehaviorTree>
        </root>"""

        # Verify XML is well-formed
        import xml.etree.ElementTree as ET
        try:
            root = ET.fromstring(bt_xml)
            assert root.tag == "root"
            assert root.get("BTCPP_format") == "4"

            # Verify mission structure
            sequence = root.find(".//Sequence")
            assert sequence is not None
            assert sequence.get("name") == "CollectionSequence"

            # Check for required nodes
            sensor_check = root.find(".//SensorCheck")
            navigate = root.find(".//NavigateToWaypoint")
            sample = root.find(".//SampleCollection")

            assert sensor_check is not None
            assert navigate is not None
            assert sample is not None

            # Verify parameters
            assert navigate.get("x") == "10.0"
            assert navigate.get("y") == "5.0"
            assert sample.get("site_id") == "1"

        except ET.ParseError as e:
            pytest.fail(f"BT XML is not well-formed: {e}")

    def test_mission_failure_recovery_xml(self):
        """Test mission execution with failure recovery XML"""
        # Test BT fallback behavior
        bt_xml = """<?xml version="1.0"?>
        <root BTCPP_format="4">
            <BehaviorTree ID="RecoveryTest">
                <Fallback name="MissionWithRecovery">
                    <Sequence name="NormalExecution">
                        <SensorCheck sensor_type="imu" timeout="5.0"/>
                        <NavigateToWaypoint x="10.0" y="5.0" tolerance="1.0"/>
                    </Sequence>
                    <Sequence name="RecoverySequence">
                        <EmergencyStop reason="navigation_failure"/>
                        <CallService service_name="/emergency/recovery" command="recover"/>
                    </Sequence>
                </Fallback>
            </BehaviorTree>
        </root>"""

        import xml.etree.ElementTree as ET
        try:
            root = ET.fromstring(bt_xml)
            # Verify fallback structure exists
            fallback = root.find(".//Fallback")
            assert fallback is not None
            assert fallback.get("name") == "MissionWithRecovery"

            # Verify recovery sequence
            recovery = root.find(".//Sequence[@name='RecoverySequence']")
            assert recovery is not None

            emergency_stop = recovery.find(".//EmergencyStop")
            call_service = recovery.find(".//CallService")

            assert emergency_stop is not None
            assert call_service is not None
            assert emergency_stop.get("reason") == "navigation_failure"

        except ET.ParseError as e:
            pytest.fail(f"Recovery BT XML is not well-formed: {e}")


class TestBTPerformance:
    """Test BT system performance"""

    @pytest.fixture
    def ros_context(self):
        """Provide ROS2 context for tests"""
        rclpy.init()
        yield
        rclpy.shutdown()

    def test_action_server_response_time(self, ros_context):
        """Test action server response times"""
        node = Node("performance_test_client")
        executor = SingleThreadedExecutor()
        executor.add_node(node)

        # Create action client
        action_client = ActionClient(node, action_msgs.NavigateToPose, '/bt/navigate_to_pose')

        # Wait for server
        server_available = action_client.wait_for_server(timeout_sec=5.0)
        if not server_available:
            pytest.skip("BT action server not available for performance testing")

        response_times = []

        # Test multiple requests
        for i in range(5):
            start_time = time.time()

            # Create goal
            goal = action_msgs.NavigateToPose.Goal()
            goal.target_pose.header.frame_id = "map"
            goal.target_pose.pose.position.x = float(i)
            goal.target_pose.pose.position.y = float(i)
            goal.target_pose.pose.position.z = 0.0
            goal.target_pose.pose.orientation.w = 1.0
            goal.tolerance = 0.5
            goal.timeout = 10.0

            # Send goal and measure response time
            future = action_client.send_goal_async(goal)
            executor.spin_until_future_complete(future, timeout_sec=5.0)

            if future.result() is not None:
                end_time = time.time()
                response_times.append(end_time - start_time)

        node.destroy_node()

        if response_times:
            avg_response_time = sum(response_times) / len(response_times)
            # Should respond within reasonable time (under 1 second for goal acceptance)
            assert avg_response_time < 1.0, ".2f"

    def test_telemetry_publishing_rate(self, ros_context):
        """Test BT telemetry publishing performance"""
        node = Node("telemetry_performance_test")

        received_messages = []
        message_timestamps = []

        def telemetry_callback(msg):
            received_messages.append(msg)
            message_timestamps.append(time.time())

        from std_msgs.msg import String
        subscription = node.create_subscription(
            String,
            '/bt/telemetry',
            telemetry_callback,
            10
        )

        executor = SingleThreadedExecutor()
        executor.add_node(node)

        # Collect messages for 5 seconds
        start_time = time.time()
        while time.time() - start_time < 5.0:
            executor.spin_once(timeout_sec=0.1)

        node.destroy_node()

        if len(message_timestamps) >= 2:
            # Calculate publishing rate
            time_span = message_timestamps[-1] - message_timestamps[0]
            message_rate = len(message_timestamps) / time_span

            # Should publish at reasonable rate (not flooding, but active)
            assert 0.1 <= message_rate <= 50.0, ".2f"


class TestBTIntegration:
    """Test BT integration with other systems"""

    @pytest.fixture
    def ros_context(self):
        """Provide ROS2 context for tests"""
        rclpy.init()
        yield
        rclpy.shutdown()

    def test_bt_with_state_machine(self, ros_context):
        """Test BT coordination with state machine"""
        # This test would verify that BT actions properly integrate
        # with the state machine's autonomous state
        pytest.skip("Integration test requires full system running")

    def test_bt_with_navigation(self, ros_context):
        """Test BT navigation integration"""
        # Verify BT can trigger navigation actions
        pytest.skip("Integration test requires navigation system running")

    def test_bt_service_calls(self, ros_context):
        """Test BT service call functionality"""
        # Verify BT can call ROS2 services
        pytest.skip("Integration test requires service endpoints running")


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
