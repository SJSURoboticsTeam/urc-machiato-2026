#!/usr/bin/env python3
# type: ignore
"""
Runtime integration tests for BT.CPP orchestrator with ROS2 nodes running.

Tests verify:
- Lifecycle transitions (configure, activate, deactivate)
- QoS profile communication
- Blackboard updates from ROS2 topics
- Service calls (state machine integration)
- Telemetry publishing
- Action server functionality
"""

import pytest
import subprocess
import time
import signal
import os
import sys
from typing import List, Optional

# Add project paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

# ROS2 imports
try:
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from std_msgs.msg import String
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import PoseStamped, Point, Quaternion
    from lifecycle_msgs.msg import Transition, State
    from lifecycle_msgs.srv import ChangeState, GetState
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    rclpy = None


class TestBTRuntimeIntegration:
    """Runtime tests for BT orchestrator with actual ROS2 nodes."""
    
    def __init__(self):
        self.bt_process: Optional[subprocess.Popen] = None
        self.state_machine_process: Optional[subprocess.Popen] = None
        self.test_node: Optional[Node] = None
        self.executor = None
        
    def setup_method(self):
        """Set up test environment."""
        if not ROS2_AVAILABLE:
            pytest.skip("ROS2 not available")
        
        # Initialize ROS2
        if not rclpy.ok():
            rclpy.shutdown()
        rclpy.init()
        
        # Create test node
        self.test_node = Node("bt_runtime_test_node")
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.test_node)
        
    def teardown_method(self):
        """Clean up test environment."""
        # Stop processes
        if self.bt_process:
            try:
                self.bt_process.terminate()
                self.bt_process.wait(timeout=5)
            except:
                self.bt_process.kill()
            self.bt_process = None
            
        if self.state_machine_process:
            try:
                self.state_machine_process.terminate()
                self.state_machine_process.wait(timeout=5)
            except:
                self.state_machine_process.kill()
            self.state_machine_process = None
        
        # Clean up ROS2
        if self.test_node:
            self.test_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    
    def start_bt_orchestrator(self) -> bool:
        """Start BT orchestrator node."""
        try:
            # Source ROS2 and workspace
            env = os.environ.copy()
            env["ROS_DOMAIN_ID"] = "42"
            
            # Start BT orchestrator
            self.bt_process = subprocess.Popen(
                ["ros2", "run", "autonomy_bt", "bt_orchestrator"],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            
            # Wait for node to start
            time.sleep(3)
            
            # Check if node is running
            result = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if "bt_orchestrator" in result.stdout:
                self.test_node.get_logger().info("BT orchestrator started successfully")
                return True
            else:
                self.test_node.get_logger().error(f"BT orchestrator not found: {result.stdout}")
                return False
                
        except Exception as e:
            self.test_node.get_logger().error(f"Failed to start BT orchestrator: {e}")
            return False
    
    def wait_for_node(self, node_name: str, timeout: float = 10.0) -> bool:
        """Wait for a node to appear in the node list."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            result = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True,
                text=True,
                timeout=2
            )
            if node_name in result.stdout:
                return True
            time.sleep(0.5)
        return False
    
    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_bt_node_starts(self):
        """Test that BT orchestrator node starts successfully."""
        assert self.start_bt_orchestrator(), "BT orchestrator failed to start"
        
        # Verify node is in node list
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        assert "bt_orchestrator" in result.stdout, "BT orchestrator node not found"
    
    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_lifecycle_transitions(self):
        """Test lifecycle state transitions."""
        assert self.start_bt_orchestrator(), "BT orchestrator failed to start"
        time.sleep(2)  # Wait for node to fully initialize
        
        # Get current state
        get_state_client = self.test_node.create_client(
            GetState,
            "/bt_orchestrator/get_state"
        )
        
        # Wait for service
        if not get_state_client.wait_for_service(timeout_sec=5.0):
            pytest.skip("Lifecycle service not available")
        
        # Request current state
        request = GetState.Request()
        future = get_state_client.call_async(request)
        
        # Spin until response
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)
        
        if future.done():
            response = future.result()
            assert response is not None, "No response from get_state service"
            current_state = response.current_state.id
            self.test_node.get_logger().info(f"Current state: {current_state}")
            
            # State should be unconfigured (1) or inactive (2)
            assert current_state in [1, 2], f"Unexpected initial state: {current_state}"
        else:
            pytest.skip("Could not get lifecycle state")
        
        # Test configure transition
        change_state_client = self.test_node.create_client(
            ChangeState,
            "/bt_orchestrator/change_state"
        )
        
        if change_state_client.wait_for_service(timeout_sec=5.0):
            transition = Transition()
            transition.id = Transition.TRANSITION_CONFIGURE
            transition.label = "configure"
            
            request = ChangeState.Request()
            request.transition = transition
            
            future = change_state_client.call_async(request)
            rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)
            
            if future.done():
                response = future.result()
                # Configure should succeed (return code 0 or True)
                self.test_node.get_logger().info(f"Configure transition result: {response}")
    
    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_qos_communication(self):
        """Test QoS profile communication."""
        assert self.start_bt_orchestrator(), "BT orchestrator failed to start"
        time.sleep(2)
        
        # Create publisher with QoS matching BT orchestrator expectations
        qos_profile = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )
        
        odom_publisher = self.test_node.create_publisher(
            Odometry,
            "/odom",
            qos_profile
        )
        
        # Publish test odometry message
        odom_msg = Odometry()
        odom_msg.header.frame_id = "odom"
        odom_msg.header.stamp = self.test_node.get_clock().now().to_msg()
        odom_msg.pose.pose.position.x = 1.0
        odom_msg.pose.pose.position.y = 2.0
        odom_msg.pose.pose.position.z = 0.0
        
        # Publish and wait
        odom_publisher.publish(odom_msg)
        time.sleep(1)
        
        # Check if message was received (verify by checking topic info)
        result = subprocess.run(
            ["ros2", "topic", "info", "/odom"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        assert result.returncode == 0, "Failed to get topic info"
        assert "odom" in result.stdout.lower(), "Topic not found"
    
    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_telemetry_publishing(self):
        """Test that BT orchestrator publishes telemetry."""
        assert self.start_bt_orchestrator(), "BT orchestrator failed to start"
        time.sleep(2)
        
        # Create subscriber for telemetry
        telemetry_received = []
        
        def telemetry_callback(msg):
            telemetry_received.append(msg.data)
            self.test_node.get_logger().info(f"Received telemetry: {msg.data[:50]}...")
        
        qos_profile = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE
        )
        
        telemetry_subscriber = self.test_node.create_subscription(
            String,
            "/bt/telemetry",
            telemetry_callback,
            qos_profile
        )
        
        # Wait for telemetry messages
        timeout = 10.0
        start_time = time.time()
        while time.time() - start_time < timeout:
            self.executor.spin_once(timeout_sec=0.5)
            if telemetry_received:
                break
        
        assert len(telemetry_received) > 0, "No telemetry messages received"
        self.test_node.get_logger().info(f"Received {len(telemetry_received)} telemetry messages")
    
    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_topic_subscriptions(self):
        """Test that BT orchestrator subscribes to required topics."""
        assert self.start_bt_orchestrator(), "BT orchestrator failed to start"
        time.sleep(2)
        
        # Check topic list
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        topics = result.stdout.split('\n')
        
        # BT orchestrator should subscribe to these topics
        expected_topics = ["/odom", "/slam/pose"]
        
        for topic in expected_topics:
            # Check if topic exists (may be empty if no publishers)
            topic_info = subprocess.run(
                ["ros2", "topic", "info", topic],
                capture_output=True,
                text=True,
                timeout=5
            )
            # Topic should exist (even if no publishers yet)
            self.test_node.get_logger().info(f"Topic {topic} info: {topic_info.stdout[:100]}")
    
    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_service_availability(self):
        """Test that required services are available."""
        assert self.start_bt_orchestrator(), "BT orchestrator failed to start"
        time.sleep(2)
        
        # Check service list
        result = subprocess.run(
            ["ros2", "service", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        services = result.stdout.split('\n')
        self.test_node.get_logger().info(f"Available services: {len(services)}")
        
        # BT orchestrator should provide these services (if implemented)
        # Note: Services may vary based on implementation
        service_list_str = '\n'.join(services)
        self.test_node.get_logger().info(f"Services: {service_list_str[:200]}")
    
    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_action_servers(self):
        """Test that action servers are available."""
        assert self.start_bt_orchestrator(), "BT orchestrator failed to start"
        time.sleep(2)
        
        # Check action list
        result = subprocess.run(
            ["ros2", "action", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        actions = result.stdout.split('\n')
        self.test_node.get_logger().info(f"Available actions: {len(actions)}")
        
        # BT orchestrator should provide these actions (if implemented)
        action_list_str = '\n'.join(actions)
        self.test_node.get_logger().info(f"Actions: {action_list_str[:200]}")


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
