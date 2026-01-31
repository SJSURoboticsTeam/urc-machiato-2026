#!/usr/bin/env python3
# type: ignore
"""
Full System Integration Test with Simulator

Tests complete system with simulator providing sensor data:
- BT Orchestrator
- State Machine
- Sensor Simulator (provides /odom, /slam/pose, IMU, GPS)
- Full connectivity and data flow verification

This addresses the gap where topics exist but have no publishers.
"""

import pytest
import subprocess
import time
import os
import sys
import json
import tempfile
from typing import Optional, Dict, Any, List

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))

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


class FullSystemWithSimulatorTest:
    """Full system integration tests with simulator providing sensor data."""
    
    def __init__(self):
        self.processes: List[subprocess.Popen] = []
        self.test_node: Optional[Node] = None
        self.executor = None
        self.test_results: Dict[str, Any] = {
            'connectivity': {},
            'data_availability': {},
            'integration': {},
            'simulator': {}
        }
        self.mock_script_path: Optional[str] = None
        
    def setup_method(self):
        """Set up test environment."""
        if not ROS2_AVAILABLE:
            pytest.skip("ROS2 not available")
        
        if not rclpy.ok():
            rclpy.shutdown()
        rclpy.init()
        
        self.test_node = Node("full_system_simulator_test")
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
        
        if self.mock_script_path and os.path.exists(self.mock_script_path):
            try:
                os.unlink(self.mock_script_path)
            except:
                pass
        
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
                text=True
            )
            self.processes.append(proc)
            time.sleep(wait_time)
            
            # Verify node is running
            result = subprocess.run(
                ["ros2", "node", "list"],
                capture_output=True,
                text=True,
                timeout=5
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
    
    def start_mock_sensor_publisher(self) -> bool:
        """Start a simple mock sensor publisher."""
        env = os.environ.copy()
        env["ROS_DOMAIN_ID"] = "42"
        
        # Create a simple Python script to publish odometry
        mock_script = """#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
import time
import math

class MockSensorPublisher(Node):
    def __init__(self):
        super().__init__('mock_sensor_publisher')
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.slam_pub = self.create_publisher(PoseStamped, '/slam/pose', 10)
        self.timer = self.create_timer(0.1, self.publish_data)
        self.start_time = time.time()
        self.get_logger().info('Mock sensor publisher started')
    
    def publish_data(self):
        # Publish odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'odom'
        current_time = time.time() - self.start_time
        odom.pose.pose.position.x = 0.1 * math.sin(0.1 * current_time)
        odom.pose.pose.position.y = 0.05 * current_time
        odom.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(odom)
        
        # Publish SLAM pose
        slam = PoseStamped()
        slam.header.stamp = self.get_clock().now().to_msg()
        slam.header.frame_id = 'map'
        slam.pose.position.x = odom.pose.pose.position.x
        slam.pose.position.y = odom.pose.pose.position.y
        slam.pose.orientation.w = 1.0
        self.slam_pub.publish(slam)

if __name__ == '__main__':
    rclpy.init()
    node = MockSensorPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
"""
        
        # Write and execute mock script
        with tempfile.NamedTemporaryFile(mode='w', suffix='.py', delete=False) as f:
            f.write(mock_script)
            self.mock_script_path = f.name
        
        try:
            os.chmod(self.mock_script_path, 0o755)
            proc = subprocess.Popen(
                ["python3", self.mock_script_path],
                env=env,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE,
                text=True
            )
            self.processes.append(proc)
            time.sleep(2)
            
            # Check if topics are being published
            result = subprocess.run(
                ["ros2", "topic", "info", "/odom"],
                capture_output=True,
                text=True,
                timeout=5
            )
            
            if "Publisher count: 1" in result.stdout or "Publisher count: 1" in result.stderr:
                self.test_node.get_logger().info("Mock sensor publisher started")
                return True
            return False
        except Exception as e:
            self.test_node.get_logger().error(f"Failed to start mock publisher: {e}")
            return False
    
    def start_sensor_simulator(self) -> bool:
        """Start sensor simulator to provide odometry and SLAM data."""
        # Try to find sensor simulator
        simulator_paths = [
            "src/autonomy/perception/simulation/autonomy_simulation/sensor_simulator.py",
            "simulation/enhanced_simulation.py",
            "tests/mock_topics_publisher.py"
        ]
        
        for path in simulator_paths:
            full_path = os.path.join(os.path.dirname(__file__), '../..', path)
            if os.path.exists(full_path):
                return self.start_node(
                    ["python3", full_path],
                    "sensor_simulator",
                    wait_time=3.0
                )
        
        # If no simulator found, create a simple mock publisher
        return self.start_mock_sensor_publisher()
    
    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_full_system_with_simulator(self):
        """Test full system with simulator providing sensor data."""
        # Start sensor simulator first
        sim_started = self.start_sensor_simulator()
        assert sim_started, "Sensor simulator failed to start"
        
        # Start BT orchestrator
        bt_started = self.start_node(
            ["ros2", "run", "autonomy_bt", "bt_orchestrator"],
            "bt_orchestrator",
            wait_time=5
        )
        assert bt_started, "BT orchestrator failed to start"
        
        # Start state machine
        state_machine_script = os.path.join(
            os.path.dirname(__file__), 
            '../../src/core/adaptive_state_machine.py'
        )
        state_started = False
        if os.path.exists(state_machine_script):
            state_started = self.start_node(
                ["python3", state_machine_script],
                "adaptive_state_machine",
                wait_time=3
            )
        
        time.sleep(3)
        
        # Verify nodes
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        assert "bt_orchestrator" in result.stdout, "BT orchestrator not found"
        if state_started:
            assert "adaptive_state_machine" in result.stdout or "state" in result.stdout.lower(), "State machine not found"
        
        # Verify topics have publishers
        odom_info = subprocess.run(
            ["ros2", "topic", "info", "/odom"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        has_odom_publisher = "Publisher count: 1" in odom_info.stdout or "Publisher count: 1" in odom_info.stderr
        self.test_results['simulator']['odom_publisher'] = has_odom_publisher
        assert has_odom_publisher, "Odometry topic has no publisher"
        
        # Verify data is flowing
        echo_result = subprocess.run(
            ["timeout", "2", "ros2", "topic", "echo", "/odom", "--once"],
            capture_output=True,
            text=True,
            timeout=3
        )
        
        has_odom_data = echo_result.returncode == 0 and len(echo_result.stdout) > 0
        self.test_results['data_availability']['odom_data'] = has_odom_data
        assert has_odom_data, "No odometry data received"
    
    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_blackboard_updates_from_simulator(self):
        """Test that BT blackboard receives updates from simulator."""
        # Start simulator
        assert self.start_sensor_simulator(), "Simulator failed to start"
        
        # Start BT orchestrator
        assert self.start_node(
            ["ros2", "run", "autonomy_bt", "bt_orchestrator"],
            "bt_orchestrator",
            wait_time=5
        ), "BT orchestrator failed to start"
        
        time.sleep(3)
        
        # Verify odometry is being published
        result = subprocess.run(
            ["timeout", "2", "ros2", "topic", "echo", "/odom", "--once"],
            capture_output=True,
            text=True,
            timeout=3
        )
        
        has_data = result.returncode == 0 and len(result.stdout) > 0
        self.test_results['data_availability']['blackboard_odom'] = has_data
        
        # BT orchestrator should be subscribed to /odom
        # Verify subscription exists
        odom_info = subprocess.run(
            ["ros2", "topic", "info", "/odom"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        has_subscriber = "Subscription count: 2" in odom_info.stdout or "Subscription count: 1" in odom_info.stdout
        self.test_results['connectivity']['bt_odom_subscription'] = has_subscriber
        
        assert has_data, "No odometry data from simulator"
        assert has_subscriber, "BT orchestrator not subscribed to odometry"
    
    @pytest.mark.skipif(not ROS2_AVAILABLE, reason="ROS2 not available")
    def test_consistency_with_simulator(self):
        """Comprehensive consistency check with simulator running."""
        # Start all components
        assert self.start_sensor_simulator(), "Simulator failed"
        assert self.start_node(
            ["ros2", "run", "autonomy_bt", "bt_orchestrator"],
            "bt_orchestrator"
        ), "BT orchestrator failed"
        
        state_machine_script = os.path.join(
            os.path.dirname(__file__), 
            '../../src/core/adaptive_state_machine.py'
        )
        if os.path.exists(state_machine_script):
            self.start_node(
                ["python3", state_machine_script],
                "adaptive_state_machine"
            )
        
        time.sleep(3)
        
        consistency_report = {
            'nodes': {},
            'topics': {},
            'services': {},
            'data_flow': {}
        }
        
        # Check nodes
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        nodes = result.stdout
        consistency_report['nodes']['bt_orchestrator'] = "bt_orchestrator" in nodes
        consistency_report['nodes']['state_machine'] = "adaptive_state_machine" in nodes or "state" in nodes.lower()
        consistency_report['nodes']['simulator'] = "sensor_simulator" in nodes or "mock_sensor" in nodes.lower()
        
        # Check topics with publishers
        odom_info = subprocess.run(
            ["ros2", "topic", "info", "/odom"],
            capture_output=True,
            text=True,
            timeout=5
        )
        consistency_report['topics']['/odom'] = "/odom" in subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True,
            text=True
        ).stdout
        consistency_report['topics']['/odom_has_publisher'] = "Publisher count: 1" in odom_info.stdout or "Publisher count: 1" in odom_info.stderr
        
        slam_info = subprocess.run(
            ["ros2", "topic", "info", "/slam/pose"],
            capture_output=True,
            text=True,
            timeout=5
        )
        consistency_report['topics']['/slam/pose'] = "/slam/pose" in subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True,
            text=True
        ).stdout
        consistency_report['topics']['/slam_has_publisher'] = "Publisher count: 1" in slam_info.stdout or "Publisher count: 1" in slam_info.stderr
        
        # Check data flow
        odom_echo = subprocess.run(
            ["timeout", "2", "ros2", "topic", "echo", "/odom", "--once"],
            capture_output=True,
            text=True,
            timeout=3
        )
        consistency_report['data_flow']['odom_publishing'] = (
            odom_echo.returncode == 0 and len(odom_echo.stdout) > 0
        )
        
        self.test_results['consistency'] = consistency_report
        
        # Print report
        self.test_node.get_logger().info(f"Consistency Report: {json.dumps(consistency_report, indent=2)}")
        
        # Assertions
        assert consistency_report['nodes']['bt_orchestrator'], "BT orchestrator not running"
        assert consistency_report['topics']['/odom_has_publisher'], "Odometry has no publisher"
        assert consistency_report['data_flow']['odom_publishing'], "Odometry not publishing data"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short", "-s"])
