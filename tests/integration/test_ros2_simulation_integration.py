#!/usr/bin/env python3
"""
ROS2 Simulation Integration Tests

Tests ROS2 message compatibility and topic bridging in simulation.

Author: URC 2026 Testing Team
"""

import pytest
import asyncio
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from simulation.ros2.ros2_message_adapter import (
    ROS2MessageAdapter, ROS2TopicBridge, STANDARD_TOPICS
)
from simulation.integration.full_stack_simulator import create_full_stack_simulator
from src.core.ros2_mock import Twist


class TestROS2MessageAdapter:
    """Test ROS2 message adapter."""
    
    def test_dict_to_twist_simple(self):
        """Test simple dict to Twist conversion."""
        data = {'linear': 0.5, 'angular': 0.2}
        twist = ROS2MessageAdapter.dict_to_twist(data)
        
        assert twist.linear_x == 0.5
        assert twist.angular_z == 0.2
        assert twist.linear_y == 0.0
    
    def test_dict_to_twist_nested(self):
        """Test nested dict to Twist conversion."""
        data = {
            'linear': {'x': 0.5, 'y': 0.1, 'z': 0.0},
            'angular': {'x': 0.0, 'y': 0.0, 'z': 0.2}
        }
        twist = ROS2MessageAdapter.dict_to_twist(data)
        
        assert twist.linear_x == 0.5
        assert twist.linear_y == 0.1
        assert twist.angular_z == 0.2
    
    def test_twist_to_dict(self):
        """Test Twist to dict conversion."""
        twist = Twist()
        twist.linear_x = 0.5
        twist.angular_z = 0.2
        
        data = ROS2MessageAdapter.twist_to_dict(twist)
        
        assert data['linear']['x'] == 0.5
        assert data['angular']['z'] == 0.2
    
    def test_roundtrip_conversion(self):
        """Test roundtrip dict → Twist → dict."""
        original = {'linear': 0.5, 'angular': 0.2}
        twist = ROS2MessageAdapter.dict_to_twist(original)
        result = ROS2MessageAdapter.twist_to_dict(twist)
        
        assert result['linear']['x'] == 0.5
        assert result['angular']['z'] == 0.2
    
    def test_simplified_twist(self):
        """Test simplified twist creation."""
        twist = ROS2MessageAdapter.simplified_twist(0.5, 0.2)
        
        assert twist.linear_x == 0.5
        assert twist.angular_z == 0.2


class TestROS2TopicBridge:
    """Test ROS2 topic bridge."""
    
    @pytest.fixture
    def bridge(self):
        """Create topic bridge."""
        return ROS2TopicBridge()
    
    def test_publish_and_get_latest(self, bridge):
        """Test publishing and retrieving messages."""
        twist = Twist()
        twist.linear_x = 0.5
        
        bridge.publish('/test', twist)
        
        latest = bridge.get_latest('/test')
        assert latest is not None
        assert latest.linear_x == 0.5
    
    def test_subscribe_callback(self, bridge):
        """Test subscriber callbacks."""
        received = []
        
        def callback(msg):
            received.append(msg)
        
        bridge.subscribe('/test', callback)
        
        twist = Twist()
        twist.linear_x = 0.5
        bridge.publish('/test', twist)
        
        assert len(received) == 1
        assert received[0].linear_x == 0.5
    
    def test_message_history(self, bridge):
        """Test message history retrieval."""
        for i in range(5):
            twist = Twist()
            twist.linear_x = float(i)
            bridge.publish('/test', twist)
        
        history = bridge.get_history('/test', count=3)
        assert len(history) == 3
        assert history[-1].linear_x == 4.0
    
    def test_clear_topic(self, bridge):
        """Test clearing topic history."""
        twist = Twist()
        bridge.publish('/test', twist)
        
        assert bridge.get_latest('/test') is not None
        
        bridge.clear_topic('/test')
        
        assert bridge.get_latest('/test') is None


class TestFullStackROS2Integration:
    """Test full stack simulator with ROS2."""
    
    @pytest.fixture
    def simulator(self):
        """Create full stack simulator."""
        sim = create_full_stack_simulator('perfect')
        yield sim
        sim.shutdown()
    
    async def test_websocket_to_ros2_conversion(self, simulator):
        """Test WebSocket commands convert to ROS2 messages."""
        client_id = await simulator.websocket_sim.connect()
        
        # Send WebSocket command
        await simulator.websocket_sim.receive('driveCommands', {
            'linear': 0.5,
            'angular': 0.2
        }, client_id)
        
        await asyncio.sleep(0.1)
        
        # Check ROS2 state has Twist message
        ros2_cmd = simulator.ros2_state['cmd_vel_teleop']
        assert ros2_cmd is not None
        assert isinstance(ros2_cmd, Twist)
        assert ros2_cmd.linear_x == 0.5
        assert ros2_cmd.angular_z == 0.2
    
    async def test_ros2_topic_bridge_integration(self, simulator):
        """Test ROS2 topic bridge receives messages."""
        client_id = await simulator.websocket_sim.connect()
        
        # Subscribe to topic
        received = []
        simulator.ros2_bridge.subscribe(STANDARD_TOPICS['cmd_vel_teleop'], 
                                       lambda msg: received.append(msg))
        
        # Send command
        await simulator.websocket_sim.receive('driveCommands', {
            'linear': 0.5,
            'angular': 0.2
        }, client_id)
        
        await asyncio.sleep(0.1)
        
        # Check message received on topic
        assert len(received) > 0
        assert received[0].linear_x == 0.5
    
    async def test_ros2_message_in_firmware(self, simulator):
        """Test ROS2 messages propagate to firmware."""
        client_id = await simulator.websocket_sim.connect()
        
        # Send command
        await simulator.websocket_sim.receive('driveCommands', {
            'linear': 0.5,
            'angular': 0.0
        }, client_id)
        
        await asyncio.sleep(0.2)
        
        # Check firmware received command
        motor_status = simulator.firmware_sim.get_motor_status(0)
        assert motor_status['velocity_actual'] > 0


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
