#!/usr/bin/env python3
"""
ROS2 Message Adapter for Simulation

Converts between dictionary messages used in WebSocket/simulation
and actual ROS2 message types.

Author: URC 2026 Integration Team
"""

import sys
from pathlib import Path
from typing import Dict, Any, Optional

# Add src to path for ros2_mock
sys.path.insert(0, str(Path(__file__).parent.parent.parent / "src"))

from core.ros2_mock import Twist, PoseStamped, NavSatFix


class ROS2MessageAdapter:
    """Adapter between simulation dictionaries and ROS2 messages."""
    
    @staticmethod
    def dict_to_twist(data: Dict[str, Any]) -> Twist:
        """Convert dictionary to Twist message.
        
        Args:
            data: Dictionary with 'linear' and 'angular' keys
            
        Returns:
            Twist message
        """
        twist = Twist()
        
        # Handle nested dict or direct values
        if isinstance(data.get('linear'), dict):
            twist.linear_x = data['linear'].get('x', 0.0)
            twist.linear_y = data['linear'].get('y', 0.0)
            twist.linear_z = data['linear'].get('z', 0.0)
        else:
            twist.linear_x = data.get('linear', 0.0)
            twist.linear_y = 0.0
            twist.linear_z = 0.0
        
        if isinstance(data.get('angular'), dict):
            twist.angular_x = data['angular'].get('x', 0.0)
            twist.angular_y = data['angular'].get('y', 0.0)
            twist.angular_z = data['angular'].get('z', 0.0)
        else:
            twist.angular_x = 0.0
            twist.angular_y = 0.0
            twist.angular_z = data.get('angular', 0.0)
        
        return twist
    
    @staticmethod
    def twist_to_dict(twist: Twist) -> Dict[str, Any]:
        """Convert Twist message to dictionary.
        
        Args:
            twist: Twist message
            
        Returns:
            Dictionary representation
        """
        return {
            'linear': {
                'x': twist.linear_x,
                'y': twist.linear_y,
                'z': twist.linear_z
            },
            'angular': {
                'x': twist.angular_x,
                'y': twist.angular_y,
                'z': twist.angular_z
            },
            'timestamp': twist.timestamp
        }
    
    @staticmethod
    def dict_to_twist_with_frame(data: Dict[str, Any], frame_id: str = "base_link") -> Twist:
        """Convert dictionary to Twist message with frame ID.
        
        Args:
            data: Dictionary with twist data
            frame_id: Frame ID for the message
            
        Returns:
            Twist message with frame_id set
        """
        twist = ROS2MessageAdapter.dict_to_twist(data)
        twist.frame_id = frame_id
        return twist
    
    @staticmethod
    def dict_to_pose_stamped(data: Dict[str, Any], frame_id: str = "map") -> PoseStamped:
        """Convert dictionary to PoseStamped message.
        
        Args:
            data: Dictionary with position and orientation
            frame_id: Frame ID
            
        Returns:
            PoseStamped message
        """
        pose = PoseStamped()
        pose.frame_id = frame_id
        
        if 'position' in data:
            pose.position_x = data['position'].get('x', 0.0)
            pose.position_y = data['position'].get('y', 0.0)
            pose.position_z = data['position'].get('z', 0.0)
        
        if 'orientation' in data:
            pose.orientation_x = data['orientation'].get('x', 0.0)
            pose.orientation_y = data['orientation'].get('y', 0.0)
            pose.orientation_z = data['orientation'].get('z', 0.0)
            pose.orientation_w = data['orientation'].get('w', 1.0)
        
        return pose
    
    @staticmethod
    def simplified_twist(linear: float, angular: float) -> Twist:
        """Create Twist message from simplified inputs.
        
        Args:
            linear: Linear velocity (m/s) - forward/backward
            angular: Angular velocity (rad/s) - rotation
            
        Returns:
            Twist message
        """
        twist = Twist()
        twist.linear_x = linear
        twist.angular_z = angular
        return twist


class ROS2TopicBridge:
    """Bridge for ROS2 topic simulation."""
    
    def __init__(self):
        """Initialize topic bridge."""
        self.topics: Dict[str, list] = {}
        self.subscribers: Dict[str, list] = {}
        
    def publish(self, topic: str, message: Any):
        """Publish message to topic.
        
        Args:
            topic: Topic name
            message: ROS2 message
        """
        if topic not in self.topics:
            self.topics[topic] = []
        
        self.topics[topic].append(message)
        
        # Call subscribers
        if topic in self.subscribers:
            for callback in self.subscribers[topic]:
                try:
                    callback(message)
                except Exception as e:
                    print(f"Subscriber error: {e}")
    
    def subscribe(self, topic: str, callback: callable):
        """Subscribe to topic.
        
        Args:
            topic: Topic name
            callback: Callback function
        """
        if topic not in self.subscribers:
            self.subscribers[topic] = []
        
        self.subscribers[topic].append(callback)
    
    def get_latest(self, topic: str) -> Optional[Any]:
        """Get latest message on topic.
        
        Args:
            topic: Topic name
            
        Returns:
            Latest message or None
        """
        if topic in self.topics and self.topics[topic]:
            return self.topics[topic][-1]
        return None
    
    def get_history(self, topic: str, count: int = 10) -> list:
        """Get message history for topic.
        
        Args:
            topic: Topic name
            count: Number of messages to retrieve
            
        Returns:
            List of messages
        """
        if topic in self.topics:
            return self.topics[topic][-count:]
        return []
    
    def clear_topic(self, topic: str):
        """Clear topic history.
        
        Args:
            topic: Topic name
        """
        if topic in self.topics:
            self.topics[topic] = []


# Standard topic names
STANDARD_TOPICS = {
    'cmd_vel_teleop': '/cmd_vel/teleop',
    'cmd_vel_autonomy': '/cmd_vel/autonomy',
    'cmd_vel_out': '/cmd_vel',
    'velocity_feedback': '/velocity_feedback',
    'odom': '/odom',
    'pose': '/pose',
    'gps': '/gps/fix',
    'imu': '/imu/data',
}


def create_topic_bridge():
    """Create and return a configured topic bridge."""
    return ROS2TopicBridge()


if __name__ == '__main__':
    # Test adapter
    print("Testing ROS2 Message Adapter...")
    
    # Test dict to Twist
    data = {'linear': 0.5, 'angular': 0.2}
    twist = ROS2MessageAdapter.dict_to_twist(data)
    print(f"✅ Dict to Twist: linear_x={twist.linear_x}, angular_z={twist.angular_z}")
    
    # Test Twist to dict
    twist_dict = ROS2MessageAdapter.twist_to_dict(twist)
    print(f"✅ Twist to Dict: {twist_dict}")
    
    # Test topic bridge
    bridge = create_topic_bridge()
    
    def subscriber_callback(msg):
        print(f"✅ Received: linear_x={msg.linear_x}")
    
    bridge.subscribe('/cmd_vel', subscriber_callback)
    bridge.publish('/cmd_vel', twist)
    
    latest = bridge.get_latest('/cmd_vel')
    print(f"✅ Latest message: {latest}")
    
    print("\n✅ All adapter tests passed!")
