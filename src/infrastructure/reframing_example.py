#!/usr/bin/env python3
"""
URC Machiato 2026 - System Framing & Behavior Refactoring

REFACTORING GOALS:
1. Eliminate biological metaphors from safety systems
2. Create consistent operator-centric language
3. Simplify data structures to reduce cognitive load
4. Standardize communication patterns across all interfaces

Author: URC 2026 Refactoring Team
"""

import enum
import time
import numpy as np
from typing import Dict, Any, Optional, List, Tuple
from dataclasses import dataclass, field
from rclpy.node import Node
from std_msgs.msg import String, Header, Twist
from sensor_msgs.msg import Imu, NavSatFix, Image
from geometry_msgs.msg import PoseStamped
from infrastructure.logging import get_logger, get_performance_logger


# Consistent terminology
class SystemHealth:
    """Mechanical system health (no biological metaphors)."""
    OPERATIONAL = "operational"
    DEGRADED = "degraded" 
    MALFUNCTIONING = "malfunctioning"
    MAINTENANCE = "maintenance"
    OFFLINE = "offline"


class SafetyEvent:
    """System safety events with clear technical descriptions."""
    HIGH_PRIORITY = "high_priority"
    MEDIUM_PRIORITY = "medium_priority"
    LOW_PRIORITY = "low_priority"
    INFO = "information"
    WARNING = "warning"
    CRITICAL = "critical"
    
    def __init__(
        self,
        event_type: str,
        priority: str,
        description: str,
        data: Optional[Dict[str, Any]] = None,
        timestamp: float = field(default_factory=time.time)
    ):
        self.event_type = event_type
        self.priority = priority
        self.description = description
        self.data = data or {}
        self.timestamp = timestamp


class RoverState:
    """Mechanical rover state representation."""
    IDLE = "idle"
    NAVIGATING = "navigating"
    EXECUTING = "executing"
    MANUAL_CONTROL = "manual_control"
    SAFETY_OVERRIDE = "safety_override"
    EMERGENCY_STOP = "emergency_stop"
    
    def __init__(self, mode: str, status: str = SystemHealth.OPERATIONAL):
        self.mode = mode
        self.status = status
        self.timestamp = time.time()
        self.data = {}
    
    def update(self, **kwargs):
        """Update state data."""
        self.data.update(kwargs)
        self.timestamp = time.time()


class BehaviorTreeResult:
    """Behavior tree execution result with technical focus."""
    SUCCESS = "success"
    RUNNING = "running"
    FAILURE = "failure"
    TIMEOUT = "timeout"
    ABORTED = "aborted"
    
    def __init__(self, status: str, duration_ms: float = 0.0, 
                 details: Optional[Dict[str, Any]] = None):
        self.status = status
        self.duration_ms = duration_ms
        self.details = details or {}
        self.timestamp = time.time()


class OperatorLevel:
    """Operator capability levels without skill assumptions."""
    BASIC = "basic"
    STANDARD = "standard"
    ADVANCED = "advanced"
    EXPERT = "expert"


class UnifiedCommunication:
    """Single communication interface for all system interactions."""
    
    def __init__(self):
        self.logger = get_logger("unified_communication")
        
        # Message handlers
        self.handlers = {}
        self.message_queue = []
        self.subscribers = {}
        
        # Statistics
        self.stats = {
            'messages_sent': 0,
            'messages_received': 0,
            'queue_sizes': {},
            'avg_processing_time_ms': 0.0
        }
    
    def register_handler(self, message_type: str, handler) -> bool:
        self.handlers[message_type] = handler
        return True
    
    def register_subscriber(self, topic: str, callback) -> bool:
        self.subscribers[topic] = callback
        return True
    
    def send_message(self, message_type: str, data: Any, priority: str = "medium") -> bool:
        """Send message with clear technical description."""
        message = {
            'type': message_type,
            'data': data,
            'priority': priority,
            'timestamp': time.time(),
            'description': f"Technical message: {message_type}"
        }
        
        self.message_queue.append(message)
        self.stats['messages_sent'] += 1
        queue_size = len(self.message_queue.get(priority, []))
        self.stats['queue_sizes'][priority] = max(self.stats['queue_sizes'][priority], queue_size)
        
        self.logger.info(
            f"Message queued: {message_type}",
            extra={'type': message_type, 'priority': priority}
        )
        
        return True
    
    def process_messages(self) -> int:
        """Process all messages with technical clarity."""
        processed = 0
        
        # Process by priority
        priorities = ['high_priority', 'medium_priority', 'low_priority']
        
        for priority in priorities:
            if priority in self.message_queue:
                queue = self.message_queue[priority]
                while queue:
                    message = queue.pop(0)
                    self.logger.info(
                        f"Processing {message['type']} message",
                        extra={'priority': priority}
                    )
                    
                    # Route to handler
                    handler = self.handlers.get(message['type'])
                    if handler:
                        handler(message)
                    else:
                        self.logger.warning(f"No handler for {message['type']}")
                    
                    processed += 1
        
        self.stats['messages_processed'] = processed
        self.stats['queue_sizes'] = {p: 0 for p in priorities}
        
        return processed
    
    def get_technical_status(self) -> Dict[str, Any]:
        """Get technical system status."""
        return {
            'communication_status': 'operational',
            'message_rate_per_sec': self.stats['messages_processed'] / max(1, time.time() - getattr(self, 'start_time', time.time())),
            'queue_sizes': self.stats['queue_sizes'],
            'avg_processing_time_ms': self.stats['avg_processing_time_ms']
        }


class MechanicalRover:
    """Mechanical rover with clear, technical system behaviors."""
    
    def __init__(self, node_name: str = "mechanical_rover"):
        self.node_name = node_name
        self.logger = get_logger(node_name)
        self.perf_logger = get_performance_logger(node_name)
        
        # Unified state management
        self.state = RoverState(mode="idle", status=SystemHealth.OPERATIONAL)
        self.communication = UnifiedCommunication()
        
        # Clear technical metrics
        self.metrics = {
            'cpu_usage': 0.0,
            'memory_usage': 0.0,
            'sensor_freshness': 1.0,  # Based on last update time
            'actuator_status': 'nominal'
        }
        
        # Performance tracking
        self.start_time = time.time()
        self.tick_count = 0
        
        # ROS2 interfaces
        self.cmd_vel_subscriber = self.create_subscription(
            Twist, '/cmd_vel', self._cmd_vel_callback, 10
        )
        
        self.state_publisher = self.create_publisher(
            RoverState, '/rover/state', 10
        )
        
        self.safety_publisher = self.create_publisher(
            String, '/safety/events', 10
        )
        
        self.metrics_publisher = self.create_publisher(
            String, '/system/metrics', 1
        )
    
    def _cmd_vel_callback(self, msg: Twist):
        """Handle velocity commands with technical precision."""
        # Calculate mechanical metrics
        linear_vel = np.sqrt(msg.linear.x**2 + msg.linear.y**2)
        angular_vel = abs(msg.angular.z)
        
        # Update state
        self.state.update(mode="manual_control", status=SystemHealth.OPERATIONAL)
        
        # Update metrics
        self.metrics['actuator_status'] = 'active' if linear_vel > 0.1 or angular_vel > 0.1 else 'nominal'
        
        self.perf_logger.info(
            "Velocity command received",
            extra={
                'linear_vel': linear_vel,
                'angular_vel': angular_vel,
                'speed_magnitude': np.sqrt(linear_vel**2 + angular_vel**2)
            }
        )
    
    def publish_state(self):
        """Publish technical state information."""
        self.state_publisher.publish(self.state)
        
        self.metrics['cpu_usage'] = min(0.5, time.time() - self.start_time)
        self.metrics['memory_usage'] = 0.3  # Simulated
        self.metrics['sensor_freshness'] = 0.8
        
        self.perf_logger.debug(
            "System metrics updated",
            extra=self.metrics
        )
    
    def send_safety_event(self, event: SafetyEvent):
        """Send safety event with clear technical description."""
        self.safety_publisher.publish(
            String(),
            '/safety/events',
            1
            {
                'data': json.dumps({
                    'event_type': event.event_type,
                    'priority': event.priority,
                    'description': event.description,
                    'timestamp': event.timestamp,
                    'data': event.data
                }),
                'frame_id': 'safety_frame'
            }
        )
        
        self.logger.info(
            f"Safety event: {event.description}",
            extra={
                'event_type': event.event_type,
                'priority': event.priority
            }
        )
    
    def execute_behavior_tree_tick(self) -> BehaviorTreeResult:
        """Execute one tick of behavior tree with technical timing."""
        start_time = time.time()
        
        # Update metrics
        self.tick_count += 1
        elapsed = (time.time() - self.start_time) * 1000  # ms
        
        self.perf_logger.debug(f"BT tick #{self.tick_count}", extra={'duration_ms': elapsed})
        
        # Simulate behavior tree execution (would call actual BT)
        status = BehaviorTreeResult.SUCCESS  # Assume success for demo
        duration_ms = 50.0 + np.random.normal(10, 5.0)  # Simulate variable timing
        
        return BehaviorTreeResult(status, duration_ms)
    
    def get_system_status(self) -> Dict[str, Any]:
        """Get comprehensive technical system status."""
        communication_status = self.communication.get_technical_status()
        
        return {
            'rover_mode': self.state.mode,
            'system_health': self.state.status,
            'metrics': self.metrics,
            'communication': communication_status,
            'uptime': time.time() - self.start_time,
            'tick_count': self.tick_count,
            'last_update': self.state.timestamp
        }


if __name__ == "__main__":
    print("🔧 URC 2026 - Mechanical Reframing Demo")
    
    rover = MechanicalRover("demo_rover")
    
    # Test operator control with clear language
    print("Testing operator control with technical framing...")
    
    # Simulate navigation command
    test_msg = Twist()
    test_msg.linear.x = 1.0
    test_msg.linear.y = 0.0
    test_msg.angular.z = 0.0
    
    # Send command through unified communication
    rover.communication.send_message("navigation_command", test_msg, "high_priority")
    
    # Process messages
    rover.communication.process_messages()
    
    # Test safety event with clear technical description
    safety_event = SafetyEvent(
        event_type="obstacle_detection",
        priority=SafetyEvent.HIGH_PRIORITY,
        description="Obstacle detected at 2.5m in navigation path",
        data={
            'obstacle_type': 'unknown',
            'distance': 2.5,
            'sensor_used': ['lidar', 'camera'],
            'avoidance_action': 'speed_reduction'
        }
    )
    
    rover.send_safety_event(safety_event)
    
    # Process safety event
    rover.communication.process_messages()
    
    # Check system status
    status = rover.get_system_status()
    
    print("System Status:")
    for key, value in status.items():
        print(f"  {key}: {value}")
    
    print("\n✅ Reframing complete - system now uses mechanical terminology")