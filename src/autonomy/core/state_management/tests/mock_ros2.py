#!/usr/bin/env python3
"""
Mock ROS2 Interfaces for Testing

Provides mock implementations of ROS2 components to enable testing
without requiring full ROS2 environment setup.
"""

import threading
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional


@dataclass
class MockMessage:
    """Base class for mock ROS2 messages."""
    timestamp: float = None

    def __post_init__(self):
        if self.timestamp is None:
            self.timestamp = time.time()


class MockPublisher:
    """Mock ROS2 publisher for testing."""

    def __init__(self, topic_name: str, message_type: type, qos_profile=None):
        self.topic_name = topic_name
        self.message_type = message_type
        self.qos_profile = qos_profile
        self.published_messages = []
        self.publish_count = 0

    def publish(self, message):
        """Mock publish method."""
        self.published_messages.append(message)
        self.publish_count += 1

    def get_last_message(self):
        """Get the last published message."""
        return self.published_messages[-1] if self.published_messages else None

    def get_message_count(self) -> int:
        """Get total number of published messages."""
        return self.publish_count


class MockSubscription:
    """Mock ROS2 subscription for testing."""

    def __init__(self, topic_name: str, message_type: type, callback: Callable, qos_profile=None):
        self.topic_name = topic_name
        self.message_type = message_type
        self.callback = callback
        self.qos_profile = qos_profile
        self.received_messages = []
        self.receive_count = 0

    def inject_message(self, message):
        """Inject a message to simulate topic reception."""
        self.received_messages.append(message)
        self.receive_count += 1
        self.callback(message)

    def get_received_messages(self) -> List:
        """Get all received messages."""
        return self.received_messages.copy()


class MockService:
    """Mock ROS2 service for testing."""

    def __init__(self, service_name: str, service_type: type, callback: Callable):
        self.service_name = service_name
        self.service_type = service_type
        self.callback = callback
        self.call_count = 0
        self.last_request = None
        self.last_response = None

    def call(self, request):
        """Mock service call."""
        self.call_count += 1
        self.last_request = request
        self.last_response = self.callback(request)
        return self.last_response


class MockTimer:
    """Mock ROS2 timer for testing."""

    def __init__(self, period: float, callback: Callable, clock=None):
        self.period = period
        self.callback = callback
        self.clock = clock
        self.is_active = False
        self.thread = None
        self.cancelled = False

    def start(self):
        """Start the mock timer."""
        if self.is_active:
            return

        self.is_active = True
        self.cancelled = False
        self.thread = threading.Thread(target=self._timer_loop, daemon=True)
        self.thread.start()

    def cancel(self):
        """Cancel the mock timer."""
        self.cancelled = True
        self.is_active = False
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=1.0)

    def _timer_loop(self):
        """Timer loop implementation."""
        while not self.cancelled:
            time.sleep(self.period)
            if not self.cancelled:
                try:
                    # Timer callback logic here
                    pass
                except Exception as e:
                    self.get_logger().info(f"Timer callback error: {e}")
class MockNode:
    """Mock ROS2 node for testing."""

    def __init__(self, node_name: str = "mock_node"):
        self.node_name = node_name
        self.publishers = {}
        self.subscriptions = {}
        self.services = {}
        self.timers = []
        self.parameters = {}
        self.logger = MockLogger()

    def create_publisher(self, msg_type, topic, qos_profile=None, **kwargs):
        """Create a mock publisher."""
        publisher = MockPublisher(topic, msg_type, qos_profile)
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(self, msg_type, topic, callback, qos_profile=None, **kwargs):
        """Create a mock subscription."""
        subscription = MockSubscription(topic, msg_type, callback, qos_profile)
        self.subscriptions[topic] = subscription
        return subscription

    def create_service(self, srv_type, service_name, callback, **kwargs):
        """Create a mock service."""
        service = MockService(service_name, srv_type, callback)
        self.services[service_name] = service
        return service

    def create_timer(self, period, callback, **kwargs):
        """Create a mock timer."""
        timer = MockTimer(period, callback)
        self.timers.append(timer)
        return timer

    def declare_parameter(self, name: str, value: Any = None, **kwargs):
        """Declare a parameter."""
        self.parameters[name] = value

    def get_parameter(self, name: str):
        """Get a parameter value."""
        return MockParameter(self.parameters.get(name))

    def destroy_node(self):
        """Clean up node resources."""
        # Cancel all timers
        for timer in self.timers:
            timer.cancel()

        # Clear collections
        self.publishers.clear()
        self.subscriptions.clear()
        self.services.clear()
        self.timers.clear()


class MockParameter:
    """Mock parameter for testing."""

    def __init__(self, value):
        self._value = value

    def get_parameter_value(self):
        """Get parameter value (ROS2 style)."""
        return self._value

    @property
    def value(self):
        """Get parameter value directly."""
        return self._value


class MockLogger:
    """Mock logger for testing."""

    def __init__(self):
        self.messages = []
        self.level = "INFO"

    def info(self, msg: str):
        """Log info message."""
        self._log("INFO", msg)

    def warning(self, msg: str):
        """Log warning message."""
        self._log("WARNING", msg)

    def error(self, msg: str):
        """Log error message."""
        self._log("ERROR", msg)

    def debug(self, msg: str):
        """Log debug message."""
        self._log("DEBUG", msg)

    def _log(self, level: str, msg: str):
        """Internal logging method."""
        entry = {
            'level': level,
            'message': msg,
            'timestamp': time.time()
        }
        self.messages.append(entry)
        self.get_logger().info(f"[{level}] {msg}")
    def get_messages(self, level: str = None) -> List[Dict]:
        """Get logged messages, optionally filtered by level."""
        if level:
            return [msg for msg in self.messages if msg['level'] == level]
        return self.messages.copy()


class MockClock:
    """Mock ROS2 clock for testing."""

    def __init__(self):
        self._start_time = time.time()

    def now(self):
        """Get current time as ROS2 Time message."""
        return MockTimeMessage(time.time() - self._start_time)


class MockTimeMessage:
    """Mock ROS2 Time message."""

    def __init__(self, seconds: float):
        self.seconds = int(seconds)
        self.nanoseconds = int((seconds - self.seconds) * 1e9)

    def to_msg(self):
        """Convert to ROS2 message format."""
        return self


# Mock ROS2 modules for import compatibility
class MockRCLPY:
    """Mock rclpy module."""

    @staticmethod
    def init():
        """Mock initialization."""
        pass

    @staticmethod
    def shutdown():
        """Mock shutdown."""
        pass

    @staticmethod
    def spin_once(node=None, timeout_sec=None):
        """Mock spin once."""
        time.sleep(0.001)

    class QoSProfile:
        """Mock QoS profile."""
        def __init__(self, **kwargs):
            for key, value in kwargs.items():
                setattr(self, key, value)

    class Node:
        """Factory for mock nodes."""
        def __new__(cls, node_name: str = "mock_node"):
            return MockNode(node_name)


# Global mock instances
mock_rclpy = MockRCLPY()

# Monkey patch for testing (only if ROS2 not available)
def setup_mock_environment():
    """Setup mock environment when ROS2 is not available."""
    try:
        import rclpy

        # ROS2 is available, don't mock
        return False
    except ImportError:
        # ROS2 not available, setup mocks
        import sys
        sys.modules['rclpy'] = mock_rclpy
        return True


if __name__ == "__main__":
    # Test the mock environment
        self.get_logger().info("[TEST] Testing Mock ROS2 Environment")
    if setup_mock_environment():
        self.get_logger().info("[SUCCESS] Mock environment setup successful")
        # Test basic functionality
        import rclpy
        node = rclpy.Node("test_node")

        publisher = node.create_publisher(str, "/test_topic", None)
        publisher.publish("test message")
        self.get_logger().info(f"[SUCCESS] Published {publisher.get_message_count()} messages")
        timer = node.create_timer(0.1, lambda: None)
        timer.start()
        time.sleep(0.05)
        timer.cancel()
        self.get_logger().info("[SUCCESS] Timer functionality working")
        node.destroy_node()
        self.get_logger().info("[SUCCESS] Mock environment test completed successfully")
    else:
        self.get_logger().info("ℹ️  ROS2 is available, using real implementation")
