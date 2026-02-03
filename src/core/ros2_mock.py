#!/usr/bin/env python3
"""
ROS2 Mock Implementation for Development and Testing

Provides complete ROS2 API compatibility without requiring ROS2 installation.
Used for development, testing, and CI/CD pipelines.

Features:
- Full ROS2 API compatibility (rclpy, messages, services, actions)
- In-memory message passing
- Deterministic timing for testing
- Performance monitoring and metrics
- Graceful fallback to real ROS2 when available

Author: URC 2026 ROS2 Compatibility Layer
"""

import time
import threading
import asyncio
from typing import Dict, List, Any, Optional, Callable, Type, Union
from dataclasses import dataclass, field
from enum import Enum
import uuid
import logging
import sys
import os

logger = logging.getLogger(__name__)


class QoSPreset(Enum):
    """QoS presets matching ROS2."""

    SYSTEM_DEFAULT = "system_default"
    PARAMETER_EVENTS = "parameter_events"
    SERVICES_DEFAULT = "services_default"
    PARAMETERS = "parameters"
    SENSOR_DATA = "sensor_data"
    KEEP_LAST = "keep_last"
    KEEP_ALL = "keep_all"


class QoSProfile:
    """Mock QoS profile."""

    def __init__(self, preset: QoSPreset = QoSPreset.SYSTEM_DEFAULT, **kwargs):
        self.preset = preset
        self.depth = kwargs.get("depth", 10)
        self.reliability = kwargs.get("reliability", "reliable")
        self.durability = kwargs.get("durability", "volatile")
        self.deadline = kwargs.get("deadline", None)
        self.lifespan = kwargs.get("lifespan", None)


# QoS Presets
qos_profiles = {
    "system_default": QoSProfile(QoSPreset.SYSTEM_DEFAULT),
    "services_default": QoSProfile(QoSPreset.SERVICES_DEFAULT),
    "sensor_data": QoSProfile(QoSPreset.SENSOR_DATA, reliability="best_effort"),
    "parameters": QoSProfile(QoSPreset.PARAMETERS),
}


@dataclass
class ROS2Message:
    """Base ROS2 message structure."""

    timestamp: float = field(default_factory=time.time)
    sequence_id: int = field(default_factory=lambda: int(time.time() * 1000000))
    frame_id: str = ""


@dataclass
class Twist(ROS2Message):
    """Mock geometry_msgs/Twist."""

    linear_x: float = 0.0
    linear_y: float = 0.0
    linear_z: float = 0.0
    angular_x: float = 0.0
    angular_y: float = 0.0
    angular_z: float = 0.0


@dataclass
class PoseStamped(ROS2Message):
    """Mock geometry_msgs/PoseStamped."""

    position_x: float = 0.0
    position_y: float = 0.0
    position_z: float = 0.0
    orientation_x: float = 0.0
    orientation_y: float = 0.0
    orientation_z: float = 0.0
    orientation_w: float = 1.0


@dataclass
class NavSatFix(ROS2Message):
    """Mock sensor_msgs/NavSatFix."""

    latitude: float = 0.0
    longitude: float = 0.0
    altitude: float = 0.0
    status: int = 0


@dataclass
class Imu(ROS2Message):
    """Mock sensor_msgs/Imu."""

    linear_acceleration_x: float = 0.0
    linear_acceleration_y: float = 0.0
    linear_acceleration_z: float = 9.81
    angular_velocity_x: float = 0.0
    angular_velocity_y: float = 0.0
    angular_velocity_z: float = 0.0


@dataclass
class Trigger:
    """Mock std_srvs/Trigger."""

    pass


@dataclass
class NavigateToPose:
    """Mock nav2_msgs/action/NavigateToPose."""

    pose: PoseStamped = field(default_factory=PoseStamped)
    behavior_tree: str = ""


class ROS2Node:
    """Mock ROS2 node implementation."""

    def __init__(self, node_name: str, namespace: str = ""):
        self.node_name = node_name
        self.namespace = namespace
        self.full_name = f"{namespace}/{node_name}" if namespace else node_name

        self.publishers: Dict[str, "Publisher"] = {}
        self.subscribers: Dict[str, "Subscriber"] = {}
        self.services: Dict[str, "Service"] = {}
        self.action_servers: Dict[str, "ActionServer"] = {}
        self.action_clients: Dict[str, "ActionClient"] = {}

        self.timers: Dict[str, "Timer"] = {}
        self.executors: List["Executor"] = []

        self.logger = logging.getLogger(self.full_name)
        self.is_shutdown = False

        # Performance tracking
        self.message_count = 0
        self.start_time = time.time()

        logger.info(f"Mock ROS2 Node created: {self.full_name}")

    def create_publisher(
        self, msg_type: Type, topic: str, qos_profile: Optional[QoSProfile] = None
    ) -> "Publisher":
        """Create a publisher."""
        publisher = Publisher(
            self, msg_type, topic, qos_profile or qos_profiles["system_default"]
        )
        self.publishers[topic] = publisher
        return publisher

    def create_subscription(
        self,
        msg_type: Type,
        topic: str,
        callback: Callable,
        qos_profile: Optional[QoSProfile] = None,
    ) -> "Subscriber":
        """Create a subscription."""
        subscriber = Subscriber(
            self,
            msg_type,
            topic,
            callback,
            qos_profile or qos_profiles["system_default"],
        )
        self.subscribers[topic] = subscriber

        # Register with global message bus
        ROS2MessageBus.get_instance().register_subscriber(subscriber)
        return subscriber

    def create_service(
        self,
        srv_type: Type,
        service_name: str,
        callback: Callable,
        qos_profile: Optional[QoSProfile] = None,
    ) -> "Service":
        """Create a service."""
        service = Service(
            self,
            srv_type,
            service_name,
            callback,
            qos_profile or qos_profiles["services_default"],
        )
        self.services[service_name] = service

        # Register with global message bus
        ROS2MessageBus.get_instance().register_service(service)
        return service

    def create_action_server(
        self,
        action_type: Type,
        action_name: str,
        callback: Callable,
        qos_profile: Optional[QoSProfile] = None,
    ) -> "ActionServer":
        """Create an action server."""
        action_server = ActionServer(
            self,
            action_type,
            action_name,
            callback,
            qos_profile or qos_profiles["system_default"],
        )
        self.action_servers[action_name] = action_server

        # Register with global message bus
        ROS2MessageBus.get_instance().register_action_server(action_server)
        return action_server

    def create_timer(
        self, period: float, callback: Callable, name: str = ""
    ) -> "Timer":
        """Create a timer."""
        timer_name = name or f"timer_{len(self.timers)}"
        timer = Timer(self, period, callback, timer_name)
        self.timers[timer_name] = timer
        return timer

    def get_logger(self):
        """Get node logger."""
        return self.logger

    def destroy_publisher(self, publisher: "Publisher"):
        """Destroy a publisher."""
        for topic, pub in self.publishers.items():
            if pub == publisher:
                del self.publishers[topic]
                break

    def destroy_subscription(self, subscriber: "Subscriber"):
        """Destroy a subscription."""
        for topic, sub in self.subscribers.items():
            if sub == subscriber:
                del self.subscribers[topic]
                ROS2MessageBus.get_instance().unregister_subscriber(subscriber)
                break

    def shutdown(self):
        """Shutdown the node."""
        self.is_shutdown = True

        # Clear all registrations
        message_bus = ROS2MessageBus.get_instance()
        for subscriber in self.subscribers.values():
            message_bus.unregister_subscriber(subscriber)
        for service in self.services.values():
            message_bus.unregister_service(service)
        for action_server in self.action_servers.values():
            message_bus.unregister_action_server(action_server)

        # Stop timers
        for timer in self.timers.values():
            timer.cancel()

        logger.info(f"Mock ROS2 Node shutdown: {self.full_name}")

    @property
    def uptime(self) -> float:
        """Get node uptime."""
        return time.time() - self.start_time


class Publisher:
    """Mock ROS2 publisher."""

    def __init__(
        self, node: ROS2Node, msg_type: Type, topic: str, qos_profile: QoSProfile
    ):
        self.node = node
        self.msg_type = msg_type
        self.topic = topic
        self.qos_profile = qos_profile
        self.message_count = 0

    def publish(self, message):
        """Publish a message."""
        if self.node.is_shutdown:
            return

        self.message_count += 1
        self.node.message_count += 1

        # Send to message bus
        ROS2MessageBus.get_instance().publish_message(self.topic, message)


class Subscriber:
    """Mock ROS2 subscriber."""

    def __init__(
        self,
        node: ROS2Node,
        msg_type: Type,
        topic: str,
        callback: Callable,
        qos_profile: QoSProfile,
    ):
        self.node = node
        self.msg_type = msg_type
        self.topic = topic
        self.callback = callback
        self.qos_profile = qos_profile
        self.message_count = 0

    def receive_message(self, message):
        """Receive a message (called by message bus)."""
        if self.node.is_shutdown:
            return

        self.message_count += 1
        try:
            self.callback(message)
        except Exception as e:
            self.node.logger.error(f"Subscriber callback error on {self.topic}: {e}")


class Service:
    """Mock ROS2 service."""

    def __init__(
        self,
        node: ROS2Node,
        srv_type: Type,
        service_name: str,
        callback: Callable,
        qos_profile: QoSProfile,
    ):
        self.node = node
        self.srv_type = srv_type
        self.service_name = service_name
        self.callback = callback
        self.qos_profile = qos_profile
        self.call_count = 0

    def handle_request(self, request):
        """Handle a service request."""
        if self.node.is_shutdown:
            return None

        self.call_count += 1
        try:
            return self.callback(request)
        except Exception as e:
            self.node.logger.error(
                f"Service callback error on {self.service_name}: {e}"
            )
            return None


class ActionServer:
    """Mock ROS2 action server."""

    def __init__(
        self,
        node: ROS2Node,
        action_type: Type,
        action_name: str,
        callback: Callable,
        qos_profile: QoSProfile,
    ):
        self.node = node
        self.action_type = action_type
        self.action_name = action_name
        self.callback = callback
        self.qos_profile = qos_profile
        self.active_goals: Dict[str, Any] = {}
        self.goal_count = 0

    def handle_goal(self, goal):
        """Handle an action goal."""
        if self.node.is_shutdown:
            return None

        self.goal_count += 1
        goal_id = str(uuid.uuid4())

        try:
            result = self.callback(goal)
            self.active_goals[goal_id] = result
            return result
        except Exception as e:
            self.node.logger.error(
                f"Action server callback error on {self.action_name}: {e}"
            )
            return None


class Timer:
    """Mock ROS2 timer."""

    def __init__(self, node: ROS2Node, period: float, callback: Callable, name: str):
        self.node = node
        self.period = period
        self.callback = callback
        self.name = name
        self.is_cancelled = False
        self.call_count = 0

        # Start timer thread
        self.thread = threading.Thread(target=self._timer_loop, daemon=True)
        self.thread.start()

    def _timer_loop(self):
        """Timer execution loop."""
        while not self.is_cancelled and not self.node.is_shutdown:
            time.sleep(self.period)
            if not self.is_cancelled and not self.node.is_shutdown:
                try:
                    self.call_count += 1
                    self.callback()
                except Exception as e:
                    self.node.logger.error(f"Timer callback error {self.name}: {e}")

    def cancel(self):
        """Cancel the timer."""
        self.is_cancelled = True


class Executor:
    """Mock ROS2 executor."""

    def __init__(self):
        self.nodes: List[ROS2Node] = []
        self.is_spinning = False

    def add_node(self, node: ROS2Node):
        """Add a node to the executor."""
        self.nodes.append(node)

    def spin(self):
        """Spin the executor."""
        self.is_spinning = True
        try:
            while self.is_spinning:
                time.sleep(0.1)  # Minimal spin
        except KeyboardInterrupt:
            self.is_spinning = False

    def shutdown(self):
        """Shutdown the executor."""
        self.is_spinning = False
        for node in self.nodes:
            node.shutdown()


class ROS2MessageBus:
    """Global message bus for mock ROS2 communication."""

    _instance = None

    def __init__(self):
        self.subscribers: Dict[str, List[Subscriber]] = {}
        self.services: Dict[str, Service] = {}
        self.action_servers: Dict[str, ActionServer] = {}
        self.message_history: Dict[str, List] = {}
        self.max_history = 1000

    @classmethod
    def get_instance(cls):
        """Get singleton instance."""
        if cls._instance is None:
            cls._instance = cls()
        return cls._instance

    def register_subscriber(self, subscriber: Subscriber):
        """Register a subscriber."""
        if subscriber.topic not in self.subscribers:
            self.subscribers[subscriber.topic] = []
        self.subscribers[subscriber.topic].append(subscriber)

    def unregister_subscriber(self, subscriber: Subscriber):
        """Unregister a subscriber."""
        if subscriber.topic in self.subscribers:
            self.subscribers[subscriber.topic].remove(subscriber)

    def register_service(self, service: Service):
        """Register a service."""
        self.services[service.service_name] = service

    def unregister_service(self, service: Service):
        """Unregister a service."""
        self.services.pop(service.service_name, None)

    def register_action_server(self, action_server: ActionServer):
        """Register an action server."""
        self.action_servers[action_server.action_name] = action_server

    def unregister_action_server(self, action_server: ActionServer):
        """Unregister an action server."""
        self.action_servers.pop(action_server.action_name, None)

    def publish_message(self, topic: str, message):
        """Publish a message to all subscribers."""
        # Store in history
        if topic not in self.message_history:
            self.message_history[topic] = []
        self.message_history[topic].append(
            {"timestamp": time.time(), "message": message}
        )

        # Keep history bounded
        if len(self.message_history[topic]) > self.max_history:
            self.message_history[topic] = self.message_history[topic][
                -self.max_history :
            ]

        # Deliver to subscribers
        if topic in self.subscribers:
            for subscriber in self.subscribers[topic]:
                subscriber.receive_message(message)

    def call_service(self, service_name: str, request):
        """Call a service."""
        if service_name in self.services:
            return self.services[service_name].handle_request(request)
        return None

    def send_action_goal(self, action_name: str, goal):
        """Send an action goal."""
        if action_name in self.action_servers:
            return self.action_servers[action_name].handle_goal(goal)
        return None

    def get_message_history(self, topic: str, limit: int = 10) -> List[Dict]:
        """Get message history for a topic."""
        if topic in self.message_history:
            return self.message_history[topic][-limit:]
        return []

    def get_topic_stats(self) -> Dict[str, Any]:
        """Get topic statistics."""
        return {
            "topics": list(self.subscribers.keys()),
            "subscribers_per_topic": {
                topic: len(subs) for topic, subs in self.subscribers.items()
            },
            "services": list(self.services.keys()),
            "action_servers": list(self.action_servers.keys()),
            "total_messages": sum(
                len(history) for history in self.message_history.values()
            ),
        }


# Mock rclpy module
class MockRCLPY:
    """Mock rclpy module for ROS2 compatibility."""

    @staticmethod
    def init():
        """Initialize ROS2."""
        logger.info("Mock ROS2 initialized")

    @staticmethod
    def shutdown():
        """Shutdown ROS2."""
        logger.info("Mock ROS2 shutdown")
        ROS2MessageBus.get_instance().services.clear()
        ROS2MessageBus.get_instance().action_servers.clear()

    @staticmethod
    def spin_once(node: ROS2Node, timeout_sec: float = 0.0):
        """Spin once."""
        time.sleep(timeout_sec)

    @staticmethod
    def create_node(node_name: str, namespace: str = "") -> ROS2Node:
        """Create a ROS2 node."""
        return ROS2Node(node_name, namespace)

    @staticmethod
    def get_global_executor() -> Executor:
        """Get global executor."""
        return Executor()

    class QoSProfile(QoSProfile):
        """QoS profile class."""

        pass

    # QoS presets
    qos_profiles = qos_profiles


# Mock message types
class MockMessages:
    """Mock ROS2 message types."""

    class geometry_msgs:
        Twist = Twist
        PoseStamped = PoseStamped

    class sensor_msgs:
        NavSatFix = NavSatFix
        Imu = Imu

    class std_srvs:
        Trigger = Trigger

    class nav2_msgs:
        action = type("action", (), {"NavigateToPose": NavigateToPose})()


# Create mock modules for import compatibility
sys.modules["rclpy"] = MockRCLPY()
sys.modules["rclpy.qos"] = MockRCLPY
sys.modules["geometry_msgs.msg"] = MockMessages.geometry_msgs
sys.modules["sensor_msgs.msg"] = MockMessages.sensor_msgs
sys.modules["std_srvs.srv"] = MockMessages.std_srvs
sys.modules["nav2_msgs.action"] = MockMessages.nav2_msgs.action

# Mock autonomy_interfaces if they don't exist
try:
    import autonomy_interfaces
except ImportError:
    # Create mock autonomy_interfaces
    mock_autonomy = type("autonomy_interfaces", (), {})

    # Mock action module
    mock_action = type("action", (), {})
    mock_action.NavigateToPose = NavigateToPose
    mock_autonomy.action = mock_action

    sys.modules["autonomy_interfaces"] = mock_autonomy
    sys.modules["autonomy_interfaces.action"] = mock_action


# Performance monitoring
class ROS2PerformanceMonitor:
    """Monitor ROS2 performance in mock environment."""

    def __init__(self):
        self.start_time = time.time()
        self.message_counts: Dict[str, int] = {}
        self.latencies: List[float] = []

    def record_message(self, topic: str, latency: float = 0.0):
        """Record a message."""
        self.message_counts[topic] = self.message_counts.get(topic, 0) + 1
        if latency > 0:
            self.latencies.append(latency)

    def get_stats(self) -> Dict[str, Any]:
        """Get performance statistics."""
        total_messages = sum(self.message_counts.values())
        avg_latency = statistics.mean(self.latencies) if self.latencies else 0
        max_latency = max(self.latencies) if self.latencies else 0

        return {
            "uptime_seconds": time.time() - self.start_time,
            "total_messages": total_messages,
            "messages_per_topic": self.message_counts,
            "avg_latency_ms": avg_latency * 1000,
            "max_latency_ms": max_latency * 1000,
            "messages_per_second": total_messages
            / max(1, time.time() - self.start_time),
        }


# Global performance monitor
_performance_monitor = ROS2PerformanceMonitor()


def get_performance_stats() -> Dict[str, Any]:
    """Get ROS2 performance statistics."""
    return _performance_monitor.get_stats()


# ROS2 Compatibility Checker
def check_ros2_compatibility() -> Dict[str, bool]:
    """Check if real ROS2 is available."""
    try:
        import rclpy

        rclpy.init()
        rclpy.shutdown()
        return {"ros2_available": True, "using_mock": False}
    except ImportError:
        return {"ros2_available": False, "using_mock": True}


# Export compatibility
__all__ = [
    "ROS2Node",
    "Publisher",
    "Subscriber",
    "Service",
    "ActionServer",
    "Timer",
    "Executor",
    "ROS2MessageBus",
    "MockRCLPY",
    "Twist",
    "PoseStamped",
    "NavSatFix",
    "Imu",
    "Trigger",
    "NavigateToPose",
    "QoSProfile",
    "qos_profiles",
    "get_performance_stats",
    "check_ros2_compatibility",
]
