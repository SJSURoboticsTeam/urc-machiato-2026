#!/usr/bin/env python3
"""
Jazzy QoS Profiles for URC 2026 Mars Rover
Optimized communication patterns using Cyclone DDS and Iceoryx2

This module defines QoS profiles for different communication patterns:
- Safety-critical: Reflex layer (lowest latency, best effort)
- Hard real-time: Motion control (guaranteed delivery, bounded latency)
- Soft real-time: Autonomy (reliable, prioritized)
- Best effort: Telemetry (high throughput, eventual consistency)
"""

import logging
import rclpy
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from rclpy.qos import QoSHistoryPolicy

# Try to import Jazzy-specific QoS features
try:
    from rclpy.qos import QoSLivelinessPolicy, QoSDeadline, QoSLifespan

    JAZZY_QOS_AVAILABLE = True
except ImportError:
    JAZZY_QOS_AVAILABLE = False
    QoSLivelinessPolicy = None
    QoSDeadline = None
    QoSLifespan = None

logger = logging.getLogger(__name__)


class JazzyQoSProfiles:
    """
    QoS profiles optimized for Jazzy + Cyclone DDS + Iceoryx2

    Each profile is tuned for specific communication patterns in the
    layered architecture: Reflex → Motion → Autonomy → Planning
    """

    @staticmethod
    def safety_critical_sensor_data() -> QoSProfile:
        """
        Reflex Layer Sensor Data
        - Highest priority, lowest latency
        - 100Hz operation, <1ms latency target
        """
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Add Jazzy-specific features if available
        if JAZZY_QOS_AVAILABLE and QoSDeadline and QoSLifespan and QoSLivelinessPolicy:
            qos.deadline = QoSDeadline(milliseconds=10)  # 100Hz max latency
            qos.lifespan = QoSLifespan(milliseconds=100)  # Data expires quickly
            qos.liveliness = QoSLivelinessPolicy.AUTOMATIC
            qos.liveliness_lease_duration = rclpy.duration.Duration(
                seconds=0.1
            )  # 100ms

        return qos

    @staticmethod
    def hard_realtime_motion_commands() -> QoSProfile:
        """
        Motion Control Commands
        - Guaranteed delivery with bounded latency
        - 50Hz operation, <20ms latency target
        """
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=3,
        )

        # Add Jazzy-specific features if available
        if JAZZY_QOS_AVAILABLE and QoSDeadline and QoSLifespan and QoSLivelinessPolicy:
            qos.deadline = QoSDeadline(milliseconds=20)  # 50Hz max latency
            qos.lifespan = QoSLifespan(
                milliseconds=200
            )  # Data valid for motion planning
            qos.liveliness = QoSLivelinessPolicy.AUTOMATIC
            qos.liveliness_lease_duration = rclpy.duration.Duration(
                seconds=0.2
            )  # 200ms

        return qos

    @staticmethod
    def soft_realtime_autonomy_status() -> QoSProfile:
        """
        Autonomy Status Updates (BT + State Machine)
        - Reliable delivery with some latency tolerance
        - 10Hz operation, <100ms latency target
        """
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
        )

        # Add Jazzy-specific features if available
        if JAZZY_QOS_AVAILABLE and QoSDeadline and QoSLifespan and QoSLivelinessPolicy:
            qos.deadline = QoSDeadline(milliseconds=100)  # 10Hz max latency
            qos.lifespan = QoSLifespan(seconds=1)  # Data valid for decision making
            qos.liveliness = QoSLivelinessPolicy.AUTOMATIC
            qos.liveliness_lease_duration = rclpy.duration.Duration(seconds=1)  # 1s

        return qos

    @staticmethod
    def best_effort_telemetry() -> QoSProfile:
        """
        Telemetry and Monitoring Data
        - High throughput, eventual consistency
        - Best effort, may lose data under load
        """
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=50,
        )

        # Add Jazzy-specific features if available
        if JAZZY_QOS_AVAILABLE and QoSDeadline and QoSLifespan and QoSLivelinessPolicy:
            qos.deadline = QoSDeadline(seconds=1)  # 1Hz monitoring is fine
            qos.lifespan = QoSLifespan(seconds=10)  # Telemetry data ages out
            qos.liveliness = QoSLivelinessPolicy.AUTOMATIC
            qos.liveliness_lease_duration = rclpy.duration.Duration(seconds=5)  # 5s

        return qos

    @staticmethod
    def intra_process_high_frequency() -> QoSProfile:
        """
        Intra-process communication (Iceoryx2)
        - Zero-copy shared memory
        - Extremely high frequency, low latency
        """
        qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )

        # Add Jazzy-specific features if available
        if JAZZY_QOS_AVAILABLE and QoSDeadline and QoSLifespan and QoSLivelinessPolicy:
            qos.deadline = QoSDeadline(milliseconds=1)  # 1000Hz potential
            qos.lifespan = QoSLifespan(milliseconds=10)  # Very fresh data only
            qos.liveliness = QoSLivelinessPolicy.AUTOMATIC
            qos.liveliness_lease_duration = rclpy.duration.Duration(
                milliseconds=100
            )  # 100ms

        return qos


# Convenience functions for common patterns
def get_reflex_sensor_qos() -> QoSProfile:
    """QoS for reflex layer sensor data"""
    return JazzyQoSProfiles.safety_critical_sensor_data()


def get_motion_command_qos() -> QoSProfile:
    """QoS for motion control commands"""
    return JazzyQoSProfiles.hard_realtime_motion_commands()


def get_autonomy_status_qos() -> QoSProfile:
    """QoS for autonomy status updates"""
    return JazzyQoSProfiles.soft_realtime_autonomy_status()


def get_telemetry_qos() -> QoSProfile:
    """QoS for telemetry data"""
    return JazzyQoSProfiles.best_effort_telemetry()


def get_intra_process_qos() -> QoSProfile:
    """QoS for intra-process communication"""
    return JazzyQoSProfiles.intra_process_high_frequency()


# QoS Event Callbacks for monitoring
def qos_event_callback(event_type: str, event_data: dict):
    """
    Callback for QoS events (liveliness changes, deadline misses, etc.)
    """
    if event_type == "liveliness_changed":
        logger.warning(
            f"Liveliness changed: alive_count={event_data.get('alive_count', 0)}"
        )
    elif event_type == "deadline_missed":
        logger.error("QoS deadline missed!")
    elif event_type == "liveliness_lost":
        logger.critical("QoS liveliness lost!")
    else:
        logger.debug(f"QoS event: {event_type}")


# Utility function to create publishers with QoS profiles
def create_jazzy_publisher(node, msg_type, topic: str, qos_profile: QoSProfile):
    """
    Create a publisher with Jazzy-optimized QoS
    """
    publisher = node.create_publisher(msg_type, topic, qos_profile)

    # Jazzy: Enable QoS event monitoring if available
    if hasattr(publisher, "add_on_qos_event_callback"):
        publisher.add_on_qos_event_callback(qos_event_callback)

    return publisher


# Utility function to create subscribers with QoS profiles
def create_jazzy_subscription(
    node, msg_type, topic: str, callback, qos_profile: QoSProfile
):
    """
    Create a subscription with Jazzy-optimized QoS
    """
    subscription = node.create_subscription(msg_type, topic, callback, qos_profile)

    # Jazzy: Enable QoS event monitoring if available
    if hasattr(subscription, "add_on_qos_event_callback"):
        subscription.add_on_qos_event_callback(qos_event_callback)

    return subscription
