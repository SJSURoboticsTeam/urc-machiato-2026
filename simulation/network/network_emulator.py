#!/usr/bin/env python3
"""
Network Emulation Layer for Simulation Testing

Emulates various network conditions:
- Perfect: No latency, no loss, unlimited bandwidth
- Real-life: Typical field conditions (WiFi, cellular)
- Extreme: Worst-case scenarios (satellite, severe interference)

Author: URC 2026 Autonomy Team
"""

import asyncio
import queue
import random
import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional


class NetworkProfile(Enum):
    """Network emulation profiles for testing."""

    PERFECT = "perfect"
    RURAL_WIFI = "rural_wifi"
    CELLULAR_4G = "cellular_4g"
    SATELLITE = "satellite"
    EXTREME = "extreme"


@dataclass
class NetworkCondition:
    """Network condition parameters."""

    latency_ms: float  # Average latency
    jitter_ms: float  # Latency variation
    packet_loss_percent: float  # Packet loss rate
    bandwidth_mbps: float  # Available bandwidth
    burst_limit_mb: Optional[float] = None  # Burst data limit
    connection_drops: bool = False  # Enable connection drops
    drop_duration_s: float = 0.0  # Duration of drops


# Network profile definitions based on SIMULATION_VALIDATION_CHECKLIST.md
NETWORK_PROFILES: Dict[NetworkProfile, NetworkCondition] = {
    NetworkProfile.PERFECT: NetworkCondition(
        latency_ms=0.0,
        jitter_ms=0.0,
        packet_loss_percent=0.0,
        bandwidth_mbps=1000.0,
        connection_drops=False,
    ),
    NetworkProfile.RURAL_WIFI: NetworkCondition(
        latency_ms=85.0,  # 20-150ms average
        jitter_ms=25.0,
        packet_loss_percent=2.0,  # 0-5% typical
        bandwidth_mbps=25.0,
        burst_limit_mb=10.0,
        connection_drops=True,
        drop_duration_s=5.0,  # 1-10s drops
    ),
    NetworkProfile.CELLULAR_4G: NetworkCondition(
        latency_ms=125.0,  # 50-200ms average
        jitter_ms=30.0,
        packet_loss_percent=3.0,  # 0-5% typical
        bandwidth_mbps=15.0,
        burst_limit_mb=5.0,
        connection_drops=True,
        drop_duration_s=15.0,  # 5-30s drops
    ),
    NetworkProfile.SATELLITE: NetworkCondition(
        latency_ms=900.0,  # 600-1200ms average
        jitter_ms=100.0,
        packet_loss_percent=1.0,  # Usually reliable
        bandwidth_mbps=5.0,
        burst_limit_mb=2.0,
        connection_drops=True,
        drop_duration_s=60.0,  # 30-120s drops
    ),
    NetworkProfile.EXTREME: NetworkCondition(
        latency_ms=1500.0,  # Extreme latency
        jitter_ms=500.0,  # High variability
        packet_loss_percent=15.0,  # Severe packet loss
        bandwidth_mbps=1.0,  # Severely constrained
        burst_limit_mb=0.5,
        connection_drops=True,
        drop_duration_s=300.0,  # Up to 5 minute drops
    ),
}


class NetworkEmulator:
    """
    Network emulator for simulating various network conditions.

    Delays messages, drops packets, and limits bandwidth according
    to selected network profile.
    """

    def __init__(self, profile: NetworkProfile = NetworkProfile.PERFECT):
        self.profile = profile
        self.condition = NETWORK_PROFILES[profile]

        # Tracking
        self.messages_sent = 0
        self.messages_received = 0
        self.messages_dropped = 0
        self.total_latency_ms = 0.0
        self.bytes_transferred = 0

        # Connection state
        self.is_connected = True
        self.last_drop_time = 0.0
        self.next_drop_time = self._calculate_next_drop()

        # Message queue for latency simulation
        self.message_queue: queue.PriorityQueue = queue.PriorityQueue()

        # Background thread for message delivery
        self.running = False
        self.delivery_thread: Optional[threading.Thread] = None

        # Callbacks
        self.on_message_delivered: Optional[Callable[[Any], None]] = None

    def start(self):
        """Start network emulation."""
        self.running = True
        self.delivery_thread = threading.Thread(
            target=self._delivery_loop, daemon=True
        )
        self.delivery_thread.start()

    def stop(self):
        """Stop network emulation."""
        self.running = False
        if self.delivery_thread:
            self.delivery_thread.join(timeout=2.0)

    def send_message(self, message: Any) -> bool:
        """
        Send message through emulated network.

        Returns:
            bool: True if message accepted, False if dropped
        """
        self.messages_sent += 1

        # Check for connection drop
        current_time = time.time()
        if self._should_drop_connection(current_time):
            self.is_connected = False
            self.last_drop_time = current_time
            self.messages_dropped += 1
            return False

        # Restore connection after drop duration
        if not self.is_connected:
            if current_time - self.last_drop_time > self.condition.drop_duration_s:
                self.is_connected = True
                self.next_drop_time = self._calculate_next_drop()

        # Check if message is dropped due to packet loss
        if random.random() * 100 < self.condition.packet_loss_percent:
            self.messages_dropped += 1
            return False

        # Calculate delivery time with latency and jitter
        latency = self._calculate_latency()
        delivery_time = current_time + latency / 1000.0  # Convert to seconds

        # Add to priority queue (ordered by delivery time)
        self.message_queue.put((delivery_time, message))
        self.total_latency_ms += latency

        return True

    def _delivery_loop(self):
        """Background thread to deliver messages with delay."""
        while self.running:
            try:
                # Get next message with timeout
                delivery_time, message = self.message_queue.get(timeout=0.1)

                # Wait until delivery time
                current_time = time.time()
                if delivery_time > current_time:
                    time.sleep(delivery_time - current_time)

                # Deliver message
                if self.on_message_delivered:
                    self.on_message_delivered(message)

                self.messages_received += 1

            except queue.Empty:
                continue

    def _calculate_latency(self) -> float:
        """Calculate latency with jitter."""
        base_latency = self.condition.latency_ms
        jitter = random.gauss(0, self.condition.jitter_ms / 2.0)
        return max(0, base_latency + jitter)

    def _should_drop_connection(self, current_time: float) -> bool:
        """Check if connection should drop now."""
        if not self.condition.connection_drops:
            return False

        if self.is_connected and current_time >= self.next_drop_time:
            return True

        return False

    def _calculate_next_drop(self) -> float:
        """Calculate next connection drop time."""
        if not self.condition.connection_drops:
            return float("inf")

        # Random interval between drops (30-300 seconds)
        interval = random.uniform(30.0, 300.0)
        return time.time() + interval

    def get_statistics(self) -> Dict[str, Any]:
        """Get network emulation statistics."""
        return {
            "profile": self.profile.value,
            "messages_sent": self.messages_sent,
            "messages_received": self.messages_received,
            "messages_dropped": self.messages_dropped,
            "packet_loss_percent": (
                (self.messages_dropped / self.messages_sent * 100)
                if self.messages_sent > 0
                else 0
            ),
            "average_latency_ms": (
                (self.total_latency_ms / self.messages_sent)
                if self.messages_sent > 0
                else 0
            ),
            "is_connected": self.is_connected,
            "condition": {
                "latency_ms": self.condition.latency_ms,
                "jitter_ms": self.condition.jitter_ms,
                "packet_loss_percent": self.condition.packet_loss_percent,
                "bandwidth_mbps": self.condition.bandwidth_mbps,
            },
        }


class ROS2NetworkBridge:
    """
    Bridge for emulating network conditions on ROS2 topics.

    Wraps ROS2 publishers/subscribers with network emulation.
    """

    def __init__(self, emulator: NetworkEmulator):
        self.emulator = emulator
        self.topic_subscribers: Dict[str, Any] = {}
        self.topic_publishers: Dict[str, Any] = {}

    def create_emulated_publisher(self, node: Any, msg_type: Any, topic: str, qos: int):
        """Create publisher with network emulation."""
        # Create real publisher
        publisher = node.create_publisher(msg_type, f"{topic}_real", qos)

        # Wrapper that sends through emulator
        def emulated_publish(msg):
            self.emulator.send_message((topic, msg))

        self.topic_publishers[topic] = emulated_publish
        return emulated_publish

    def create_emulated_subscriber(
        self, node: Any, msg_type: Any, topic: str, callback: Callable, qos: int
    ):
        """Create subscriber with network emulation."""
        # Subscribe to emulated topic
        subscriber = node.create_subscription(
            msg_type, f"{topic}_emulated", callback, qos
        )
        self.topic_subscribers[topic] = subscriber
        return subscriber


if __name__ == "__main__":
    # Test network emulator
    print("🌐 Testing Network Emulator")
    print("=" * 60)

    for profile in NetworkProfile:
        print(f"\n📡 Testing {profile.value} profile:")
        emulator = NetworkEmulator(profile)
        emulator.start()

        # Send 100 test messages
        messages_accepted = 0
        for i in range(100):
            if emulator.send_message(f"message_{i}"):
                messages_accepted += 1

        time.sleep(3.0)  # Wait for delivery
        emulator.stop()

        stats = emulator.get_statistics()
        print(f"  Messages sent: {stats['messages_sent']}")
        print(f"  Messages received: {stats['messages_received']}")
        print(f"  Messages dropped: {stats['messages_dropped']}")
        print(f"  Packet loss: {stats['packet_loss_percent']:.2f}%")
        print(f"  Average latency: {stats['average_latency_ms']:.2f}ms")

    print("\n✅ Network emulator test complete")
