#!/usr/bin/env python3
"""
QoS Profiling System for URC Rover

Provides comprehensive Quality of Service monitoring and profiling for ROS2 topics,
WebSocket telemetry, and network performance. Measures latency, jitter, bandwidth
utilization, and packet loss rates to establish baselines and detect degradation.

Handles URC band restrictions:
- 900 MHz band: 8 MHz max bandwidth, sub-band switching (902-910, 911-919, 920-928 MHz)
- 2.4 GHz band: No restrictions but interference-tolerant protocols required
- Encourages frequency hopping and automatic channel selection
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from typing import Dict, List, Any, Optional, Tuple
import time
import threading
import statistics
from collections import deque
import psutil
import numpy as np

from autonomy_interfaces.msg import SystemState, ContextState
from autonomy_interfaces.srv import GetQoSProfile, GetNetworkStats


class AdaptiveQoSManager:
    """Adaptive QoS management based on network conditions."""

    def __init__(self, network_monitor):
        self.network_monitor = network_monitor
        self.topic_profiles = self._load_topic_profiles()
        self.adaptive_profiles = {}

    def _load_topic_profiles(self) -> Dict[str, QoSProfile]:
        """Load optimized QoS profiles for different topic types."""
        return {
            # Critical control topics - maximum reliability
            '/cmd_vel': QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10,
                deadline=rclpy.duration.Duration(seconds=0.1),  # 100ms deadline
                liveliness=LivelinessPolicy.AUTOMATIC,
                liveliness_lease_duration=rclpy.duration.Duration(seconds=1)
            ),

            # Sensor data - best effort with some history
            '/imu/data': QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=5,
                deadline=rclpy.duration.Duration(seconds=0.05)  # 50ms deadline
            ),

            # Telemetry - reliable with compression hints
            '/telemetry/*': QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
                history=HistoryPolicy.KEEP_LAST,
                depth=50
            )
        }

    def apply_system_qos_profiles(self, node):
        """Apply optimized QoS profiles to all publishers/subscribers."""
        for topic_name, profile in self.topic_profiles.items():
            # Update existing publishers
            for pub in node.publishers:
                if pub.topic_name == topic_name or topic_name.endswith('/*'):
                    pub.qos_profile = profile

            # Update existing subscribers
            for sub in node.subscriptions:
                if sub.topic_name == topic_name or topic_name.endswith('/*'):
                    sub.qos_profile = profile

    def adapt_topic_qos(self, topic_name: str, current_profile: QoSProfile) -> QoSProfile:
        """Adapt QoS profile based on current network conditions."""
        network_quality = self.network_monitor.assess_network_quality()
        rules = self._get_adaptation_rules(network_quality)

        # Create adapted profile
        adapted = QoSProfile()
        adapted.reliability = rules['reliability']
        adapted.history = HistoryPolicy.KEEP_LAST
        adapted.depth = rules['depth']
        adapted.durability = current_profile.durability  # Keep existing
        adapted.deadline = rclpy.duration.Duration(milliseconds=rules['deadline_ms'])

        return adapted

    def _get_adaptation_rules(self, network_quality: str) -> Dict:
        """Load rules for adapting QoS based on network conditions."""
        adaptation_rules = {
            'excellent': {
                'reliability': ReliabilityPolicy.RELIABLE,
                'depth': 50,
                'deadline_ms': 100
            },
            'good': {
                'reliability': ReliabilityPolicy.RELIABLE,
                'depth': 20,
                'deadline_ms': 200
            },
            'fair': {
                'reliability': ReliabilityPolicy.BEST_EFFORT,
                'depth': 10,
                'deadline_ms': 500
            },
            'poor': {
                'reliability': ReliabilityPolicy.BEST_EFFORT,
                'depth': 5,
                'deadline_ms': 1000
            }
        }

        return adaptation_rules.get(network_quality, adaptation_rules['fair'])


class QoSProfiler(Node):
    """
    Comprehensive QoS profiling for the URC rover system.

    Measures and tracks Quality of Service metrics across:
    - ROS2 topic latency and jitter
    - WebSocket telemetry performance
    - Network bandwidth utilization
    - Packet loss and reliability
    - URC band compliance and switching
    """

    def __init__(self):
        super().__init__('qos_profiler')

        # Configuration for URC band restrictions
        self.urc_band_config = {
            '900mhz': {
                'max_bandwidth_mhz': 8.0,
                'sub_bands': {
                    'low': {'range': (902, 910), 'current': False},
                    'mid': {'range': (911, 919), 'current': False},
                    'high': {'range': (920, 928), 'current': False}
                },
                'current_subband': None
            },
            '2_4ghz': {
                'max_bandwidth_mhz': None,  # No restriction
                'interference_tolerant': True,
                'frequency_hopping': False
            }
        }

        # QoS measurement data structures
        self.topic_metrics = {}  # topic_name -> metrics
        self.websocket_metrics = {
            'latency_samples': deque(maxlen=1000),
            'jitter_samples': deque(maxlen=1000),
            'packet_loss_rate': 0.0,
            'bandwidth_usage': 0.0,
            'connection_drops': 0
        }

        self.network_metrics = {
            'bandwidth_up': deque(maxlen=100),
            'bandwidth_down': deque(maxlen=100),
            'latency': deque(maxlen=100),
            'packet_loss': deque(maxlen=100),
            'current_band': 'unknown',
            'signal_strength': 0.0
        }

        # Safety monitoring data
        self.safety_properties = {
            'control_loop_timing': {'violations': 0, 'threshold_ms': 20.0},
            'telemetry_latency': {'violations': 0, 'threshold_ms': 200.0},
            'battery_safety': {'violations': 0, 'threshold_percent': 10.0},
            'geofence_compliance': {'violations': 0, 'threshold': 0}  # 0 = no violation
        }

        # Adaptive telemetry parameters
        self.adaptive_params = {
            'current_telemetry_rate': 5.0,  # Hz
            'min_telemetry_rate': 1.0,
            'max_telemetry_rate': 10.0,
            'bandwidth_target': 0.5,  # Target bandwidth utilization (50%)
            'latency_target': 100.0,  # Target latency in ms
            'adaptation_rate': 0.1,   # How aggressively to adapt
            'last_adaptation': time.time()
        }

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('profiling_rate', 1.0),              # Hz - how often to update metrics
                ('history_window', 60.0),             # seconds - rolling window for averages
                ('bandwidth_measurement_interval', 1.0),  # seconds
                ('latency_test_rate', 0.1),           # Hz - how often to measure latency
                ('alert_thresholds', {
                    'latency_critical': 500.0,       # ms
                    'packet_loss_critical': 0.1,     # 10%
                    'bandwidth_critical': 0.9        # 90% utilization
                })
            ]
        )

        # Get parameters
        self.profiling_rate = self.get_parameter('profiling_rate').value
        self.history_window = self.get_parameter('history_window').value
        self.bandwidth_interval = self.get_parameter('bandwidth_measurement_interval').value
        self.latency_test_rate = self.get_parameter('latency_test_rate').value
        self.alert_thresholds = self.get_parameter('alert_thresholds').value

        # Setup ROS2 interfaces
        self._setup_subscribers()
        self._setup_services()
        self._setup_timers()

        # Bandwidth monitoring
        self.last_bandwidth_check = time.time()
        self.last_bytes_sent = psutil.net_io_counters().bytes_sent
        self.last_bytes_recv = psutil.net_io_counters().bytes_recv

        self.get_logger().info("QoS Profiler initialized with URC band awareness")

    def _setup_subscribers(self):
        """Setup ROS2 subscribers for QoS monitoring."""
        callback_group = MutuallyExclusiveCallbackGroup()

        # Monitor critical topics for QoS
        critical_topics = [
            ('/state_machine/current_state', SystemState),
            ('/state_machine/context', ContextState),
            ('/hardware/battery_state', 'sensor_msgs/BatteryState'),
            ('/hardware/gps', 'sensor_msgs/NavSatFix'),
            ('/hardware/imu', 'sensor_msgs/Imu')
        ]

        for topic_name, msg_type in critical_topics:
            self.topic_metrics[topic_name] = {
                'samples': deque(maxlen=1000),
                'latency': deque(maxlen=100),
                'jitter': deque(maxlen=100),
                'packet_loss': 0,
                'last_seq_num': None,
                'expected_count': 0,
                'received_count': 0
            }

            # Create subscriber for QoS monitoring
            try:
                self.create_subscription(
                    msg_type, topic_name,
                    lambda msg, topic=topic_name: self._topic_callback(msg, topic),
                    QoSProfile(
                        reliability=ReliabilityPolicy.BEST_EFFORT,
                        history=HistoryPolicy.KEEP_LAST,
                        depth=1
                    ),
                    callback_group=callback_group
                )
            except Exception as e:
                self.get_logger().warn(f"Could not subscribe to {topic_name}: {e}")

    def _setup_services(self):
        """Setup ROS2 services for QoS data access."""
        self.create_service(
            GetQoSProfile,
            '/qos_profiler/get_profile',
            self._handle_get_profile
        )

        self.create_service(
            GetNetworkStats,
            '/qos_profiler/get_network_stats',
            self._handle_get_network_stats
        )

    def _setup_timers(self):
        """Setup periodic timers for QoS measurements."""
        # Main profiling timer
        self.create_timer(
            1.0 / self.profiling_rate,
            self._profiling_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Bandwidth measurement timer
        self.create_timer(
            self.bandwidth_interval,
            self._bandwidth_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Latency test timer
        self.create_timer(
            1.0 / self.latency_test_rate,
            self._latency_test_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

        # Safety monitoring timer
        self.create_timer(
            0.1,  # 10Hz safety monitoring
            self._safety_monitoring_callback,
            callback_group=MutuallyExclusiveCallbackGroup()
        )

    def _topic_callback(self, msg, topic_name: str):
        """Handle incoming ROS2 messages for QoS profiling."""
        now = time.time()

        metrics = self.topic_metrics[topic_name]
        metrics['samples'].append(now)
        metrics['received_count'] += 1

        # Calculate latency if message has timestamp
        if hasattr(msg, 'header') and hasattr(msg.header, 'stamp'):
            msg_time = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            latency = (now - msg_time) * 1000  # Convert to milliseconds
            metrics['latency'].append(latency)

            # Calculate jitter
            if len(metrics['latency']) > 1:
                recent_latencies = list(metrics['latency'])[-10:]  # Last 10 samples
                if len(recent_latencies) > 1:
                    jitter = statistics.stdev(recent_latencies)
                    metrics['jitter'].append(jitter)

        # Check for sequence number gaps (packet loss detection)
        if hasattr(msg, 'header') and hasattr(msg.header, 'seq'):
            if metrics['last_seq_num'] is not None:
                expected_seq = metrics['last_seq_num'] + 1
                if msg.header.seq > expected_seq:
                    gap = msg.header.seq - expected_seq
                    metrics['packet_loss'] += gap
            metrics['last_seq_num'] = msg.header.seq

    def _profiling_callback(self):
        """Periodic QoS profiling and analysis."""
        # Analyze topic performance
        for topic_name, metrics in self.topic_metrics.items():
            if len(metrics['latency']) > 0:
                avg_latency = statistics.mean(metrics['latency'])
                if len(metrics['latency']) > 1:
                    jitter = statistics.stdev(metrics['latency'])
                else:
                    jitter = 0.0

                packet_loss_rate = metrics['packet_loss'] / max(1, metrics['received_count'])

                # Log warnings for degraded performance
                if avg_latency > self.alert_thresholds['latency_critical']:
                    self.get_logger().warn(
                        f"High latency on {topic_name}: {avg_latency:.1f}ms"
                    )
                if packet_loss_rate > self.alert_thresholds['packet_loss_critical']:
                    self.get_logger().warn(
                        f"High packet loss on {topic_name}: {packet_loss_rate:.1%}"
                    )

        # Adaptive telemetry adjustment
        self._adapt_telemetry_rate()

        # Check URC band compliance
        self._check_band_compliance()

    def _bandwidth_callback(self):
        """Measure network bandwidth utilization."""
        now = time.time()
        dt = now - self.last_bandwidth_check

        if dt > 0:
            current_bytes_sent = psutil.net_io_counters().bytes_sent
            current_bytes_recv = psutil.net_io_counters().bytes_recv

            bytes_sent_delta = current_bytes_sent - self.last_bytes_sent
            bytes_recv_delta = current_bytes_recv - self.last_bytes_recv

            # Convert to Mbps
            bandwidth_up = (bytes_sent_delta * 8) / (dt * 1_000_000)
            bandwidth_down = (bytes_recv_delta * 8) / (dt * 1_000_000)

            self.network_metrics['bandwidth_up'].append(bandwidth_up)
            self.network_metrics['bandwidth_down'].append(bandwidth_down)

            self.last_bandwidth_check = now
            self.last_bytes_sent = current_bytes_sent
            self.last_bytes_recv = current_bytes_recv

            # Check bandwidth utilization against URC limits
            total_bandwidth = bandwidth_up + bandwidth_down
            max_allowed = self.urc_band_config['900mhz']['max_bandwidth_mhz']

            if max_allowed and total_bandwidth > max_allowed:
                self.get_logger().warn(
                    f"Bandwidth usage ({total_bandwidth:.2f} Mbps) exceeds URC 900MHz limit ({max_allowed} MHz)"
                )

    def _latency_test_callback(self):
        """Perform active latency measurements."""
        # Send ping-like messages and measure round-trip time
        # This would require coordination with the competition bridge
        pass

    def _safety_monitoring_callback(self):
        """Monitor critical safety properties in real-time."""
        # Check control loop timing (should be < 20ms)
        control_topic = '/state_machine/current_state'
        if control_topic in self.topic_metrics:
            latencies = list(self.topic_metrics[control_topic]['latency'])
            if latencies:
                recent_avg = statistics.mean(latencies[-10:])  # Last 10 samples
                if recent_avg > self.safety_properties['control_loop_timing']['threshold_ms']:
                    self.safety_properties['control_loop_timing']['violations'] += 1
                    self.get_logger().error(
                        f"Control loop timing violation: {recent_avg:.1f}ms > {self.safety_properties['control_loop_timing']['threshold_ms']}ms"
                    )

        # Add more safety property checks here as needed

    def _adapt_telemetry_rate(self):
        """Adaptively adjust telemetry rate based on network conditions."""
        now = time.time()

        # Only adapt if enough time has passed since last adaptation
        if now - self.adaptive_params['last_adaptation'] < 5.0:  # Minimum 5 seconds between adaptations
            return

        # Calculate current network conditions
        bandwidth_up = list(self.network_metrics['bandwidth_up'])
        bandwidth_down = list(self.network_metrics['bandwidth_down'])

        if not bandwidth_up or not bandwidth_down:
            return

        avg_bandwidth = (statistics.mean(bandwidth_up) + statistics.mean(bandwidth_down)) / 2.0
        max_allowed = self.urc_band_config['900mhz']['max_bandwidth_mhz'] or 100.0  # Fallback for 2.4GHz
        bandwidth_utilization = avg_bandwidth / max_allowed

        # Calculate latency
        latencies = []
        for topic_metrics in self.topic_metrics.values():
            latencies.extend(list(topic_metrics['latency']))
        avg_latency = statistics.mean(latencies) if latencies else 0.0

        # Determine adaptation direction
        target_rate = self.adaptive_params['current_telemetry_rate']

        # If bandwidth utilization is too high, reduce rate
        if bandwidth_utilization > 0.8:  # Over 80% utilization
            target_rate *= 0.9  # Reduce by 10%
        # If latency is too high, reduce rate
        elif avg_latency > self.adaptive_params['latency_target']:
            target_rate *= 0.95  # Reduce by 5%
        # If conditions are good, can increase rate
        elif bandwidth_utilization < 0.5 and avg_latency < self.adaptive_params['latency_target'] * 0.5:
            target_rate *= 1.05  # Increase by 5%

        # Apply bounds
        target_rate = max(self.adaptive_params['min_telemetry_rate'],
                         min(self.adaptive_params['max_telemetry_rate'], target_rate))

        # Smooth the adaptation
        adaptation_factor = self.adaptive_params['adaptation_rate']
        new_rate = (self.adaptive_params['current_telemetry_rate'] * (1 - adaptation_factor) +
                   target_rate * adaptation_factor)

        if abs(new_rate - self.adaptive_params['current_telemetry_rate']) > 0.1:  # Only change if significant
            self.adaptive_params['current_telemetry_rate'] = new_rate
            self.adaptive_params['last_adaptation'] = now

            # Publish adaptation command to competition bridge
            self._publish_telemetry_rate_adaptation(new_rate)

            self.get_logger().info(
                f"Adapted telemetry rate to {new_rate:.1f} Hz "
                f"(bandwidth: {bandwidth_utilization:.1%}, latency: {avg_latency:.1f}ms)"
            )

    def _check_band_compliance(self):
        """Check compliance with URC band restrictions."""
        # This would require integration with the radio hardware to detect current band
        # For now, log warnings about potential violations
        total_bandwidth = (sum(self.network_metrics['bandwidth_up']) / len(self.network_metrics['bandwidth_up']) +
                          sum(self.network_metrics['bandwidth_down']) / len(self.network_metrics['bandwidth_down']))

        max_900mhz = self.urc_band_config['900mhz']['max_bandwidth_mhz']
        if max_900mhz and total_bandwidth > max_900mhz:
            self.get_logger().warn(
                f"Potential URC 900MHz band violation: {total_bandwidth:.2f} > {max_900mhz} MHz"
            )

    def _publish_telemetry_rate_adaptation(self, new_rate: float):
        """Publish telemetry rate adaptation to the competition bridge."""
        # This would send a message to adjust the competition bridge's telemetry rate
        # Implementation depends on the specific message interface
        pass

    def _handle_get_profile(self, request, response):
        """Handle QoS profile service requests."""
        response.topic_profiles = []

        for topic_name, metrics in self.topic_metrics.items():
            profile = {
                'topic_name': topic_name,
                'avg_latency_ms': statistics.mean(metrics['latency']) if metrics['latency'] else 0.0,
                'jitter_ms': statistics.stdev(metrics['latency']) if len(metrics['latency']) > 1 else 0.0,
                'packet_loss_rate': metrics['packet_loss'] / max(1, metrics['received_count']),
                'samples_count': len(metrics['samples'])
            }
            response.topic_profiles.append(profile)

        response.network_stats.bandwidth_up_mbps = (
            statistics.mean(self.network_metrics['bandwidth_up'])
            if self.network_metrics['bandwidth_up'] else 0.0
        )
        response.network_stats.bandwidth_down_mbps = (
            statistics.mean(self.network_metrics['bandwidth_down'])
            if self.network_metrics['bandwidth_down'] else 0.0
        )
        response.network_stats.current_band = self.network_metrics['current_band']
        response.network_stats.signal_strength = self.network_metrics['signal_strength']

        return response

    def _handle_get_network_stats(self, request, response):
        """Handle network statistics service requests."""
        response.bandwidth_up_mbps = (
            statistics.mean(self.network_metrics['bandwidth_up'])
            if self.network_metrics['bandwidth_up'] else 0.0
        )
        response.bandwidth_down_mbps = (
            statistics.mean(self.network_metrics['bandwidth_down'])
            if self.network_metrics['bandwidth_down'] else 0.0
        )
        response.latency_ms = (
            statistics.mean(self.network_metrics['latency'])
            if self.network_metrics['latency'] else 0.0
        )
        response.packet_loss_rate = (
            statistics.mean(self.network_metrics['packet_loss'])
            if self.network_metrics['packet_loss'] else 0.0
        )

        return response


def main(args=None):
    rclpy.init(args=args)

    profiler = QoSProfiler()

    # Use multi-threaded executor for concurrent QoS monitoring
    executor = MultiThreadedExecutor()
    executor.add_node(profiler)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        profiler.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()


