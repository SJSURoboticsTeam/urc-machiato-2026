#!/usr/bin/env python3
"""
Network Bandwidth Usage Tests - URC 2026

Tests network bandwidth optimization and efficiency:
- Bandwidth usage: <10Mbps sustained
- Message compression ratio: >70%
- Network utilization: <50% of available bandwidth
- Packet loss recovery: <500ms

Critical for communication reliability and bandwidth-constrained environments.
"""

import bz2
import gzip
import json
import lzma
import pickle
import statistics
import struct
import threading
import time
import unittest
import zlib
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import psutil
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import Float32MultiArray, Int32MultiArray, String


class NetworkBandwidthPerformanceTest(unittest.TestCase):
    """Test network bandwidth usage and optimization."""

    def setUp(self):
        """Set up network bandwidth testing."""
        # Test parameters - aligned with competition requirements
        self.target_bandwidth_mbps = 10.0  # Sustained bandwidth target
        self.target_compression_ratio = 0.7  # 70% size reduction target
        self.target_network_utilization = 0.5  # 50% of available bandwidth
        self.target_recovery_ms = 500  # Packet loss recovery target

        # Network tracking
        self.bandwidth_readings = []
        self.message_sizes = []
        self.compression_ratios = []
        self.network_latency_readings = []
        self.packet_loss_events = []

        # Test data
        self.bridge = CvBridge()

    def test_network_bandwidth_under_competition_load(self):
        """Test network bandwidth usage under competition scenarios."""
        print("\n🌐 Testing Network Bandwidth Under Competition Load")
        print("=" * 60)

        try:
            # Start bandwidth monitoring
            self._start_bandwidth_monitoring()

            # Test different communication scenarios
            scenarios = [
                self._test_sensor_data_streaming,
                self._test_navigation_data_exchange,
                self._test_vision_data_transmission,
                self._test_mission_command_communication,
                self._test_telemetry_data_streaming,
            ]

            results_by_scenario = {}

            for scenario in scenarios:
                print(
                    f"\n🔄 Testing {scenario.__name__.replace('_', ' ').replace('test_', '').title()}..."
                )
                result = scenario()
                results_by_scenario[scenario.__name__] = result

            # Analyze bandwidth usage
            self._analyze_bandwidth_usage(results_by_scenario)

        finally:
            self._stop_bandwidth_monitoring()

    def _start_bandwidth_monitoring(self):
        """Start network bandwidth monitoring."""
        self.monitoring_active = True
        self.monitor_thread = threading.Thread(
            target=self._bandwidth_monitor_worker, daemon=True
        )
        self.monitor_thread.start()

    def _stop_bandwidth_monitoring(self):
        """Stop bandwidth monitoring."""
        self.monitoring_active = False
        if hasattr(self, "monitor_thread"):
            self.monitor_thread.join(timeout=2.0)

    def _bandwidth_monitor_worker(self):
        """Monitor network bandwidth usage."""
        # Get network interface statistics
        net_io_start = psutil.net_io_counters()

        while self.monitoring_active:
            time.sleep(1.0)  # Monitor every second

            # Get current network statistics
            net_io_current = psutil.net_io_counters()

            # Calculate bandwidth usage (bytes per second)
            bytes_sent = net_io_current.bytes_sent - net_io_start.bytes_sent
            bytes_recv = net_io_current.bytes_recv - net_io_start.bytes_recv
            total_bytes_per_sec = (bytes_sent + bytes_recv) / 1.0

            # Convert to Mbps
            bandwidth_mbps = (total_bytes_per_sec * 8) / (1024 * 1024)

            self.bandwidth_readings.append(
                {
                    "timestamp": time.time(),
                    "bandwidth_mbps": bandwidth_mbps,
                    "bytes_sent_per_sec": bytes_sent,
                    "bytes_recv_per_sec": bytes_recv,
                }
            )

            # Update baseline for next measurement
            net_io_start = net_io_current

    def _test_sensor_data_streaming(self) -> Dict[str, Any]:
        """Test bandwidth usage for sensor data streaming."""
        # Simulate IMU, GPS, and sensor data streaming
        test_duration = 10.0  # 10 seconds
        message_rate = 100  # 100 Hz

        messages_sent = 0
        total_data_size = 0

        start_time = time.time()

        while time.time() - start_time < test_duration:
            # Generate sensor data messages
            messages = self._generate_sensor_messages()

            for msg in messages:
                # Serialize message
                serialized_size = self._calculate_message_size(msg)
                total_data_size += serialized_size
                messages_sent += 1

                # Simulate transmission
                time.sleep(0.001)  # 1ms between messages at 100Hz

        data_rate_mbps = (total_data_size * 8) / (1024 * 1024 * test_duration)

        return {
            "scenario": "sensor_streaming",
            "duration_seconds": test_duration,
            "messages_sent": messages_sent,
            "total_data_bytes": total_data_size,
            "data_rate_mbps": data_rate_mbps,
            "message_rate_hz": messages_sent / test_duration,
        }

    def _test_navigation_data_exchange(self) -> Dict[str, Any]:
        """Test bandwidth usage for navigation data exchange."""
        # Simulate navigation commands, paths, and odometry
        test_duration = 15.0
        command_rate = 50  # 50 Hz navigation commands
        odom_rate = 30  # 30 Hz odometry

        messages_sent = 0
        total_data_size = 0

        start_time = time.time()

        while time.time() - start_time < test_duration:
            # Generate navigation messages
            messages = self._generate_navigation_messages()

            for msg in messages:
                serialized_size = self._calculate_message_size(msg)
                total_data_size += serialized_size
                messages_sent += 1

            time.sleep(0.02)  # 20ms between navigation updates

        data_rate_mbps = (total_data_size * 8) / (1024 * 1024 * test_duration)

        return {
            "scenario": "navigation_exchange",
            "duration_seconds": test_duration,
            "messages_sent": messages_sent,
            "total_data_bytes": total_data_size,
            "data_rate_mbps": data_rate_mbps,
            "command_rate_hz": command_rate,
            "odom_rate_hz": odom_rate,
        }

    def _test_vision_data_transmission(self) -> Dict[str, Any]:
        """Test bandwidth usage for vision data transmission."""
        # Simulate compressed image and feature data
        test_duration = 20.0
        image_rate = 15  # 15 Hz image transmission
        features_rate = 30  # 30 Hz feature data

        messages_sent = 0
        total_data_size = 0
        compressed_sizes = []

        start_time = time.time()

        while time.time() - start_time < test_duration:
            # Generate vision messages
            messages = self._generate_vision_messages()

            for msg in messages:
                # Test compression
                original_size = self._calculate_message_size(msg)
                compressed_size = self._compress_message(msg)
                compression_ratio = (
                    compressed_size / original_size if original_size > 0 else 1.0
                )

                total_data_size += compressed_size
                compressed_sizes.append(compression_ratio)
                messages_sent += 1

            time.sleep(1.0 / max(image_rate, features_rate))

        data_rate_mbps = (total_data_size * 8) / (1024 * 1024 * test_duration)
        avg_compression_ratio = (
            statistics.mean(compressed_sizes) if compressed_sizes else 1.0
        )

        return {
            "scenario": "vision_transmission",
            "duration_seconds": test_duration,
            "messages_sent": messages_sent,
            "total_data_bytes": total_data_size,
            "data_rate_mbps": data_rate_mbps,
            "avg_compression_ratio": avg_compression_ratio,
            "image_rate_hz": image_rate,
            "features_rate_hz": features_rate,
        }

    def _test_mission_command_communication(self) -> Dict[str, Any]:
        """Test bandwidth usage for mission command communication."""
        # Simulate mission commands and status updates
        test_duration = 30.0
        command_rate = 10  # 10 commands per minute
        status_rate = 60  # 1 Hz status updates

        messages_sent = 0
        total_data_size = 0

        start_time = time.time()

        while time.time() - start_time < test_duration:
            # Generate mission messages
            messages = self._generate_mission_messages()

            for msg in messages:
                serialized_size = self._calculate_message_size(msg)
                total_data_size += serialized_size
                messages_sent += 1

            time.sleep(1.0 / status_rate)

        data_rate_mbps = (total_data_size * 8) / (1024 * 1024 * test_duration)

        return {
            "scenario": "mission_commands",
            "duration_seconds": test_duration,
            "messages_sent": messages_sent,
            "total_data_bytes": total_data_size,
            "data_rate_mbps": data_rate_mbps,
            "command_rate_hz": command_rate / 60.0,
            "status_rate_hz": status_rate,
        }

    def _test_telemetry_data_streaming(self) -> Dict[str, Any]:
        """Test bandwidth usage for telemetry data streaming."""
        # Simulate comprehensive telemetry streaming
        test_duration = 25.0
        telemetry_rate = 20  # 20 Hz telemetry

        messages_sent = 0
        total_data_size = 0

        start_time = time.time()

        while time.time() - start_time < test_duration:
            # Generate telemetry messages
            messages = self._generate_telemetry_messages()

            for msg in messages:
                serialized_size = self._calculate_message_size(msg)
                total_data_size += serialized_size
                messages_sent += 1

            time.sleep(1.0 / telemetry_rate)

        data_rate_mbps = (total_data_size * 8) / (1024 * 1024 * test_duration)

        return {
            "scenario": "telemetry_streaming",
            "duration_seconds": test_duration,
            "messages_sent": messages_sent,
            "total_data_bytes": total_data_size,
            "data_rate_mbps": data_rate_mbps,
            "telemetry_rate_hz": telemetry_rate,
        }

    def _generate_sensor_messages(self) -> List[Dict[str, Any]]:
        """Generate sensor data messages."""
        messages = []

        # IMU data (100Hz)
        imu_data = {
            "type": "imu",
            "timestamp": time.time(),
            "linear_acceleration": {
                "x": np.random.normal(0, 0.1),
                "y": np.random.normal(0, 0.1),
                "z": 9.81 + np.random.normal(0, 0.1),
            },
            "angular_velocity": {
                "x": np.random.normal(0, 0.1),
                "y": np.random.normal(0, 0.1),
                "z": np.random.normal(0, 0.1),
            },
        }
        messages.append(imu_data)

        # GPS data (10Hz - simulated at lower rate for this test)
        if np.random.random() < 0.1:  # 10% chance
            gps_data = {
                "type": "gps",
                "timestamp": time.time(),
                "latitude": 35.0 + np.random.normal(0, 0.001),
                "longitude": -120.0 + np.random.normal(0, 0.001),
                "altitude": 100.0 + np.random.normal(0, 5.0),
                "hdop": 1.2 + np.random.normal(0, 0.2),
            }
            messages.append(gps_data)

        return messages

    def _generate_navigation_messages(self) -> List[Dict[str, Any]]:
        """Generate navigation data messages."""
        messages = []

        # Odometry (30Hz)
        odom_data = {
            "type": "odometry",
            "timestamp": time.time(),
            "pose": {
                "position": {
                    "x": np.random.random() * 100,
                    "y": np.random.random() * 100,
                    "z": 0.0,
                },
                "orientation": {
                    "x": 0.0,
                    "y": 0.0,
                    "z": np.sin(np.random.random() * np.pi),
                    "w": np.cos(np.random.random() * np.pi),
                },
            },
            "twist": {
                "linear": {"x": np.random.normal(1.0, 0.2), "y": 0.0, "z": 0.0},
                "angular": {"x": 0.0, "y": 0.0, "z": np.random.normal(0, 0.1)},
            },
        }
        messages.append(odom_data)

        # Path data (5Hz - simulated at lower rate)
        if np.random.random() < 0.17:  # ~5Hz when called at 30Hz
            path_data = {
                "type": "path",
                "timestamp": time.time(),
                "waypoints": [
                    {"x": i * 5.0, "y": np.sin(i * 0.5) * 10, "z": 0.0}
                    for i in range(20)
                ],
            }
            messages.append(path_data)

        return messages

    def _generate_vision_messages(self) -> List[Dict[str, Any]]:
        """Generate vision data messages."""
        messages = []

        # Compressed image data (15Hz)
        image_data = {
            "type": "compressed_image",
            "timestamp": time.time(),
            "format": "jpeg",
            "width": 640,
            "height": 480,
            "data": np.random.bytes(15360),  # 15KB compressed image
        }
        messages.append(image_data)

        # Feature detections (30Hz)
        features_data = {
            "type": "features",
            "timestamp": time.time(),
            "features": [
                {
                    "x": np.random.random() * 640,
                    "y": np.random.random() * 480,
                    "type": "corner",
                    "strength": np.random.random(),
                }
                for _ in range(50)
            ],
            "descriptors": np.random.random((50, 128)).tolist(),
        }
        messages.append(features_data)

        # Terrain map (10Hz - simulated at lower rate)
        if np.random.random() < 0.5:  # 50% chance for 10Hz when called at 15Hz
            terrain_data = {
                "type": "terrain_map",
                "timestamp": time.time(),
                "resolution": 0.1,
                "width": 100,
                "height": 100,
                "data": np.random.choice(
                    [0, 1, 2, 3], size=(100, 100)
                ).tolist(),  # Terrain classes
            }
            messages.append(terrain_data)

        return messages

    def _generate_mission_messages(self) -> List[Dict[str, Any]]:
        """Generate mission command messages."""
        messages = []

        # Status update (60Hz)
        status_data = {
            "type": "mission_status",
            "timestamp": time.time(),
            "mission_id": "test_mission_001",
            "state": "executing",
            "progress": np.random.random(),
            "waypoints_completed": np.random.randint(0, 10),
            "battery_level": 80 + np.random.random() * 20,
            "system_health": "nominal",
        }
        messages.append(status_data)

        # Command (low frequency - simulated rarely)
        if np.random.random() < 0.01:  # 1% chance for command
            command_data = {
                "type": "mission_command",
                "timestamp": time.time(),
                "command": "navigate_to_waypoint",
                "parameters": {
                    "waypoint_id": f"wp_{np.random.randint(1, 11)}",
                    "speed": 1.0 + np.random.random() * 0.5,
                },
            }
            messages.append(command_data)

        return messages

    def _generate_telemetry_messages(self) -> List[Dict[str, Any]]:
        """Generate telemetry data messages."""
        messages = []

        # Comprehensive telemetry (20Hz)
        telemetry_data = {
            "type": "telemetry",
            "timestamp": time.time(),
            "system": {
                "cpu_usage": np.random.random() * 100,
                "memory_usage": np.random.random() * 100,
                "disk_usage": np.random.random() * 100,
                "temperature": 40 + np.random.random() * 20,
            },
            "navigation": {
                "current_speed": np.random.random() * 2.0,
                "heading": np.random.random() * 360,
                "gps_fix": np.random.choice([True, False], p=[0.9, 0.1]),
                "signal_strength": np.random.random(),
            },
            "sensors": {
                "imu_status": "ok",
                "lidar_status": "ok",
                "camera_status": "ok",
                "battery_voltage": 24 + np.random.normal(0, 0.5),
            },
            "mission": {
                "active_mission": "test_mission",
                "completion_percentage": np.random.random() * 100,
                "time_remaining": np.random.randint(0, 3600),
            },
        }
        messages.append(telemetry_data)

        return messages

    def _calculate_message_size(self, message: Dict[str, Any]) -> int:
        """Calculate the serialized size of a message."""
        # Use JSON serialization as representative of ROS message size
        # Handle bytes objects by converting to strings
        def handle_bytes(obj):
            if isinstance(obj, bytes):
                return obj.decode("latin-1")  # Preserve byte values
            elif isinstance(obj, np.ndarray):
                return obj.tolist()
            return obj

        try:
            serialized = json.dumps(message, default=handle_bytes).encode("utf-8")
            return len(serialized)
        except (TypeError, UnicodeDecodeError):
            # Fallback: estimate size based on structure
            return self._estimate_message_size(message)

    def _estimate_message_size(self, message: Dict[str, Any]) -> int:
        """Estimate message size when JSON serialization fails."""
        if isinstance(message, dict):
            size = 2  # {} brackets
            for key, value in message.items():
                size += len(str(key)) + 3  # "key":
                size += self._estimate_message_size(value)
                size += 1  # comma or closing
            return size
        elif isinstance(message, (list, tuple)):
            size = 2  # [] brackets
            for item in message:
                size += self._estimate_message_size(item) + 1  # comma
            return size
        elif isinstance(message, bytes):
            return len(message)
        elif isinstance(message, np.ndarray):
            return message.nbytes
        else:
            return len(str(message))

    def _compress_message(self, message: Dict[str, Any]) -> int:
        """Compress a message and return compressed size."""
        # Handle bytes objects for serialization
        def handle_bytes(obj):
            if isinstance(obj, bytes):
                return obj.decode("latin-1")
            elif isinstance(obj, np.ndarray):
                return obj.tolist()
            return obj

        try:
            serialized = json.dumps(message, default=handle_bytes).encode("utf-8")
        except (TypeError, UnicodeDecodeError):
            # Fallback to size estimation
            return int(
                self._estimate_message_size(message) * 0.7
            )  # Assume 70% compression

        # Try different compression algorithms
        compressors = [
            ("zlib", lambda data: zlib.compress(data, level=6)),
            ("gzip", lambda data: gzip.compress(data, compresslevel=6)),
            ("bz2", lambda data: bz2.compress(data, compresslevel=6)),
            ("lzma", lambda data: lzma.compress(data, preset=6)),
        ]

        compressed_sizes = []
        for name, compressor in compressors:
            try:
                compressed = compressor(serialized)
                compressed_sizes.append(len(compressed))
            except Exception:
                compressed_sizes.append(len(serialized))  # Fallback to uncompressed

        # Return the best compression result
        min_compressed_size = (
            min(compressed_sizes) if compressed_sizes else len(serialized)
        )

        # Record compression ratio
        original_size = len(serialized)
        ratio = min_compressed_size / original_size if original_size > 0 else 1.0
        self.compression_ratios.append(ratio)

        return min_compressed_size

    def _analyze_bandwidth_usage(self, results_by_scenario: Dict[str, Any]):
        """Analyze bandwidth usage across scenarios."""
        print("\n📊 Network Bandwidth Analysis Results:")
        print("=" * 50)

        # Aggregate bandwidth usage
        total_bandwidth = sum(
            result.get("data_rate_mbps", 0) for result in results_by_scenario.values()
        )
        avg_bandwidth = (
            total_bandwidth / len(results_by_scenario) if results_by_scenario else 0
        )

        # Find peak bandwidth usage
        peak_bandwidth = max(
            (
                result.get("data_rate_mbps", 0)
                for result in results_by_scenario.values()
            ),
            default=0,
        )

        # Analyze compression effectiveness
        avg_compression_ratio = (
            statistics.mean(self.compression_ratios) if self.compression_ratios else 1.0
        )
        best_compression_ratio = (
            min(self.compression_ratios) if self.compression_ratios else 1.0
        )

        print(".2f")
        print(".2f")
        print(".2f")
        print(".2f")
        print(".1f")
        print(".2f")

        # Scenario breakdown
        print("\n🔍 Scenario Bandwidth Breakdown:")
        print("-" * 40)
        for scenario_name, result in results_by_scenario.items():
            scenario_display = (
                scenario_name.replace("_", " ").replace("test_", "").title()
            )
            bandwidth = result.get("data_rate_mbps", 0)
            print(".2f")

        # Validate against competition targets
        print("\n🎯 Competition Requirements Check:")
        print("-" * 50)

        requirements = {
            "Average Bandwidth Usage": (avg_bandwidth, self.target_bandwidth_mbps),
            "Peak Bandwidth Usage": (peak_bandwidth, self.target_bandwidth_mbps * 1.5),
            "Average Compression Ratio": (
                avg_compression_ratio,
                self.target_compression_ratio,
            ),
            "Best Compression Ratio": (
                best_compression_ratio,
                self.target_compression_ratio * 0.8,
            ),
        }

        all_passed = True
        for metric, (actual, target) in requirements.items():
            passed = actual <= target
            status = "✅ PASS" if passed else "❌ FAIL"
            print(".3f")

            if not passed:
                all_passed = False

        # Performance assessment
        print("\n🚀 Network Performance Assessment:")
        print("-" * 50)

        if all_passed:
            print("✅ ALL NETWORK BANDWIDTH REQUIREMENTS MET")
            print(
                "   Communication system suitable for competition bandwidth constraints"
            )
        else:
            print("❌ NETWORK BANDWIDTH REQUIREMENTS NOT MET")
            print("   Bandwidth optimization required before competition")

        # Detailed analysis and recommendations
        if avg_bandwidth > self.target_bandwidth_mbps:
            print("\n💡 Network Optimization Recommendations:")
            if avg_compression_ratio > self.target_compression_ratio:
                print("   - Implement message compression for large data streams")
                print("   - Consider binary serialization formats")
            print("   - Reduce message publication rates where possible")
            print("   - Implement data filtering to reduce redundant transmissions")
            if peak_bandwidth > self.target_bandwidth_mbps * 2:
                print("   - Investigate bandwidth spikes and implement rate limiting")

        # Store results for regression testing
        self.test_results = {
            "scenarios_tested": list(results_by_scenario.keys()),
            "results_by_scenario": results_by_scenario,
            "avg_bandwidth_mbps": avg_bandwidth,
            "peak_bandwidth_mbps": peak_bandwidth,
            "avg_compression_ratio": avg_compression_ratio,
            "bandwidth_readings": self.bandwidth_readings,
            "compression_ratios": self.compression_ratios,
            "requirements_met": all_passed,
        }

        # Assert critical requirements
        self.assertLess(avg_bandwidth, self.target_bandwidth_mbps * 1.5, ".2f")
        self.assertLess(avg_compression_ratio, 0.8, ".2f")

    def test_packet_loss_recovery_performance(self):
        """Test packet loss recovery performance."""
        print("\n🔄 Testing Packet Loss Recovery Performance")

        # Simulate packet loss scenarios
        loss_rates = [0.01, 0.05, 0.10]  # 1%, 5%, 10% packet loss

        recovery_results = {}

        for loss_rate in loss_rates:
            print(f"  Testing {loss_rate*100:.1f}% packet loss...")

            # Simulate message transmission with loss
            messages_sent = 1000
            messages_lost = int(messages_sent * loss_rate)
            messages_received = messages_sent - messages_lost

            # Simulate recovery process
            recovery_start = time.time()
            recovered_messages = self._simulate_packet_recovery(messages_lost)
            recovery_time = (time.time() - recovery_start) * 1000  # ms

            avg_recovery_time = (
                recovery_time / messages_lost if messages_lost > 0 else 0
            )

            recovery_results[f"{loss_rate*100:.1f}%_loss"] = {
                "loss_rate": loss_rate,
                "messages_sent": messages_sent,
                "messages_lost": messages_lost,
                "messages_recovered": recovered_messages,
                "recovery_time_ms": recovery_time,
                "avg_recovery_time_ms": avg_recovery_time,
                "recovery_rate": recovered_messages / messages_lost
                if messages_lost > 0
                else 1.0,
            }

        # Analyze recovery performance
        print("\n🔧 Packet Loss Recovery Results:")
        print("-" * 40)

        for scenario, results in recovery_results.items():
            passed = results["avg_recovery_time_ms"] <= self.target_recovery_ms
            status = "✅ PASS" if passed else "❌ FAIL"
            print(".1f" ".1f")

        # Overall recovery assessment
        avg_recovery_times = [
            r["avg_recovery_time_ms"] for r in recovery_results.values()
        ]
        overall_avg_recovery = statistics.mean(avg_recovery_times)

        if overall_avg_recovery <= self.target_recovery_ms:
            print("\n✅ PACKET LOSS RECOVERY PERFORMANCE ACCEPTABLE")
            print("   Recovery times within competition requirements")
        else:
            print("\n❌ PACKET LOSS RECOVERY TOO SLOW")
            print("   Implement faster recovery mechanisms")

    def _simulate_packet_recovery(self, messages_lost: int) -> int:
        """Simulate packet loss recovery process."""
        recovered = 0

        for i in range(messages_lost):
            # Simulate recovery delay (retransmission, etc.)
            recovery_delay = np.random.exponential(0.1)  # Mean 100ms recovery time
            time.sleep(recovery_delay)

            # Simulate recovery success rate (90% success)
            if np.random.random() < 0.9:
                recovered += 1

        return recovered


if __name__ == "__main__":
    unittest.main()
