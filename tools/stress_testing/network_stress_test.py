#!/usr/bin/env python3
"""
Network Stress Testing System - URC 2026

Comprehensive network performance testing under various conditions:
- Real ROS2 service integration
- Network emulation (latency, packet loss, bandwidth limits)
- Stress testing scenarios
- Performance metrics collection
- Detailed reporting

Author: URC 2026 Autonomy Team
"""

import asyncio
import json
import logging
import statistics
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple

import numpy as np
import psutil
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import BatteryState, Imu, NavSatFix
from std_msgs.msg import Float32, Int32, String

# Import simulation components
from simulation.network.network_emulator import NetworkEmulator, NetworkProfile
from simulation.network.network_factory import NetworkFactory


@dataclass
class NetworkTestResult:
    """Results from a network performance test."""

    test_name: str
    duration_seconds: float
    messages_sent: int
    messages_received: int
    messages_dropped: int
    average_latency_ms: float
    packet_loss_percent: float
    bandwidth_used_mbps: float
    cpu_usage_percent: float
    memory_usage_mb: float
    network_profile: str
    success: bool
    error_message: Optional[str] = None


@dataclass
class StressTestScenario:
    """Definition of a stress test scenario."""

    name: str
    description: str
    network_profile: NetworkProfile
    message_rate_hz: int
    test_duration_seconds: int
    concurrent_publishers: int
    message_size_bytes: int
    burst_mode: bool = False
    burst_duration_seconds: float = 1.0


class ROS2NetworkStressTester(Node):
    """
    ROS2 Network Stress Testing Node

    Tests network performance under various conditions by:
    1. Creating network-emulated ROS2 publishers/subscribers
    2. Running stress scenarios with high message rates
    3. Measuring latency, throughput, and reliability
    4. Generating comprehensive performance reports
    """

    def __init__(self):
        super().__init__("network_stress_tester")

        # Test configuration
        self.test_active = False
        self.current_scenario: Optional[StressTestScenario] = None
        self.network_emulator: Optional[NetworkEmulator] = None

        # Performance tracking
        self.test_results: List[NetworkTestResult] = []
        self.message_counts = {}
        self.latency_measurements = []
        self.bandwidth_measurements = []
        self.system_metrics = []

        # ROS2 communication
        self.publishers = {}
        self.subscribers = {}
        self.test_topics = []

        # Control topics
        self.control_sub = self.create_subscription(
            String, "/network_test/control", self._control_callback, 10
        )

        self.status_pub = self.create_publisher(String, "/network_test/status", 10)

        self.results_pub = self.create_publisher(String, "/network_test/results", 10)

        self.get_logger().info("Network Stress Tester initialized")

    def _control_callback(self, msg):
        """Handle test control commands."""
        try:
            command = json.loads(msg.data)

            if command["type"] == "start_test":
                scenario = self._create_scenario_from_config(command["scenario"])
                asyncio.create_task(self.run_stress_test(scenario))

            elif command["type"] == "stop_test":
                self.stop_current_test()

            elif command["type"] == "get_status":
                self._publish_status()

        except Exception as e:
            self.get_logger().error(f"Control command error: {e}")

    def _create_scenario_from_config(
        self, config: Dict[str, Any]
    ) -> StressTestScenario:
        """Create test scenario from configuration."""
        return StressTestScenario(
            name=config.get("name", "custom_test"),
            description=config.get("description", "Custom network stress test"),
            network_profile=NetworkProfile(config.get("network_profile", "rural_wifi")),
            message_rate_hz=config.get("message_rate_hz", 100),
            test_duration_seconds=config.get("duration_seconds", 30),
            concurrent_publishers=config.get("concurrent_publishers", 5),
            message_size_bytes=config.get("message_size_bytes", 1024),
            burst_mode=config.get("burst_mode", False),
            burst_duration_seconds=config.get("burst_duration_seconds", 1.0),
        )

    async def run_stress_test(self, scenario: StressTestScenario):
        """Run a network stress test scenario."""
        self.get_logger().info(f"Starting stress test: {scenario.name}")
        self.current_scenario = scenario
        self.test_active = True

        try:
            # Initialize network emulator
            self.network_emulator = NetworkFactory.create(
                {"profile": scenario.network_profile.value}
            )
            self.network_emulator.start()

            # Setup ROS2 topics with network emulation
            await self._setup_emulated_topics(scenario)

            # Start performance monitoring
            monitor_thread = threading.Thread(target=self._monitor_performance)
            monitor_thread.start()

            # Run test scenario
            start_time = time.time()
            await self._execute_test_scenario(scenario)
            duration = time.time() - start_time

            # Collect results
            result = await self._collect_test_results(scenario, duration)
            self.test_results.append(result)

            # Publish results
            self._publish_test_results(result)

            monitor_thread.join(timeout=5.0)

        except Exception as e:
            self.get_logger().error(f"Stress test failed: {e}")
            error_result = NetworkTestResult(
                test_name=scenario.name,
                duration_seconds=0.0,
                messages_sent=0,
                messages_received=0,
                messages_dropped=0,
                average_latency_ms=0.0,
                packet_loss_percent=0.0,
                bandwidth_used_mbps=0.0,
                cpu_usage_percent=0.0,
                memory_usage_mb=0.0,
                network_profile=scenario.network_profile.value,
                success=False,
                error_message=str(e),
            )
            self.test_results.append(error_result)

        finally:
            self._cleanup_test()
            self.test_active = False

    async def _setup_emulated_topics(self, scenario: StressTestScenario):
        """Setup ROS2 topics with network emulation."""
        self.test_topics = []

        for i in range(scenario.concurrent_publishers):
            topic_name = f"/network_test/data_{i}"

            # Create publisher
            publisher = self.create_publisher(String, topic_name, 10)
            self.publishers[topic_name] = publisher

            # Create subscriber with latency tracking
            def create_callback(topic_name):
                def callback(msg):
                    self._handle_test_message(topic_name, msg)

                return callback

            subscriber = self.create_subscription(
                String, topic_name, create_callback(topic_name), 10
            )
            self.subscribers[topic_name] = subscriber
            self.test_topics.append(topic_name)

            # Initialize message tracking
            self.message_counts[topic_name] = {
                "sent": 0,
                "received": 0,
                "latencies": [],
            }

    async def _execute_test_scenario(self, scenario: StressTestScenario):
        """Execute the actual test scenario."""
        end_time = time.time() + scenario.test_duration_seconds

        with ThreadPoolExecutor(max_workers=scenario.concurrent_publishers) as executor:
            futures = []

            for i in range(scenario.concurrent_publishers):
                topic_name = f"/network_test/data_{i}"
                future = executor.submit(
                    self._publisher_worker, topic_name, scenario, end_time
                )
                futures.append(future)

            # Wait for all publishers to complete
            for future in futures:
                future.result()

    def _publisher_worker(
        self, topic_name: str, scenario: StressTestScenario, end_time: float
    ):
        """Worker function for publishing test messages."""
        publisher = self.publishers[topic_name]

        while time.time() < end_time and self.test_active:
            if scenario.burst_mode:
                # Burst mode: send many messages quickly
                burst_end = time.time() + scenario.burst_duration_seconds
                while time.time() < burst_end and self.test_active:
                    self._send_test_message(
                        publisher, topic_name, scenario.message_size_bytes
                    )
                    time.sleep(0.001)  # 1ms between messages in burst

                # Wait before next burst
                time.sleep(0.1)
            else:
                # Continuous mode
                self._send_test_message(
                    publisher, topic_name, scenario.message_size_bytes
                )
                time.sleep(1.0 / scenario.message_rate_hz)

    def _send_test_message(self, publisher, topic_name: str, message_size: int):
        """Send a test message."""
        # Create test message with timestamp
        message_data = {
            "timestamp": time.time(),
            "topic": topic_name,
            "data": "x" * message_size,  # Fixed size test data
            "sequence": self.message_counts[topic_name]["sent"],
        }

        msg = String()
        msg.data = json.dumps(message_data)

        # Send through network emulator if available
        if self.network_emulator:
            if self.network_emulator.send_message(msg):
                self.message_counts[topic_name]["sent"] += 1
        else:
            publisher.publish(msg)
            self.message_counts[topic_name]["sent"] += 1

    def _handle_test_message(self, topic_name: str, msg):
        """Handle received test message."""
        try:
            message_data = json.loads(msg.data)
            send_time = message_data["timestamp"]
            latency = (time.time() - send_time) * 1000  # Convert to milliseconds

            self.message_counts[topic_name]["received"] += 1
            self.message_counts[topic_name]["latencies"].append(latency)
            self.latency_measurements.append(latency)

        except Exception as e:
            self.get_logger().error(f"Message handling error: {e}")

    def _monitor_performance(self):
        """Monitor system performance during test."""
        while self.test_active:
            try:
                # System metrics
                cpu_percent = psutil.cpu_percent(interval=1)
                memory = psutil.virtual_memory()
                memory_mb = memory.used / (1024 * 1024)

                # Network metrics (simplified)
                net_io = psutil.net_io_counters()
                bandwidth_mbps = (
                    (net_io.bytes_sent + net_io.bytes_recv) * 8 / (1024 * 1024)
                )  # Rough estimate

                self.system_metrics.append(
                    {
                        "timestamp": time.time(),
                        "cpu_percent": cpu_percent,
                        "memory_mb": memory_mb,
                        "bandwidth_mbps": bandwidth_mbps,
                    }
                )

                self.bandwidth_measurements.append(bandwidth_mbps)

                time.sleep(1.0)

            except Exception as e:
                self.get_logger().error(f"Performance monitoring error: {e}")
                break

    async def _collect_test_results(
        self, scenario: StressTestScenario, duration: float
    ) -> NetworkTestResult:
        """Collect and analyze test results."""
        # Aggregate message counts
        total_sent = sum(counts["sent"] for counts in self.message_counts.values())
        total_received = sum(
            counts["received"] for counts in self.message_counts.values()
        )

        # Calculate packet loss
        packet_loss = (
            ((total_sent - total_received) / total_sent * 100) if total_sent > 0 else 0
        )

        # Calculate average latency
        avg_latency = (
            statistics.mean(self.latency_measurements)
            if self.latency_measurements
            else 0
        )

        # Calculate average bandwidth and system usage
        avg_bandwidth = (
            statistics.mean(self.bandwidth_measurements)
            if self.bandwidth_measurements
            else 0
        )
        avg_cpu = (
            statistics.mean([m["cpu_percent"] for m in self.system_metrics])
            if self.system_metrics
            else 0
        )
        avg_memory = (
            statistics.mean([m["memory_mb"] for m in self.system_metrics])
            if self.system_metrics
            else 0
        )

        # Get network emulator statistics
        emulator_stats = (
            self.network_emulator.get_statistics() if self.network_emulator else {}
        )

        return NetworkTestResult(
            test_name=scenario.name,
            duration_seconds=duration,
            messages_sent=total_sent,
            messages_received=total_received,
            messages_dropped=total_sent - total_received,
            average_latency_ms=avg_latency,
            packet_loss_percent=packet_loss,
            bandwidth_used_mbps=avg_bandwidth,
            cpu_usage_percent=avg_cpu,
            memory_usage_mb=avg_memory,
            network_profile=scenario.network_profile.value,
            success=packet_loss < 5.0 and avg_latency < 500.0,  # Basic success criteria
        )

    def _publish_status(self):
        """Publish current test status."""
        status = {
            "active": self.test_active,
            "current_scenario": self.current_scenario.name
            if self.current_scenario
            else None,
            "total_tests_run": len(self.test_results),
            "network_emulator_active": self.network_emulator is not None,
        }

        msg = String()
        msg.data = json.dumps(status)
        self.status_pub.publish(msg)

    def _publish_test_results(self, result: NetworkTestResult):
        """Publish test results."""
        result_data = {
            "test_name": result.test_name,
            "duration_seconds": result.duration_seconds,
            "messages_sent": result.messages_sent,
            "messages_received": result.messages_received,
            "messages_dropped": result.messages_dropped,
            "average_latency_ms": result.average_latency_ms,
            "packet_loss_percent": result.packet_loss_percent,
            "bandwidth_used_mbps": result.bandwidth_used_mbps,
            "cpu_usage_percent": result.cpu_usage_percent,
            "memory_usage_mb": result.memory_usage_mb,
            "network_profile": result.network_profile,
            "success": result.success,
            "error_message": result.error_message,
        }

        msg = String()
        msg.data = json.dumps(result_data)
        self.results_pub.publish(msg)

    def stop_current_test(self):
        """Stop the currently running test."""
        self.test_active = False
        if self.network_emulator:
            self.network_emulator.stop()

    def _cleanup_test(self):
        """Clean up after test completion."""
        # Clear message counts and measurements
        self.message_counts.clear()
        self.latency_measurements.clear()
        self.bandwidth_measurements.clear()
        self.system_metrics.clear()

        # Stop network emulator
        if self.network_emulator:
            self.network_emulator.stop()
            self.network_emulator = None

    def generate_comprehensive_report(self) -> Dict[str, Any]:
        """Generate comprehensive test report."""
        if not self.test_results:
            return {"status": "no_tests_run"}

        # Aggregate statistics
        total_tests = len(self.test_results)
        successful_tests = sum(1 for r in self.test_results if r.success)
        avg_latency = statistics.mean([r.average_latency_ms for r in self.test_results])
        avg_packet_loss = statistics.mean(
            [r.packet_loss_percent for r in self.test_results]
        )
        avg_bandwidth = statistics.mean(
            [r.bandwidth_used_mbps for r in self.test_results]
        )

        # Performance by network profile
        profile_stats = {}
        for result in self.test_results:
            profile = result.network_profile
            if profile not in profile_stats:
                profile_stats[profile] = []
            profile_stats[profile].append(result)

        profile_summary = {}
        for profile, results in profile_stats.items():
            profile_summary[profile] = {
                "tests_run": len(results),
                "success_rate": sum(1 for r in results if r.success) / len(results),
                "avg_latency_ms": statistics.mean(
                    [r.average_latency_ms for r in results]
                ),
                "avg_packet_loss": statistics.mean(
                    [r.packet_loss_percent for r in results]
                ),
                "avg_bandwidth_mbps": statistics.mean(
                    [r.bandwidth_used_mbps for r in results]
                ),
            }

        return {
            "summary": {
                "total_tests": total_tests,
                "successful_tests": successful_tests,
                "success_rate_percent": (successful_tests / total_tests) * 100,
                "average_latency_ms": avg_latency,
                "average_packet_loss_percent": avg_packet_loss,
                "average_bandwidth_mbps": avg_bandwidth,
            },
            "performance_by_profile": profile_summary,
            "individual_results": [
                {
                    "test_name": r.test_name,
                    "success": r.success,
                    "latency_ms": r.average_latency_ms,
                    "packet_loss_percent": r.packet_loss_percent,
                    "bandwidth_mbps": r.bandwidth_used_mbps,
                }
                for r in self.test_results
            ],
            "recommendations": self._generate_recommendations(),
        }

    def _generate_recommendations(self) -> List[str]:
        """Generate performance recommendations."""
        recommendations = []

        if self.test_results:
            avg_latency = statistics.mean(
                [r.average_latency_ms for r in self.test_results]
            )
            avg_packet_loss = statistics.mean(
                [r.packet_loss_percent for r in self.test_results]
            )

            if avg_latency > 200:
                recommendations.append(
                    "Consider implementing message batching to reduce latency"
                )

            if avg_packet_loss > 2.0:
                recommendations.append(
                    "Implement reliable message delivery protocols for high packet loss environments"
                )

            if any(r.bandwidth_used_mbps > 15 for r in self.test_results):
                recommendations.append(
                    "Optimize message sizes and implement compression for bandwidth-constrained networks"
                )

        return recommendations


def create_stress_test_scenarios() -> List[StressTestScenario]:
    """Create predefined stress test scenarios."""
    return [
        StressTestScenario(
            name="perfect_network_baseline",
            description="Baseline test under perfect network conditions",
            network_profile=NetworkProfile.PERFECT,
            message_rate_hz=100,
            test_duration_seconds=30,
            concurrent_publishers=3,
            message_size_bytes=512,
        ),
        StressTestScenario(
            name="rural_wifi_stress",
            description="High-load test under rural WiFi conditions",
            network_profile=NetworkProfile.RURAL_WIFI,
            message_rate_hz=200,
            test_duration_seconds=45,
            concurrent_publishers=5,
            message_size_bytes=1024,
        ),
        StressTestScenario(
            name="cellular_4g_burst",
            description="Burst traffic test under 4G cellular conditions",
            network_profile=NetworkProfile.CELLULAR_4G,
            message_rate_hz=50,
            test_duration_seconds=60,
            concurrent_publishers=8,
            message_size_bytes=2048,
            burst_mode=True,
            burst_duration_seconds=2.0,
        ),
        StressTestScenario(
            name="satellite_extreme",
            description="Extreme conditions test under satellite connection",
            network_profile=NetworkProfile.SATELLITE,
            message_rate_hz=20,
            test_duration_seconds=90,
            concurrent_publishers=2,
            message_size_bytes=4096,
        ),
        StressTestScenario(
            name="competition_load_test",
            description="Full competition load simulation",
            network_profile=NetworkProfile.EXTREME,
            message_rate_hz=500,
            test_duration_seconds=120,
            concurrent_publishers=10,
            message_size_bytes=8192,
            burst_mode=True,
            burst_duration_seconds=5.0,
        ),
    ]


async def run_network_stress_tests():
    """Run comprehensive network stress tests."""
    print("🌐 Starting Network Stress Testing Suite")
    print("=" * 60)

    # Initialize ROS2
    rclpy.init()

    # Create stress tester
    tester = ROS2NetworkStressTester()

    # Get test scenarios
    scenarios = create_stress_test_scenarios()

    print(f"📋 Running {len(scenarios)} stress test scenarios...")

    # Run all scenarios
    for i, scenario in enumerate(scenarios, 1):
        print(f"\n🔄 Scenario {i}/{len(scenarios)}: {scenario.name}")
        print(f"   {scenario.description}")
        print(f"   Network: {scenario.network_profile.value}")
        print(
            f"   Load: {scenario.message_rate_hz}Hz x {scenario.concurrent_publishers} publishers"
        )
        print(f"   Duration: {scenario.test_duration_seconds}s")

        await tester.run_stress_test(scenario)

        # Brief pause between tests
        await asyncio.sleep(2.0)

    # Generate comprehensive report
    print("\n📊 Generating Comprehensive Performance Report...")
    report = tester.generate_comprehensive_report()

    # Save report
    report_file = f"network_stress_test_report_{int(time.time())}.json"
    with open(report_file, "w") as f:
        json.dump(report, f, indent=2)

    # Display summary
    print("\n🎯 NETWORK STRESS TESTING RESULTS")
    print("=" * 60)

    summary = report["summary"]
    print(f"Total Tests Run: {summary['total_tests']}")
    print(f"Success Rate: {summary['success_rate_percent']:.1f}%")
    print(f"Average Latency: {summary['average_latency_ms']:.1f}ms")
    print(f"Average Packet Loss: {summary['average_packet_loss_percent']:.2f}%")
    print(f"Average Bandwidth: {summary['average_bandwidth_mbps']:.2f}Mbps")

    print("\n📈 Performance by Network Profile:")
    for profile, stats in report["performance_by_profile"].items():
        print(f"  {profile}:")
        print(".1f")
        print(".1f")
        print(".2f")

    if report.get("recommendations"):
        print("\n💡 Recommendations:")
        for rec in report["recommendations"]:
            print(f"  • {rec}")

    print(f"\n📄 Detailed report saved to: {report_file}")

    # Cleanup
    tester.destroy_node()
    rclpy.shutdown()


def main():
    """Main entry point."""
    asyncio.run(run_network_stress_tests())


if __name__ == "__main__":
    main()
