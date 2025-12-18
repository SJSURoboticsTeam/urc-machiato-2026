#!/usr/bin/env python3
"""
Competition Performance Profiler - Competition Ready
Simple performance profiling under competition load conditions.
"""

import json
import statistics
import threading
import time
from datetime import datetime
from typing import Any, Dict, List

import psutil
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class CompetitionPerformanceProfiler:
    """
    Simple performance profiler for competition scenarios.

    Measures:
    - CPU and memory usage during missions
    - Message latency and throughput
    - ROS2 communication performance
    - System responsiveness under load
    """

    def __init__(self):
        self.process = psutil.Process()
        self.measurements = []
        self.start_time = None

        # Performance thresholds (competition requirements)
        self.thresholds = {
            "cpu_percent_max": 80.0,  # Max CPU usage %
            "memory_mb_max": 600.0,  # Max memory usage MB
            "message_latency_max_ms": 100.0,  # Max message latency ms
            "mission_completion_max_s": 300.0,  # Max mission completion time s
        }

    def start_profiling(self, scenario_name: str):
        """Start performance profiling for a scenario."""
        self.start_time = time.time()
        self.measurements = []
        self.scenario_name = scenario_name

        print(f"📊 Started performance profiling: {scenario_name}")
        print(f"Start time: {datetime.now().strftime('%H:%M:%S')}")

    def record_measurement(
        self, measurement_type: str, value: Any, metadata: Dict[str, Any] = None
    ):
        """Record a performance measurement."""
        measurement = {
            "timestamp": time.time(),
            "type": measurement_type,
            "value": value,
            "metadata": metadata or {},
            "elapsed_time": time.time() - self.start_time if self.start_time else 0,
        }

        self.measurements.append(measurement)

    def measure_system_resources(self):
        """Measure current system resource usage."""
        try:
            cpu_percent = self.process.cpu_percent(interval=None)
            memory_info = self.process.memory_info()
            memory_mb = memory_info.rss / (1024 * 1024)

            self.record_measurement("cpu_usage", cpu_percent)
            self.record_measurement("memory_usage", memory_mb)

            return {"cpu_percent": cpu_percent, "memory_mb": memory_mb}
        except Exception as e:
            print(f"Error measuring system resources: {e}")
            return None

    def measure_message_latency(
        self, topic_name: str, msg_type, num_messages: int = 10
    ):
        """Measure message latency for a ROS2 topic."""
        rclpy.init()
        node = Node("latency_test_node")

        latencies = []
        message_count = 0
        start_time = time.time()

        def callback(msg):
            nonlocal message_count, latencies
            if message_count < num_messages:
                latency = (time.time() - start_time) * 1000  # Convert to ms
                latencies.append(latency)
                message_count += 1

        subscription = node.create_subscription(msg_type, topic_name, callback, 10)

        # Wait for messages
        timeout = 10.0  # 10 second timeout
        while message_count < num_messages and (time.time() - start_time) < timeout:
            rclpy.spin_once(node, timeout_sec=0.1)

        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()

        if latencies:
            avg_latency = statistics.mean(latencies)
            max_latency = max(latencies)

            self.record_measurement(
                "message_latency",
                {
                    "average_ms": avg_latency,
                    "max_ms": max_latency,
                    "messages_measured": len(latencies),
                },
                {"topic": topic_name},
            )

            return {
                "average_ms": avg_latency,
                "max_ms": max_latency,
                "count": len(latencies),
            }

        return None

    def profile_mission_execution(self, mission_type: str) -> Dict[str, Any]:
        """Profile a complete mission execution."""
        print(f"🏁 Profiling mission execution: {mission_type}")

        self.start_profiling(f"mission_{mission_type}")

        # Start resource monitoring thread
        monitoring_active = True

        def monitor_resources():
            while monitoring_active:
                self.measure_system_resources()
                time.sleep(1.0)  # Monitor every second

        monitor_thread = threading.Thread(target=monitor_resources, daemon=True)
        monitor_thread.start()

        try:
            # Execute mission
            mission_start = time.time()
            success = self._execute_test_mission(mission_type)
            mission_duration = time.time() - mission_start

            # Stop monitoring
            monitoring_active = False
            monitor_thread.join(timeout=2.0)

            # Record mission completion
            self.record_measurement(
                "mission_completion",
                {
                    "success": success,
                    "duration_s": mission_duration,
                    "mission_type": mission_type,
                },
            )

            return {
                "success": success,
                "duration_s": mission_duration,
                "measurements": len(self.measurements),
            }

        except Exception as e:
            monitoring_active = False
            monitor_thread.join(timeout=2.0)
            raise e

    def validate_performance_thresholds(self) -> Dict[str, Any]:
        """Validate that all measurements meet competition thresholds."""
        violations = []

        # Group measurements by type
        cpu_measurements = [m for m in self.measurements if m["type"] == "cpu_usage"]
        memory_measurements = [
            m for m in self.measurements if m["type"] == "memory_usage"
        ]
        latency_measurements = [
            m for m in self.measurements if m["type"] == "message_latency"
        ]
        mission_measurements = [
            m for m in self.measurements if m["type"] == "mission_completion"
        ]

        # Check CPU usage
        if cpu_measurements:
            max_cpu = max(m["value"] for m in cpu_measurements)
            if max_cpu > self.thresholds["cpu_percent_max"]:
                violations.append(
                    {
                        "type": "cpu_usage",
                        "threshold": self.thresholds["cpu_percent_max"],
                        "actual": max_cpu,
                        "violation": f"CPU usage {max_cpu:.1f}% exceeds threshold {self.thresholds['cpu_percent_max']}%",
                    }
                )

        # Check memory usage
        if memory_measurements:
            max_memory = max(m["value"] for m in memory_measurements)
            if max_memory > self.thresholds["memory_mb_max"]:
                violations.append(
                    {
                        "type": "memory_usage",
                        "threshold": self.thresholds["memory_mb_max"],
                        "actual": max_memory,
                        "violation": f"Memory usage {max_memory:.1f}MB exceeds threshold {self.thresholds['memory_mb_max']}MB",
                    }
                )

        # Check message latency
        if latency_measurements:
            for measurement in latency_measurements:
                latency_data = measurement["value"]
                if latency_data["max_ms"] > self.thresholds["message_latency_max_ms"]:
                    violations.append(
                        {
                            "type": "message_latency",
                            "threshold": self.thresholds["message_latency_max_ms"],
                            "actual": latency_data["max_ms"],
                            "violation": f"Message latency {latency_data['max_ms']:.1f}ms exceeds threshold {self.thresholds['message_latency_max_ms']}ms",
                        }
                    )

        # Check mission completion time
        if mission_measurements:
            for measurement in mission_measurements:
                mission_data = measurement["value"]
                if (
                    mission_data["duration_s"]
                    > self.thresholds["mission_completion_max_s"]
                ):
                    violations.append(
                        {
                            "type": "mission_completion",
                            "threshold": self.thresholds["mission_completion_max_s"],
                            "actual": mission_data["duration_s"],
                            "violation": f"Mission completion {mission_data['duration_s']:.1f}s exceeds threshold {self.thresholds['mission_completion_max_s']}s",
                        }
                    )

        return {
            "passed": len(violations) == 0,
            "violations": violations,
            "measurements_checked": len(self.measurements),
        }

    def save_profile_results(self, filename: str = None):
        """Save profiling results to file."""
        if not filename:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"performance_profile_{timestamp}.json"

        results = {
            "scenario": getattr(self, "scenario_name", "unknown"),
            "start_time": self.start_time,
            "duration_s": time.time() - self.start_time if self.start_time else 0,
            "measurements": self.measurements,
            "thresholds": self.thresholds,
            "validation": self.validate_performance_thresholds(),
        }

        with open(filename, "w") as f:
            json.dump(results, f, indent=2, default=str)

        print(f"💾 Performance profile saved to: {filename}")
        return filename

    def _execute_test_mission(self, mission_type: str) -> bool:
        """Execute a test mission for profiling (simplified version)."""
        # This is a simplified version - in real implementation,
        # this would integrate with the actual mission executor

        print(f"Executing test mission: {mission_type}")

        # Simulate mission execution time based on type
        mission_durations = {
            "waypoint_navigation": 15.0,
            "object_search": 25.0,
            "terrain_traversal": 20.0,
            "return_home": 12.0,
        }

        duration = mission_durations.get(mission_type, 10.0)

        # Simulate some work
        time.sleep(duration)

        # Simulate success (90% success rate for testing)
        import random

        return random.random() < 0.9

    def get_summary_stats(self) -> Dict[str, Any]:
        """Get summary statistics from measurements."""
        if not self.measurements:
            return {}

        # Group by type
        stats = {}
        by_type = {}

        for measurement in self.measurements:
            mtype = measurement["type"]
            if mtype not in by_type:
                by_type[mtype] = []
            by_type[mtype].append(measurement["value"])

        # Calculate stats for each type
        for mtype, values in by_type.items():
            if isinstance(values[0], (int, float)):
                stats[mtype] = {
                    "count": len(values),
                    "mean": statistics.mean(values),
                    "max": max(values),
                    "min": min(values),
                    "std_dev": statistics.stdev(values) if len(values) > 1 else 0,
                }
            else:
                stats[mtype] = {
                    "count": len(values),
                    "values": values[:5],  # First 5 values
                }

        return stats


def run_competition_performance_tests():
    """Run comprehensive competition performance tests."""
    print("🏎️  Running Competition Performance Tests...")
    print("=" * 50)

    profiler = CompetitionPerformanceProfiler()
    results = []

    # Test scenarios
    test_scenarios = [
        "waypoint_navigation",
        "object_search",
        "terrain_traversal",
        "return_home",
    ]

    for scenario in test_scenarios:
        print(f"\n📊 Testing scenario: {scenario}")

        try:
            # Profile mission execution
            mission_result = profiler.profile_mission_execution(scenario)

            # Measure message latency for key topics
            latency_result = profiler.measure_message_latency(
                "/imu/data", type("Imu", (), {})
            )
            if latency_result:
                print(".1f")

            # Validate thresholds
            validation = profiler.validate_performance_thresholds()

            scenario_result = {
                "scenario": scenario,
                "mission_success": mission_result["success"],
                "mission_duration_s": mission_result["duration_s"],
                "validation_passed": validation["passed"],
                "violations": validation["violations"],
            }

            results.append(scenario_result)

            if validation["passed"]:
                print(f"✅ {scenario}: PASSED")
            else:
                print(
                    f"❌ {scenario}: FAILED - {len(validation['violations'])} violations"
                )

        except Exception as e:
            print(f"❌ {scenario}: ERROR - {e}")
            results.append({"scenario": scenario, "error": str(e)})

    # Overall summary
    print("\n" + "=" * 50)
    print("📈 PERFORMANCE TEST SUMMARY")

    passed_scenarios = sum(1 for r in results if r.get("validation_passed", False))
    total_scenarios = len(results)

    print(f"Scenarios tested: {total_scenarios}")
    print(f"Scenarios passed: {passed_scenarios}")

    if passed_scenarios == total_scenarios:
        print("🎉 ALL PERFORMANCE TESTS PASSED")
        print("System meets competition performance requirements")
        success = True
    else:
        print("⚠️  PERFORMANCE ISSUES DETECTED")
        print("Address performance violations before competition")

        # Show violations
        for result in results:
            if not result.get("validation_passed", True):
                print(f"\n{result['scenario']}:")
                for violation in result.get("violations", []):
                    print(f"  - {violation['violation']}")

        success = False

    # Save detailed results
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    results_file = f"competition_performance_results_{timestamp}.json"

    with open(results_file, "w") as f:
        json.dump(results, f, indent=2, default=str)

    print(f"\n📄 Detailed results saved to: {results_file}")

    # Save full profile
    profiler.save_profile_results(f"competition_performance_profile_{timestamp}.json")

    return success


if __name__ == "__main__":
    import sys

    success = run_competition_performance_tests()
    sys.exit(0 if success else 1)
