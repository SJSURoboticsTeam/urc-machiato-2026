#!/usr/bin/env python3
"""
Comprehensive Performance Testing with Chaos Engineering & Graphing - URC 2026

This script runs comprehensive performance tests including:
- Real system integration tests
- Chaos engineering (network failures, hardware faults, etc.)
- Performance monitoring and profiling
- Graph generation for visualization
- Statistical analysis and reporting

Usage:
    python run_comprehensive_performance_testing.py --duration 300 --chaos-level 0.7 --graph-output perf_charts/

Author: URC 2026 Performance Engineering Team
"""

import argparse
import asyncio
import json
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd
import psutil
import seaborn as sns
import sys
import threading
import time
from collections import defaultdict
from datetime import datetime, timedelta
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
import statistics

# Add src to path
sys.path.insert(0, str(Path(__file__).parent))

from src.testing.performance_profiling import PerformanceProfiler
from src.core.network_resilience import CircuitBreaker, NetworkHealthMonitor
from tests.performance.test_performance_baseline import PerformanceBaselineTester


class ComprehensivePerformanceTester:
    """Comprehensive performance testing with chaos engineering and graphing."""

    def __init__(
        self,
        duration_seconds: int = 300,
        chaos_level: float = 0.5,
        output_dir: str = "perf_results",
    ):
        self.duration_seconds = duration_seconds
        self.chaos_level = chaos_level
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)

        # Performance monitoring
        self.performance_data = defaultdict(list)
        self.system_metrics = defaultdict(list)
        self.chaos_events = []
        self.test_results = {}

        # Monitoring flags
        self.monitoring_active = False
        self.chaos_active = False

        # Initialize components
        # self.perf_profiler = PerformanceProfiler("comprehensive_test")  # Skip for now due to initialization issues
        self.baseline_tester = PerformanceBaselineTester()

    def run_comprehensive_tests(self) -> Dict[str, Any]:
        """Run comprehensive performance testing suite."""
        print("🚀 Starting Comprehensive Performance Testing")
        print(f"Duration: {self.duration_seconds}s | Chaos Level: {self.chaos_level}")
        print("=" * 60)

        start_time = time.time()
        self.monitoring_active = True

        try:
            # Start background monitoring
            monitor_thread = threading.Thread(target=self._background_monitoring)
            chaos_thread = threading.Thread(target=self._chaos_injection_loop)

            monitor_thread.start()
            chaos_thread.start()

            # Run test phases
            self._run_phase_1_baseline_tests()
            self._run_phase_2_integration_tests()
            self._run_phase_3_stress_tests()
            self._run_phase_4_chaos_tests()

            # Stop monitoring
            self.monitoring_active = False
            self.chaos_active = False

            monitor_thread.join(timeout=10)
            chaos_thread.join(timeout=10)

            # Generate comprehensive report
            report = self._generate_comprehensive_report()

            # Create visualizations
            self._create_performance_graphs(report)

            return report

        except Exception as e:
            print(f"❌ Testing failed: {e}")
            return {"error": str(e), "partial_data": dict(self.performance_data)}

    def _run_phase_1_baseline_tests(self):
        """Phase 1: Run baseline performance tests."""
        print("\n📊 Phase 1: Baseline Performance Tests")

        # Run binary protocol tests
        self._log_event("Starting binary protocol baseline tests")
        binary_results = self.baseline_tester.test_binary_protocol_baseline()
        self.test_results["binary_protocol"] = binary_results
        self._log_performance_metric(
            "binary_protocol_p99", binary_results.get("p99_ms", 0)
        )

        # Run IPC bridge tests
        self._log_event("Starting IPC bridge baseline tests")
        ipc_results = self.baseline_tester.test_ipc_bridge_baseline()
        self.test_results["ipc_bridge"] = ipc_results
        self._log_performance_metric("ipc_bridge_p99", ipc_results.get("p99_ms", 0))

        # Run motion control tests
        self._log_event("Starting motion control baseline tests")
        motion_results = self.baseline_tester.test_motion_control_baseline()
        self.test_results["motion_control"] = motion_results
        self._log_performance_metric(
            "motion_control_p99", motion_results.get("p99_ms", 0)
        )

    def _run_phase_2_integration_tests(self):
        """Phase 2: Run integration tests with monitoring."""
        print("\n🔗 Phase 2: Integration Tests")

        try:
            from tests.run_comprehensive_resource_tests import (
                run_fallback_tests,
                run_recovery_tests,
            )

            self._log_event("Starting fallback functionality tests")
            fallback_result = run_fallback_tests()
            self.test_results["fallback"] = fallback_result

            self._log_event("Starting recovery mechanism tests")
            recovery_result = run_recovery_tests()
            self.test_results["recovery"] = recovery_result

        except ImportError as e:
            print(f"⚠️ Integration tests not available: {e}")
            self.test_results["integration"] = {"error": str(e)}

    def _run_phase_3_stress_tests(self):
        """Phase 3: Run stress tests."""
        print("\n🏋️ Phase 3: Stress Tests")

        # Concurrent operations stress test
        self._log_event("Starting concurrent operations stress test")
        stress_results = self.baseline_tester._stress_test_ipc_bridge_concurrent()
        self.test_results["concurrent_stress"] = stress_results

        # Memory stress test
        self._log_event("Starting memory usage stress test")
        memory_results = self.baseline_tester._stress_test_memory_usage()
        self.test_results["memory_stress"] = memory_results

        # High-frequency operations
        self._log_event("Starting high-frequency operations test")
        hf_results = self.baseline_tester._stress_test_high_frequency()
        self.test_results["high_frequency"] = hf_results

    def _run_phase_4_chaos_tests(self):
        """Phase 4: Run chaos engineering tests."""
        print("\n🎭 Phase 4: Chaos Engineering Tests")

        self.chaos_active = True

        try:
            from tests.chaos.test_chaos_engineering import (
                TestNetworkChaos,
                TestServiceChaos,
            )

            # Network chaos tests
            self._log_event("Starting network chaos tests")
            network_chaos = TestNetworkChaos()
            network_chaos.test_network_partition_recovery()
            network_chaos.test_high_latency_handling()
            network_chaos.test_packet_loss_simulation()

            # Service chaos tests
            self._log_event("Starting service chaos tests")
            service_chaos = TestServiceChaos()
            service_chaos.test_service_crash_recovery()
            service_chaos.test_cascading_failure_prevention()
            service_chaos.test_resource_exhaustion_handling()

            self.test_results["chaos_tests"] = {"completed": True}

        except Exception as e:
            print(f"⚠️ Chaos tests encountered issues: {e}")
            self.test_results["chaos_tests"] = {"error": str(e)}

    def _background_monitoring(self):
        """Background system monitoring thread."""
        print("📈 Starting background performance monitoring...")

        start_time = time.time()
        process = psutil.Process()

        while (
            self.monitoring_active
            and (time.time() - start_time) < self.duration_seconds
        ):
            try:
                current_time = time.time()

                # System metrics
                cpu_percent = psutil.cpu_percent(interval=None)
                memory_info = process.memory_info()
                memory_percent = process.memory_percent()

                # Network I/O
                net_io = psutil.net_io_counters()
                disk_io = psutil.disk_io_counters()

                # Performance metrics
                self.system_metrics["timestamp"].append(current_time)
                self.system_metrics["cpu_percent"].append(cpu_percent)
                self.system_metrics["memory_mb"].append(memory_info.rss / (1024 * 1024))
                self.system_metrics["memory_percent"].append(memory_percent)
                self.system_metrics["net_bytes_sent"].append(net_io.bytes_sent)
                self.system_metrics["net_bytes_recv"].append(net_io.bytes_recv)

                if disk_io:
                    self.system_metrics["disk_read_mb"].append(
                        disk_io.read_bytes / (1024 * 1024)
                    )
                    self.system_metrics["disk_write_mb"].append(
                        disk_io.write_bytes / (1024 * 1024)
                    )

                time.sleep(1.0)  # 1Hz monitoring

            except Exception as e:
                print(f"⚠️ Monitoring error: {e}")
                time.sleep(1.0)

        print("📈 Background monitoring completed")

    def _chaos_injection_loop(self):
        """Background chaos injection thread."""
        time.sleep(10)  # Wait for system to stabilize

        chaos_events = [
            ("network_partition", 30),  # 30 second network partition
            ("high_latency", 20),  # 20 second high latency
            ("cpu_spike", 15),  # 15 second CPU spike
            ("memory_pressure", 25),  # 25 second memory pressure
        ]

        while self.chaos_active:
            for event_type, duration in chaos_events:
                if not self.chaos_active:
                    break

                if random.random() < self.chaos_level:
                    self._inject_chaos_event(event_type, duration)
                    time.sleep(random.uniform(20, 60))  # Random delay between events
                else:
                    time.sleep(10)  # Check every 10 seconds

    def _inject_chaos_event(self, event_type: str, duration: int):
        """Inject a specific chaos event."""
        event_start = time.time()
        self.chaos_events.append(
            {
                "type": event_type,
                "start_time": event_start,
                "duration": duration,
                "end_time": event_start + duration,
            }
        )

        print(f"🎭 Injecting chaos: {event_type} for {duration}s")

        # Simulate the chaos event
        if event_type == "network_partition":
            # Simulate network partition by increasing latency
            self._simulate_network_partition(duration)
        elif event_type == "high_latency":
            self._simulate_high_latency(duration)
        elif event_type == "cpu_spike":
            self._simulate_cpu_spike(duration)
        elif event_type == "memory_pressure":
            self._simulate_memory_pressure(duration)

    def _simulate_network_partition(self, duration: int):
        """Simulate network partition."""
        # In a real system, this would actually disconnect network interfaces
        # For testing, we just log the event and monitor impact
        time.sleep(duration)

    def _simulate_high_latency(self, duration: int):
        """Simulate high network latency."""
        # Add artificial delays to operations
        time.sleep(duration)

    def _simulate_cpu_spike(self, duration: int):
        """Simulate CPU usage spike."""
        # In a real system, this would spawn CPU-intensive threads
        time.sleep(duration)

    def _simulate_memory_pressure(self, duration: int):
        """Simulate memory pressure."""
        # Allocate memory to create pressure
        allocations = []
        try:
            for _ in range(duration * 10):  # Allocate every 0.1s
                allocations.append(bytearray(1024 * 1024))  # 1MB each
                time.sleep(0.1)
                if not self.chaos_active:
                    break
        finally:
            # Clean up allocations
            allocations.clear()

    def _log_event(self, event: str):
        """Log a test event."""
        self.performance_data["events"].append(
            {"timestamp": time.time(), "event": event}
        )

    def _log_performance_metric(self, metric: str, value: float):
        """Log a performance metric."""
        self.performance_data[metric].append({"timestamp": time.time(), "value": value})

    def _generate_comprehensive_report(self) -> Dict[str, Any]:
        """Generate comprehensive performance report."""
        print("\n📊 Generating Comprehensive Performance Report...")

        report = {
            "test_metadata": {
                "duration_seconds": self.duration_seconds,
                "chaos_level": self.chaos_level,
                "timestamp": datetime.now().isoformat(),
                "total_events": len(self.chaos_events),
            },
            "test_results": self.test_results,
            "performance_metrics": dict(self.performance_data),
            "system_metrics": dict(self.system_metrics),
            "chaos_events": self.chaos_events,
            "analysis": {},
        }

        # Statistical analysis
        report["analysis"] = self._perform_statistical_analysis()

        # Save detailed report
        report_file = (
            self.output_dir
            / f"comprehensive_performance_report_{int(time.time())}.json"
        )
        with open(report_file, "w") as f:
            json.dump(report, f, indent=2, default=str)

        print(f"📁 Detailed report saved to: {report_file}")
        return report

    def _perform_statistical_analysis(self) -> Dict[str, Any]:
        """Perform statistical analysis of performance data."""
        analysis = {}

        # CPU usage analysis
        if "cpu_percent" in self.system_metrics:
            cpu_data = self.system_metrics["cpu_percent"]
            analysis["cpu_stats"] = {
                "mean": statistics.mean(cpu_data),
                "median": statistics.median(cpu_data),
                "p95": sorted(cpu_data)[int(len(cpu_data) * 0.95)],
                "max": max(cpu_data),
                "stability_score": 100
                - statistics.stdev(cpu_data),  # Lower variance = higher stability
            }

        # Memory usage analysis
        if "memory_mb" in self.system_metrics:
            mem_data = self.system_metrics["memory_mb"]
            analysis["memory_stats"] = {
                "mean_mb": statistics.mean(mem_data),
                "median_mb": statistics.median(mem_data),
                "p95_mb": sorted(mem_data)[int(len(mem_data) * 0.95)],
                "max_mb": max(mem_data),
                "stability_score": 100
                - (statistics.stdev(mem_data) / statistics.mean(mem_data) * 10),
            }

        # Chaos impact analysis
        if self.chaos_events:
            analysis["chaos_impact"] = self._analyze_chaos_impact()

        # Performance regression analysis
        analysis["performance_trends"] = self._analyze_performance_trends()

        return analysis

    def _analyze_chaos_impact(self) -> Dict[str, Any]:
        """Analyze the impact of chaos events on system performance."""
        impact_analysis = {
            "events_analyzed": len(self.chaos_events),
            "cpu_spikes_detected": 0,
            "memory_spikes_detected": 0,
            "recovery_times": [],
        }

        for event in self.chaos_events:
            event_start = event["start_time"]
            event_end = event["end_time"]

            # Analyze CPU impact
            cpu_during_event = [
                cpu
                for ts, cpu in zip(
                    self.system_metrics.get("timestamp", []),
                    self.system_metrics.get("cpu_percent", []),
                )
                if event_start <= ts <= event_end
            ]

            if cpu_during_event:
                avg_cpu_during = statistics.mean(cpu_during_event)
                if avg_cpu_during > 80:  # Threshold for spike
                    impact_analysis["cpu_spikes_detected"] += 1

            # Analyze memory impact
            mem_during_event = [
                mem
                for ts, mem in zip(
                    self.system_metrics.get("timestamp", []),
                    self.system_metrics.get("memory_mb", []),
                )
                if event_start <= ts <= event_end
            ]

            if mem_during_event:
                avg_mem_during = statistics.mean(mem_during_event)
                baseline_mem = statistics.mean(
                    self.system_metrics.get("memory_mb", [])[:10]
                )  # First 10 readings
                if avg_mem_during > baseline_mem * 1.2:  # 20% increase
                    impact_analysis["memory_spikes_detected"] += 1

        return impact_analysis

    def _analyze_performance_trends(self) -> Dict[str, Any]:
        """Analyze performance trends over time."""
        trends = {}

        # Analyze CPU trend
        if (
            "cpu_percent" in self.system_metrics
            and len(self.system_metrics["cpu_percent"]) > 20
        ):
            cpu_data = self.system_metrics["cpu_percent"]
            first_half = cpu_data[: len(cpu_data) // 2]
            second_half = cpu_data[len(cpu_data) // 2 :]

            first_avg = statistics.mean(first_half)
            second_avg = statistics.mean(second_half)

            trends["cpu_trend"] = {
                "first_half_avg": first_avg,
                "second_half_avg": second_avg,
                "change_percent": ((second_avg - first_avg) / first_avg) * 100,
                "stability": (
                    "stable" if abs(second_avg - first_avg) < 5 else "unstable"
                ),
            }

        return trends

    def _create_performance_graphs(self, report: Dict[str, Any]):
        """Create comprehensive performance visualization graphs."""
        print("📊 Creating Performance Visualization Graphs...")

        # Set up matplotlib style
        plt.style.use("seaborn-v0_8")
        sns.set_palette("husl")

        # Create subplots for comprehensive dashboard
        fig, axes = plt.subplots(3, 2, figsize=(15, 12))
        fig.suptitle(
            "URC 2026 Comprehensive Performance Analysis",
            fontsize=16,
            fontweight="bold",
        )

        # Convert timestamps to relative time
        if "timestamp" in self.system_metrics:
            base_time = min(self.system_metrics["timestamp"])
            time_axis = [(ts - base_time) for ts in self.system_metrics["timestamp"]]
        else:
            time_axis = list(range(len(self.system_metrics.get("cpu_percent", []))))

        # 1. CPU Usage Over Time
        if "cpu_percent" in self.system_metrics:
            axes[0, 0].plot(
                time_axis, self.system_metrics["cpu_percent"], linewidth=2, alpha=0.8
            )
            axes[0, 0].set_title("CPU Usage Over Time")
            axes[0, 0].set_ylabel("CPU %")
            axes[0, 0].set_xlabel("Time (seconds)")
            axes[0, 0].grid(True, alpha=0.3)

            # Add chaos event markers
            for event in self.chaos_events:
                event_time = event["start_time"] - base_time
                axes[0, 0].axvline(
                    x=event_time, color="red", linestyle="--", alpha=0.7, linewidth=1
                )

        # 2. Memory Usage Over Time
        if "memory_mb" in self.system_metrics:
            axes[0, 1].plot(
                time_axis,
                self.system_metrics["memory_mb"],
                linewidth=2,
                alpha=0.8,
                color="orange",
            )
            axes[0, 1].set_title("Memory Usage Over Time")
            axes[0, 1].set_ylabel("Memory (MB)")
            axes[0, 1].set_xlabel("Time (seconds)")
            axes[0, 1].grid(True, alpha=0.3)

            # Add chaos event markers
            for event in self.chaos_events:
                event_time = event["start_time"] - base_time
                axes[0, 1].axvline(
                    x=event_time, color="red", linestyle="--", alpha=0.7, linewidth=1
                )

        # 3. Performance Metrics Comparison
        if self.test_results:
            categories = []
            p99_values = []

            for test_name, results in self.test_results.items():
                if isinstance(results, dict) and "p99_ms" in results:
                    categories.append(test_name.replace("_", " ").title())
                    p99_values.append(results["p99_ms"])

            if categories:
                bars = axes[1, 0].bar(categories, p99_values, alpha=0.8)
                axes[1, 0].set_title("P99 Latency by Test Category")
                axes[1, 0].set_ylabel("Latency (ms)")
                axes[1, 0].tick_params(axis="x", rotation=45)

                # Color bars based on performance
                for i, (bar, value) in enumerate(zip(bars, p99_values)):
                    if value < 1.0:
                        bar.set_color("green")
                    elif value < 5.0:
                        bar.set_color("orange")
                    else:
                        bar.set_color("red")

        # 4. Chaos Events Timeline
        if self.chaos_events:
            event_times = [
                (event["start_time"] - base_time) for event in self.chaos_events
            ]
            event_durations = [event["duration"] for event in self.chaos_events]
            event_types = [
                event["type"].replace("_", " ").title() for event in self.chaos_events
            ]

            axes[1, 1].barh(event_types, event_durations, left=event_times, alpha=0.7)
            axes[1, 1].set_title("Chaos Events Timeline")
            axes[1, 1].set_xlabel("Time (seconds)")
            axes[1, 1].set_ylabel("Chaos Event Type")
            axes[1, 1].grid(True, alpha=0.3)

        # 5. Statistical Distribution Analysis
        analysis = report.get("analysis", {})
        if "cpu_stats" in analysis:
            cpu_stats = analysis["cpu_stats"]
            stat_names = ["Mean", "Median", "P95", "Max"]
            stat_values = [
                cpu_stats["mean"],
                cpu_stats["median"],
                cpu_stats["p95"],
                cpu_stats["max"],
            ]

            axes[2, 0].bar(stat_names, stat_values, alpha=0.8, color="skyblue")
            axes[2, 0].set_title("CPU Usage Statistics")
            axes[2, 0].set_ylabel("CPU %")
            axes[2, 0].grid(True, alpha=0.3)

            # Add value labels on bars
            for i, v in enumerate(stat_values):
                axes[2, 0].text(i, v + 0.5, ".1f", ha="center", va="bottom")

        # 6. System Health Overview
        health_metrics = []
        health_labels = []

        if "cpu_stats" in analysis:
            health_metrics.append(analysis["cpu_stats"]["stability_score"])
            health_labels.append("CPU Stability")

        if "memory_stats" in analysis:
            health_metrics.append(analysis["memory_stats"]["stability_score"])
            health_labels.append("Memory Stability")

        # Add test success rate
        total_tests = len(self.test_results)
        successful_tests = sum(
            1
            for r in self.test_results.values()
            if isinstance(r, dict) and r.get("success", False)
        )
        success_rate = (successful_tests / total_tests * 100) if total_tests > 0 else 0
        health_metrics.append(success_rate)
        health_labels.append("Test Success")

        if health_metrics:
            colors = [
                "green" if x > 80 else "orange" if x > 60 else "red"
                for x in health_metrics
            ]
            bars = axes[2, 1].bar(
                health_labels, health_metrics, alpha=0.8, color=colors
            )
            axes[2, 1].set_title("System Health Overview")
            axes[2, 1].set_ylabel("Health Score (%)")
            axes[2, 1].set_ylim(0, 100)
            axes[2, 1].grid(True, alpha=0.3)

            # Add value labels
            for bar, value in zip(bars, health_metrics):
                axes[2, 1].text(
                    bar.get_x() + bar.get_width() / 2,
                    bar.get_height() + 1,
                    ".1f",
                    ha="center",
                    va="bottom",
                )

        plt.tight_layout()

        # Save the comprehensive dashboard
        dashboard_file = (
            self.output_dir / f"performance_dashboard_{int(time.time())}.png"
        )
        plt.savefig(dashboard_file, dpi=300, bbox_inches="tight")
        print(f"📊 Performance dashboard saved to: {dashboard_file}")

        # Create additional specialized graphs
        self._create_specialized_graphs(report)

        plt.close("all")

    def _create_specialized_graphs(self, report: Dict[str, Any]):
        """Create specialized performance graphs."""

        # 1. Chaos Impact Analysis
        if self.chaos_events:
            self._create_chaos_impact_graph()

        # 2. Performance Distribution Analysis
        self._create_performance_distribution_graph()

        # 3. Time Series Analysis
        self._create_time_series_analysis()

        # 4. Comparative Performance Analysis
        self._create_comparative_analysis()

    def _create_chaos_impact_graph(self):
        """Create chaos impact visualization."""
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        fig.suptitle(
            "Chaos Engineering Impact Analysis", fontsize=14, fontweight="bold"
        )

        # CPU impact during chaos events
        if "cpu_percent" in self.system_metrics:
            base_time = min(self.system_metrics["timestamp"])
            time_axis = [(ts - base_time) for ts in self.system_metrics["timestamp"]]
            cpu_data = self.system_metrics["cpu_percent"]

            ax1.plot(time_axis, cpu_data, alpha=0.7, label="CPU Usage")

            # Highlight chaos periods
            for event in self.chaos_events:
                start_time = event["start_time"] - base_time
                end_time = event["end_time"] - base_time
                ax1.axvspan(
                    start_time,
                    end_time,
                    alpha=0.3,
                    color="red",
                    label=f"{event['type'].replace('_', ' ').title()}",
                )

            ax1.set_title("CPU Usage During Chaos Events")
            ax1.set_xlabel("Time (seconds)")
            ax1.set_ylabel("CPU %")
            ax1.grid(True, alpha=0.3)
            ax1.legend()

        # Memory impact during chaos events
        if "memory_mb" in self.system_metrics:
            mem_data = self.system_metrics["memory_mb"]
            ax2.plot(
                time_axis, mem_data, alpha=0.7, color="orange", label="Memory Usage"
            )

            # Highlight chaos periods
            for event in self.chaos_events:
                start_time = event["start_time"] - base_time
                end_time = event["end_time"] - base_time
                ax2.axvspan(start_time, end_time, alpha=0.3, color="red")

            ax2.set_title("Memory Usage During Chaos Events")
            ax2.set_xlabel("Time (seconds)")
            ax2.set_ylabel("Memory (MB)")
            ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        chaos_file = self.output_dir / f"chaos_impact_analysis_{int(time.time())}.png"
        plt.savefig(chaos_file, dpi=300, bbox_inches="tight")
        plt.close()

    def _create_performance_distribution_graph(self):
        """Create performance distribution analysis."""
        fig, axes = plt.subplots(2, 2, figsize=(12, 10))
        fig.suptitle(
            "Performance Distribution Analysis", fontsize=14, fontweight="bold"
        )

        # CPU usage distribution
        if "cpu_percent" in self.system_metrics:
            cpu_data = self.system_metrics["cpu_percent"]
            axes[0, 0].hist(cpu_data, bins=20, alpha=0.7, edgecolor="black")
            axes[0, 0].set_title("CPU Usage Distribution")
            axes[0, 0].set_xlabel("CPU %")
            axes[0, 0].set_ylabel("Frequency")
            axes[0, 0].grid(True, alpha=0.3)

        # Memory usage distribution
        if "memory_mb" in self.system_metrics:
            mem_data = self.system_metrics["memory_mb"]
            axes[0, 1].hist(
                mem_data, bins=20, alpha=0.7, color="orange", edgecolor="black"
            )
            axes[0, 1].set_title("Memory Usage Distribution")
            axes[0, 1].set_xlabel("Memory (MB)")
            axes[0, 1].set_ylabel("Frequency")
            axes[0, 1].grid(True, alpha=0.3)

        # Performance metrics box plot
        perf_data = []
        perf_labels = []

        for test_name, results in self.test_results.items():
            if isinstance(results, dict) and "p99_ms" in results:
                perf_data.append(results["p99_ms"])
                perf_labels.append(test_name.replace("_", " ").title())

        if perf_data:
            axes[1, 0].boxplot(perf_data, labels=perf_labels)
            axes[1, 0].set_title("Performance Metrics Distribution")
            axes[1, 0].set_ylabel("Latency (ms)")
            axes[1, 0].tick_params(axis="x", rotation=45)
            axes[1, 0].grid(True, alpha=0.3)

        # Chaos event frequency
        if self.chaos_events:
            event_types = [event["type"] for event in self.chaos_events]
            event_counts = pd.Series(event_types).value_counts()

            axes[1, 1].bar(
                event_counts.index.str.replace("_", " ").str.title(),
                event_counts.values,
                alpha=0.7,
                color="red",
            )
            axes[1, 1].set_title("Chaos Event Frequency")
            axes[1, 1].set_xlabel("Event Type")
            axes[1, 1].set_ylabel("Count")
            axes[1, 1].tick_params(axis="x", rotation=45)
            axes[1, 1].grid(True, alpha=0.3)

        plt.tight_layout()
        dist_file = self.output_dir / f"performance_distribution_{int(time.time())}.png"
        plt.savefig(dist_file, dpi=300, bbox_inches="tight")
        plt.close()

    def _create_time_series_analysis(self):
        """Create time series analysis graphs."""
        fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
        fig.suptitle("Time Series Performance Analysis", fontsize=14, fontweight="bold")

        if "timestamp" in self.system_metrics:
            base_time = min(self.system_metrics["timestamp"])
            time_axis = [(ts - base_time) for ts in self.system_metrics["timestamp"]]

            # Rolling averages
            if "cpu_percent" in self.system_metrics:
                cpu_data = pd.Series(self.system_metrics["cpu_percent"])
                cpu_rolling = cpu_data.rolling(window=10).mean()

                ax1.plot(
                    time_axis,
                    self.system_metrics["cpu_percent"],
                    alpha=0.5,
                    label="Raw CPU",
                )
                ax1.plot(
                    time_axis,
                    cpu_rolling,
                    linewidth=2,
                    label="10-point Rolling Average",
                )
                ax1.set_title("CPU Usage Time Series with Trend Analysis")
                ax1.set_xlabel("Time (seconds)")
                ax1.set_ylabel("CPU %")
                ax1.legend()
                ax1.grid(True, alpha=0.3)

            # Memory with trend
            if "memory_mb" in self.system_metrics:
                mem_data = pd.Series(self.system_metrics["memory_mb"])
                mem_rolling = mem_data.rolling(window=10).mean()

                ax2.plot(
                    time_axis,
                    self.system_metrics["memory_mb"],
                    alpha=0.5,
                    color="orange",
                    label="Raw Memory",
                )
                ax2.plot(
                    time_axis,
                    mem_rolling,
                    linewidth=2,
                    color="darkorange",
                    label="10-point Rolling Average",
                )
                ax2.set_title("Memory Usage Time Series with Trend Analysis")
                ax2.set_xlabel("Time (seconds)")
                ax2.set_ylabel("Memory (MB)")
                ax2.legend()
                ax2.grid(True, alpha=0.3)

        plt.tight_layout()
        ts_file = self.output_dir / f"time_series_analysis_{int(time.time())}.png"
        plt.savefig(ts_file, dpi=300, bbox_inches="tight")
        plt.close()

    def _create_comparative_analysis(self):
        """Create comparative performance analysis."""
        fig, ax = plt.subplots(1, 1, figsize=(10, 6))
        fig.suptitle(
            "Performance Requirements vs Actual Results", fontsize=14, fontweight="bold"
        )

        # Define requirements
        requirements = {
            "Binary Protocol": {"required": 1.0, "actual": None},
            "IPC Bridge": {"required": 5.0, "actual": None},
            "Motion Control": {"required": 20.0, "actual": None},
            "End-to-End Pipeline": {"required": 5.0, "actual": None},
        }

        # Fill in actual results
        for test_name, results in self.test_results.items():
            if isinstance(results, dict) and "p99_ms" in results:
                if "binary" in test_name.lower():
                    requirements["Binary Protocol"]["actual"] = results["p99_ms"]
                elif "ipc" in test_name.lower():
                    requirements["IPC Bridge"]["actual"] = results["p99_ms"]
                elif "motion" in test_name.lower():
                    requirements["Motion Control"]["actual"] = results["p99_ms"]
                elif (
                    "end_to_end" in test_name.lower() or "pipeline" in test_name.lower()
                ):
                    requirements["End-to-End Pipeline"]["actual"] = results["p99_ms"]

        # Create comparison chart
        categories = list(requirements.keys())
        required_values = [req["required"] for req in requirements.values()]
        actual_values = [
            req["actual"] if req["actual"] is not None else 0
            for req in requirements.values()
        ]

        x = np.arange(len(categories))
        width = 0.35

        bars1 = ax.bar(
            x - width / 2,
            required_values,
            width,
            label="Required (ms)",
            alpha=0.8,
            color="red",
        )
        bars2 = ax.bar(
            x + width / 2,
            actual_values,
            width,
            label="Actual (ms)",
            alpha=0.8,
            color="green",
        )

        ax.set_xlabel("Performance Category")
        ax.set_ylabel("P99 Latency (ms)")
        ax.set_title("Requirements vs Actual Performance")
        ax.set_xticks(x)
        ax.set_xticklabels(categories, rotation=45)
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Add value labels
        for bar in bars1:
            height = bar.get_height()
            ax.text(
                bar.get_x() + bar.get_width() / 2.0,
                height + 0.1,
                ".1f",
                ha="center",
                va="bottom",
            )

        for bar in bars2:
            height = bar.get_height()
            if height > 0:
                ax.text(
                    bar.get_x() + bar.get_width() / 2.0,
                    height + 0.1,
                    ".1f",
                    ha="center",
                    va="bottom",
                )

        plt.tight_layout()
        comp_file = self.output_dir / f"requirements_comparison_{int(time.time())}.png"
        plt.savefig(comp_file, dpi=300, bbox_inches="tight")
        plt.close()


def main():
    """Main entry point for comprehensive performance testing."""
    parser = argparse.ArgumentParser(
        description="URC 2026 Comprehensive Performance Testing"
    )
    parser.add_argument(
        "--duration",
        type=int,
        default=300,
        help="Test duration in seconds (default: 300)",
    )
    parser.add_argument(
        "--chaos-level",
        type=float,
        default=0.5,
        help="Chaos injection level 0.0-1.0 (default: 0.5)",
    )
    parser.add_argument(
        "--output-dir",
        type=str,
        default="perf_results",
        help="Output directory for results and graphs (default: perf_results)",
    )
    parser.add_argument(
        "--quick", action="store_true", help="Run quick test suite (reduced duration)"
    )
    parser.add_argument(
        "--no-chaos", action="store_true", help="Disable chaos engineering tests"
    )

    args = parser.parse_args()

    # Adjust parameters for quick mode
    if args.quick:
        args.duration = 60
        args.chaos_level = 0.2

    if args.no_chaos:
        args.chaos_level = 0.0

    print("🎯 URC 2026 Comprehensive Performance Testing")
    print("=" * 50)
    print(f"Duration: {args.duration}s")
    print(f"Chaos Level: {args.chaos_level}")
    print(f"Output Directory: {args.output_dir}")
    print(f"Quick Mode: {args.quick}")
    print(f"Chaos Disabled: {args.no_chaos}")
    print()

    # Run comprehensive testing
    tester = ComprehensivePerformanceTester(
        duration_seconds=args.duration,
        chaos_level=args.chaos_level,
        output_dir=args.output_dir,
    )

    start_time = time.time()
    report = tester.run_comprehensive_tests()
    total_time = time.time() - start_time

    # Print summary
    print("\n" + "=" * 50)
    print("🧪 TESTING COMPLETE")
    print("=" * 50)

    analysis = report.get("analysis", {})
    if "cpu_stats" in analysis:
        cpu_stats = analysis["cpu_stats"]
        print("📊 CPU Performance:")
        print(
            f"  Mean: {cpu_stats['mean']:.1f}%, P95: {cpu_stats['p95']:.1f}%, Max: {cpu_stats['max']:.1f}%"
        )
    if "memory_stats" in analysis:
        mem_stats = analysis["memory_stats"]
        print("🧠 Memory Performance:")
        print(
            f"  Mean: {mem_stats['mean_mb']:.1f}MB, P95: {mem_stats['p95_mb']:.1f}MB, Max: {mem_stats['max_mb']:.1f}MB"
        )
    if "chaos_impact" in analysis:
        chaos = analysis["chaos_impact"]
        print("🎭 Chaos Impact:")
        print(f"  Events Analyzed: {chaos['events_analyzed']}")
        print(f"  CPU Spikes: {chaos['cpu_spikes_detected']}")
        print(f"  Memory Spikes: {chaos['memory_spikes_detected']}")

    print(f"\n⏱️ Total Test Time: {total_time:.1f}s")
    print(f"📁 Results saved to: {args.output_dir}/")


if __name__ == "__main__":
    main()
