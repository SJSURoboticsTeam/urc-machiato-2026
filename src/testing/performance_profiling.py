#!/usr/bin/env python3
"""
Performance Profiling Framework - Latency and Throughput Testing

Comprehensive performance testing framework for URC 2026 rover systems.
Validates critical performance requirements like <20ms motion control latency.

Features:
- Motion control latency profiling (p50, p95, p99)
- Sensor fusion timing validation
- Network latency benchmarking
- Memory usage monitoring
- CPU utilization tracking
- Automated regression detection

Author: URC 2026 Performance Validation Team
"""

import time
import threading
import psutil
import statistics
from typing import Dict, List, Any, Optional, Callable, Tuple
from dataclasses import dataclass, field
from enum import Enum
import logging
import json
import os
from collections import deque

logger = logging.getLogger(__name__)


class PerformanceMetric(Enum):
    """Performance metrics to track."""

    MOTION_CONTROL_LATENCY = "motion_control_latency"
    SENSOR_FUSION_LATENCY = "sensor_fusion_latency"
    NETWORK_LATENCY = "network_latency"
    SERIALIZATION_LATENCY = "serialization_latency"
    MESSAGE_PROCESSING_LATENCY = "message_processing_latency"
    CPU_USAGE = "cpu_usage"
    MEMORY_USAGE = "memory_usage"
    DISK_IO = "disk_io"
    NETWORK_IO = "network_io"


@dataclass
class LatencyMeasurement:
    """Individual latency measurement."""

    timestamp: float
    value_ms: float
    operation: str
    metadata: Dict[str, Any] = field(default_factory=dict)


@dataclass
class PerformanceProfile:
    """Performance profile for an operation."""

    operation_name: str
    measurements: List[LatencyMeasurement] = field(default_factory=list)
    max_samples: int = 10000  # Keep last 10k samples

    def add_measurement(self, value_ms: float, metadata: Dict[str, Any] = None):
        """Add a latency measurement."""
        measurement = LatencyMeasurement(
            timestamp=time.time(),
            value_ms=value_ms,
            operation=self.operation_name,
            metadata=metadata or {},
        )

        self.measurements.append(measurement)

        # Maintain rolling window
        if len(self.measurements) > self.max_samples:
            self.measurements.pop(0)

    def get_statistics(self) -> Dict[str, Any]:
        """Calculate latency statistics."""
        if not self.measurements:
            return {"count": 0, "message": "No measurements"}

        values = [m.value_ms for m in self.measurements]
        values.sort()

        count = len(values)
        mean = statistics.mean(values)
        median = statistics.median(values)

        # Percentiles
        p50 = values[int(count * 0.5)]
        p90 = values[int(count * 0.9)]
        p95 = values[int(count * 0.95)]
        p99 = values[int(count * 0.99)]
        p999 = values[int(count * 0.999)] if count > 1000 else values[-1]

        # Additional metrics
        min_val = min(values)
        max_val = max(values)
        std_dev = statistics.stdev(values) if count > 1 else 0

        return {
            "count": count,
            "mean_ms": mean,
            "median_ms": median,
            "p50_ms": p50,
            "p90_ms": p90,
            "p95_ms": p95,
            "p99_ms": p99,
            "p999_ms": p999,
            "min_ms": min_val,
            "max_ms": max_val,
            "std_dev_ms": std_dev,
            "last_measurement_time": (
                self.measurements[-1].timestamp if self.measurements else None
            ),
        }


@dataclass
class PerformanceRequirement:
    """Performance requirement specification."""

    metric: PerformanceMetric
    operation: str
    max_latency_ms: float
    percentile: str = "p99"  # p50, p95, p99, p999, max
    description: str = ""
    enabled: bool = True


class PerformanceProfiler:
    """
    Comprehensive performance profiling system.

    Tracks latency, throughput, and resource usage across the system.
    Validates performance against requirements.
    """

    # Default performance requirements (based on critical review)
    DEFAULT_REQUIREMENTS = [
        PerformanceRequirement(
            PerformanceMetric.MOTION_CONTROL_LATENCY,
            "motion_control_loop",
            max_latency_ms=20.0,
            percentile="p99",
            description="Motion control must complete within 20ms (50Hz loop)",
        ),
        PerformanceRequirement(
            PerformanceMetric.SENSOR_FUSION_LATENCY,
            "sensor_fusion_update",
            max_latency_ms=50.0,
            percentile="p95",
            description="Sensor fusion within 50ms for 20Hz update rate",
        ),
        PerformanceRequirement(
            PerformanceMetric.NETWORK_LATENCY,
            "websocket_message_roundtrip",
            max_latency_ms=100.0,
            percentile="p95",
            description="WebSocket message processing within 100ms",
        ),
        PerformanceRequirement(
            PerformanceMetric.SERIALIZATION_LATENCY,
            "binary_sensor_encoding",
            max_latency_ms=5.0,
            percentile="p99",
            description="Binary serialization under 5ms",
        ),
        PerformanceRequirement(
            PerformanceMetric.CPU_USAGE,
            "system_cpu_usage",
            max_latency_ms=75.0,  # % CPU
            percentile="p95",
            description="CPU usage under 75% sustained",
        ),
        PerformanceRequirement(
            PerformanceMetric.MEMORY_USAGE,
            "system_memory_usage",
            max_latency_ms=700.0,  # MB
            percentile="max",
            description="Memory usage under 700MB",
        ),
    ]

    def __init__(self, requirements: List[PerformanceRequirement] = None):
        self.requirements = requirements or self.DEFAULT_REQUIREMENTS.copy()
        self.profiles: Dict[str, PerformanceProfile] = {}
        self.system_monitoring_active = False
        self.system_monitor_thread: Optional[threading.Thread] = None

        # Performance history
        self.violations: List[Dict[str, Any]] = []
        self.baseline_stats: Dict[str, Dict[str, Any]] = {}

        self.lock = threading.RLock()

        # Initialize profiles for all requirements
        for req in self.requirements:
            if req.enabled:
                self.profiles[req.operation] = PerformanceProfile(req.operation)

        logger.info(
            f"PerformanceProfiler initialized with {len(self.requirements)} requirements"
        )

    def measure_latency(self, operation: str, func: Callable, *args, **kwargs) -> Any:
        """
        Measure latency of a function call.

        Args:
            operation: Name of the operation being measured
            func: Function to execute and measure
            *args, **kwargs: Arguments to pass to function

        Returns:
            Function result
        """
        start_time = time.perf_counter()

        try:
            result = func(*args, **kwargs)
            latency_ms = (time.perf_counter() - start_time) * 1000

            self.record_measurement(operation, latency_ms)
            return result

        except Exception as e:
            latency_ms = (time.perf_counter() - start_time) * 1000
            self.record_measurement(operation, latency_ms, {"exception": str(e)})
            raise

    def record_measurement(
        self, operation: str, latency_ms: float, metadata: Dict[str, Any] = None
    ):
        """
        Record a latency measurement.

        Args:
            operation: Operation name
            latency_ms: Latency in milliseconds
            metadata: Additional measurement metadata
        """
        with self.lock:
            if operation not in self.profiles:
                self.profiles[operation] = PerformanceProfile(operation)

            self.profiles[operation].add_measurement(latency_ms, metadata)

            # Check for violations
            self._check_violations(operation, latency_ms)

    def start_system_monitoring(self, interval_seconds: float = 1.0):
        """Start system resource monitoring."""
        with self.lock:
            if self.system_monitoring_active:
                return

            self.system_monitoring_active = True
            self.system_monitor_thread = threading.Thread(
                target=self._system_monitor_loop,
                args=(interval_seconds,),
                name="performance_monitor",
                daemon=True,
            )
            self.system_monitor_thread.start()
            logger.info(
                f"System performance monitoring started (interval: {interval_seconds}s)"
            )

    def stop_system_monitoring(self):
        """Stop system resource monitoring."""
        with self.lock:
            self.system_monitoring_active = False
            if self.system_monitor_thread:
                self.system_monitor_thread.join(timeout=5.0)
            logger.info("System performance monitoring stopped")

    def _system_monitor_loop(self, interval_seconds: float):
        """System monitoring loop."""
        while self.system_monitoring_active:
            try:
                # CPU usage
                cpu_percent = psutil.cpu_percent(interval=None)
                self.record_measurement("system_cpu_usage", cpu_percent)

                # Memory usage
                memory = psutil.virtual_memory()
                memory_mb = memory.used / (1024 * 1024)
                self.record_measurement("system_memory_usage", memory_mb)

                # Disk I/O (simplified)
                disk_io = psutil.disk_io_counters()
                if disk_io:
                    read_mb = disk_io.read_bytes / (1024 * 1024)
                    write_mb = disk_io.write_bytes / (1024 * 1024)
                    self.record_measurement("system_disk_read_mb", read_mb)
                    self.record_measurement("system_disk_write_mb", write_mb)

                # Network I/O
                net_io = psutil.net_io_counters()
                if net_io:
                    sent_mb = net_io.bytes_sent / (1024 * 1024)
                    recv_mb = net_io.bytes_recv / (1024 * 1024)
                    self.record_measurement("system_network_sent_mb", sent_mb)
                    self.record_measurement("system_network_recv_mb", recv_mb)

            except Exception as e:
                logger.error(f"System monitoring error: {e}")

            time.sleep(interval_seconds)

    def _check_violations(self, operation: str, latency_ms: float):
        """Check if measurement violates requirements."""
        for req in self.requirements:
            if req.operation == operation and req.enabled:
                # Get current statistics
                profile = self.profiles.get(operation)
                if not profile:
                    continue

                stats = profile.get_statistics()
                if "count" not in stats or stats["count"] == 0:
                    continue

                # Check the specified percentile
                if req.percentile not in stats:
                    continue

                threshold_value = stats[req.percentile + "_ms"]

                if threshold_value > req.max_latency_ms:
                    violation = {
                        "timestamp": time.time(),
                        "operation": operation,
                        "requirement": req.description,
                        "threshold_ms": req.max_latency_ms,
                        "actual_ms": threshold_value,
                        "percentile": req.percentile,
                        "violation_amount_ms": threshold_value - req.max_latency_ms,
                        "sample_count": stats["count"],
                    }

                    self.violations.append(violation)

                    logger.warning(
                        f"PERFORMANCE VIOLATION: {operation} "
                        f"{req.percentile}={threshold_value:.1f}ms > "
                        f"required {req.max_latency_ms:.1f}ms "
                        f"(+{violation['violation_amount_ms']:.1f}ms)"
                    )

    def get_performance_report(self) -> Dict[str, Any]:
        """Generate comprehensive performance report."""
        with self.lock:
            report = {
                "timestamp": time.time(),
                "requirements": [],
                "profiles": {},
                "violations": self.violations[-50:],  # Last 50 violations
                "summary": {},
            }

            # Requirements status
            for req in self.requirements:
                if not req.enabled:
                    continue

                profile = self.profiles.get(req.operation)
                if not profile:
                    status = "no_data"
                    compliant = False
                else:
                    stats = profile.get_statistics()
                    if stats.get("count", 0) == 0:
                        status = "no_data"
                        compliant = False
                    else:
                        percentile_key = req.percentile + "_ms"
                        actual_value = stats.get(percentile_key, float("inf"))
                        compliant = actual_value <= req.max_latency_ms
                        status = "compliant" if compliant else "violating"

                report["requirements"].append(
                    {
                        "operation": req.operation,
                        "description": req.description,
                        "required_max_ms": req.max_latency_ms,
                        "percentile": req.percentile,
                        "status": status,
                        "compliant": compliant,
                    }
                )

            # Profile statistics
            for operation, profile in self.profiles.items():
                report["profiles"][operation] = profile.get_statistics()

            # Summary statistics
            total_measurements = sum(
                len(p.measurements) for p in self.profiles.values()
            )
            compliant_requirements = sum(
                1 for r in report["requirements"] if r["compliant"]
            )
            total_requirements = len(
                [r for r in report["requirements"] if r["status"] != "no_data"]
            )

            report["summary"] = {
                "total_measurements": total_measurements,
                "total_violations": len(self.violations),
                "compliant_requirements": compliant_requirements,
                "total_requirements": total_requirements,
                "compliance_rate_percent": (
                    compliant_requirements / max(total_requirements, 1)
                )
                * 100,
                "monitoring_active": self.system_monitoring_active,
            }

            return report

    def establish_baseline(self, measurement_period_seconds: float = 60.0):
        """
        Establish performance baseline by measuring for a period.

        Args:
            measurement_period_seconds: How long to measure baseline
        """
        logger.info(
            f"Establishing performance baseline for {measurement_period_seconds}s..."
        )

        # Start monitoring if not active
        was_monitoring = self.system_monitoring_active
        if not was_monitoring:
            self.start_system_monitoring()

        # Wait for measurements
        time.sleep(measurement_period_seconds)

        # Capture current stats as baseline
        report = self.get_performance_report()
        self.baseline_stats = report["profiles"].copy()

        logger.info("Performance baseline established")
        for operation, stats in self.baseline_stats.items():
            if stats.get("count", 0) > 0:
                logger.info(
                    f"  {operation}: p50={stats.get('p50_ms', 0):.1f}ms, "
                    f"p95={stats.get('p95_ms', 0):.1f}ms, "
                    f"p99={stats.get('p99_ms', 0):.1f}ms"
                )

        # Stop monitoring if we started it
        if not was_monitoring:
            self.stop_system_monitoring()

    def detect_regression(self) -> List[Dict[str, Any]]:
        """
        Detect performance regressions compared to baseline.

        Returns:
            List of detected regressions
        """
        if not self.baseline_stats:
            return []

        regressions = []
        current_report = self.get_performance_report()

        for operation, current_stats in current_report["profiles"].items():
            if operation not in self.baseline_stats:
                continue

            baseline_stats = self.baseline_stats[operation]

            # Check for significant regression in key percentiles
            for percentile in ["p50_ms", "p95_ms", "p99_ms"]:
                if percentile in current_stats and percentile in baseline_stats:
                    current_value = current_stats[percentile]
                    baseline_value = baseline_stats[percentile]

                    # 50% increase or more is a regression
                    if current_value > baseline_value * 1.5:
                        regressions.append(
                            {
                                "operation": operation,
                                "percentile": percentile,
                                "baseline_value_ms": baseline_value,
                                "current_value_ms": current_value,
                                "regression_factor": current_value / baseline_value,
                                "timestamp": time.time(),
                            }
                        )

        return regressions

    def export_report(self, filename: str = None) -> str:
        """Export performance report to file."""
        if not filename:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"performance_report_{timestamp}.json"

        report = self.get_performance_report()
        report["regressions"] = self.detect_regression()

        with open(filename, "w") as f:
            json.dump(report, f, indent=2, default=str)

        logger.info(f"Performance report exported to {filename}")
        return filename

    def clear_measurements(self):
        """Clear all measurements and violations."""
        with self.lock:
            for profile in self.profiles.values():
                profile.measurements.clear()
            self.violations.clear()
            logger.info("All performance measurements cleared")


# Global instance
_performance_profiler_instance: Optional[PerformanceProfiler] = None
_performance_profiler_lock = threading.Lock()


def get_performance_profiler() -> PerformanceProfiler:
    """Get global performance profiler instance."""
    global _performance_profiler_instance

    if _performance_profiler_instance is None:
        with _performance_profiler_lock:
            if _performance_profiler_instance is None:
                _performance_profiler_instance = PerformanceProfiler()

    return _performance_profiler_instance


# Convenience functions
def measure_latency(operation: str, func: Callable, *args, **kwargs) -> Any:
    """Convenience function to measure function latency."""
    profiler = get_performance_profiler()
    return profiler.measure_latency(operation, func, *args, **kwargs)


def record_measurement(
    operation: str, latency_ms: float, metadata: Dict[str, Any] = None
):
    """Convenience function to record a measurement."""
    profiler = get_performance_profiler()
    profiler.record_measurement(operation, latency_ms, metadata)


def get_performance_report() -> Dict[str, Any]:
    """Get current performance report."""
    profiler = get_performance_profiler()
    return profiler.get_performance_report()


def start_performance_monitoring(interval_seconds: float = 1.0):
    """Start system performance monitoring."""
    profiler = get_performance_profiler()
    profiler.start_system_monitoring(interval_seconds)


def stop_performance_monitoring():
    """Stop system performance monitoring."""
    profiler = get_performance_profiler()
    profiler.stop_system_monitoring()


def check_performance_violations() -> List[Dict[str, Any]]:
    """Check for current performance violations."""
    report = get_performance_report()
    return [req for req in report["requirements"] if not req["compliant"]]


# Motion control specific benchmarking
class MotionControlBenchmark:
    """Specialized benchmark for motion control performance."""

    @staticmethod
    def benchmark_motion_control_loop(
        iterations: int = 1000, target_frequency_hz: float = 50.0
    ) -> Dict[str, Any]:
        """
        Benchmark motion control loop performance.

        Args:
            iterations: Number of loop iterations to test
            target_frequency_hz: Target control frequency

        Returns:
            Benchmark results
        """
        target_period_ms = 1000.0 / target_frequency_hz
        profiler = get_performance_profiler()

        latencies = []
        violations = 0

        logger.info(
            f"Benchmarking motion control loop: {iterations} iterations at {target_frequency_hz}Hz"
        )

        for i in range(iterations):
            # Simulate motion control loop
            start_time = time.perf_counter()

            # Simulate sensor reading (1-2ms)
            time.sleep(0.001 + (i % 10) * 0.0001)  # Variable delay

            # Simulate computation (2-5ms)
            for _ in range(1000 + (i % 500)):  # Variable computation
                pass

            # Simulate motor command (0.5-1ms)
            time.sleep(0.0005)

            end_time = time.perf_counter()
            latency_ms = (end_time - start_time) * 1000

            latencies.append(latency_ms)

            # Check deadline
            if latency_ms > target_period_ms:
                violations += 1

            # Record measurement
            profiler.record_measurement("motion_control_loop", latency_ms)

        latencies.sort()
        p99_latency = latencies[int(len(latencies) * 0.99)]

        result = {
            "iterations": iterations,
            "target_period_ms": target_period_ms,
            "p99_latency_ms": p99_latency,
            "deadline_violations": violations,
            "violation_rate_percent": (violations / iterations) * 100,
            "deadline_met": p99_latency <= target_period_ms,
            "margin_ms": target_period_ms - p99_latency,
        }

        logger.info(
            f"Motion control benchmark complete: "
            f"p99={p99_latency:.1f}ms, target={target_period_ms:.1f}ms, "
            f"violations={violations}/{iterations} ({result['violation_rate_percent']:.1f}%)"
        )

        return result
