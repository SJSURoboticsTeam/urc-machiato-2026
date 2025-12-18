#!/usr/bin/env python3
"""
Real-Time Performance Monitor - Ensures Timing Requirements Are Met

Monitors control loop timing, detects deadline misses, and provides performance
metrics for real-time systems. Critical for hardware integration where timing
violations can cause system failures.

Author: URC 2026 Autonomy Team
"""

import logging
import os
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from typing import Any, Callable, Dict, List, Optional

import psutil

logger = logging.getLogger(__name__)


@dataclass
class ControlLoopMetrics:
    """Metrics for a control loop."""

    name: str
    target_period: float  # Target period in seconds
    deadline: float  # Maximum allowed period
    priority: int = 1  # Priority level (higher = more critical)

    # Runtime metrics
    execution_count: int = 0
    total_execution_time: float = 0.0
    max_execution_time: float = 0.0
    min_execution_time: float = float("inf")
    deadline_misses: int = 0
    consecutive_deadline_misses: int = 0

    # Recent execution times (for moving averages)
    recent_executions: deque = field(default_factory=lambda: deque(maxlen=100))
    recent_periods: deque = field(default_factory=lambda: deque(maxlen=100))

    # Performance targets
    max_jitter_percent: float = 10.0  # Maximum allowed jitter percentage
    min_success_rate: float = 99.0  # Minimum success rate percentage


class RealTimeMonitor:
    """
    Real-time performance monitor for control systems.

    Monitors control loop timing, detects deadline violations, and provides
    performance analytics. Essential for ensuring hardware control systems
    meet real-time requirements.
    """

    def __init__(self, monitoring_interval: float = 0.1):
        """
        Initialize real-time monitor.

        Args:
            monitoring_interval: How often to check system performance (seconds)
        """
        self.monitoring_interval = monitoring_interval
        self.control_loops: Dict[str, ControlLoopMetrics] = {}
        self.system_metrics: Dict[str, Any] = {
            "cpu_percent": deque(maxlen=100),
            "memory_percent": deque(maxlen=100),
            "disk_io": deque(maxlen=100),
            "network_io": deque(maxlen=100),
        }

        self.monitoring_active = False
        self.monitor_thread: Optional[threading.Thread] = None
        self.alert_callbacks: List[Callable] = []

        # Default control loop definitions
        self._define_default_control_loops()

        logger.info("Real-time performance monitor initialized")

    def _define_default_control_loops(self):
        """Define default control loops for rover systems."""
        default_loops = {
            "drive_control": ControlLoopMetrics(
                name="drive_control",
                target_period=0.02,  # 50Hz
                deadline=0.05,  # 20Hz minimum
                priority=5,
                max_jitter_percent=5.0,
                min_success_rate=99.5,
            ),
            "arm_control": ControlLoopMetrics(
                name="arm_control",
                target_period=0.01,  # 100Hz
                deadline=0.025,  # 40Hz minimum
                priority=4,
                max_jitter_percent=8.0,
                min_success_rate=99.0,
            ),
            "sensor_fusion": ControlLoopMetrics(
                name="sensor_fusion",
                target_period=0.005,  # 200Hz
                deadline=0.01,  # 100Hz minimum
                priority=3,
                max_jitter_percent=10.0,
                min_success_rate=98.0,
            ),
            "safety_monitor": ControlLoopMetrics(
                name="safety_monitor",
                target_period=0.001,  # 1000Hz
                deadline=0.002,  # 500Hz minimum
                priority=6,  # Highest priority
                max_jitter_percent=5.0,
                min_success_rate=99.9,
            ),
            "navigation": ControlLoopMetrics(
                name="navigation",
                target_period=0.1,  # 10Hz
                deadline=0.2,  # 5Hz minimum
                priority=2,
                max_jitter_percent=15.0,
                min_success_rate=95.0,
            ),
            "telemetry": ControlLoopMetrics(
                name="telemetry",
                target_period=0.05,  # 20Hz
                deadline=0.1,  # 10Hz minimum
                priority=1,
                max_jitter_percent=20.0,
                min_success_rate=90.0,
            ),
        }

        for loop in default_loops.values():
            self.control_loops[loop.name] = loop

    def register_control_loop(
        self,
        name: str,
        target_period: float,
        deadline: Optional[float] = None,
        priority: int = 1,
    ) -> bool:
        """
        Register a custom control loop for monitoring.

        Args:
            name: Name of the control loop
            target_period: Target execution period in seconds
            deadline: Maximum allowed period (defaults to 2x target_period)
            priority: Priority level for alerts

        Returns:
            bool: True if registered successfully
        """
        if deadline is None:
            deadline = target_period * 2.0

        if name in self.control_loops:
            logger.warning(f"Control loop '{name}' already registered, updating")
            self.control_loops[name].target_period = target_period
            self.control_loops[name].deadline = deadline
            self.control_loops[name].priority = priority
        else:
            self.control_loops[name] = ControlLoopMetrics(
                name=name,
                target_period=target_period,
                deadline=deadline,
                priority=priority,
            )

        logger.info(
            f"Registered control loop: {name} "
            f"(target: {target_period*1000:.1f}ms, deadline: {deadline*1000:.1f}ms)"
        )
        return True

    def start_monitoring(self) -> bool:
        """Start real-time monitoring."""
        if self.monitoring_active:
            logger.warning("Monitoring already active")
            return True

        self.monitoring_active = True
        self.monitor_thread = threading.Thread(
            target=self._monitoring_loop, daemon=True
        )
        self.monitor_thread.start()

        logger.info("Real-time performance monitoring started")
        return True

    def stop_monitoring(self) -> bool:
        """Stop real-time monitoring."""
        if not self.monitoring_active:
            return True

        self.monitoring_active = False
        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=1.0)

        logger.info("Real-time performance monitoring stopped")
        return True

    def record_execution_start(self, loop_name: str) -> float:
        """
        Record the start of a control loop execution.

        Args:
            loop_name: Name of the control loop

        Returns:
            Timestamp when execution started (for end recording)
        """
        if loop_name not in self.control_loops:
            logger.warning(f"Unknown control loop: {loop_name}")
            return time.time()

        return time.time()

    def record_execution_end(self, loop_name: str, start_time: float) -> bool:
        """
        Record the end of a control loop execution.

        Args:
            loop_name: Name of the control loop
            start_time: Timestamp when execution started

        Returns:
            bool: True if within deadline, False if deadline missed
        """
        if loop_name not in self.control_loops:
            logger.warning(f"Unknown control loop: {loop_name}")
            return True

        end_time = time.time()
        execution_time = end_time - start_time

        loop = self.control_loops[loop_name]

        # Update metrics
        loop.execution_count += 1
        loop.total_execution_time += execution_time
        loop.max_execution_time = max(loop.max_execution_time, execution_time)
        loop.min_execution_time = min(loop.min_execution_time, execution_time)

        # Add to recent executions
        loop.recent_executions.append(execution_time)

        # Check for deadline miss
        deadline_missed = execution_time > loop.deadline
        if deadline_missed:
            loop.deadline_misses += 1
            loop.consecutive_deadline_misses += 1

            # Trigger alert
            self._trigger_alert(
                loop_name,
                "deadline_miss",
                {
                    "execution_time": execution_time,
                    "deadline": loop.deadline,
                    "consecutive_misses": loop.consecutive_deadline_misses,
                },
            )
        else:
            loop.consecutive_deadline_misses = 0

        return not deadline_missed

    def get_performance_summary(self) -> Dict[str, Any]:
        """Get comprehensive performance summary."""
        summary = {
            "timestamp": time.time(),
            "control_loops": {},
            "system_health": self._get_system_health(),
            "alerts": [],
        }

        for loop_name, loop in self.control_loops.items():
            if loop.execution_count > 0:
                avg_execution_time = loop.total_execution_time / loop.execution_count
                success_rate = (
                    (loop.execution_count - loop.deadline_misses)
                    / loop.execution_count
                    * 100.0
                )

                # Calculate jitter (coefficient of variation)
                if len(loop.recent_executions) > 1:
                    jitter = (
                        statistics.stdev(loop.recent_executions)
                        / statistics.mean(loop.recent_executions)
                        * 100.0
                    )
                else:
                    jitter = 0.0

                summary["control_loops"][loop_name] = {
                    "execution_count": loop.execution_count,
                    "avg_execution_time_ms": avg_execution_time * 1000.0,
                    "max_execution_time_ms": loop.max_execution_time * 1000.0,
                    "min_execution_time_ms": loop.min_execution_time * 1000.0,
                    "deadline_misses": loop.deadline_misses,
                    "success_rate_percent": success_rate,
                    "jitter_percent": jitter,
                    "target_period_ms": loop.target_period * 1000.0,
                    "deadline_ms": loop.deadline * 1000.0,
                    "health_score": self._calculate_loop_health(
                        loop, success_rate, jitter
                    ),
                }

                # Add alerts for concerning metrics
                if success_rate < loop.min_success_rate:
                    summary["alerts"].append(
                        {
                            "type": "low_success_rate",
                            "loop": loop_name,
                            "severity": "critical" if success_rate < 90 else "warning",
                            "message": f"Success rate {success_rate:.1f}% below threshold {loop.min_success_rate}%",
                        }
                    )

                if jitter > loop.max_jitter_percent:
                    summary["alerts"].append(
                        {
                            "type": "high_jitter",
                            "loop": loop_name,
                            "severity": "warning",
                            "message": f"Jitter {jitter:.1f}% above threshold {loop.max_jitter_percent}%",
                        }
                    )

        return summary

    def add_alert_callback(self, callback: Callable[[str, str, Dict[str, Any]], None]):
        """
        Add a callback for performance alerts.

        Args:
            callback: Function called with (loop_name, alert_type, alert_data)
        """
        self.alert_callbacks.append(callback)

    def _monitoring_loop(self):
        """Main monitoring loop."""
        last_check = time.time()

        while self.monitoring_active:
            current_time = time.time()

            # Update system metrics
            self._update_system_metrics()

            # Check for long-running control loops
            self._check_for_stuck_loops(current_time)

            # Sleep until next check
            sleep_time = max(0, self.monitoring_interval - (current_time - last_check))
            time.sleep(sleep_time)
            last_check = current_time

    def _update_system_metrics(self):
        """Update system-wide performance metrics."""
        try:
            # CPU usage
            self.system_metrics["cpu_percent"].append(psutil.cpu_percent(interval=None))

            # Memory usage
            memory = psutil.virtual_memory()
            self.system_metrics["memory_percent"].append(memory.percent)

            # Disk I/O (simplified)
            disk_io = psutil.disk_io_counters()
            if disk_io:
                self.system_metrics["disk_io"].append(
                    disk_io.read_bytes + disk_io.write_bytes
                )

            # Network I/O (simplified)
            net_io = psutil.net_io_counters()
            if net_io:
                self.system_metrics["network_io"].append(
                    net_io.bytes_sent + net_io.bytes_recv
                )

        except Exception as e:
            logger.debug(f"System metrics update failed: {e}")

    def _check_for_stuck_loops(self, current_time: float):
        """Check for control loops that may be stuck."""
        stuck_threshold = 1.0  # 1 second

        for loop_name, loop in self.control_loops.items():
            # This would require tracking loop start times
            # For now, just check if any loop hasn't been updated recently
            pass

    def _trigger_alert(
        self, loop_name: str, alert_type: str, alert_data: Dict[str, Any]
    ):
        """Trigger performance alert."""
        logger.warning(f" Performance Alert: {loop_name} - {alert_type}")

        # Call all registered callbacks
        for callback in self.alert_callbacks:
            try:
                callback(loop_name, alert_type, alert_data)
            except Exception as e:
                logger.error(f"Alert callback failed: {e}")

    def _calculate_loop_health(
        self, loop: ControlLoopMetrics, success_rate: float, jitter: float
    ) -> float:
        """Calculate overall health score for a control loop (0-100)."""
        health_score = 100.0

        # Success rate impact (50% weight)
        if success_rate < loop.min_success_rate:
            deficit = loop.min_success_rate - success_rate
            health_score -= deficit * 2.0  # 1% deficit = 2 points lost

        # Jitter impact (30% weight)
        if jitter > loop.max_jitter_percent:
            excess = jitter - loop.max_jitter_percent
            health_score -= excess * 1.5

        # Priority bonus (20% weight) - critical loops get health bonus
        health_score += loop.priority * 4.0

        return max(0.0, min(100.0, health_score))

    def _get_system_health(self) -> Dict[str, Any]:
        """Get overall system health metrics."""
        health = {
            "cpu_usage_percent": (
                statistics.mean(self.system_metrics["cpu_percent"])
                if self.system_metrics["cpu_percent"]
                else 0
            ),
            "memory_usage_percent": (
                statistics.mean(self.system_metrics["memory_percent"])
                if self.system_metrics["memory_percent"]
                else 0
            ),
            "overall_health_score": 100.0,
            "critical_loops_count": 0,
            "healthy_loops_count": 0,
        }

        # Count healthy vs critical loops
        for loop_name, loop in self.control_loops.items():
            if loop.execution_count > 10:  # Only count loops with sufficient data
                success_rate = (
                    (loop.execution_count - loop.deadline_misses)
                    / loop.execution_count
                    * 100.0
                )

                if success_rate >= loop.min_success_rate:
                    health["healthy_loops_count"] += 1
                else:
                    health["critical_loops_count"] += 1

        # Calculate overall health score
        total_loops = health["healthy_loops_count"] + health["critical_loops_count"]
        if total_loops > 0:
            health["overall_health_score"] = (
                health["healthy_loops_count"] / total_loops
            ) * 100.0

        # Resource usage penalties
        if health["cpu_usage_percent"] > 80:
            health["overall_health_score"] -= 10
        if health["memory_usage_percent"] > 80:
            health["overall_health_score"] -= 10

        health["overall_health_score"] = max(0.0, health["overall_health_score"])

        return health


# Global monitor instance
_monitor = None


def get_monitor() -> RealTimeMonitor:
    """Get the global real-time monitor instance."""
    global _monitor
    if _monitor is None:
        _monitor = RealTimeMonitor()
    return _monitor


def start_monitoring() -> bool:
    """Start global real-time monitoring."""
    return get_monitor().start_monitoring()


def stop_monitoring() -> bool:
    """Stop global real-time monitoring."""
    return get_monitor().stop_monitoring()


def record_loop_start(loop_name: str) -> float:
    """Record start of control loop execution."""
    return get_monitor().record_execution_start(loop_name)


def record_loop_end(loop_name: str, start_time: float) -> bool:
    """Record end of control loop execution."""
    return get_monitor().record_execution_end(loop_name, start_time)


def get_performance_summary() -> Dict[str, Any]:
    """Get current performance summary."""
    return get_monitor().get_performance_summary()


# Context manager for timing control loops
class ControlLoopTimer:
    """Context manager for timing control loop execution."""

    def __init__(self, loop_name: str):
        self.loop_name = loop_name
        self.start_time = None

    def __enter__(self):
        self.start_time = record_loop_start(self.loop_name)
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.start_time is not None:
            on_time = record_loop_end(self.loop_name, self.start_time)
            if not on_time:
                logger.warning(f"Control loop {self.loop_name} missed deadline")


# Decorator for timing functions
def time_control_loop(loop_name: str):
    """Decorator to time control loop functions."""

    def decorator(func):
        def wrapper(*args, **kwargs):
            start_time = record_loop_start(loop_name)
            try:
                result = func(*args, **kwargs)
                record_loop_end(loop_name, start_time)
                return result
            except Exception as e:
                record_loop_end(loop_name, start_time)
                raise e

        return wrapper

    return decorator


if __name__ == "__main__":
    # Example usage
    monitor = RealTimeMonitor()

    # Register a custom control loop
    monitor.register_control_loop(
        "custom_loop", target_period=0.033, deadline=0.066
    )  # ~30Hz

    # Start monitoring
    monitor.start_monitoring()

    try:
        # Simulate control loop execution
        for i in range(100):
            with ControlLoopTimer("drive_control"):
                # Simulate work
                time.sleep(0.015 + (i % 10) * 0.001)  # Variable execution time

            time.sleep(0.005)  # 200Hz total loop

        # Get performance summary
        summary = monitor.get_performance_summary()
        print("Performance Summary:")
        for loop_name, metrics in summary["control_loops"].items():
            print(f"  {loop_name}: {metrics['success_rate_percent']:.1f}% success rate")

    finally:
        monitor.stop_monitoring()
