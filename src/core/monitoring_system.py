#!/usr/bin/env python3
"""
Monitoring System - High-Level System Monitoring Interface

Provides a unified monitoring interface that integrates with the observability system,
performance monitoring, and health checking using existing libraries.

This module serves as a high-level abstraction over the detailed observability
system, providing simple monitoring APIs for the application.

Libraries Used:
- psutil: System and process monitoring
- Built-in observability system for metrics and tracing
- asyncio: Async monitoring operations

Author: URC 2026 Monitoring System Team
"""

import asyncio
import time
import psutil
import threading
from typing import Dict, List, Any, Optional, Callable
from dataclasses import dataclass
from enum import Enum

# Import the observability system
from .observability import get_observability_system, HealthStatus


class MonitoringLevel(Enum):
    """Monitoring detail levels."""

    BASIC = "basic"  # Essential metrics only
    STANDARD = "standard"  # Standard monitoring
    DETAILED = "detailed"  # Comprehensive monitoring
    DEBUG = "debug"  # Debug-level monitoring


@dataclass
class SystemMetrics:
    """System-wide metrics snapshot."""

    timestamp: float
    cpu_percent: float
    memory_percent: float
    memory_mb: float
    disk_percent: float
    network_connections: int
    active_threads: int
    uptime_seconds: float


@dataclass
class ComponentMetrics:
    """Component-specific metrics."""

    component_name: str
    status: str
    response_time_ms: Optional[float] = None
    error_count: int = 0
    last_check: Optional[float] = None
    metadata: Optional[Dict[str, Any]] = None


class MonitoringSystem:
    """
    High-level monitoring system that integrates with observability.

    Provides:
    - System resource monitoring
    - Component health tracking
    - Performance metrics aggregation
    - Alert generation and thresholds
    """

    def __init__(self, monitoring_level: MonitoringLevel = MonitoringLevel.STANDARD):
        self.monitoring_level = monitoring_level
        self.observability = get_observability_system()

        # Monitoring state
        self._monitoring_active = False
        self._monitor_thread: Optional[threading.Thread] = None
        self._component_monitors: Dict[str, Callable] = {}

        # Metrics history
        self.metrics_history: List[SystemMetrics] = []
        self.max_history_size = 100

        # Thresholds
        self.alert_thresholds = {
            "cpu_percent": 80.0,
            "memory_percent": 85.0,
            "disk_percent": 90.0,
        }

        # Alerts
        self.active_alerts: List[Dict[str, Any]] = []

    def start_monitoring(self, interval_seconds: float = 30.0):
        """Start the monitoring system."""
        if self._monitoring_active:
            return

        self._monitoring_active = True
        self._monitor_thread = threading.Thread(
            target=self._monitoring_loop, args=(interval_seconds,), daemon=True
        )
        self._monitor_thread.start()

        print(f"📊 Monitoring system started (level: {self.monitoring_level.value})")

    def stop_monitoring(self):
        """Stop the monitoring system."""
        self._monitoring_active = False
        if self._monitor_thread:
            self._monitor_thread.join(timeout=5.0)

        print("📊 Monitoring system stopped")

    def _monitoring_loop(self, interval: float):
        """Main monitoring loop."""
        while self._monitoring_active:
            try:
                self._collect_system_metrics()
                self._check_component_health()
                self._update_observability_metrics()
                self._check_alerts()

                if self.monitoring_level == MonitoringLevel.DEBUG:
                    self._log_debug_info()

            except Exception as e:
                print(f"⚠️ Monitoring error: {e}")

            time.sleep(interval)

    def _collect_system_metrics(self):
        """Collect system-wide metrics."""
        try:
            # CPU metrics
            cpu_percent = psutil.cpu_percent(interval=1.0)

            # Memory metrics
            memory = psutil.virtual_memory()
            memory_percent = memory.percent
            memory_mb = memory.used / 1024 / 1024

            # Disk metrics
            disk = psutil.disk_usage("/")
            disk_percent = disk.percent

            # Network metrics
            network_connections = len(psutil.net_connections())

            # Process metrics
            process = psutil.Process()
            active_threads = process.num_threads()

            # Uptime
            uptime_seconds = time.time() - psutil.boot_time()

            metrics = SystemMetrics(
                timestamp=time.time(),
                cpu_percent=cpu_percent,
                memory_percent=memory_percent,
                memory_mb=memory_mb,
                disk_percent=disk_percent,
                network_connections=network_connections,
                active_threads=active_threads,
                uptime_seconds=uptime_seconds,
            )

            self.metrics_history.append(metrics)
            if len(self.metrics_history) > self.max_history_size:
                self.metrics_history.pop(0)

        except Exception as e:
            print(f"⚠️ Failed to collect system metrics: {e}")

    def _check_component_health(self):
        """Check health of registered components."""
        for component_name, health_check in self._component_monitors.items():
            try:
                start_time = time.time()
                result = health_check()
                response_time = (time.time() - start_time) * 1000

                # Create component metrics
                metrics = ComponentMetrics(
                    component_name=component_name,
                    status=result.get("status", "unknown"),
                    response_time_ms=response_time,
                    error_count=result.get("error_count", 0),
                    last_check=time.time(),
                    metadata=result.get("metadata", {}),
                )

                # Store in observability system
                if hasattr(self.observability, "record_component_health"):
                    self.observability.record_component_health(metrics)

            except Exception as e:
                print(f"⚠️ Component health check failed for {component_name}: {e}")

    def _update_observability_metrics(self):
        """Update observability system with latest metrics."""
        if self.metrics_history:
            latest = self.metrics_history[-1]

            # Update system metrics in observability
            if hasattr(self.observability, "update_system_metrics"):
                self.observability.update_system_metrics()

    def _check_alerts(self):
        """Check for alert conditions."""
        if not self.metrics_history:
            return

        latest = self.metrics_history[-1]

        # Check CPU threshold
        if latest.cpu_percent > self.alert_thresholds["cpu_percent"]:
            self._create_alert(
                "high_cpu", f"CPU usage: {latest.cpu_percent:.1f}%", latest.cpu_percent
            )

        # Check memory threshold
        if latest.memory_percent > self.alert_thresholds["memory_percent"]:
            self._create_alert(
                "high_memory",
                f"Memory usage: {latest.memory_percent:.1f}%",
                latest.memory_percent,
            )

        # Check disk threshold
        if latest.disk_percent > self.alert_thresholds["disk_percent"]:
            self._create_alert(
                "high_disk",
                f"Disk usage: {latest.disk_percent:.1f}%",
                latest.disk_percent,
            )

    def _create_alert(self, alert_type: str, message: str, value: float):
        """Create an alert if it doesn't already exist."""
        # Check if alert already exists
        for alert in self.active_alerts:
            if alert["type"] == alert_type:
                alert["last_updated"] = time.time()
                alert["value"] = value
                return

        # Create new alert
        alert = {
            "type": alert_type,
            "message": message,
            "value": value,
            "created": time.time(),
            "last_updated": time.time(),
        }

        self.active_alerts.append(alert)
        print(f"🚨 ALERT: {message}")

    def _log_debug_info(self):
        """Log debug information."""
        if self.metrics_history:
            latest = self.metrics_history[-1]
            print(
                f"📊 DEBUG - CPU: {latest.cpu_percent:.1f}%, MEM: {latest.memory_percent:.1f}%, "
                f"DISK: {latest.disk_percent:.1f}%, THREADS: {latest.active_threads}"
            )

    def register_component_monitor(
        self, component_name: str, health_check: Callable[[], Dict[str, Any]]
    ):
        """Register a component for health monitoring."""
        self._component_monitors[component_name] = health_check

    def unregister_component_monitor(self, component_name: str):
        """Unregister a component from health monitoring."""
        self._component_monitors.pop(component_name, None)

    def get_system_status(self) -> Dict[str, Any]:
        """Get current system status."""
        if not self.metrics_history:
            return {"status": "no_data"}

        latest = self.metrics_history[-1]

        return {
            "timestamp": latest.timestamp,
            "cpu_percent": latest.cpu_percent,
            "memory_percent": latest.memory_percent,
            "disk_percent": latest.disk_percent,
            "network_connections": latest.network_connections,
            "active_threads": latest.active_threads,
            "uptime_seconds": latest.uptime_seconds,
            "active_alerts": len(self.active_alerts),
            "monitored_components": len(self._component_monitors),
            "monitoring_level": self.monitoring_level.value,
            "monitoring_active": self._monitoring_active,
        }

    def get_alerts(self) -> List[Dict[str, Any]]:
        """Get active alerts."""
        return self.active_alerts.copy()

    def clear_alert(self, alert_type: str):
        """Clear an alert by type."""
        self.active_alerts = [
            alert for alert in self.active_alerts if alert["type"] != alert_type
        ]

    def set_alert_threshold(self, metric: str, threshold: float):
        """Set alert threshold for a metric."""
        if metric in self.alert_thresholds:
            self.alert_thresholds[metric] = threshold

    def get_metrics_history(self, limit: Optional[int] = None) -> List[SystemMetrics]:
        """Get metrics history."""
        if limit:
            return self.metrics_history[-limit:]
        return self.metrics_history.copy()


# Global instance
_monitoring_instance = None
_monitoring_lock = threading.Lock()


def get_monitoring_system(
    monitoring_level: MonitoringLevel = MonitoringLevel.STANDARD,
) -> MonitoringSystem:
    """Get or create the global monitoring system instance."""
    global _monitoring_instance

    if _monitoring_instance is None:
        with _monitoring_lock:
            if _monitoring_instance is None:
                _monitoring_instance = MonitoringSystem(monitoring_level)

    return _monitoring_instance


def start_monitoring(interval_seconds: float = 30.0):
    """Start the global monitoring system."""
    system = get_monitoring_system()
    system.start_monitoring(interval_seconds)


def stop_monitoring():
    """Stop the global monitoring system."""
    global _monitoring_instance
    if _monitoring_instance:
        _monitoring_instance.stop_monitoring()
        _monitoring_instance = None
