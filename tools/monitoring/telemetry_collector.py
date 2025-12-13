#!/usr/bin/env python3
"""
Telemetry Collector - Structured Logging and Performance Monitoring

Collects, aggregates, and analyzes system telemetry data for debugging,
performance monitoring, and hardware integration validation.

Author: URC 2026 Autonomy Team
"""

import json
import logging
import statistics
import threading
import time
from collections import deque
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional, Union

logger = logging.getLogger(__name__)


@dataclass
class TelemetryMetric:
    """Individual telemetry metric."""

    name: str
    value: Union[int, float, str, bool]
    timestamp: float
    component: str
    metric_type: str = "gauge"  # gauge, counter, histogram
    tags: Dict[str, str] = field(default_factory=dict)


@dataclass
class TelemetryEvent:
    """Telemetry event with context."""

    event_type: str
    message: str
    timestamp: float
    component: str
    severity: str = "info"  # debug, info, warning, error, critical
    metadata: Dict[str, Any] = field(default_factory=dict)


class TelemetryCollector:
    """
    Collects and analyzes system telemetry data.

    Provides structured logging, performance metrics aggregation,
    and alerting for system monitoring during hardware integration.
    """

    def __init__(self, max_metrics: int = 10000, max_events: int = 1000):
        """
        Initialize telemetry collector.

        Args:
            max_metrics: Maximum number of metrics to store
            max_events: Maximum number of events to store
        """
        self.max_metrics = max_metrics
        self.max_events = max_events

        # Storage
        self.metrics: deque = deque(maxlen=max_metrics)
        self.events: deque = deque(maxlen=max_events)
        self.metric_aggregates: Dict[str, Dict[str, Any]] = {}

        # Locks for thread safety
        self.metrics_lock = threading.RLock()
        self.events_lock = threading.RLock()

        # Alert callbacks
        self.alert_callbacks: List[Callable] = []

        # Auto-flush settings
        self.auto_flush_interval = 60.0  # seconds
        self.last_flush = time.time()
        self.flush_thread: Optional[threading.Thread] = None
        self.flush_enabled = False

        logger.info("Telemetry collector initialized")

    def record_metric(
        self,
        component: str,
        metric_name: str,
        value: Union[int, float, str, bool],
        metric_type: str = "gauge",
        tags: Optional[Dict[str, str]] = None,
    ) -> bool:
        """
        Record a telemetry metric.

        Args:
            component: Component that generated the metric
            metric_name: Name of the metric
            value: Metric value
            metric_type: Type of metric (gauge, counter, histogram)
            tags: Additional tags for categorization

        Returns:
            bool: True if recorded successfully
        """
        if tags is None:
            tags = {}

        metric = TelemetryMetric(
            name=metric_name,
            value=value,
            timestamp=time.time(),
            component=component,
            metric_type=metric_type,
            tags=tags,
        )

        with self.metrics_lock:
            self.metrics.append(metric)

            # Update aggregates
            key = f"{component}.{metric_name}"
            if key not in self.metric_aggregates:
                self.metric_aggregates[key] = {
                    "count": 0,
                    "sum": 0,
                    "min": float("inf"),
                    "max": float("-inf"),
                    "values": deque(maxlen=100),  # Keep recent values for stats
                    "last_update": 0,
                }

            agg = self.metric_aggregates[key]
            agg["count"] += 1
            agg["sum"] += value if isinstance(value, (int, float)) else 0
            agg["min"] = (
                min(agg["min"], value)
                if isinstance(value, (int, float))
                else agg["min"]
            )
            agg["max"] = (
                max(agg["max"], value)
                if isinstance(value, (int, float))
                else agg["max"]
            )
            agg["values"].append(value)
            agg["last_update"] = metric.timestamp

        return True

    def record_event(
        self,
        component: str,
        event_type: str,
        message: str,
        severity: str = "info",
        metadata: Optional[Dict[str, Any]] = None,
    ) -> bool:
        """
        Record a telemetry event.

        Args:
            component: Component that generated the event
            event_type: Type of event
            message: Event message
            severity: Event severity level
            metadata: Additional event metadata

        Returns:
            bool: True if recorded successfully
        """
        if metadata is None:
            metadata = {}

        event = TelemetryEvent(
            event_type=event_type,
            message=message,
            timestamp=time.time(),
            component=component,
            severity=severity,
            metadata=metadata,
        )

        with self.events_lock:
            self.events.append(event)

            # Trigger alerts for critical events
            if severity in ["error", "critical"]:
                self._trigger_alert(event)

        # Log the event
        log_method = getattr(logger, severity, logger.info)
        log_method(f"[{component}] {event_type}: {message}")

        return True

    def get_performance_summary(self) -> Dict[str, Any]:
        """
        Get comprehensive performance summary from telemetry data.

        Returns:
            Dict containing performance metrics and statistics
        """
        summary = {
            "timestamp": time.time(),
            "metrics_summary": {},
            "events_summary": {},
            "system_health": self._calculate_system_health(),
        }

        # Aggregate metrics summary
        with self.metrics_lock:
            for key, agg in self.metric_aggregates.items():
                if agg["count"] > 0:
                    numeric_values = [
                        v for v in agg["values"] if isinstance(v, (int, float))
                    ]

                    summary["metrics_summary"][key] = {
                        "count": agg["count"],
                        "average": (
                            agg["sum"] / len(numeric_values) if numeric_values else None
                        ),
                        "min": agg["min"] if agg["min"] != float("inf") else None,
                        "max": agg["max"] if agg["max"] != float("-inf") else None,
                        "std_dev": (
                            statistics.stdev(numeric_values)
                            if len(numeric_values) > 1
                            else 0
                        ),
                        "last_value": agg["values"][-1] if agg["values"] else None,
                        "last_update": agg["last_update"],
                    }

        # Events summary
        with self.events_lock:
            severity_counts = {}
            component_counts = {}
            recent_events = []

            # Get recent events (last 100)
            for event in list(self.events)[-100:]:
                recent_events.append(
                    {
                        "timestamp": event.timestamp,
                        "component": event.component,
                        "event_type": event.event_type,
                        "severity": event.severity,
                        "message": event.message,
                    }
                )

                severity_counts[event.severity] = (
                    severity_counts.get(event.severity, 0) + 1
                )
                component_counts[event.component] = (
                    component_counts.get(event.component, 0) + 1
                )

            summary["events_summary"] = {
                "total_events": len(self.events),
                "severity_breakdown": severity_counts,
                "component_breakdown": component_counts,
                "recent_events": recent_events[-10:],  # Last 10 events
            }

        return summary

    def get_component_health(self, component_name: str) -> Dict[str, Any]:
        """
        Get health status for a specific component.

        Args:
            component_name: Name of component to check

        Returns:
            Dict containing component health metrics
        """
        health = {
            "component": component_name,
            "health_score": 100.0,
            "issues": [],
            "metrics": {},
            "recent_events": [],
        }

        # Check component metrics
        component_metrics = {
            k: v
            for k, v in self.metric_aggregates.items()
            if k.startswith(f"{component_name}.")
        }

        for metric_key, agg in component_metrics.items():
            metric_name = metric_key.split(".", 1)[1]
            health["metrics"][metric_name] = {
                "current": agg["values"][-1] if agg["values"] else None,
                "average": agg["sum"] / len(agg["values"]) if agg["values"] else None,
                "status": "nominal",
            }

            # Check for anomalous values (simplified)
            if agg["values"] and isinstance(agg["values"][-1], (int, float)):
                avg = statistics.mean(agg["values"])
                std = statistics.stdev(agg["values"]) if len(agg["values"]) > 1 else 0

                if std > 0 and abs(agg["values"][-1] - avg) > 3 * std:
                    health["issues"].append(
                        f"Anomalous {metric_name}: {agg['values'][-1]}"
                    )
                    health["metrics"][metric_name]["status"] = "anomalous"
                    health["health_score"] -= 10

        # Check recent events for this component
        with self.events_lock:
            component_events = [e for e in self.events if e.component == component_name]
            recent_events = component_events[-5:]  # Last 5 events

            for event in recent_events:
                health["recent_events"].append(
                    {
                        "timestamp": event.timestamp,
                        "type": event.event_type,
                        "severity": event.severity,
                        "message": event.message,
                    }
                )

                # Penalize health for errors/warnings
                if event.severity in ["error", "critical"]:
                    health["health_score"] -= 20
                    health["issues"].append(f"Critical event: {event.message}")
                elif event.severity == "warning":
                    health["health_score"] -= 5

        health["health_score"] = max(0.0, min(100.0, health["health_score"]))
        return health

    def start_auto_flush(
        self, output_dir: str = "telemetry_logs", flush_interval: float = 60.0
    ) -> bool:
        """
        Start automatic telemetry flushing to disk.

        Args:
            output_dir: Directory to save telemetry logs
            flush_interval: How often to flush data (seconds)

        Returns:
            bool: True if auto-flush started successfully
        """
        self.flush_enabled = True
        self.auto_flush_interval = flush_interval
        self.output_dir = Path(output_dir)
        self.output_dir.mkdir(exist_ok=True)

        self.flush_thread = threading.Thread(target=self._auto_flush_loop, daemon=True)
        self.flush_thread.start()

        logger.info(
            f"Auto-flush started (interval: {flush_interval}s, dir: {output_dir})"
        )
        return True

    def stop_auto_flush(self) -> bool:
        """Stop automatic telemetry flushing."""
        self.flush_enabled = False
        if self.flush_thread and self.flush_thread.is_alive():
            self.flush_thread.join(timeout=5.0)

        logger.info("Auto-flush stopped")
        return True

    def flush_to_disk(self, filename: Optional[str] = None) -> str:
        """
        Flush current telemetry data to disk.

        Args:
            filename: Optional filename (auto-generated if None)

        Returns:
            str: Path to the saved file
        """
        if filename is None:
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            filename = f"telemetry_{timestamp}.json"

        filepath = self.output_dir / filename

        data = {
            "flush_timestamp": time.time(),
            "metrics": list(self.metrics),
            "events": list(self.events),
            "summary": self.get_performance_summary(),
        }

        with open(filepath, "w") as f:
            json.dump(data, f, indent=2, default=str)

        logger.info(f"Telemetry flushed to {filepath}")
        return str(filepath)

    def add_alert_callback(self, callback: Callable) -> bool:
        """
        Add callback for telemetry alerts.

        Args:
            callback: Function to call on alerts

        Returns:
            bool: True if added successfully
        """
        self.alert_callbacks.append(callback)
        return True

    def _trigger_alert(self, event: TelemetryEvent):
        """Trigger alert for critical events."""
        alert_data = {
            "event_type": event.event_type,
            "message": event.message,
            "component": event.component,
            "severity": event.severity,
            "metadata": event.metadata,
            "timestamp": event.timestamp,
        }

        for callback in self.alert_callbacks:
            try:
                callback(event.component, event.event_type, alert_data)
            except Exception as e:
                logger.error(f"Alert callback failed: {e}")

    def _calculate_system_health(self) -> Dict[str, Any]:
        """Calculate overall system health from telemetry."""
        health = {
            "overall_score": 100.0,
            "component_count": 0,
            "healthy_components": 0,
            "degraded_components": 0,
            "critical_components": 0,
            "active_alerts": 0,
        }

        # Count components and their health
        component_healths = {}
        components = set()

        # Extract components from metrics
        for metric in self.metrics:
            components.add(metric.component)

        # Get health for each component
        for component in components:
            comp_health = self.get_component_health(component)
            component_healths[component] = comp_health

            health["component_count"] += 1

            if comp_health["health_score"] >= 80:
                health["healthy_components"] += 1
            elif comp_health["health_score"] >= 50:
                health["degraded_components"] += 1
            else:
                health["critical_components"] += 1

        # Count active alerts (recent critical events)
        recent_time = time.time() - 300  # Last 5 minutes
        with self.events_lock:
            for event in self.events:
                if event.timestamp > recent_time and event.severity in [
                    "error",
                    "critical",
                ]:
                    health["active_alerts"] += 1

        # Calculate overall score
        if health["component_count"] > 0:
            healthy_ratio = health["healthy_components"] / health["component_count"]
            health["overall_score"] = healthy_ratio * 100.0

            # Penalties for alerts
            health["overall_score"] -= health["active_alerts"] * 5.0
            health["overall_score"] -= health["critical_components"] * 10.0

        health["overall_score"] = max(0.0, min(100.0, health["overall_score"]))

        return health

    def _auto_flush_loop(self):
        """Background loop for automatic flushing."""
        while self.flush_enabled:
            current_time = time.time()

            if current_time - self.last_flush >= self.auto_flush_interval:
                try:
                    self.flush_to_disk()
                    self.last_flush = current_time
                except Exception as e:
                    logger.error(f"Auto-flush failed: {e}")

            time.sleep(1.0)


# Global collector instance
_collector = None


def get_collector() -> TelemetryCollector:
    """Get the global telemetry collector instance."""
    global _collector
    if _collector is None:
        _collector = TelemetryCollector()
    return _collector


def record_metric(
    component: str,
    metric_name: str,
    value: Union[int, float, str, bool],
    metric_type: str = "gauge",
    tags: Optional[Dict[str, str]] = None,
) -> bool:
    """Record a telemetry metric."""
    return get_collector().record_metric(
        component, metric_name, value, metric_type, tags
    )


def record_event(
    component: str,
    event_type: str,
    message: str,
    severity: str = "info",
    metadata: Optional[Dict[str, Any]] = None,
) -> bool:
    """Record a telemetry event."""
    return get_collector().record_event(
        component, event_type, message, severity, metadata
    )


def get_performance_summary() -> Dict[str, Any]:
    """Get current performance summary."""
    return get_collector().get_performance_summary()


def get_component_health(component_name: str) -> Dict[str, Any]:
    """Get health status for a component."""
    return get_collector().get_component_health(component_name)


# Context manager for timing operations
class TelemetryTimer:
    """Context manager for timing operations with telemetry."""

    def __init__(self, component: str, operation: str):
        self.component = component
        self.operation = operation
        self.start_time = None

    def __enter__(self):
        self.start_time = time.time()
        record_event(self.component, "operation_start", f"Started {self.operation}")
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        end_time = time.time()
        duration = end_time - self.start_time

        record_metric(self.component, f"{self.operation}_duration", duration)

        if exc_type is None:
            record_event(
                self.component,
                "operation_complete",
                f"Completed {self.operation} in {duration:.3f}s",
            )
        else:
            record_event(
                self.component,
                "operation_failed",
                f"Failed {self.operation} after {duration:.3f}s: {exc_val}",
                severity="error",
            )


# Decorator for telemetry
def with_telemetry(component: str, operation: Optional[str] = None):
    """Decorator to add telemetry to functions."""

    def decorator(func):
        op_name = operation or func.__name__

        def wrapper(*args, **kwargs):
            with TelemetryTimer(component, op_name):
                try:
                    result = func(*args, **kwargs)
                    record_metric(component, f"{op_name}_success", 1, "counter")
                    return result
                except Exception as e:
                    record_metric(component, f"{op_name}_failure", 1, "counter")
                    record_event(
                        component,
                        "function_error",
                        f"Function {op_name} failed: {e}",
                        severity="error",
                    )
                    raise

        return wrapper

    return decorator


if __name__ == "__main__":
    # Example usage
    collector = TelemetryCollector()

    # Record some metrics
    collector.record_metric("drive_system", "velocity", 1.5)
    collector.record_metric("drive_system", "current", 2.3)
    collector.record_metric("arm_system", "joint_temp", 35.2)

    # Record some events
    collector.record_event("power_system", "battery_low", "Battery level below 20%")
    collector.record_event("navigation", "waypoint_reached", "Reached waypoint 5")

    # Get summary
    summary = collector.get_performance_summary()
    print("Telemetry Summary:")
    print(f"  Metrics collected: {len(summary['metrics_summary'])}")
    print(f"  Events recorded: {summary['events_summary']['total_events']}")
    print(".1f")

    # Get component health
    drive_health = collector.get_component_health("drive_system")
    print(f"Drive system health: {drive_health['health_score']:.1f}%")
