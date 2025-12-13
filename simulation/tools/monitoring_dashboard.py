"""Real-time simulation monitoring and performance dashboard.

Provides comprehensive monitoring of simulation performance, system resources,
and generates alerts for performance issues.

Author: URC 2026 Autonomy Team
"""

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional

import psutil

from simulation.core.logging_config import get_simulation_logger


class AlertLevel(Enum):
    """Alert severity levels."""
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"
    CRITICAL = "critical"


@dataclass
class Alert:
    """Performance alert."""
    timestamp: float
    level: AlertLevel
    message: str
    metric: str
    value: Any
    threshold: Any
    component: str


class SimulationMonitor:
    """Real-time simulation monitoring and metrics collection."""

    def __init__(self, simulation_manager=None):
        """Initialize simulation monitor.

        Args:
            simulation_manager: SimulationManager instance to monitor
        """
        self.sim_manager = simulation_manager
        self.logger = get_simulation_logger(__name__, "monitor")

        # Monitoring state
        self.monitoring = False
        self.monitor_thread: Optional[threading.Thread] = None
        self.monitoring_interval = 1.0  # seconds

        # Metrics storage
        self.metrics_history: Dict[float, Dict[str, Any]] = {}
        self.alerts: List[Alert] = []
        self.max_metrics_history = 1000  # Keep last 1000 measurements

        # Performance data
        self.performance_data: Dict[str, List[Any]] = {
            "step_times": [],
            "memory_usage": [],
            "cpu_usage": [],
            "network_stats": [],
            "step_counts": [],
        }

        # Alert thresholds
        self.alert_thresholds = {
            "memory_usage_percent": {"warning": 75.0, "critical": 90.0},
            "cpu_usage_percent": {"warning": 80.0, "critical": 95.0},
            "network_latency_ms": {"warning": 500.0, "critical": 1000.0},
            "step_time_seconds": {"warning": 0.1, "critical": 0.5},
            "packet_loss_percent": {"warning": 5.0, "critical": 15.0},
        }

        # Callbacks for alerts
        self.alert_callbacks: List[Callable[[Alert], None]] = []

    def start_monitoring(self, interval: float = 1.0):
        """Start real-time monitoring.

        Args:
            interval: Monitoring interval in seconds
        """
        if self.monitoring:
            self.logger.warning("Monitoring already running")
            return

        self.monitoring_interval = interval
        self.monitoring = True

        self.monitor_thread = threading.Thread(
            target=self._monitoring_loop,
            daemon=True,
            name="SimulationMonitor"
        )
        self.monitor_thread.start()

        self.logger.info("Started simulation monitoring",
                        interval=interval)

    def stop_monitoring(self):
        """Stop monitoring."""
        if not self.monitoring:
            return

        self.monitoring = False

        if self.monitor_thread and self.monitor_thread.is_alive():
            self.monitor_thread.join(timeout=2.0)

        self.logger.info("Stopped simulation monitoring")

    def add_alert_callback(self, callback: Callable[[Alert], None]):
        """Add callback for alert notifications.

        Args:
            callback: Function to call when alerts are generated
        """
        self.alert_callbacks.append(callback)

    def set_alert_threshold(self, metric: str, level: str, threshold: float):
        """Set custom alert threshold.

        Args:
            metric: Metric name
            level: Alert level ("warning", "critical")
            threshold: Threshold value
        """
        if metric not in self.alert_thresholds:
            self.alert_thresholds[metric] = {}

        self.alert_thresholds[metric][level] = threshold
        self.logger.info("Updated alert threshold",
                        metric=metric,
                        level=level,
                        threshold=threshold)

    def _monitoring_loop(self):
        """Main monitoring loop."""
        while self.monitoring:
            try:
                metrics = self._collect_metrics()
                self._check_alerts(metrics)
                self._store_metrics(metrics)
                time.sleep(self.monitoring_interval)
            except Exception as e:
                self.logger.error("Monitoring loop error", error=str(e))
                time.sleep(self.monitoring_interval)

    def _collect_metrics(self) -> Dict[str, Any]:
        """Collect current system and simulation metrics.

        Returns:
            Dict with current metrics
        """
        current_time = time.time()

        # System metrics
        system_metrics = {
            "memory_usage_percent": psutil.virtual_memory().percent,
            "memory_used_gb": psutil.virtual_memory().used / (1024**3),
            "memory_available_gb": psutil.virtual_memory().available / (1024**3),
            "cpu_usage_percent": psutil.cpu_percent(),
            "cpu_count": psutil.cpu_count(),
            "disk_usage_percent": psutil.disk_usage('/').percent,
            "network_connections": len(psutil.net_connections()),
        }

        # Simulation metrics
        sim_metrics = {}
        if self.sim_manager:
            stats = self.sim_manager.get_statistics()
            sim_metrics.update({
                "simulation_running": self.sim_manager.is_running,
                "simulation_step_count": self.sim_manager.step_count,
                "simulation_time": self.sim_manager.time_manager.current_time,
                "active_sensors": len(self.sim_manager.sensors),
                "data_points_recorded": len(self.sim_manager.data_recorder.data),
                "network_latency_ms": (
                    self.sim_manager.network.get_current_latency()
                    if self.sim_manager.network else 0.0
                ),
                "network_packet_loss": (
                    self.sim_manager.network.get_packet_loss_rate()
                    if hasattr(self.sim_manager.network, 'get_packet_loss_rate')
                    else 0.0
                ),
            })

            # Step time calculation (if we have recent steps)
            if hasattr(self.sim_manager, '_last_step_time'):
                step_time = time.time() - self.sim_manager._last_step_time
                sim_metrics["last_step_time_seconds"] = step_time

        # Combine all metrics
        metrics = {
            "timestamp": current_time,
            "system": system_metrics,
            "simulation": sim_metrics,
        }

        return metrics

    def _check_alerts(self, metrics: Dict[str, Any]):
        """Check metrics against thresholds and generate alerts.

        Args:
            metrics: Current metrics
        """
        system_metrics = metrics.get("system", {})
        sim_metrics = metrics.get("simulation", {})

        # Memory alerts
        memory_usage = system_metrics.get("memory_usage_percent", 0)
        self._check_threshold("memory_usage_percent", memory_usage, "system")

        # CPU alerts
        cpu_usage = system_metrics.get("cpu_usage_percent", 0)
        self._check_threshold("cpu_usage_percent", cpu_usage, "system")

        # Network alerts
        network_latency = sim_metrics.get("network_latency_ms", 0)
        self._check_threshold("network_latency_ms", network_latency, "network")

        packet_loss = sim_metrics.get("network_packet_loss", 0)
        self._check_threshold("packet_loss_percent", packet_loss, "network")

        # Step time alerts
        step_time = sim_metrics.get("last_step_time_seconds", 0)
        self._check_threshold("step_time_seconds", step_time, "simulation")

    def _check_threshold(self, metric: str, value: float, component: str):
        """Check if metric exceeds threshold and create alert.

        Args:
            metric: Metric name
            value: Current value
            component: Component name
        """
        if metric not in self.alert_thresholds:
            return

        thresholds = self.alert_thresholds[metric]

        # Check critical threshold first
        if "critical" in thresholds and value >= thresholds["critical"]:
            self._create_alert(AlertLevel.CRITICAL, metric, value,
                             thresholds["critical"], component)
        elif "warning" in thresholds and value >= thresholds["warning"]:
            self._create_alert(AlertLevel.WARNING, metric, value,
                             thresholds["warning"], component)

    def _create_alert(self, level: AlertLevel, metric: str, value: Any,
                     threshold: Any, component: str):
        """Create and store alert.

        Args:
            level: Alert severity level
            metric: Metric name
            value: Current value
            threshold: Threshold value
            component: Component name
        """
        alert = Alert(
            timestamp=time.time(),
            level=level,
            message=f"{metric} exceeded {level.value} threshold: {value:.2f} >= {threshold:.2f}",
            metric=metric,
            value=value,
            threshold=threshold,
            component=component
        )

        self.alerts.append(alert)

        # Keep only recent alerts
        if len(self.alerts) > 100:
            self.alerts = self.alerts[-100:]

        # Log alert
        log_method = {
            AlertLevel.INFO: self.logger.info,
            AlertLevel.WARNING: self.logger.warning,
            AlertLevel.ERROR: self.logger.error,
            AlertLevel.CRITICAL: self.logger.critical,
        }.get(level, self.logger.warning)

        log_method(alert.message,
                  metric=metric,
                  value=value,
                  threshold=threshold,
                  component=component)

        # Call alert callbacks
        for callback in self.alert_callbacks:
            try:
                callback(alert)
            except Exception as e:
                self.logger.error("Alert callback failed", error=str(e))

    def _store_metrics(self, metrics: Dict[str, Any]):
        """Store metrics in history.

        Args:
            metrics: Metrics to store
        """
        timestamp = metrics["timestamp"]
        self.metrics_history[timestamp] = metrics

        # Maintain history size limit
        if len(self.metrics_history) > self.max_metrics_history:
            # Remove oldest entries
            oldest_keys = sorted(self.metrics_history.keys())[:len(self.metrics_history) - self.max_metrics_history]
            for key in oldest_keys:
                del self.metrics_history[key]

        # Update performance data
        system = metrics.get("system", {})
        simulation = metrics.get("simulation", {})

        self.performance_data["memory_usage"].append(system.get("memory_usage_percent", 0))
        self.performance_data["cpu_usage"].append(system.get("cpu_usage_percent", 0))
        self.performance_data["step_counts"].append(simulation.get("simulation_step_count", 0))

        # Maintain performance data size
        max_perf_data = 1000
        for key in self.performance_data:
            if len(self.performance_data[key]) > max_perf_data:
                self.performance_data[key] = self.performance_data[key][-max_perf_data:]

    def get_dashboard_data(self) -> Dict[str, Any]:
        """Get comprehensive dashboard data.

        Returns:
            Dict with all monitoring data for dashboard display
        """
        # Current metrics
        current_metrics = {}
        if self.metrics_history:
            latest_timestamp = max(self.metrics_history.keys())
            current_metrics = self.metrics_history[latest_timestamp]

        # Performance trends (last 100 points)
        trends = {}
        for key, data in self.performance_data.items():
            if data:
                trends[key] = {
                    "current": data[-1],
                    "average": sum(data) / len(data),
                    "min": min(data),
                    "max": max(data),
                    "recent": data[-100:] if len(data) > 100 else data,
                }

        # Active alerts (last 10)
        recent_alerts = [
            {
                "timestamp": alert.timestamp,
                "level": alert.level.value,
                "message": alert.message,
                "metric": alert.metric,
                "value": alert.value,
                "component": alert.component,
            }
            for alert in self.alerts[-10:]
        ]

        # System information
        system_info = {
            "total_memory_gb": psutil.virtual_memory().total / (1024**3),
            "cpu_count": psutil.cpu_count(),
            "cpu_logical_count": psutil.cpu_count(logical=True),
            "platform": psutil.platform(),
            "python_version": f"{psutil.python_version_tuple()[0]}.{psutil.python_version_tuple()[1]}",
        }

        # Simulation statistics
        sim_stats = {}
        if self.sim_manager:
            sim_stats = self.sim_manager.get_statistics()
            if hasattr(self.sim_manager.data_recorder, 'get_rl_statistics'):
                sim_stats["rl_metrics"] = self.sim_manager.data_recorder.get_rl_statistics()

        return {
            "current_metrics": current_metrics,
            "performance_trends": trends,
            "recent_alerts": recent_alerts,
            "alert_counts": self._get_alert_counts(),
            "system_info": system_info,
            "simulation_stats": sim_stats,
            "monitoring_status": {
                "active": self.monitoring,
                "interval": self.monitoring_interval,
                "metrics_history_size": len(self.metrics_history),
                "alert_count": len(self.alerts),
            }
        }

    def _get_alert_counts(self) -> Dict[str, int]:
        """Get counts of alerts by level.

        Returns:
            Dict with alert counts by level
        """
        counts = {level.value: 0 for level in AlertLevel}

        for alert in self.alerts:
            counts[alert.level.value] += 1

        return counts

    def get_performance_report(self) -> Dict[str, Any]:
        """Generate detailed performance report.

        Returns:
            Dict with comprehensive performance analysis
        """
        if not self.metrics_history:
            return {"status": "no_data"}

        # Time range
        timestamps = sorted(self.metrics_history.keys())
        time_range = timestamps[-1] - timestamps[0]

        # Aggregate statistics
        report = {
            "time_range_seconds": time_range,
            "total_measurements": len(self.metrics_history),
            "alert_summary": self._get_alert_counts(),
        }

        # System performance
        system_stats = {
            "memory_usage_avg": 0.0,
            "cpu_usage_avg": 0.0,
            "memory_usage_peak": 0.0,
            "cpu_usage_peak": 0.0,
        }

        memory_values = []
        cpu_values = []

        for metrics in self.metrics_history.values():
            system = metrics.get("system", {})
            memory_values.append(system.get("memory_usage_percent", 0))
            cpu_values.append(system.get("cpu_usage_percent", 0))

        if memory_values:
            system_stats["memory_usage_avg"] = sum(memory_values) / len(memory_values)
            system_stats["memory_usage_peak"] = max(memory_values)

        if cpu_values:
            system_stats["cpu_usage_avg"] = sum(cpu_values) / len(cpu_values)
            system_stats["cpu_usage_peak"] = max(cpu_values)

        report["system_performance"] = system_stats

        # Simulation performance
        sim_stats = {
            "avg_step_time": 0.0,
            "total_steps": 0,
            "simulation_time": 0.0,
        }

        step_times = []
        for metrics in self.metrics_history.values():
            simulation = metrics.get("simulation", {})
            if "last_step_time_seconds" in simulation:
                step_times.append(simulation["last_step_time_seconds"])
            sim_stats["total_steps"] = max(sim_stats["total_steps"],
                                         simulation.get("simulation_step_count", 0))
            sim_stats["simulation_time"] = max(sim_stats["simulation_time"],
                                             simulation.get("simulation_time", 0.0))

        if step_times:
            sim_stats["avg_step_time"] = sum(step_times) / len(step_times)

        report["simulation_performance"] = sim_stats

        return report

    def export_metrics(self, filepath: str, format: str = "json") -> bool:
        """Export monitoring data to file.

        Args:
            filepath: Path to export file
            format: Export format ("json", "csv")

        Returns:
            bool: True if export successful
        """
        try:
            from pathlib import Path
            path = Path(filepath)
            path.parent.mkdir(parents=True, exist_ok=True)

            if format == "json":
                import json
                data = {
                    "metrics_history": self.metrics_history,
                    "alerts": [vars(alert) for alert in self.alerts],
                    "performance_data": self.performance_data,
                    "exported_at": time.time(),
                }
                with open(path, "w") as f:
                    json.dump(data, f, indent=2, default=str)

            elif format == "csv":
                import csv
                with open(path, "w", newline="") as f:
                    writer = csv.writer(f)

                    # Write header
                    writer.writerow(["timestamp", "memory_usage", "cpu_usage",
                                   "step_count", "alerts"])

                    # Write data
                    for timestamp in sorted(self.metrics_history.keys()):
                        metrics = self.metrics_history[timestamp]
                        system = metrics.get("system", {})
                        simulation = metrics.get("simulation", {})

                        writer.writerow([
                            timestamp,
                            system.get("memory_usage_percent", 0),
                            system.get("cpu_usage_percent", 0),
                            simulation.get("simulation_step_count", 0),
                            len([a for a in self.alerts if a.timestamp <= timestamp])
                        ])

            self.logger.info("Metrics exported", filepath=str(path), format=format)
            return True

        except Exception as e:
            self.logger.error("Failed to export metrics", error=str(e))
            return False
