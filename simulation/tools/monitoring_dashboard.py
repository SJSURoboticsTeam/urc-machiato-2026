"""Enhanced Real-time Monitoring Dashboard with Telemetry Analytics

Provides comprehensive monitoring using Polars-powered telemetry system:
- Real-time performance analytics with Polars DataFrames
- Advanced anomaly detection and trend prediction
- Interactive visualizations with rich formatting
- Time-series analytics with memory-efficient storage
- Health scoring and predictive maintenance alerts

Author: URC 2026 Autonomy Team
"""

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional
from datetime import datetime, timedelta
import json

# Enhanced monitoring libraries
try:
    import polars as pl
    POLARS_AVAILABLE = True
except ImportError:
    POLARS_AVAILABLE = False

try:
    import psutil
    PSUTIL_AVAILABLE = True
except ImportError:
    PSUTIL_AVAILABLE = False

# Import telemetry system
try:
    from src.core.telemetry_system import get_telemetry_system, record_metric
    TELEMETRY_AVAILABLE = True
except ImportError:
    TELEMETRY_AVAILABLE = False

# Import resource manager
try:
    from src.core.mission_resource_manager import get_mission_resource_manager
    RESOURCE_MANAGER_AVAILABLE = True
except ImportError:
    RESOURCE_MANAGER_AVAILABLE = False

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
        """Initialize enhanced simulation monitor with telemetry analytics.

        Args:
            simulation_manager: SimulationManager instance to monitor
        """
        self.sim_manager = simulation_manager
        self.logger = get_simulation_logger(__name__, "monitor")

        # Enhanced telemetry integration (HIGH PRIORITY)
        if TELEMETRY_AVAILABLE:
            self.telemetry = get_telemetry_system()
            self.logger.info("Telemetry system integrated for monitoring")
        else:
            self.telemetry = None
            self.logger.warning("Telemetry system not available - using basic monitoring")

        # Mission resource manager integration
        if RESOURCE_MANAGER_AVAILABLE:
            self.resource_manager = get_mission_resource_manager()
            self.logger.info("Mission Resource Manager integrated for adaptive monitoring")
        else:
            self.resource_manager = None
            self.logger.warning("Mission Resource Manager not available - basic monitoring only")

        # Polars-based high-performance metrics storage (HIGH PRIORITY)
        if POLARS_AVAILABLE:
            self.metrics_schema = {
                'timestamp': pl.Datetime('ns'),
                'component': pl.Utf8,
                'metric_name': pl.Utf8,
                'value': pl.Float64,
                'unit': pl.Utf8,
                'tags': pl.Utf8  # JSON string of tags
            }
            self.metrics_df = pl.DataFrame(schema=self.metrics_schema)
            self.metrics_lock = threading.Lock()
            self.logger.info("Polars-based metrics storage initialized")
        else:
            self.metrics_df = None
            self.metrics_lock = None
            self.logger.warning("Polars not available - using legacy metrics storage")

        # Monitoring state
        self.monitoring = False
        self.monitor_thread: Optional[threading.Thread] = None
        self.monitoring_interval = 1.0  # seconds

        # Enhanced metrics storage (legacy fallback)
        self.metrics_history: Dict[float, Dict[str, Any]] = {}
        self.alerts: List[Alert] = []
        self.max_metrics_history = 10000  # Increased for Polars efficiency

        # Advanced analytics state
        self.anomaly_detection_enabled = True
        self.predictive_analytics_enabled = True
        self.health_scoring_enabled = True
        self.alert_cooldown = 30.0  # seconds
        self.last_alert_times: Dict[str, float] = {}

        # Performance data with enhanced tracking
        self.performance_data: Dict[str, List[Any]] = {
            "step_times": [],
            "memory_usage": [],
            "cpu_usage": [],
            "network_stats": [],
            "step_counts": [],
            "health_scores": [],  # New: system health tracking
            "anomaly_counts": [],  # New: anomaly detection
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
            target=self._monitoring_loop, daemon=True, name="SimulationMonitor"
        )
        self.monitor_thread.start()

        self.logger.info("Started simulation monitoring", interval=interval)

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
        self.logger.info(
            "Updated alert threshold", metric=metric, level=level, threshold=threshold
        )

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
            "disk_usage_percent": psutil.disk_usage("/").percent,
            "network_connections": len(psutil.net_connections()),
        }

        # Simulation metrics
        sim_metrics = {}
        if self.sim_manager:
            stats = self.sim_manager.get_statistics()
            sim_metrics.update(
                {
                    "simulation_running": self.sim_manager.is_running,
                    "simulation_step_count": self.sim_manager.step_count,
                    "simulation_time": self.sim_manager.time_manager.current_time,
                    "active_sensors": len(self.sim_manager.sensors),
                    "data_points_recorded": len(self.sim_manager.data_recorder.data),
                    "network_latency_ms": (
                        self.sim_manager.network.get_current_latency()
                        if self.sim_manager.network
                        else 0.0
                    ),
                    "network_packet_loss": (
                        self.sim_manager.network.get_packet_loss_rate()
                        if hasattr(self.sim_manager.network, "get_packet_loss_rate")
                        else 0.0
                    ),
                }
            )

            # Step time calculation (if we have recent steps)
            if hasattr(self.sim_manager, "_last_step_time"):
                step_time = time.time() - self.sim_manager._last_step_time
                sim_metrics["last_step_time_seconds"] = step_time

        # Resource manager metrics
        resource_metrics = {}
        if self.resource_manager:
            resource_status = self.resource_manager.get_resource_status()
            resource_metrics.update({
                "component_status": resource_status.get("component_status", {}),
                "mission_profile": resource_status.get("mission_profile", "unknown"),
                "adaptive_scaling_active": resource_status.get("adaptive_scaling", False),
                "resource_manager_memory_mb": resource_status.get("current_resources", {}).get("memory_mb", 0),
                "resource_manager_cpu_percent": resource_status.get("current_resources", {}).get("cpu_percent", 0),
            })

        # Combine all metrics
        metrics = {
            "timestamp": current_time,
            "system": system_metrics,
            "simulation": sim_metrics,
            "resource_manager": resource_metrics,
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
        memory_alert = self._check_threshold("memory_usage_percent", memory_usage, "system")

        # CPU alerts
        cpu_usage = system_metrics.get("cpu_usage_percent", 0)
        cpu_alert = self._check_threshold("cpu_usage_percent", cpu_usage, "system")

        # Adaptive scaling integration
        if self.resource_manager and self.resource_manager.adaptive_scaling_enabled:
            self._trigger_adaptive_scaling(memory_usage, cpu_usage, memory_alert, cpu_alert)

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
            self._create_alert(
                AlertLevel.CRITICAL, metric, value, thresholds["critical"], component
            )
        elif "warning" in thresholds and value >= thresholds["warning"]:
            self._create_alert(
                AlertLevel.WARNING, metric, value, thresholds["warning"], component
            )

    def _create_alert(
        self, level: AlertLevel, metric: str, value: Any, threshold: Any, component: str
    ):
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
            component=component,
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

        log_method(
            alert.message,
            metric=metric,
            value=value,
            threshold=threshold,
            component=component,
        )

        # Call alert callbacks
        for callback in self.alert_callbacks:
            try:
                callback(alert)
            except Exception as e:
                self.logger.error("Alert callback failed", error=str(e))

    def _trigger_adaptive_scaling(self, memory_usage: float, cpu_usage: float,
                                memory_alert: bool, cpu_alert: bool):
        """Trigger adaptive scaling based on resource usage and alerts.

        Args:
            memory_usage: Current memory usage percentage
            cpu_usage: Current CPU usage percentage
            memory_alert: Whether memory alert was triggered
            cpu_alert: Whether CPU alert was triggered
        """
        if not self.resource_manager:
            return

        # Emergency scaling for critical alerts
        if memory_alert or cpu_alert:
            if memory_usage >= self.alert_thresholds.get("memory_usage_percent", {}).get("critical", 90) or \
               cpu_usage >= self.alert_thresholds.get("cpu_usage_percent", {}).get("critical", 95):
                self.logger.warning("Critical resource usage - triggering emergency scaling")
                # Emergency scaling is handled by the resource manager internally
                return

        # Gradual scaling for warning thresholds
        memory_warning = self.alert_thresholds.get("memory_usage_percent", {}).get("warning", 75)
        cpu_warning = self.alert_thresholds.get("cpu_usage_percent", {}).get("warning", 80)

        if memory_usage >= memory_warning or cpu_usage >= cpu_warning:
            self.logger.info("Resource warning threshold reached - triggering adaptive scaling")
            # Gradual scaling is handled by the resource manager internally

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
            oldest_keys = sorted(self.metrics_history.keys())[
                : len(self.metrics_history) - self.max_metrics_history
            ]
            for key in oldest_keys:
                del self.metrics_history[key]

        # Update performance data
        system = metrics.get("system", {})
        simulation = metrics.get("simulation", {})

        self.performance_data["memory_usage"].append(
            system.get("memory_usage_percent", 0)
        )
        self.performance_data["cpu_usage"].append(system.get("cpu_usage_percent", 0))
        self.performance_data["step_counts"].append(
            simulation.get("simulation_step_count", 0)
        )

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
            if hasattr(self.sim_manager.data_recorder, "get_rl_statistics"):
                sim_stats[
                    "rl_metrics"
                ] = self.sim_manager.data_recorder.get_rl_statistics()

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
            },
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
            sim_stats["total_steps"] = max(
                sim_stats["total_steps"], simulation.get("simulation_step_count", 0)
            )
            sim_stats["simulation_time"] = max(
                sim_stats["simulation_time"], simulation.get("simulation_time", 0.0)
            )

        if step_times:
            sim_stats["avg_step_time"] = sum(step_times) / len(step_times)

        report["simulation_performance"] = sim_stats

        return report

    # ============================================================================
    # ENHANCED ANALYTICS METHODS (HIGH PRIORITY)
    # ============================================================================

    def record_metric_polars(self, component: str, metric_name: str,
                           value: float, unit: str = "",
                           tags: Optional[Dict[str, Any]] = None) -> None:
        """Record metric using high-performance Polars storage."""
        if not POLARS_AVAILABLE or not self.metrics_df or not self.metrics_lock:
            # Fallback to legacy recording
            self.record_metric_legacy(component, metric_name, value, unit, tags)
            return

        with self.metrics_lock:
            # Create new row
            row_data = {
                'timestamp': datetime.now(),
                'component': component,
                'metric_name': metric_name,
                'value': float(value),
                'unit': unit,
                'tags': json.dumps(tags or {})
            }

            # Convert to Polars DataFrame and append
            row_df = pl.DataFrame([row_data])
            self.metrics_df = pl.concat([self.metrics_df, row_df], how='diagonal')

            # Maintain size limit
            if len(self.metrics_df) > self.max_metrics_history:
                # Keep most recent entries
                self.metrics_df = self.metrics_df.tail(self.max_metrics_history)

        # Also record to telemetry system if available
        if TELEMETRY_AVAILABLE and self.telemetry:
            self.telemetry.record_point(
                measurement=f"simulation.{component}",
                fields={metric_name: value},
                tags={"unit": unit, **(tags or {})}
            )

    def record_metric_legacy(self, component: str, metric_name: str,
                           value: float, unit: str = "",
                           tags: Optional[Dict[str, Any]] = None) -> None:
        """Legacy metric recording for fallback."""
        timestamp = time.time()
        metric_key = f"{component}.{metric_name}"

        if metric_key not in self.metrics_history:
            self.metrics_history[metric_key] = []

        self.metrics_history[metric_key].append({
            'timestamp': timestamp,
            'value': value,
            'unit': unit,
            'tags': tags or {}
        })

        # Maintain size limit
        if len(self.metrics_history[metric_key]) > self.max_metrics_history:
            self.metrics_history[metric_key] = self.metrics_history[metric_key][-self.max_metrics_history:]

    def get_health_score(self) -> float:
        """Calculate comprehensive system health score using telemetry analytics."""
        if TELEMETRY_AVAILABLE and self.telemetry:
            return self.telemetry.get_health_score()

        # Fallback health calculation
        try:
            if PSUTIL_AVAILABLE:
                cpu_usage = psutil.cpu_percent()
                memory = psutil.virtual_memory()

                # Simple health score based on resource usage
                cpu_health = max(0, 1.0 - cpu_usage / 100.0)
                memory_health = max(0, 1.0 - memory.percent / 100.0)

                return (cpu_health + memory_health) / 2.0
            else:
                return 0.8  # Default good health if no monitoring available
        except Exception:
            return 0.5  # Neutral health on error

    def detect_anomalies(self, component: str, metric_name: str,
                        time_window_minutes: int = 10) -> List[Dict[str, Any]]:
        """Detect anomalies in metrics using statistical analysis."""
        if TELEMETRY_AVAILABLE and self.telemetry:
            measurement = f"simulation.{component}"
            return self.telemetry.get_anomalies(measurement)

        # Fallback anomaly detection using stored metrics
        if POLARS_AVAILABLE and self.metrics_df:
            with self.metrics_lock:
                # Filter data
                component_data = self.metrics_df.filter(
                    (pl.col('component') == component) &
                    (pl.col('metric_name') == metric_name) &
                    (pl.col('timestamp') >= datetime.now() - timedelta(minutes=time_window_minutes))
                )

                if len(component_data) < 5:
                    return []

                # Simple anomaly detection based on standard deviation
                values = component_data['value'].to_list()
                if not values:
                    return []

                mean = sum(values) / len(values)
                variance = sum((x - mean) ** 2 for x in values) / len(values)
                std_dev = variance ** 0.5

                anomalies = []
                for i, (timestamp, value) in enumerate(zip(
                    component_data['timestamp'].to_list(),
                    values
                )):
                    if std_dev > 0:
                        z_score = abs(value - mean) / std_dev
                        if z_score > 3.0:  # 3-sigma rule
                            anomalies.append({
                                'timestamp': timestamp,
                                'component': component,
                                'metric': metric_name,
                                'value': value,
                                'z_score': z_score,
                                'severity': 'high' if z_score > 4.0 else 'medium'
                            })

                return anomalies

        return []

    def predict_performance_trend(self, component: str, metric_name: str) -> Dict[str, Any]:
        """Predict performance trends using telemetry analytics."""
        if TELEMETRY_AVAILABLE and self.telemetry:
            measurement = f"simulation.{component}"
            return self.telemetry.predict_trend(measurement, metric_name)

        # Fallback trend analysis
        if POLARS_AVAILABLE and self.metrics_df:
            with self.metrics_lock:
                # Get recent data
                recent_data = self.metrics_df.filter(
                    (pl.col('component') == component) &
                    (pl.col('metric_name') == metric_name) &
                    (pl.col('timestamp') >= datetime.now() - timedelta(minutes=30))
                )

                if len(recent_data) < 5:
                    return {'trend': 'insufficient_data'}

                # Simple linear regression
                values = recent_data['value'].to_list()
                n = len(values)

                if n < 2:
                    return {'trend': 'insufficient_data'}

                # Calculate slope
                x = list(range(n))
                y = values

                slope = sum((xi - sum(x)/n) * (yi - sum(y)/n) for xi, yi in zip(x, y)) / sum((xi - sum(x)/n)**2 for xi in x)

                trend = "increasing" if slope > 0.01 else "decreasing" if slope < -0.01 else "stable"

                return {
                    'trend': trend,
                    'slope': slope,
                    'confidence': min(1.0, max(0.0, 1.0 - abs(slope) * 10))
                }

        return {'trend': 'analytics_unavailable'}

    def get_performance_analytics(self, time_window_minutes: int = 30) -> Dict[str, Any]:
        """Get comprehensive performance analytics."""
        analytics = {
            'health_score': self.get_health_score(),
            'anomalies_detected': 0,
            'performance_trends': {},
            'resource_usage': {},
            'timestamp': datetime.now().isoformat()
        }

        # Analyze key components
        components_to_analyze = ['simulation', 'network', 'memory', 'cpu']
        metrics_to_analyze = ['step_time', 'memory_usage', 'cpu_usage', 'latency']

        for component in components_to_analyze:
            for metric in metrics_to_analyze:
                try:
                    anomalies = self.detect_anomalies(component, metric, time_window_minutes)
                    analytics['anomalies_detected'] += len(anomalies)

                    trend = self.predict_performance_trend(component, metric)
                    analytics['performance_trends'][f"{component}.{metric}"] = trend

                except Exception as e:
                    self.logger.debug(f"Failed to analyze {component}.{metric}: {e}")

        # Resource usage summary
        if PSUTIL_AVAILABLE:
            try:
                memory = psutil.virtual_memory()
                analytics['resource_usage'] = {
                    'cpu_percent': psutil.cpu_percent(),
                    'memory_percent': memory.percent,
                    'memory_used_gb': memory.used / (1024**3),
                    'memory_available_gb': memory.available / (1024**3)
                }
            except Exception as e:
                self.logger.debug(f"Failed to get resource usage: {e}")

        return analytics

    def generate_enhanced_report(self, time_window_minutes: int = 30) -> Dict[str, Any]:
        """Generate enhanced monitoring report with analytics."""
        base_report = self.generate_report()

        # Add advanced analytics
        analytics = self.get_performance_analytics(time_window_minutes)

        enhanced_report = {
            **base_report,
            'analytics': analytics,
            'recommendations': self._generate_recommendations(analytics),
            'alerts_summary': self._summarize_alerts(),
            'data_quality': self._assess_data_quality()
        }

        return enhanced_report

    def _generate_recommendations(self, analytics: Dict[str, Any]) -> List[str]:
        """Generate performance improvement recommendations."""
        recommendations = []

        health_score = analytics.get('health_score', 1.0)
        if health_score < 0.7:
            recommendations.append("System health is degraded - investigate high resource usage")

        anomaly_count = analytics.get('anomalies_detected', 0)
        if anomaly_count > 5:
            recommendations.append(f"High anomaly count ({anomaly_count}) detected - review system stability")

        # Check performance trends
        trends = analytics.get('performance_trends', {})
        concerning_trends = []
        for metric, trend_data in trends.items():
            if trend_data.get('trend') == 'increasing' and 'memory' in metric:
                concerning_trends.append(f"Memory usage increasing for {metric}")

        if concerning_trends:
            recommendations.extend(concerning_trends)

        # Resource usage recommendations
        resource_usage = analytics.get('resource_usage', {})
        cpu_percent = resource_usage.get('cpu_percent', 0)
        memory_percent = resource_usage.get('memory_percent', 0)

        if cpu_percent > 80:
            recommendations.append("High CPU usage detected - consider performance optimization")
        if memory_percent > 85:
            recommendations.append("High memory usage detected - monitor for memory leaks")

        return recommendations

    def _summarize_alerts(self) -> Dict[str, Any]:
        """Summarize recent alerts by severity."""
        summary = {
            'total_alerts': len(self.alerts),
            'by_severity': {},
            'recent_alerts': [],
            'most_common_types': {}
        }

        # Count by severity
        severity_counts = {}
        type_counts = {}

        for alert in self.alerts[-100:]:  # Last 100 alerts
            severity = alert.level.value
            severity_counts[severity] = severity_counts.get(severity, 0) + 1

            alert_type = alert.metric
            type_counts[alert_type] = type_counts.get(alert_type, 0) + 1

            # Add to recent alerts (last 10)
            if len(summary['recent_alerts']) < 10:
                summary['recent_alerts'].append({
                    'timestamp': alert.timestamp,
                    'level': severity,
                    'message': alert.message,
                    'metric': alert.metric
                })

        summary['by_severity'] = severity_counts
        summary['most_common_types'] = dict(sorted(type_counts.items(), key=lambda x: x[1], reverse=True)[:5])

        return summary

    def _assess_data_quality(self) -> Dict[str, Any]:
        """Assess quality of monitoring data."""
        quality = {
            'data_points_total': 0,
            'data_points_recent': 0,
            'missing_data_rate': 0.0,
            'outlier_rate': 0.0,
            'quality_score': 1.0
        }

        if POLARS_AVAILABLE and self.metrics_df:
            with self.metrics_lock:
                total_points = len(self.metrics_df)
                quality['data_points_total'] = total_points

                # Recent data (last 5 minutes)
                recent_cutoff = datetime.now() - timedelta(minutes=5)
                recent_data = self.metrics_df.filter(pl.col('timestamp') >= recent_cutoff)
                quality['data_points_recent'] = len(recent_data)

                if total_points > 0:
                    # Simple quality assessment
                    null_count = self.metrics_df.null_count().sum_horizontal()[0]
                    quality['missing_data_rate'] = null_count / (total_points * len(self.metrics_schema))

                    # Assess if we have diverse metrics
                    unique_metrics = self.metrics_df['metric_name'].n_unique()
                    quality['quality_score'] = min(1.0, unique_metrics / 10.0)  # Expect at least 10 different metrics

        return quality

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
                    writer.writerow(
                        [
                            "timestamp",
                            "memory_usage",
                            "cpu_usage",
                            "step_count",
                            "alerts",
                        ]
                    )

                    # Write data
                    for timestamp in sorted(self.metrics_history.keys()):
                        metrics = self.metrics_history[timestamp]
                        system = metrics.get("system", {})
                        simulation = metrics.get("simulation", {})

                        writer.writerow(
                            [
                                timestamp,
                                system.get("memory_usage_percent", 0),
                                system.get("cpu_usage_percent", 0),
                                simulation.get("simulation_step_count", 0),
                                len(
                                    [a for a in self.alerts if a.timestamp <= timestamp]
                                ),
                            ]
                        )

            self.logger.info("Metrics exported", filepath=str(path), format=format)
            return True

        except Exception as e:
            self.logger.error("Failed to export metrics", error=str(e))
            return False
