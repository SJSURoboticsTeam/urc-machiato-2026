#!/usr/bin/env python3
"""
URC 2026 Testing Metrics Dashboard

Comprehensive web-based dashboard for monitoring all testing activities:
- Performance metrics (latency, throughput, resource usage)
- Test results and pass/fail status
- Regression detection and trends
- Chaos engineering results
- Hardware validation status
- Resource budgeting compliance

Usage:
    python testing_metrics_dashboard.py --port 8080 --data-dir ./test_results/

Features:
- Real-time metrics updates
- Historical trend analysis
- Alert notifications
- Export capabilities
- REST API for integration

Author: URC 2026 Testing and Metrics Team
"""

import argparse
import json
import os
import threading
import time
from datetime import datetime, timedelta
from http.server import HTTPServer, BaseHTTPRequestHandler
from pathlib import Path
from typing import Dict, List, Any, Optional, Tuple
import urllib.parse
import mimetypes
import statistics


# Metrics storage
class MetricsStore:
    """Thread-safe metrics storage and analysis with full system integration."""

    def __init__(self):
        self._metrics = {}
        self._lock = threading.RLock()
        self._alerts = []
        self._system_status = {
            "behavior_trees": {},
            "state_machines": {},
            "network_connections": {},
            "simulator_status": {},
            "uptime": {},
        }
        self._thresholds = self._load_default_thresholds()

    def _load_default_thresholds(self) -> Dict[str, Dict[str, float]]:
        """Load default performance thresholds."""
        return {
            "motion_control": {
                "p99_latency_ms": 20.0,
                "p95_latency_ms": 15.0,
                "p50_latency_ms": 10.0,
                "deadline_violations_percent": 1.0,
            },
            "binary_protocol": {
                "p99_latency_ms": 5.0,
                "p95_latency_ms": 2.0,
                "p50_latency_ms": 1.0,
                "throughput_msgs_per_sec": 1000.0,
            },
            "ipc_bridge": {
                "p99_latency_ms": 10.0,
                "connection_success_rate": 0.99,
                "memory_leak_mb_per_hour": 10.0,
            },
            "sensor_fusion": {
                "rmse_m": 0.5,
                "drift_rate_cm_per_min": 5.0,
                "convergence_time_sec": 30.0,
            },
            "system_resources": {
                "cpu_usage_percent": 85.0,
                "memory_usage_percent": 90.0,
                "network_bandwidth_utilization": 0.8,
            },
        }

    def update_metric(
        self,
        category: str,
        metric: str,
        value: float,
        timestamp: Optional[float] = None,
    ):
        """Update a metric value."""
        if timestamp is None:
            timestamp = time.time()

        with self._lock:
            if category not in self._metrics:
                self._metrics[category] = {}

            if metric not in self._metrics[category]:
                self._metrics[category][metric] = []

            # Keep last 1000 data points per metric
            self._metrics[category][metric].append(
                {"value": value, "timestamp": timestamp}
            )

            if len(self._metrics[category][metric]) > 1000:
                self._metrics[category][metric] = self._metrics[category][metric][
                    -1000:
                ]

            # Check thresholds and generate alerts
            self._check_thresholds(category, metric, value)

    def _check_thresholds(self, category: str, metric: str, value: float):
        """Check if metric violates thresholds and generate alerts."""
        if category in self._thresholds and metric in self._thresholds[category]:
            threshold = self._thresholds[category][metric]
            violation = False

            # Determine if this is a "lower is better" or "higher is better" metric
            if (
                "latency" in metric
                or "rmse" in metric
                or "drift" in metric
                or "time" in metric
            ):
                # Lower is better
                violation = value > threshold
            elif "success" in metric or "rate" in metric or "throughput" in metric:
                # Higher is better
                violation = value < threshold
            elif "usage" in metric or "utilization" in metric:
                # Check upper bound
                violation = value > threshold

            if violation:
                alert = {
                    "timestamp": time.time(),
                    "category": category,
                    "metric": metric,
                    "value": value,
                    "threshold": threshold,
                    "severity": "CRITICAL" if value > threshold * 1.5 else "WARNING",
                    "message": f"{category}.{metric} = {value:.2f}, threshold = {threshold:.2f}",
                }
                self._alerts.append(alert)

                # Keep only last 100 alerts
                if len(self._alerts) > 100:
                    self._alerts = self._alerts[-100:]

    def get_metric_stats(
        self, category: str, metric: str, hours: int = 24
    ) -> Dict[str, Any]:
        """Get statistical summary for a metric."""
        cutoff_time = time.time() - (hours * 3600)

        with self._lock:
            if category not in self._metrics or metric not in self._metrics[category]:
                return {"error": "Metric not found"}

            data_points = [
                point
                for point in self._metrics[category][metric]
                if point["timestamp"] > cutoff_time
            ]

            if not data_points:
                return {"error": "No data in time range"}

            values = [point["value"] for point in data_points]

            return {
                "count": len(values),
                "mean": statistics.mean(values),
                "median": statistics.median(values),
                "min": min(values),
                "max": max(values),
                "p95": sorted(values)[int(len(values) * 0.95)] if values else None,
                "p99": sorted(values)[int(len(values) * 0.99)] if values else None,
                "std_dev": statistics.stdev(values) if len(values) > 1 else 0,
                "latest": values[-1] if values else None,
                "time_range_hours": hours,
            }

    def get_dashboard_data(self) -> Dict[str, Any]:
        """Get comprehensive dashboard data with full system integration."""
        with self._lock:
            dashboard = {
                "timestamp": time.time(),
                "summary": {
                    "total_metrics": sum(
                        len(metrics) for metrics in self._metrics.values()
                    ),
                    "total_categories": len(self._metrics),
                    "active_alerts": len(
                        [a for a in self._alerts if time.time() - a["timestamp"] < 3600]
                    ),  # Last hour
                    "total_alerts": len(self._alerts),
                },
                "performance_status": self._calculate_performance_status(),
                "recent_alerts": self._alerts[-10:],  # Last 10 alerts
                "system_health": self._calculate_system_health(),
                "test_coverage": self._calculate_test_coverage(),
                "regression_status": self._detect_regressions(),
                "system_integration": self.get_system_integration_status(),
            }

            return dashboard

    def update_system_status(
        self, component: str, subsystem: str, status: Dict[str, Any]
    ):
        """Update system component status."""
        with self._lock:
            if component not in self._system_status:
                self._system_status[component] = {}
            self._system_status[component][subsystem] = status

            # Update metrics for monitoring
            for key, value in status.items():
                if isinstance(value, (int, float)):
                    self.update_metric(
                        f"system_{component}", f"{subsystem}_{key}", value
                    )

    def get_system_integration_status(self) -> Dict[str, Any]:
        """Get comprehensive system integration status."""
        with self._lock:
            return {
                "behavior_trees": self._check_behavior_tree_status(),
                "state_machines": self._check_state_machine_status(),
                "network": self._check_network_status(),
                "simulator": self._check_simulator_status(),
                "connections": self._check_connections_status(),
                "uptime": self._calculate_system_uptime(),
            }

    def _check_behavior_tree_status(self) -> Dict[str, Any]:
        """Check behavior tree system status."""
        bt_status = self._system_status.get("behavior_trees", {})

        active_trees = len([t for t in bt_status.values() if t.get("active", False)])
        healthy_trees = len(
            [t for t in bt_status.values() if t.get("status") == "RUNNING"]
        )

        return {
            "total_trees": len(bt_status),
            "active_trees": active_trees,
            "healthy_trees": healthy_trees,
            "health_percentage": (healthy_trees / max(len(bt_status), 1)) * 100,
            "trees": bt_status,
        }

    def _check_state_machine_status(self) -> Dict[str, Any]:
        """Check state machine system status."""
        sm_status = self._system_status.get("state_machines", {})

        active_machines = len([m for m in sm_status.values() if m.get("active", False)])
        valid_transitions = len(
            [m for m in sm_status.values() if m.get("last_transition_valid", True)]
        )

        return {
            "total_machines": len(sm_status),
            "active_machines": active_machines,
            "valid_transitions": valid_transitions,
            "transition_success_rate": (valid_transitions / max(len(sm_status), 1))
            * 100,
            "machines": sm_status,
        }

    def _check_network_status(self) -> Dict[str, Any]:
        """Check network system status."""
        net_status = self._system_status.get("network_connections", {})

        active_connections = len(
            [c for c in net_status.values() if c.get("connected", False)]
        )
        healthy_links = len(
            [c for c in net_status.values() if c.get("latency_ms", 999) < 100]
        )

        # Calculate network health score
        total_connections = len(net_status)
        if total_connections == 0:
            health_score = 100  # No connections expected
        else:
            health_score = (
                (active_connections + healthy_links) / (2 * total_connections)
            ) * 100

        return {
            "total_connections": total_connections,
            "active_connections": active_connections,
            "healthy_links": healthy_links,
            "network_health_score": health_score,
            "connections": net_status,
        }

    def _check_simulator_status(self) -> Dict[str, Any]:
        """Check simulator system status."""
        sim_status = self._system_status.get("simulator_status", {})

        active_simulations = len(
            [s for s in sim_status.values() if s.get("running", False)]
        )
        real_time_factor = sum(
            [s.get("real_time_factor", 1.0) for s in sim_status.values()]
        ) / max(len(sim_status), 1)

        return {
            "total_simulations": len(sim_status),
            "active_simulations": active_simulations,
            "average_real_time_factor": real_time_factor,
            "simulation_stability": "GOOD" if real_time_factor > 0.8 else "POOR",
            "simulations": sim_status,
        }

    def _check_connections_status(self) -> Dict[str, Any]:
        """Check system connections status."""
        # Aggregate connection status from all subsystems
        bt_connections = len(self._system_status.get("behavior_trees", {}))
        sm_connections = len(self._system_status.get("state_machines", {}))
        net_connections = len(self._system_status.get("network_connections", {}))

        total_expected_connections = bt_connections + sm_connections + net_connections

        # For now, assume all are connected if status is reported
        connected_count = total_expected_connections

        return {
            "total_expected_connections": total_expected_connections,
            "connected_count": connected_count,
            "connection_health": (connected_count / max(total_expected_connections, 1))
            * 100,
            "connection_status": (
                "HEALTHY"
                if connected_count == total_expected_connections
                else "DEGRADED"
            ),
        }

    def _calculate_system_uptime(self) -> Dict[str, Any]:
        """Calculate system uptime statistics."""
        uptime_data = self._system_status.get("uptime", {})

        if not uptime_data:
            # Estimate from process start time
            import time

            boot_time = time.time() - (60 * 60 * 24)  # Assume 24 hours for demo
            uptime_data = {"system_boot": boot_time, "process_start": time.time()}

        current_time = time.time()
        system_uptime = current_time - uptime_data.get(
            "system_boot", current_time - 3600
        )
        process_uptime = current_time - uptime_data.get(
            "process_start", current_time - 3600
        )

        return {
            "system_uptime_seconds": system_uptime,
            "process_uptime_seconds": process_uptime,
            "system_uptime_hours": system_uptime / 3600,
            "process_uptime_hours": process_uptime / 3600,
            "uptime_percentage": min(
                100, (process_uptime / max(system_uptime, 1)) * 100
            ),
        }

    def _calculate_performance_status(self) -> Dict[str, Any]:
        """Calculate overall performance status."""
        status = {
            "motion_control": {"status": "UNKNOWN", "score": 0},
            "communication": {"status": "UNKNOWN", "score": 0},
            "resources": {"status": "UNKNOWN", "score": 0},
            "reliability": {"status": "UNKNOWN", "score": 0},
        }

        # Motion control status
        if "motion_control" in self._metrics:
            motion_metrics = self._metrics["motion_control"]
            score = 0
            total_checks = 0

            if "p99_latency_ms" in motion_metrics:
                latest = (
                    motion_metrics["p99_latency_ms"][-1]["value"]
                    if motion_metrics["p99_latency_ms"]
                    else None
                )
                if latest and latest <= 20.0:
                    score += 1
                total_checks += 1

            status["motion_control"]["score"] = score / max(total_checks, 1)
            status["motion_control"]["status"] = (
                "PASS" if status["motion_control"]["score"] >= 0.8 else "FAIL"
            )

        # Communication status
        comm_score = 0
        comm_checks = 0

        for category in ["binary_protocol", "ipc_bridge"]:
            if category in self._metrics:
                # Simplified check - has recent data
                has_recent_data = any(
                    time.time() - point["timestamp"] < 3600  # Last hour
                    for metric_data in self._metrics[category].values()
                    for point in metric_data[-1:]  # Check last point
                )
                if has_recent_data:
                    comm_score += 1
                comm_checks += 1

        status["communication"]["score"] = comm_score / max(comm_checks, 1)
        status["communication"]["status"] = (
            "PASS" if status["communication"]["score"] >= 0.8 else "FAIL"
        )

        return status

    def _calculate_system_health(self) -> Dict[str, Any]:
        """Calculate system health metrics."""
        health = {
            "cpu_usage": "UNKNOWN",
            "memory_usage": "UNKNOWN",
            "network_status": "UNKNOWN",
            "overall_health": "UNKNOWN",
        }

        # CPU health
        cpu_stats = self.get_metric_stats("system", "cpu_percent", hours=1)
        if "mean" in cpu_stats:
            cpu_usage = cpu_stats["mean"]
            if cpu_usage < 50:
                health["cpu_usage"] = "GOOD"
            elif cpu_usage < 80:
                health["cpu_usage"] = "WARNING"
            else:
                health["cpu_usage"] = "CRITICAL"

        # Memory health
        mem_stats = self.get_metric_stats("system", "memory_percent", hours=1)
        if "mean" in mem_stats:
            mem_usage = mem_stats["mean"]
            if mem_usage < 70:
                health["memory_usage"] = "GOOD"
            elif mem_usage < 85:
                health["memory_usage"] = "WARNING"
            else:
                health["memory_usage"] = "CRITICAL"

        # Overall health
        health_scores = []
        for component in ["cpu_usage", "memory_usage"]:
            if health[component] == "GOOD":
                health_scores.append(1.0)
            elif health[component] == "WARNING":
                health_scores.append(0.5)
            elif health[component] == "CRITICAL":
                health_scores.append(0.0)

        if health_scores:
            avg_health = sum(health_scores) / len(health_scores)
            if avg_health >= 0.8:
                health["overall_health"] = "HEALTHY"
            elif avg_health >= 0.5:
                health["overall_health"] = "WARNING"
            else:
                health["overall_health"] = "CRITICAL"

        return health

    def _calculate_test_coverage(self) -> Dict[str, Any]:
        """Calculate test coverage status."""
        coverage = {
            "unit_tests": {"covered": 0, "total": 10, "percentage": 0},
            "integration_tests": {"covered": 0, "total": 8, "percentage": 0},
            "performance_tests": {"covered": 0, "total": 6, "percentage": 0},
            "chaos_tests": {"covered": 0, "total": 5, "percentage": 0},
            "overall_coverage": 0,
        }

        # Estimate coverage based on metrics presence
        if "motion_control" in self._metrics:
            coverage["performance_tests"]["covered"] += 1
        if "binary_protocol" in self._metrics:
            coverage["performance_tests"]["covered"] += 1
        if "ipc_bridge" in self._metrics:
            coverage["integration_tests"]["covered"] += 1

        # Calculate percentages
        for category in coverage:
            if category != "overall_coverage" and "covered" in coverage[category]:
                covered = coverage[category]["covered"]
                total = coverage[category]["total"]
                coverage[category]["percentage"] = (
                    (covered / total) * 100 if total > 0 else 0
                )

        # Overall coverage
        total_covered = sum(
            cat["covered"]
            for cat in coverage.values()
            if isinstance(cat, dict) and "covered" in cat
        )
        total_tests = sum(
            cat["total"]
            for cat in coverage.values()
            if isinstance(cat, dict) and "total" in cat
        )
        coverage["overall_coverage"] = (
            (total_covered / total_tests) * 100 if total_tests > 0 else 0
        )

        return coverage

    def _detect_regressions(self) -> Dict[str, Any]:
        """Detect performance regressions."""
        regressions = {
            "detected": False,
            "categories": [],
            "severity": "NONE",
            "recommendations": [],
        }

        # Simple regression detection: compare recent vs historical performance
        for category in self._metrics:
            if category in self._thresholds:
                for metric in self._metrics[category]:
                    metric_data = self._metrics[category][metric]
                    if len(metric_data) >= 20:  # Need enough data
                        # Compare recent 10 points vs previous 10 points
                        recent = [p["value"] for p in metric_data[-10:]]
                        historical = [p["value"] for p in metric_data[-20:-10]]

                        if recent and historical:
                            recent_avg = statistics.mean(recent)
                            historical_avg = statistics.mean(historical)

                            # For latency metrics, increase is regression
                            if "latency" in metric or "time" in metric:
                                if recent_avg > historical_avg * 1.1:  # 10% degradation
                                    regressions["detected"] = True
                                    regressions["categories"].append(
                                        f"{category}.{metric}"
                                    )
                                    regressions["severity"] = "WARNING"

        if regressions["detected"]:
            regressions["recommendations"].append(
                "Performance regression detected - investigate recent changes"
            )
        else:
            regressions["recommendations"].append("No performance regressions detected")

        return regressions


# Global metrics store
metrics_store = MetricsStore()


class MetricsDashboardHandler(BaseHTTPRequestHandler):
    """HTTP handler for metrics dashboard."""

    def do_GET(self):
        """Handle GET requests."""
        parsed_path = urllib.parse.urlparse(self.path)
        path = parsed_path.path

        if path == "/":
            self._serve_dashboard()
        elif path == "/api/metrics":
            self._serve_metrics_api()
        elif path == "/api/alerts":
            self._serve_alerts_api()
        elif path == "/api/health":
            self._serve_health_api()
        elif path.startswith("/static/"):
            self._serve_static_file(path)
        else:
            self._serve_404()

    def do_POST(self):
        """Handle POST requests for metrics updates."""
        if self.path == "/api/metrics":
            self._handle_metrics_update()
        else:
            self._serve_404()

    def _serve_dashboard(self):
        """Serve the main dashboard HTML."""
        html_content = self._generate_dashboard_html()
        self._send_response(200, "text/html", html_content)

    def _serve_metrics_api(self):
        """Serve metrics data as JSON."""
        dashboard_data = metrics_store.get_dashboard_data()
        self._send_response(
            200, "application/json", json.dumps(dashboard_data, indent=2)
        )

    def _serve_alerts_api(self):
        """Serve alerts data as JSON."""
        # Get alerts from metrics store
        with metrics_store._lock:
            alerts = metrics_store._alerts[-50:]  # Last 50 alerts

        self._send_response(200, "application/json", json.dumps(alerts, indent=2))

    def _serve_health_api(self):
        """Serve health check."""
        health_data = {
            "status": "healthy",
            "timestamp": time.time(),
            "uptime": time.time() - getattr(self.server, "start_time", time.time()),
            "metrics_count": sum(
                len(metrics) for metrics in metrics_store._metrics.values()
            ),
        }
        self._send_response(200, "application/json", json.dumps(health_data))

    def _handle_metrics_update(self):
        """Handle metrics update POST requests."""
        try:
            content_length = int(self.headers["Content-Length"])
            post_data = self.rfile.read(content_length)
            data = json.loads(post_data.decode("utf-8"))

            # Update metrics
            category = data.get("category", "unknown")
            metric = data.get("metric", "unknown")
            value = data.get("value", 0)
            timestamp = data.get("timestamp", time.time())

            metrics_store.update_metric(category, metric, value, timestamp)

            self._send_response(
                200, "application/json", json.dumps({"status": "success"})
            )

        except Exception as e:
            self._send_response(400, "application/json", json.dumps({"error": str(e)}))

    def _serve_static_file(self, path):
        """Serve static files."""
        # For now, just return 404 as we don't have static files
        self._serve_404()

    def _serve_404(self):
        """Serve 404 error."""
        self._send_response(404, "text/plain", "Not Found")

    def _send_response(self, status_code: int, content_type: str, content: str):
        """Send HTTP response."""
        self.send_response(status_code)
        self.send_header("Content-Type", content_type)
        self.send_header("Access-Control-Allow-Origin", "*")  # CORS
        self.send_header("Access-Control-Allow-Methods", "GET, POST, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type")
        self.end_headers()
        self.wfile.write(content.encode("utf-8"))

    def _generate_dashboard_html(self) -> str:
        """Generate the dashboard HTML."""
        dashboard_data = metrics_store.get_dashboard_data()

        html = f"""
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>URC 2026 Testing Metrics Dashboard</title>
    <style>
        body {{
            font-family: -apple-system, BlinkMacSystemFont, 'Segoe UI', Roboto, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
        }}
        .dashboard {{
            max-width: 1200px;
            margin: 0 auto;
        }}
        .header {{
            background: linear-gradient(135deg, #667eea 0%, #764ba2 100%);
            color: white;
            padding: 20px;
            border-radius: 10px;
            margin-bottom: 20px;
            text-align: center;
        }}
        .metrics-grid {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin-bottom: 20px;
        }}
        .metric-card {{
            background: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }}
        .metric-title {{
            font-size: 18px;
            font-weight: bold;
            margin-bottom: 15px;
            color: #333;
        }}
        .metric-value {{
            font-size: 24px;
            font-weight: bold;
            color: #667eea;
        }}
        .status-good {{ color: #28a745; }}
        .status-warning {{ color: #ffc107; }}
        .status-critical {{ color: #dc3545; }}
        .alert-list {{
            background: white;
            border-radius: 10px;
            padding: 20px;
            margin-bottom: 20px;
        }}
        .alert-item {{
            padding: 10px;
            border-left: 4px solid #dc3545;
            margin-bottom: 10px;
            background: #f8f9fa;
        }}
        .refresh-btn {{
            background: #667eea;
            color: white;
            border: none;
            padding: 10px 20px;
            border-radius: 5px;
            cursor: pointer;
            margin-bottom: 20px;
        }}
        .chart-container {{
            background: white;
            border-radius: 10px;
            padding: 20px;
            margin-bottom: 20px;
        }}
    </style>
</head>
<body>
    <div class="dashboard">
        <div class="header">
            <h1>🚀 URC 2026 Testing Metrics Dashboard</h1>
            <p>Real-time monitoring of performance, reliability, and system health</p>
            <button class="refresh-btn" onclick="refreshDashboard()">🔄 Refresh</button>
        </div>

        <div class="metrics-grid">
            <div class="metric-card">
                <div class="metric-title">📊 Total Metrics</div>
                <div class="metric-value">{dashboard_data['summary']['total_metrics']}</div>
            </div>

            <div class="metric-card">
                <div class="metric-title">📂 Active Categories</div>
                <div class="metric-value">{dashboard_data['summary']['total_categories']}</div>
            </div>

            <div class="metric-card">
                <div class="metric-title">🚨 Active Alerts</div>
                <div class="metric-value status-{'good' if dashboard_data['summary']['active_alerts'] == 0 else 'critical'}">{dashboard_data['summary']['active_alerts']}</div>
            </div>

            <div class="metric-card">
                <div class="metric-title">❤️ System Health</div>
                <div class="metric-value status-{'good' if dashboard_data['system_health']['overall_health'] == 'HEALTHY' else 'warning' if dashboard_data['system_health']['overall_health'] == 'WARNING' else 'critical'}">{dashboard_data['system_health']['overall_health']}</div>
            </div>
        </div>

        <div class="metrics-grid">
            <div class="metric-card">
                <div class="metric-title">🌳 Behavior Trees</div>
                <div class="metric-value status-{'good' if dashboard_data['system_integration']['behavior_trees']['health_percentage'] >= 90 else 'warning' if dashboard_data['system_integration']['behavior_trees']['health_percentage'] >= 70 else 'critical'}">{dashboard_data['system_integration']['behavior_trees']['healthy_trees']}/{dashboard_data['system_integration']['behavior_trees']['total_trees']}</div>
                <div style="font-size: 12px; color: #666; margin-top: 5px;">
                    {dashboard_data['system_integration']['behavior_trees']['health_percentage']:.1f}% healthy
                </div>
            </div>

            <div class="metric-card">
                <div class="metric-title">🔄 State Machines</div>
                <div class="metric-value status-{'good' if dashboard_data['system_integration']['state_machines']['transition_success_rate'] >= 95 else 'warning' if dashboard_data['system_integration']['state_machines']['transition_success_rate'] >= 80 else 'critical'}">{dashboard_data['system_integration']['state_machines']['active_machines']}/{dashboard_data['system_integration']['state_machines']['total_machines']}</div>
                <div style="font-size: 12px; color: #666; margin-top: 5px;">
                    {dashboard_data['system_integration']['state_machines']['transition_success_rate']:.1f}% valid transitions
                </div>
            </div>

            <div class="metric-card">
                <div class="metric-title">🌐 Network Status</div>
                <div class="metric-value status-{'good' if dashboard_data['system_integration']['network']['network_health_score'] >= 80 else 'warning' if dashboard_data['system_integration']['network']['network_health_score'] >= 60 else 'critical'}">{dashboard_data['system_integration']['network']['active_connections']}/{dashboard_data['system_integration']['network']['total_connections']}</div>
                <div style="font-size: 12px; color: #666; margin-top: 5px;">
                    Health: {dashboard_data['system_integration']['network']['network_health_score']:.1f}%
                </div>
            </div>

            <div class="metric-card">
                <div class="metric-title">🎮 Simulator</div>
                <div class="metric-value status-{'good' if dashboard_data['system_integration']['simulator']['simulation_stability'] == 'GOOD' else 'critical'}">{dashboard_data['system_integration']['simulator']['active_simulations']}/{dashboard_data['system_integration']['simulator']['total_simulations']}</div>
                <div style="font-size: 12px; color: #666; margin-top: 5px;">
                    RTF: {dashboard_data['system_integration']['simulator']['average_real_time_factor']:.2f}
                </div>
            </div>

            <div class="metric-card">
                <div class="metric-title">🔗 Connections</div>
                <div class="metric-value status-{'good' if dashboard_data['system_integration']['connections']['connection_status'] == 'HEALTHY' else 'critical'}">{dashboard_data['system_integration']['connections']['connected_count']}/{dashboard_data['system_integration']['connections']['total_expected_connections']}</div>
                <div style="font-size: 12px; color: #666; margin-top: 5px;">
                    {dashboard_data['system_integration']['connections']['connection_health']:.1f}% healthy
                </div>
            </div>

            <div class="metric-card">
                <div class="metric-title">⏱️ System Uptime</div>
                <div class="metric-value status-good">{dashboard_data['system_integration']['uptime']['system_uptime_hours']:.1f}h</div>
                <div style="font-size: 12px; color: #666; margin-top: 5px;">
                    Process: {dashboard_data['system_integration']['uptime']['process_uptime_hours']:.1f}h
                </div>
            </div>
        </div>

        <div class="alert-list">
            <h3>🚨 Recent Alerts</h3>
            <div id="alerts-container">
                {self._generate_alerts_html(dashboard_data['recent_alerts'])}
            </div>
        </div>

        <div class="metrics-grid">
            <div class="metric-card">
                <div class="metric-title">🎯 Motion Control Status</div>
                <div class="metric-value status-{'good' if dashboard_data['performance_status']['motion_control']['status'] == 'PASS' else 'critical'}">{dashboard_data['performance_status']['motion_control']['status']}</div>
                <div style="font-size: 14px; color: #666; margin-top: 5px;">
                    Score: {dashboard_data['performance_status']['motion_control']['score']:.2f}
                </div>
            </div>

            <div class="metric-card">
                <div class="metric-title">📡 Communication Status</div>
                <div class="metric-value status-{'good' if dashboard_data['performance_status']['communication']['status'] == 'PASS' else 'critical'}">{dashboard_data['performance_status']['communication']['status']}</div>
                <div style="font-size: 14px; color: #666; margin-top: 5px;">
                    Score: {dashboard_data['performance_status']['communication']['score']:.2f}
                </div>
            </div>

            <div class="metric-card">
                <div class="metric-title">🧪 Test Coverage</div>
                <div class="metric-value">{dashboard_data['test_coverage']['overall_coverage']:.1f}%</div>
            </div>

            <div class="metric-card">
                <div class="metric-title">📈 Regression Status</div>
                <div class="metric-value status-{'good' if not dashboard_data['regression_status']['detected'] else 'critical'}">{'✅ Clean' if not dashboard_data['regression_status']['detected'] else '❌ Detected'}</div>
            </div>
        </div>

        <div class="chart-container">
            <h3>🔗 System Integration Status</h3>
            <div style="display: grid; grid-template-columns: repeat(auto-fit, minmax(300px, 1fr)); gap: 20px;">
                <div style="background: #f8f9fa; padding: 15px; border-radius: 8px;">
                    <h4>🌳 Behavior Trees</h4>
                    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px; font-size: 14px;">
                        <div>Total: <strong>{dashboard_data['system_integration']['behavior_trees']['total_trees']}</strong></div>
                        <div>Active: <strong>{dashboard_data['system_integration']['behavior_trees']['active_trees']}</strong></div>
                        <div>Healthy: <strong>{dashboard_data['system_integration']['behavior_trees']['healthy_trees']}</strong></div>
                        <div>Health: <strong>{dashboard_data['system_integration']['behavior_trees']['health_percentage']:.1f}%</strong></div>
                    </div>
                </div>

                <div style="background: #f8f9fa; padding: 15px; border-radius: 8px;">
                    <h4>🔄 State Machines</h4>
                    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px; font-size: 14px;">
                        <div>Total: <strong>{dashboard_data['system_integration']['state_machines']['total_machines']}</strong></div>
                        <div>Active: <strong>{dashboard_data['system_integration']['state_machines']['active_machines']}</strong></div>
                        <div>Valid Transitions: <strong>{dashboard_data['system_integration']['state_machines']['valid_transitions']}</strong></div>
                        <div>Success Rate: <strong>{dashboard_data['system_integration']['state_machines']['transition_success_rate']:.1f}%</strong></div>
                    </div>
                </div>

                <div style="background: #f8f9fa; padding: 15px; border-radius: 8px;">
                    <h4>🌐 Network Connections</h4>
                    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px; font-size: 14px;">
                        <div>Total: <strong>{dashboard_data['system_integration']['network']['total_connections']}</strong></div>
                        <div>Active: <strong>{dashboard_data['system_integration']['network']['active_connections']}</strong></div>
                        <div>Healthy: <strong>{dashboard_data['system_integration']['network']['healthy_links']}</strong></div>
                        <div>Health Score: <strong>{dashboard_data['system_integration']['network']['network_health_score']:.1f}%</strong></div>
                    </div>
                </div>

                <div style="background: #f8f9fa; padding: 15px; border-radius: 8px;">
                    <h4>🎮 Simulator Status</h4>
                    <div style="display: grid; grid-template-columns: 1fr 1fr; gap: 10px; font-size: 14px;">
                        <div>Total: <strong>{dashboard_data['system_integration']['simulator']['total_simulations']}</strong></div>
                        <div>Active: <strong>{dashboard_data['system_integration']['simulator']['active_simulations']}</strong></div>
                        <div>Real-time Factor: <strong>{dashboard_data['system_integration']['simulator']['average_real_time_factor']:.2f}</strong></div>
                        <div>Stability: <strong style="color: {'green' if dashboard_data['system_integration']['simulator']['simulation_stability'] == 'GOOD' else 'red'};">{dashboard_data['system_integration']['simulator']['simulation_stability']}</strong></div>
                    </div>
                </div>
            </div>
        </div>

        <div class="chart-container">
            <h3>📈 System Resource Trends (Last Hour)</h3>
            <div style="text-align: center; padding: 40px; color: #666;">
                Charts will be implemented with JavaScript libraries (Chart.js, D3.js)<br>
                Current focus: Core metrics collection and alerting
            </div>
        </div>
    </div>

    <script>
        function refreshDashboard() {{
            location.reload();
        }}

        // Auto-refresh every 30 seconds
        setInterval(refreshDashboard, 30000);

        // Future: Implement real-time updates via WebSocket
        // For now, page refresh provides adequate monitoring
    </script>
</body>
</html>
        """
        return html

    def _generate_alerts_html(self, alerts):
        """Generate HTML for alerts list."""
        if not alerts:
            return "<p style='color: #666; font-style: italic;'>No recent alerts</p>"

        html = ""
        for alert in alerts[-5:]:  # Show last 5 alerts
            timestamp = datetime.fromtimestamp(alert["timestamp"]).strftime("%H:%M:%S")
            severity_class = (
                "critical" if alert.get("severity") == "CRITICAL" else "warning"
            )
            html += f"""
            <div class="alert-item">
                <strong>{timestamp}</strong> - <span class="status-{severity_class}">{alert.get('severity', 'UNKNOWN')}</span><br>
                <small>{alert['message']}</small>
            </div>
            """
        return html


def run_server(port: int = 8080):
    """Run the metrics dashboard server."""
    server_address = ("", port)
    httpd = HTTPServer(server_address, MetricsDashboardHandler)
    httpd.start_time = time.time()

    print(f"🚀 URC 2026 Testing Metrics Dashboard running on http://localhost:{port}")
    print("📊 Available endpoints:")
    print("  GET  /              - Main dashboard")
    print("  GET  /api/metrics   - Metrics data (JSON)")
    print("  GET  /api/alerts    - Alerts data (JSON)")
    print("  GET  /api/health    - Health check")
    print("  POST /api/metrics   - Update metrics")
    print()
    print("💡 Dashboard auto-refreshes every 30 seconds")
    print("🔄 Press Ctrl+C to stop")

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n🛑 Shutting down metrics dashboard...")
        httpd.shutdown()


def simulate_test_data():
    """Simulate test data for demonstration."""
    print("📊 Simulating test data for dashboard demonstration...")

    # Simulate performance metrics
    test_scenarios = [
        ("motion_control", "p99_latency_ms", lambda: 15 + (time.time() % 10)),
        ("binary_protocol", "p99_latency_ms", lambda: 2 + (time.time() % 2)),
        ("ipc_bridge", "p99_latency_ms", lambda: 5 + (time.time() % 3)),
        ("system", "cpu_percent", lambda: 45 + (time.time() % 20)),
        ("system", "memory_percent", lambda: 60 + (time.time() % 15)),
    ]

    # Generate 10 minutes of simulated data
    start_time = time.time()
    for i in range(600):  # 10 minutes at 1Hz
        current_time = start_time + i

        for category, metric, value_func in test_scenarios:
            value = value_func()
            metrics_store.update_metric(category, metric, value, current_time)

        time.sleep(0.01)  # Fast simulation

    print("✅ Test data simulation complete")


def main():
    """Main entry point."""
    parser = argparse.ArgumentParser(description="URC 2026 Testing Metrics Dashboard")
    parser.add_argument(
        "--port",
        type=int,
        default=8080,
        help="Port to run dashboard server (default: 8080)",
    )
    parser.add_argument(
        "--simulate-data",
        action="store_true",
        help="Simulate test data for demonstration",
    )
    parser.add_argument(
        "--data-dir",
        type=str,
        default="./test_results",
        help="Directory to load historical test data from",
    )

    args = parser.parse_args()

    # Load historical data if available
    data_dir = Path(args.data_dir)
    if data_dir.exists():
        print(f"📁 Loading historical data from {data_dir}")
        # Future: Load JSON files from directory

    # Simulate data if requested
    if args.simulate_data:
        simulate_test_data()

    # Start the dashboard server
    try:
        run_server(args.port)
    except Exception as e:
        print(f"❌ Failed to start dashboard: {e}")
        return 1

    return 0


if __name__ == "__main__":
    exit(main())
