#!/usr/bin/env python3
"""
System Monitor - URC 2026 Rover

Real-time monitoring and alerting system for rover operations.
Tracks performance metrics, system health, and provides alerts.

Features:
- Real-time performance monitoring
- Health status tracking
- Alert generation and escalation
- Metrics collection and reporting
- Automated diagnostics
"""

import json
import threading
import time
from collections import deque
from datetime import datetime
from typing import Any, Dict, List, Optional

import psutil


class SystemMonitor:
    """Main system monitoring and alerting coordinator."""

    def __init__(self):
        self.metrics = {}
        self.alerts = deque(maxlen=100)  # Keep last 100 alerts
        self.health_status = {}
        self.thresholds = self._load_default_thresholds()
        self.monitoring_active = False
        self.monitor_thread = None

    def _load_default_thresholds(self) -> Dict[str, Any]:
        """Load default monitoring thresholds."""
        return {
            "cpu_percent": {"warning": 70, "critical": 90},
            "memory_percent": {"warning": 75, "critical": 90},
            "disk_percent": {"warning": 80, "critical": 95},
            "network_latency_ms": {"warning": 100, "critical": 500},
            "ros_topic_hz_min": {"warning": 5, "critical": 1},  # Minimum Hz
            "mission_timeout_min": {"warning": 15, "critical": 30},  # Minutes
        }

    def start_monitoring(self):
        """Start the monitoring system."""
        if self.monitoring_active:
            return

        self.monitoring_active = True
        self.monitor_thread = threading.Thread(
            target=self._monitoring_loop, daemon=True
        )
        self.monitor_thread.start()

        self._log_event("info", "System monitoring started")

    def stop_monitoring(self):
        """Stop the monitoring system."""
        self.monitoring_active = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=5.0)

        self._log_event("info", "System monitoring stopped")

    def _monitoring_loop(self):
        """Main monitoring loop."""
        while self.monitoring_active:
            try:
                self._collect_system_metrics()
                self._check_health_status()
                self._process_alerts()

                time.sleep(5.0)  # Monitor every 5 seconds

            except Exception as e:
                self._generate_alert(
                    "critical", "monitoring_error", f"Monitoring loop error: {e}"
                )
                time.sleep(10.0)  # Back off on errors

    def _collect_system_metrics(self):
        """Collect current system metrics."""
        timestamp = datetime.now().isoformat()

        # CPU metrics
        self.metrics["cpu"] = {
            "timestamp": timestamp,
            "percent": psutil.cpu_percent(interval=1),
            "count": psutil.cpu_count(),
            "frequency": psutil.cpu_freq().current if psutil.cpu_freq() else None,
        }

        # Memory metrics
        memory = psutil.virtual_memory()
        self.metrics["memory"] = {
            "timestamp": timestamp,
            "total_gb": memory.total / (1024**3),
            "used_gb": memory.used / (1024**3),
            "percent": memory.percent,
            "available_gb": memory.available / (1024**3),
        }

        # Disk metrics
        disk = psutil.disk_usage("/")
        self.metrics["disk"] = {
            "timestamp": timestamp,
            "total_gb": disk.total / (1024**3),
            "used_gb": disk.used / (1024**3),
            "percent": disk.percent,
            "free_gb": disk.free / (1024**3),
        }

        # Network metrics
        network = psutil.net_io_counters()
        self.metrics["network"] = {
            "timestamp": timestamp,
            "bytes_sent": network.bytes_sent,
            "bytes_recv": network.bytes_recv,
            "packets_sent": network.packets_sent,
            "packets_recv": network.packets_recv,
            "errin": network.errin,
            "errout": network.errout,
        }

    def _check_health_status(self):
        """Check system health against thresholds."""
        # CPU health
        cpu_percent = self.metrics.get("cpu", {}).get("percent", 0)
        if cpu_percent >= self.thresholds["cpu_percent"]["critical"]:
            self._generate_alert("critical", "high_cpu", f"CPU usage: {cpu_percent}%")
        elif cpu_percent >= self.thresholds["cpu_percent"]["warning"]:
            self._generate_alert("warning", "high_cpu", f"CPU usage: {cpu_percent}%")

        # Memory health
        memory_percent = self.metrics.get("memory", {}).get("percent", 0)
        if memory_percent >= self.thresholds["memory_percent"]["critical"]:
            self._generate_alert(
                "critical", "high_memory", f"Memory usage: {memory_percent}%"
            )
        elif memory_percent >= self.thresholds["memory_percent"]["warning"]:
            self._generate_alert(
                "warning", "high_memory", f"Memory usage: {memory_percent}%"
            )

        # Disk health
        disk_percent = self.metrics.get("disk", {}).get("percent", 0)
        if disk_percent >= self.thresholds["disk_percent"]["critical"]:
            self._generate_alert("critical", "low_disk", f"Disk usage: {disk_percent}%")
        elif disk_percent >= self.thresholds["disk_percent"]["warning"]:
            self._generate_alert("warning", "low_disk", f"Disk usage: {disk_percent}%")

    def _generate_alert(self, severity: str, alert_type: str, message: str):
        """Generate an alert."""
        alert = {
            "timestamp": datetime.now().isoformat(),
            "severity": severity,  # 'info', 'warning', 'critical'
            "type": alert_type,
            "message": message,
            "acknowledged": False,
        }

        self.alerts.append(alert)
        self._log_event(severity, f"Alert generated: {alert_type} - {message}")

        # In production, this would trigger notifications
        if severity == "critical":
            self._escalate_critical_alert(alert)

    def _escalate_critical_alert(self, alert: Dict[str, Any]):
        """Escalate critical alerts."""
        # In production, this would:
        # - Send SMS/email notifications
        # - Trigger automated responses
        # - Log to external monitoring systems
        self._log_event("critical", f"CRITICAL ALERT ESCALATION: {alert['message']}")

    def _process_alerts(self):
        """Process and clean up old alerts."""
        current_time = datetime.now()

        # Mark old alerts as resolved if they're not critical
        for alert in list(self.alerts):
            alert_time = datetime.fromisoformat(alert["timestamp"])
            age_minutes = (current_time - alert_time).total_seconds() / 60

            # Auto-resolve non-critical alerts after 30 minutes
            if (
                not alert["acknowledged"]
                and alert["severity"] != "critical"
                and age_minutes > 30
            ):
                alert["resolved"] = True
                alert["resolved_at"] = current_time.isoformat()

    def _log_event(self, level: str, message: str):
        """Log monitoring events."""
        timestamp = datetime.now().isoformat()
        log_entry = f"[{timestamp}] [{level.upper()}] {message}"

        # In production, this would write to log files and external systems
        print(log_entry)

    def get_system_status(self) -> Dict[str, Any]:
        """Get current system status overview."""
        return {
            "timestamp": datetime.now().isoformat(),
            "metrics": self.metrics,
            "health_score": self._calculate_health_score(),
            "active_alerts": [a for a in self.alerts if not a.get("resolved", False)],
            "recent_alerts": list(self.alerts)[-10:],  # Last 10 alerts
        }

    def _calculate_health_score(self) -> float:
        """Calculate overall system health score (0-100)."""
        # Simple health scoring based on current metrics
        score = 100.0

        # CPU impact
        cpu_percent = self.metrics.get("cpu", {}).get("percent", 0)
        if cpu_percent > 90:
            score -= 30
        elif cpu_percent > 70:
            score -= 10

        # Memory impact
        memory_percent = self.metrics.get("memory", {}).get("percent", 0)
        if memory_percent > 90:
            score -= 30
        elif memory_percent > 75:
            score -= 10

        # Alert impact
        active_critical = sum(
            1
            for a in self.alerts
            if a["severity"] == "critical" and not a.get("resolved", False)
        )
        score -= active_critical * 5

        return max(0.0, min(100.0, score))

    def acknowledge_alert(self, alert_index: int):
        """Acknowledge an alert to prevent further notifications."""
        if 0 <= alert_index < len(self.alerts):
            self.alerts[alert_index]["acknowledged"] = True
            self.alerts[alert_index]["acknowledged_at"] = datetime.now().isoformat()

    def export_metrics(self, format: str = "json") -> str:
        """Export metrics in specified format."""
        if format == "json":
            return json.dumps(
                {
                    "system_status": self.get_system_status(),
                    "alerts": list(self.alerts),
                    "thresholds": self.thresholds,
                },
                indent=2,
                default=str,
            )
        elif format == "prometheus":
            # Prometheus format for integration with monitoring systems
            lines = []
            for metric_name, metric_data in self.metrics.items():
                if isinstance(metric_data, dict):
                    for key, value in metric_data.items():
                        if isinstance(value, (int, float)):
                            lines.append(f"{metric_name}_{key} {value}")
            return "\n".join(lines)

        return "Unsupported format"


class ComponentMonitor:
    """Monitor specific rover components."""

    def __init__(self, system_monitor: SystemMonitor):
        self.system_monitor = system_monitor
        self.component_health = {}

    def monitor_mission_system(self):
        """Monitor mission execution system."""
        # In production, this would check:
        # - Active mission status
        # - Waypoint progress
        # - Mission timeouts
        # - Success/failure rates

        # Placeholder for demonstration
        self.component_health["mission_system"] = {
            "status": "healthy",
            "active_missions": 0,
            "completed_missions": 0,
            "failed_missions": 0,
        }

    def monitor_navigation_system(self):
        """Monitor navigation and localization."""
        # In production, this would check:
        # - GPS signal strength
        # - SLAM quality metrics
        # - Path planning success rate
        # - Odometry drift

        self.component_health["navigation_system"] = {
            "status": "healthy",
            "gps_satellites": 8,
            "slam_quality": 0.95,
            "localization_confidence": 0.92,
        }

    def monitor_safety_system(self):
        """Monitor safety and emergency systems."""
        # In production, this would check:
        # - E-stop status
        # - Safety sensor health
        # - Emergency recovery readiness

        self.component_health["safety_system"] = {
            "status": "healthy",
            "emergency_stops": 0,
            "safety_sensors_active": 6,
            "recovery_systems_ready": True,
        }


def main():
    """Main monitoring system entry point."""
    import argparse

    parser = argparse.ArgumentParser(description="URC 2026 System Monitor")
    parser.add_argument("--start", action="store_true", help="Start monitoring")
    parser.add_argument("--stop", action="store_true", help="Stop monitoring")
    parser.add_argument("--status", action="store_true", help="Show current status")
    parser.add_argument(
        "--export", choices=["json", "prometheus"], help="Export metrics"
    )

    args = parser.parse_args()

    monitor = SystemMonitor()

    if args.start:
        monitor.start_monitoring()
        print("System monitoring started. Press Ctrl+C to stop.")
        try:
            while True:
                time.sleep(1)
        except KeyboardInterrupt:
            monitor.stop_monitoring()
            print("\nMonitoring stopped.")

    elif args.stop:
        monitor.stop_monitoring()
        print("Monitoring stopped.")

    elif args.export:
        if not monitor.monitoring_active:
            print("Monitoring not active. Start monitoring first with --start")
            return
        print(monitor.export_metrics(args.export))

    else:  # status
        if monitor.monitoring_active:
            status = monitor.get_system_status()
            print(json.dumps(status, indent=2, default=str))
        else:
            print("Monitoring not active. Use --start to begin monitoring.")


if __name__ == "__main__":
    main()
