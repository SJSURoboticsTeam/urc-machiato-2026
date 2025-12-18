#!/usr/bin/env python3
"""
Central Logging System for URC 2026 Rover

Provides structured logging, aggregation, correlation, and health monitoring
across all system components.
"""

import structlog
import logging
import json
import time
import threading
from collections import deque
from typing import Dict, List, Any, Optional
import uuid
import re


class CentralLogger:
    """
    Centralized logging system with correlation and aggregation capabilities.

    Features:
    - Structured logging with correlation IDs
    - Log aggregation and analysis
    - Alert generation and escalation
    - Component health monitoring
    - Performance metrics tracking
    """

    def __init__(self):
        self.setup_structured_logging()

        # Log storage and correlation
        self.log_buffer = deque(maxlen=10000)  # Keep last 10k logs
        self.correlation_map: Dict[str, List[Dict]] = {}
        self.alert_patterns = self._load_alert_patterns()

        # Health monitoring
        self.component_health: Dict[str, float] = {}
        self.error_rates: Dict[str, float] = {}

        # Cleanup thread
        self.cleanup_thread = threading.Thread(target=self._cleanup_old_correlations, daemon=True)
        self.cleanup_thread.start()

    def setup_structured_logging(self):
        """Configure structured logging."""
        structlog.configure(
            processors=[
                structlog.contextvars.merge_contextvars,
                structlog.processors.add_log_level,
                structlog.processors.TimeStamper(fmt="iso"),
                self._add_correlation_id,
                structlog.processors.JSONRenderer()
            ],
            wrapper_class=structlog.make_filtering_bound_logger(logging.INFO),
            context_class=dict,
            logger_factory=structlog.WriteLoggerFactory(),
            cache_logger_on_first_use=True,
        )

        self.logger = structlog.get_logger()

    def _add_correlation_id(self, logger, method_name, event_dict):
        """Add correlation ID to log entries."""
        if 'correlation_id' not in event_dict:
            event_dict['correlation_id'] = str(uuid.uuid4())
        return event_dict

    def _load_alert_patterns(self) -> Dict[str, str]:
        """Load patterns that trigger alerts."""
        return {
            r"connection.*failed": "CONNECTION_ISSUE",
            r"timeout.*exceeded": "TIMEOUT_ISSUE",
            r"memory.*leak": "RESOURCE_ISSUE",
            r"sensor.*failed": "HARDWARE_ISSUE",
            r"critical.*error": "CRITICAL_SYSTEM_ERROR",
            r"emergency.*stop": "SAFETY_SYSTEM_TRIGGERED",
            r"power.*critical": "POWER_SYSTEM_ISSUE"
        }

    def log_with_correlation(self, level: str, message: str,
                           correlation_id: str = None, **context) -> str:
        """Log with correlation ID and structured context."""
        if correlation_id is None:
            correlation_id = str(uuid.uuid4())

        log_data = {
            "correlation_id": correlation_id,
            "component": context.get("component", "unknown"),
            "operation": context.get("operation", "unknown"),
            "timestamp": time.time(),
            **context
        }

        # Store log for aggregation
        self.log_buffer.append({
            "level": level,
            "message": message,
            "correlation_id": correlation_id,
            **log_data
        })

        # Check for alerts
        full_message = f"{message} {' '.join(str(v) for v in context.values())}"
        self._check_alert_patterns(full_message, correlation_id)

        # Log using structlog
        if level == "error":
            self.logger.error(message, **log_data)
        elif level == "warning":
            self.logger.warning(message, **log_data)
        elif level == "info":
            self.logger.info(message, **log_data)
        else:
            self.logger.debug(message, **log_data)

        return correlation_id

    def _check_alert_patterns(self, message: str, correlation_id: str):
        """Check message against alert patterns."""
        for pattern, alert_type in self.alert_patterns.items():
            if re.search(pattern, message, re.IGNORECASE):
                self._trigger_alert(alert_type, {
                    "correlation_id": correlation_id,
                    "message": message,
                    "timestamp": time.time()
                })
                break

    def _trigger_alert(self, alert_type: str, context: Dict[str, Any]):
        """Trigger an alert for critical issues."""
        alert = {
            "type": alert_type,
            "severity": "high" if alert_type in ["CRITICAL_SYSTEM_ERROR", "SAFETY_SYSTEM_TRIGGERED"] else "medium",
            "timestamp": time.time(),
            "correlation_id": context.get("correlation_id"),
            "message": context.get("message", "Alert triggered")
        }

        # Log alert
        self.logger.error("ALERT_TRIGGERED", alert_type=alert_type, **context)

        # Store alert for escalation (could integrate with external systems)
        self.get_logger().info(f"[ALERT] ALERT: {alert_type} - {context.get('message', 'Unknown issue')}")
    def get_correlated_logs(self, correlation_id: str) -> List[Dict]:
        """Get all logs for a correlation ID."""
        return [log for log in self.log_buffer if log.get("correlation_id") == correlation_id]

    def analyze_component_health(self) -> Dict[str, float]:
        """Analyze component health from recent logs."""
        # Analyze last hour of logs
        cutoff_time = time.time() - 3600
        recent_logs = [log for log in self.log_buffer if log.get("timestamp", 0) > cutoff_time]

        health_scores = {}

        # Group by component
        components = {}
        for log in recent_logs:
            component = log.get("component", "unknown")
            if component not in components:
                components[component] = []
            components[component].append(log)

        # Calculate health score per component
        for component, logs in components.items():
            total_logs = len(logs)
            error_logs = len([log for log in logs if log.get("level") in ["error", "critical"]])

            if total_logs > 0:
                error_rate = error_logs / total_logs
                # Convert to health score (0-100, higher is better)
                health_scores[component] = max(0, 100 - (error_rate * 200))
            else:
                health_scores[component] = 100  # No logs = assume healthy

        return health_scores

    def get_system_status(self) -> Dict[str, Any]:
        """Get overall system status from logs."""
        health_scores = self.analyze_component_health()

        # Calculate overall system health
        if health_scores:
            overall_health = sum(health_scores.values()) / len(health_scores)
        else:
            overall_health = 100

        # Get recent alerts
        recent_alerts = []
        cutoff_time = time.time() - 300  # Last 5 minutes
        for log in self.log_buffer:
            if (log.get("level") in ["error", "critical"] and
                log.get("timestamp", 0) > cutoff_time):
                recent_alerts.append({
                    "timestamp": log.get("timestamp"),
                    "component": log.get("component"),
                    "message": log.get("message"),
                    "correlation_id": log.get("correlation_id")
                })

        return {
            "overall_health": overall_health,
            "component_health": health_scores,
            "recent_alerts": recent_alerts[-10:],  # Last 10 alerts
            "total_logs": len(self.log_buffer),
            "active_correlations": len(self.correlation_map)
        }

    def _cleanup_old_correlations(self):
        """Clean up old correlation data."""
        while True:
            try:
                # Clean old correlations (keep last 24 hours)
                cutoff = time.time() - 86400
                self.correlation_map = {
                    k: v for k, v in self.correlation_map.items()
                    if v and v[-1].get("timestamp", 0) > cutoff
                }

                # Clean old logs (keep last 24 hours)
                self.log_buffer = deque(
                    [log for log in self.log_buffer if log.get("timestamp", 0) > cutoff],
                    maxlen=10000
                )

                time.sleep(300)  # Clean every 5 minutes

            except Exception as e:
                self.logger.error("Error during log cleanup", error=str(e))
                time.sleep(60)


# Global logger instance
central_logger = CentralLogger()


def get_logger(component: str = "unknown"):
    """Get a component-specific logger."""
    return ComponentLogger(component, central_logger)


class ComponentLogger:
    """Component-specific logger that integrates with central logging."""

    def __init__(self, component: str, central_logger: CentralLogger):
        self.component = component
        self.central_logger = central_logger
        self.current_correlation_id = None

    def set_correlation_id(self, correlation_id: str):
        """Set correlation ID for this component."""
        self.current_correlation_id = correlation_id

    def info(self, message: str, **context):
        """Log info message."""
        # Remove correlation_id from context if it exists to avoid conflict
        context.pop('correlation_id', None)
        correlation_id = self.central_logger.log_with_correlation(
            "info", message,
            correlation_id=self.current_correlation_id,
            component=self.component,
            **context
        )
        if not self.current_correlation_id:
            self.current_correlation_id = correlation_id

    def warning(self, message: str, **context):
        """Log warning message."""
        context.pop('correlation_id', None)
        correlation_id = self.central_logger.log_with_correlation(
            "warning", message,
            correlation_id=self.current_correlation_id,
            component=self.component,
            **context
        )
        if not self.current_correlation_id:
            self.current_correlation_id = correlation_id

    def error(self, message: str, **context):
        """Log error message."""
        context.pop('correlation_id', None)
        correlation_id = self.central_logger.log_with_correlation(
            "error", message,
            correlation_id=self.current_correlation_id,
            component=self.component,
            **context
        )
        if not self.current_correlation_id:
            self.current_correlation_id = correlation_id

    def critical(self, message: str, **context):
        """Log critical message."""
        context.pop('correlation_id', None)
        correlation_id = self.central_logger.log_with_correlation(
            "error", message,
            correlation_id=self.current_correlation_id,
            component=self.component,
            level="critical",
            **context
        )
        if not self.current_correlation_id:
            self.current_correlation_id = correlation_id
