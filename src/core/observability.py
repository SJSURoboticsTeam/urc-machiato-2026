#!/usr/bin/env python3
"""
URC 2026 Observability Module

Provides comprehensive monitoring, metrics, and tracing capabilities using industry-standard libraries:
- Prometheus client for metrics collection
- OpenTelemetry for distributed tracing
- Structured logging integration
- Health checks and alerting

Author: URC 2026 Team
"""

import time
import threading
import psutil
from typing import Dict, Any, Optional, Callable, List
from dataclasses import dataclass
from enum import Enum
import logging

# Optional dependencies - graceful degradation if not available
try:
    from prometheus_client import Counter, Gauge, Histogram, start_http_server, REGISTRY
    PROMETHEUS_AVAILABLE = True
except ImportError:
    PROMETHEUS_AVAILABLE = False
    # Mock prometheus classes
    class Counter:
        def __init__(self, *args, **kwargs): pass
        def inc(self, value=1): pass
        def labels(self, **kwargs): return self

    class Gauge:
        def __init__(self, *args, **kwargs): pass
        def set(self, value): pass
        def inc(self, value=1): pass
        def dec(self, value=1): pass
        def labels(self, **kwargs): return self

    class Histogram:
        def __init__(self, *args, **kwargs): pass
        def observe(self, value): pass
        def labels(self, **kwargs): return self

    def start_http_server(*args, **kwargs): pass

try:
    from opentelemetry import trace
    from opentelemetry.sdk.trace import TracerProvider
    from opentelemetry.sdk.trace.export import BatchSpanProcessor, ConsoleSpanExporter
    from opentelemetry.trace import Status, StatusCode
    OPENTELEMETRY_AVAILABLE = True
except ImportError:
    OPENTELEMETRY_AVAILABLE = False
    # Mock opentelemetry classes
    class TracerProvider: pass
    class BatchSpanProcessor: pass
    class ConsoleSpanExporter: pass
    class Status: pass
    StatusCode = type('StatusCode', (), {'OK': 'OK', 'ERROR': 'ERROR'})()

class HealthStatus(Enum):
    """Health status enumeration."""
    HEALTHY = "healthy"
    DEGRADED = "degraded"
    UNHEALTHY = "unhealthy"
    UNKNOWN = "unknown"

@dataclass
class HealthCheckResult:
    """Result of a health check."""
    status: HealthStatus
    message: str
    timestamp: float
    details: Optional[Dict[str, Any]] = None

class ObservabilitySystem:
    """
    Centralized observability system for URC 2026 rover.

    Features:
    - Prometheus metrics collection
    - OpenTelemetry distributed tracing
    - Health checks and monitoring
    - Performance profiling
    - Alerting and notifications
    """

    def __init__(self, service_name: str = "urc_2026_rover", enable_prometheus: bool = True,
                 enable_tracing: bool = True, prometheus_port: int = 8000):
        self.service_name = service_name
        self.enable_prometheus = enable_prometheus and PROMETHEUS_AVAILABLE
        self.enable_tracing = enable_tracing and OPENTELEMETRY_AVAILABLE

        # Initialize components
        self._init_metrics()
        self._init_tracing()
        self._init_health_checks()

        # System state
        self.start_time = time.time()
        self._shutdown = False

        # Start prometheus server if enabled
        if self.enable_prometheus:
            try:
                start_http_server(prometheus_port)
                print(f"📊 Prometheus metrics server started on port {prometheus_port}")
            except Exception as e:
                print(f"⚠️ Failed to start Prometheus server: {e}")

    def _init_metrics(self):
        """Initialize Prometheus metrics."""
        if not self.enable_prometheus:
            return

        # System metrics
        self.cpu_usage = Gauge('urc_cpu_usage_percent', 'CPU usage percentage')
        self.memory_usage = Gauge('urc_memory_usage_mb', 'Memory usage in MB')
        self.disk_usage = Gauge('urc_disk_usage_percent', 'Disk usage percentage')

        # Application metrics
        self.active_connections = Gauge('urc_active_connections', 'Number of active connections')
        self.request_count = Counter('urc_requests_total', 'Total number of requests', ['method', 'endpoint', 'status'])
        self.request_duration = Histogram('urc_request_duration_seconds', 'Request duration in seconds', ['method', 'endpoint'])

        # ROS2 specific metrics
        self.ros2_topics_active = Gauge('urc_ros2_topics_active', 'Number of active ROS2 topics')
        self.ros2_nodes_active = Gauge('urc_ros2_nodes_active', 'Number of active ROS2 nodes')
        self.ros2_message_rate = Counter('urc_ros2_messages_total', 'Total ROS2 messages published', ['topic'])

        # Mission metrics
        self.mission_duration = Histogram('urc_mission_duration_seconds', 'Mission duration in seconds', ['mission_type'])
        self.mission_success = Counter('urc_mission_success_total', 'Mission success/failure count', ['mission_type', 'result'])

        # Safety metrics
        self.safety_violations = Counter('urc_safety_violations_total', 'Safety violations detected', ['violation_type'])
        self.emergency_stops = Counter('urc_emergency_stops_total', 'Emergency stops triggered')

    def _init_tracing(self):
        """Initialize OpenTelemetry tracing."""
        if not self.enable_tracing:
            return

        try:
            # Set up tracer provider
            trace.set_tracer_provider(TracerProvider())
            tracer_provider = trace.get_tracer_provider()

            # Add console exporter for development
            span_processor = BatchSpanProcessor(ConsoleSpanExporter())
            tracer_provider.add_span_processor(span_processor)

            # Get tracer
            self.tracer = trace.get_tracer(__name__)
            print("🔍 OpenTelemetry tracing initialized")
        except Exception as e:
            print(f"⚠️ Failed to initialize tracing: {e}")
            self.tracer = None

    def _init_health_checks(self):
        """Initialize health check system."""
        self.health_checks: Dict[str, Callable[[], HealthCheckResult]] = {}

        # Register default health checks
        self.register_health_check('system_resources', self._check_system_resources)
        self.register_health_check('memory_usage', self._check_memory_usage)
        self.register_health_check('disk_space', self._check_disk_space)

    def register_health_check(self, name: str, check_func: Callable[[], HealthCheckResult]):
        """Register a health check function."""
        self.health_checks[name] = check_func

    def unregister_health_check(self, name: str):
        """Unregister a health check."""
        self.health_checks.pop(name, None)

    def run_health_checks(self) -> Dict[str, HealthCheckResult]:
        """Run all registered health checks."""
        results = {}
        for name, check_func in self.health_checks.items():
            try:
                results[name] = check_func()
            except Exception as e:
                results[name] = HealthCheckResult(
                    status=HealthStatus.UNHEALTHY,
                    message=f"Health check failed: {e}",
                    timestamp=time.time()
                )
        return results

    def _check_system_resources(self) -> HealthCheckResult:
        """Check basic system resources."""
        try:
            cpu_percent = psutil.cpu_percent(interval=1)
            memory = psutil.virtual_memory()
            disk = psutil.disk_usage('/')

            status = HealthStatus.HEALTHY
            message = "System resources normal"
            details = {
                'cpu_percent': cpu_percent,
                'memory_percent': memory.percent,
                'disk_percent': disk.percent
            }

            # Check thresholds
            if cpu_percent > 90 or memory.percent > 90 or disk.percent > 90:
                status = HealthStatus.DEGRADED
                message = "High resource usage detected"
            elif cpu_percent > 95 or memory.percent > 95 or disk.percent > 95:
                status = HealthStatus.UNHEALTHY
                message = "Critical resource usage"

            return HealthCheckResult(
                status=status,
                message=message,
                timestamp=time.time(),
                details=details
            )
        except Exception as e:
            return HealthCheckResult(
                status=HealthStatus.UNKNOWN,
                message=f"System resource check failed: {e}",
                timestamp=time.time()
            )

    def _check_memory_usage(self) -> HealthCheckResult:
        """Check memory usage."""
        try:
            memory = psutil.virtual_memory()
            process = psutil.Process()

            status = HealthStatus.HEALTHY
            message = "Memory usage normal"

            if memory.percent > 85:
                status = HealthStatus.DEGRADED
                message = "High memory usage"
            elif memory.percent > 95:
                status = HealthStatus.UNHEALTHY
                message = "Critical memory usage"

            return HealthCheckResult(
                status=status,
                message=message,
                timestamp=time.time(),
                details={
                    'total_mb': memory.total / 1024 / 1024,
                    'used_mb': memory.used / 1024 / 1024,
                    'available_mb': memory.available / 1024 / 1024,
                    'percent': memory.percent,
                    'process_rss_mb': process.memory_info().rss / 1024 / 1024
                }
            )
        except Exception as e:
            return HealthCheckResult(
                status=HealthStatus.UNKNOWN,
                message=f"Memory check failed: {e}",
                timestamp=time.time()
            )

    def _check_disk_space(self) -> HealthCheckResult:
        """Check disk space availability."""
        try:
            disk = psutil.disk_usage('/')

            status = HealthStatus.HEALTHY
            message = "Disk space adequate"

            if disk.percent > 85:
                status = HealthStatus.DEGRADED
                message = "Low disk space"
            elif disk.percent > 95:
                status = HealthStatus.UNHEALTHY
                message = "Critical disk space"

            return HealthCheckResult(
                status=status,
                message=message,
                timestamp=time.time(),
                details={
                    'total_gb': disk.total / 1024 / 1024 / 1024,
                    'used_gb': disk.used / 1024 / 1024 / 1024,
                    'free_gb': disk.free / 1024 / 1024 / 1024,
                    'percent': disk.percent
                }
            )
        except Exception as e:
            return HealthCheckResult(
                status=HealthStatus.UNKNOWN,
                message=f"Disk check failed: {e}",
                timestamp=time.time()
            )

    def record_request(self, method: str, endpoint: str, status: str, duration: float):
        """Record an HTTP request."""
        if self.enable_prometheus:
            self.request_count.labels(method=method, endpoint=endpoint, status=status).inc()
            self.request_duration.labels(method=method, endpoint=endpoint).observe(duration)

    def record_mission_start(self, mission_type: str):
        """Record mission start."""
        if self.enable_tracing and self.tracer:
            with self.tracer.start_as_current_span(f"mission_{mission_type}") as span:
                span.set_attribute("mission.type", mission_type)
                span.set_attribute("mission.start_time", time.time())

    def record_mission_end(self, mission_type: str, success: bool, duration: float):
        """Record mission completion."""
        if self.enable_prometheus:
            result = "success" if success else "failure"
            self.mission_success.labels(mission_type=mission_type, result=result).inc()
            self.mission_duration.labels(mission_type=mission_type).observe(duration)

        if self.enable_tracing and self.tracer:
            with self.tracer.start_as_current_span(f"mission_complete_{mission_type}") as span:
                span.set_attribute("mission.type", mission_type)
                span.set_attribute("mission.success", success)
                span.set_attribute("mission.duration", duration)
                span.set_status(Status(StatusCode.OK if success else StatusCode.ERROR))

    def record_safety_violation(self, violation_type: str):
        """Record a safety violation."""
        if self.enable_prometheus:
            self.safety_violations.labels(violation_type=violation_type).inc()

    def record_emergency_stop(self):
        """Record an emergency stop."""
        if self.enable_prometheus:
            self.emergency_stops.inc()

    def update_system_metrics(self):
        """Update system-level metrics."""
        if not self.enable_prometheus:
            return

        try:
            # CPU usage
            self.cpu_usage.set(psutil.cpu_percent())

            # Memory usage
            memory = psutil.virtual_memory()
            self.memory_usage.set(memory.used / 1024 / 1024)

            # Disk usage
            disk = psutil.disk_usage('/')
            self.disk_usage.set(disk.percent)

        except Exception as e:
            print(f"⚠️ Failed to update system metrics: {e}")

    def create_tracer_span(self, name: str, **attributes):
        """Create a tracing span context manager."""
        if not self.enable_tracing or not self.tracer:
            return _MockSpan()

        return self.tracer.start_as_current_span(name, **attributes)

    def shutdown(self):
        """Shutdown the observability system."""
        self._shutdown = True
        print("🔍 Observability system shutting down")

    def add_health_check(self, name: str, check_func: Callable[[], Dict[str, Any]]):
        """Add a health check (legacy compatibility method)."""
        def wrapped_check():
            result = check_func()
            return HealthCheckResult(
                status=HealthStatus.HEALTHY if result.get('healthy', True) else HealthStatus.UNHEALTHY,
                message=result.get('message', ''),
                timestamp=time.time(),
                details=result
            )
        self.register_health_check(name, wrapped_check)

    def create_correlation_context(self):
        """Create a correlation context for tracing (legacy compatibility)."""
        return self.create_tracer_span("correlation_context")

    def stop_monitoring(self):
        """Stop monitoring (legacy compatibility)."""
        self.shutdown()

    @property
    def logger(self):
        """Get a logger instance (legacy compatibility)."""
        class LegacyLogger:
            def __init__(self, obs_system):
                self.obs = obs_system

            def info(self, msg, **kwargs):
                print(f"[OBS] {msg}")

            def error(self, msg, error=None, **kwargs):
                print(f"[OBS ERROR] {msg}: {error}")

            def warn(self, msg, **kwargs):
                print(f"[OBS WARN] {msg}")

        return LegacyLogger(self)

class _MockSpan:
    """Mock span for when tracing is disabled."""
    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        pass

    def set_attribute(self, key, value):
        pass

    def set_status(self, status):
        pass

    def record_exception(self, exception):
        pass

# Global instance
_observability_instance = None
_observability_lock = threading.Lock()

def get_observability_system(service_name: str = "urc_2026_rover") -> ObservabilitySystem:
    """Get or create the global observability system instance."""
    global _observability_instance

    if _observability_instance is None:
        with _observability_lock:
            if _observability_instance is None:
                _observability_instance = ObservabilitySystem(service_name)

    return _observability_instance

def init_observability(service_name: str = "urc_2026_rover", enable_prometheus: bool = True,
                      enable_tracing: bool = True, prometheus_port: int = 8000):
    """Initialize the observability system."""
    global _observability_instance

    with _observability_lock:
        _observability_instance = ObservabilitySystem(
            service_name=service_name,
            enable_prometheus=enable_prometheus,
            enable_tracing=enable_tracing,
            prometheus_port=prometheus_port
        )

    return _observability_instance

def shutdown_observability():
    """Shutdown the observability system."""
    global _observability_instance

    if _observability_instance:
        _observability_instance.shutdown()
        _observability_instance = None
