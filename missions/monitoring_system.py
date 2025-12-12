"""
Monitoring System for URC 2026 Mission Execution

Provides non-invasive monitoring, event recording, and system health tracking
for mission execution and system diagnostics.
"""

import threading
import time
from dataclasses import dataclass
from enum import Enum
from typing import Any, Callable, Dict, List, Optional


class SamplingRate(Enum):
    """Monitoring sampling rates."""
    HIGH = "high"      # 100Hz - real-time systems
    MEDIUM = "medium"  # 10Hz - control systems
    LOW = "low"        # 1Hz - status monitoring


class MonitoringEvent:
    """Represents a monitoring event."""

    def __init__(
        self,
        event_type: str,
        source: str,
        data: Dict[str, Any],
        timestamp: Optional[float] = None
    ):
        self.event_type = event_type
        self.source = source
        self.data = data
        self.timestamp = timestamp or time.time()

    def to_dict(self) -> Dict[str, Any]:
        """Convert event to dictionary."""
        return {
            'event_type': self.event_type,
            'source': self.source,
            'data': self.data,
            'timestamp': self.timestamp
        }


@dataclass
class MonitoringConfig:
    """Configuration for monitoring system."""
    sampling_rate: SamplingRate = SamplingRate.MEDIUM
    max_buffer_size: int = 500
    enable_async_processing: bool = True
    alert_thresholds: Optional[Dict[str, float]] = None

    def __post_init__(self):
        if self.alert_thresholds is None:
            self.alert_thresholds = {
                'cpu_percent': 90.0,
                'memory_percent': 85.0,
                'disk_usage_percent': 95.0,
                'temperature_celsius': 80.0
            }


class Monitor:
    """System monitoring and event recording."""

    def __init__(self, config: MonitoringConfig):
        self.config = config
        self.events: List[MonitoringEvent] = []
        self.is_active = False
        self.monitor_thread: Optional[threading.Thread] = None
        self.lock = threading.Lock()

        # Callbacks for different event types
        self.callbacks: Dict[str, List[Callable]] = {}

    def start_monitoring(self):
        """Start the monitoring system."""
        if self.is_active:
            return

        self.is_active = True
        if self.config.enable_async_processing:
            self.monitor_thread = threading.Thread(
                target=self._monitor_loop, daemon=True
            )
            self.monitor_thread.start()

    def stop_monitoring(self):
        """Stop the monitoring system."""
        self.is_active = False
        if self.monitor_thread:
            self.monitor_thread.join(timeout=1.0)

    def _monitor_loop(self):
        """Main monitoring loop."""
        # Calculate sampling interval
        intervals = {
            SamplingRate.HIGH: 0.01,    # 100Hz
            SamplingRate.MEDIUM: 0.1,   # 10Hz
            SamplingRate.LOW: 1.0      # 1Hz
        }
        interval = intervals.get(self.config.sampling_rate, 0.1)

        while self.is_active:
            try:
                # Collect system metrics
                metrics = self._collect_system_metrics()

                # Check thresholds and generate alerts
                self._check_thresholds(metrics)

                time.sleep(interval)

            except Exception as e:
                # Log error but continue monitoring
                error_event = MonitoringEvent(
                    'monitoring_error',
                    'monitor_system',
                    {'error': str(e), 'component': 'monitor_loop'}
                )
                self._record_event(error_event)

    def _collect_system_metrics(self) -> Dict[str, Any]:
        """Collect basic system metrics."""
        # Placeholder for system metrics collection
        # In a real implementation, this would use psutil or similar
        return {
            'cpu_percent': 45.0,  # Placeholder
            'memory_percent': 60.0,  # Placeholder
            'disk_usage_percent': 75.0,  # Placeholder
            'temperature_celsius': 55.0,  # Placeholder
            'timestamp': time.time()
        }

    def _check_thresholds(self, metrics: Dict[str, Any]):
        """Check if metrics exceed thresholds."""
        if self.config.alert_thresholds is None:
            return
        for metric_name, threshold in self.config.alert_thresholds.items():
            if metric_name in metrics:
                value = metrics[metric_name]
                if value > threshold:
                    alert_event = MonitoringEvent(
                        'threshold_exceeded',
                        'monitor_system',
                        {
                            'metric': metric_name,
                            'value': value,
                            'threshold': threshold,
                            'severity': 'warning'
                        }
                    )
                    self._record_event(alert_event)

    def _record_event(self, event: MonitoringEvent):
        """Record a monitoring event."""
        with self.lock:
            self.events.append(event)

            # Maintain buffer size
            if len(self.events) > self.config.max_buffer_size:
                self.events.pop(0)

            # Trigger callbacks
            if event.event_type in self.callbacks:
                for callback in self.callbacks[event.event_type]:
                    try:
                        callback(event)
                    except Exception:
                        # Don't let callback errors break monitoring
                        pass

    def record_event(
        self,
        event_type: str,
        source: str,
        data: Dict[str, Any]
    ):
        """Record a custom event."""
        event = MonitoringEvent(event_type, source, data)
        self._record_event(event)

    def get_recent_events(self, limit: int = 50) -> List[MonitoringEvent]:
        """Get recent monitoring events."""
        with self.lock:
            return self.events[-limit:].copy()

    def get_events_by_type(self, event_type: str) -> List[MonitoringEvent]:
        """Get events of a specific type."""
        with self.lock:
            return [e for e in self.events if e.event_type == event_type]

    def add_callback(self, event_type: str, callback: Callable):
        """Add a callback for a specific event type."""
        if event_type not in self.callbacks:
            self.callbacks[event_type] = []
        self.callbacks[event_type].append(callback)

    def clear_events(self):
        """Clear all recorded events."""
        with self.lock:
            self.events.clear()


# Global monitor instance with thread-safe initialization
_monitor_instance: Optional[Monitor] = None
_monitor_lock = threading.Lock()


def get_monitor(config: Optional[MonitoringConfig] = None) -> Monitor:
    """Get the global monitor instance with thread-safe singleton pattern."""
    global _monitor_instance

    # Double-checked locking pattern for thread safety
    if _monitor_instance is None:
        with _monitor_lock:
            # Check again inside lock to prevent race condition
            if _monitor_instance is None:
                if config is None:
                    config = MonitoringConfig()
                _monitor_instance = Monitor(config)
                _monitor_instance.start_monitoring()

    return _monitor_instance


def record_detection(source: str, data: Dict[str, Any]):
    """Record a detection event."""
    monitor = get_monitor()
    monitor.record_event('detection', source, data)


def record_emergency(source: str, data: Dict[str, Any]):
    """Record an emergency event."""
    monitor = get_monitor()
    monitor.record_event('emergency', source, data)


def record_failure(source: str, data: Dict[str, Any]):
    """Record a failure event."""
    monitor = get_monitor()
    monitor.record_event('failure', source, data)
