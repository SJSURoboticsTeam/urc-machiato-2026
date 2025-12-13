"""Comprehensive tracing system for simulation debugging and performance analysis.

Provides detailed tracing of simulation operations, performance profiling,
and debugging capabilities with minimal overhead when disabled.

Author: URC 2026 Autonomy Team
"""

import functools
import threading
import time
from collections import defaultdict
from contextlib import contextmanager
from dataclasses import asdict, dataclass
from typing import Any, Callable, Dict, List, Optional

from simulation.core.logging_config import get_simulation_logger


@dataclass
class TraceSpan:
    """Represents a traced operation span."""
    name: str
    start_time: float
    end_time: Optional[float] = None
    duration: Optional[float] = None
    attributes: Dict[str, Any] = None
    parent: Optional['TraceSpan'] = None
    children: List['TraceSpan'] = None
    thread_id: int = None
    operation_id: str = None

    def __post_init__(self):
        if self.attributes is None:
            self.attributes = {}
        if self.children is None:
            self.children = []
        if self.thread_id is None:
            self.thread_id = threading.get_ident()

    def complete(self):
        """Mark span as completed."""
        self.end_time = time.time()
        self.duration = self.end_time - self.start_time

    def add_child(self, child: 'TraceSpan'):
        """Add child span."""
        if self.children is None:
            self.children = []
        self.children.append(child)
        child.parent = self


class SimulationTracer:
    """Advanced tracing system for simulation debugging and profiling."""

    def __init__(self):
        """Initialize simulation tracer."""
        self.logger = get_simulation_logger(__name__, "tracer")

        # Tracing state
        self.enabled = True
        self.traces: List[TraceSpan] = []
        self.active_spans: List[TraceSpan] = []
        self.max_traces = 10000  # Limit memory usage

        # Performance statistics
        self.operation_stats = defaultdict(list)
        self.thread_stats = defaultdict(list)

        # Tracing configuration
        self.auto_profile_threshold = 0.1  # Log operations > 100ms
        self.enable_thread_tracking = True

        # Operation counters
        self.operation_counters = defaultdict(int)
        self.error_counters = defaultdict(int)

    def enable(self):
        """Enable tracing."""
        self.enabled = True
        self.logger.info("Tracing enabled")

    def disable(self):
        """Disable tracing."""
        self.enabled = False
        self.logger.info("Tracing disabled")

    @contextmanager
    def trace_context(self, name: str, **attributes):
        """Context manager for tracing operations.

        Args:
            name: Operation name
            **attributes: Additional trace attributes
        """
        if not self.enabled:
            yield None
            return

        # Create span
        span = TraceSpan(
            name=name,
            start_time=time.time(),
            attributes=attributes.copy(),
            operation_id=f"{name}_{self.operation_counters[name]}"
        )

        # Add to active spans
        if self.active_spans:
            parent = self.active_spans[-1]
            parent.add_child(span)
        self.active_spans.append(span)

        try:
            yield span
        finally:
            span.complete()
            self._record_span(span)
            self.active_spans.pop()

    def trace_method(self, name: str = None, auto_log_slow: bool = True):
        """Decorator for tracing methods.

        Args:
            name: Custom operation name (default: method name)
            auto_log_slow: Automatically log slow operations

        Returns:
            Decorator function
        """
        def decorator(func: Callable):
            operation_name = name or f"{func.__qualname__}"

            @functools.wraps(func)
            def wrapper(*args, **kwargs):
                with self.trace_context(
                    operation_name,
                    function=func.__name__,
                    module=func.__module__,
                    args_count=len(args),
                    kwargs_count=len(kwargs)
                ) as span:
                    start_time = time.time()
                    try:
                        result = func(*args, **kwargs)

                        # Add return info to span
                        if span:
                            span.attributes["success"] = True
                            span.attributes["return_type"] = type(result).__name__

                        return result

                    except Exception as e:
                        # Record error
                        if span:
                            span.attributes["success"] = False
                            span.attributes["error"] = str(e)
                            span.attributes["error_type"] = type(e).__name__

                        self.error_counters[operation_name] += 1
                        raise
                    finally:
                        # Auto-log slow operations
                        if auto_log_slow and span and span.duration:
                            if span.duration > self.auto_profile_threshold:
                                self.logger.log_performance(
                                    operation_name,
                                    span.duration,
                                    slow_operation=True,
                                    **span.attributes
                                )

            return wrapper
        return decorator

    def trace_function_call(self, func: Callable, *args, **kwargs):
        """Trace a function call directly.

        Args:
            func: Function to trace
            *args: Function arguments
            **kwargs: Function keyword arguments

        Returns:
            Function result
        """
        operation_name = f"{func.__name__}"

        with self.trace_context(
            operation_name,
            function=func.__name__,
            args_count=len(args),
            kwargs_count=len(kwargs)
        ) as span:
            try:
                result = func(*args, **kwargs)
                if span:
                    span.attributes["success"] = True
                return result
            except Exception as e:
                if span:
                    span.attributes["success"] = False
                    span.attributes["error"] = str(e)
                raise

    def _record_span(self, span: TraceSpan):
        """Record completed span.

        Args:
            span: Completed trace span
        """
        self.traces.append(span)
        self.operation_counters[span.name] += 1

        # Maintain size limit
        if len(self.traces) > self.max_traces:
            # Remove oldest traces
            remove_count = len(self.traces) - self.max_traces
            self.traces = self.traces[remove_count:]

        # Update statistics
        if span.duration is not None:
            self.operation_stats[span.name].append(span.duration)

            # Keep only recent statistics
            if len(self.operation_stats[span.name]) > 1000:
                self.operation_stats[span.name] = self.operation_stats[span.name][-1000:]

        # Thread tracking
        if self.enable_thread_tracking:
            self.thread_stats[span.thread_id].append(span)

    def get_performance_report(self) -> Dict[str, Any]:
        """Generate comprehensive performance report.

        Returns:
            Dict with detailed performance analysis
        """
        if not self.traces:
            return {"status": "no_traces"}

        report = {
            "total_operations": len(self.traces),
            "unique_operations": len(self.operation_stats),
            "total_errors": sum(self.error_counters.values()),
            "time_range": self._get_trace_time_range(),
        }

        # Operation statistics
        operation_report = {}
        for op_name, durations in self.operation_stats.items():
            if not durations:
                continue

            operation_report[op_name] = {
                "count": len(durations),
                "total_time": sum(durations),
                "avg_time": sum(durations) / len(durations),
                "min_time": min(durations),
                "max_time": max(durations),
                "p50_time": sorted(durations)[int(len(durations) * 0.5)],
                "p95_time": sorted(durations)[int(len(durations) * 0.95)],
                "p99_time": sorted(durations)[int(len(durations) * 0.99)],
                "errors": self.error_counters.get(op_name, 0),
            }

        report["operations"] = operation_report

        # Thread analysis
        if self.enable_thread_tracking:
            thread_report = {}
            for thread_id, spans in self.thread_stats.items():
                thread_report[str(thread_id)] = {
                    "operation_count": len(spans),
                    "total_time": sum(span.duration for span in spans if span.duration),
                    "operations": [span.name for span in spans[-10:]],  # Last 10
                }
            report["threads"] = thread_report

        # Slowest operations
        all_durations = []
        for op_name, durations in self.operation_stats.items():
            for duration in durations:
                all_durations.append((op_name, duration))

        all_durations.sort(key=lambda x: x[1], reverse=True)
        report["slowest_operations"] = [
            {"operation": op, "duration": dur}
            for op, dur in all_durations[:10]
        ]

        # Error analysis
        if self.error_counters:
            error_report = {}
            for op_name, error_count in self.error_counters.items():
                op_count = self.operation_counters.get(op_name, 0)
                error_rate = error_count / op_count if op_count > 0 else 0
                error_report[op_name] = {
                    "error_count": error_count,
                    "total_count": op_count,
                    "error_rate": error_rate,
                }
            report["errors"] = error_report

        return report

    def _get_trace_time_range(self) -> Dict[str, float]:
        """Get time range of traces.

        Returns:
            Dict with start and end times
        """
        if not self.traces:
            return {"start": 0, "end": 0, "duration": 0}

        start_times = [span.start_time for span in self.traces]
        end_times = [span.end_time for span in self.traces if span.end_time]

        return {
            "start": min(start_times),
            "end": max(end_times) if end_times else max(start_times),
            "duration": (max(end_times) if end_times else max(start_times)) - min(start_times),
        }

    def get_operation_timeline(self, operation_name: str = None,
                              max_entries: int = 100) -> List[Dict[str, Any]]:
        """Get timeline of operations.

        Args:
            operation_name: Specific operation to filter (None for all)
            max_entries: Maximum number of entries to return

        Returns:
            List of operation timeline entries
        """
        timeline = []

        for span in self.traces[-max_entries:]:  # Most recent entries
            if operation_name and span.name != operation_name:
                continue

            entry = {
                "operation": span.name,
                "start_time": span.start_time,
                "duration": span.duration or 0,
                "thread_id": span.thread_id,
                "success": span.attributes.get("success", True),
                "attributes": span.attributes,
            }
            timeline.append(entry)

        return timeline

    def clear_traces(self):
        """Clear all recorded traces."""
        self.traces.clear()
        self.operation_stats.clear()
        self.thread_stats.clear()
        self.operation_counters.clear()
        self.error_counters.clear()
        self.logger.info("Traces cleared")

    def export_traces(self, filepath: str, format: str = "json") -> bool:
        """Export traces to file.

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
                    "traces": [asdict(span) for span in self.traces],
                    "performance_report": self.get_performance_report(),
                    "exported_at": time.time(),
                }
                with open(path, "w") as f:
                    json.dump(data, f, indent=2, default=str)

            elif format == "csv":
                import csv
                with open(path, "w", newline="") as f:
                    writer = csv.writer(f)
                    writer.writerow([
                        "operation", "start_time", "duration", "thread_id",
                        "success", "error", "attributes"
                    ])

                    for span in self.traces:
                        writer.writerow([
                            span.name,
                            span.start_time,
                            span.duration or 0,
                            span.thread_id,
                            span.attributes.get("success", True),
                            span.attributes.get("error", ""),
                            str(span.attributes)
                        ])

            self.logger.info("Traces exported", filepath=str(path), format=format)
            return True

        except Exception as e:
            self.logger.error("Failed to export traces", error=str(e))
            return False

    def get_active_operations(self) -> List[Dict[str, Any]]:
        """Get currently active operations.

        Returns:
            List of active operation information
        """
        active_ops = []
        for span in self.active_spans:
            active_ops.append({
                "operation": span.name,
                "start_time": span.start_time,
                "duration_so_far": time.time() - span.start_time,
                "thread_id": span.thread_id,
                "attributes": span.attributes,
            })
        return active_ops

    def profile_function(self, func: Callable, *args, **kwargs) -> Dict[str, Any]:
        """Profile a function execution.

        Args:
            func: Function to profile
            *args: Function arguments
            **kwargs: Function keyword arguments

        Returns:
            Dict with profiling results
        """
        profile_data = {
            "function": func.__name__,
            "start_time": time.time(),
            "traces_before": len(self.traces),
        }

        # Enable tracing if disabled
        was_enabled = self.enabled
        if not was_enabled:
            self.enable()

        try:
            # Execute with tracing
            with self.trace_context(f"profile_{func.__name__}",
                                   profiling=True,
                                   args_count=len(args),
                                   kwargs_count=len(kwargs)):
                result = func(*args, **kwargs)

            profile_data["result"] = "success"
            profile_data["traces_after"] = len(self.traces)
            profile_data["new_traces"] = profile_data["traces_after"] - profile_data["traces_before"]

            return profile_data

        except Exception as e:
            profile_data["result"] = "error"
            profile_data["error"] = str(e)
            raise
        finally:
            profile_data["end_time"] = time.time()
            profile_data["total_time"] = profile_data["end_time"] - profile_data["start_time"]

            # Restore tracing state
            if not was_enabled:
                self.disable()
