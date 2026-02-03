#!/usr/bin/env python3
"""
URC 2026 Autonomy System - Core Utilities (Consolidated & Optimized)

This module provides essential utilities for ROS2 nodes including:
- Lifecycle Management base classes
- Zero-copy shared memory hooks
- Consistent logging and error handling
- Standardized operation results
"""

import json
import logging
import os
import time
from dataclasses import dataclass
from typing import Any, Callable, Dict, List, Optional, Tuple, TypeVar, Union

import yaml
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy

# Type definitions
T = TypeVar("T")
Result = Tuple[Optional[Any], Optional[str]]

# ===== EXCEPTIONS =====


class ValidationError(Exception):
    """Raised when input validation fails."""

    pass


class ProcessingError(Exception):
    """Raised when data processing fails."""

    pass


class CommunicationError(Exception):
    """Raised when communication operations fail."""

    pass


class ConfigurationError(Exception):
    """Raised when configuration is invalid."""

    pass


class LifecycleError(Exception):
    """Raised when lifecycle operations fail."""

    pass


# ===== ERROR HANDLING DECORATORS =====


def handle_exceptions(logger=None, rethrow: bool = True, default_return=None):
    """
    Decorator for consistent exception handling across nodes.

    Args:
        logger: Optional logger instance
        rethrow: Whether to rethrow exceptions after logging
        default_return: Default value to return on exception
    """

    def decorator(func):
        def wrapper(*args, **kwargs):
            try:
                return func(*args, **kwargs)
            except (
                ValidationError,
                ProcessingError,
                CommunicationError,
                ConfigurationError,
                LifecycleError,
            ) as e:
                # Domain-specific exceptions - log as warnings
                if logger:
                    logger.warn(
                        f"{func.__name__} failed with domain error",
                        error=str(e),
                        error_type=type(e).__name__,
                    )
                else:
                    print(f"WARNING: {func.__name__} failed: {e}")
                if rethrow:
                    raise
                return default_return
            except Exception as e:
                # Unexpected exceptions - log as errors
                if logger:
                    logger.error(
                        f"{func.__name__} failed with unexpected error",
                        error=str(e),
                        error_type=type(e).__name__,
                    )
                else:
                    print(f"ERROR: {func.__name__} failed: {e}")
                if rethrow:
                    raise
                return default_return

        return wrapper

    return decorator


def validate_inputs(**validators):
    """
    Decorator for input validation.

    Args:
        validators: Dict of parameter_name -> validation_function
    """

    def decorator(func):
        def wrapper(*args, **kwargs):
            # Get function signature to map args to parameter names
            import inspect

            sig = inspect.signature(func)
            bound_args = sig.bind(*args, **kwargs)
            bound_args.apply_defaults()

            for param_name, validator_func in validators.items():
                if param_name in bound_args.arguments:
                    value = bound_args.arguments[param_name]
                    if not validator_func(value):
                        raise ValidationError(
                            f"Validation failed for parameter '{param_name}': {value}"
                        )

            return func(*args, **kwargs)

        return wrapper

    return decorator


# ===== QoS PROFILES =====
# Standardized QoS profiles for consistent communication patterns


class QoSProfiles:
    """Standardized QoS profiles for URC 2026 rover."""

    @staticmethod
    def sensor_data() -> QoSProfile:
        """QoS for sensor data: BEST_EFFORT, volatile, high frequency."""
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

    @staticmethod
    def command_data() -> QoSProfile:
        """QoS for commands: RELIABLE, transient local, low latency."""
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )

    @staticmethod
    def state_data() -> QoSProfile:
        """QoS for state information: RELIABLE, transient local."""
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

    @staticmethod
    def diagnostic_data() -> QoSProfile:
        """QoS for diagnostics: RELIABLE, keep all for analysis."""
        return QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_ALL,
            depth=100,
        )

    @staticmethod
    def high_frequency_sensor() -> QoSProfile:
        """QoS for high-frequency sensors like IMU: BEST_EFFORT, minimal depth."""
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5,
        )


# ===== PERFORMANCE MONITORING =====


class PerformanceMonitor:
    """Performance monitoring utilities for ROS2 nodes."""

    def __init__(self, node, component_name: str):
        self.node = node
        self.component_name = component_name
        self.logger = NodeLogger(node, f"{component_name}_perf")
        self.start_time = node.get_clock().now()
        self.message_counts = {}
        self.latency_measurements = []
        self.error_counts = {}

    def track_message(self, topic_name: str, message_size: int = 0):
        """Track message publication/reception for throughput monitoring."""
        if topic_name not in self.message_counts:
            self.message_counts[topic_name] = {
                "count": 0,
                "bytes": 0,
                "last_time": self.node.get_clock().now(),
            }

        self.message_counts[topic_name]["count"] += 1
        self.message_counts[topic_name]["bytes"] += message_size
        self.message_counts[topic_name]["last_time"] = self.node.get_clock().now()

    def measure_latency(self, operation_name: str, duration_ns: int):
        """Track operation latency."""
        self.latency_measurements.append(
            {
                "operation": operation_name,
                "duration_ns": duration_ns,
                "timestamp": self.node.get_clock().now().nanoseconds,
            }
        )

        # Keep only recent measurements
        if len(self.latency_measurements) > 100:
            self.latency_measurements = self.latency_measurements[-50:]

    def track_error(self, error_type: str, error_message: str = ""):
        """Track error occurrences."""
        if error_type not in self.error_counts:
            self.error_counts[error_type] = 0
        self.error_counts[error_type] += 1

        if len(self.error_counts) > 10:  # Limit error types tracked
            # Remove oldest error type
            oldest_error = min(
                self.error_counts.keys(), key=lambda k: self.error_counts[k]
            )
            del self.error_counts[oldest_error]

    def get_performance_report(self) -> Dict[str, Any]:
        """Generate performance report."""
        uptime = (self.node.get_clock().now() - self.start_time).nanoseconds / 1e9

        report = {
            "component": self.component_name,
            "uptime_seconds": uptime,
            "message_stats": {},
            "latency_stats": {},
            "error_stats": self.error_counts.copy(),
        }

        # Message throughput stats
        for topic, stats in self.message_counts.items():
            time_diff = (
                self.node.get_clock().now() - stats["last_time"]
            ).nanoseconds / 1e9
            if time_diff > 0:
                hz = stats["count"] / max(uptime, 1.0)
                bps = stats["bytes"] / max(uptime, 1.0)
                report["message_stats"][topic] = {
                    "total_messages": stats["count"],
                    "messages_per_second": hz,
                    "bytes_per_second": bps,
                }

        # Latency stats
        if self.latency_measurements:
            operations = {}
            for measurement in self.latency_measurements:
                op = measurement["operation"]
                if op not in operations:
                    operations[op] = []
                operations[op].append(measurement["duration_ns"] / 1e6)  # Convert to ms

            for op, latencies in operations.items():
                if latencies:
                    report["latency_stats"][op] = {
                        "avg_ms": sum(latencies) / len(latencies),
                        "min_ms": min(latencies),
                        "max_ms": max(latencies),
                        "count": len(latencies),
                    }

        return report

    def log_performance_summary(self):
        """Log a summary of performance metrics."""
        report = self.get_performance_report()

        self.logger.info(
            "Performance Summary",
            uptime=f"{report['uptime_seconds']:.1f}s",
            messages=len(report["message_stats"]),
            errors=sum(report["error_stats"].values()),
        )

        for topic, stats in report["message_stats"].items():
            self.logger.debug(f"Topic {topic}: {stats['messages_per_second']:.1f} Hz")

        for error_type, count in report["error_stats"].items():
            if count > 0:
                self.logger.warn(f"Errors {error_type}: {count}")


# ===== PARAMETER MANAGEMENT =====


class ParameterManager:
    """Standardized parameter declaration and validation for ROS2 nodes."""

    def __init__(self, node):
        self.node = node
        self.logger = NodeLogger(node, node.get_name())

    def declare_parameters(
        self, param_specs: Dict[str, Dict[str, Any]]
    ) -> Dict[str, Any]:
        """
        Declare parameters with consistent error handling and defaults.

        Args:
            param_specs: Dict of param_name -> {"default": value, "type": type, "description": str}

        Returns:
            Dict of parameter names to their values
        """
        parameters = {}
        param_list = []

        for name, spec in param_specs.items():
            default = spec.get("default")
            param_type = spec.get("type", type(default))
            description = spec.get("description", "")

            param_list.append(
                (
                    name,
                    default,
                    ParameterDescriptor(type=param_type, description=description),
                )
            )

        try:
            # Declare all parameters at once for efficiency
            self.node.declare_parameters("", param_list)
            self.logger.debug(
                f"Declared {len(param_list)} parameters", param_count=len(param_list)
            )
        except Exception as e:
            self.logger.error("Failed to declare parameters", error=e)
            return {}

        # Get all parameter values
        for name in param_specs.keys():
            try:
                parameters[name] = self.node.get_parameter(name).value
            except Exception as e:
                default_val = param_specs[name]["default"]
                self.logger.warn(
                    f"Using default for parameter {name}",
                    default=default_val,
                    error=str(e),
                )
                parameters[name] = default_val

        return parameters

    def get_validated_parameter(
        self,
        name: str,
        expected_type: type,
        default: Any = None,
        required: bool = False,
    ) -> Any:
        """
        Get and validate a parameter value with consistent error handling.

        Args:
            name: Parameter name
            expected_type: Expected type
            default: Default value if not found
            required: Whether parameter is required

        Returns:
            Validated parameter value

        Raises:
            ValidationError: If parameter validation fails
        """
        try:
            value = self.node.get_parameter(name).value
        except Exception:
            if required:
                raise ValidationError(f"Required parameter '{name}' not found")
            return default

        if not isinstance(value, expected_type):
            try:
                value = expected_type(value)
            except (ValueError, TypeError):
                if required:
                    raise ValidationError(
                        f"Parameter '{name}' must be of type {expected_type.__name__}"
                    )
                self.logger.warn(
                    f"Type conversion failed for {name}, using default",
                    value=value,
                    expected_type=expected_type.__name__,
                )
                return default

        return value


# ===== STANDARDIZED RESULTS =====


@dataclass
class Failure:
    """Represents a failure with error details."""

    message: str
    code: Optional[str] = None
    details: Optional[Dict[str, Any]] = None


@dataclass
class OperationResult:
    """Result of an operation with success/failure status."""

    success: bool
    data: Any = None
    error: Optional[str] = None


def success(data: Any = None) -> OperationResult:
    """Create a successful operation result."""
    return OperationResult(success=True, data=data)


def failure(
    message: str, code: Optional[str] = None, details: Optional[Dict[str, Any]] = None
) -> OperationResult:
    """Create a failed operation result."""
    return OperationResult(
        success=False, error=message, data=Failure(message, code, details)
    )


# ===== LOGGING =====


class NodeLogger:
    """Standardized logging helper for ROS2 nodes with consistent formatting and levels."""

    # Standardized log levels for URC 2026
    TRACE = 0  # Very detailed diagnostic information
    DEBUG = 10  # Detailed debugging information
    INFO = 20  # General information about system operation
    WARN = 30  # Warning about potential issues
    ERROR = 40  # Error that doesn't stop operation
    CRITICAL = 50  # Critical errors that may require intervention

    def __init__(self, node, component: str):
        self._logger = node.get_logger()
        self._component = component

    def _format_message(self, message: str, **context) -> str:
        """Format message with consistent structure."""
        base_msg = f"[{self._component}] {message}"

        # Add structured context if provided
        if context:
            # Filter out None values and format consistently
            ctx_parts = []
            for k, v in context.items():
                if v is not None:
                    if isinstance(v, float):
                        ctx_parts.append(f"{k}={v:.3f}")
                    else:
                        ctx_parts.append(f"{k}={v}")
            if ctx_parts:
                return f"{base_msg} | {' | '.join(ctx_parts)}"

        return base_msg

    def trace(self, message: str, **context):
        """Log trace-level diagnostic information."""
        self._logger.debug(f"TRACE: {self._format_message(message, **context)}")

    def debug(self, message: str, **context):
        """Log debug information for development."""
        self._logger.debug(self._format_message(message, **context))

    def info(self, message: str, **context):
        """Log general operational information."""
        self._logger.info(self._format_message(message, **context))

    def warn(self, message: str, operation: str = None, **context):
        """Log warnings about potential issues."""
        if operation:
            message = f"{operation} warning: {message}"
        self._logger.warn(self._format_message(message, **context))

    def error(
        self,
        message: str,
        error: Optional[Exception] = None,
        operation: str = None,
        **context,
    ):
        """Log errors that don't stop operation."""
        if operation:
            message = f"{operation} error: {message}"
        if error:
            context.update(
                {"error_type": type(error).__name__, "error_message": str(error)}
            )
        self._logger.error(self._format_message(message, **context))

    def critical(self, message: str, error: Optional[Exception] = None, **context):
        """Log critical errors requiring intervention."""
        if error:
            context.update(
                {"error_type": type(error).__name__, "error_message": str(error)}
            )
        self._logger.fatal(self._format_message(f"CRITICAL: {message}", **context))

    # Convenience methods for common operations
    def operation_start(self, operation: str, **context):
        """Log start of an operation."""
        self.info(f"Starting {operation}", operation=operation, **context)

    def operation_success(
        self, operation: str, duration_ms: Optional[float] = None, **context
    ):
        """Log successful completion of an operation."""
        if duration_ms is not None:
            context["duration_ms"] = duration_ms
        self.info(f"Completed {operation}", operation=operation, **context)

    def operation_failed(
        self, operation: str, error: Optional[Exception] = None, **context
    ):
        """Log failed operation."""
        self.error(f"Failed {operation}", error=error, operation=operation, **context)


# ===== SHARED MEMORY & ZERO-COPY =====


class SharedMemoryHook:
    """Hook for zero-copy shared memory integration (iceoryx/shm)."""

    @staticmethod
    def is_enabled() -> bool:
        return os.environ.get("USE_ZERO_COPY", "false").lower() == "true"

    @staticmethod
    def get_shm_buffer(topic_name: str, size: int):
        """Placeholder for actual SHM buffer retrieval."""
        if not SharedMemoryHook.is_enabled():
            return None
        # In a real implementation, this would interface with a shared memory library
        return None


# ===== BASE NODES =====


class StateMachineNode(LifecycleNode):
    """Base node with state machine and Lifecycle management."""

    def __init__(self, node_name: str, initial_state: str = "idle"):
        super().__init__(node_name)
        self._current_state = initial_state
        self._state_entry_time = self.get_clock().now()
        self._state_history = []
        self.logger = NodeLogger(self, node_name)

        # Zero-copy optimization check
        if SharedMemoryHook.is_enabled():
            self.logger.info("Zero-copy shared memory ENABLED")

    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info("Configuring node...")
        return TransitionCallbackReturn.SUCCESS

    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info("Activating node...")
        return TransitionCallbackReturn.SUCCESS

    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info("Deactivating node...")
        return TransitionCallbackReturn.SUCCESS

    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        self.logger.info("Cleaning up node...")
        return TransitionCallbackReturn.SUCCESS

    def transition_to(self, new_state: str, reason: str = ""):
        old_state = self._current_state
        if old_state == new_state:
            return
        self._current_state = new_state
        self._state_entry_time = self.get_clock().now()
        self.logger.info(f"Transition: {old_state} -> {new_state}", reason=reason)


class AutonomyNode(StateMachineNode):
    """Compatibility wrapper for autonomy nodes."""

    def __init__(self, node_name: str, params=None):
        super().__init__(node_name)
        self.params = (
            params
            or type("Params", (), {"update_rate": 10.0, "node_specific_params": {}})()
        )
        self.interface_factory = self  # Self-referential for legacy compatibility


# ===== COMPATIBILITY STUBS =====


class NodeParameters:
    @classmethod
    def for_navigation(cls):
        return type(
            "Params",
            (),
            {"update_rate": 20.0, "node_specific_params": {"waypoint_tolerance": 1.0}},
        )()


class MessagePipeline:
    def __init__(self, logger, tracer=None):
        self.logger = logger
        self.steps = []

    def add_step(self, func, name):
        self.steps.append((func, name))
        return self

    def process(self, data):
        curr = data
        for f, n in self.steps:
            curr = f(curr)
        return type("Res", (), {"value": curr})()


# ===== UTILS =====


def safe_execute(func: Callable, *args, **kwargs) -> Result:
    try:
        return func(*args, **kwargs), None
    except Exception as e:
        return None, str(e)


def load_config_file(path: str) -> Dict[str, Any]:
    if not os.path.exists(path):
        return {}
    with open(path, "r") as f:
        return yaml.safe_load(f) or {}
