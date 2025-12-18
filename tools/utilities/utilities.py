#!/usr/bin/env python3
"""
URC 2026 Autonomy System - Core Utilities

Essential utilities for ROS2 nodes - simplified for easy understanding:

- safe_execute(): Error handling wrapper
- NodeLogger: Simple logging with context
- get_validated_parameter(): Parameter validation
- StateMachineNode: Basic state machine base class
- load_config_file(): YAML config loading

Author: URC 2026 Autonomy Team
"""

from typing import Any, Callable, Dict, List, Optional, Tuple

from rclpy.node import Node


# ===== ERROR HANDLING =====

Result = Tuple[Optional[Any], Optional[str]]
"""Type alias for (result, error) tuples. None result indicates success."""


def safe_execute(func: Callable, *args, **kwargs) -> Result:
    """
    Execute function with consistent error handling.

    Args:
        func: Function to execute
        *args: Positional arguments
        **kwargs: Keyword arguments

    Returns:
        Tuple of (result, error_message). None error indicates success.
    """
    try:
        result = func(*args, **kwargs)
        return result, None
    except Exception as e:
        return None, str(e)


# ===== LOGGING =====

class NodeLogger:
    """
    Simple logging helper for ROS2 nodes with consistent formatting.
    """

    def __init__(self, node: Node, component: str):
        """
        Initialize logger for a node.

        Args:
            node: ROS2 node instance
            component: Component name for context
        """
        self._logger = node.get_logger()
        self._component = component

    def _format_message(self, message: str, **context) -> str:
        """Format log message with context."""
        if context:
            ctx_str = " | ".join(f"{k}={v}" for k, v in context.items())
            return f"{message} | {ctx_str}"
        return message

    def debug(self, message: str, **context) -> None:
        """Log debug message."""
        self._logger.debug(self._format_message(message, **context))

    def info(self, message: str, **context) -> None:
        """Log info message."""
        self._logger.info(self._format_message(message, **context))

    def warn(self, message: str, **context) -> None:
        """Log warning message."""
        self._logger.warn(self._format_message(message, **context))

    def error(self, message: str, error: Optional[Exception] = None, **context) -> None:
        """Log error message."""
        if error:
            context.update({"error_type": type(error).__name__, "error_message": str(error)})
        self._logger.error(self._format_message(message, **context))

    def critical(self, message: str, **context) -> None:
        """Log critical message."""
        self._logger.fatal(self._format_message(message, **context))


# ===== PARAMETER MANAGEMENT =====

def get_validated_parameter(
    node: Node,
    name: str,
    default_value: Any,
    validator: Optional[Callable[[Any], bool]] = None,
    logger: Optional[NodeLogger] = None,
) -> Any:
    """
    Get ROS2 parameter with validation.

    Args:
        node: ROS2 node instance
        name: Parameter name
        default_value: Default value if invalid
        validator: Optional validation function
        logger: Optional logger for errors

    Returns:
        Validated parameter value or default
    """
    try:
        value = node.get_parameter(name).value
        if validator and not validator(value):
            error_msg = f"Parameter '{name}' failed validation, using default: {default_value}"
                if logger:
                logger.warn(error_msg, parameter=name, value=value, default=default_value)
                else:
                node.get_logger().warn(error_msg)
                return default_value
        return value
    except Exception as e:
        error_msg = f"Failed to get parameter '{name}': {e}, using default: {default_value}"
        if logger:
            logger.error(error_msg, parameter=name, default=default_value)
        else:
            node.get_logger().error(error_msg)
        return default_value


# ===== STATE MACHINE =====

class StateMachineNode(Node):
    """
    Simple ROS2 node with state machine functionality.
    """

    def __init__(self, node_name: str, initial_state: str = "idle"):
        """
        Initialize state machine node.

        Args:
            node_name: ROS2 node name
            initial_state: Initial state name
        """
        super().__init__(node_name)
        self._current_state = initial_state
        self._state_entry_time = self.get_clock().now()
        self._state_history: List[Dict[str, Any]] = []
        self._max_history_size = 10
        self.logger = NodeLogger(self, node_name)
        self.logger.info("State machine initialized", initial_state=initial_state)

    def transition_to(self, new_state: str, reason: str = "") -> None:
        """
        Transition to a new state with logging.

        Args:
            new_state: New state name
            reason: Optional reason for transition
        """
        old_state = self._current_state
        if old_state == new_state:
            return

        # Record transition
        transition = {
            "from": old_state,
            "to": new_state,
            "reason": reason,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "time_in_previous_state": self.time_in_state(),
        }
        self._state_history.append(transition)

        # Maintain history size
        if len(self._state_history) > self._max_history_size:
            self._state_history = self._state_history[-self._max_history_size // 2 :]

        # Update state
        self._current_state = new_state
        self._state_entry_time = self.get_clock().now()

        # Log transition
        self.logger.info(
            f"State transition: {old_state} → {new_state}",
            old_state=old_state,
            new_state=new_state,
            reason=reason,
        )

    @property
    def current_state(self) -> str:
        """Get current state name."""
        return self._current_state

    def time_in_state(self) -> float:
        """Get time spent in current state in seconds."""
        return (self.get_clock().now() - self._state_entry_time).nanoseconds / 1e9

    def get_state_history(self, limit: Optional[int] = None) -> List[Dict[str, Any]]:
        """
        Get recent state transition history.

        Args:
            limit: Maximum transitions to return

        Returns:
            List of transition dictionaries
        """
        history = self._state_history
        if limit:
            history = history[-limit:]
        return history


# ===== CONFIGURATION MANAGEMENT =====

def load_config_file(
    filename: str, required_keys: Optional[List[str]] = None, logger: Optional[NodeLogger] = None
) -> Optional[Dict[str, Any]]:
    """
    Load YAML configuration file with basic validation.

    Args:
        filename: Path to YAML file
        required_keys: Required keys to validate
        logger: Optional logger for errors

    Returns:
        Configuration dict or None if failed
    """
    try:
        import yaml

        with open(filename, "r") as f:
            config = yaml.safe_load(f) or {}

        # Validate required keys
        if required_keys:
            missing = [k for k in required_keys if k not in config]
            if missing:
                error_msg = f"Missing required config keys in {filename}: {missing}"
                if logger:
                    logger.error(error_msg, file=filename, missing_keys=missing)
                else:
                    print(f"ERROR: {error_msg}")
                return None

        if logger:
            logger.info("Configuration loaded successfully", file=filename, keys=list(config.keys()))
        return config

    except Exception as e:
        error_msg = f"Failed to load configuration file {filename}: {e}"
        if logger:
            logger.error(error_msg, file=filename, error=str(e))
        else:
            print(f"ERROR: {error_msg}")
        return None


# ===== COMPATIBILITY CLASSES =====
# Simple stubs for backward compatibility with existing code

class NodeParameters:
    """Simple parameter container for backward compatibility."""

    def __init__(self):
        self.update_rate = 10.0
        self.timeout = 5.0
        self.debug_mode = False

    @classmethod
    def for_navigation(cls):
        """Navigation-specific parameters."""
        params = cls()
        params.update_rate = 20.0
        params.timeout = 30.0
        return params

    @classmethod
    def for_vision(cls):
        """Vision-specific parameters."""
        params = cls()
        params.update_rate = 15.0
        params.timeout = 10.0
        return params

    def declare_all(self, node):
        """Declare parameters (compatibility method)."""
        pass

    def load_all(self, node, logger):
        """Load parameters (compatibility method)."""
        pass


class AutonomyNode(StateMachineNode):
    """Simple compatibility wrapper around StateMachineNode."""

    def __init__(self, node_name: str, params=None):
        super().__init__(node_name)
        self.params = params or NodeParameters()
        self.logger = NodeLogger(self, node_name)


class MessagePipeline:
    """Simple message processing pipeline for compatibility."""

    def __init__(self, logger, tracer=None):
        self.logger = logger
        self.tracer = tracer
        self.steps = []

    def add_step(self, step_func, step_name):
        """Add processing step."""
        self.steps.append((step_func, step_name))
        return self

    def process(self, data):
        """Process data through pipeline."""
        current_data = data
        for step_func, step_name in self.steps:
            try:
                current_data = step_func(current_data)
            except Exception as e:
                if self.logger:
                    self.logger.error(f"Pipeline step failed: {step_name}", error=e)
                raise
        return type('Result', (), {'value': current_data})()