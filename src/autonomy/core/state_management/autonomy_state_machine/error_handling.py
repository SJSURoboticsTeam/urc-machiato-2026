#!/usr/bin/env python3
"""
Error Handling Utilities - Standardized error handling for the state machine

Provides consistent error handling patterns, logging, and error categorization
across all state machine components.
"""

import logging
import traceback
from contextlib import contextmanager
from typing import Any, Callable, Dict, Optional


class StateMachineError(Exception):
    """Base exception for state machine operations."""

    def __init__(self, message: str, component: str = "", error_code: str = ""):
        self.component = component
        self.error_code = error_code
        super().__init__(f"[{component}] {message}")


class TransitionError(StateMachineError):
    """Raised when state transitions fail."""

    pass


class ContextError(StateMachineError):
    """Raised when context evaluation fails."""

    pass


class PolicyError(StateMachineError):
    """Raised when policy evaluation fails."""

    pass


class MonitoringError(StateMachineError):
    """Raised when monitoring operations fail."""

    pass


def handle_service_error(
    logger: logging.Logger,
    error: Exception,
    context: str = "",
    component: str = "",
    include_traceback: bool = False,
) -> None:
    """
    Standardized error handling for service operations.

    Args:
        logger: Logger instance to use for logging
        error: The exception that occurred
        context: Additional context about where the error occurred
        component: Which component the error occurred in
        include_traceback: Whether to include full traceback in logs
    """
    error_msg = f"{context}: {str(error)}" if context else str(error)
    component_prefix = f"[{component}] " if component else ""

    if isinstance(error, StateMachineError):
        logger.error(f"{component_prefix}{error_msg}")
    else:
        logger.error(f"{component_prefix}Unexpected error - {error_msg}")

    if include_traceback:
        logger.debug(f"{component_prefix}Traceback: {traceback.format_exc()}")


@contextmanager
def error_boundary(
    logger: logging.Logger,
    component: str = "",
    context: str = "",
    reraise: bool = True,
    error_type: type = Exception,
):
    """
    Context manager for standardized error handling.

    Usage:
        with error_boundary(logger, "AdaptiveStateMachine", "state transition"):
            # risky code here
            pass
    """
    try:
        yield
    except error_type as e:
        handle_service_error(logger, e, context, component, include_traceback=True)
        if reraise:
            raise
    except Exception as e:
        handle_service_error(
            logger,
            e,
            f"Unexpected error in {context}",
            component,
            include_traceback=True,
        )
        if reraise:
            raise


def safe_execute(
    func: Callable,
    logger: logging.Logger,
    component: str = "",
    context: str = "",
    default_return: Any = None,
    error_type: type = Exception,
) -> Any:
    """
    Execute a function safely with standardized error handling.

    Args:
        func: Function to execute
        logger: Logger instance
        component: Component name for error context
        context: Additional context
        default_return: Value to return on error
        error_type: Type of exception to catch

    Returns:
        Function result or default_return on error
    """
    try:
        return func()
    except error_type as e:
        handle_service_error(logger, e, context, component)
        return default_return
    except Exception as e:
        handle_service_error(logger, e, f"Unexpected error in {context}", component)
        return default_return


def validate_transition_request(
    current_state: Any, target_state: Any, reason: str = ""
) -> None:
    """
    Validate state transition request parameters.

    Args:
        current_state: Current state
        target_state: Target state
        reason: Transition reason

    Raises:
        ValueError: If parameters are invalid
    """
    if current_state is None:
        raise ValueError("Current state cannot be None")

    if target_state is None:
        raise ValueError("Target state cannot be None")

    if not reason or not reason.strip():
        raise ValueError("Transition reason cannot be empty")


def log_operation_start(
    logger: logging.Logger, operation: str, component: str = "", **kwargs
) -> None:
    """Log the start of an operation with structured context."""
    component_prefix = f"[{component}] " if component else ""
    context_str = " ".join(f"{k}={v}" for k, v in kwargs.items())
    logger.info(f"{component_prefix}Starting {operation} {context_str}".strip())


def log_operation_complete(
    logger: logging.Logger,
    operation: str,
    component: str = "",
    duration: Optional[float] = None,
    **kwargs,
) -> None:
    """Log the completion of an operation with structured context."""
    component_prefix = f"[{component}] " if component else ""
    duration_str = f" ({duration:.3f}s)" if duration else ""
    context_str = " ".join(f"{k}={v}" for k, v in kwargs.items())
    logger.info(
        f"{component_prefix}Completed {operation}{duration_str} {context_str}".strip()
    )


def create_error_summary(errors: list) -> Dict[str, Any]:
    """
    Create a summary of multiple errors for reporting.

    Args:
        errors: List of error dictionaries or exceptions

    Returns:
        Summary dictionary with error statistics
    """
    summary = {
        "total_errors": len(errors),
        "error_types": {},
        "components": {},
        "timestamps": [],
    }

    for error in errors:
        if isinstance(error, dict):
            error_type = error.get("type", "Unknown")
            component = error.get("component", "Unknown")
            timestamp = error.get("timestamp")
        else:
            error_type = type(error).__name__
            component = (
                getattr(error, "component", "Unknown")
                if hasattr(error, "component")
                else "Unknown"
            )
            timestamp = None

        summary["error_types"][error_type] = (
            summary["error_types"].get(error_type, 0) + 1
        )
        summary["components"][component] = summary["components"].get(component, 0) + 1

        if timestamp:
            summary["timestamps"].append(timestamp)

    return summary
