#!/usr/bin/env python3
"""
Simplified Utilities Module for URC 2026 Competition

This module provides basic utility functions for the competition codebase.
"""

import os
import yaml
import json
from typing import Any, Dict, Optional, Callable, TypeVar, Union
from dataclasses import dataclass
from enum import Enum


# Type definitions
T = TypeVar('T')


class ValidationError(Exception):
    """Raised when validation fails."""
    pass


class ProcessingError(Exception):
    """Raised when processing fails."""
    pass


@dataclass
class OperationResult:
    """Result of an operation with success/failure status."""
    success: bool
    data: Any = None
    error: Optional[str] = None


@dataclass
class Failure:
    """Represents a failure with error details."""
    message: str
    code: Optional[str] = None
    details: Optional[Dict[str, Any]] = None


@dataclass
class NodeParameters:
    """Container for ROS2 node parameters."""
    name: str
    namespace: str = ""
    parameters: Dict[str, Any] = None

    def __post_init__(self):
        if self.parameters is None:
            self.parameters = {}


@dataclass
class MessagePipeline:
    """Pipeline for processing messages."""
    name: str
    processors: list[Callable] = None
    validators: list[Callable] = None

    def __post_init__(self):
        if self.processors is None:
            self.processors = []
        if self.validators is None:
            self.validators = []


class AutonomyNode:
    """Simplified base class for autonomy nodes."""
    def __init__(self, name: str, namespace: str = ""):
        self.name = name
        self.namespace = namespace
        self.logger = self._setup_logger()

    def _setup_logger(self):
        """Set up basic logging."""
        import logging
        logger = logging.getLogger(f"{self.namespace}.{self.name}" if self.namespace else self.name)
        logger.setLevel(logging.INFO)
        if not logger.handlers:
            handler = logging.StreamHandler()
            formatter = logging.Formatter('%(asctime)s - %(name)s - %(levelname)s - %(message)s')
            handler.setFormatter(formatter)
            logger.addHandler(handler)
        return logger


def safe_execute(func: Callable[[], T], default: T = None) -> tuple[T, Optional[Exception]]:
    """
    Execute a function safely, returning result and any exception.

    Args:
        func: Function to execute
        default: Default value to return on failure

    Returns:
        Tuple of (result, exception) where exception is None on success
    """
    try:
        return func(), None
    except Exception as e:
        return default, e


def get_validated_parameter(params: Dict[str, Any], key: str, expected_type: type,
                          default: Any = None, required: bool = False) -> Any:
    """
    Get and validate a parameter from a dictionary.

    Args:
        params: Parameter dictionary
        key: Parameter key
        expected_type: Expected type
        default: Default value
        required: Whether parameter is required

    Returns:
        Validated parameter value

    Raises:
        ValidationError: If parameter is invalid or missing when required
    """
    if key not in params:
        if required:
            raise ValidationError(f"Required parameter '{key}' is missing")
        return default

    value = params[key]
    if not isinstance(value, expected_type):
        try:
            # Try to convert
            value = expected_type(value)
        except (ValueError, TypeError):
            raise ValidationError(f"Parameter '{key}' must be of type {expected_type.__name__}")

    return value


def load_config_file(config_path: str) -> Dict[str, Any]:
    """
    Load configuration from YAML or JSON file.

    Args:
        config_path: Path to config file

    Returns:
        Configuration dictionary

    Raises:
        FileNotFoundError: If config file doesn't exist
        ValueError: If config file format is invalid
    """
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Configuration file not found: {config_path}")

    try:
        with open(config_path, 'r') as f:
            if config_path.endswith('.yaml') or config_path.endswith('.yml'):
                return yaml.safe_load(f) or {}
            elif config_path.endswith('.json'):
                return json.load(f) or {}
            else:
                raise ValueError(f"Unsupported config file format: {config_path}")
    except Exception as e:
        raise ValueError(f"Failed to load config file {config_path}: {e}")


def success(data: Any = None) -> OperationResult:
    """Create a successful operation result."""
    return OperationResult(success=True, data=data)


def failure(message: str, code: Optional[str] = None, details: Optional[Dict[str, Any]] = None) -> OperationResult:
    """Create a failed operation result."""
    return OperationResult(success=False, error=message, data=Failure(message, code, details))
