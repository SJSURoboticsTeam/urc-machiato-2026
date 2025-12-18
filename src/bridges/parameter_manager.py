#!/usr/bin/env python3
"""
Parameter Manager - URC Competition Parameter Validation & Management

Handles ROS2 parameter validation, retrieval with defaults, and
configuration management for the competition bridge.
"""

from typing import Any, Dict, Optional

import rclpy
from constants import (
    ADAPTATION_RATE,
    BANDWIDTH_MEASUREMENT_WINDOW_SEC,
    BANDWIDTH_TARGET_UTILIZATION,
    DEFAULT_COMPETITION_LOG_FILE,
    DEFAULT_DDS_DOMAIN_ID,
    DEFAULT_MAX_WEBSOCKET_CLIENTS,
    DEFAULT_TELEMETRY_RATE_HZ,
    DEFAULT_WEBSOCKET_PORT,
    LATENCY_TARGET_MS,
    MAX_TELEMETRY_RATE_HZ,
    MIN_TELEMETRY_RATE_HZ,
)
from rclpy.node import Node


class ParameterManager:
    """
    Manages ROS2 parameter validation and retrieval with proper defaults.

    Provides safe parameter access with fallback defaults and validation
    for all competition bridge configuration parameters.
    """

    def __init__(self, node: Node, logger):
        """
        Initialize the Parameter Manager.

        Args:
            node: ROS2 node instance for parameter access
            logger: Logger instance for parameter operations
        """
        self.node = node
        self.logger = logger

        # Parameter configuration with defaults and validation
        self.parameter_config = {
            # WebSocket Configuration
            "websocket_port": {
                "default": DEFAULT_WEBSOCKET_PORT,
                "type": int,
                "min": 1024,
                "max": 65535,
                "description": "WebSocket server port",
            },
            "max_websocket_clients": {
                "default": DEFAULT_MAX_WEBSOCKET_CLIENTS,
                "type": int,
                "min": 1,
                "max": 100,
                "description": "Maximum WebSocket clients",
            },
            # Telemetry Configuration
            "telemetry_rate_hz": {
                "default": DEFAULT_TELEMETRY_RATE_HZ,
                "type": float,
                "min": MIN_TELEMETRY_RATE_HZ,
                "max": MAX_TELEMETRY_RATE_HZ,
                "description": "Telemetry publishing rate in Hz",
            },
            # Feature Toggles
            "enable_websocket_redundancy": {
                "default": True,
                "type": bool,
                "description": "Enable WebSocket redundancy system",
            },
            "redundancy_role": {
                "default": "primary",
                "type": str,
                "allowed_values": ["primary", "secondary", "tertiary", "emergency"],
                "description": "WebSocket redundancy role",
            },
            "enable_state_sync": {
                "default": True,
                "type": bool,
                "description": "Enable state synchronization",
            },
            "enable_dds_redundancy": {
                "default": True,
                "type": bool,
                "description": "Enable DDS domain redundancy",
            },
            "enable_dynamic_config": {
                "default": True,
                "type": bool,
                "description": "Enable dynamic configuration",
            },
            "enable_recovery_coordinator": {
                "default": True,
                "type": bool,
                "description": "Enable recovery coordinator",
            },
            # DDS Configuration
            "primary_domain_id": {
                "default": DEFAULT_DDS_DOMAIN_ID,
                "type": int,
                "min": 0,
                "max": 255,
                "description": "Primary DDS domain ID",
            },
            # Logging Configuration
            "competition_log_file": {
                "default": DEFAULT_COMPETITION_LOG_FILE,
                "type": str,
                "description": "Competition telemetry log file",
            },
            "enable_data_logging": {
                "default": True,
                "type": bool,
                "description": "Enable competition data logging",
            },
            # Adaptive Telemetry Parameters
            "adaptive_telemetry_enabled": {
                "default": True,
                "type": bool,
                "description": "Enable adaptive telemetry",
            },
            "min_telemetry_rate": {
                "default": MIN_TELEMETRY_RATE_HZ,
                "type": float,
                "min": 0.1,
                "max": 10.0,
                "description": "Minimum telemetry rate",
            },
            "max_telemetry_rate": {
                "default": MAX_TELEMETRY_RATE_HZ,
                "type": float,
                "min": 1.0,
                "max": 50.0,
                "description": "Maximum telemetry rate",
            },
            "bandwidth_target_utilization": {
                "default": BANDWIDTH_TARGET_UTILIZATION,
                "type": float,
                "min": 0.1,
                "max": 1.0,
                "description": "Target bandwidth utilization",
            },
            "latency_target_ms": {
                "default": LATENCY_TARGET_MS,
                "type": float,
                "min": 10.0,
                "max": 1000.0,
                "description": "Target latency in milliseconds",
            },
            "adaptation_rate": {
                "default": ADAPTATION_RATE,
                "type": float,
                "min": 0.01,
                "max": 1.0,
                "description": "Telemetry adaptation rate",
            },
            "bandwidth_measurement_window": {
                "default": BANDWIDTH_MEASUREMENT_WINDOW_SEC,
                "type": float,
                "min": 1.0,
                "max": 60.0,
                "description": "Bandwidth measurement window in seconds",
            },
            # Defensive Maximization Parameters
            "defensive_maximization_enabled": {
                "default": False,
                "type": bool,
                "description": "Enable defensive resource maximization",
            },
            "aggressive_channel_switching": {
                "default": False,
                "type": bool,
                "description": "Enable aggressive channel switching",
            },
            "bandwidth_reservation_ratio": {
                "default": 0.7,
                "type": float,
                "min": 0.1,
                "max": 1.0,
                "description": "Bandwidth reservation ratio",
            },
            "transmission_power_maximization": {
                "default": False,
                "type": bool,
                "description": "Enable transmission power maximization",
            },
        }

        # Declare all parameters with defaults
        self._declare_parameters()

    def _declare_parameters(self) -> None:
        """Declare all parameters with their defaults."""
        for param_name, config in self.parameter_config.items():
            try:
                self.node.declare_parameter(
                    param_name,
                    config["default"],
                    descriptor=rclpy.parameter_descriptor.ParameterDescriptor(
                        description=config.get("description", "")
                    ),
                )
            except Exception as e:
                self.logger.warning(f"Failed to declare parameter {param_name}: {e}")

    def get_parameter(self, param_name: str, default_value: Any = None) -> Any:
        """
        Get a parameter value with validation and fallback to default.

        Args:
            param_name: Name of the parameter to retrieve
            default_value: Optional override for default value

        Returns:
            Parameter value or appropriate default
        """
        if param_name not in self.parameter_config:
            self.logger.warning(f"Unknown parameter: {param_name}")
            return default_value

        config = self.parameter_config[param_name]
        use_default = default_value if default_value is not None else config["default"]

        try:
            # Get parameter value
            param_value = self.node.get_parameter(param_name).value

            # Validate parameter
            if not self._validate_parameter(param_name, param_value):
                self.logger.warning(
                    f"Parameter {param_name} validation failed, using default: {use_default}"
                )
                return use_default

            return param_value

        except Exception as e:
            self.logger.warning(
                f"Failed to get parameter {param_name}, using default {use_default}: {e}"
            )
            return use_default

    def _validate_parameter(self, param_name: str, value: Any) -> bool:
        """
        Validate a parameter value against its configuration.

        Args:
            param_name: Name of the parameter
            value: Value to validate

        Returns:
            True if valid, False otherwise
        """
        if param_name not in self.parameter_config:
            return False

        config = self.parameter_config[param_name]
        expected_type = config["type"]

        # Type checking
        if not isinstance(value, expected_type):
            self.logger.warning(
                f"Parameter {param_name} type mismatch: expected {expected_type.__name__}, "
                f"got {type(value).__name__}"
            )
            return False

        # Range validation for numeric types
        if expected_type in (int, float):
            if "min" in config and value < config["min"]:
                self.logger.warning(
                    f"Parameter {param_name} below minimum: {value} < {config['min']}"
                )
                return False
            if "max" in config and value > config["max"]:
                self.logger.warning(
                    f"Parameter {param_name} above maximum: {value} > {config['max']}"
                )
                return False

        # Allowed values validation
        if "allowed_values" in config:
            if value not in config["allowed_values"]:
                self.logger.warning(
                    f"Parameter {param_name} not in allowed values: {value} not in {config['allowed_values']}"
                )
                return False

        return True

    def set_parameter(self, param_name: str, value: Any) -> bool:
        """
        Set a parameter value with validation.

        Args:
            param_name: Name of the parameter to set
            value: Value to set

        Returns:
            True if successfully set, False otherwise
        """
        if param_name not in self.parameter_config:
            self.logger.warning(f"Unknown parameter: {param_name}")
            return False

        # Validate before setting
        if not self._validate_parameter(param_name, value):
            self.logger.error(f"Cannot set invalid parameter {param_name} = {value}")
            return False

        try:
            self.node.set_parameter(rclpy.parameter.Parameter(param_name, value))
            self.logger.info(f"Parameter {param_name} set to {value}")
            return True
        except Exception as e:
            self.logger.error(f"Failed to set parameter {param_name}: {e}")
            return False

    def get_all_parameters(self) -> Dict[str, Any]:
        """
        Get all parameter values.

        Returns:
            Dictionary of all parameter names and their current values
        """
        parameters = {}
        for param_name in self.parameter_config.keys():
            parameters[param_name] = self.get_parameter(param_name)
        return parameters

    def get_parameter_info(self, param_name: str) -> Optional[Dict[str, Any]]:
        """
        Get parameter configuration information.

        Args:
            param_name: Name of the parameter

        Returns:
            Parameter configuration dictionary or None if not found
        """
        if param_name not in self.parameter_config:
            return None

        config = self.parameter_config[param_name].copy()
        config["current_value"] = self.get_parameter(param_name)
        return config

    def get_parameter_summary(self) -> Dict[str, Any]:
        """
        Get a summary of all parameters for status reporting.

        Returns:
            Summary dictionary with parameter status
        """
        total_params = len(self.parameter_config)
        valid_params = 0

        for param_name in self.parameter_config.keys():
            try:
                value = self.get_parameter(param_name)
                if self._validate_parameter(param_name, value):
                    valid_params += 1
            except:
                pass

        return {
            "total_parameters": total_params,
            "valid_parameters": valid_params,
            "invalid_parameters": total_params - valid_params,
            "validation_rate": valid_params / total_params if total_params > 0 else 0.0,
        }
