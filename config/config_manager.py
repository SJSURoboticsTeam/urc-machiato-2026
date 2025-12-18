#!/usr/bin/env python3
"""
Configuration Management System - Competition Ready
Simple, validated configuration management with environment support.
"""

import json
import os
from pathlib import Path
from typing import Any, Dict, List, Optional

import yaml


class ConfigurationManager:
    """
    Simple configuration manager with validation.

    Features:
    - Environment-aware configurations
    - Schema validation
    - Default value handling
    - Configuration override support
    """

    def __init__(self, config_dir: str = "config"):
        self.config_dir = Path(config_dir)
        self.config_cache = {}
        self.validation_errors = []

        # Define configuration schemas
        self.schemas = {
            "simulation": self._get_simulation_schema(),
            "mission": self._get_mission_schema(),
            "safety": self._get_safety_schema(),
            "network": self._get_network_schema(),
            "performance": self._get_performance_schema(),
        }

    def load_config(self, environment: str = "development") -> Dict[str, Any]:
        """
        Load configuration for specified environment.

        Priority order:
        1. Environment-specific config (e.g., competition.yaml)
        2. Local overrides (local.yaml)
        3. Default config (rover.yaml)
        """
        if environment in self.config_cache:
            return self.config_cache[environment]

        config_files = [
            self.config_dir / f"{environment}.yaml",
            self.config_dir / "local.yaml",
            self.config_dir / "rover.yaml",
        ]

        merged_config = {}

        for config_file in config_files:
            if config_file.exists():
                try:
                    with open(config_file, "r") as f:
                        file_config = yaml.safe_load(f) or {}
                        merged_config = self._deep_merge(merged_config, file_config)
                except Exception as e:
                    print(f"Warning: Failed to load {config_file}: {e}")

        # Validate configuration
        self._validate_config(merged_config)

        # Cache and return
        self.config_cache[environment] = merged_config
        return merged_config

    def get_value(
        self, key: str, environment: str = "development", default: Any = None
    ) -> Any:
        """Get configuration value with dot notation support."""
        config = self.load_config(environment)

        # Support dot notation (e.g., 'simulation.update_rate_hz')
        keys = key.split(".")
        value = config

        try:
            for k in keys:
                value = value[k]
            return value
        except (KeyError, TypeError):
            return default

    def set_value(self, key: str, value: Any, environment: str = "development"):
        """Set configuration value (for runtime overrides)."""
        config = self.load_config(environment)

        keys = key.split(".")
        target = config

        # Navigate to the parent of the target key
        for k in keys[:-1]:
            if k not in target:
                target[k] = {}
            target = target[k]

        # Set the value
        target[keys[-1]] = value

        # Clear cache to force reload
        if environment in self.config_cache:
            del self.config_cache[environment]

    def validate_config(self, environment: str = "development") -> bool:
        """Validate configuration for an environment."""
        config = self.load_config(environment)
        return self._validate_config(config)

    def save_config(self, config: Dict[str, Any], environment: str = "development"):
        """Save configuration to file."""
        config_file = self.config_dir / f"{environment}.yaml"
        self.config_dir.mkdir(exist_ok=True)

        with open(config_file, "w") as f:
            yaml.dump(config, f, default_flow_style=False, indent=2)

        # Clear cache
        if environment in self.config_cache:
            del self.config_cache[environment]

    def get_validation_errors(self) -> List[str]:
        """Get list of validation errors."""
        return self.validation_errors.copy()

    def _deep_merge(
        self, base: Dict[str, Any], overlay: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Deep merge two dictionaries."""
        result = base.copy()

        for key, value in overlay.items():
            if (
                key in result
                and isinstance(result[key], dict)
                and isinstance(value, dict)
            ):
                result[key] = self._deep_merge(result[key], value)
            else:
                result[key] = value

        return result

    def _validate_config(self, config: Dict[str, Any]) -> bool:
        """Validate configuration against schemas."""
        self.validation_errors = []

        for section_name, schema in self.schemas.items():
            if section_name in config:
                section_config = config[section_name]
                if not self._validate_section(section_config, schema, section_name):
                    return False

        return len(self.validation_errors) == 0

    def _validate_section(
        self, config: Dict[str, Any], schema: Dict[str, Any], section_name: str
    ) -> bool:
        """Validate a configuration section."""
        valid = True

        for key, rules in schema.items():
            if key not in config:
                if rules.get("required", False):
                    self.validation_errors.append(
                        f"Missing required config: {section_name}.{key}"
                    )
                    valid = False
                continue

            value = config[key]
            value_type = rules.get("type")

            # Type validation
            if value_type == "number":
                if not isinstance(value, (int, float)):
                    self.validation_errors.append(
                        f"Invalid type for {section_name}.{key}: expected number"
                    )
                    valid = False
                else:
                    # Range validation
                    min_val = rules.get("min")
                    max_val = rules.get("max")
                    if min_val is not None and value < min_val:
                        self.validation_errors.append(
                            f"Value too low for {section_name}.{key}: {value} < {min_val}"
                        )
                        valid = False
                    if max_val is not None and value > max_val:
                        self.validation_errors.append(
                            f"Value too high for {section_name}.{key}: {value} > {max_val}"
                        )
                        valid = False

            elif value_type == "boolean":
                if not isinstance(value, bool):
                    self.validation_errors.append(
                        f"Invalid type for {section_name}.{key}: expected boolean"
                    )
                    valid = False

            elif value_type == "string":
                if not isinstance(value, str):
                    self.validation_errors.append(
                        f"Invalid type for {section_name}.{key}: expected string"
                    )
                    valid = False

            elif value_type == "list":
                if not isinstance(value, list):
                    self.validation_errors.append(
                        f"Invalid type for {section_name}.{key}: expected list"
                    )
                    valid = False

        return valid

    # Schema definitions

    def _get_simulation_schema(self) -> Dict[str, Dict[str, Any]]:
        """Get simulation configuration schema."""
        return {
            "simulation_update_rate_hz": {
                "type": "number",
                "min": 1,
                "max": 100,
                "required": True,
            },
            "simulation_odom_noise": {
                "type": "number",
                "min": 0.0,
                "max": 1.0,
                "required": True,
            },
            "simulation_imu_noise": {
                "type": "number",
                "min": 0.0,
                "max": 0.1,
                "required": True,
            },
            "simulation_gps_noise": {
                "type": "number",
                "min": 0.0,
                "max": 10.0,
                "required": True,
            },
        }

    def _get_mission_schema(self) -> Dict[str, Dict[str, Any]]:
        """Get mission configuration schema."""
        return {
            "mission_execution_rate_hz": {
                "type": "number",
                "min": 1,
                "max": 50,
                "required": True,
            },
            "mission_timeout_seconds": {
                "type": "number",
                "min": 60,
                "max": 3600,
                "required": True,
            },
            "mission_max_speed_mps": {
                "type": "number",
                "min": 0.1,
                "max": 2.0,
                "required": True,
            },
        }

    def _get_safety_schema(self) -> Dict[str, Dict[str, Any]]:
        """Get safety configuration schema."""
        return {
            "safety_emergency_stop_enabled": {"type": "boolean", "required": True},
            "safety_collision_threshold_m": {
                "type": "number",
                "min": 0.1,
                "max": 5.0,
                "required": False,
            },
        }

    def _get_network_schema(self) -> Dict[str, Dict[str, Any]]:
        """Get network configuration schema."""
        return {
            "network_qos_depth_sensor": {
                "type": "number",
                "min": 1,
                "max": 100,
                "required": True,
            },
            "network_qos_depth_command": {
                "type": "number",
                "min": 1,
                "max": 50,
                "required": True,
            },
        }

    def _get_performance_schema(self) -> Dict[str, Dict[str, Any]]:
        """Get performance configuration schema."""
        return {
            "performance_monitoring_enabled": {"type": "boolean", "required": False},
            "performance_log_interval_seconds": {
                "type": "number",
                "min": 1,
                "max": 300,
                "required": False,
            },
        }


def get_environment() -> str:
    """Get current environment from environment variable."""
    return os.getenv("URC_ENV", "development")


def create_competition_config():
    """Create a competition-ready configuration."""
    config = {
        "simulation": {
            "simulation_update_rate_hz": 20,  # Faster updates for competition
            "simulation_odom_noise": 0.005,  # Lower noise for precision
            "simulation_imu_noise": 0.0005,  # Lower IMU noise
            "simulation_gps_noise": 0.2,  # Lower GPS noise
        },
        "mission": {
            "mission_execution_rate_hz": 20,  # Faster mission execution
            "mission_timeout_seconds": 900,  # 15 minutes per mission
            "mission_max_speed_mps": 1.5,  # Higher speed for competition
            "mission_heading_tolerance_degrees": 3.0,  # Tighter heading control
            "mission_emergency_stop_enabled": True,
            "mission_auto_recovery_enabled": True,
        },
        "safety": {
            "safety_emergency_stop_enabled": True,
            "safety_collision_threshold_m": 1.0,
            "safety_motor_current_limit_a": 15.0,
            "safety_battery_voltage_min_v": 11.0,
        },
        "network": {
            "network_qos_depth_sensor": 5,  # Keep more sensor messages
            "network_qos_depth_command": 10,  # Keep more command messages
            "network_qos_depth_status": 3,  # Keep status messages
            "network_timeout_ms": 500,  # Shorter timeouts
        },
        "performance": {
            "performance_monitoring_enabled": True,
            "performance_log_interval_seconds": 30,
            "performance_cpu_warning_percent": 80,
            "performance_memory_warning_mb": 500,
        },
    }

    return config


# Global configuration manager instance
_config_manager = None


def get_config_manager() -> ConfigurationManager:
    """Get global configuration manager instance."""
    global _config_manager
    if _config_manager is None:
        _config_manager = ConfigurationManager()
    return _config_manager


def get_config(key: str, default: Any = None, environment: str = None) -> Any:
    """Convenience function to get configuration value."""
    if environment is None:
        environment = get_environment()
    return get_config_manager().get_value(key, environment, default)


def set_config(key: str, value: Any, environment: str = None):
    """Convenience function to set configuration value."""
    if environment is None:
        environment = get_environment()
    get_config_manager().set_value(key, value, environment)


if __name__ == "__main__":
    # Test configuration management
    import sys

    manager = ConfigurationManager()

    # Test loading default config
    print("Loading development configuration...")
    config = manager.load_config("development")
    print(f"Config keys: {list(config.keys())}")

    # Test validation
    print("Validating configuration...")
    if manager.validate_config("development"):
        print("✅ Configuration is valid")
    else:
        print("❌ Configuration validation failed:")
        for error in manager.get_validation_errors():
            print(f"  - {error}")

    # Test competition config creation
    print("Creating competition configuration...")
    comp_config = create_competition_config()
    manager.save_config(comp_config, "competition")
    print("✅ Competition configuration saved")

    # Test competition config validation
    print("Validating competition configuration...")
    if manager.validate_config("competition"):
        print("✅ Competition configuration is valid")
        sys.exit(0)
    else:
        print("❌ Competition configuration validation failed:")
        for error in manager.get_validation_errors():
            print(f"  - {error}")
        sys.exit(1)
