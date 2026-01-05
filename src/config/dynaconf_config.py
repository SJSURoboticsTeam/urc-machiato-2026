#!/usr/bin/env python3
"""
Dynaconf Configuration Management System

Advanced configuration management using Dynaconf:
- Environment-aware configurations
- Runtime configuration updates
- Validation and type checking
- Multiple format support (YAML, JSON, TOML, etc.)
- Secret management and encryption

Author: URC 2026 Configuration Team
"""

import os
import tempfile
from pathlib import Path
from typing import Any, Dict, List, Optional, Union
import logging

try:
    from dynaconf import Dynaconf, ValidationError
    from dynaconf.utils.boxing import DynaBox
    DYNAONF_AVAILABLE = True
except ImportError:
    DYNAONF_AVAILABLE = False
    Dynaconf = None
    ValidationError = Exception

logger = logging.getLogger(__name__)


class URCDynaconf:
    """
    URC 2026 Configuration Management using Dynaconf.

    Provides advanced configuration management with:
    - Environment-based configuration loading
    - Runtime configuration updates
    - Validation and type checking
    - Multiple configuration sources
    - Hot reloading capabilities
    """

    def __init__(self, config_dir: str = "config", environment: str = "development"):
        """
        Initialize Dynaconf configuration system.

        Args:
            config_dir: Directory containing configuration files
            environment: Default environment (development, competition, etc.)
        """
        if not DYNAONF_AVAILABLE:
            raise ImportError("Dynaconf required for advanced configuration management")

        self.config_dir = Path(config_dir)
        self.environment = environment
        self._config = None

        # Initialize configuration
        self._initialize_config()

    def _initialize_config(self):
        """Initialize Dynaconf with URC-specific settings."""
        # Define configuration sources in priority order
        settings_files = [
            self.config_dir / "rover.yaml",           # Base configuration
            self.config_dir / f"{self.environment}.yaml",  # Environment-specific
            self.config_dir / "local.yaml",           # Local overrides (optional)
        ]

        # Filter to existing files
        existing_files = [f for f in settings_files if f.exists()]

        # Dynaconf configuration
        self._config = Dynaconf(
            settings_files=existing_files,
            environments=True,
            envvar_prefix="URC",
            merge_enabled=True,
            load_dotenv=True,
            dotenv_path=self.config_dir / ".env",
            secrets=self.config_dir / "secrets.yaml",  # For sensitive data
            includes=[self.config_dir / "includes"],   # Include additional config files
            preload=["src.config.validators"],         # Load validation rules
        )

        # Set environment
        os.environ.setdefault("URC_ENV", self.environment)

        # Add custom validators
        self._setup_validators()

        logger.info(f"Dynaconf configuration initialized for environment: {self.environment}")

    def _setup_validators(self):
        """Setup configuration validators."""
        try:
            # Add validation rules
            self._config.validators.register(
                # Simulation parameters
                ValidationError(
                    "simulation_update_rate_hz",
                    must_exist=True,
                    gte=1,
                    lte=100,
                    is_type_of=(int, float)
                ),
                ValidationError(
                    "simulation_odom_noise",
                    must_exist=True,
                    gte=0.0,
                    lte=1.0,
                    is_type_of=(int, float)
                ),

                # Mission parameters
                ValidationError(
                    "mission_execution_rate_hz",
                    must_exist=True,
                    gte=1,
                    lte=50,
                    is_type_of=(int, float)
                ),
                ValidationError(
                    "mission_max_speed_mps",
                    must_exist=True,
                    gte=0.0,
                    lte=5.0,
                    is_type_of=(int, float)
                ),

                # Safety parameters
                ValidationError(
                    "safety_distance_meters",
                    must_exist=True,
                    gt=0.0,
                    lte=2.0,
                    is_type_of=(int, float)
                ),
                ValidationError(
                    "safety_max_speed_mps",
                    must_exist=True,
                    gte=0.0,
                    lte=5.0,
                    is_type_of=(int, float)
                ),
                ValidationError(
                    "safety_thermal_limit_celsius",
                    must_exist=True,
                    gte=50,
                    lte=100,
                    is_type_of=(int, float)
                ),

                # Hardware settings
                ValidationError(
                    "hardware_use_mock",
                    must_exist=True,
                    is_type_of=bool
                ),

                # Waypoints validation
                ValidationError(
                    "waypoints",
                    is_type_of=list,
                    when=ValidationError("waypoints", must_exist=True)
                )
            )

        except Exception as e:
            logger.warning(f"Validator setup failed: {e}")

    def get(self, key: str, default: Any = None) -> Any:
        """
        Get configuration value.

        Args:
            key: Configuration key (dot notation supported)
            default: Default value if key not found

        Returns:
            Configuration value
        """
        try:
            return self._config.get(key, default)
        except Exception as e:
            logger.warning(f"Failed to get config {key}: {e}")
            return default

    def set(self, key: str, value: Any, persist: bool = False) -> bool:
        """
        Set configuration value.

        Args:
            key: Configuration key
            value: Value to set
            persist: Whether to persist to file

        Returns:
            True if successful
        """
        try:
            self._config[key] = value

            if persist:
                self.save_config()

            logger.debug(f"Configuration set: {key} = {value}")
            return True

        except Exception as e:
            logger.error(f"Failed to set config {key}: {e}")
            return False

    def update(self, updates: Dict[str, Any], persist: bool = False) -> bool:
        """
        Update multiple configuration values.

        Args:
            updates: Dictionary of key-value pairs
            persist: Whether to persist changes

        Returns:
            True if successful
        """
        try:
            for key, value in updates.items():
                self._config[key] = value

            if persist:
                self.save_config()

            logger.debug(f"Configuration updated: {list(updates.keys())}")
            return True

        except Exception as e:
            logger.error(f"Failed to update config: {e}")
            return False

    def save_config(self, filename: str = None) -> bool:
        """
        Save current configuration to file.

        Args:
            filename: Optional filename (defaults to environment-specific file)

        Returns:
            True if successful
        """
        try:
            if filename is None:
                filename = f"{self.environment}.yaml"

            output_path = self.config_dir / filename

            # Convert to dictionary for YAML export
            config_dict = dict(self._config)

            import yaml
            with open(output_path, 'w') as f:
                yaml.dump(config_dict, f, default_flow_style=False, sort_keys=False)

            logger.info(f"Configuration saved to {output_path}")
            return True

        except Exception as e:
            logger.error(f"Failed to save config: {e}")
            return False

    def reload_config(self) -> bool:
        """
        Reload configuration from files.

        Returns:
            True if successful
        """
        try:
            self._config.reload()
            logger.info("Configuration reloaded")
            return True

        except Exception as e:
            logger.error(f"Failed to reload config: {e}")
            return False

    def validate_config(self) -> List[str]:
        """
        Validate current configuration.

        Returns:
            List of validation errors (empty if valid)
        """
        errors = []

        try:
            # Run Dynaconf validation
            self._config.validators.validate()

            # Additional URC-specific validation
            errors.extend(self._validate_urc_specific())

        except ValidationError as e:
            errors.append(str(e))
        except Exception as e:
            errors.append(f"Validation error: {e}")

        if errors:
            logger.warning(f"Configuration validation failed: {errors}")

        return errors

    def _validate_urc_specific(self) -> List[str]:
        """URC-specific validation rules."""
        errors = []

        # Check safety speed vs mission speed consistency
        safety_speed = self.get("safety_max_speed_mps", 0)
        mission_speed = self.get("mission_max_speed_mps", 0)

        if safety_speed > mission_speed:
            errors.append(
                f"Safety speed ({safety_speed}) should not exceed mission speed ({mission_speed})"
            )

        # Check waypoint structure
        waypoints = self.get("waypoints", [])
        if waypoints:
            for i, wp in enumerate(waypoints):
                required_fields = ["name", "x", "y"]
                missing = [field for field in required_fields if field not in wp]
                if missing:
                    errors.append(f"Waypoint {i} missing required fields: {missing}")

        # Check hardware consistency
        use_mock = self.get("hardware_use_mock", True)
        if not use_mock:
            # Check that real hardware endpoints are configured
            required_hw = [
                "hardware_camera_front_url",
                "hardware_camera_rear_url"
            ]
            for hw_param in required_hw:
                if not self.get(hw_param):
                    errors.append(f"Hardware parameter {hw_param} required when not using mock hardware")

        return errors

    def get_environment_config(self) -> Dict[str, Any]:
        """
        Get environment-specific configuration.

        Returns:
            Dictionary of environment configuration
        """
        return {
            'environment': self.environment,
            'config_dir': str(self.config_dir),
            'config_files': [str(f) for f in (self.config_dir / f"{self.environment}.yaml",) if f.exists()],
            'validation_errors': self.validate_config(),
            'is_valid': len(self.validate_config()) == 0
        }

    def export_config(self, format: str = "yaml") -> str:
        """
        Export current configuration in specified format.

        Args:
            format: Export format ("yaml", "json", "toml")

        Returns:
            Configuration as string
        """
        config_dict = dict(self._config)

        if format.lower() == "yaml":
            import yaml
            return yaml.dump(config_dict, default_flow_style=False, sort_keys=False)
        elif format.lower() == "json":
            import json
            return json.dumps(config_dict, indent=2)
        elif format.lower() == "toml":
            try:
                import toml
                return toml.dumps(config_dict)
            except ImportError:
                raise ImportError("toml package required for TOML export")
        else:
            raise ValueError(f"Unsupported export format: {format}")

    def get_config_tree(self) -> Dict[str, Any]:
        """
        Get complete configuration as nested dictionary.

        Returns:
            Nested configuration dictionary
        """
        return dict(self._config)

    def __getitem__(self, key: str) -> Any:
        """Allow dictionary-style access."""
        return self.get(key)

    def __setitem__(self, key: str, value: Any):
        """Allow dictionary-style assignment."""
        self.set(key, value)

    def __contains__(self, key: str) -> bool:
        """Check if key exists in configuration."""
        return key in self._config


# Global configuration instance
_config_instance = None

def get_urc_config(config_dir: str = "config", environment: str = "development") -> URCDynaconf:
    """
    Get global URC configuration instance.

    Args:
        config_dir: Configuration directory
        environment: Environment name

    Returns:
        URCDynaconf instance
    """
    global _config_instance

    if _config_instance is None:
        _config_instance = URCDynaconf(config_dir=config_dir, environment=environment)

    return _config_instance


def reset_global_config():
    """Reset global configuration instance (for testing)."""
    global _config_instance
    _config_instance = None


# Convenience functions for common operations
def config_get(key: str, default: Any = None) -> Any:
    """Get configuration value from global instance."""
    config = get_urc_config()
    return config.get(key, default)


def config_set(key: str, value: Any, persist: bool = False) -> bool:
    """Set configuration value in global instance."""
    config = get_urc_config()
    return config.set(key, value, persist)


def validate_current_config() -> List[str]:
    """Validate current global configuration."""
    config = get_urc_config()
    return config.validate_config()

