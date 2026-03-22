#!/usr/bin/env python3
"""
URC 2026 Unified Configuration - Dynaconf Core

Single source of truth for environment-aware configuration loading.
Combines the best of Dynaconf (environment merging, hot reload) with
Pydantic validation (type safety, schema generation).

Loading hierarchy:
1. config/rover.yaml (base configuration)
2. config/{environment}.yaml (environment overrides)
3. config/local.yaml (local developer overrides, gitignored)
4. Environment variables (URC_ prefix)

Author: URC 2026 Configuration Team
"""

import logging
import os
from pathlib import Path
from typing import Any, Dict, List, Optional

logger = logging.getLogger(__name__)

# Dynaconf imports with graceful fallback
try:
    from dynaconf import Dynaconf, Validator
    from dynaconf.utils.boxing import DynaBox

    DYNACONF_AVAILABLE = True
except ImportError:
    DYNACONF_AVAILABLE = False
    Dynaconf = None
    Validator = None
    DynaBox = None


def _get_project_root() -> Path:
    """Get the project root directory."""
    # Navigate from src/infrastructure/config to project root
    return Path(__file__).parent.parent.parent.parent


def _get_config_dir() -> Path:
    """Get the configuration directory."""
    return _get_project_root() / "config"


def _get_environment() -> str:
    """Get current environment from environment variable."""
    return os.environ.get("URC_ENV", "development")


class URCSettings:
    """
    Unified settings manager using Dynaconf.

    Provides environment-aware configuration loading with:
    - File-based configuration merging
    - Environment variable overrides
    - Hot reload capabilities
    - Validation hooks
    """

    _instance: Optional["URCSettings"] = None
    _settings: Optional[Dynaconf] = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if self._settings is not None:
            return

        if not DYNACONF_AVAILABLE:
            logger.warning("Dynaconf not available, using fallback configuration")
            self._settings = self._create_fallback_settings()
            return

        self._initialize_dynaconf()

    def _initialize_dynaconf(self):
        """Initialize Dynaconf with URC-specific settings."""
        config_dir = _get_config_dir()
        environment = _get_environment()

        # Build settings file list in priority order
        settings_files = [
            str(config_dir / "rover.yaml"),
            str(config_dir / f"{environment}.yaml"),
            str(config_dir / "local.yaml"),
        ]

        # Filter to existing files
        existing_files = [f for f in settings_files if Path(f).exists()]

        logger.debug(f"Loading configuration files: {existing_files}")

        self._settings = Dynaconf(
            settings_files=existing_files,
            envvar_prefix="URC",
            environments=True,
            env_switcher="URC_ENV",
            load_dotenv=True,
            dotenv_path=str(config_dir / ".env"),
            merge_enabled=True,
            root_path=str(config_dir),
        )

        # Register validators
        self._register_validators()

        logger.info(f"Configuration loaded for environment: {environment}")

    def _create_fallback_settings(self) -> Dict[str, Any]:
        """Create fallback settings when Dynaconf is unavailable."""
        return {
            "environment": _get_environment(),
            "debug": os.environ.get("URC_DEBUG", "false").lower() == "true",
            "log_level": os.environ.get("URC_LOG_LEVEL", "INFO"),
            "simulation": {
                "enabled": True,
                "update_rate_hz": 10.0,
                "gazebo_world": "urc_mars_world.world",
                "physics_engine": "ode",
                "real_time_factor": 1.0,
                "sensor_noise": True,
                "odom_noise": 0.01,
            },
            "navigation": {
                "update_rate_hz": 10.0,
                "waypoint_tolerance_m": 1.0,
                "max_linear_velocity_ms": 2.0,
                "max_angular_velocity_rads": 1.0,
                "acceleration_limit": 1.0,
                "deceleration_limit": 2.0,
                "obstacle_avoidance_enabled": True,
                "path_replanning_enabled": True,
            },
            "safety": {
                "emergency_stop_timeout": 1.0,
                "emergency_stop_enabled": True,
                "thermal_warning_threshold": 70.0,
                "battery_critical_threshold": 20.0,
                "motor_overtemp_threshold": 80.0,
                "max_speed_mps": 2.0,
                "distance_meters": 0.5,
                "safety_layer_count": 3,
                "heartbeat_timeout_seconds": 5.0,
                "geofencing_enabled": True,
                "communication_timeout_seconds": 30.0,
            },
            "network": {
                "websocket_port": 8765,
                "websocket_host": "localhost",
                "http_port": 8080,
                "retry_attempts": 3,
                "retry_backoff_factor": 0.5,
                "connection_timeout": 5.0,
                "circuit_breaker_threshold": 5,
                "circuit_breaker_timeout": 30.0,
                "enable_cors": True,
                "max_connections": 100,
                "can_enabled": True,
                "can_interface": "can0",
                "can_bitrate": 500000,
                "serial_enabled": False,
            },
            "hardware": {
                "use_mock": True,
            },
            "mission": {
                "max_mission_duration": 1800.0,
                "execution_rate_hz": 10.0,
                "waypoint_tolerance": 1.0,
                "sample_collection_timeout": 300.0,
                "return_home_on_failure": True,
                "autonomous_mode": False,
            },
            "waypoints": [],
        }

    def _register_validators(self):
        """Register Dynaconf validators for URC configuration."""
        if not DYNACONF_AVAILABLE or Validator is None:
            return

        try:
            self._settings.validators.register(
                # Simulation validators
                Validator(
                    "simulation.update_rate_hz",
                    default=10.0,
                    gte=1.0,
                    lte=100.0,
                ),
                Validator(
                    "simulation.real_time_factor",
                    default=1.0,
                    gte=0.1,
                    lte=10.0,
                ),
                # Navigation validators
                Validator(
                    "navigation.max_linear_velocity_ms",
                    default=2.0,
                    gte=0.1,
                    lte=5.0,
                ),
                Validator(
                    "navigation.waypoint_tolerance_m",
                    default=1.0,
                    gte=0.1,
                    lte=10.0,
                ),
                # Safety validators
                Validator(
                    "safety.battery_critical_threshold",
                    default=20.0,
                    gte=5.0,
                    lte=50.0,
                ),
                Validator(
                    "safety.thermal_warning_threshold",
                    default=70.0,
                    gte=40.0,
                    lte=100.0,
                ),
                # Network validators
                Validator(
                    "network.websocket_port",
                    default=8765,
                    gte=1024,
                    lte=65535,
                ),
            )

            self._settings.validators.validate()
            logger.debug("Configuration validation passed")

        except Exception as e:
            logger.warning(f"Configuration validation warning: {e}")

    def get(self, key: str, default: Any = None) -> Any:
        """Get configuration value by key (supports dot notation)."""
        if isinstance(self._settings, dict):
            # Fallback mode
            keys = key.split(".")
            value = self._settings
            for k in keys:
                if isinstance(value, dict):
                    value = value.get(k, default)
                else:
                    return default
            return value

        try:
            return self._settings.get(key, default)
        except Exception:
            return default

    def set(self, key: str, value: Any, persist: bool = False) -> bool:
        """Set configuration value (optionally persist to file)."""
        if isinstance(self._settings, dict):
            keys = key.split(".")
            target = self._settings
            for k in keys[:-1]:
                target = target.setdefault(k, {})
            target[keys[-1]] = value
            return True

        try:
            self._settings[key] = value
            if persist:
                self._persist_changes()
            return True
        except Exception as e:
            logger.error(f"Failed to set config {key}: {e}")
            return False

    def _persist_changes(self):
        """Persist configuration changes to file."""
        # Implementation for persisting changes would go here
        # For now, log a warning
        logger.warning("Configuration persistence not yet implemented")

    def reload(self) -> bool:
        """Reload configuration from files."""
        if isinstance(self._settings, dict):
            self._settings = self._create_fallback_settings()
            return True

        try:
            self._settings.reload()
            logger.info("Configuration reloaded")
            return True
        except Exception as e:
            logger.error(f"Failed to reload configuration: {e}")
            return False

    def to_dict(self) -> Dict[str, Any]:
        """Export current configuration as dictionary."""
        if isinstance(self._settings, dict):
            return self._settings.copy()

        return dict(self._settings)

    def get_environment(self) -> str:
        """Get current environment name."""
        return _get_environment()

    def is_development(self) -> bool:
        """Check if running in development environment."""
        return self.get_environment() == "development"

    def is_production(self) -> bool:
        """Check if running in production or competition environment."""
        return self.get_environment() in ["production", "competition"]

    def is_simulation(self) -> bool:
        """Check if simulation is enabled."""
        return self.get("simulation.enabled", True)

    def __getitem__(self, key: str) -> Any:
        """Dictionary-style access."""
        return self.get(key)

    def __setitem__(self, key: str, value: Any):
        """Dictionary-style assignment."""
        self.set(key, value)

    def __contains__(self, key: str) -> bool:
        """Check if key exists."""
        return self.get(key) is not None


# Module-level singleton
_settings_instance: Optional[URCSettings] = None


def get_settings() -> URCSettings:
    """Get the global settings instance."""
    global _settings_instance
    if _settings_instance is None:
        _settings_instance = URCSettings()
    return _settings_instance


def reset_settings():
    """Reset settings instance (primarily for testing)."""
    global _settings_instance
    _settings_instance = None
    URCSettings._instance = None
    URCSettings._settings = None


def config_get(key: str, default: Any = None) -> Any:
    """Convenience function to get config value."""
    return get_settings().get(key, default)


def config_set(key: str, value: Any, persist: bool = False) -> bool:
    """Convenience function to set config value."""
    return get_settings().set(key, value, persist)


# Environment helpers
def get_environment() -> str:
    """Get current environment."""
    return get_settings().get_environment()


def is_development() -> bool:
    """Check if in development mode."""
    return get_settings().is_development()


def is_production() -> bool:
    """Check if in production mode."""
    return get_settings().is_production()


def is_simulation() -> bool:
    """Check if simulation is enabled."""
    return get_settings().is_simulation()
