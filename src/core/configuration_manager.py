#!/usr/bin/env python3
"""
Centralized Configuration Manager with Validation

Provides unified configuration management with:
- YAML/JSON config files
- Runtime validation using Pydantic
- Environment variable overrides
- Hot-reload capabilities
- Type safety and auto-completion

Author: URC 2026 Configuration Team
"""

import os
import yaml
import json
from pathlib import Path
from typing import Dict, Any, Optional, List, Union
from dataclasses import dataclass, field
from enum import Enum

try:
    from pydantic import BaseModel, Field, ValidationError, validator
    from pydantic.fields import FieldInfo
    PYDANTIC_AVAILABLE = True
except ImportError:
    PYDANTIC_AVAILABLE = False
    # Fallback for validation
    class BaseModel:
        def __init__(self, **data): pass
    class Field: pass
    class ValidationError(Exception): pass

logger = None  # Will be set when imported


class Environment(Enum):
    """Deployment environments."""
    DEVELOPMENT = "development"
    TESTING = "testing"
    STAGING = "staging"
    PRODUCTION = "production"


class LogLevel(Enum):
    """Logging levels."""
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


# Pydantic Models for Configuration Validation

class ROS2Config(BaseModel):
    """ROS2 configuration."""
    namespace: str = Field(default="urc", description="ROS2 namespace")
    domain_id: Optional[int] = Field(default=None, description="DDS domain ID")
    use_sim_time: bool = Field(default=False, description="Use simulation time")
    qos_preset: str = Field(default="reliable", description="QoS preset")

    class Config:
        validate_assignment = True


class DatabaseConfig(BaseModel):
    """Database configuration."""
    type: str = Field(default="sqlite", description="Database type")
    path: str = Field(default="data/urc_database.db", description="Database path")
    connection_pool_size: int = Field(default=5, ge=1, le=20, description="Connection pool size")
    enable_migrations: bool = Field(default=True, description="Enable database migrations")
    backup_interval_hours: int = Field(default=24, ge=1, description="Backup interval")

    class Config:
        validate_assignment = True


class NetworkConfig(BaseModel):
    """Network configuration."""
    websocket_port: int = Field(default=8765, ge=1024, le=65535, description="WebSocket port")
    http_port: int = Field(default=8080, ge=1024, le=65535, description="HTTP API port")
    enable_cors: bool = Field(default=True, description="Enable CORS")
    max_connections: int = Field(default=100, ge=1, description="Max connections")
    connection_timeout_sec: float = Field(default=30.0, ge=1.0, description="Connection timeout")

    class Config:
        validate_assignment = True


class NavigationConfig(BaseModel):
    """Navigation system configuration."""
    update_rate_hz: float = Field(default=10.0, ge=1.0, le=100.0, description="Navigation update rate")
    waypoint_tolerance_m: float = Field(default=1.0, ge=0.1, description="Waypoint reach tolerance")
    max_linear_velocity_ms: float = Field(default=2.0, ge=0.1, description="Max linear velocity")
    max_angular_velocity_rads: float = Field(default=1.0, ge=0.1, description="Max angular velocity")
    obstacle_avoidance_enabled: bool = Field(default=True, description="Enable obstacle avoidance")
    path_planning_algorithm: str = Field(default="astar", description="Path planning algorithm")

    class Config:
        validate_assignment = True


class SafetyConfig(BaseModel):
    """Safety system configuration."""
    emergency_stop_timeout_sec: float = Field(default=5.0, ge=1.0, description="Emergency stop timeout")
    heartbeat_interval_sec: float = Field(default=1.0, ge=0.1, description="Heartbeat interval")
    max_sensor_age_sec: float = Field(default=2.0, ge=0.1, description="Max sensor data age")
    enable_circuit_breaker: bool = Field(default=True, description="Enable circuit breaker")
    failure_threshold_percent: float = Field(default=80.0, ge=0.0, le=100.0, description="Failure threshold")

    class Config:
        validate_assignment = True


class MonitoringConfig(BaseModel):
    """Monitoring and observability configuration."""
    enabled: bool = Field(default=True, description="Enable monitoring")
    metrics_interval_sec: float = Field(default=5.0, ge=1.0, description="Metrics collection interval")
    enable_prometheus: bool = Field(default=True, description="Enable Prometheus metrics")
    prometheus_port: int = Field(default=9090, ge=1024, le=65535, description="Prometheus port")
    log_level: str = Field(default="INFO", description="Logging level")
    enable_structured_logging: bool = Field(default=True, description="Enable structured logging")

    class Config:
        validate_assignment = True


class SystemConfig(BaseModel):
    """Complete system configuration."""
    environment: str = Field(default="development", description="Deployment environment")
    version: str = Field(default="1.0.0", description="System version")

    ros2: ROS2Config = Field(default_factory=ROS2Config, description="ROS2 configuration")
    database: DatabaseConfig = Field(default_factory=DatabaseConfig, description="Database configuration")
    network: NetworkConfig = Field(default_factory=NetworkConfig, description="Network configuration")
    navigation: NavigationConfig = Field(default_factory=NavigationConfig, description="Navigation configuration")
    safety: SafetyConfig = Field(default_factory=SafetyConfig, description="Safety configuration")
    monitoring: MonitoringConfig = Field(default_factory=MonitoringConfig, description="Monitoring configuration")

    class Config:
        validate_assignment = True


class ConfigurationManager:
    """
    Centralized configuration management with validation.

    Features:
    - Multiple config file formats (YAML, JSON)
    - Environment variable overrides
    - Runtime validation
    - Hot-reload capabilities
    - Type safety with Pydantic
    """

    def __init__(self, config_dir: Union[str, Path] = "config"):
        self.config_dir = Path(config_dir)
        self.config_dir.mkdir(exist_ok=True)

        self.current_config: Optional[SystemConfig] = None
        self.config_files: Dict[str, Path] = {}
        self.environment_overrides: Dict[str, Any] = {}

        self._scan_config_files()
        self._load_environment_overrides()

    def _scan_config_files(self):
        """Scan for available configuration files."""
        config_files = list(self.config_dir.glob("*.yaml")) + list(self.config_dir.glob("*.yml")) + list(self.config_dir.glob("*.json"))

        for config_file in config_files:
            env_name = config_file.stem.lower()
            self.config_files[env_name] = config_file

    def _load_environment_overrides(self):
        """Load environment variable overrides."""
        # Environment variable format: URC_CONFIG_<SECTION>_<KEY>
        # Example: URC_CONFIG_NETWORK_WEBSOCKET_PORT=8766

        prefix = "URC_CONFIG_"
        for env_key, env_value in os.environ.items():
            if env_key.startswith(prefix):
                config_path = env_key[len(prefix):].lower().split('_')
                if len(config_path) >= 2:
                    section = config_path[0]
                    key = '_'.join(config_path[1:])

                    if section not in self.environment_overrides:
                        self.environment_overrides[section] = {}

                    # Try to convert value to appropriate type
                    try:
                        # Try int
                        env_value = int(env_value)
                    except ValueError:
                        try:
                            # Try float
                            env_value = float(env_value)
                        except ValueError:
                            # Try boolean
                            if env_value.lower() in ('true', 'false'):
                                env_value = env_value.lower() == 'true'
                            # Otherwise keep as string

                    self.environment_overrides[section][key] = env_value

    def load_config(self, environment: str = "development") -> SystemConfig:
        """
        Load configuration for specified environment.

        Args:
            environment: Environment name (development, testing, production)

        Returns:
            Validated SystemConfig object

        Raises:
            ValidationError: If configuration is invalid
            FileNotFoundError: If config file doesn't exist
        """
        if environment not in self.config_files:
            raise FileNotFoundError(f"Configuration file for environment '{environment}' not found")

        config_file = self.config_files[environment]

        # Load config from file
        if config_file.suffix.lower() in ('.yaml', '.yml'):
            with open(config_file, 'r') as f:
                config_data = yaml.safe_load(f)
        elif config_file.suffix.lower() == '.json':
            with open(config_file, 'r') as f:
                config_data = json.load(f)
        else:
            raise ValueError(f"Unsupported config file format: {config_file.suffix}")

        # Set environment in config
        config_data['environment'] = environment

        # Apply environment overrides
        config_data = self._apply_environment_overrides(config_data)

        # Validate with Pydantic
        if PYDANTIC_AVAILABLE:
            try:
                self.current_config = SystemConfig(**config_data)
            except ValidationError as e:
                raise ValidationError(f"Configuration validation failed: {e}")
        else:
            # Fallback without validation
            print("⚠️ Pydantic not available, skipping validation")
            self.current_config = SystemConfig(**config_data)

        return self.current_config

    def _apply_environment_overrides(self, config_data: Dict[str, Any]) -> Dict[str, Any]:
        """Apply environment variable overrides to config."""
        for section, overrides in self.environment_overrides.items():
            if section in config_data:
                if isinstance(config_data[section], dict):
                    config_data[section].update(overrides)
                else:
                    config_data[section] = overrides

        return config_data

    def get_config(self) -> SystemConfig:
        """Get current configuration."""
        if self.current_config is None:
            raise RuntimeError("Configuration not loaded. Call load_config() first.")
        return self.current_config

    def update_config(self, updates: Dict[str, Any]) -> SystemConfig:
        """Update configuration at runtime."""
        if self.current_config is None:
            raise RuntimeError("Configuration not loaded. Call load_config() first.")

        # Apply updates
        for key, value in updates.items():
            if hasattr(self.current_config, key):
                setattr(self.current_config, key, value)

        # Re-validate
        if PYDANTIC_AVAILABLE:
            try:
                self.current_config = SystemConfig(**self.current_config.dict())
            except ValidationError as e:
                raise ValidationError(f"Configuration update validation failed: {e}")

        return self.current_config

    def save_config(self, environment: str = None) -> bool:
        """Save current configuration to file."""
        if self.current_config is None:
            return False

        env = environment or self.current_config.environment
        config_file = self.config_files.get(env)

        if not config_file:
            config_file = self.config_dir / f"{env}.yaml"

        try:
            config_dict = self.current_config.dict() if PYDANTIC_AVAILABLE else self.current_config.__dict__

            with open(config_file, 'w') as f:
                yaml.dump(config_dict, f, default_flow_style=False, sort_keys=False)

            return True
        except Exception as e:
            print(f"❌ Failed to save configuration: {e}")
            return False

    def validate_config(self, config_data: Dict[str, Any]) -> List[str]:
        """Validate configuration data and return error messages."""
        errors = []

        if PYDANTIC_AVAILABLE:
            try:
                SystemConfig(**config_data)
            except ValidationError as e:
                for error in e.errors():
                    field_path = '.'.join(str(loc) for loc in error['loc'])
                    errors.append(f"{field_path}: {error['msg']}")
        else:
            errors.append("Pydantic not available for validation")

        return errors

    def get_available_environments(self) -> List[str]:
        """Get list of available configuration environments."""
        return list(self.config_files.keys())

    def create_default_config(self, environment: str = "development") -> bool:
        """Create default configuration file for environment."""
        default_config = SystemConfig(environment=environment)
        config_file = self.config_dir / f"{environment}.yaml"

        try:
            with open(config_file, 'w') as f:
                yaml.dump(default_config.dict() if PYDANTIC_AVAILABLE else default_config.__dict__,
                         f, default_flow_style=False, sort_keys=False)

            self.config_files[environment] = config_file
            return True
        except Exception as e:
            print(f"❌ Failed to create default config: {e}")
            return False

    def watch_config_changes(self, callback: callable):
        """Watch for configuration file changes (basic implementation)."""
        # In a full implementation, this would use inotify or similar
        # For now, just a placeholder
        print("⚠️ Config watching not implemented (requires additional libraries)")


# Global configuration manager instance
_config_manager = None

def get_config_manager(config_dir: Union[str, Path] = "config") -> ConfigurationManager:
    """Get global configuration manager instance."""
    global _config_manager
    if _config_manager is None:
        _config_manager = ConfigurationManager(config_dir)
    return _config_manager

def load_system_config(environment: str = "development") -> SystemConfig:
    """Load system configuration for environment."""
    manager = get_config_manager()
    return manager.load_config(environment)

def get_system_config() -> SystemConfig:
    """Get current system configuration."""
    manager = get_config_manager()
    return manager.get_config()

# Export key components
__all__ = [
    'ConfigurationManager',
    'SystemConfig',
    'ROS2Config',
    'DatabaseConfig',
    'NetworkConfig',
    'NavigationConfig',
    'SafetyConfig',
    'MonitoringConfig',
    'Environment',
    'LogLevel',
    'get_config_manager',
    'load_system_config',
    'get_system_config'
]
