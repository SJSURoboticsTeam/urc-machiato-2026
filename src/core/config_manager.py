#!/usr/bin/env python3
"""
URC 2026 Configuration Management Module

Provides type-safe, environment-aware configuration management using pydantic.
Supports validation, environment-specific settings, and nested configurations.

Author: URC 2026 Configuration Team
"""

from enum import Enum
import os
from pathlib import Path
from typing import Any

import orjson
from pydantic import BaseModel, Field, root_validator, validator
from pydantic_settings import BaseSettings, SettingsConfigDict


class Environment(str, Enum):
    """Deployment environments."""
    DEVELOPMENT = "development"
    TESTING = "testing"
    PRODUCTION = "production"
    COMPETITION = "competition"


class LogLevel(str, Enum):
    """Logging levels."""
    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


class SyncConfig(BaseModel):
    """Synchronization engine configuration."""
    max_sync_delay_ms: float = Field(default=50.0, gt=0, le=1000)
    temporal_consistency_threshold: float = Field(default=0.05, gt=0, le=1.0)
    buffer_max_size: int = Field(default=30, gt=0, le=1000)
    adaptive_buffering: bool = Field(default=True)
    camera_ids: list[str] = Field(default=["front", "rear", "left", "right"])

    @validator('camera_ids')
    def validate_camera_ids(cls, v):
        """Validate camera ID format."""
        for cam_id in v:
            if not cam_id.replace('_', '').replace('-', '').isalnum():
                raise ValueError(f"Invalid camera ID format: {cam_id}")
        return v


class NetworkConfig(BaseModel):
    """Network communication configuration."""
    retry_attempts: int = Field(default=3, ge=0, le=10)
    retry_backoff_factor: float = Field(default=0.5, gt=0)
    connection_timeout: float = Field(default=5.0, gt=0)
    circuit_breaker_threshold: int = Field(default=5, ge=1)
    circuit_breaker_timeout: float = Field(default=30.0, gt=0)


class SafetyConfig(BaseModel):
    """Safety system configuration."""
    emergency_stop_timeout: float = Field(default=1.0, gt=0)
    thermal_warning_threshold: float = Field(default=70.0, gt=0, le=150)
    battery_critical_threshold: float = Field(default=20.0, gt=0, le=100)
    motor_overtemp_threshold: float = Field(default=80.0, gt=0, le=150)
    safety_layer_count: int = Field(default=3, ge=1, le=5)


class MissionConfig(BaseModel):
    """Mission execution configuration."""
    max_mission_duration: float = Field(default=1800.0, gt=0)  # 30 minutes
    waypoint_tolerance: float = Field(default=1.0, gt=0)
    sample_collection_timeout: float = Field(default=300.0, gt=0)  # 5 minutes
    return_home_on_failure: bool = Field(default=True)
    autonomous_mode: bool = Field(default=False)


class DatabaseConfig(BaseModel):
    """Database configuration."""
    url: str = Field(default="sqlite:///urc_missions.db")
    connection_pool_size: int = Field(default=5, gt=0)
    connection_timeout: float = Field(default=30.0, gt=0)
    enable_migrations: bool = Field(default=True)


class RedisConfig(BaseModel):
    """Redis cache configuration."""
    host: str = Field(default="localhost")
    port: int = Field(default=6379, gt=0, le=65535)
    db: int = Field(default=0, ge=0)
    password: str | None = Field(default=None)
    ttl_seconds: int = Field(default=3600, gt=0)  # 1 hour default TTL


class APIConfig(BaseModel):
    """REST API configuration."""
    host: str = Field(default="0.0.0.0")
    port: int = Field(default=8000, gt=0, le=65535)
    cors_origins: list[str] = Field(default=["http://localhost:3000"])
    enable_docs: bool = Field(default=True)
    api_key_required: bool = Field(default=False)


class RoverConfig(BaseSettings):
    """
    Main rover configuration with environment-aware settings.

    Uses pydantic-settings for automatic environment variable loading
    and validation.
    """

    # Environment settings
    environment: Environment = Field(default=Environment.DEVELOPMENT)
    debug: bool = Field(default=False)

    # Logging
    log_level: LogLevel = Field(default=LogLevel.INFO)
    log_format: str = Field(default="json")
    log_file: str | None = Field(default=None)

    # Core systems
    sync: SyncConfig = Field(default_factory=SyncConfig)
    network: NetworkConfig = Field(default_factory=NetworkConfig)
    safety: SafetyConfig = Field(default_factory=SafetyConfig)
    mission: MissionConfig = Field(default_factory=MissionConfig)

    # Optional advanced features
    database: DatabaseConfig | None = Field(default=None)
    redis: RedisConfig | None = Field(default=None)
    api: APIConfig | None = Field(default=None)

    # Metadata
    version: str = Field(default="1.0.0")
    config_file_path: str | None = Field(default=None)

    model_config = SettingsConfigDict(
        env_prefix="ROVER_",
        env_nested_delimiter="__",
        env_file=".env",
        env_file_encoding="utf-8",
        case_sensitive=False,
        extra="ignore"
    )

    @root_validator(pre=True)
    def load_config_file(cls, values):
        """Load configuration from file if specified."""
        config_file = values.get('config_file_path') or os.getenv('ROVER_CONFIG_FILE')

        if config_file and Path(config_file).exists():
            try:
                with open(config_file, 'rb') as f:
                    file_config = orjson.loads(f.read())

                # Merge file config with environment values
                # Environment variables take precedence
                merged_config = {**file_config, **values}
                return merged_config

            except Exception as e:
                print(f"Warning: Failed to load config file {config_file}: {e}")

        return values

    @validator('environment', pre=True)
    def validate_environment(cls, v):
        """Validate environment value."""
        if isinstance(v, str):
            return Environment(v.lower())
        return v

    def save_to_file(self, file_path: str | Path) -> None:
        """Save current configuration to JSON file."""
        config_dict = self.dict()
        # Remove sensitive data before saving
        if 'redis' in config_dict and config_dict['redis']:
            config_dict['redis'].pop('password', None)

        file_path = Path(file_path)
        file_path.parent.mkdir(parents=True, exist_ok=True)

        with open(file_path, 'wb') as f:
            f.write(orjson.dumps(config_dict, option=orjson.OPT_INDENT_2))

    @classmethod
    def from_file(cls, file_path: str | Path) -> 'RoverConfig':
        """Load configuration from JSON file."""
        with open(file_path, 'rb') as f:
            config_data = orjson.loads(f.read())

        return cls(**config_data)

    def get_nested_value(self, key_path: str, default=None) -> Any:
        """Get nested configuration value using dot notation."""
        keys = key_path.split('.')
        value = self.dict()

        for key in keys:
            if isinstance(value, dict):
                value = value.get(key, default)
            else:
                return default

        return value

    def is_production(self) -> bool:
        """Check if running in production environment."""
        return self.environment in [Environment.PRODUCTION, Environment.COMPETITION]

    def is_development(self) -> bool:
        """Check if running in development environment."""
        return self.environment == Environment.DEVELOPMENT


# Global configuration instance
_config_instance: RoverConfig | None = None


def get_config() -> RoverConfig:
    """Get the global configuration instance."""
    global _config_instance
    if _config_instance is None:
        _config_instance = RoverConfig()
    return _config_instance


def reload_config() -> RoverConfig:
    """Reload configuration from environment and files."""
    global _config_instance
    _config_instance = RoverConfig()
    return _config_instance


def create_default_config(file_path: str | Path) -> None:
    """Create a default configuration file."""
    config = RoverConfig()
    config.save_to_file(file_path)
    print(f"Default configuration saved to {file_path}")


# Convenience functions for common config access
def get_sync_config() -> SyncConfig:
    """Get synchronization configuration."""
    return get_config().sync


def get_network_config() -> NetworkConfig:
    """Get network configuration."""
    return get_config().network


def get_safety_config() -> SafetyConfig:
    """Get safety configuration."""
    return get_config().safety


def get_mission_config() -> MissionConfig:
    """Get mission configuration."""
    return get_config().mission


# Example usage and validation
if __name__ == "__main__":
    # Create and display default configuration
    config = get_config()
    print("URC 2026 Rover Configuration Loaded:")
    print(f"Environment: {config.environment}")
    print(f"Debug Mode: {config.debug}")
    print(f"Log Level: {config.log_level}")

    print("\nSync Configuration:")
    print(f"  Max Sync Delay: {config.sync.max_sync_delay_ms}ms")
    print(f"  Cameras: {config.sync.camera_ids}")

    print("\nSafety Configuration:")
    print(f"  Thermal Warning: {config.safety.thermal_warning_threshold}°C")
    print(f"  Battery Critical: {config.safety.battery_critical_threshold}%")

    # Validate configuration
    try:
        config.validate(config.dict())
        print("\n✅ Configuration validation successful")
    except Exception as e:
        print(f"\n❌ Configuration validation failed: {e}")

    # Save example configuration
    example_path = Path("config/rover_config.example.json")
    config.save_to_file(example_path)
    print(f"\n💾 Example configuration saved to {example_path}")
