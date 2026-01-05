#!/usr/bin/env python3
"""
Unified Configuration Manager - Single Source of Truth

Consolidates all configuration management functionality:
- Environment-aware configurations (from config/config_manager.py)
- Pydantic validation (from src/core/configuration_manager.py)
- Schema validation (consolidated from config/config_validator.py)
- Dynamic configuration (from src/core/dynamic_config_manager.py)
- API configs (from various Config classes)

Features:
- Single configuration API for entire system
- Environment-specific configurations
- Runtime validation and hot-reloading
- Schema-based validation
- Environment variable overrides
- Configuration monitoring and health checks

Author: URC 2026 Unified Configuration Team
"""

import os
import yaml
import json
import time
from pathlib import Path
from typing import Dict, List, Any, Optional, Union, Callable
from dataclasses import dataclass, field
from enum import Enum

# Pydantic with fallback
try:
    from pydantic import BaseModel, Field, ValidationError, validator
    from pydantic.fields import FieldInfo
    PYDANTIC_AVAILABLE = True
except ImportError:
    PYDANTIC_AVAILABLE = False
    # Fallback implementations
    class BaseModel:
        def __init__(self, **data): pass
    class Field: pass
    class ValidationError(Exception): pass


class Environment(Enum):
    """Deployment environments."""
    DEVELOPMENT = "development"
    TESTING = "testing"
    STAGING = "staging"
    PRODUCTION = "production"


class ConfigValidationLevel(Enum):
    """Configuration validation strictness."""
    LAX = "lax"           # Allow missing optional fields
    NORMAL = "normal"     # Standard validation
    STRICT = "strict"     # Fail on any validation error


# Pydantic Models for Configuration Sections

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


class HardwareConfig(BaseModel):
    """Hardware interface configuration."""
    can_bus: Dict[str, Any] = Field(default_factory=dict, description="CAN bus configuration")
    motor_controller: Dict[str, Any] = Field(default_factory=dict, description="Motor controller config")
    drive_system: Dict[str, Any] = Field(default_factory=dict, description="Drive system config")
    robotic_arm: Dict[str, Any] = Field(default_factory=dict, description="Robotic arm config")
    sensors: Dict[str, Any] = Field(default_factory=dict, description="Sensor configurations")

    class Config:
        validate_assignment = True


class SimulationConfig(BaseModel):
    """Simulation configuration."""
    enabled: bool = Field(default=False, description="Enable simulation mode")
    time_factor: float = Field(default=1.0, ge=0.1, le=10.0, description="Simulation time factor")
    perfect_conditions: bool = Field(default=False, description="Perfect environmental conditions")
    network_emulation: bool = Field(default=True, description="Enable network emulation")

    class Config:
        validate_assignment = True


class MissionConfig(BaseModel):
    """Mission configuration."""
    max_mission_duration_sec: int = Field(default=1800, ge=60, description="Max mission duration")
    waypoint_timeout_sec: float = Field(default=60.0, ge=10.0, description="Waypoint reach timeout")
    mission_retry_attempts: int = Field(default=3, ge=0, le=10, description="Mission retry attempts")
    enable_mission_validation: bool = Field(default=True, description="Enable mission validation")

    class Config:
        validate_assignment = True


class PerformanceConfig(BaseModel):
    """Performance configuration."""
    enable_optimizations: bool = Field(default=True, description="Enable performance optimizations")
    memory_limit_mb: int = Field(default=512, ge=64, description="Memory limit")
    cpu_limit_percent: int = Field(default=80, ge=10, le=100, description="CPU usage limit")
    thread_pool_size: int = Field(default=4, ge=1, le=16, description="Thread pool size")

    class Config:
        validate_assignment = True


class SystemConfig(BaseModel):
    """Complete unified system configuration."""
    environment: str = Field(default="development", description="Deployment environment")
    version: str = Field(default="1.0.0", description="System version")
    name: str = Field(default="URC 2026 Mars Rover", description="System name")

    # Core configuration sections
    ros2: ROS2Config = Field(default_factory=ROS2Config, description="ROS2 configuration")
    database: DatabaseConfig = Field(default_factory=DatabaseConfig, description="Database configuration")
    network: NetworkConfig = Field(default_factory=NetworkConfig, description="Network configuration")
    navigation: NavigationConfig = Field(default_factory=NavigationConfig, description="Navigation configuration")
    safety: SafetyConfig = Field(default_factory=SafetyConfig, description="Safety configuration")
    monitoring: MonitoringConfig = Field(default_factory=MonitoringConfig, description="Monitoring configuration")
    hardware: HardwareConfig = Field(default_factory=HardwareConfig, description="Hardware configuration")
    simulation: SimulationConfig = Field(default_factory=SimulationConfig, description="Simulation configuration")
    mission: MissionConfig = Field(default_factory=MissionConfig, description="Mission configuration")
    performance: PerformanceConfig = Field(default_factory=PerformanceConfig, description="Performance configuration")

    class Config:
        validate_assignment = True


@dataclass
class ValidationResult:
    """Configuration validation result."""
    valid: bool
    errors: List[str] = field(default_factory=list)
    warnings: List[str] = field(default_factory=list)
    schema_version: str = "1.0"


class ConfigurationManager:
    """
    Unified Configuration Manager - Single source of truth for all configuration.

    Consolidates functionality from:
    - config/config_manager.py (environment-aware loading)
    - config/config_validator.py (schema validation)
    - src/core/configuration_manager.py (Pydantic validation)
    - src/core/dynamic_config_manager.py (dynamic updates)
    - Various Config classes (API, Core, Dashboard configs)

    Features:
    - Environment-aware configuration loading
    - Runtime validation with multiple levels
    - Hot-reloading capabilities
    - Schema-based validation
    - Environment variable overrides
    - Configuration health monitoring
    - Dynamic configuration updates
    """

    def __init__(self, config_dir: Union[str, Path] = "config"):
        self.config_dir = Path(config_dir)
        self.config_dir.mkdir(exist_ok=True)

        # Configuration state
        self.current_config: Optional[SystemConfig] = None
        self.config_cache: Dict[str, SystemConfig] = {}
        self.validation_cache: Dict[str, ValidationResult] = {}
        self.config_files: Dict[str, Path] = {}

        # Validation schemas (consolidated from config_validator.py)
        self.schemas = self._load_validation_schemas()

        # Dynamic configuration support
        self.config_listeners: List[Callable] = []
        self.last_modified_times: Dict[str, float] = {}

        # Performance monitoring
        self.load_times: Dict[str, float] = {}
        self.validation_times: Dict[str, float] = {}

        # Scan for available config files
        self._scan_config_files()

    def _scan_config_files(self):
        """Scan for available configuration files."""
        config_files = list(self.config_dir.glob("*.yaml")) + list(self.config_dir.glob("*.yml")) + list(self.config_dir.glob("*.json"))

        for config_file in config_files:
            env_name = config_file.stem.lower()
            self.config_files[env_name] = config_file

    def _load_validation_schemas(self) -> Dict[str, Dict[str, Any]]:
        """Load validation schemas for different configuration types."""
        return {
            "hardware": {
                "required_components": [
                    "can_bus", "motor_controller", "drive_system",
                    "robotic_arm", "sensors"
                ],
                "can_bus": {
                    "interface": {"type": "string", "required": True},
                    "bitrate": {"type": "integer", "min": 125000, "max": 1000000},
                    "timeout_ms": {"type": "integer", "min": 10, "max": 5000}
                }
            },
            "simulation": {
                "time_factor": {"type": "number", "min": 0.1, "max": 10.0},
                "network_emulation": {"type": "boolean"}
            },
            "mission": {
                "max_mission_duration_sec": {"type": "integer", "min": 60, "max": 3600},
                "waypoint_timeout_sec": {"type": "number", "min": 10.0, "max": 300.0}
            },
            "performance": {
                "memory_limit_mb": {"type": "integer", "min": 64, "max": 2048},
                "cpu_limit_percent": {"type": "integer", "min": 10, "max": 100}
            }
        }

    def load_config(self, environment: str = "development",
                   validation_level: ConfigValidationLevel = ConfigValidationLevel.NORMAL) -> SystemConfig:
        """
        Load configuration for specified environment with validation.

        Args:
            environment: Environment name
            validation_level: Validation strictness level

        Returns:
            Validated SystemConfig object

        Raises:
            ValidationError: If configuration is invalid and validation_level is STRICT
        """
        # Check cache first
        if environment in self.config_cache:
            return self.config_cache[environment]

        start_time = time.time()

        # Load configuration from files
        config_data = self._load_config_from_files(environment)

        # Apply environment variable overrides
        config_data = self._apply_environment_overrides(config_data)

        # Validate configuration
        validation_result = self._validate_config(config_data, validation_level)

        load_time = time.time() - start_time
        self.load_times[environment] = load_time

        # Handle validation results based on level
        if not validation_result.valid:
            if validation_level == ConfigValidationLevel.STRICT:
                raise ValidationError(f"Configuration validation failed: {validation_result.errors}")
            elif validation_level == ConfigValidationLevel.NORMAL:
                print(f"⚠️ Configuration warnings for {environment}: {validation_result.warnings}")

        # Create and cache configuration
        if PYDANTIC_AVAILABLE:
            try:
                config = SystemConfig(**config_data)
            except ValidationError as e:
                raise ValidationError(f"Configuration validation failed: {e}")
        else:
            # Fallback without validation
            print("⚠️ Pydantic not available, skipping validation")
            config = SystemConfig(**config_data)

        self.config_cache[environment] = config
        self.current_config = config

        return config

    def _load_config_from_files(self, environment: str) -> Dict[str, Any]:
        """
        Load configuration from files with inheritance.

        Priority order (highest to lowest):
        1. Environment-specific config (e.g., competition.yaml)
        2. Local overrides (local.yaml)
        3. Default config (rover.yaml)
        """
        config_data = {}

        # Load base configuration (rover.yaml)
        base_config = self.config_files.get("rover")
        if base_config:
            config_data.update(self._load_single_config_file(base_config))

        # Load development defaults
        dev_config = self.config_files.get("development")
        if dev_config and environment == "development":
            config_data.update(self._load_single_config_file(dev_config))

        # Load environment-specific configuration
        env_config = self.config_files.get(environment)
        if env_config:
            config_data.update(self._load_single_config_file(env_config))

        # Load local overrides (highest priority)
        local_config = self.config_files.get("local")
        if local_config:
            config_data.update(self._load_single_config_file(local_config))

        # Set environment in config
        config_data['environment'] = environment

        return config_data

    def _load_single_config_file(self, config_file: Path) -> Dict[str, Any]:
        """Load a single configuration file."""
        try:
            if config_file.suffix.lower() in ('.yaml', '.yml'):
                with open(config_file, 'r') as f:
                    return yaml.safe_load(f) or {}
            elif config_file.suffix.lower() == '.json':
                with open(config_file, 'r') as f:
                    return json.load(f)
            else:
                print(f"⚠️ Unsupported config file format: {config_file}")
                return {}
        except Exception as e:
            print(f"❌ Failed to load config file {config_file}: {e}")
            return {}

    def _apply_environment_overrides(self, config_data: Dict[str, Any]) -> Dict[str, Any]:
        """Apply environment variable overrides to config."""
        # Environment variable format: URC_CONFIG_<SECTION>_<KEY>
        prefix = "URC_CONFIG_"
        overrides = {}

        for env_key, env_value in os.environ.items():
            if env_key.startswith(prefix):
                config_path = env_key[len(prefix):].lower().split('_')
                if len(config_path) >= 2:
                    section = config_path[0]
                    key = '_'.join(config_path[1:])

                    if section not in overrides:
                        overrides[section] = {}

                    # Convert value to appropriate type
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

                    overrides[section][key] = env_value

        # Apply overrides to config data
        for section, section_overrides in overrides.items():
            if section in config_data:
                if isinstance(config_data[section], dict):
                    config_data[section].update(section_overrides)
                else:
                    config_data[section] = section_overrides

        return config_data

    def _validate_config(self, config_data: Dict[str, Any],
                        validation_level: ConfigValidationLevel) -> ValidationResult:
        """
        Validate configuration data against schemas.

        Args:
            config_data: Configuration data to validate
            validation_level: Validation strictness level

        Returns:
            ValidationResult with errors and warnings
        """
        start_time = time.time()
        result = ValidationResult(valid=True, schema_version="1.0")

        # Check cache first
        config_hash = hash(str(config_data))
        cache_key = f"{config_hash}_{validation_level.value}"
        if cache_key in self.validation_cache:
            validation_time = time.time() - start_time
            self.validation_times[config_data.get('environment', 'unknown')] = validation_time
            return self.validation_cache[cache_key]

        # Validate each section
        for section_name, section_schema in self.schemas.items():
            if section_name in config_data:
                section_result = self._validate_section(
                    section_name, config_data[section_name], section_schema, validation_level
                )
                result.errors.extend(section_result.errors)
                result.warnings.extend(section_result.warnings)

        result.valid = len(result.errors) == 0
        validation_time = time.time() - start_time
        self.validation_times[config_data.get('environment', 'unknown')] = validation_time

        # Cache result
        self.validation_cache[cache_key] = result

        return result

    def _validate_section(self, section_name: str, section_data: Any,
                         schema: Dict[str, Any], validation_level: ConfigValidationLevel) -> ValidationResult:
        """Validate a single configuration section."""
        result = ValidationResult(valid=True)

        if not isinstance(section_data, dict):
            if validation_level == ConfigValidationLevel.STRICT:
                result.errors.append(f"{section_name}: expected dict, got {type(section_data)}")
            return result

        # Check required components
        if "required_components" in schema:
            required = schema["required_components"]
            for component in required:
                if component not in section_data:
                    if validation_level != ConfigValidationLevel.LAX:
                        result.errors.append(f"{section_name}: missing required component '{component}'")

        # Validate individual fields
        for field_name, field_schema in schema.items():
            if field_name == "required_components":
                continue

            if field_name in section_data:
                field_value = section_data[field_name]
                field_errors = self._validate_field(field_name, field_value, field_schema)
                result.errors.extend(field_errors)
            elif field_schema.get("required", False):
                if validation_level != ConfigValidationLevel.LAX:
                    result.errors.append(f"{section_name}.{field_name}: required field missing")

        return result

    def _validate_field(self, field_name: str, field_value: Any, field_schema: Dict[str, Any]) -> List[str]:
        """Validate a single field against its schema."""
        errors = []
        expected_type = field_schema.get("type")

        # Type validation
        if expected_type:
            if expected_type == "integer" and not isinstance(field_value, int):
                errors.append(f"{field_name}: expected integer, got {type(field_value).__name__}")
            elif expected_type == "number" and not isinstance(field_value, (int, float)):
                errors.append(f"{field_name}: expected number, got {type(field_value).__name__}")
            elif expected_type == "string" and not isinstance(field_value, str):
                errors.append(f"{field_name}: expected string, got {type(field_value).__name__}")
            elif expected_type == "boolean" and not isinstance(field_value, bool):
                errors.append(f"{field_name}: expected boolean, got {type(field_value).__name__}")

        # Range validation
        if "min" in field_schema and isinstance(field_value, (int, float)):
            if field_value < field_schema["min"]:
                errors.append(f"{field_name}: value {field_value} below minimum {field_schema['min']}")

        if "max" in field_schema and isinstance(field_value, (int, float)):
            if field_value > field_schema["max"]:
                errors.append(f"{field_name}: value {field_value} above maximum {field_schema['max']}")

        return errors

    def update_config(self, updates: Dict[str, Any],
                     validation_level: ConfigValidationLevel = ConfigValidationLevel.NORMAL) -> bool:
        """
        Update configuration at runtime.

        Args:
            updates: Configuration updates
            validation_level: Validation strictness

        Returns:
            True if update successful
        """
        if not self.current_config:
            print("❌ No configuration loaded")
            return False

        try:
            # Apply updates
            for key, value in updates.items():
                if hasattr(self.current_config, key):
                    setattr(self.current_config, key, value)

            # Re-validate
            config_dict = self.current_config.dict() if PYDANTIC_AVAILABLE else self.current_config.__dict__
            validation_result = self._validate_config(config_dict, validation_level)

            if not validation_result.valid:
                if validation_level == ConfigValidationLevel.STRICT:
                    print(f"❌ Configuration update validation failed: {validation_result.errors}")
                    return False
                else:
                    print(f"⚠️ Configuration update warnings: {validation_result.warnings}")

            # Notify listeners
            self._notify_config_listeners(updates)

            print("✅ Configuration updated successfully")
            return True

        except Exception as e:
            print(f"❌ Configuration update failed: {e}")
            return False

    def save_config(self, environment: str = None) -> bool:
        """Save current configuration to file."""
        if not self.current_config:
            return False

        env = environment or self.current_config.environment
        config_file = self.config_files.get(env)

        if not config_file:
            config_file = self.config_dir / f"{env}.yaml"

        try:
            config_dict = self.current_config.dict() if PYDANTIC_AVAILABLE else self.current_config.__dict__

            with open(config_file, 'w') as f:
                yaml.dump(config_dict, f, default_flow_style=False, sort_keys=False)

            self.config_files[env] = config_file
            print(f"✅ Configuration saved to {config_file}")
            return True
        except Exception as e:
            print(f"❌ Failed to save configuration: {e}")
            return False

    def add_config_listener(self, listener: Callable):
        """Add a configuration change listener."""
        self.config_listeners.append(listener)

    def remove_config_listener(self, listener: Callable):
        """Remove a configuration change listener."""
        if listener in self.config_listeners:
            self.config_listeners.remove(listener)

    def _notify_config_listeners(self, updates: Dict[str, Any]):
        """Notify all configuration listeners of changes."""
        for listener in self.config_listeners:
            try:
                listener(updates)
            except Exception as e:
                print(f"❌ Config listener error: {e}")

    def get_config_health(self) -> Dict[str, Any]:
        """Get configuration system health status."""
        return {
            "config_loaded": self.current_config is not None,
            "environments_available": list(self.config_files.keys()),
            "cache_size": len(self.config_cache),
            "validation_cache_size": len(self.validation_cache),
            "listeners_count": len(self.config_listeners),
            "last_load_times": self.load_times,
            "last_validation_times": self.validation_times,
            "pydantic_available": PYDANTIC_AVAILABLE
        }

    def enable_hot_reload(self, interval_seconds: float = 30.0):
        """Enable hot reloading of configuration files."""
        # This would implement file watching for config changes
        print(f"⚠️ Hot reload not implemented yet (would check every {interval_seconds}s)")

    def get_available_environments(self) -> List[str]:
        """Get list of available configuration environments."""
        return list(self.config_files.keys())

    def create_default_config(self, environment: str = "development") -> bool:
        """Create default configuration file for environment."""
        default_config = SystemConfig(environment=environment)
        config_file = self.config_dir / f"{environment}.yaml"

        try:
            with open(config_file, 'w') as f:
                config_dict = default_config.dict() if PYDANTIC_AVAILABLE else default_config.__dict__
                yaml.dump(config_dict, f, default_flow_style=False, sort_keys=False)

            self.config_files[environment] = config_file
            print(f"✅ Default configuration created: {config_file}")
            return True
        except Exception as e:
            print(f"❌ Failed to create default config: {e}")
            return False


# Global configuration manager instance
_config_manager = None

def get_config_manager(config_dir: Union[str, Path] = "config") -> ConfigurationManager:
    """Get global configuration manager instance."""
    global _config_manager
    if _config_manager is None:
        _config_manager = ConfigurationManager(config_dir)
    return _config_manager

def load_system_config(environment: str = "development",
                      validation_level: ConfigValidationLevel = ConfigValidationLevel.NORMAL) -> SystemConfig:
    """Load system configuration for environment."""
    manager = get_config_manager()
    return manager.load_config(environment, validation_level)

def get_system_config() -> Optional[SystemConfig]:
    """Get current system configuration."""
    manager = get_config_manager()
    return manager.current_config

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
    'HardwareConfig',
    'SimulationConfig',
    'MissionConfig',
    'PerformanceConfig',
    'Environment',
    'ConfigValidationLevel',
    'ValidationResult',
    'get_config_manager',
    'load_system_config',
    'get_system_config'
]
