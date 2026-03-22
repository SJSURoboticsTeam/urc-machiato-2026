#!/usr/bin/env python3
"""
URC 2026 Unified Configuration - Pydantic Schemas

Type-safe configuration models with automatic validation.
Provides schema generation for documentation and IDE support.

Author: URC 2026 Configuration Team
"""

from enum import Enum
from pathlib import Path
from typing import Any, Dict, List, Optional, Union

try:
    from pydantic import BaseModel, Field, field_validator, ConfigDict
    from pydantic_settings import BaseSettings, SettingsConfigDict

    PYDANTIC_AVAILABLE = True
except ImportError:
    PYDANTIC_AVAILABLE = False

    # Minimal fallback
    class BaseModel:
        def __init__(self, **data):
            for k, v in data.items():
                setattr(self, k, v)

        def model_dump(self):
            return self.__dict__

        dict = model_dump  # Alias for compatibility

    class BaseSettings(BaseModel):
        pass

    def Field(**kwargs):
        return kwargs.get("default")

    ConfigDict = dict
    SettingsConfigDict = dict


# ============================================================================
# Enumerations
# ============================================================================


class Environment(str, Enum):
    """Deployment environments."""

    DEVELOPMENT = "development"
    TESTING = "testing"
    PRODUCTION = "production"
    COMPETITION = "competition"
    SIMULATION = "simulation"


class LogLevel(str, Enum):
    """Logging levels."""

    DEBUG = "DEBUG"
    INFO = "INFO"
    WARNING = "WARNING"
    ERROR = "ERROR"
    CRITICAL = "CRITICAL"


class RoverMode(str, Enum):
    """Rover operational modes."""

    COMPETITION = "competition"
    DEVELOPMENT = "development"
    SIMULATION = "simulation"
    TESTING = "testing"


class SafetyLevel(str, Enum):
    """Safety levels."""

    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class SensorType(str, Enum):
    """Supported sensor types."""

    IMU = "imu"
    GPS = "gps"
    CAMERA = "camera"
    LIDAR = "lidar"
    ENCODER = "encoder"


class MotorType(str, Enum):
    """Supported motor types."""

    DC_BRUSHED = "dc_brushed"
    DC_BRUSHLESS = "dc_brushless"
    STEPPER = "stepper"
    SERVO = "servo"


class CommunicationProtocol(str, Enum):
    """Supported communication protocols."""

    CAN = "can"
    SERIAL = "serial"
    I2C = "i2c"
    SPI = "spi"
    ETHERNET = "ethernet"


class MissionType(str, Enum):
    """Mission types."""

    AUTONOMOUS_NAVIGATION = "autonomous_navigation"
    SAMPLE_COLLECTION = "sample_collection"
    DELIVERY = "delivery"
    EQUIPMENT_SERVICING = "equipment_servicing"
    SCIENCE = "science"


# ============================================================================
# Configuration Schemas
# ============================================================================


class SimulationConfig(BaseModel):
    """Simulation-specific configuration."""

    if PYDANTIC_AVAILABLE:
        model_config = ConfigDict(extra="ignore")

    enabled: bool = True
    gazebo_world: str = "urc_mars_world.world"
    physics_engine: str = "ode"
    real_time_factor: float = Field(default=1.0, ge=0.1, le=10.0)
    sensor_noise: bool = True
    update_rate_hz: float = Field(default=10.0, ge=1.0, le=100.0)
    odom_noise: float = Field(default=0.01, ge=0.0, le=1.0)


class SyncConfig(BaseModel):
    """Synchronization engine configuration."""

    if PYDANTIC_AVAILABLE:
        model_config = ConfigDict(extra="ignore")

    max_sync_delay_ms: float = Field(default=50.0, gt=0, le=1000)
    temporal_consistency_threshold: float = Field(default=0.05, gt=0, le=1.0)
    buffer_max_size: int = Field(default=30, gt=0, le=1000)
    adaptive_buffering: bool = True
    camera_ids: List[str] = Field(
        default_factory=lambda: ["front", "rear", "left", "right"]
    )


class NetworkConfig(BaseModel):
    """Network communication configuration."""

    if PYDANTIC_AVAILABLE:
        model_config = ConfigDict(extra="ignore")

    retry_attempts: int = Field(default=3, ge=0, le=10)
    retry_backoff_factor: float = Field(default=0.5, gt=0)
    connection_timeout: float = Field(default=5.0, gt=0)
    circuit_breaker_threshold: int = Field(default=5, ge=1)
    circuit_breaker_timeout: float = Field(default=30.0, gt=0)
    websocket_port: int = Field(default=8765, ge=1024, le=65535)
    websocket_host: str = "localhost"
    http_port: int = Field(default=8080, ge=1024, le=65535)
    enable_cors: bool = True
    max_connections: int = Field(default=100, ge=1)

    # CAN bus
    can_enabled: bool = True
    can_interface: str = "can0"
    can_bitrate: int = Field(default=500000, ge=10000, le=1000000)

    # Serial
    serial_enabled: bool = False
    serial_port: Optional[str] = None
    serial_baudrate: int = Field(default=115200, ge=9600, le=115200)


class NavigationConfig(BaseModel):
    """Navigation system configuration."""

    if PYDANTIC_AVAILABLE:
        model_config = ConfigDict(extra="ignore")

    update_rate_hz: float = Field(default=10.0, ge=1.0, le=100.0)
    waypoint_tolerance_m: float = Field(default=1.0, ge=0.1)
    max_linear_velocity_ms: float = Field(default=2.0, ge=0.1, le=5.0)
    max_angular_velocity_rads: float = Field(default=1.0, ge=0.1, le=5.0)
    acceleration_limit: float = Field(default=1.0, gt=0, le=2.0)
    deceleration_limit: float = Field(default=2.0, gt=0, le=3.0)
    obstacle_avoidance_enabled: bool = True
    path_replanning_enabled: bool = True


class SafetyConfig(BaseModel):
    """Safety system configuration."""

    if PYDANTIC_AVAILABLE:
        model_config = ConfigDict(extra="ignore")

    level: SafetyLevel = SafetyLevel.HIGH
    emergency_stop_timeout: float = Field(default=1.0, gt=0)
    emergency_stop_enabled: bool = True
    thermal_warning_threshold: float = Field(default=70.0, gt=0, le=150)
    battery_critical_threshold: float = Field(default=20.0, gt=0, le=100)
    motor_overtemp_threshold: float = Field(default=80.0, gt=0, le=150)
    safety_layer_count: int = Field(default=3, ge=1, le=5)
    max_speed_mps: float = Field(default=2.0, ge=0.0, le=5.0)
    distance_meters: float = Field(default=0.5, gt=0.0, le=2.0)
    heartbeat_timeout_seconds: float = Field(default=5.0, gt=0)
    geofencing_enabled: bool = True
    communication_timeout_seconds: float = Field(default=30.0, gt=0)


class MissionConfig(BaseModel):
    """Mission execution configuration."""

    if PYDANTIC_AVAILABLE:
        model_config = ConfigDict(extra="ignore")

    max_mission_duration: float = Field(default=1800.0, gt=0)  # 30 minutes
    execution_rate_hz: float = Field(default=10.0, ge=1, le=50)
    waypoint_tolerance: float = Field(default=1.0, gt=0)
    sample_collection_timeout: float = Field(default=300.0, gt=0)  # 5 minutes
    return_home_on_failure: bool = True
    autonomous_mode: bool = False


class HardwareConfig(BaseModel):
    """Hardware interface configuration."""

    if PYDANTIC_AVAILABLE:
        model_config = ConfigDict(extra="ignore")

    use_mock: bool = True
    camera_front_url: Optional[str] = None
    camera_rear_url: Optional[str] = None


class DatabaseConfig(BaseModel):
    """Database configuration."""

    if PYDANTIC_AVAILABLE:
        model_config = ConfigDict(extra="ignore")

    url: str = "sqlite:///urc_missions.db"
    connection_pool_size: int = Field(default=5, gt=0)
    connection_timeout: float = Field(default=30.0, gt=0)
    enable_migrations: bool = True


class RedisConfig(BaseModel):
    """Redis cache configuration."""

    if PYDANTIC_AVAILABLE:
        model_config = ConfigDict(extra="ignore")

    host: str = "localhost"
    port: int = Field(default=6379, gt=0, le=65535)
    db: int = Field(default=0, ge=0)
    password: Optional[str] = None
    ttl_seconds: int = Field(default=3600, gt=0)


class APIConfig(BaseModel):
    """REST API configuration."""

    if PYDANTIC_AVAILABLE:
        model_config = ConfigDict(extra="ignore")

    host: str = "0.0.0.0"
    port: int = Field(default=8000, gt=0, le=65535)
    cors_origins: List[str] = Field(default_factory=lambda: ["http://localhost:3000"])
    enable_docs: bool = True
    api_key_required: bool = False


class PerformanceConfig(BaseModel):
    """Performance configuration."""

    if PYDANTIC_AVAILABLE:
        model_config = ConfigDict(extra="ignore")

    enable_optimizations: bool = True
    memory_limit_mb: int = Field(default=512, ge=64)
    cpu_limit_percent: int = Field(default=80, ge=10, le=100)
    thread_pool_size: int = Field(default=4, ge=1, le=16)


class WaypointConfig(BaseModel):
    """Waypoint configuration."""

    if PYDANTIC_AVAILABLE:
        model_config = ConfigDict(extra="ignore")

    name: str = Field(min_length=1, max_length=50)
    x: float = Field(ge=-1000, le=1000)
    y: float = Field(ge=-1000, le=1000)
    heading: float = Field(default=0.0, ge=-180, le=180)
    tolerance: float = Field(default=0.5, gt=0, le=5.0)
    description: Optional[str] = Field(default=None, max_length=200)


# ============================================================================
# Main Configuration Model
# ============================================================================


class RoverConfig(BaseModel):
    """
    Main rover configuration with all subsystems.

    This is the unified configuration model that combines all
    configuration sources into a single validated structure.
    """

    if PYDANTIC_AVAILABLE:
        model_config = ConfigDict(extra="ignore")

    # Environment settings
    environment: Environment = Environment.DEVELOPMENT
    debug: bool = False

    # Logging
    log_level: LogLevel = LogLevel.INFO
    log_format: str = "json"
    log_file: Optional[str] = None

    # Dashboard / UI
    title: str = "URC 2026 Rover"
    enable_real_time: bool = True
    refresh_interval: float = Field(default=1.0, gt=0)

    # Core systems
    simulation: SimulationConfig = Field(default_factory=SimulationConfig)
    sync: SyncConfig = Field(default_factory=SyncConfig)
    network: NetworkConfig = Field(default_factory=NetworkConfig)
    navigation: NavigationConfig = Field(default_factory=NavigationConfig)
    safety: SafetyConfig = Field(default_factory=SafetyConfig)
    mission: MissionConfig = Field(default_factory=MissionConfig)
    hardware: HardwareConfig = Field(default_factory=HardwareConfig)

    # Optional advanced features
    database: Optional[DatabaseConfig] = None
    redis: Optional[RedisConfig] = None
    api: Optional[APIConfig] = None
    performance: PerformanceConfig = Field(default_factory=PerformanceConfig)

    # Waypoints
    waypoints: List[WaypointConfig] = Field(default_factory=list)

    # Metadata
    version: str = "2.0.0"
    config_file_path: Optional[str] = None

    @classmethod
    def from_settings(cls, settings) -> "RoverConfig":
        """Create RoverConfig from Dynaconf settings."""
        if hasattr(settings, "to_dict"):
            data = settings.to_dict()
        elif isinstance(settings, dict):
            data = settings
        else:
            data = dict(settings)

        # Map settings to config structure
        env_value = data.get("environment", "development")
        if isinstance(env_value, str):
            try:
                env_value = Environment(env_value)
            except ValueError:
                env_value = Environment.DEVELOPMENT

        log_value = data.get("log_level", "INFO")
        if isinstance(log_value, str):
            try:
                log_value = LogLevel(log_value)
            except ValueError:
                log_value = LogLevel.INFO

        config_data = {
            "environment": env_value,
            "debug": data.get("debug", False),
            "log_level": log_value,
        }

        # Map nested configurations - explicitly create sub-config instances
        if "simulation" in data and isinstance(data["simulation"], dict):
            config_data["simulation"] = SimulationConfig(**data["simulation"])
        if "sync" in data and isinstance(data["sync"], dict):
            config_data["sync"] = SyncConfig(**data["sync"])
        if "network" in data and isinstance(data["network"], dict):
            config_data["network"] = NetworkConfig(**data["network"])
        if "navigation" in data and isinstance(data["navigation"], dict):
            config_data["navigation"] = NavigationConfig(**data["navigation"])
        if "safety" in data and isinstance(data["safety"], dict):
            safety_data = data["safety"].copy()
            # Convert level string to enum if needed
            if "level" in safety_data and isinstance(safety_data["level"], str):
                try:
                    safety_data["level"] = SafetyLevel(safety_data["level"])
                except ValueError:
                    safety_data["level"] = SafetyLevel.HIGH
            config_data["safety"] = SafetyConfig(**safety_data)
        if "mission" in data and isinstance(data["mission"], dict):
            config_data["mission"] = MissionConfig(**data["mission"])
        if "hardware" in data and isinstance(data["hardware"], dict):
            config_data["hardware"] = HardwareConfig(**data["hardware"])
        if "waypoints" in data and isinstance(data["waypoints"], list):
            config_data["waypoints"] = [
                WaypointConfig(**wp) if isinstance(wp, dict) else wp
                for wp in data["waypoints"]
            ]

        return cls(**config_data)

    def is_production(self) -> bool:
        """Check if running in production environment."""
        return self.environment in [Environment.PRODUCTION, Environment.COMPETITION]

    def is_development(self) -> bool:
        """Check if running in development environment."""
        return self.environment == Environment.DEVELOPMENT

    def is_simulation(self) -> bool:
        """Check if simulation is enabled."""
        return self.simulation.enabled

    def get_nested_value(self, key_path: str, default: Any = None) -> Any:
        """Get nested configuration value using dot notation."""
        keys = key_path.split(".")
        value = self.model_dump() if hasattr(self, "model_dump") else self.dict()

        for key in keys:
            if isinstance(value, dict):
                value = value.get(key, default)
            else:
                return default

        return value


# Alias for backward compatibility
SystemConfig = RoverConfig
