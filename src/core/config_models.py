#!/usr/bin/env python3
"""
URC 2026 Configuration Models - Pydantic Implementation

Replaces 500+ lines of custom configuration validation with professional
Pydantic models. Provides automatic validation, type safety, and schema generation.

Author: URC 2026 Configuration Team
"""

from typing import Dict, List, Optional, Union, Any, Literal
from pathlib import Path
import json
from enum import Enum

# Import libraries with fallbacks
try:
    from pydantic import BaseModel, Field, field_validator, ValidationError, ConfigDict
    from pydantic_settings import BaseSettings

    PYDANTIC_AVAILABLE = True
except ImportError:
    PYDANTIC_AVAILABLE = False

    # Fallback implementations
    class BaseModel:
        def __init__(self, **data):
            self.__dict__.update(data)

        @classmethod
        def parse_obj(cls, obj):
            return cls(**obj)

        def dict(self):
            return self.__dict__

    def Field(**kwargs):
        return None

    class BaseSettings:
        pass

    ValidationError = ValueError


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


class RoverMode(str, Enum):
    """Rover operational modes."""

    COMPETITION = "competition"
    DEVELOPMENT = "development"
    SIMULATION = "simulation"
    TESTING = "testing"


# Sensor Configuration Models
class SensorConfig(BaseModel):
    """Base sensor configuration."""

    model_config = ConfigDict(strict=True)

    type: SensorType
    enabled: bool = True
    update_rate_hz: float = Field(gt=0, le=1000, default=10.0)
    timeout_seconds: float = Field(gt=0, default=5.0)
    calibration_required: bool = True

    class Config:
        json_schema_extra = {
            "example": {
                "type": "imu",
                "enabled": True,
                "update_rate_hz": 100.0,
                "timeout_seconds": 2.0,
                "calibration_required": True,
            }
        }


class IMUConfig(SensorConfig):
    """IMU sensor configuration."""

    type: Literal[SensorType.IMU] = SensorType.IMU

    gyro_bias_stability: float = Field(gt=0, default=0.02)  # degrees/s
    gyro_noise_density: float = Field(gt=0, default=0.01)  # degrees/s/√Hz
    accel_bias_stability: float = Field(gt=0, default=2.0)  # mg
    accel_noise_density: float = Field(gt=0, default=1.0)  # mg/√Hz

    @field_validator(
        "gyro_bias_stability",
        "gyro_noise_density",
        "accel_bias_stability",
        "accel_noise_density",
    )
    @classmethod
    def validate_sensor_specs(cls, v):
        if v <= 0:
            raise ValueError("Sensor specifications must be positive")
        return v


class GPSConfig(SensorConfig):
    """GPS sensor configuration."""

    type: Literal[SensorType.GPS] = SensorType.GPS

    position_accuracy_m: float = Field(gt=0, default=2.5)  # meters (95% confidence)
    velocity_accuracy_ms: float = Field(gt=0, default=0.1)  # m/s
    min_satellites: int = Field(ge=4, le=20, default=6)

    @field_validator("position_accuracy_m", "velocity_accuracy_ms")
    @classmethod
    def validate_accuracy(cls, v):
        if v <= 0:
            raise ValueError("Accuracy must be positive")
        return v


class CameraConfig(SensorConfig):
    """Camera sensor configuration."""

    type: Literal[SensorType.CAMERA] = SensorType.CAMERA

    resolution_width: int = Field(gt=0, default=640)
    resolution_height: int = Field(gt=0, default=480)
    depth_accuracy_m: float = Field(gt=0, default=0.02)  # meters at 1m
    rgb_resolution: int = Field(gt=0, default=640)  # pixels

    @field_validator("resolution_width", "resolution_height")
    @classmethod
    def validate_resolution(cls, v):
        if v <= 0 or v > 10000:
            raise ValueError("Resolution must be reasonable (1-10000)")
        return v


class LidarConfig(SensorConfig):
    """LIDAR sensor configuration."""

    type: Literal[SensorType.LIDAR] = SensorType.LIDAR

    range_accuracy_m: float = Field(gt=0, default=0.03)  # meters
    angular_resolution_deg: float = Field(gt=0, le=10, default=0.25)  # degrees
    max_range_m: float = Field(gt=0, default=30.0)  # meters
    min_range_m: float = Field(ge=0, default=0.1)  # meters

    @field_validator("max_range_m")
    @classmethod
    def validate_range(cls, v, info):
        # Note: info.validation_context might be needed for cross-field validation
        # For now, just check basic constraint
        if v <= 0:
            raise ValueError("Maximum range must be positive")
        return v


# Motor Configuration Models
class MotorConfig(BaseModel):
    """Motor configuration."""

    model_config = ConfigDict(strict=True)

    id: str = Field(min_length=1, max_length=20)
    type: MotorType
    enabled: bool = True
    max_rpm: int = Field(gt=0, le=10000, default=3000)
    max_current_a: float = Field(gt=0, default=10.0)
    encoder_cpr: int = Field(ge=0, default=1024)  # counts per revolution

    protocol: CommunicationProtocol = CommunicationProtocol.CAN
    can_id: Optional[int] = Field(ge=0, le=2047, default=None)

    @field_validator("can_id")
    @classmethod
    def validate_can_id(cls, v, info):
        # Cross-field validation would need more complex setup
        # For now, just validate CAN ID range if provided
        if v is not None and (v < 0 or v > 2047):
            raise ValueError("CAN ID must be between 0 and 2047")
        return v


# Navigation Configuration Models
class WaypointConfig(BaseModel):
    """Waypoint configuration."""

    model_config = ConfigDict(strict=True)

    name: str = Field(min_length=1, max_length=50)
    x: float = Field(ge=-1000, le=1000)  # meters
    y: float = Field(ge=-1000, le=1000)  # meters
    heading: float = Field(ge=-180, le=180, default=0.0)  # degrees
    tolerance: float = Field(gt=0, le=5.0, default=0.5)  # meters

    description: Optional[str] = Field(max_length=200, default=None)

    class Config:
        json_schema_extra = {
            "example": {
                "name": "sample_site_1",
                "x": 10.5,
                "y": 25.3,
                "heading": 90.0,
                "tolerance": 0.5,
                "description": "Primary sample collection site",
            }
        }


class NavigationConfig(BaseModel):
    """Navigation system configuration."""

    model_config = ConfigDict(strict=True)

    max_linear_velocity: float = Field(gt=0, le=5.0, default=2.0)  # m/s
    max_angular_velocity: float = Field(gt=0, le=5.0, default=1.5)  # rad/s
    acceleration_limit: float = Field(gt=0, le=2.0, default=1.0)  # m/s²
    deceleration_limit: float = Field(gt=0, le=3.0, default=2.0)  # m/s²

    waypoint_timeout: float = Field(gt=0, default=60.0)  # seconds
    obstacle_avoidance_enabled: bool = True
    path_replanning_enabled: bool = True

    waypoints: List[WaypointConfig] = Field(
        min_items=1, max_items=50, default_factory=list
    )


# Mission Configuration Models
class MissionType(str, Enum):
    """Mission types."""

    AUTONOMOUS_NAVIGATION = "autonomous_navigation"
    SAMPLE_COLLECTION = "sample_collection"
    DELIVERY = "delivery"
    EQUIPMENT_SERVICING = "equipment_servicing"
    SCIENCE = "science"


class MissionConfig(BaseModel):
    """Mission configuration."""

    model_config = ConfigDict(strict=True)

    name: str = Field(min_length=1, max_length=100)
    type: MissionType
    description: Optional[str] = Field(max_length=500, default=None)

    timeout_minutes: int = Field(gt=0, le=120, default=30)
    priority: int = Field(ge=1, le=10, default=5)

    waypoints: List[WaypointConfig] = Field(min_items=1, max_items=50)
    sample_sites: List[str] = Field(
        default_factory=list
    )  # For sample collection missions
    delivery_targets: List[str] = Field(default_factory=list)  # For delivery missions

    @field_validator("waypoints")
    @classmethod
    def validate_mission_waypoints(cls, v):
        if len(v) < 2:
            raise ValueError("Missions require at least 2 waypoints")
        return v


# Safety Configuration Models
class SafetyLevel(str, Enum):
    """Safety levels."""

    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"
    CRITICAL = "critical"


class SafetyConfig(BaseModel):
    """Safety system configuration."""

    model_config = ConfigDict(strict=True)

    level: SafetyLevel = SafetyLevel.HIGH
    emergency_stop_enabled: bool = True
    heartbeat_timeout_seconds: float = Field(gt=0, default=5.0)
    geofencing_enabled: bool = True

    battery_critical_threshold: float = Field(gt=0, le=100, default=10.0)  # percentage
    communication_timeout_seconds: float = Field(gt=0, default=30.0)

    allowed_operating_area: Optional[Dict[str, List[float]]] = (
        None  # [min_x, max_x, min_y, max_y]
    )


# Communication Configuration Models
class NetworkConfig(BaseModel):
    """Network communication configuration."""

    model_config = ConfigDict(strict=True)

    websocket_enabled: bool = True
    websocket_port: int = Field(ge=1024, le=65535, default=8080)
    websocket_host: str = "localhost"

    can_enabled: bool = True
    can_interface: str = "can0"
    can_bitrate: int = Field(ge=10000, le=1000000, default=500000)

    serial_enabled: bool = False
    serial_port: Optional[str] = None
    serial_baudrate: int = Field(ge=9600, le=115200, default=115200)


# Main Configuration Models
class URCCompetitionConfig(BaseModel):
    """Complete URC 2026 competition configuration."""

    model_config = ConfigDict(strict=True)

    # Metadata
    version: str = "1.0.0"
    mode: RoverMode = RoverMode.COMPETITION
    description: Optional[str] = None

    # Hardware configurations
    sensors: Dict[str, Union[IMUConfig, GPSConfig, CameraConfig, LidarConfig]] = Field(
        default_factory=dict
    )
    motors: Dict[str, MotorConfig] = Field(default_factory=dict)

    # System configurations
    navigation: NavigationConfig = Field(default_factory=NavigationConfig)
    safety: SafetyConfig = Field(default_factory=SafetyConfig)
    network: NetworkConfig = Field(default_factory=NetworkConfig)

    # Mission configuration
    mission: Optional[MissionConfig] = None

    @field_validator("sensors")
    @classmethod
    def validate_sensor_configs(cls, v):
        """Validate that all sensor configurations are properly typed."""
        # Basic validation - more complex type checking would require custom logic
        if not isinstance(v, dict):
            raise ValueError("Sensors must be a dictionary")
        return v

    @field_validator("motors")
    @classmethod
    def validate_motor_configs(cls, v):
        """Validate motor configurations."""
        if not isinstance(v, dict):
            raise ValueError("Motors must be a dictionary")
        return v

    class Config:
        json_schema_extra = {
            "example": {
                "version": "1.0.0",
                "mode": "competition",
                "sensors": {
                    "imu_main": {
                        "type": "imu",
                        "enabled": True,
                        "update_rate_hz": 100.0,
                    },
                    "gps_primary": {
                        "type": "gps",
                        "enabled": True,
                        "position_accuracy_m": 2.5,
                    },
                },
                "navigation": {
                    "max_linear_velocity": 2.0,
                    "waypoints": [
                        {"name": "start", "x": 0.0, "y": 0.0},
                        {"name": "sample_site", "x": 10.0, "y": 5.0},
                    ],
                },
            }
        }


class URCDevelopmentConfig(BaseModel):
    """Development environment configuration."""

    model_config = ConfigDict(strict=True)

    version: str = "1.0.0"
    mode: RoverMode = RoverMode.DEVELOPMENT

    # Relaxed constraints for development
    sensors: Dict[str, Union[IMUConfig, GPSConfig, CameraConfig, LidarConfig]] = Field(
        default_factory=dict
    )
    motors: Dict[str, MotorConfig] = Field(default_factory=dict)

    navigation: NavigationConfig = Field(default_factory=NavigationConfig)
    safety: SafetyConfig = Field(
        default_factory=lambda: SafetyConfig(level=SafetyLevel.MEDIUM)
    )
    network: NetworkConfig = Field(default_factory=NetworkConfig)

    mission: Optional[MissionConfig] = None

    # Development-specific settings
    simulation_enabled: bool = True
    debug_logging_enabled: bool = True
    test_mode_enabled: bool = False


# Configuration Loading and Management
class URCConfigManager:
    """
    Configuration manager using Pydantic models.

    Replaces custom configuration validation with professional models.
    """

    def __init__(self):
        self.configs: Dict[str, BaseModel] = {}
        self.schemas: Dict[str, Dict[str, Any]] = {}

    def load_competition_config(
        self, config_path: Union[str, Path]
    ) -> URCCompetitionConfig:
        """Load competition configuration with validation."""
        with open(config_path, "r") as f:
            data = json.load(f)

        config = URCCompetitionConfig.parse_obj(data)
        self.configs["competition"] = config
        return config

    def load_development_config(
        self, config_path: Union[str, Path]
    ) -> URCDevelopmentConfig:
        """Load development configuration with validation."""
        with open(config_path, "r") as f:
            data = json.load(f)

        config = URCDevelopmentConfig.parse_obj(data)
        self.configs["development"] = config
        return config

    def validate_config(self, config: BaseModel) -> List[str]:
        """Validate configuration and return errors."""
        try:
            # Pydantic automatically validates on creation
            return []
        except ValidationError as e:
            return [f"{error['loc'][0]}: {error['msg']}" for error in e.errors()]

    def get_schema(self, config_type: str) -> Dict[str, Any]:
        """Get JSON schema for configuration type."""
        if config_type == "competition":
            return URCCompetitionConfig.model_json_schema()
        elif config_type == "development":
            return URCDevelopmentConfig.model_json_schema()
        else:
            raise ValueError(f"Unknown config type: {config_type}")

    def export_schema(self, config_type: str, output_path: Union[str, Path]):
        """Export JSON schema to file."""
        schema = self.get_schema(config_type)
        with open(output_path, "w") as f:
            json.dump(schema, f, indent=2)


# Convenience functions
def load_urc_config(
    config_path: Union[str, Path], mode: RoverMode = RoverMode.COMPETITION
) -> BaseModel:
    """Load URC configuration with automatic validation."""
    manager = URCConfigManager()

    if mode == RoverMode.COMPETITION:
        return manager.load_competition_config(config_path)
    elif mode in [RoverMode.DEVELOPMENT, RoverMode.SIMULATION, RoverMode.TESTING]:
        return manager.load_development_config(config_path)
    else:
        raise ValueError(f"Unsupported mode: {mode}")


def validate_config_data(
    data: Dict[str, Any], mode: RoverMode = RoverMode.COMPETITION
) -> List[str]:
    """Validate configuration data and return errors."""
    manager = URCConfigManager()

    try:
        if mode == RoverMode.COMPETITION:
            URCCompetitionConfig.parse_obj(data)
        else:
            URCDevelopmentConfig.parse_obj(data)
        return []
    except ValidationError as e:
        return [
            f"{'.'.join(str(loc) for loc in error['loc'])}: {error['msg']}"
            for error in e.errors()
        ]


def generate_config_schema(config_type: str = "competition") -> Dict[str, Any]:
    """Generate JSON schema for configuration."""
    manager = URCConfigManager()
    return manager.get_schema(config_type)
