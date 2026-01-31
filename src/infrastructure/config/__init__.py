#!/usr/bin/env python3
"""
URC 2026 Unified Configuration System (Simplified)

Single entry point for all configuration needs.
Simplified configuration only; stubs used if simplified_config is unavailable.

Usage:
    from src.infrastructure.config import get_urc_config
    config = get_urc_config()

Author: URC 2026 Configuration Team
"""

import logging
from typing import Any, Optional, Dict

logger = logging.getLogger(__name__)

# ============================================================================
# Settings Functions (simplified only; stubs if import fails)
# ============================================================================

try:
    from .simplified_config import (
        URCSettings,
        get_settings,
        reset_settings,
        config_get,
        config_set,
        get_environment,
        is_development,
        is_production,
        is_simulation,
    )
    logger.info("Using simplified configuration system")
except (ImportError, AttributeError) as e:
    logger.warning("Simplified config unavailable, using stubs: %s", e)

    class URCSettings:
        """Stub configuration settings."""
        def __init__(self):
            self.data = {}

    def get_settings():
        return URCSettings()

    def reset_settings():
        pass

    def config_get(key, default=None):
        return default

    def config_set(key, value):
        pass

    def get_environment():
        return "development"

    def is_development():
        return True

    def is_production():
        return False

    def is_simulation():
        return False


# ============================================================================
# Enums
# ============================================================================

class Environment:
    """Environment types."""
    DEVELOPMENT = "development"
    PRODUCTION = "production"
    SIMULATION = "simulation"


class LogLevel:
    """Logging levels."""
    DEBUG = "debug"
    INFO = "info"
    WARNING = "warning"
    ERROR = "error"


class RoverMode:
    """Rover operation modes."""
    AUTONOMOUS = "autonomous"
    TELEOPERATION = "teleoperation"
    SIMULATION = "simulation"


class SafetyLevel:
    """Safety levels."""
    HIGH = "high"
    MEDIUM = "medium"
    LOW = "low"


class SensorType:
    """Sensor types."""
    GPS = "gps"
    IMU = "imu"
    CAMERA = "camera"
    LIDAR = "lidar"


class MotorType:
    """Motor types."""
    BRUSHLESS = "brushless"
    BRUSHED = "brushed"


class CommunicationProtocol:
    """Communication protocols."""
    CANBUS = "canbus"
    WEBSOCKET = "websocket"


class MissionType:
    """Mission types."""
    AUTONOMOUS = "autonomous"
    TELEOPERATED = "teleoperated"


# ============================================================================
# Configuration Classes
# ============================================================================

class RoverConfig:
    """Main rover configuration."""
    
    def __init__(self):
        self.navigation = {}
        self.safety = {}
        self.sensors = {}
        self.motors = {}
        self.communication = {}
        self.simulation = {}
        self.sync = {}
        self.network = {}
        self.mission = {}
        self.hardware = {}
        self.database = {}
    
    @classmethod
    def from_settings(cls, settings: URCSettings) -> "RoverConfig":
        """Create config from settings."""
        config = cls()
        return config


class SystemConfig:
    """System configuration."""
    pass


# Sub-configs
class SimulationConfig:
    """Simulation configuration."""
    enabled: bool = False


class SyncConfig:
    """Synchronization configuration."""
    enabled: bool = True


class NetworkConfig:
    """Network configuration."""
    timeout: float = 5.0


class NavigationConfig:
    """Navigation configuration."""
    pass


class SafetyConfig:
    """Safety configuration."""
    pass


class MissionConfig:
    """Mission configuration."""
    pass


class HardwareConfig:
    """Hardware configuration."""
    pass


class DatabaseConfig:
    """Database configuration."""
    pass


class RedisConfig:
    """Redis configuration."""
    pass


class APIConfig:
    """API configuration."""
    pass


class PerformanceConfig:
    """Performance configuration."""
    pass


class WaypointConfig:
    """Waypoint configuration."""
    pass


class NetworkMonitorConfig:
    """Network monitoring configuration."""
    pass


class HealthMonitorConfig:
    """Health monitoring configuration."""
    pass


class ObservabilityConfig:
    """Observability configuration."""
    pass


class TelemetryConfig:
    """Telemetry configuration."""
    pass


class StateManagementConfig:
    """State management configuration."""
    pass


class CommunicationConfig:
    """Communication configuration."""
    pass


# ============================================================================
# Validation Functions (stubs for compatibility)
# ============================================================================

class ValidationError(Exception):
    """Configuration validation error."""
    pass


class URCConfigValidator:
    """Configuration validator."""
    pass


def validate_config(config: RoverConfig) -> bool:
    """Validate configuration."""
    return True


def get_validation_errors(config: RoverConfig) -> list:
    """Get validation errors."""
    return []


def get_validation_report(config: RoverConfig) -> dict:
    """Get validation report."""
    return {"valid": True, "errors": []}


# ============================================================================
# Module-level Configuration Cache
# ============================================================================

_rover_config: Optional[RoverConfig] = None


def get_urc_config(force_reload: bool = False) -> RoverConfig:
    """
    Get the unified URC configuration.
    
    This is the primary entry point for configuration access.
    
    Args:
        force_reload: Force reload from settings
        
    Returns:
        RoverConfig instance
    """
    global _rover_config
    
    if _rover_config is None or force_reload:
        try:
            settings = get_settings()
            _rover_config = RoverConfig.from_settings(settings)
        except Exception as e:
            logger.warning(f"Failed to load config: {e}, using default")
            _rover_config = RoverConfig()
    
    return _rover_config


def reload_config() -> RoverConfig:
    """Reload configuration from all sources."""
    global _rover_config
    reset_settings()
    _rover_config = None
    return get_urc_config(force_reload=True)


def get_config() -> RoverConfig:
    """Alias for get_urc_config() - backward compatibility."""
    return get_urc_config()


def get_system_config() -> RoverConfig:
    """Alias for get_urc_config() - backward compatibility."""
    return get_urc_config()


# ============================================================================
# Convenience Accessors
# ============================================================================

def get_sync_config() -> SyncConfig:
    """Get synchronization configuration."""
    return SyncConfig()


def get_network_config() -> NetworkConfig:
    """Get network configuration."""
    return NetworkConfig()


def get_safety_config() -> SafetyConfig:
    """Get safety configuration."""
    return SafetyConfig()


def get_mission_config() -> MissionConfig:
    """Get mission configuration."""
    return MissionConfig()


def get_navigation_config() -> NavigationConfig:
    """Get navigation configuration."""
    return NavigationConfig()


def get_simulation_config() -> SimulationConfig:
    """Get simulation configuration."""
    return SimulationConfig()


def get_hardware_config() -> HardwareConfig:
    """Get hardware configuration."""
    return HardwareConfig()


# ============================================================================
# Module Exports
# ============================================================================

__all__ = [
    # Main entry points
    "get_urc_config",
    "get_config",
    "get_system_config",
    "reload_config",
    
    # Settings access
    "URCSettings",
    "get_settings",
    "reset_settings",
    "config_get",
    "config_set",
    
    # Environment helpers
    "get_environment",
    "is_development",
    "is_production",
    "is_simulation",
    
    # Main config models
    "RoverConfig",
    "SystemConfig",
    
    # Enums
    "Environment",
    "LogLevel",
    "RoverMode",
    "SafetyLevel",
    "SensorType",
    "MotorType",
    "CommunicationProtocol",
    "MissionType",
    
    # Sub-config models
    "SimulationConfig",
    "SyncConfig",
    "NetworkConfig",
    "NavigationConfig",
    "SafetyConfig",
    "MissionConfig",
    "HardwareConfig",
    "DatabaseConfig",
    "RedisConfig",
    "APIConfig",
    "PerformanceConfig",
    "WaypointConfig",
    
    # Convenience accessors
    "get_sync_config",
    "get_network_config",
    "get_safety_config",
    "get_mission_config",
    "get_navigation_config",
    "get_simulation_config",
    "get_hardware_config",
    
    # Validation
    "URCConfigValidator",
    "ValidationError",
    "validate_config",
    "get_validation_errors",
    "get_validation_report",
]
