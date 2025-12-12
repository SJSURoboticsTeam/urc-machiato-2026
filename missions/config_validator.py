"""
Configuration Validation Module for URC 2026 Mission System.

Provides validation for mission configurations, system parameters,
and startup requirements.
"""

import os
import sys
import threading
from typing import Any, Dict, List

try:
    import rclpy
    RCLPY_AVAILABLE = True
except ImportError:
    RCLPY_AVAILABLE = False

# Add project paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))


class ValidationError(Exception):
    """Configuration validation error."""


class ConfigurationValidator:
    """Validates mission and system configurations."""

    def __init__(self):
        self.errors: List[str] = []
        self.warnings: List[str] = []

    def validate_startup_configuration(self) -> bool:
        """Validate system startup configuration."""
        self.errors = []
        self.warnings = []

        # Check environment variables
        required_env_vars = [
            'ROS_DOMAIN_ID',
            'ROS_VERSION'
        ]

        for var in required_env_vars:
            if not os.getenv(var):
                self.errors.append(f"Missing required environment variable: {var}")

        # Check ROS2 installation
        if not RCLPY_AVAILABLE:
            self.errors.append("ROS2 rclpy not available")
        else:
            try:
                # Basic ROS2 functionality check
                rclpy.init()
                rclpy.shutdown()
            except Exception as e:
                self.errors.append(f"ROS2 initialization failed: {e}")

        # Check required directories
        required_dirs = [
            'missions',
            'Autonomy/code',
            'config'
        ]

        for dir_path in required_dirs:
            if not os.path.exists(dir_path):
                self.warnings.append(f"Directory not found: {dir_path}")

        # Check configuration files
        config_files = [
            'config/mission_configs.yaml',
            'config/development.yaml'
        ]

        for config_file in config_files:
            if not os.path.exists(config_file):
                self.warnings.append(f"Configuration file not found: {config_file}")

        return len(self.errors) == 0

    def validate_mission_config(self, config: Dict[str, Any]) -> bool:
        """Validate mission configuration."""
        self.errors = []
        self.warnings = []

        # Check required fields
        required_fields = ['mission_type', 'waypoints']
        for field in required_fields:
            if field not in config:
                self.errors.append(f"Missing required field: {field}")

        # Validate mission type
        valid_mission_types = [
            'waypoint_navigation',
            'object_detection',
            'follow_me',
            'delivery',
            'emergency_response'
        ]

        mission_type = config.get('mission_type')
        if mission_type and mission_type not in valid_mission_types:
            self.errors.append(f"Invalid mission type: {mission_type}")

        # Validate waypoints
        waypoints = config.get('waypoints', [])
        if isinstance(waypoints, list):
            for i, waypoint in enumerate(waypoints):
                if not isinstance(waypoint, dict):
                    self.errors.append(f"Waypoint {i} must be a dictionary")
                    continue

                required_wp_fields = ['latitude', 'longitude']
                for field in required_wp_fields:
                    if field not in waypoint:
                        self.errors.append(f"Waypoint {i} missing field: {field}")

                # Validate coordinate ranges
                lat = waypoint.get('latitude', 0)
                lon = waypoint.get('longitude', 0)

                if not -90 <= lat <= 90:
                    self.errors.append(f"Waypoint {i} latitude out of range: {lat}")

                if not -180 <= lon <= 180:
                    self.errors.append(f"Waypoint {i} longitude out of range: {lon}")
        else:
            self.errors.append("Waypoints must be a list")

        # Validate timeout
        timeout = config.get('timeout', 300)
        if not isinstance(timeout, (int, float)) or timeout <= 0:
            self.errors.append("Timeout must be a positive number")

        return len(self.errors) == 0

    def get_errors(self) -> List[str]:
        """Get validation errors."""
        return self.errors.copy()

    def get_warnings(self) -> List[str]:
        """Get validation warnings."""
        return self.warnings.copy()

    def clear(self):
        """Clear validation results."""
        self.errors = []
        self.warnings = []


# Global validator instance with thread safety
_validator = ConfigurationValidator()
_validator_lock = threading.Lock()


def validate_startup_configuration():
    """Validate system startup configuration with thread safety."""
    with _validator_lock:
        if not _validator.validate_startup_configuration():
            errors = _validator.get_errors()
            warnings = _validator.get_warnings()

            print("❌ Configuration validation failed:")
            for error in errors:
                print(f"  • {error}")

            if warnings:
                print("\nWarnings:")
                for warning in warnings:
                    print(f"  • {warning}")

            # Exit with error if there are critical failures
            if errors:
                sys.exit(1)


def validate_mission_configuration(config: Dict[str, Any]) -> bool:
    """Validate mission configuration."""
    return _validator.validate_mission_config(config)
