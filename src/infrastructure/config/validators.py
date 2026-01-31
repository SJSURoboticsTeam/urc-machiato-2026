#!/usr/bin/env python3
"""
URC 2026 Unified Configuration - URC-Specific Validators

Custom validation rules for URC competition requirements.
Ensures configuration consistency and safety compliance.

Author: URC 2026 Configuration Team
"""

import logging
from typing import Any, Dict, List, Optional

from .schemas import RoverConfig, SafetyLevel

logger = logging.getLogger(__name__)


class ValidationError(Exception):
    """Configuration validation error."""
    
    def __init__(self, message: str, field: Optional[str] = None):
        self.message = message
        self.field = field
        super().__init__(message)


class URCConfigValidator:
    """
    URC-specific configuration validator.
    
    Validates configuration for:
    - Safety compliance
    - Competition requirements
    - Hardware consistency
    - Performance constraints
    """
    
    def __init__(self, config: RoverConfig):
        self.config = config
        self.errors: List[str] = []
        self.warnings: List[str] = []
    
    def validate(self) -> bool:
        """
        Run all validations.
        
        Returns:
            True if configuration is valid, False otherwise.
        """
        self.errors = []
        self.warnings = []
        
        # Run all validation checks
        self._validate_safety_constraints()
        self._validate_navigation_limits()
        self._validate_hardware_consistency()
        self._validate_network_configuration()
        self._validate_mission_parameters()
        self._validate_simulation_settings()
        self._validate_waypoints()
        
        # Log results
        if self.errors:
            logger.error(f"Configuration validation failed with {len(self.errors)} errors")
            for error in self.errors:
                logger.error(f"  - {error}")
        
        if self.warnings:
            logger.warning(f"Configuration has {len(self.warnings)} warnings")
            for warning in self.warnings:
                logger.warning(f"  - {warning}")
        
        return len(self.errors) == 0
    
    def _validate_safety_constraints(self):
        """Validate safety-related constraints."""
        safety = self.config.safety
        navigation = self.config.navigation
        
        # Safety speed must not exceed navigation max speed
        if safety.max_speed_mps > navigation.max_linear_velocity_ms:
            self.errors.append(
                f"Safety max speed ({safety.max_speed_mps} m/s) exceeds "
                f"navigation max velocity ({navigation.max_linear_velocity_ms} m/s)"
            )
        
        # Emergency stop timeout must be reasonable
        if safety.emergency_stop_timeout > 5.0:
            self.warnings.append(
                f"Emergency stop timeout ({safety.emergency_stop_timeout}s) "
                "is longer than recommended (5s)"
            )
        
        # Critical battery threshold validation
        if safety.battery_critical_threshold < 10.0:
            self.warnings.append(
                f"Battery critical threshold ({safety.battery_critical_threshold}%) "
                "is dangerously low"
            )
        
        # Competition mode requires HIGH safety level
        env_value = self.config.environment.value if hasattr(self.config.environment, 'value') else self.config.environment
        if env_value == "competition":
            if safety.level != SafetyLevel.HIGH:
                self.errors.append(
                    f"Competition mode requires HIGH safety level, "
                    f"got {safety.level.value}"
                )
            
            if not safety.emergency_stop_enabled:
                self.errors.append(
                    "Emergency stop must be enabled in competition mode"
                )
    
    def _validate_navigation_limits(self):
        """Validate navigation parameter limits."""
        nav = self.config.navigation
        
        # Deceleration should be greater than or equal to acceleration
        if nav.deceleration_limit < nav.acceleration_limit:
            self.warnings.append(
                f"Deceleration limit ({nav.deceleration_limit} m/s^2) is less than "
                f"acceleration limit ({nav.acceleration_limit} m/s^2)"
            )
        
        # Angular velocity should be reasonable for the linear velocity
        max_safe_angular = nav.max_linear_velocity_ms * 2.0
        if nav.max_angular_velocity_rads > max_safe_angular:
            self.warnings.append(
                f"Angular velocity ({nav.max_angular_velocity_rads} rad/s) may be "
                f"too high for linear velocity ({nav.max_linear_velocity_ms} m/s)"
            )
    
    def _validate_hardware_consistency(self):
        """Validate hardware configuration consistency."""
        hardware = self.config.hardware
        
        # If not using mock hardware, camera URLs should be configured
        if not hardware.use_mock:
            if not hardware.camera_front_url:
                self.errors.append(
                    "camera_front_url required when not using mock hardware"
                )
            if not hardware.camera_rear_url:
                self.warnings.append(
                    "camera_rear_url not configured for real hardware"
                )
        
        # Simulation should use mock hardware
        if self.config.simulation.enabled and not hardware.use_mock:
            self.warnings.append(
                "Simulation enabled but mock hardware disabled - "
                "consider enabling mock hardware"
            )
    
    def _validate_network_configuration(self):
        """Validate network configuration."""
        network = self.config.network
        
        # Port conflicts
        if network.websocket_port == network.http_port:
            self.errors.append(
                f"WebSocket port ({network.websocket_port}) conflicts with "
                f"HTTP port ({network.http_port})"
            )
        
        # Reasonable retry configuration
        if network.retry_attempts > 5:
            self.warnings.append(
                f"High retry attempts ({network.retry_attempts}) may cause delays"
            )
        
        # Circuit breaker configuration
        if network.circuit_breaker_threshold < 3:
            self.warnings.append(
                f"Circuit breaker threshold ({network.circuit_breaker_threshold}) "
                "is very sensitive"
            )
    
    def _validate_mission_parameters(self):
        """Validate mission configuration."""
        mission = self.config.mission
        
        # Mission duration should be reasonable
        if mission.max_mission_duration < 300:  # 5 minutes
            self.warnings.append(
                f"Mission duration ({mission.max_mission_duration}s) "
                "is very short"
            )
        
        if mission.max_mission_duration > 7200:  # 2 hours
            self.warnings.append(
                f"Mission duration ({mission.max_mission_duration}s) "
                "is very long"
            )
        
        # Execution rate should be reasonable
        if mission.execution_rate_hz < 5:
            self.warnings.append(
                f"Mission execution rate ({mission.execution_rate_hz} Hz) "
                "may be too slow for responsive control"
            )
    
    def _validate_simulation_settings(self):
        """Validate simulation configuration."""
        sim = self.config.simulation
        
        if sim.enabled:
            # Real-time factor should be reasonable
            if sim.real_time_factor > 2.0:
                self.warnings.append(
                    f"Simulation real-time factor ({sim.real_time_factor}) "
                    "is high - may cause instability"
                )
            
            if sim.real_time_factor < 0.5:
                self.warnings.append(
                    f"Simulation real-time factor ({sim.real_time_factor}) "
                    "is low - simulation will run slowly"
                )
            
            # Sensor noise should be enabled for realistic simulation
            if not sim.sensor_noise:
                self.warnings.append(
                    "Sensor noise disabled - simulation may be unrealistic"
                )
    
    def _validate_waypoints(self):
        """Validate waypoint configuration."""
        waypoints = self.config.waypoints
        
        if not waypoints:
            return  # No waypoints to validate
        
        # Check for duplicate waypoint names
        names = [wp.name for wp in waypoints]
        duplicates = set([n for n in names if names.count(n) > 1])
        if duplicates:
            self.errors.append(
                f"Duplicate waypoint names: {duplicates}"
            )
        
        # Check waypoint structure
        for i, wp in enumerate(waypoints):
            # Waypoint tolerance should be reasonable
            if wp.tolerance < 0.1:
                self.warnings.append(
                    f"Waypoint '{wp.name}' tolerance ({wp.tolerance}m) "
                    "is very tight"
                )
            
            # Check for unreasonable positions
            if abs(wp.x) > 500 or abs(wp.y) > 500:
                self.warnings.append(
                    f"Waypoint '{wp.name}' position ({wp.x}, {wp.y}) "
                    "is far from origin"
                )
    
    def get_errors(self) -> List[str]:
        """Get list of validation errors."""
        return self.errors.copy()
    
    def get_warnings(self) -> List[str]:
        """Get list of validation warnings."""
        return self.warnings.copy()
    
    def get_report(self) -> Dict[str, Any]:
        """Get validation report as dictionary."""
        return {
            "valid": len(self.errors) == 0,
            "errors": self.errors.copy(),
            "warnings": self.warnings.copy(),
            "error_count": len(self.errors),
            "warning_count": len(self.warnings),
        }


def validate_config(config: RoverConfig) -> bool:
    """
    Validate URC configuration.
    
    Args:
        config: RoverConfig instance to validate
        
    Returns:
        True if valid, False otherwise
    """
    validator = URCConfigValidator(config)
    return validator.validate()


def get_validation_errors(config: RoverConfig) -> List[str]:
    """
    Get validation errors for configuration.
    
    Args:
        config: RoverConfig instance to validate
        
    Returns:
        List of error messages (empty if valid)
    """
    validator = URCConfigValidator(config)
    validator.validate()
    return validator.get_errors()


def get_validation_report(config: RoverConfig) -> Dict[str, Any]:
    """
    Get full validation report for configuration.
    
    Args:
        config: RoverConfig instance to validate
        
    Returns:
        Dictionary with validation results
    """
    validator = URCConfigValidator(config)
    validator.validate()
    return validator.get_report()
