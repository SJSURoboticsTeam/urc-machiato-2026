#!/usr/bin/env python3
"""
Hardware Interface Factory - Unified Factory for Mock/Hardware Component Switching

Provides factory methods for creating hardware interfaces that can seamlessly switch
between mock implementations (for development/testing) and real hardware drivers.

This enables gradual hardware integration and reliable testing throughout development.

Author: URC 2026 Autonomy Team
"""

import logging
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Type, Union

logger = logging.getLogger(__name__)


class HardwareInterface(ABC):
    """Base class for all hardware interfaces."""

    @abstractmethod
    def initialize(self) -> bool:
        """Initialize the hardware interface."""
        pass

    @abstractmethod
    def shutdown(self) -> bool:
        """Shutdown the hardware interface."""
        pass

    @abstractmethod
    def is_mock(self) -> bool:
        """Return True if this is a mock implementation."""
        pass

    @abstractmethod
    def get_status(self) -> Dict[str, Any]:
        """Get current status of the hardware interface."""
        pass


class HardwareInterfaceFactory:
    """
    Factory for creating hardware interfaces with mock/hardware switching.

    Supports runtime configuration of which components should use mock vs hardware
    implementations, enabling gradual hardware integration testing.
    """

    # Registry of available implementations
    _implementations = {
        "can_bus": {
            "mock": "simulation.can.can_bus_mock_simulator.CANBusMockSimulator",
            "hardware": None,  # To be filled when hardware drivers are implemented
        },
        "motor_controller": {
            "mock": "simulation.can.can_bus_mock_simulator.MotorControllerMock",
            "hardware": None,
        },
        "drive_system": {
            "mock": "simulation.drive.drive_system_simulator.DriveSystemSimulator",
            "hardware": None,
        },
        "robotic_arm": {
            "mock": "simulation.manipulator.robotic_arm_simulator.RoboticArmSimulator",
            "hardware": None,
        },
        "science_payload": {
            "mock": "simulation.science.science_payload_simulator.SciencePayloadSimulator",
            "hardware": None,
        },
        "power_system": {
            "mock": "simulation.power.power_system_simulator.PowerSystemSimulator",
            "hardware": None,
        },
        "sensor_imu": {
            "mock": "simulation.can.can_bus_mock_simulator.SensorInterfaceMock",
            "hardware": None,
        },
        "sensor_gps": {
            "mock": "simulation.can.can_bus_mock_simulator.SensorInterfaceMock",
            "hardware": None,
        },
    }

    # Current configuration (can be changed at runtime)
    _current_config = {
        "can_bus": "mock",
        "motor_controller": "mock",
        "drive_system": "mock",
        "robotic_arm": "mock",
        "science_payload": "mock",
        "power_system": "mock",
        "sensor_imu": "mock",
        "sensor_gps": "mock",
    }

    # Cache of created instances
    _instance_cache: Dict[str, Any] = {}

    @classmethod
    def set_configuration(cls, config: Dict[str, str]) -> bool:
        """
        Set the configuration for which components should use mock vs hardware.

        Args:
            config: Dict mapping component names to 'mock' or 'hardware'

        Returns:
            bool: True if configuration is valid and applied
        """
        # Validate configuration
        for component, mode in config.items():
            if component not in cls._implementations:
                logger.error(f"Unknown component: {component}")
                return False
            if mode not in ["mock", "hardware"]:
                logger.error(
                    f"Invalid mode for {component}: {mode} (must be 'mock' or 'hardware')"
                )
                return False
            if (
                mode == "hardware"
                and cls._implementations[component]["hardware"] is None
            ):
                logger.error(f"Hardware implementation not available for {component}")
                return False

        cls._current_config.update(config)
        logger.info(f"Hardware configuration updated: {config}")

        # Clear cache to force recreation with new config
        cls._instance_cache.clear()

        return True

    @classmethod
    def load_configuration_from_file(cls, config_file: str) -> bool:
        """
        Load configuration from YAML file.

        Args:
            config_file: Path to YAML configuration file

        Returns:
            bool: True if configuration loaded successfully
        """
        try:
            import yaml

            with open(config_file, "r") as f:
                config = yaml.safe_load(f)

            # Extract hardware configuration
            if "hardware" in config:
                hardware_config = config["hardware"]
                return cls.set_configuration(hardware_config)
            else:
                logger.error(f"No 'hardware' section found in {config_file}")
                return False

        except Exception as e:
            logger.error(f"Failed to load configuration from {config_file}: {e}")
            return False

    @classmethod
    def create_can_bus(cls, node_id: Optional[int] = None, **kwargs) -> Any:
        """Create CAN bus interface."""
        return cls._create_interface("can_bus", node_id=node_id, **kwargs)

    @classmethod
    def create_motor_controller(cls, node_id: int, motor_name: str, **kwargs) -> Any:
        """Create motor controller interface."""
        return cls._create_interface(
            "motor_controller", node_id=node_id, motor_name=motor_name, **kwargs
        )

    @classmethod
    def create_drive_system(cls, config: Optional[Dict] = None, **kwargs) -> Any:
        """Create drive system interface."""
        return cls._create_interface("drive_system", config=config, **kwargs)

    @classmethod
    def create_robotic_arm(cls, config: Optional[Dict] = None, **kwargs) -> Any:
        """Create robotic arm interface."""
        return cls._create_interface("robotic_arm", config=config, **kwargs)

    @classmethod
    def create_science_payload(cls, config: Optional[Dict] = None, **kwargs) -> Any:
        """Create science payload interface."""
        return cls._create_interface("science_payload", config=config, **kwargs)

    @classmethod
    def create_power_system(cls, config: Optional[Dict] = None, **kwargs) -> Any:
        """Create power system interface."""
        return cls._create_interface("power_system", config=config, **kwargs)

    @classmethod
    def create_sensor_imu(cls, node_id: int, **kwargs) -> Any:
        """Create IMU sensor interface."""
        return cls._create_interface(
            "sensor_imu", node_id=node_id, sensor_type="imu", **kwargs
        )

    @classmethod
    def create_sensor_gps(cls, node_id: int, **kwargs) -> Any:
        """Create GPS sensor interface."""
        return cls._create_interface(
            "sensor_gps", node_id=node_id, sensor_type="gps", **kwargs
        )

    @classmethod
    def _create_interface(cls, component_name: str, **kwargs) -> Any:
        """
        Internal method to create interface instances.

        Args:
            component_name: Name of the component to create
            **kwargs: Arguments to pass to the constructor

        Returns:
            Interface instance
        """
        # Check cache first
        cache_key = (component_name, frozenset(kwargs.items()) if kwargs else None)
        if cache_key in cls._instance_cache:
            return cls._instance_cache[cache_key]

        # Get implementation
        mode = cls._current_config.get(component_name, "mock")
        implementation_path = cls._implementations[component_name][mode]

        if implementation_path is None:
            raise ValueError(f"No {mode} implementation available for {component_name}")

        # Import and instantiate
        try:
            module_path, class_name = implementation_path.rsplit(".", 1)
            module = __import__(module_path, fromlist=[class_name])
            class_obj = getattr(module, class_name)

            instance = class_obj(**kwargs)

            # Cache the instance
            cls._instance_cache[cache_key] = instance

            logger.info(f"Created {mode} {component_name} interface: {class_name}")
            return instance

        except Exception as e:
            logger.error(f"Failed to create {component_name} interface ({mode}): {e}")
            raise

    @classmethod
    def get_current_configuration(cls) -> Dict[str, str]:
        """Get current hardware configuration."""
        return cls._current_config.copy()

    @classmethod
    def get_available_components(cls) -> Dict[str, Dict[str, Optional[str]]]:
        """Get all available components and their implementations."""
        return cls._implementations.copy()

    @classmethod
    def validate_configuration(cls, config: Dict[str, str]) -> List[str]:
        """
        Validate a hardware configuration.

        Args:
            config: Configuration to validate

        Returns:
            List of validation error messages (empty if valid)
        """
        errors = []

        for component, mode in config.items():
            if component not in cls._implementations:
                errors.append(f"Unknown component: {component}")
                continue

            if mode not in ["mock", "hardware"]:
                errors.append(
                    f"Invalid mode for {component}: {mode} (must be 'mock' or 'hardware')"
                )
                continue

            if (
                mode == "hardware"
                and cls._implementations[component]["hardware"] is None
            ):
                errors.append(f"Hardware implementation not available for {component}")

        return errors

    @classmethod
    def reset_cache(cls):
        """Reset the instance cache (useful for testing)."""
        cls._instance_cache.clear()
        logger.info("Hardware interface cache reset")


# Convenience functions for common usage patterns
def create_full_rover_system(mock_all: bool = True) -> Dict[str, Any]:
    """
    Create a complete rover system with all components.

    Args:
        mock_all: If True, use all mock implementations

    Returns:
        Dict containing all system components
    """
    if mock_all:
        HardwareInterfaceFactory.set_configuration(
            {
                "can_bus": "mock",
                "motor_controller": "mock",
                "drive_system": "mock",
                "robotic_arm": "mock",
                "science_payload": "mock",
                "power_system": "mock",
                "sensor_imu": "mock",
                "sensor_gps": "mock",
            }
        )

    return {
        "can_bus": HardwareInterfaceFactory.create_can_bus(),
        "drive_system": HardwareInterfaceFactory.create_drive_system(),
        "robotic_arm": HardwareInterfaceFactory.create_robotic_arm(),
        "science_payload": HardwareInterfaceFactory.create_science_payload(),
        "power_system": HardwareInterfaceFactory.create_power_system(),
        "imu_sensor": HardwareInterfaceFactory.create_sensor_imu(20),
        "gps_sensor": HardwareInterfaceFactory.create_sensor_gps(30),
    }


def switch_to_hardware_integration_phase(phase: str) -> bool:
    """
    Switch to a predefined hardware integration phase.

    Args:
        phase: Integration phase name

    Returns:
        bool: True if phase switch successful
    """
    phases = {
        "development": {
            "can_bus": "mock",
            "motor_controller": "mock",
            "drive_system": "mock",
            "robotic_arm": "mock",
            "science_payload": "mock",
            "power_system": "mock",
            "sensor_imu": "mock",
            "sensor_gps": "mock",
        },
        "can_bus_integration": {
            "can_bus": "hardware",  # Requires CAN hardware
            "motor_controller": "mock",
            "drive_system": "mock",
            "robotic_arm": "mock",
            "science_payload": "mock",
            "power_system": "mock",
            "sensor_imu": "mock",
            "sensor_gps": "mock",
        },
        "drive_system_integration": {
            "can_bus": "hardware",
            "motor_controller": "hardware",  # Requires motor controllers
            "drive_system": "hardware",  # Requires drive system hardware
            "robotic_arm": "mock",
            "science_payload": "mock",
            "power_system": "mock",
            "sensor_imu": "mock",
            "sensor_gps": "mock",
        },
        "full_hardware": {
            "can_bus": "hardware",
            "motor_controller": "hardware",
            "drive_system": "hardware",
            "robotic_arm": "hardware",  # Requires arm hardware
            "science_payload": "hardware",  # Requires science hardware
            "power_system": "hardware",  # Requires power hardware
            "sensor_imu": "hardware",  # Requires IMU hardware
            "sensor_gps": "hardware",  # Requires GPS hardware
        },
    }

    if phase not in phases:
        logger.error(f"Unknown integration phase: {phase}")
        logger.info(f"Available phases: {list(phases.keys())}")
        return False

    config = phases[phase]
    validation_errors = HardwareInterfaceFactory.validate_configuration(config)

    if validation_errors:
        logger.error(f"Configuration validation failed for phase '{phase}':")
        for error in validation_errors:
            logger.error(f"  - {error}")
        return False

    success = HardwareInterfaceFactory.set_configuration(config)
    if success:
        logger.info(f"Successfully switched to integration phase: {phase}")

    return success
