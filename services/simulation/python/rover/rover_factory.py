"""Rover factory for creating and managing simulated rover models.

Provides unified interface for creating different rover configurations with
consistent physics and control models.

Author: URC 2026 Autonomy Team
"""

import logging
from typing import Any, Dict, Optional

from simulation.rover.base_rover import BaseRover
from simulation.rover.urc_rover import URCRover


class RoverFactory:
    """Factory for creating simulated rover models.

    Manages rover creation, configuration, and lifecycle.
    Provides consistent interface for all rover types.
    """

    # Registry of available rover types
    _rover_types = {
        "urc_rover": URCRover,
        "simple_rover": URCRover,  # Alias for backward compatibility
        # Add more rover types here as they are implemented
        # "ackermann_rover": AckermannRover,
        # "differential_rover": DifferentialRover,
        # "omni_rover": OmniRover,
    }

    @classmethod
    def create(cls, config: Dict[str, Any]) -> BaseRover:
        """Create a rover instance.

        Args:
            config: Rover configuration dictionary

        Returns:
            BaseRover: Configured rover instance

        Raises:
            ValueError: If rover model is not supported
        """
        model_name = config.get("model", "urc_rover")

        if model_name not in cls._rover_types:
            available_models = list(cls._rover_types.keys())
            raise ValueError(
                f"Unsupported rover model '{model_name}'. "
                f"Available models: {available_models}"
            )

        rover_class = cls._rover_types[model_name]

        # Validate configuration
        cls._validate_config(model_name, config)

        # Create rover instance
        try:
            rover = rover_class(config)
            return rover
        except Exception as e:
            logger = logging.getLogger(__name__)
            logger.error(f"Failed to create {model_name} rover: {e}")
            raise

    @classmethod
    def register_rover_type(cls, model_name: str, rover_class: type):
        """Register a new rover type.

        Args:
            model_name: Name of rover model
            rover_class: Rover class that inherits from BaseRover
        """
        if not issubclass(rover_class, BaseRover):
            raise TypeError("Rover class must inherit from BaseRover")

        cls._rover_types[model_name] = rover_class

        logger = logging.getLogger(__name__)
        logger.info(f"Registered rover model: {model_name}")

    @classmethod
    def get_available_models(cls) -> list[str]:
        """Get list of available rover models.

        Returns:
            list: Available rover model names
        """
        return list(cls._rover_types.keys())

    @classmethod
    def get_rover_info(cls, model_name: str) -> Optional[Dict[str, Any]]:
        """Get information about a rover model.

        Args:
            model_name: Type of rover model

        Returns:
            Dict with rover information or None if not found
        """
        if model_name not in cls._rover_types:
            return None

        rover_class = cls._rover_types[model_name]

        return {
            "model": model_name,
            "class": rover_class.__name__,
            "module": rover_class.__module__,
            "description": rover_class.__doc__ or "No description available",
        }

    @classmethod
    def _validate_config(cls, model_name: str, config: Dict[str, Any]) -> None:
        """Validate rover configuration.

        Args:
            model_name: Type of rover model
            config: Configuration to validate

        Raises:
            ValueError: If configuration is invalid
        """
        # Common validation
        if "model" not in config:
            raise ValueError("Rover configuration must include 'model' field")

        # Model-specific validation
        if model_name in ["urc_rover", "simple_rover"]:
            cls._validate_urc_rover_config(config)

    @classmethod
    def _validate_urc_rover_config(cls, config: Dict[str, Any]) -> None:
        """Validate URC rover configuration."""
        # Wheel configuration
        wheel_count = config.get("wheel_count", 4)
        if wheel_count not in [4, 6]:
            raise ValueError(f"URC rover wheel_count must be 4 or 6, got {wheel_count}")

        # Dimensions
        if "wheelbase" in config:
            wheelbase = config["wheelbase"]
            if not (0.1 <= wheelbase <= 2.0):
                raise ValueError(
                    f"Wheelbase {wheelbase}m not in valid range [0.1, 2.0]"
                )

        if "track_width" in config:
            track_width = config["track_width"]
            if not (0.1 <= track_width <= 1.5):
                raise ValueError(
                    f"Track width {track_width}m not in valid range [0.1, 1.5]"
                )

        # Mass
        if "mass" in config:
            mass = config["mass"]
            if not (10 <= mass <= 200):
                raise ValueError(f"Mass {mass}kg not in valid range [10, 200]")

        # Performance limits
        if "max_velocity" in config:
            max_vel = config["max_velocity"]
            if not (0.1 <= max_vel <= 5.0):
                raise ValueError(
                    f"Max velocity {max_vel}m/s not in valid range [0.1, 5.0]"
                )

        if "max_acceleration" in config:
            max_acc = config["max_acceleration"]
            if not (0.1 <= max_acc <= 3.0):
                raise ValueError(
                    f"Max acceleration {max_acc}m/s² not in valid range [0.1, 3.0]"
                )
