"""Environment factory for creating and managing simulation environments.

Provides unified interface for creating different environmental conditions
with consistent configuration and initialization.

Author: URC 2026 Autonomy Team
"""

import logging
from typing import Any, Dict, Optional

from simulation.environments.extreme_environment import ExtremeEnvironment
from simulation.environments.perfect_environment import PerfectEnvironment
from simulation.environments.real_life_environment import RealLifeEnvironment


class EnvironmentFactory:
    """Factory for creating simulated environments.

    Manages environment creation, configuration, and lifecycle.
    Provides consistent interface for all environment types.
    """

    # Registry of available environment types
    _environment_types = {
        "perfect": PerfectEnvironment,
        "real_life": RealLifeEnvironment,
        "extreme": ExtremeEnvironment,
        # Add more environment types as needed
        # "mars": MarsEnvironment,
        # "lunar": LunarEnvironment,
        # "custom": CustomEnvironment,
    }

    @classmethod
    def create(cls, config: Dict[str, Any]) -> Any:
        """Create an environment instance.

        Args:
            config: Environment configuration dictionary

        Returns:
            Environment instance

        Raises:
            ValueError: If environment tier is not supported
        """
        tier_name = config.get("tier", "perfect")

        if tier_name not in cls._environment_types:
            available_tiers = list(cls._environment_types.keys())
            raise ValueError(
                f"Unsupported environment tier '{tier_name}'. "
                f"Available tiers: {available_tiers}"
            )

        environment_class = cls._environment_types[tier_name]

        # Validate configuration
        cls._validate_config(tier_name, config)

        # Create environment instance
        try:
            environment = environment_class(config)
            return environment
        except Exception as e:
            logger = logging.getLogger(__name__)
            logger.error(f"Failed to create {tier_name} environment: {e}")
            raise

    @classmethod
    def register_environment_type(cls, tier_name: str, environment_class: type):
        """Register a new environment type.

        Args:
            tier_name: Name of environment tier
            environment_class: Environment class
        """
        cls._environment_types[tier_name] = environment_class

        logger = logging.getLogger(__name__)
        logger.info(f"Registered environment tier: {tier_name}")

    @classmethod
    def get_available_tiers(cls) -> list[str]:
        """Get list of available environment tiers.

        Returns:
            list: Available environment tier names
        """
        return list(cls._environment_types.keys())

    @classmethod
    def get_environment_info(cls, tier_name: str) -> Optional[Dict[str, Any]]:
        """Get information about an environment tier.

        Args:
            tier_name: Name of environment tier

        Returns:
            Dict with environment information or None if not found
        """
        if tier_name not in cls._environment_types:
            return None

        environment_class = cls._environment_types[tier_name]

        return {
            "tier": tier_name,
            "class": environment_class.__name__,
            "module": environment_class.__module__,
            "description": environment_class.__doc__ or "No description available",
        }

    @classmethod
    def _validate_config(cls, tier_name: str, config: Dict[str, Any]) -> None:
        """Validate environment configuration.

        Args:
            tier_name: Type of environment tier
            config: Configuration to validate

        Raises:
            ValueError: If configuration is invalid
        """
        # Common validation
        if "tier" not in config:
            raise ValueError("Environment configuration must include 'tier' field")

        # Tier-specific validation
        if tier_name == "perfect":
            cls._validate_perfect_config(config)
        elif tier_name == "real_life":
            cls._validate_real_life_config(config)
        elif tier_name == "extreme":
            cls._validate_extreme_config(config)

    @classmethod
    def _validate_perfect_config(cls, config: Dict[str, Any]) -> None:
        """Validate perfect environment configuration."""
        # Perfect environment has minimal requirements
        pass

    @classmethod
    def _validate_real_life_config(cls, config: Dict[str, Any]) -> None:
        """Validate real-life environment configuration."""
        # Real-life should have reasonable environmental parameters
        if "visibility" in config and not (0.1 <= config["visibility"] <= 1.0):
            raise ValueError("Real-life visibility must be between 0.1 and 1.0")

        if "temperature" in config and not (-20 <= config["temperature"] <= 60):
            raise ValueError("Real-life temperature must be between -20°C and 60°C")

    @classmethod
    def _validate_extreme_config(cls, config: Dict[str, Any]) -> None:
        """Validate extreme environment configuration."""
        # Extreme should have challenging parameters
        if "visibility" in config and config["visibility"] > 0.5:
            raise ValueError("Extreme environment visibility should be ≤ 0.5")

        if "wind_speed" in config and config["wind_speed"] < 10:
            raise ValueError("Extreme environment wind speed should be ≥ 10 m/s")

        if "temperature" in config and abs(config["temperature"] - 25) < 20:
            raise ValueError("Extreme environment temperature should differ from 25°C by ≥ 20°C")
