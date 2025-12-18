"""Network factory for creating and managing simulated network conditions.

Provides unified interface for creating different network profiles with
consistent configuration and initialization.

Author: URC 2026 Autonomy Team
"""

import logging
from typing import Any, Dict, Optional

from simulation.network.network_emulator import NetworkEmulator, NetworkProfile


class NetworkFactory:
    """Factory for creating simulated network conditions.

    Manages network profile creation, configuration, and lifecycle.
    Provides consistent interface for all network types.
    """

    @classmethod
    def create(cls, config: Dict[str, Any]) -> NetworkEmulator:
        """Create a network emulator instance.

        Args:
            config: Network configuration dictionary

        Returns:
            NetworkEmulator: Configured network emulator instance

        Raises:
            ValueError: If network profile is not supported
        """
        profile_name = config.get("profile", "perfect")

        try:
            profile = NetworkProfile(profile_name)
        except ValueError:
            available_profiles = [p.value for p in NetworkProfile]
            raise ValueError(
                f"Unsupported network profile '{profile_name}'. "
                f"Available profiles: {available_profiles}"
            )

        # Create network emulator
        try:
            emulator = NetworkEmulator(profile)

            # Configure additional settings
            if "max_queue_size" in config:
                emulator.max_queue_size = config["max_queue_size"]

            if "custom_latency" in config:
                # Allow custom latency override for testing
                custom_config = config["custom_latency"]
                emulator.condition.latency_ms = custom_config.get(
                    "base_ms", emulator.condition.latency_ms
                )
                emulator.condition.jitter_ms = custom_config.get(
                    "jitter_ms", emulator.condition.jitter_ms
                )

            return emulator

        except Exception as e:
            logger = logging.getLogger(__name__)
            logger.error(f"Failed to create {profile_name} network emulator: {e}")
            raise

    @classmethod
    def get_available_profiles(cls) -> list[str]:
        """Get list of available network profiles.

        Returns:
            list: Available network profile names
        """
        return [profile.value for profile in NetworkProfile]

    @classmethod
    def get_profile_info(cls, profile_name: str) -> Optional[Dict[str, Any]]:
        """Get information about a network profile.

        Args:
            profile_name: Name of network profile

        Returns:
            Dict with profile information or None if not found
        """
        try:
            profile = NetworkProfile(profile_name)
            condition = NetworkEmulator.NETWORK_PROFILES[profile]

            return {
                "name": profile.value,
                "latency_ms": condition.latency_ms,
                "jitter_ms": condition.jitter_ms,
                "packet_loss_percent": condition.packet_loss_percent,
                "bandwidth_mbps": condition.bandwidth_mbps,
                "connection_drops": condition.connection_drops,
                "description": cls._get_profile_description(profile),
            }

        except ValueError:
            return None

    @classmethod
    def create_custom_profile(
        cls,
        name: str,
        latency_ms: float,
        jitter_ms: float,
        packet_loss_percent: float,
        bandwidth_mbps: float,
        connection_drops: bool = False,
        drop_duration_s: float = 5.0,
    ) -> NetworkEmulator:
        """Create a custom network profile.

        Args:
            name: Custom profile name
            latency_ms: Base latency in milliseconds
            jitter_ms: Latency variation in milliseconds
            packet_loss_percent: Packet loss percentage
            bandwidth_mbps: Bandwidth in Mbps
            connection_drops: Whether connection drops occur
            drop_duration_s: Duration of connection drops

        Returns:
            NetworkEmulator: Custom network emulator
        """
        # Create a temporary profile for custom settings
        custom_profile = NetworkProfile.PERFECT  # Use as base
        emulator = NetworkEmulator(custom_profile)

        # Override with custom settings
        emulator.condition.latency_ms = latency_ms
        emulator.condition.jitter_ms = jitter_ms
        emulator.condition.packet_loss_percent = packet_loss_percent
        emulator.condition.bandwidth_mbps = bandwidth_mbps
        emulator.condition.connection_drops = connection_drops
        emulator.condition.drop_duration_s = drop_duration_s

        emulator.profile_name = name  # Custom identifier

        return emulator

    @classmethod
    def _get_profile_description(cls, profile: NetworkProfile) -> str:
        """Get human-readable description of network profile.

        Args:
            profile: Network profile enum

        Returns:
            str: Profile description
        """
        descriptions = {
            NetworkProfile.PERFECT: "Ideal network conditions - no latency, no packet loss",
            NetworkProfile.RURAL_WIFI: "Rural WiFi conditions - moderate latency, occasional packet loss",
            NetworkProfile.CELLULAR_4G: "4G cellular network - higher latency, variable packet loss",
            NetworkProfile.SATELLITE: "Satellite connection - high latency, reliable but slow",
            NetworkProfile.EXTREME: "Extreme conditions - very high latency, significant packet loss",
        }

        return descriptions.get(profile, "Unknown network profile")

    @classmethod
    def validate_config(cls, config: Dict[str, Any]) -> bool:
        """Validate network configuration.

        Args:
            config: Configuration to validate

        Returns:
            bool: True if configuration is valid
        """
        if "profile" not in config:
            return False

        profile_name = config["profile"]
        if profile_name not in cls.get_available_profiles():
            return False

        # Validate numeric parameters if present
        if "max_queue_size" in config:
            if (
                not isinstance(config["max_queue_size"], int)
                or config["max_queue_size"] <= 0
            ):
                return False

        if "custom_latency" in config:
            custom = config["custom_latency"]
            if not all(key in custom for key in ["base_ms", "jitter_ms"]):
                return False
            if custom["base_ms"] < 0 or custom["jitter_ms"] < 0:
                return False

        return True
