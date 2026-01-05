#!/usr/bin/env python3
"""
Feature Flags System for URC 2026

Provides centralized feature flag management for mission-based component enabling/disabling.
Allows components to check their operational status based on current mission requirements.

Key Features:
- Mission-based feature toggling
- Runtime feature flag updates
- Component status queries
- Integration with MissionResourceManager
"""

import logging
from typing import Dict, Any, Optional, Callable, Set
from enum import Enum

logger = logging.getLogger(__name__)


class FeatureStatus(Enum):
    """Feature operational status."""
    ENABLED = "enabled"
    DISABLED = "disabled"
    LIGHTWEIGHT = "lightweight"
    UNAVAILABLE = "unavailable"


class FeatureFlagManager:
    """
    Centralized feature flag management system.

    Provides runtime feature flag evaluation based on mission profiles
    and system capabilities.
    """

    def __init__(self):
        """Initialize feature flag manager."""
        self.current_mission_profile: Optional[str] = None
        self.mission_config: Dict[str, Any] = {}
        self.capability_flags: Dict[str, bool] = {}
        self.feature_callbacks: Dict[str, Callable[[FeatureStatus], None]] = {}

        # Initialize capability detection
        self._detect_system_capabilities()

        logger.info("Feature Flag Manager initialized")

    def _detect_system_capabilities(self):
        """Detect available system capabilities and libraries."""

        # Check for ML libraries
        try:
            import torch
            self.capability_flags['torch_available'] = True
        except ImportError:
            self.capability_flags['torch_available'] = False

        try:
            import detectron2
            self.capability_flags['detectron_available'] = True
        except ImportError:
            self.capability_flags['detectron_available'] = False

        try:
            import tensorflow as tf
            self.capability_flags['tensorflow_available'] = True
        except ImportError:
            self.capability_flags['tensorflow_available'] = False

        # Check for resource manager
        try:
            from src.core.mission_resource_manager import get_mission_resource_manager
            self.capability_flags['resource_manager_available'] = True
            self.resource_manager = get_mission_resource_manager()
        except ImportError:
            self.capability_flags['resource_manager_available'] = False
            self.resource_manager = None

        # Hardware capabilities (would be detected from actual hardware)
        self.capability_flags['camera_available'] = True  # Assume available for simulation
        self.capability_flags['slam_capable'] = True
        self.capability_flags['excavation_available'] = True

        logger.info(f"Detected capabilities: {self.capability_flags}")

    def set_mission_profile(self, mission_type: str, config: Optional[Dict[str, Any]] = None):
        """
        Set the current mission profile and update feature flags accordingly.

        Args:
            mission_type: Mission profile name
            config: Optional mission configuration override
        """
        self.current_mission_profile = mission_type

        if config:
            self.mission_config = config
        else:
            # Load from resource manager if available
            if self.resource_manager:
                full_config = self.resource_manager.config
                mission_profiles = full_config.get('mission_profiles', {})
                self.mission_config = mission_profiles.get(mission_type, {})
            else:
                # Fallback defaults
                self.mission_config = self._get_default_mission_config(mission_type)

        # Update resource manager if available
        if self.resource_manager:
            self.resource_manager.switch_mission_profile(mission_type)

        logger.info(f"Feature flags updated for mission profile: {mission_type}")

    def _get_default_mission_config(self, mission_type: str) -> Dict[str, Any]:
        """Get default configuration for a mission type."""
        defaults = {
            'waypoint_navigation': {
                'computer_vision_enabled': False,
                'terrain_analysis_enabled': False,
                'slam_loop_closure_enabled': False,
                'excavation_enabled': False,
                'advanced_path_planning': False,
            },
            'object_search': {
                'computer_vision_enabled': True,
                'terrain_analysis_enabled': False,
                'slam_loop_closure_enabled': True,
                'excavation_enabled': False,
                'advanced_path_planning': False,
            },
            'sample_collection': {
                'computer_vision_enabled': True,
                'terrain_analysis_enabled': True,
                'slam_loop_closure_enabled': True,
                'excavation_enabled': True,
                'advanced_path_planning': True,
            }
        }
        return defaults.get(mission_type, {})

    def is_feature_enabled(self, feature_name: str) -> bool:
        """
        Check if a feature is enabled for the current mission.

        Args:
            feature_name: Name of the feature to check

        Returns:
            True if feature is enabled
        """
        return self.get_feature_status(feature_name) == FeatureStatus.ENABLED

    def get_feature_status(self, feature_name: str) -> FeatureStatus:
        """
        Get the status of a feature for the current mission.

        Args:
            feature_name: Name of the feature to check

        Returns:
            FeatureStatus enum value
        """
        # Check mission-specific configuration first
        mission_enabled = self.mission_config.get(f"{feature_name}_enabled",
                                                self.mission_config.get(feature_name, True))

        # Check capability availability
        capability_required = self._get_capability_requirement(feature_name)
        if capability_required and not self.capability_flags.get(capability_required, False):
            return FeatureStatus.UNAVAILABLE

        # Check resource manager status if available
        if self.resource_manager:
            resource_status = self.resource_manager.get_resource_status()
            component_status = resource_status.get('component_status', {})

            # Map feature names to component names
            component_mapping = {
                'computer_vision': 'computer_vision',
                'slam': 'slam_full',
                'terrain_analysis': 'terrain_analysis',
                'excavation': 'excavation',
                'advanced_path_planning': 'advanced_path_planning'
            }

            component_name = component_mapping.get(feature_name)
            if component_name:
                status_str = component_status.get(component_name, 'disabled')
                try:
                    return FeatureStatus(status_str)
                except ValueError:
                    pass

        # Fallback to mission configuration
        if mission_enabled:
            return FeatureStatus.ENABLED
        else:
            return FeatureStatus.DISABLED

    def _get_capability_requirement(self, feature_name: str) -> Optional[str]:
        """Get the capability flag required for a feature."""
        capability_map = {
            'computer_vision': 'torch_available',  # At least basic vision
            'ml_vision': 'torch_available',       # Full ML vision
            'object_detection': 'detectron_available',
            'terrain_ml': 'tensorflow_available',
        }
        return capability_map.get(feature_name)

    def register_feature_callback(self, feature_name: str, callback: Callable[[FeatureStatus], None]):
        """
        Register a callback to be called when a feature's status changes.

        Args:
            feature_name: Name of the feature
            callback: Function to call with new FeatureStatus
        """
        self.feature_callbacks[feature_name] = callback

    def get_enabled_features(self) -> Set[str]:
        """
        Get set of currently enabled features.

        Returns:
            Set of enabled feature names
        """
        enabled = set()
        common_features = [
            'computer_vision', 'slam', 'terrain_analysis', 'excavation',
            'advanced_path_planning', 'ml_vision', 'object_detection', 'terrain_ml'
        ]

        for feature in common_features:
            if self.is_feature_enabled(feature):
                enabled.add(feature)

        return enabled

    def get_feature_info(self, feature_name: str) -> Dict[str, Any]:
        """
        Get detailed information about a feature.

        Args:
            feature_name: Name of the feature

        Returns:
            Dictionary with feature information
        """
        status = self.get_feature_status(feature_name)
        capability_req = self._get_capability_requirement(feature_name)

        info = {
            'name': feature_name,
            'status': status.value,
            'mission_enabled': self.mission_config.get(f"{feature_name}_enabled",
                                                     self.mission_config.get(feature_name, True)),
            'capability_required': capability_req,
            'capability_available': self.capability_flags.get(capability_req, False) if capability_req else True,
            'mission_profile': self.current_mission_profile
        }

        return info

    def get_all_feature_status(self) -> Dict[str, Dict[str, Any]]:
        """
        Get status information for all features.

        Returns:
            Dictionary mapping feature names to their status info
        """
        features = [
            'computer_vision', 'slam', 'terrain_analysis', 'excavation',
            'advanced_path_planning', 'ml_vision', 'object_detection', 'terrain_ml'
        ]

        return {feature: self.get_feature_info(feature) for feature in features}


# Global instance
_feature_flag_manager = None

def get_feature_flag_manager() -> FeatureFlagManager:
    """Get the global feature flag manager instance."""
    global _feature_flag_manager
    if _feature_flag_manager is None:
        _feature_flag_manager = FeatureFlagManager()
    return _feature_flag_manager


# Convenience functions for easy feature checking
def is_feature_enabled(feature_name: str) -> bool:
    """
    Convenience function to check if a feature is enabled.

    Args:
        feature_name: Name of the feature to check

    Returns:
        True if feature is enabled
    """
    manager = get_feature_flag_manager()
    return manager.is_feature_enabled(feature_name)


def require_feature(feature_name: str) -> bool:
    """
    Require a feature to be enabled, raise exception if not.

    Args:
        feature_name: Name of the required feature

    Returns:
        True if feature is available

    Raises:
        RuntimeError: If feature is not enabled
    """
    if not is_feature_enabled(feature_name):
        raise RuntimeError(f"Required feature '{feature_name}' is not enabled for current mission")
    return True


def with_feature(feature_name: str):
    """
    Decorator to conditionally execute functions based on feature availability.

    Args:
        feature_name: Feature that must be enabled for function to run

    Returns:
        Decorated function that only executes if feature is enabled
    """
    def decorator(func):
        def wrapper(*args, **kwargs):
            if is_feature_enabled(feature_name):
                return func(*args, **kwargs)
            else:
                logger.debug(f"Skipping {func.__name__} - feature '{feature_name}' not enabled")
                return None
        return wrapper
    return decorator


def get_enabled_features() -> Set[str]:
    """
    Get set of currently enabled features.

    Returns:
        Set of enabled feature names
    """
    manager = get_feature_flag_manager()
    return manager.get_enabled_features()


if __name__ == "__main__":
    # Demo the feature flag system
    print("🔧 FEATURE FLAG SYSTEM DEMO")
    print("=" * 40)

    manager = get_feature_flag_manager()

    print("📊 Available capabilities:")
    for cap, available in manager.capability_flags.items():
        print(f"  {cap}: {'✅' if available else '❌'}")

    print("\n🎯 Testing mission profiles:")

    # Test waypoint navigation
    manager.set_mission_profile('waypoint_navigation')
    print(f"Waypoint navigation - enabled features: {get_enabled_features()}")

    # Test sample collection
    manager.set_mission_profile('sample_collection')
    print(f"Sample collection - enabled features: {get_enabled_features()}")

    # Test feature checking
    print("\n🔍 Feature status checks:")    print(f"  Computer vision enabled: {is_feature_enabled('computer_vision')}")
    print(f"  ML vision enabled: {is_feature_enabled('ml_vision')}")
    print(f"  Terrain analysis enabled: {is_feature_enabled('terrain_analysis')}")

    print("\n✅ Feature flag system demo completed!")
