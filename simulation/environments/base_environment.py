"""Base environment interface for simulation.

Provides common interface and functionality for all simulated environments
including weather, terrain, and environmental effects.

Author: URC 2026 Autonomy Team
"""

import logging
from abc import ABC, abstractmethod
from typing import Any, Dict

import numpy as np


class BaseEnvironment(ABC):
    """Abstract base class for all simulated environments.

    Provides common functionality for:
    - Weather simulation
    - Terrain modeling
    - Environmental effects
    - State management
    - Time-based evolution
    """

    def __init__(self, config: Dict[str, Any]):
        """Initialize base environment.

        Args:
            config: Environment configuration dictionary
        """
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")

        # Configuration
        self.tier_name = config.get("tier", "unknown")
        self.update_rate_hz = config.get("update_rate", 1.0)

        # Weather conditions
        self.temperature = config.get("temperature", 25.0)  # Celsius
        self.humidity = config.get("humidity", 0.45)  # 0-1
        self.wind_speed = config.get("wind_speed", 0.0)  # m/s
        self.wind_direction = config.get("wind_direction", 0.0)  # radians
        self.visibility = config.get("visibility", 1.0)  # 0-1 (1 = perfect visibility)
        self.dust_density = config.get("dust_density", 0.0)  # 0-1 (1 = extreme dust)

        # Terrain conditions
        self.terrain_difficulty = config.get("terrain_difficulty", 0.0)  # 0-1
        self.slope_angle = config.get("slope_angle", 0.0)  # radians
        self.traction = config.get("traction", 1.0)  # 0-1 (1 = perfect traction)
        self.surface_type = config.get("surface_type", "pavement")

        # Lighting conditions
        self.lighting = config.get("lighting", 1.0)  # 0-1 (1 = bright sunlight)

        # State tracking
        self.simulation_time = 0.0
        self.last_update_time = 0.0

        # Dynamic effects
        self.weather_patterns = []
        self.terrain_changes = []

        self.logger.info(f"Initialized {self.__class__.__name__} environment")

    @abstractmethod
    def _initialize_conditions(self):
        """Initialize environment-specific conditions."""
        pass

    @abstractmethod
    def _update_weather(self, dt: float):
        """Update weather conditions over time.

        Args:
            dt: Time step in seconds
        """
        pass

    @abstractmethod
    def _update_terrain(self, dt: float):
        """Update terrain conditions over time.

        Args:
            dt: Time step in seconds
        """
        pass

    def step(self, dt: float) -> Dict[str, Any]:
        """Execute environment simulation step.

        Args:
            dt: Time step in seconds

        Returns:
            Dict containing current environmental state
        """
        self.simulation_time += dt

        # Update environmental conditions
        self._update_weather(dt)
        self._update_terrain(dt)

        # Apply dynamic effects
        self._apply_dynamic_effects(dt)

        # Get current state
        state = self.get_state()
        state["timestamp"] = self.simulation_time

        return state

    def get_state(self) -> Dict[str, Any]:
        """Get current environment state.

        Returns:
            Dict containing complete environment state
        """
        return {
            "tier_name": self.tier_name,
            "temperature": self.temperature,
            "humidity": self.humidity,
            "wind_speed": self.wind_speed,
            "wind_direction": self.wind_direction,
            "visibility": self.visibility,
            "dust_density": self.dust_density,
            "terrain_difficulty": self.terrain_difficulty,
            "slope_angle": self.slope_angle,
            "traction": self.traction,
            "surface_type": self.surface_type,
            "lighting": self.lighting,
            "simulation_time": self.simulation_time,
            "is_simulation": True,
        }

    def reset(self):
        """Reset environment to initial state."""
        self.simulation_time = 0.0
        self.last_update_time = 0.0
        self.weather_patterns.clear()
        self.terrain_changes.clear()

        # Re-initialize conditions
        self._initialize_conditions()

        self.logger.info(f"Reset {self.tier_name} environment")

    def get_environmental_effects(self) -> Dict[str, Any]:
        """Get environmental effects on sensors and systems.

        Returns:
            Dict with effect multipliers and modifiers
        """
        return {
            # Sensor degradation factors
            "gps_accuracy_factor": self._calculate_gps_accuracy_factor(),
            "imu_noise_multiplier": self._calculate_imu_noise_multiplier(),
            "camera_visibility_factor": self.visibility,
            "lidar_range_factor": self._calculate_lidar_range_factor(),

            # Rover performance factors
            "traction_multiplier": self.traction,
            "rolling_resistance_factor": 1.0 + self.terrain_difficulty * 0.5,
            "wind_resistance_factor": 1.0 + self.wind_speed * 0.01,

            # Safety factors
            "collision_risk_factor": 1.0 - self.visibility,
            "thermal_stress_factor": abs(self.temperature - 25.0) / 50.0,
        }

    def _calculate_gps_accuracy_factor(self) -> float:
        """Calculate GPS accuracy degradation factor.

        Returns:
            float: Accuracy factor (0.0 = no accuracy, 1.0 = perfect)
        """
        # Dust reduces satellite visibility
        dust_factor = 1.0 - self.dust_density * 0.5

        # Wind can cause multipath effects
        wind_factor = max(0.7, 1.0 - self.wind_speed * 0.02)

        # Visibility affects atmospheric conditions
        visibility_factor = self.visibility

        return min(1.0, dust_factor * wind_factor * visibility_factor)

    def _calculate_imu_noise_multiplier(self) -> float:
        """Calculate IMU noise multiplier.

        Returns:
            float: Noise multiplier (>= 1.0)
        """
        # Temperature affects sensor noise
        temp_noise = 1.0 + abs(self.temperature - 25.0) * 0.01

        # Vibration from terrain
        terrain_noise = 1.0 + self.terrain_difficulty * 0.5

        # Wind-induced vibration
        wind_noise = 1.0 + self.wind_speed * 0.02

        return temp_noise * terrain_noise * wind_noise

    def _calculate_lidar_range_factor(self) -> float:
        """Calculate LiDAR range effectiveness factor.

        Returns:
            float: Range factor (0.0 = no range, 1.0 = full range)
        """
        # Dust heavily affects LiDAR
        dust_factor = 1.0 - self.dust_density * 0.8

        # Moisture/humidity affects returns
        humidity_factor = 1.0 - self.humidity * 0.3

        # Lighting can affect some sensors
        lighting_factor = 0.8 + self.lighting * 0.2

        return min(1.0, dust_factor * humidity_factor * lighting_factor)

    def _apply_dynamic_effects(self, dt: float):
        """Apply dynamic environmental effects.

        Args:
            dt: Time step in seconds
        """
        # Apply gradual changes to create realistic variation
        self._apply_weather_variation(dt)
        self._apply_terrain_variation(dt)
        self._apply_lighting_variation(dt)

    def _apply_weather_variation(self, dt: float):
        """Apply gradual weather variations.

        Args:
            dt: Time step in seconds
        """
        # Small random variations
        self.temperature += np.random.normal(0, 0.1 * dt)
        self.wind_direction += np.random.normal(0, 0.1 * dt)
        self.humidity = np.clip(self.humidity + np.random.normal(0, 0.01 * dt), 0, 1)

    def _apply_terrain_variation(self, dt: float):
        """Apply gradual terrain variations.

        Args:
            dt: Time step in seconds
        """
        # Terrain can change slightly over time (simulating rover movement)
        if np.random.random() < 0.01:  # 1% chance per second
            self.slope_angle += np.random.normal(0, 0.01)
            self.traction = np.clip(self.traction + np.random.normal(0, 0.02), 0.1, 1.0)

    def _apply_lighting_variation(self, dt: float):
        """Apply gradual lighting variations.

        Args:
            dt: Time step in seconds
        """
        # Simulate cloud movement or time of day
        self.lighting = np.clip(self.lighting + np.random.normal(0, 0.05 * dt), 0.1, 1.0)

    def set_weather_conditions(self, **conditions):
        """Set specific weather conditions.

        Args:
            **conditions: Weather parameters to set
        """
        valid_params = [
            "temperature", "humidity", "wind_speed", "wind_direction",
            "visibility", "dust_density", "lighting"
        ]

        for param, value in conditions.items():
            if param in valid_params:
                setattr(self, param, value)
                self.logger.info(f"Set {param} to {value}")
            else:
                self.logger.warning(f"Unknown weather parameter: {param}")

    def set_terrain_conditions(self, **conditions):
        """Set specific terrain conditions.

        Args:
            **conditions: Terrain parameters to set
        """
        valid_params = [
            "terrain_difficulty", "slope_angle", "traction", "surface_type"
        ]

        for param, value in conditions.items():
            if param in valid_params:
                setattr(self, param, value)
                self.logger.info(f"Set {param} to {value}")
            else:
                self.logger.warning(f"Unknown terrain parameter: {param}")

    def get_performance_impact(self) -> Dict[str, Any]:
        """Get performance impact assessment.

        Returns:
            Dict with performance impact metrics
        """
        effects = self.get_environmental_effects()

        return {
            "navigation_accuracy_degradation": 1.0 - effects["gps_accuracy_factor"],
            "sensor_reliability_degradation": (
                (effects["imu_noise_multiplier"] - 1.0) / 2.0 +
                (1.0 - effects["camera_visibility_factor"]) / 2.0 +
                (1.0 - effects["lidar_range_factor"]) / 2.0
            ) / 3.0,
            "rover_performance_degradation": (
                (1.0 - effects["traction_multiplier"]) +
                (effects["rolling_resistance_factor"] - 1.0) +
                (effects["wind_resistance_factor"] - 1.0)
            ) / 3.0,
            "safety_risk_increase": effects["collision_risk_factor"],
            "overall_difficulty_score": (
                self.terrain_difficulty +
                (1.0 - self.visibility) +
                self.dust_density +
                self.wind_speed / 20.0 +
                abs(self.temperature - 25.0) / 50.0
            ) / 5.0,
        }
