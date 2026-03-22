"""Real-life environment simulation.

Provides realistic URC field conditions with moderate environmental effects.
Represents typical competition scenarios with expected challenges.

Author: URC 2026 Autonomy Team
"""

import math
from typing import Any, Dict

import numpy as np

from simulation.environments.base_environment import BaseEnvironment


class RealLifeEnvironment(BaseEnvironment):
    """Real-life environment with typical URC field conditions.

    Moderate environmental effects, realistic sensor degradation,
    challenging but manageable terrain conditions.
    """

    def __init__(self, config: Dict[str, Any]):
        """Initialize real-life environment.

        Args:
            config: Environment configuration
        """
        # Set real-life defaults based on URC 2026 expectations
        config.setdefault("tier", "real_life")
        config.setdefault("temperature", 35.0)  # Hot desert conditions
        config.setdefault("humidity", 0.15)  # Dry desert air
        config.setdefault("wind_speed", 15.0)  # Moderate wind (34 mph)
        config.setdefault("wind_direction", math.pi / 4)  # 45-degree wind
        config.setdefault("visibility", 0.8)  # 80% visibility (light dust)
        config.setdefault("dust_density", 0.3)  # Moderate dust
        config.setdefault("terrain_difficulty", 0.4)  # Moderate terrain
        config.setdefault("slope_angle", 0.1)  # 5.7-degree slope
        config.setdefault("traction", 0.85)  # Good but not perfect traction
        config.setdefault("surface_type", "sand")  # Desert sand
        config.setdefault("lighting", 0.9)  # Slight cloud cover

        super().__init__(config)
        self._initialize_conditions()

    def _initialize_conditions(self):
        """Initialize real-life environmental conditions."""
        # Add some initial variation
        self.temperature += np.random.normal(0, 2.0)  # ±2°C variation
        self.wind_speed += np.random.normal(0, 2.0)  # ±2 m/s wind variation
        self.visibility *= np.random.uniform(0.95, 1.05)  # ±5% visibility variation

    def _update_weather(self, dt: float):
        """Update weather conditions with realistic patterns.

        Args:
            dt: Time step in seconds
        """
        # Simulate desert weather patterns
        time_hours = self.simulation_time / 3600.0

        # Temperature variation (diurnal cycle)
        base_temp = 35.0
        diurnal_variation = 10.0 * math.sin(2 * math.pi * time_hours / 24.0)
        self.temperature = base_temp + diurnal_variation

        # Add realistic noise
        self.temperature += np.random.normal(0, 0.5)

        # Wind gusts (occasional stronger winds)
        if np.random.random() < 0.02:  # 2% chance per second
            gust_strength = np.random.uniform(5.0, 15.0)
            self.wind_speed = min(25.0, self.wind_speed + gust_strength)

            # Gust duration
            gust_duration = np.random.uniform(5.0, 30.0)
            # Schedule gust end (simplified)
            self._schedule_wind_change(self.wind_speed - gust_strength, gust_duration)

        # Gradual wind direction changes
        self.wind_direction += np.random.normal(0, 0.02)  # Slow direction drift

        # Dust storms (rare but possible)
        if np.random.random() < 0.001:  # 0.1% chance per second
            self._trigger_dust_storm()

        # Humidity changes (very dry but can vary)
        humidity_target = 0.15 + np.random.normal(0, 0.05)
        humidity_change = (humidity_target - self.humidity) * 0.01
        self.humidity += humidity_change

        # Visibility affected by wind and dust
        wind_visibility_factor = max(0.7, 1.0 - self.wind_speed * 0.02)
        dust_visibility_factor = 1.0 - self.dust_density * 0.5
        self.visibility = min(1.0, wind_visibility_factor * dust_visibility_factor)

    def _update_terrain(self, dt: float):
        """Update terrain conditions with realistic changes.

        Args:
            dt: Time step in seconds
        """
        # Terrain can change as rover moves (different surface types)
        if np.random.random() < 0.005:  # 0.5% chance per second
            # Switch between different surface types
            surface_types = ["sand", "rock", "gravel", "hard_pack"]
            new_surface = np.random.choice(surface_types)

            if new_surface != self.surface_type:
                self.surface_type = new_surface
                self._update_surface_properties()

        # Slope variations
        slope_change = np.random.normal(0, 0.01)  # Small slope variations
        self.slope_angle += slope_change
        self.slope_angle = np.clip(self.slope_angle, -0.3, 0.3)  # ±17 degrees

        # Traction affected by surface moisture (rare in desert)
        if np.random.random() < 0.001:  # Very rare moisture
            moisture_effect = np.random.uniform(0.05, 0.15)
            self.traction = min(1.0, self.traction + moisture_effect)

            # Moisture evaporates quickly
            self._schedule_traction_change(
                self.traction - moisture_effect, 300.0
            )  # 5 minutes

    def _update_surface_properties(self):
        """Update terrain properties based on surface type."""
        surface_properties = {
            "sand": {"traction": 0.75, "difficulty": 0.6, "resistance": 1.3},
            "rock": {"traction": 0.9, "difficulty": 0.3, "resistance": 0.8},
            "gravel": {"traction": 0.8, "difficulty": 0.4, "resistance": 1.1},
            "hard_pack": {"traction": 0.95, "difficulty": 0.2, "resistance": 0.9},
        }

        props = surface_properties.get(self.surface_type, surface_properties["sand"])

        # Smooth transition to new properties
        transition_rate = 0.1
        self.traction = (
            self.traction * (1 - transition_rate) + props["traction"] * transition_rate
        )
        self.terrain_difficulty = (
            self.terrain_difficulty * (1 - transition_rate)
            + props["difficulty"] * transition_rate
        )

    def _schedule_wind_change(self, target_speed: float, delay_seconds: float):
        """Schedule a wind speed change.

        Args:
            target_speed: Target wind speed
            delay_seconds: Delay before change
        """
        # Simplified implementation - in practice would use threading.Timer
        self._wind_change_target = target_speed
        self._wind_change_time = self.simulation_time + delay_seconds

    def _schedule_traction_change(self, target_traction: float, delay_seconds: float):
        """Schedule a traction change.

        Args:
            target_traction: Target traction value
            delay_seconds: Delay before change
        """
        self._traction_change_target = target_traction
        self._traction_change_time = self.simulation_time + delay_seconds

    def _trigger_dust_storm(self):
        """Trigger a dust storm event."""
        self.logger.warning("Dust storm triggered!")

        # Increase dust density
        original_dust = self.dust_density
        self.dust_density = min(0.8, original_dust + np.random.uniform(0.3, 0.5))

        # Increase wind
        original_wind = self.wind_speed
        self.wind_speed = min(30.0, original_wind + np.random.uniform(10.0, 20.0))

        # Reduce visibility dramatically
        self.visibility = max(0.1, self.visibility * 0.2)

        # Schedule storm end (30-120 minutes)
        storm_duration = np.random.uniform(1800.0, 7200.0)  # 30-120 minutes
        self._schedule_storm_end(original_dust, original_wind, storm_duration)

    def _schedule_storm_end(
        self, original_dust: float, original_wind: float, duration: float
    ):
        """Schedule dust storm end.

        Args:
            original_dust: Original dust density
            original_wind: Original wind speed
            duration: Storm duration in seconds
        """
        self._storm_end_time = self.simulation_time + duration
        self._storm_original_dust = original_dust
        self._storm_original_wind = original_wind

    def get_environmental_effects(self) -> Dict[str, Any]:
        """Get environmental effects for real-life conditions.

        Returns:
            Dict with realistic environmental effects
        """
        base_effects = super().get_environmental_effects()

        # Enhance with real-life specific effects
        return {
            **base_effects,
            # GPS affected by dust and atmospheric conditions
            "gps_accuracy_factor": min(1.0, base_effects["gps_accuracy_factor"] * 0.9),
            # IMU affected by temperature and vibration
            "imu_noise_multiplier": base_effects["imu_noise_multiplier"] * 1.2,
            # Camera affected by dust and lighting
            "camera_visibility_factor": self.visibility
            * 0.85,  # Additional dust factor
            # LiDAR affected by dust scatter
            "lidar_range_factor": base_effects["lidar_range_factor"] * 0.9,
        }

    def get_performance_impact(self) -> Dict[str, Any]:
        """Get performance impact for real-life conditions.

        Returns:
            Dict with realistic performance impact
        """
        base_impact = super().get_performance_impact()

        # Real-life specific impacts
        return {
            **base_impact,
            "navigation_accuracy_degradation": 0.15,  # 15% accuracy loss
            "sensor_reliability_degradation": 0.2,  # 20% reliability loss
            "rover_performance_degradation": 0.25,  # 25% performance loss
            "safety_risk_increase": 0.3,  # 30% higher risk
            "overall_difficulty_score": 0.4,  # Moderate difficulty
        }
