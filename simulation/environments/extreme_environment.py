"""Extreme environment simulation.

Provides worst-case scenarios with severe environmental degradation.
Tests system robustness and failure handling under extreme conditions.

Author: URC 2026 Autonomy Team
"""

import math
from typing import Any, Dict

import numpy as np

from simulation.environments.base_environment import BaseEnvironment


class ExtremeEnvironment(BaseEnvironment):
    """Extreme environment with worst-case conditions.

    Severe environmental effects, extreme sensor degradation,
    challenging terrain conditions that test system limits.
    """

    def __init__(self, config: Dict[str, Any]):
        """Initialize extreme environment.

        Args:
            config: Environment configuration
        """
        # Set extreme defaults
        config.setdefault("tier", "extreme")
        config.setdefault("temperature", 50.0)  # Extreme heat (122°F)
        config.setdefault("humidity", 0.05)  # Extremely dry
        config.setdefault("wind_speed", 25.0)  # High wind (56 mph)
        config.setdefault("wind_direction", math.pi / 2)  # Crosswind
        config.setdefault("visibility", 0.2)  # 80% visibility loss (dust storm)
        config.setdefault("dust_density", 0.9)  # Severe dust storm
        config.setdefault("terrain_difficulty", 0.9)  # Extremely rough terrain
        config.setdefault("slope_angle", 0.5)  # 28-degree slope
        config.setdefault("traction", 0.4)  # Poor traction
        config.setdefault("surface_type", "loose_sand")  # Worst traction
        config.setdefault("lighting", 0.3)  # Poor lighting

        super().__init__(config)
        self._initialize_conditions()

    def _initialize_conditions(self):
        """Initialize extreme environmental conditions."""
        # Add extreme variations
        self.temperature += np.random.normal(0, 5.0)  # ±5°C variation
        self.wind_speed += np.random.normal(0, 5.0)  # ±5 m/s variation
        self.visibility *= np.random.uniform(0.8, 1.0)  # 20-100% of already poor visibility

        # Initialize extreme event timers
        self._next_dust_burst = np.random.uniform(10.0, 60.0)  # 10-60 seconds
        self._next_wind_gust = np.random.uniform(5.0, 30.0)  # 5-30 seconds
        self._next_terrain_shift = np.random.uniform(20.0, 120.0)  # 20-120 seconds

    def _update_weather(self, dt: float):
        """Update weather conditions with extreme patterns.

        Args:
            dt: Time step in seconds
        """
        # Extreme temperature fluctuations
        temp_variation = np.random.normal(0, 2.0)  # Large temperature swings
        self.temperature += temp_variation

        # Keep temperature in extreme range
        if self.temperature < 40.0:
            self.temperature = 40.0 + np.random.uniform(0, 10.0)
        elif self.temperature > 60.0:
            self.temperature = 60.0 - np.random.uniform(0, 10.0)

        # Extreme wind patterns
        self.simulation_time += dt

        # Frequent wind gusts
        if self.simulation_time >= self._next_wind_gust:
            gust_strength = np.random.uniform(10.0, 30.0)  # 22-67 mph gusts
            self.wind_speed = min(40.0, self.wind_speed + gust_strength)

            # Rapid wind direction changes
            self.wind_direction += np.random.uniform(-math.pi/2, math.pi/2)

            # Schedule next gust
            self._next_wind_gust = self.simulation_time + np.random.uniform(5.0, 20.0)

        # Gradual wind decrease after gust
        wind_decay = 0.1 * dt  # Lose 0.1 m/s per second
        self.wind_speed = max(15.0, self.wind_speed - wind_decay)

        # Extreme dust events
        if self.simulation_time >= self._next_dust_burst:
            # Dust bursts that temporarily worsen conditions
            dust_increase = np.random.uniform(0.1, 0.3)
            self.dust_density = min(1.0, self.dust_density + dust_increase)

            # Schedule dust burst end
            burst_duration = np.random.uniform(30.0, 120.0)  # 30-120 seconds
            self._dust_burst_end = self.simulation_time + burst_duration
            self._dust_burst_original = self.dust_density - dust_increase

            # Schedule next burst
            self._next_dust_burst = self.simulation_time + np.random.uniform(60.0, 300.0)

        # Handle dust burst end
        if hasattr(self, '_dust_burst_end') and self.simulation_time >= self._dust_burst_end:
            # Gradually reduce dust
            dust_reduction = 0.02 * dt
            self.dust_density = max(self._dust_burst_original, self.dust_density - dust_reduction)

        # Extreme humidity variations (very low but can spike)
        humidity_spike = np.random.random()
        if humidity_spike < 0.001:  # 0.1% chance
            self.humidity = np.random.uniform(0.1, 0.3)  # Temporary moisture
            self._humidity_spike_end = self.simulation_time + np.random.uniform(10.0, 60.0)
        elif hasattr(self, '_humidity_spike_end') and self.simulation_time >= self._humidity_spike_end:
            self.humidity = 0.05  # Back to extreme dryness

        # Visibility affected by multiple factors
        dust_factor = 1.0 - self.dust_density * 0.9  # Dust heavily affects visibility
        wind_factor = max(0.1, 1.0 - self.wind_speed * 0.03)  # Wind kicks up dust
        humidity_factor = 1.0 - self.humidity * 0.2  # Moisture can help or hurt
        lighting_factor = 0.5 + self.lighting * 0.5  # Poor lighting

        self.visibility = min(1.0, dust_factor * wind_factor * humidity_factor * lighting_factor)
        self.visibility = max(0.05, self.visibility)  # Never completely blind

    def _update_terrain(self, dt: float):
        """Update terrain conditions with extreme changes.

        Args:
            dt: Time step in seconds
        """
        # Extreme terrain shifts
        if self.simulation_time >= self._next_terrain_shift:
            # Sudden terrain difficulty changes
            difficulty_change = np.random.uniform(-0.3, 0.3)
            self.terrain_difficulty += difficulty_change
            self.terrain_difficulty = np.clip(self.terrain_difficulty, 0.5, 1.0)  # Always difficult

            # Slope changes
            slope_change = np.random.uniform(-0.2, 0.2)
            self.slope_angle += slope_change
            self.slope_angle = np.clip(self.slope_angle, -0.8, 0.8)  # ±45 degrees

            # Traction affected by terrain
            if self.terrain_difficulty > 0.8:
                self.traction = max(0.2, self.traction - 0.1)  # Very poor traction
            else:
                self.traction = min(0.6, self.traction + 0.05)  # Slightly better

            # Schedule next shift
            self._next_terrain_shift = self.simulation_time + np.random.uniform(30.0, 120.0)

        # Continuous traction degradation
        if np.random.random() < 0.01:  # 1% chance per second
            # Wheels dig in, traction worsens
            traction_loss = np.random.uniform(0.05, 0.15)
            self.traction = max(0.1, self.traction - traction_loss)

            # Recovery takes time
            self._traction_recovery_time = self.simulation_time + np.random.uniform(60.0, 300.0)

        # Traction recovery
        if hasattr(self, '_traction_recovery_time') and self.simulation_time >= self._traction_recovery_time:
            traction_gain = 0.01 * dt
            self.traction = min(0.5, self.traction + traction_gain)  # Limited recovery

        # Surface type changes (extreme conditions cause surface breakdown)
        if np.random.random() < 0.005:  # 0.5% chance
            extreme_surfaces = ["loose_sand", "mud", "ice", "rock_slide"]
            self.surface_type = np.random.choice(extreme_surfaces)
            self.logger.warning(f"Extreme surface change: {self.surface_type}")

    def get_environmental_effects(self) -> Dict[str, Any]:
        """Get environmental effects for extreme conditions.

        Returns:
            Dict with severe environmental effects
        """
        base_effects = super().get_environmental_effects()

        # Extreme conditions cause severe degradation
        return {
            **base_effects,
            # GPS nearly unusable
            "gps_accuracy_factor": base_effects["gps_accuracy_factor"] * 0.3,  # 70% degradation
            # IMU heavily affected
            "imu_noise_multiplier": base_effects["imu_noise_multiplier"] * 3.0,  # 3x noise
            # Camera severely impaired
            "camera_visibility_factor": self.visibility * 0.4,  # Additional impairment
            # LiDAR very limited range
            "lidar_range_factor": base_effects["lidar_range_factor"] * 0.5,  # 50% range loss
            # Rover struggles
            "traction_multiplier": self.traction * 0.7,  # Additional traction loss
            "rolling_resistance_factor": base_effects["rolling_resistance_factor"] * 1.5,
            "wind_resistance_factor": base_effects["wind_resistance_factor"] * 2.0,
            # High safety risk
            "collision_risk_factor": min(0.9, base_effects["collision_risk_factor"] * 2.0),
            "thermal_stress_factor": 0.8,  # Severe thermal stress
        }

    def get_performance_impact(self) -> Dict[str, Any]:
        """Get performance impact for extreme conditions.

        Returns:
            Dict with severe performance impact
        """
        base_impact = super().get_performance_impact()

        # Extreme conditions cause massive degradation
        return {
            **base_impact,
            "navigation_accuracy_degradation": 0.7,  # 70% accuracy loss
            "sensor_reliability_degradation": 0.6,  # 60% reliability loss
            "rover_performance_degradation": 0.75,  # 75% performance loss
            "safety_risk_increase": 0.8,  # 80% higher risk
            "overall_difficulty_score": 0.9,  # Extreme difficulty
        }

    def trigger_extreme_event(self, event_type: str):
        """Trigger a specific extreme environmental event.

        Args:
            event_type: Type of extreme event
        """
        if event_type == "dust_storm":
            self._trigger_severe_dust_storm()
        elif event_type == "wind_shear":
            self._trigger_wind_shear()
        elif event_type == "terrain_collapse":
            self._trigger_terrain_collapse()
        elif event_type == "thermal_extreme":
            self._trigger_thermal_extreme()
        else:
            self.logger.warning(f"Unknown extreme event type: {event_type}")

    def _trigger_severe_dust_storm(self):
        """Trigger a severe dust storm."""
        self.logger.critical("SEVERE DUST STORM triggered!")

        self.dust_density = 1.0  # Maximum dust
        self.visibility = 0.05  # Near zero visibility
        self.wind_speed = min(50.0, self.wind_speed + 20.0)

        # Long duration storm
        self._storm_end_time = self.simulation_time + np.random.uniform(600.0, 1800.0)  # 10-30 minutes

    def _trigger_wind_shear(self):
        """Trigger wind shear event."""
        self.logger.critical("WIND SHEAR event triggered!")

        # Rapid wind changes
        self.wind_speed = np.random.uniform(30.0, 50.0)
        self.wind_direction += np.random.uniform(-math.pi, math.pi)

        # Affects for several minutes
        self._wind_shear_end = self.simulation_time + np.random.uniform(180.0, 600.0)

    def _trigger_terrain_collapse(self):
        """Trigger terrain collapse event."""
        self.logger.critical("TERRAIN COLLAPSE event triggered!")

        # Sudden terrain degradation
        self.terrain_difficulty = 1.0
        self.traction = 0.1  # Nearly no traction
        self.slope_angle += np.random.uniform(0.2, 0.4)

        # Recovery takes time
        self._terrain_recovery_time = self.simulation_time + np.random.uniform(300.0, 900.0)

    def _trigger_thermal_extreme(self):
        """Trigger thermal extreme event."""
        self.logger.critical("THERMAL EXTREME event triggered!")

        # Extreme temperature spike or drop
        temp_change = np.random.choice([-30.0, 30.0])  # Sudden cold or heat
        self.temperature += temp_change

        # Duration of extreme thermal event
        self._thermal_event_end = self.simulation_time + np.random.uniform(300.0, 1200.0)
