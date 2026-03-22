"""MDRS-specific environmental stressors for URC competition simulation.

Implements realistic environmental conditions specific to the Mars Desert
Research Station in Hanksville, Utah, including temperature cycling,
dust storms, terrain challenges, and communication obstacles.

Author: URC 2026 Autonomy Team
"""

import logging
import math
import random
from dataclasses import dataclass
from enum import Enum
from typing import Any, Dict, List, Optional, Tuple

import numpy as np

from simulation.environments.base_environment import BaseEnvironment


class TerrainType(Enum):
    """MDRS terrain types based on local geology."""

    MANCOS_SHALES = "mancos_shales"
    MORRISON_FORMATION = "morrison_formation"
    ALLUVIAL_FAN = "alluvial_fan"
    DESERT_PAVEMENT = "desert_pavement"
    SANDY_FLATS = "sandy_flats"
    ROCKY_OUTCROP = "rocky_outcrop"


class WeatherCondition(Enum):
    """MDRS weather conditions."""

    CLEAR_SKIES = "clear_skies"
    PARTLY_CLOUDY = "partly_cloudy"
    DUST_STORM = "dust_storm"
    HIGH_WINDS = "high_winds"
    HOT_DAY = "hot_day"
    COLD_NIGHT = "cold_night"


@dataclass
class TerrainProperties:
    """Properties for different terrain types."""

    traction_coefficient: float
    rolling_resistance: float
    obstacle_density: float  # 0-1
    visibility_factor: float  # 0-1
    communication_attenuation: float  # dB
    slope_variance: float  # radians
    description: str


# MDRS terrain properties based on actual site characteristics
MDRS_TERRAIN_PROPERTIES: Dict[TerrainType, TerrainProperties] = {
    TerrainType.MANCOS_SHALES: TerrainProperties(
        traction_coefficient=0.4,
        rolling_resistance=0.08,
        obstacle_density=0.3,
        visibility_factor=0.8,
        communication_attenuation=3.0,
        slope_variance=0.15,
        description="Soft, clay-rich shale formations, slippery when wet",
    ),
    TerrainType.MORRISON_FORMATION: TerrainProperties(
        traction_coefficient=0.6,
        rolling_resistance=0.06,
        obstacle_density=0.5,
        visibility_factor=0.7,
        communication_attenuation=5.0,
        slope_variance=0.25,
        description="Mixed sedimentary rock formations with moderate obstacles",
    ),
    TerrainType.ALLUVIAL_FAN: TerrainProperties(
        traction_coefficient=0.5,
        rolling_resistance=0.12,
        obstacle_density=0.2,
        visibility_factor=0.9,
        communication_attenuation=2.0,
        slope_variance=0.10,
        description="Sandy, gravelly deposits from water erosion",
    ),
    TerrainType.DESERT_PAVEMENT: TerrainProperties(
        traction_coefficient=0.7,
        rolling_resistance=0.04,
        obstacle_density=0.1,
        visibility_factor=1.0,
        communication_attenuation=1.0,
        slope_variance=0.05,
        description="Compact desert surface, relatively smooth",
    ),
    TerrainType.SANDY_FLATS: TerrainProperties(
        traction_coefficient=0.3,
        rolling_resistance=0.15,
        obstacle_density=0.1,
        visibility_factor=0.6,
        communication_attenuation=2.5,
        slope_variance=0.02,
        description="Fine sand areas, challenging traction",
    ),
    TerrainType.ROCKY_OUTCROP: TerrainProperties(
        traction_coefficient=0.8,
        rolling_resistance=0.05,
        obstacle_density=0.8,
        visibility_factor=0.5,
        communication_attenuation=8.0,
        slope_variance=0.40,
        description="Dense rock formations, high obstacles",
    ),
}


@dataclass
class EnvironmentalStressor:
    """Environmental stressor definition."""

    stressor_type: str
    severity: float  # 0-1
    duration_minutes: int
    effects: Dict[str, float]
    recovery_time_minutes: int = 0


class MDRSEnvironment(BaseEnvironment):
    """MDRS-specific environment simulation for URC competition.

    Implements realistic environmental conditions based on actual MDRS
    location characteristics and competition day observations.
    """

    def __init__(self, config: Dict[str, Any]):
        """Initialize MDR environment."""
        super().__init__(config)

        self.location_name = "Mars Desert Research Station"
        self.latitude = 38.4070  # Actual MDRS coordinates
        self.longitude = -110.7920
        self.elevation_m = 1400  # Meters above sea level

        # MDRS-specific environmental parameters
        self.base_temperature_c = config.get("base_temperature", 20.0)
        self.daily_temp_range_c = config.get(
            "daily_temp_range", 25.0
        )  # Day-night variation
        self.dust_storm_probability = config.get("dust_storm_probability", 0.15)
        self.wind_gust_probability = config.get("wind_gust_probability", 0.25)

        # Terrain configuration
        self.terrain_types: List[TerrainType] = []
        self.current_terrain = TerrainType.DESERT_PAVEMENT
        self.terrain_transition_probability = config.get(
            "terrain_transition_prob", 0.02
        )

        # Metal Hab communication challenges
        self.operating_from_hab = config.get("operating_from_hab", False)
        self.antenna_cable_length_m = config.get("antenna_cable_length", 25.0)
        self.metal_attenuation_db = config.get("metal_attenuation_db", 20.0)

        # Environmental stressors
        self.active_stressors: List[EnvironmentalStressor] = []
        self.stressor_history: List[Dict[str, Any]] = []

        # Time-based environmental cycles
        self.time_of_day_hours = config.get("time_of_day", 12.0)  # 24-hour format
        self.day_length_hours = 24.0
        self.seasonal_factor = config.get(
            "seasonal_factor", 0.0
        )  # -1 (winter) to 1 (summer)

        self._initialize_mdrs_conditions()

    def _initialize_conditions(self):
        """Initialize MDRS-specific environmental conditions (abstract method implementation)."""
        self._initialize_mdrs_conditions()

    def _initialize_mdrs_conditions(self):
        """Initialize MDRS-specific environmental conditions."""
        # Set initial terrain distribution based on MDRS geology
        self.terrain_types = [
            TerrainType.DESERT_PAVEMENT,
            TerrainType.MANCOS_SHALES,
            TerrainType.MORRISON_FORMATION,
            TerrainType.ALLUVIAL_FAN,
            TerrainType.SANDY_FLATS,
            TerrainType.ROCKY_OUTCROP,
        ]

        # Set realistic initial conditions for MDRS
        self.surface_type = "desert_pavement"
        self.terrain_difficulty = 0.3  # Moderate difficulty baseline
        self.traction = MDRS_TERRAIN_PROPERTIES[
            TerrainType.DESERT_PAVEMENT
        ].traction_coefficient

        # Initialize weather based on time of day
        self._update_temperature_cycle()

        self.logger.info(f"Initialized MDRS environment at {self.location_name}")
        self.logger.info(
            f"Elevation: {self.elevation_m}m, Coordinates: {self.latitude:.4f}, {self.longitude:.4f}"
        )

    def _update_weather(self, dt: float):
        """Update MDRS weather conditions over time."""
        # Update time of day
        hours_passed = dt / 3600.0
        self.time_of_day_hours = (
            self.time_of_day_hours + hours_passed
        ) % self.day_length_hours

        # Time-based temperature variation
        self._update_temperature_cycle()

        # Random weather events
        self._update_dust_conditions(dt)
        self._update_wind_conditions(dt)

        # Update visibility based on conditions
        self._update_visibility()

        # Seasonal variations
        self._apply_seasonal_effects()

    def _update_terrain(self, dt: float):
        """Update terrain conditions based on rover movement."""
        # Simulate terrain transitions as rover moves
        if random.random() < self.terrain_transition_probability:
            self._transition_to_new_terrain()

        # Update terrain properties based on current type
        current_props = MDRS_TERRAIN_PROPERTIES[self.current_terrain]
        self.traction = current_props.traction_coefficient
        self.terrain_difficulty = current_props.obstacle_density
        self.slope_angle += random.gauss(0, current_props.slope_variance * 0.1)

        # Apply temporary terrain effects from stressors
        self._apply_terrain_stressors(dt)

    def _update_temperature_cycle(self):
        """Update temperature based on time of day."""
        # Sinusoidal temperature variation
        time_factor = math.cos(
            2 * math.pi * (self.time_of_day_hours - 14) / self.day_length_hours
        )

        # Base temperature with daily variation and seasonal adjustment
        self.temperature = (
            self.base_temperature_c
            + (self.daily_temp_range_c / 2) * time_factor
            + 10 * self.seasonal_factor
        )

        # Add random variations
        self.temperature += random.gauss(0, 1.0)

    def _update_dust_conditions(self, dt: float):
        """Update dust conditions and potential dust storms."""
        # Check for dust storm initiation
        if random.random() < self.dust_storm_probability * dt / 3600.0:
            self._initiate_dust_storm()

        # Gradual dust settling
        if self.dust_density > 0.01:
            self.dust_density *= 0.995  # Slow settling
        else:
            self.dust_density = max(0.0, self.dust_density - 0.001 * dt / 60.0)

    def _update_wind_conditions(self, dt: float):
        """Update wind conditions including gusts."""
        # Base wind with variations
        self.wind_speed += random.gauss(0, 0.5 * dt / 60.0)
        self.wind_speed = max(0, min(25, self.wind_speed))  # Cap at 25 m/s

        # Wind gusts
        if random.random() < self.wind_gust_probability * dt / 60.0:
            gust_strength = random.uniform(5, 15)
            self.wind_speed += gust_strength
            self.logger.info(f"Wind gust: +{gust_strength:.1f} m/s")

        # Wind direction changes
        self.wind_direction += random.gauss(0, 0.1 * dt / 60.0)
        self.wind_direction = self.wind_direction % (2 * math.pi)

    def _update_visibility(self):
        """Update visibility based on environmental conditions."""
        base_visibility = 1.0

        # Dust effects
        dust_factor = 1.0 - self.dust_density * 0.8

        # Wind effects (dust kicked up)
        wind_factor = 1.0 - min(0.3, self.wind_speed / 50.0)

        # Time of day effects
        if not (6 <= self.time_of_day_hours <= 18):  # Nighttime
            time_factor = 0.3
        elif self.time_of_day_hours < 8 or self.time_of_day_hours > 17:  # Dawn/dusk
            time_factor = 0.7
        else:  # Daylight
            time_factor = 1.0

        self.visibility = base_visibility * dust_factor * wind_factor * time_factor
        self.visibility = max(0.05, min(1.0, self.visibility))

    def _apply_seasonal_effects(self):
        """Apply seasonal environmental effects."""
        # Temperature adjustment
        seasonal_temp = 15 * self.seasonal_factor
        self.temperature += seasonal_temp

        # Dust probability (higher in summer)
        seasonal_dust_prob = self.dust_storm_probability * (1 + self.seasonal_factor)
        if random.random() < seasonal_dust_prob / 100:
            self.dust_density = min(0.8, self.dust_density + 0.1)

    def _transition_to_new_terrain(self):
        """Transition to a new terrain type."""
        old_terrain = self.current_terrain
        self.current_terrain = random.choice(self.terrain_types)

        if old_terrain != self.current_terrain:
            self.logger.info(
                f"Terrain transition: {old_terrain.value} -> {self.current_terrain.value}"
            )

            # Update surface type for compatibility
            self.surface_type = self.current_terrain.value

    def _initiate_dust_storm(self):
        """Initiate a dust storm event."""
        storm_severity = random.uniform(0.3, 0.9)
        storm_duration = int(random.uniform(15, 120))  # 15-120 minutes

        dust_storm = EnvironmentalStressor(
            stressor_type="dust_storm",
            severity=storm_severity,
            duration_minutes=int(storm_duration),
            effects={
                "dust_density": storm_severity * 0.8,
                "visibility_reduction": storm_severity * 0.7,
                "communication_attenuation": storm_severity * 10.0,
                "sensor_noise": storm_severity * 2.0,
            },
            recovery_time_minutes=30,
        )

        self.active_stressors.append(dust_storm)
        self.logger.warning(
            f"Dust storm initiated: severity={storm_severity:.2f}, duration={storm_duration}min"
        )

    def _apply_terrain_stressors(self, dt: float):
        """Apply active terrain-related stressors."""
        for stressor in self.active_stressors[:]:
            stressor.duration_minutes -= dt / 60.0

            if stressor.duration_minutes <= 0:
                # Stressor ended, start recovery
                self.active_stressors.remove(stressor)
                self.logger.info(f"Stressor ended: {stressor.stressor_type}")

            else:
                # Apply stressor effects
                if stressor.stressor_type == "dust_storm":
                    self.dust_density = min(
                        0.9,
                        self.dust_density
                        + stressor.effects["dust_density"] * dt / 60.0,
                    )

    def get_environmental_effects(self) -> Dict[str, Any]:
        """Get environmental effects with MDRS-specific modifications."""
        base_effects = super().get_environmental_effects()

        # Apply metal Hab communication attenuation
        if self.operating_from_hab:
            base_effects["communication_attenuation_db"] = self.metal_attenuation_db + (
                self.antenna_cable_length_m * 0.2  # Cable loss
            )

        # Apply terrain-specific effects
        terrain_props = MDRS_TERRAIN_PROPERTIES[self.current_terrain]
        base_effects["terrain_traction_factor"] = terrain_props.traction_coefficient
        base_effects["terrain_obstacle_factor"] = terrain_props.obstacle_density
        base_effects["terrain_visibility_factor"] = terrain_props.visibility_factor

        # Apply active stressor effects
        for stressor in self.active_stressors:
            for effect_name, effect_value in stressor.effects.items():
                if effect_name in base_effects:
                    base_effects[effect_name] = base_effects[effect_name] * (
                        1 + effect_value
                    )

        # Add MDRS-specific effects
        base_effects.update(
            {
                "mdrs_elevation_factor": self.elevation_m
                / 1000.0,  # Air density effects
                "time_of_day_factor": math.sin(
                    2 * math.pi * self.time_of_day_hours / self.day_length_hours
                ),
                "seasonal_factor": self.seasonal_factor,
                "terrain_type": self.current_terrain.value,
                "active_stressors": len(self.active_stressors),
            }
        )

        return base_effects

    def set_operating_from_hab(self, operating_from_hab: bool):
        """Set whether operating from metal Hab (affects communication)."""
        self.operating_from_hab = operating_from_hab
        self.logger.info(f"Operating from Hab: {operating_from_hab}")

        if operating_from_hab:
            self.logger.warning(
                f"Metal Hab attenuation: {self.metal_attenuation_db}dB + cable losses"
            )

    def get_competition_day_scenario(self, scenario_type: str) -> Dict[str, Any]:
        """Get predefined competition day scenarios."""
        scenarios = {
            "perfect_day": {
                "description": "Ideal competition conditions",
                "weather": {
                    "temperature": 22.0,
                    "humidity": 0.3,
                    "wind_speed": 3.0,
                    "visibility": 1.0,
                    "dust_density": 0.0,
                },
                "terrain_type": TerrainType.DESERT_PAVEMENT,
                "stressors": [],
            },
            "moderate_challenges": {
                "description": "Typical URC competition day",
                "weather": {
                    "temperature": 28.0,
                    "humidity": 0.4,
                    "wind_speed": 8.0,
                    "visibility": 0.8,
                    "dust_density": 0.2,
                },
                "terrain_type": TerrainType.MORRISON_FORMATION,
                "stressors": [
                    EnvironmentalStressor(
                        stressor_type="wind_gusts",
                        severity=0.4,
                        duration_minutes=20,
                        effects={"wind_speed": 5.0, "communication_attenuation": 2.0},
                    )
                ],
            },
            "challenging_day": {
                "description": "Difficult conditions testing system limits",
                "weather": {
                    "temperature": 35.0,
                    "humidity": 0.2,
                    "wind_speed": 15.0,
                    "visibility": 0.5,
                    "dust_density": 0.4,
                },
                "terrain_type": TerrainType.MANCOS_SHALES,
                "stressors": [
                    EnvironmentalStressor(
                        stressor_type="dust_storm",
                        severity=0.6,
                        duration_minutes=45,
                        effects={"dust_density": 0.5, "visibility_reduction": 0.6},
                    ),
                    EnvironmentalStressor(
                        stressor_type="thermal_stress",
                        severity=0.7,
                        duration_minutes=60,
                        effects={"thermal_noise": 1.5, "battery_drain": 1.2},
                    ),
                ],
            },
            "worst_case": {
                "description": "Extreme conditions for robustness testing",
                "weather": {
                    "temperature": 40.0,
                    "humidity": 0.1,
                    "wind_speed": 20.0,
                    "visibility": 0.3,
                    "dust_density": 0.7,
                },
                "terrain_type": TerrainType.ROCKY_OUTCROP,
                "stressors": [
                    EnvironmentalStressor(
                        stressor_type="severe_dust_storm",
                        severity=0.9,
                        duration_minutes=90,
                        effects={"dust_density": 0.8, "visibility_reduction": 0.8},
                    ),
                    EnvironmentalStressor(
                        stressor_type="equipment_failure",
                        severity=0.5,
                        duration_minutes=30,
                        effects={"sensor_reliability": 0.7},
                    ),
                ],
            },
        }

        return scenarios.get(scenario_type, scenarios["moderate_challenges"])

    def apply_scenario(self, scenario_type: str):
        """Apply a predefined competition day scenario."""
        scenario = self.get_competition_day_scenario(scenario_type)

        self.logger.info(f"Applying scenario: {scenario['description']}")

        # Apply weather conditions
        for param, value in scenario["weather"].items():
            setattr(self, param, value)

        # Set terrain
        self.current_terrain = scenario["terrain_type"]
        self.surface_type = self.terrain.value

        # Apply stressors
        self.active_stressors = scenario["stressors"].copy()

        self.logger.info(f"Scenario applied: {scenario_type}")

    def get_stressor_summary(self) -> Dict[str, Any]:
        """Get summary of current environmental stressors."""
        return {
            "active_stressors": [
                {
                    "type": s.stressor_type,
                    "severity": s.severity,
                    "remaining_minutes": s.duration_minutes,
                    "effects": s.effects,
                }
                for s in self.active_stressors
            ],
            "terrain_difficulty": self.terrain_difficulty,
            "visibility": self.visibility,
            "dust_density": self.dust_density,
            "operating_from_hab": self.operating_from_hab,
            "communication_attenuation_db": (
                self.metal_attenuation_db + (self.antenna_cable_length_m * 0.2)
                if self.operating_from_hab
                else 0.0
            ),
        }


if __name__ == "__main__":
    # Test MDRS environment
    print("[MDRS] Testing MDRS Environment Simulation")
    print("=" * 60)

    config = {
        "tier": "mdrs_competition",
        "base_temperature": 20.0,
        "daily_temp_range": 25.0,
        "dust_storm_probability": 0.15,
        "time_of_day": 10.0,
    }

    mdrs_env = MDRSEnvironment(config)

    # Test scenarios
    scenarios = ["perfect_day", "moderate_challenges", "challenging_day", "worst_case"]

    for scenario in scenarios:
        print(f"\n[SCENARIO] {scenario.upper()}:")
        mdrs_env.apply_scenario(scenario)

        # Run simulation for a few steps
        for i in range(5):
            state = mdrs_env.step(1.0)
            if i == 0:
                effects = mdrs_env.get_environmental_effects()
                print(f"  Temperature: {effects['thermal_stress_factor']:.2f}")
                print(f"  Visibility: {mdrs_env.visibility:.2f}")
                print(f"  Dust: {mdrs_env.dust_density:.2f}")
                print(f"  Terrain: {mdrs_env.current_terrain.value}")

        stressors = mdrs_env.get_stressor_summary()
        print(f"  Active stressors: {len(stressors['active_stressors'])}")

    # Test metal Hab operation
    print(f"\n[HAB] Testing Metal Hab Operation:")
    mdrs_env.set_operating_from_hab(True)
    effects = mdrs_env.get_environmental_effects()
    print(
        f"  Communication attenuation: {effects.get('communication_attenuation_db', 0):.1f} dB"
    )

    print("\n[PASS] MDRS environment test complete")
