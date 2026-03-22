"""GPS sensor simulator.

Provides realistic GPS data simulation with configurable noise,
accuracy, and failure modes for testing navigation systems.

Author: URC 2026 Autonomy Team
"""

import random
from typing import Any, Dict

import numpy as np

from simulation.sensors.base_sensor import BaseSensor


class GPSSimulator(BaseSensor):
    """GPS sensor simulator with realistic behavior.

    Simulates GPS positioning with:
    - Configurable accuracy and noise
    - Satellite visibility simulation
    - GPS-denied scenarios
    - Multipath effects
    - Update rate management
    """

    def __init__(self, config: Dict[str, Any]):
        """Initialize GPS simulator.

        Args:
            config: GPS configuration
        """
        super().__init__(config)

        # GPS-specific configuration
        self.base_position = config.get("base_position", [38.406, -110.792, 1500.0])
        self.position_noise_std = config.get("position_noise_std", 2.5)  # meters
        self.velocity_noise_std = config.get("velocity_noise_std", 0.1)  # m/s
        self.heading_noise_std = config.get("heading_noise_std", 0.5)  # degrees

        # Satellite simulation
        self.min_satellites = config.get("min_satellites", 4)
        self.max_satellites = config.get("max_satellites", 12)
        self.satellites_visible = self.max_satellites

        # GPS-specific state
        self.current_position = self.base_position.copy()
        self.current_velocity = [0.0, 0.0, 0.0]
        self.current_heading = 0.0
        self.fix_quality = 2  # RTK fixed
        self.hdop = 0.8  # Horizontal dilution of precision

        # Data bounds for validation
        self.data_bounds = {
            "latitude": {"min": -90.0, "max": 90.0},
            "longitude": {"min": -180.0, "max": 180.0},
            "altitude": {"min": -100.0, "max": 10000.0},
            "satellites_visible": {"min": 0, "max": 20},
            "hdop": {"min": 0.5, "max": 50.0},
        }

    def _generate_raw_data(self, environment_state: Dict[str, Any]) -> Dict[str, Any]:
        """Generate raw GPS data.

        Args:
            environment_state: Current environment conditions

        Returns:
            Dict containing raw GPS measurements
        """
        # Simulate movement (very slow drift for testing)
        drift = np.random.normal(0, 0.01, 3)  # Small random walk
        self.current_position = [
            pos + drift[i] for i, pos in enumerate(self.current_position)
        ]

        # Simulate velocity (mostly zero with small noise)
        self.current_velocity = np.random.normal(0, 0.05, 3).tolist()

        # Simulate heading changes
        heading_change = np.random.normal(0, 0.1)
        self.current_heading = (self.current_heading + heading_change) % 360

        # Simulate satellite visibility
        self.satellites_visible = random.randint(
            self.min_satellites, self.max_satellites
        )

        # Determine fix quality based on satellites
        if self.satellites_visible >= 8:
            self.fix_quality = 2  # RTK fixed
            self.hdop = random.uniform(0.5, 1.0)
        elif self.satellites_visible >= 4:
            self.fix_quality = 1  # GPS fixed
            self.hdop = random.uniform(1.0, 2.5)
        else:
            self.fix_quality = 0  # No fix
            self.hdop = 50.0

        return {
            "latitude": self.current_position[0],
            "longitude": self.current_position[1],
            "altitude": self.current_position[2],
            "velocity_north": self.current_velocity[0],
            "velocity_east": self.current_velocity[1],
            "velocity_down": self.current_velocity[2],
            "heading": self.current_heading,
            "satellites_visible": self.satellites_visible,
            "fix_quality": self.fix_quality,
            "hdop": self.hdop,
            "vdop": random.uniform(0.8, 2.0),  # Vertical DOP
            "pdop": random.uniform(1.0, 3.0),  # Position DOP
        }

    def _apply_environmental_effects(
        self, data: Dict[str, Any], environment: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Apply environmental effects to GPS data.

        Args:
            data: Raw GPS data
            environment: Environment conditions

        Returns:
            Dict with environmental effects applied
        """
        modified_data = data.copy()

        # GPS signal degradation from dust
        dust_density = environment.get("dust_density", 0.0)
        visibility = environment.get("visibility", 1.0)

        # Dust reduces satellite visibility
        if dust_density > 0.5:
            reduction = int(dust_density * 4)  # Up to 4 satellites lost
            modified_data["satellites_visible"] = max(
                0, data["satellites_visible"] - reduction
            )

        # Poor visibility affects accuracy
        if visibility < 0.7:
            accuracy_degradation = (1.0 - visibility) * 2.0
            modified_data["hdop"] = data["hdop"] * (1.0 + accuracy_degradation)
            modified_data["position_accuracy"] = data.get("position_accuracy", 2.5) * (
                1.0 + accuracy_degradation
            )

        # Wind can affect GPS signal (multipath)
        wind_speed = environment.get("wind_speed", 0.0)
        if wind_speed > 10.0:  # High wind
            # Add position noise from wind-induced multipath
            wind_noise = np.random.normal(0, wind_speed * 0.01, 3)
            modified_data["latitude"] += wind_noise[0] / 111000.0  # Convert to degrees
            modified_data["longitude"] += wind_noise[1] / 111000.0
            modified_data["hdop"] *= 1.2

        return modified_data

    def _add_noise(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Add GPS-specific noise to data.

        Args:
            data: Clean GPS data

        Returns:
            Dict with noise added
        """
        noisy_data = data.copy()

        # Position noise (Gaussian)
        pos_noise = np.random.normal(0, self.position_noise_std, 3)
        noisy_data["latitude"] += (
            pos_noise[0] / 111000.0
        )  # Convert meters to degrees lat
        noisy_data["longitude"] += pos_noise[1] / (
            111000.0 * np.cos(np.radians(data["latitude"]))
        )
        noisy_data["altitude"] += pos_noise[2]

        # Velocity noise
        vel_noise = np.random.normal(0, self.velocity_noise_std, 3)
        noisy_data["velocity_north"] += vel_noise[0]
        noisy_data["velocity_east"] += vel_noise[1]
        noisy_data["velocity_down"] += vel_noise[2]

        # Heading noise
        heading_noise = np.random.normal(0, self.heading_noise_std)
        noisy_data["heading"] = (data["heading"] + heading_noise) % 360

        # HDOP noise
        hdop_noise = np.random.normal(0, 0.1)
        noisy_data["hdop"] = max(0.5, data["hdop"] + hdop_noise)

        return noisy_data

    def _calculate_data_quality(self, data: Dict[str, Any]) -> float:
        """Calculate GPS data quality score.

        Args:
            data: GPS data

        Returns:
            float: Quality score (0.0 to 1.0)
        """
        if not data.get("_is_valid", True):
            return 0.0

        score = 1.0

        # Reduce score based on HDOP (lower HDOP = better)
        hdop = data.get("hdop", 50.0)
        if hdop > 2.0:
            score *= max(0.1, 2.0 / hdop)  # Penalize poor HDOP

        # Reduce score based on satellite count
        satellites = data.get("satellites_visible", 0)
        if satellites < 4:
            score *= 0.1  # Very poor with < 4 satellites
        elif satellites < 6:
            score *= 0.5  # Poor with 4-5 satellites
        elif satellites < 8:
            score *= 0.8  # Good with 6-7 satellites

        # Reduce score based on fix quality
        fix_quality = data.get("fix_quality", 0)
        if fix_quality == 0:
            score *= 0.1  # No fix
        elif fix_quality == 1:
            score *= 0.7  # GPS fix only

        return max(0.0, min(1.0, score))

    def simulate_gps_denied(self, duration_sec: float = 30.0):
        """Simulate GPS signal loss (GPS-denied scenario).

        Args:
            duration_sec: Duration of GPS denial in seconds
        """
        self.logger.info(f"Simulating GPS denial for {duration_sec}s")

        # Reduce satellites to simulate loss
        original_satellites = self.satellites_visible
        self.satellites_visible = random.randint(0, 2)  # 0-2 satellites

        # Schedule recovery
        def recover_gps():
            import time

            time.sleep(duration_sec)
            self.satellites_visible = original_satellites
            self.fix_quality = 2
            self.logger.info("GPS signal recovered")

        import threading

        recovery_thread = threading.Thread(target=recover_gps, daemon=True)
        recovery_thread.start()

    def set_position(self, latitude: float, longitude: float, altitude: float):
        """Set GPS position (for testing).

        Args:
            latitude: Latitude in degrees
            longitude: Longitude in degrees
            altitude: Altitude in meters
        """
        self.current_position = [latitude, longitude, altitude]

    def get_ground_truth(self) -> Dict[str, float]:
        """Get ground truth position (no noise).

        Returns:
            Dict with true position
        """
        return {
            "latitude": self.base_position[0],
            "longitude": self.base_position[1],
            "altitude": self.base_position[2],
            "is_ground_truth": True,
        }
