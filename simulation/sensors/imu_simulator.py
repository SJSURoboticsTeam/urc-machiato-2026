"""IMU sensor simulator.

Provides realistic IMU data simulation with configurable noise,
bias drift, and environmental effects for testing navigation systems.

Author: URC 2026 Autonomy Team
"""

import random
from typing import Any, Dict

import numpy as np

from simulation.sensors.base_sensor import BaseSensor


class IMUSimulator(BaseSensor):
    """IMU sensor simulator with realistic behavior.

    Simulates inertial measurement unit with:
    - Gyroscope and accelerometer data
    - Configurable bias and noise
    - Temperature effects
    - Vibration simulation
    - Calibration drift
    """

    def __init__(self, config: Dict[str, Any]):
        """Initialize IMU simulator.

        Args:
            config: IMU configuration
        """
        super().__init__(config)

        # IMU-specific configuration
        self.gyro_bias_drift = config.get("gyro_bias_drift", 0.01)  # deg/s
        self.accel_bias_drift = config.get("accel_bias_drift", 0.001)  # m/s²
        self.gyro_noise_std = config.get("gyro_noise_std", 0.01)  # deg/s
        self.accel_noise_std = config.get("accel_noise_std", 0.1)  # m/s²
        self.temperature_coefficient = config.get("temperature_coefficient", 0.001)

        # State tracking
        self.current_orientation = [1.0, 0.0, 0.0, 0.0]  # Quaternion (w, x, y, z)
        self.angular_velocity = [0.0, 0.0, 0.0]  # rad/s
        self.linear_acceleration = [0.0, 0.0, 9.81]  # m/s² (gravity)
        self.temperature = 25.0  # Celsius

        # Bias tracking (accumulates over time)
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.accel_bias = [0.0, 0.0, 0.0]

        # Calibration state
        self.is_calibrated = True
        self.calibration_quality = 1.0  # 0.0 to 1.0

        # Data bounds for validation
        self.data_bounds = {
            "angular_velocity_x": {"min": -10.0, "max": 10.0},  # rad/s
            "angular_velocity_y": {"min": -10.0, "max": 10.0},
            "angular_velocity_z": {"min": -10.0, "max": 10.0},
            "linear_acceleration_x": {"min": -20.0, "max": 20.0},  # m/s²
            "linear_acceleration_y": {"min": -20.0, "max": 20.0},
            "linear_acceleration_z": {"min": -20.0, "max": 20.0},
            "temperature": {"min": -40.0, "max": 85.0},  # Celsius
        }

    def _generate_raw_data(self, environment_state: Dict[str, Any]) -> Dict[str, Any]:
        """Generate raw IMU data.

        Args:
            environment_state: Current environment conditions

        Returns:
            Dict containing raw IMU measurements
        """
        # Simulate small angular velocity changes (vehicle rotation)
        ang_vel_noise = np.random.normal(0, 0.01, 3)  # Small random rotations
        self.angular_velocity = ang_vel_noise.tolist()

        # Simulate acceleration (mostly gravity with small movements)
        accel_noise = np.random.normal(0, 0.05, 3)
        self.linear_acceleration = [0.0, 0.0, 9.81]  # Gravity
        self.linear_acceleration = [
            a + n for a, n in zip(self.linear_acceleration, accel_noise)
        ]

        # Simulate temperature changes
        temp_change = np.random.normal(0, 0.1)
        self.temperature = max(-40, min(85, self.temperature + temp_change))

        # Update biases over time (drift)
        bias_update = np.random.normal(0, 0.001, 6)  # Slow bias drift
        self.gyro_bias = [b + bias_update[i] for i, b in enumerate(self.gyro_bias)]
        self.accel_bias = [b + bias_update[i+3] for i, b in enumerate(self.accel_bias)]

        return {
            "orientation_w": self.current_orientation[0],
            "orientation_x": self.current_orientation[1],
            "orientation_y": self.current_orientation[2],
            "orientation_z": self.current_orientation[3],
            "angular_velocity_x": self.angular_velocity[0],
            "angular_velocity_y": self.angular_velocity[1],
            "angular_velocity_z": self.angular_velocity[2],
            "linear_acceleration_x": self.linear_acceleration[0],
            "linear_acceleration_y": self.linear_acceleration[1],
            "linear_acceleration_z": self.linear_acceleration[2],
            "temperature": self.temperature,
            "calibration_quality": self.calibration_quality,
        }

    def _apply_environmental_effects(
        self, data: Dict[str, Any], environment: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Apply environmental effects to IMU data.

        Args:
            data: Raw IMU data
            environment: Environment conditions

        Returns:
            Dict with environmental effects applied
        """
        modified_data = data.copy()

        # Temperature effects on bias
        temperature = environment.get("temperature", 25.0)
        temp_offset = (temperature - 25.0) * self.temperature_coefficient

        # Apply temperature-induced bias
        modified_data["angular_velocity_x"] += temp_offset
        modified_data["angular_velocity_y"] += temp_offset
        modified_data["angular_velocity_z"] += temp_offset

        # Vibration effects
        terrain_difficulty = environment.get("terrain_difficulty", 0.0)
        wind_speed = environment.get("wind_speed", 0.0)

        if terrain_difficulty > 0.5 or wind_speed > 5.0:
            # Add vibration-induced noise
            vibration_noise = np.random.normal(0, 0.1 * (terrain_difficulty + wind_speed * 0.1), 6)
            modified_data["angular_velocity_x"] += vibration_noise[0]
            modified_data["angular_velocity_y"] += vibration_noise[1]
            modified_data["angular_velocity_z"] += vibration_noise[2]
            modified_data["linear_acceleration_x"] += vibration_noise[3]
            modified_data["linear_acceleration_y"] += vibration_noise[4]
            modified_data["linear_acceleration_z"] += vibration_noise[5]

        # Dust effects (can affect sensor readings)
        dust_density = environment.get("dust_density", 0.0)
        if dust_density > 0.7:
            # Add noise from dust contamination
            dust_noise = np.random.normal(0, dust_density * 0.5, 6)
            modified_data["angular_velocity_x"] += dust_noise[0]
            modified_data["angular_velocity_y"] += dust_noise[1]
            modified_data["angular_velocity_z"] += dust_noise[2]
            modified_data["linear_acceleration_x"] += dust_noise[3]
            modified_data["linear_acceleration_y"] += dust_noise[4]
            modified_data["linear_acceleration_z"] += dust_noise[5]

        return modified_data

    def _add_noise(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Add IMU-specific noise to data.

        Args:
            data: Clean IMU data

        Returns:
            Dict with noise added
        """
        noisy_data = data.copy()

        # Gyroscope noise and bias
        gyro_noise = np.random.normal(0, self.gyro_noise_std, 3)
        noisy_data["angular_velocity_x"] += gyro_noise[0] + self.gyro_bias[0]
        noisy_data["angular_velocity_y"] += gyro_noise[1] + self.gyro_bias[1]
        noisy_data["angular_velocity_z"] += gyro_noise[2] + self.gyro_bias[2]

        # Accelerometer noise and bias
        accel_noise = np.random.normal(0, self.accel_noise_std, 3)
        noisy_data["linear_acceleration_x"] += accel_noise[0] + self.accel_bias[0]
        noisy_data["linear_acceleration_y"] += accel_noise[1] + self.accel_bias[1]
        noisy_data["linear_acceleration_z"] += accel_noise[2] + self.accel_bias[2]

        # Temperature noise
        temp_noise = np.random.normal(0, 0.5)
        noisy_data["temperature"] += temp_noise

        return noisy_data

    def _calculate_data_quality(self, data: Dict[str, Any]) -> float:
        """Calculate IMU data quality score.

        Args:
            data: IMU data

        Returns:
            float: Quality score (0.0 to 1.0)
        """
        if not data.get("_is_valid", True):
            return 0.0

        score = 1.0

        # Reduce score based on calibration quality
        cal_quality = data.get("calibration_quality", 1.0)
        score *= cal_quality

        # Reduce score based on temperature extremes
        temperature = data.get("temperature", 25.0)
        if abs(temperature - 25.0) > 30.0:  # More than 30°C from nominal
            temp_penalty = min(0.5, abs(temperature - 25.0) / 60.0)
            score *= (1.0 - temp_penalty)

        # Reduce score based on bias magnitude
        gyro_bias_magnitude = np.sqrt(
            self.gyro_bias[0]**2 + self.gyro_bias[1]**2 + self.gyro_bias[2]**2
        )
        if gyro_bias_magnitude > 0.1:  # Significant gyro bias
            bias_penalty = min(0.3, gyro_bias_magnitude / 0.5)
            score *= (1.0 - bias_penalty)

        return max(0.0, min(1.0, score))

    def calibrate(self):
        """Perform IMU calibration to reduce biases."""
        self.logger.info("Performing IMU calibration...")

        # Reset biases to small random values (representing calibration error)
        self.gyro_bias = np.random.normal(0, 0.001, 3).tolist()
        self.accel_bias = np.random.normal(0, 0.01, 3).tolist()

        # Improve calibration quality
        self.calibration_quality = min(1.0, self.calibration_quality + 0.2)

        self.is_calibrated = True
        self.logger.info("IMU calibration completed")

    def get_bias_info(self) -> Dict[str, Any]:
        """Get current bias information.

        Returns:
            Dict with bias details
        """
        return {
            "gyro_bias": self.gyro_bias.copy(),
            "accel_bias": self.accel_bias.copy(),
            "gyro_bias_magnitude": np.sqrt(sum(b**2 for b in self.gyro_bias)),
            "accel_bias_magnitude": np.sqrt(sum(b**2 for b in self.accel_bias)),
            "calibration_quality": self.calibration_quality,
            "is_calibrated": self.is_calibrated,
        }

    def simulate_vibration(self, intensity: float = 1.0, duration_sec: float = 5.0):
        """Simulate vibration effects.

        Args:
            intensity: Vibration intensity (0.0 to 1.0)
            duration_sec: Duration of vibration in seconds
        """
        self.logger.info(f"Simulating vibration (intensity: {intensity}) for {duration_sec}s")

        original_noise_std = self.accel_noise_std
        self.accel_noise_std *= (1.0 + intensity * 2.0)  # Increase noise

        def end_vibration():
            import time
            time.sleep(duration_sec)
            self.accel_noise_std = original_noise_std
            self.logger.info("Vibration simulation ended")

        import threading
        vibration_thread = threading.Thread(target=end_vibration, daemon=True)
        vibration_thread.start()

    def get_ground_truth(self) -> Dict[str, Any]:
        """Get ground truth IMU state (no noise or bias).

        Returns:
            Dict with true IMU state
        """
        return {
            "angular_velocity": [0.0, 0.0, 0.0],
            "linear_acceleration": [0.0, 0.0, 9.81],
            "temperature": 25.0,
            "is_ground_truth": True,
        }
