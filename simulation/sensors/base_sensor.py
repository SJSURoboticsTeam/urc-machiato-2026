"""Base sensor interface for simulation.

Provides common interface and functionality for all simulated sensors
including noise models, update rates, and failure simulation.

Author: URC 2026 Autonomy Team
"""

import logging
import random
import time
from abc import ABC, abstractmethod
from typing import Any, Dict, Optional

import numpy as np


class BaseSensor(ABC):
    """Abstract base class for all simulated sensors.

    Provides common functionality for:
    - Update rate management
    - Noise injection
    - Failure simulation
    - Data validation
    - State management
    """

    def __init__(self, config: Dict[str, Any]):
        """Initialize base sensor.

        Args:
            config: Sensor configuration dictionary
        """
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")

        # Configuration
        self.name = config.get("name", "unknown_sensor")
        self.sensor_type = config.get("sensor_type", "unknown")
        self.update_rate_hz = config.get("update_rate", 10.0)
        self.enabled = config.get("enabled", True)

        # State
        self.last_update_time = 0.0
        self.update_count = 0
        self.error_count = 0
        self.is_connected = True
        self.is_calibrated = True

        # Performance tracking
        self.update_times: list[float] = []
        self.data_quality_scores: list[float] = []

        # Failure simulation
        self.failure_rate = config.get("failure_rate", 0.001)  # 0.1% per update
        self.recovery_time_sec = config.get("recovery_time", 5.0)

        # Noise model
        self.noise_model = config.get("noise_model", {})
        self.noise_multiplier = self.noise_model.get("multiplier", 1.0)

        # Data validation
        self.data_bounds = config.get("data_bounds", {})

        self.logger.info(
            f"Initialized {self.__class__.__name__} '{self.name}' "
            f"({self.update_rate_hz}Hz)"
        )

    @abstractmethod
    def _generate_raw_data(self, environment_state: Dict[str, Any]) -> Dict[str, Any]:
        """Generate raw sensor data based on environment.

        Args:
            environment_state: Current environment conditions

        Returns:
            Dict containing raw sensor measurements
        """
        pass

    def step(self, dt: float, environment_state: Dict[str, Any]) -> Dict[str, Any]:
        """Execute sensor update step.

        Args:
            dt: Time step in seconds
            environment_state: Current environment conditions

        Returns:
            Dict containing sensor data and metadata
        """
        if not self.enabled:
            return self._create_disabled_response()

        current_time = time.time()

        # Check if it's time for an update
        time_since_update = current_time - self.last_update_time
        update_interval = 1.0 / self.update_rate_hz

        if time_since_update < update_interval:
            # Return last data if update not due
            return self._get_cached_data()

        try:
            # Generate raw sensor data
            raw_data = self._generate_raw_data(environment_state)

            # Apply environmental degradation
            degraded_data = self._apply_environmental_effects(raw_data, environment_state)

            # Add noise
            noisy_data = self._add_noise(degraded_data)

            # Simulate failures
            if self._should_fail():
                return self._create_failure_response()

            # Validate data
            validated_data = self._validate_data(noisy_data)

            # Update state
            self.last_update_time = current_time
            self.update_count += 1

            # Track performance
            update_duration = time.time() - current_time
            self.update_times.append(update_duration)
            if len(self.update_times) > 100:
                self.update_times.pop(0)

            # Calculate data quality
            quality_score = self._calculate_data_quality(validated_data)
            self.data_quality_scores.append(quality_score)
            if len(self.data_quality_scores) > 100:
                self.data_quality_scores.pop(0)

            return self._create_success_response(validated_data, quality_score)

        except Exception as e:
            self.logger.error(f"Sensor update failed: {e}")
            self.error_count += 1
            return self._create_error_response(str(e))

    def reset(self):
        """Reset sensor to initial state."""
        self.last_update_time = 0.0
        self.update_count = 0
        self.error_count = 0
        self.is_connected = True
        self.is_calibrated = True
        self.update_times.clear()
        self.data_quality_scores.clear()

        self.logger.info(f"Reset {self.name}")

    def get_state(self) -> Dict[str, Any]:
        """Get current sensor state.

        Returns:
            Dict containing complete sensor state
        """
        return {
            "name": self.name,
            "type": self.sensor_type,
            "enabled": self.enabled,
            "connected": self.is_connected,
            "calibrated": self.is_calibrated,
            "update_rate_hz": self.update_rate_hz,
            "last_update_time": self.last_update_time,
            "update_count": self.update_count,
            "error_count": self.error_count,
            "average_update_time": (
                sum(self.update_times) / len(self.update_times)
                if self.update_times
                else 0
            ),
            "average_data_quality": (
                sum(self.data_quality_scores) / len(self.data_quality_scores)
                if self.data_quality_scores
                else 0
            ),
            "failure_rate": self.failure_rate,
        }

    def set_failure_rate(self, rate: float):
        """Set sensor failure rate.

        Args:
            rate: Failure rate (0.0 to 1.0)
        """
        if not (0.0 <= rate <= 1.0):
            raise ValueError("Failure rate must be between 0.0 and 1.0")

        self.failure_rate = rate

    def set_noise_multiplier(self, multiplier: float):
        """Set noise multiplier for environmental effects.

        Args:
            multiplier: Noise multiplier (>= 0.0)
        """
        if multiplier < 0.0:
            raise ValueError("Noise multiplier must be non-negative")

        self.noise_multiplier = multiplier

    def _apply_environmental_effects(
        self, data: Dict[str, Any], environment: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Apply environmental effects to sensor data.

        Args:
            data: Raw sensor data
            environment: Environment conditions

        Returns:
            Dict with environmental effects applied
        """
        # Base implementation - sensors can override for specific effects
        return data

    def _add_noise(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Add noise to sensor data.

        Args:
            data: Clean sensor data

        Returns:
            Dict with noise added
        """
        # Base implementation - sensors can override for specific noise models
        return data

    def _should_fail(self) -> bool:
        """Determine if sensor should fail this update.

        Returns:
            bool: True if sensor should fail
        """
        return random.random() < self.failure_rate

    def _validate_data(self, data: Dict[str, Any]) -> Dict[str, Any]:
        """Validate sensor data against bounds.

        Args:
            data: Sensor data to validate

        Returns:
            Dict with validation flags
        """
        validated = data.copy()
        validated["_is_valid"] = True
        validated["_validation_errors"] = []

        # Check bounds for each configured field
        for field, bounds in self.data_bounds.items():
            if field in data:
                value = data[field]
                if isinstance(bounds, dict):
                    min_val = bounds.get("min")
                    max_val = bounds.get("max")

                    if min_val is not None and value < min_val:
                        validated["_is_valid"] = False
                        validated["_validation_errors"].append(
                            f"{field} below minimum: {value} < {min_val}"
                        )

                    if max_val is not None and value > max_val:
                        validated["_is_valid"] = False
                        validated["_validation_errors"].append(
                            f"{field} above maximum: {value} > {max_val}"
                        )

        return validated

    def _calculate_data_quality(self, data: Dict[str, Any]) -> float:
        """Calculate data quality score (0.0 to 1.0).

        Args:
            data: Validated sensor data

        Returns:
            float: Quality score
        """
        if not data.get("_is_valid", True):
            return 0.0

        # Base implementation - sensors can override for specific quality metrics
        return 1.0

    def _get_cached_data(self) -> Dict[str, Any]:
        """Get cached sensor data when update not due.

        Returns:
            Dict with cached data and metadata
        """
        return {
            "sensor_name": self.name,
            "sensor_type": self.sensor_type,
            "timestamp": self.last_update_time,
            "is_cached": True,
            "update_count": self.update_count,
            "error_count": self.error_count,
        }

    def _create_success_response(
        self, data: Dict[str, Any], quality_score: float
    ) -> Dict[str, Any]:
        """Create successful response.

        Args:
            data: Sensor data
            quality_score: Data quality score

        Returns:
            Dict with success response
        """
        return {
            "sensor_name": self.name,
            "sensor_type": self.sensor_type,
            "timestamp": time.time(),
            "status": "success",
            "data": data,
            "quality_score": quality_score,
            "update_count": self.update_count,
            "error_count": self.error_count,
            "is_simulation": True,
        }

    def _create_failure_response(self) -> Dict[str, Any]:
        """Create failure response.

        Returns:
            Dict with failure response
        """
        self.error_count += 1
        return {
            "sensor_name": self.name,
            "sensor_type": self.sensor_type,
            "timestamp": time.time(),
            "status": "failure",
            "error": "simulated_sensor_failure",
            "recovery_time_sec": self.recovery_time_sec,
            "update_count": self.update_count,
            "error_count": self.error_count,
            "is_simulation": True,
        }

    def _create_error_response(self, error_msg: str) -> Dict[str, Any]:
        """Create error response.

        Args:
            error_msg: Error message

        Returns:
            Dict with error response
        """
        return {
            "sensor_name": self.name,
            "sensor_type": self.sensor_type,
            "timestamp": time.time(),
            "status": "error",
            "error": error_msg,
            "update_count": self.update_count,
            "error_count": self.error_count,
            "is_simulation": True,
        }

    def _create_disabled_response(self) -> Dict[str, Any]:
        """Create disabled sensor response.

        Returns:
            Dict with disabled response
        """
        return {
            "sensor_name": self.name,
            "sensor_type": self.sensor_type,
            "timestamp": time.time(),
            "status": "disabled",
            "update_count": self.update_count,
            "error_count": self.error_count,
            "is_simulation": True,
        }
