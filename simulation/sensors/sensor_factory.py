"""Sensor factory for creating and managing simulated sensors.

Provides unified interface for creating different sensor types with
consistent configuration and initialization.

Author: URC 2026 Autonomy Team
"""

import logging
from typing import Any, Dict, Optional

from simulation.sensors.base_sensor import BaseSensor
from simulation.sensors.gps_simulator import GPSSimulator
from simulation.sensors.imu_simulator import IMUSimulator


class SensorFactory:
    """Factory for creating simulated sensors.

    Manages sensor creation, configuration, and lifecycle.
    Provides consistent interface for all sensor types.
    """

    # Registry of available sensor types
    _sensor_types = {
        "gps": GPSSimulator,
        "imu": IMUSimulator,
        # Add more sensor types here as they are implemented
        # "camera": CameraSimulator,
        # "lidar": LiDARSimulator,
        # "battery": BatterySimulator,
        # "encoder": EncoderSimulator,
    }

    @classmethod
    def create(cls, sensor_type: str, config: Dict[str, Any]) -> BaseSensor:
        """Create a sensor instance.

        Args:
            sensor_type: Type of sensor to create
            config: Sensor configuration

        Returns:
            BaseSensor: Configured sensor instance

        Raises:
            ValueError: If sensor type is not supported
        """
        if sensor_type not in cls._sensor_types:
            available_types = list(cls._sensor_types.keys())
            raise ValueError(
                f"Unsupported sensor type '{sensor_type}'. "
                f"Available types: {available_types}"
            )

        sensor_class = cls._sensor_types[sensor_type]

        # Validate configuration
        cls._validate_config(sensor_type, config)

        # Create sensor instance
        try:
            sensor = sensor_class(config)
            return sensor
        except Exception as e:
            logger = logging.getLogger(__name__)
            logger.error(f"Failed to create {sensor_type} sensor: {e}")
            raise

    @classmethod
    def register_sensor_type(cls, sensor_type: str, sensor_class: type):
        """Register a new sensor type.

        Args:
            sensor_type: Name of sensor type
            sensor_class: Sensor class that inherits from BaseSensor
        """
        if not issubclass(sensor_class, BaseSensor):
            raise TypeError("Sensor class must inherit from BaseSensor")

        cls._sensor_types[sensor_type] = sensor_class

        logger = logging.getLogger(__name__)
        logger.info(f"Registered sensor type: {sensor_type}")

    @classmethod
    def get_available_types(cls) -> list[str]:
        """Get list of available sensor types.

        Returns:
            list: Available sensor type names
        """
        return list(cls._sensor_types.keys())

    @classmethod
    def get_sensor_info(cls, sensor_type: str) -> Optional[Dict[str, Any]]:
        """Get information about a sensor type.

        Args:
            sensor_type: Type of sensor

        Returns:
            Dict with sensor information or None if not found
        """
        if sensor_type not in cls._sensor_types:
            return None

        sensor_class = cls._sensor_types[sensor_type]

        return {
            "type": sensor_type,
            "class": sensor_class.__name__,
            "module": sensor_class.__module__,
            "description": sensor_class.__doc__ or "No description available",
        }

    @classmethod
    def create_sensor_suite(
        cls, sensor_configs: list[Dict[str, Any]]
    ) -> Dict[str, BaseSensor]:
        """Create a suite of sensors from configurations.

        Args:
            sensor_configs: List of sensor configurations

        Returns:
            Dict mapping sensor names to sensor instances
        """
        sensors = {}

        for config in sensor_configs:
            sensor_name = config.get("name")
            if not sensor_name:
                raise ValueError("Sensor configuration must include 'name' field")

            sensor = cls.create(config["type"], config)
            sensors[sensor_name] = sensor

        logger = logging.getLogger(__name__)
        logger.info(f"Created sensor suite with {len(sensors)} sensors")

        return sensors

    @classmethod
    def _validate_config(cls, sensor_type: str, config: Dict[str, Any]):
        """Validate sensor configuration.

        Args:
            sensor_type: Type of sensor
            config: Configuration to validate

        Raises:
            ValueError: If configuration is invalid
        """
        required_fields = ["name", "type"]

        for field in required_fields:
            if field not in config:
                raise ValueError(f"Missing required field '{field}' in sensor config")

        # Type-specific validation
        if sensor_type == "gps":
            cls._validate_gps_config(config)
        elif sensor_type == "imu":
            cls._validate_imu_config(config)
        # Add validation for other sensor types as implemented

    @classmethod
    def _validate_gps_config(cls, config: Dict[str, Any]):
        """Validate GPS sensor configuration."""
        # Update rate validation
        update_rate = config.get("update_rate", 10.0)
        if not (0.1 <= update_rate <= 50.0):
            raise ValueError(
                f"GPS update rate {update_rate}Hz not in valid range [0.1, 50.0]"
            )

        # Noise model validation
        noise_model = config.get("noise_model", {})
        if "position_noise_std" in noise_model:
            std = noise_model["position_noise_std"]
            if std < 0:
                raise ValueError(f"GPS position noise std {std} must be non-negative")

    @classmethod
    def _validate_imu_config(cls, config: Dict[str, Any]):
        """Validate IMU sensor configuration."""
        # Update rate validation
        update_rate = config.get("update_rate", 100.0)
        if not (10.0 <= update_rate <= 1000.0):
            raise ValueError(
                f"IMU update rate {update_rate}Hz not in valid range [10.0, 1000.0]"
            )

        # Noise model validation
        noise_model = config.get("noise_model", {})
        if "gyro_noise_std" in noise_model:
            std = noise_model["gyro_noise_std"]
            if std < 0:
                raise ValueError(f"IMU gyro noise std {std} must be non-negative")

        if "accel_noise_std" in noise_model:
            std = noise_model["accel_noise_std"]
            if std < 0:
                raise ValueError(f"IMU accel noise std {std} must be non-negative")
