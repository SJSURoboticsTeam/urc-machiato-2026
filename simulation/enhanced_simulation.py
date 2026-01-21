#!/usr/bin/env python3
"""
Enhanced URC 2026 Simulation - Low Complexity Realism Improvements

Addresses simulation limitations with targeted, low-complexity enhancements:
- Realistic sensor data generation
- Basic timing validation
- Simple environmental modeling
- Hardware interface mocking with realistic constraints

Author: URC 2026 Enhanced Simulation Team
"""

import time
import random
import math
from typing import Dict, List, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum


class SensorType(Enum):
    """Supported sensor types."""
    GPS = "gps"
    IMU = "imu"
    CAMERA = "camera"
    LIDAR = "lidar"
    BATTERY = "battery"


@dataclass
class SensorReading:
    """Realistic sensor reading with metadata."""
    sensor_type: SensorType
    timestamp: float
    value: Any
    accuracy: float  # 0.0 to 1.0
    noise_level: float
    calibration_offset: float = 0.0


class RealisticSensorSimulator:
    """
    Enhanced sensor simulator with realistic constraints.

    Provides more accurate sensor modeling without complex physics simulation.
    """

    def __init__(self, seed: Optional[int] = None):
        self.random = random.Random(seed)
        self.start_time = time.time()

        # Sensor characteristics (realistic values)
        self.sensor_specs = {
            SensorType.GPS: {
                "update_rate_hz": 10,
                "accuracy_m": 2.0,
                "noise_std": 0.5,
                "drift_rate": 0.01  # meters per second
            },
            SensorType.IMU: {
                "update_rate_hz": 100,
                "accel_noise": 0.01,  # m/s²
                "gyro_noise": 0.001,  # rad/s
                "bias_drift": 0.0001
            },
            SensorType.CAMERA: {
                "update_rate_hz": 30,
                "resolution": (640, 480),
                "noise_std": 5,  # pixel intensity
                "motion_blur_factor": 0.1
            },
            SensorType.LIDAR: {
                "update_rate_hz": 10,
                "range_max_m": 30,
                "angular_resolution_deg": 1.0,
                "range_noise_std": 0.02
            },
            SensorType.BATTERY: {
                "update_rate_hz": 1,
                "capacity_wh": 100,
                "voltage_nominal": 12.0,
                "discharge_rate_w": 15
            }
        }

        # State tracking
        self.last_readings: Dict[SensorType, SensorReading] = {}
        self.vehicle_state = {
            "position": [0.0, 0.0, 0.0],  # x, y, z
            "velocity": [0.0, 0.0, 0.0],  # vx, vy, vz
            "orientation": [0.0, 0.0, 0.0],  # roll, pitch, yaw
            "battery_level": 1.0  # 0.0 to 1.0
        }

    def get_sensor_reading(self, sensor_type: SensorType) -> SensorReading:
        """Get realistic sensor reading with timing constraints."""
        current_time = time.time()
        specs = self.sensor_specs[sensor_type]

        # Check update rate timing
        last_reading = self.last_readings.get(sensor_type)
        if last_reading:
            time_since_last = current_time - last_reading.timestamp
            min_interval = 1.0 / specs["update_rate_hz"]
            if time_since_last < min_interval:
                # Return last reading if too soon
                return last_reading

        # Generate reading based on sensor type
        if sensor_type == SensorType.GPS:
            reading = self._generate_gps_reading(current_time)
        elif sensor_type == SensorType.IMU:
            reading = self._generate_imu_reading(current_time)
        elif sensor_type == SensorType.CAMERA:
            reading = self._generate_camera_reading(current_time)
        elif sensor_type == SensorType.LIDAR:
            reading = self._generate_lidar_reading(current_time)
        elif sensor_type == SensorType.BATTERY:
            reading = self._generate_battery_reading(current_time)
        else:
            raise ValueError(f"Unsupported sensor type: {sensor_type}")

        self.last_readings[sensor_type] = reading
        return reading

    def _generate_gps_reading(self, timestamp: float) -> SensorReading:
        """Generate realistic GPS reading."""
        specs = self.sensor_specs[SensorType.GPS]

        # Update position based on velocity
        dt = 0.1  # 10Hz update
        pos = self.vehicle_state["position"]
        vel = self.vehicle_state["velocity"]

        pos[0] += vel[0] * dt
        pos[1] += vel[1] * dt

        # Add GPS-specific noise and accuracy
        noise = self.random.gauss(0, specs["noise_std"])
        accuracy = max(0.1, 1.0 - (noise / specs["accuracy_m"]))

        reading = {
            "latitude": 35.0 + pos[1] / 111000,  # Rough lat conversion
            "longitude": -120.0 + pos[0] / 111000,  # Rough lon conversion
            "altitude": pos[2],
            "hdop": 1.0 + abs(noise) * 0.5,
            "satellites_visible": max(4, 12 - int(abs(noise) * 2))
        }

        return SensorReading(
            sensor_type=SensorType.GPS,
            timestamp=timestamp,
            value=reading,
            accuracy=accuracy,
            noise_level=abs(noise),
            calibration_offset=0.0
        )

    def _generate_imu_reading(self, timestamp: float) -> SensorReading:
        """Generate realistic IMU reading."""
        specs = self.sensor_specs[SensorType.IMU]

        # Simulate vehicle motion
        dt = 0.01  # 100Hz update

        # Add some realistic acceleration (gravity + motion)
        accel_noise = self.random.gauss(0, specs["accel_noise"])
        gyro_noise = self.random.gauss(0, specs["gyro_noise"])

        reading = {
            "linear_acceleration": {
                "x": accel_noise,
                "y": 0.0,
                "z": 9.81 + accel_noise  # Gravity
            },
            "angular_velocity": {
                "x": gyro_noise,
                "y": 0.0,
                "z": gyro_noise
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": 0.0,
                "w": 1.0
            }
        }

        return SensorReading(
            sensor_type=SensorType.IMU,
            timestamp=timestamp,
            value=reading,
            accuracy=0.95,  # IMU typically very accurate
            noise_level=max(abs(accel_noise), abs(gyro_noise)),
            calibration_offset=0.0
        )

    def _generate_camera_reading(self, timestamp: float) -> SensorReading:
        """Generate realistic camera reading."""
        specs = self.sensor_specs[SensorType.CAMERA]

        # Simulate image capture with motion blur based on velocity
        velocity_magnitude = math.sqrt(sum(v**2 for v in self.vehicle_state["velocity"]))
        motion_blur = velocity_magnitude * specs["motion_blur_factor"]

        reading = {
            "width": specs["resolution"][0],
            "height": specs["resolution"][1],
            "format": "RGB",
            "motion_blur_level": motion_blur,
            "brightness": 0.7 + self.random.gauss(0, 0.1),
            "contrast": 1.0 + self.random.gauss(0, 0.05)
        }

        return SensorReading(
            sensor_type=SensorType.CAMERA,
            timestamp=timestamp,
            value=reading,
            accuracy=0.85,  # Cameras have some variability
            noise_level=motion_blur,
            calibration_offset=0.0
        )

    def _generate_lidar_reading(self, timestamp: float) -> SensorReading:
        """Generate realistic LiDAR reading."""
        specs = self.sensor_specs[SensorType.LIDAR]

        # Generate simple point cloud (reduced complexity)
        num_points = 360  # 1 degree resolution
        ranges = []

        for i in range(num_points):
            angle_rad = math.radians(i)
            # Simulate obstacles at different distances
            base_range = specs["range_max_m"] * (0.5 + 0.5 * self.random.random())
            noise = self.random.gauss(0, specs["range_noise_std"])
            ranges.append(max(0.1, min(specs["range_max_m"], base_range + noise)))

        reading = {
            "ranges": ranges,
            "angle_min": 0.0,
            "angle_max": 2 * math.pi,
            "range_min": 0.1,
            "range_max": specs["range_max_m"],
            "angle_increment": math.radians(specs["angular_resolution_deg"])
        }

        return SensorReading(
            sensor_type=SensorType.LIDAR,
            timestamp=timestamp,
            value=reading,
            accuracy=0.9,  # LiDAR generally reliable
            noise_level=sum(abs(r - specs["range_max_m"]/2) for r in ranges[:10]) / 10,
            calibration_offset=0.0
        )

    def _generate_battery_reading(self, timestamp: float) -> SensorReading:
        """Generate realistic battery reading."""
        specs = self.sensor_specs[SensorType.BATTERY]

        # Simulate battery discharge
        dt = 1.0  # 1Hz update
        power_consumption = specs["discharge_rate_w"] * dt / 3600  # Wh consumed
        energy_consumed = power_consumption
        self.vehicle_state["battery_level"] = max(0.0, self.vehicle_state["battery_level"] - energy_consumed / specs["capacity_wh"])

        voltage_noise = self.random.gauss(0, 0.1)
        voltage = specs["voltage_nominal"] * (0.8 + 0.4 * self.vehicle_state["battery_level"]) + voltage_noise

        reading = {
            "voltage": voltage,
            "current": specs["discharge_rate_w"] / voltage,
            "percentage": self.vehicle_state["battery_level"] * 100,
            "capacity_wh": specs["capacity_wh"],
            "temperature_c": 25.0 + self.random.gauss(0, 2)
        }

        return SensorReading(
            sensor_type=SensorType.BATTERY,
            timestamp=timestamp,
            value=reading,
            accuracy=0.98,  # Battery monitoring very accurate
            noise_level=abs(voltage_noise),
            calibration_offset=0.0
        )

    def update_vehicle_state(self, velocity: Optional[List[float]] = None,
                           angular_velocity: Optional[List[float]] = None):
        """Update vehicle state for more realistic simulation."""
        if velocity:
            self.vehicle_state["velocity"] = velocity

        if angular_velocity:
            # Update orientation (simplified)
            dt = 0.01
            self.vehicle_state["orientation"][2] += angular_velocity[2] * dt  # Yaw

    def get_vehicle_state(self) -> Dict[str, Any]:
        """Get current vehicle state."""
        return self.vehicle_state.copy()


class TimingValidator:
    """
    Simple timing validation for real-time constraints.

    Checks if operations meet timing requirements without full RTOS simulation.
    """

    def __init__(self):
        self.timing_constraints = {
            "sensor_update": 0.1,  # 10Hz max latency
            "control_loop": 0.02,  # 50Hz max latency
            "navigation_compute": 0.05,  # 20Hz max latency
            "communication": 0.01  # 100Hz max latency
        }
        self.measurements: Dict[str, List[float]] = {}

    def measure_operation(self, operation_name: str):
        """Context manager for measuring operation timing."""
        return TimingContext(self, operation_name)

    def record_timing(self, operation_name: str, duration: float):
        """Record operation timing."""
        if operation_name not in self.measurements:
            self.measurements[operation_name] = []

        self.measurements[operation_name].append(duration)

        # Check constraint
        constraint = self.timing_constraints.get(operation_name)
        if constraint and duration > constraint:
            print(f"⚠️  Timing violation: {operation_name} took {duration:.3f}s (limit: {constraint:.3f}s)")

    def get_timing_stats(self) -> Dict[str, Dict[str, float]]:
        """Get timing statistics for all operations."""
        stats = {}
        for op_name, times in self.measurements.items():
            if times:
                stats[op_name] = {
                    "count": len(times),
                    "avg": sum(times) / len(times),
                    "max": max(times),
                    "constraint": self.timing_constraints.get(op_name, 0.0)
                }
        return stats


class TimingContext:
    """Context manager for timing measurements."""

    def __init__(self, validator: TimingValidator, operation_name: str):
        self.validator = validator
        self.operation_name = operation_name
        self.start_time = None

    def __enter__(self):
        self.start_time = time.time()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.start_time:
            duration = time.time() - self.start_time
            self.validator.record_timing(self.operation_name, duration)


class EnvironmentalSimulator:
    """
    Simple environmental modeling for Mars-like conditions.

    Models basic environmental factors affecting sensor performance.
    """

    def __init__(self):
        self.conditions = {
            "dust_level": 0.0,  # 0.0 to 1.0
            "temperature_c": 20.0,
            "humidity_percent": 50.0,
            "wind_speed_ms": 0.0,
            "atmospheric_pressure_pa": 101325
        }

    def set_mars_conditions(self):
        """Set Mars-like environmental conditions."""
        self.conditions.update({
            "dust_level": 0.7,
            "temperature_c": -60.0,
            "humidity_percent": 0.1,
            "wind_speed_ms": 5.0,
            "atmospheric_pressure_pa": 600
        })

    def get_environmental_factors(self) -> Dict[str, float]:
        """Get current environmental factors affecting sensors."""
        return self.conditions.copy()

    def apply_environmental_effects(self, sensor_reading: SensorReading) -> SensorReading:
        """Apply environmental effects to sensor reading."""
        # Dust affects camera and LiDAR
        if sensor_reading.sensor_type in [SensorType.CAMERA, SensorType.LIDAR]:
            dust_factor = 1.0 - (self.conditions["dust_level"] * 0.3)
            sensor_reading.accuracy *= dust_factor

        # Temperature affects all sensors
        temp_offset = (self.conditions["temperature_c"] - 20.0) * 0.001
        sensor_reading.calibration_offset += temp_offset

        # Wind affects GPS accuracy
        if sensor_reading.sensor_type == SensorType.GPS:
            wind_factor = 1.0 - (self.conditions["wind_speed_ms"] * 0.01)
            sensor_reading.accuracy *= max(0.5, wind_factor)

        return sensor_reading


# Global instances
_sensor_simulator = RealisticSensorSimulator()
_timing_validator = TimingValidator()
_environment_simulator = EnvironmentalSimulator()

def get_sensor_simulator() -> RealisticSensorSimulator:
    """Get global sensor simulator."""
    return _sensor_simulator

def get_timing_validator() -> TimingValidator:
    """Get global timing validator."""
    return _timing_validator

def get_environment_simulator() -> EnvironmentalSimulator:
    """Get global environment simulator."""
    return _environment_simulator

# Convenience functions
def get_realistic_sensor_reading(sensor_type: SensorType) -> SensorReading:
    """Get realistic sensor reading with environmental effects."""
    reading = _sensor_simulator.get_sensor_reading(sensor_type)
    return _environment_simulator.apply_environmental_effects(reading)

def measure_operation_timing(operation_name: str):
    """Decorator for measuring operation timing."""
    def decorator(func):
        def wrapper(*args, **kwargs):
            with _timing_validator.measure_operation(operation_name):
                return func(*args, **kwargs)
        return wrapper
    return decorator

# Export key components
__all__ = [
    'RealisticSensorSimulator',
    'TimingValidator',
    'EnvironmentalSimulator',
    'SensorType',
    'SensorReading',
    'get_sensor_simulator',
    'get_timing_validator',
    'get_environment_simulator',
    'get_realistic_sensor_reading',
    'measure_operation_timing'
]




