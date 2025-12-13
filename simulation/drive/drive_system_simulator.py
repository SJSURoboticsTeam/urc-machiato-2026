#!/usr/bin/env python3
"""
Drive System Simulator - URC Machiato 2026

MOCK IMPLEMENTATION - Simulates 6-wheel differential drive rover kinematics,
power consumption, terrain interaction, and control systems.

This is a software simulation of the physical drive system hardware and should be
replaced with actual drive control hardware when available.

Author: URC 2026 Autonomy Team
"""

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple, Union

import numpy as np

logger = logging.getLogger(__name__)


class TerrainType(Enum):
    """Types of terrain for simulation."""
    HARD_SURFACE = "hard_surface"    # Concrete/asphalt
    SOFT_SOIL = "soft_soil"          # Loose dirt/sand
    ROCKY = "rocky"                  # Rocks/boulders
    GRASS = "grass"                  # Grass/vegetation
    MUD = "mud"                      # Wet mud
    SNOW = "snow"                    # Snow/ice


@dataclass
class WheelConfig:
    """Configuration for a drive wheel."""
    name: str
    position: Tuple[float, float, float]  # (x, y, z) relative to center
    radius: float                         # meters
    width: float                          # meters
    max_torque: float                     # Nm
    gear_ratio: float                     # :1


@dataclass
class TerrainProperties:
    """Terrain interaction properties."""
    friction_coefficient: float
    rolling_resistance: float
    sinkage_factor: float        # How much the wheel sinks
    vibration_amplitude: float   # Terrain-induced vibration


class DriveSystemSimulator:
    """
    MOCK IMPLEMENTATION - Complete 6-wheel differential drive simulation.

    Simulates:
    - 6-wheel differential drive kinematics
    - Terrain interaction and wheel slip
    - Power consumption and motor modeling
    - Steering and differential control
    - Suspension and vibration
    - Fault simulation and diagnostics

    This replaces physical drive system hardware for software development.
    """

    def __init__(self, config: Optional[Dict] = None):
        self.logger = logging.getLogger(__name__)

        # Default rover configuration
        if config is None:
            config = self._get_default_config()

        self._load_configuration(config)
        self._initialize_state()
        self._initialize_physics()

        self.logger.info("🚗 MOCK Drive System Simulator initialized")

    def _get_default_config(self) -> Dict:
        """Get default rover configuration."""
        return {
            "name": "URC_Rover_Simulator",
            "wheelbase": 1.2,          # meters between front/rear axles
            "track_width": 0.8,        # meters between left/right wheels
            "wheel_radius": 0.15,      # meters
            "wheel_width": 0.08,       # meters
            "mass": 50.0,              # kg
            "max_speed": 2.0,          # m/s
            "max_acceleration": 1.0,   # m/s²
            "max_steering_angle": np.pi/6,  # 30 degrees
            "motor_power": 150.0,      # Watts per motor
            "battery_voltage": 24.0,   # Volts

            "wheels": [
                # Left side
                {"name": "front_left", "position": (-0.6, 0.4, -0.15)},
                {"name": "middle_left", "position": (0.0, 0.4, -0.15)},
                {"name": "rear_left", "position": (0.6, 0.4, -0.15)},
                # Right side
                {"name": "front_right", "position": (-0.6, -0.4, -0.15)},
                {"name": "middle_right", "position": (0.0, -0.4, -0.15)},
                {"name": "rear_right", "position": (0.6, -0.4, -0.15)}
            ]
        }

    def _load_configuration(self, config: Dict):
        """Load rover configuration."""
        self.name = config["name"]
        self.wheelbase = config["wheelbase"]
        self.track_width = config["track_width"]
        self.wheel_radius = config["wheel_radius"]
        self.wheel_width = config["wheel_width"]
        self.mass = config["mass"]
        self.max_speed = config["max_speed"]
        self.max_acceleration = config["max_acceleration"]
        self.max_steering_angle = config["max_steering_angle"]
        self.motor_power = config["motor_power"]
        self.battery_voltage = config["battery_voltage"]

        # Load wheel configurations
        self.wheels = []
        for wheel_config in config["wheels"]:
            wheel = WheelConfig(
                name=wheel_config["name"],
                position=tuple(wheel_config["position"]),
                radius=self.wheel_radius,
                width=self.wheel_width,
                max_torque=50.0,  # Nm
                gear_ratio=20.0
            )
            self.wheels.append(wheel)

    def _initialize_state(self):
        """Initialize rover state."""
        # Position and orientation
        self.position = np.array([0.0, 0.0, 0.0])      # x, y, z
        self.orientation = np.array([0.0, 0.0, 0.0])   # roll, pitch, yaw
        self.velocity = np.array([0.0, 0.0, 0.0])      # vx, vy, vz
        self.angular_velocity = np.array([0.0, 0.0, 0.0])  # ωx, ωy, ωz

        # Control inputs
        self.linear_velocity_cmd = 0.0   # m/s
        self.angular_velocity_cmd = 0.0  # rad/s
        self.steering_angle_cmd = 0.0    # rad

        # Wheel states
        self.wheel_speeds = np.zeros(6)      # rad/s for each wheel
        self.wheel_torques = np.zeros(6)     # Nm for each wheel
        self.wheel_positions = np.zeros(6)   # radians (odometry)

        # Power and health
        self.battery_level = 100.0          # %
        self.total_power_consumption = 0.0  # Wh
        self.motor_temperatures = np.full(6, 25.0)  # °C

        # Simulation state
        self.last_update_time = time.time()
        self.enabled = True
        self.emergency_stop = False

        # Fault simulation
        self.faults = {
            "motor_failures": [],
            "encoder_failures": [],
            "steering_failures": [],
            "battery_low": False
        }

    def _initialize_physics(self):
        """Initialize physics parameters."""
        # Terrain properties for different surfaces
        self.terrain_properties = {
            TerrainType.HARD_SURFACE: TerrainProperties(0.8, 0.02, 0.0, 0.01),
            TerrainType.SOFT_SOIL: TerrainProperties(0.4, 0.1, 0.05, 0.1),
            TerrainType.ROCKY: TerrainProperties(0.6, 0.08, 0.02, 0.2),
            TerrainType.GRASS: TerrainProperties(0.5, 0.06, 0.01, 0.05),
            TerrainType.MUD: TerrainProperties(0.2, 0.2, 0.1, 0.15),
            TerrainType.SNOW: TerrainProperties(0.1, 0.05, 0.03, 0.08)
        }

        self.current_terrain = TerrainType.HARD_SURFACE

        # Physical constants
        self.gravity = 9.81
        self.air_density = 1.225
        self.drag_coefficient = 0.5
        self.frontal_area = 0.5  # m²

        # Differential drive parameters
        self.wheel_separation = self.track_width
        self.wheel_distance = self.wheelbase

    def set_velocity_commands(self, linear_velocity: float, angular_velocity: float) -> bool:
        """
        Set velocity commands for differential drive.

        Args:
            linear_velocity: Forward/backward velocity (m/s)
            angular_velocity: Rotational velocity (rad/s)

        Returns:
            bool: True if commands accepted
        """
        if self.emergency_stop or not self.enabled:
            return False

        # Apply limits
        self.linear_velocity_cmd = np.clip(linear_velocity, -self.max_speed, self.max_speed)
        self.angular_velocity_cmd = np.clip(angular_velocity, -2.0, 2.0)

        self.logger.debug(f"Set velocity commands: linear={self.linear_velocity_cmd}, angular={self.angular_velocity_cmd}")
        return True

    def set_steering_angle(self, angle: float) -> bool:
        """
        Set steering angle for articulated steering.

        Args:
            angle: Steering angle in radians

        Returns:
            bool: True if command accepted
        """
        if abs(angle) > self.max_steering_angle:
            self.logger.warning(f"Steering angle {angle} exceeds limit {self.max_steering_angle}")
            return False

        self.steering_angle_cmd = angle
        return True

    def update_simulation(self, dt: float):
        """
        Update drive system simulation.

        Args:
            dt: Time step in seconds
        """
        if not self.enabled or self.emergency_stop:
            self.velocity = np.zeros(3)
            self.angular_velocity = np.zeros(3)
            return

        # Calculate wheel speeds for differential drive
        left_speed = self.linear_velocity_cmd - (self.angular_velocity_cmd * self.wheel_separation / 2)
        right_speed = self.linear_velocity_cmd + (self.angular_velocity_cmd * self.wheel_separation / 2)

        # Apply terrain effects and wheel slip
        terrain = self.terrain_properties[self.current_terrain]

        # Calculate effective wheel speeds with slip
        left_slip_factor = self._calculate_slip_factor(left_speed, terrain)
        right_slip_factor = self._calculate_slip_factor(right_speed, terrain)

        effective_left_speed = left_speed * (1.0 - left_slip_factor)
        effective_right_speed = right_speed * (1.0 - right_slip_factor)

        # Update wheel states
        for i, wheel in enumerate(self.wheels):
            if "left" in wheel.name:
                target_speed = effective_left_speed / wheel.radius
            else:
                target_speed = effective_right_speed / wheel.radius

            # Simple first-order lag for wheel speed response
            speed_error = target_speed - self.wheel_speeds[i]
            self.wheel_speeds[i] += speed_error * 5.0 * dt  # 5 Hz bandwidth

            # Update wheel position (odometry)
            self.wheel_positions[i] += self.wheel_speeds[i] * dt

            # Calculate torque (simplified motor model)
            speed_rads = abs(self.wheel_speeds[i])
            max_torque_at_speed = wheel.max_torque * (1.0 - speed_rads / (50.0 / wheel.gear_ratio))
            self.wheel_torques[i] = np.clip(speed_error * 10.0, -max_torque_at_speed, max_torque_at_speed)

        # Update rover velocity and position
        avg_wheel_speed = (effective_left_speed + effective_right_speed) / 2
        angular_speed = (effective_right_speed - effective_left_speed) / self.wheel_separation

        # Apply terrain effects to actual movement
        actual_linear_velocity = avg_wheel_speed * (1.0 - terrain.rolling_resistance)
        actual_angular_velocity = angular_speed * (1.0 - terrain.rolling_resistance * 0.5)

        # Update state
        self.velocity[0] = actual_linear_velocity * np.cos(self.orientation[2])
        self.velocity[1] = actual_linear_velocity * np.sin(self.orientation[2])
        self.angular_velocity[2] = actual_angular_velocity

        # Integrate position and orientation
        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt
        self.orientation[2] += self.angular_velocity[2] * dt

        # Add terrain-induced vibration
        vibration = np.random.normal(0, terrain.vibration_amplitude, 3)
        self.position += vibration * dt

        # Update power consumption
        self._update_power_consumption(dt)

        # Update temperatures
        self._update_thermal_model(dt)

        # Check for faults
        self._check_faults()

        self.last_update_time = time.time()

    def get_drive_state(self) -> Dict:
        """Get complete drive system state."""
        # Estimate odometry from wheel encoders
        left_distance = np.mean([self.wheel_positions[i] * self.wheel_radius
                               for i, w in enumerate(self.wheels) if "left" in w.name])
        right_distance = np.mean([self.wheel_positions[i] * self.wheel_radius
                                for i, w in enumerate(self.wheels) if "right" in w.name])

        return {
            "position": self.position.tolist(),
            "orientation": self.orientation.tolist(),
            "velocity": {
                "linear": self.velocity[:2].tolist(),
                "angular": self.angular_velocity[2]
            },
            "wheel_speeds": self.wheel_speeds.tolist(),
            "wheel_torques": self.wheel_torques.tolist(),
            "odometry": {
                "left_distance": left_distance,
                "right_distance": right_distance,
                "total_distance": (left_distance + right_distance) / 2
            },
            "power_consumption": self.total_power_consumption,
            "battery_level": self.battery_level,
            "motor_temperatures": self.motor_temperatures.tolist(),
            "terrain_type": self.current_terrain.value,
            "enabled": self.enabled,
            "emergency_stop": self.emergency_stop,
            "faults": self.faults.copy(),
            "mock": True,
            "simulated": True
        }

    def set_terrain(self, terrain_type: TerrainType) -> bool:
        """Set current terrain type."""
        if terrain_type in self.terrain_properties:
            self.current_terrain = terrain_type
            self.logger.info(f"Terrain changed to: {terrain_type.value}")
            return True
        else:
            self.logger.error(f"Unknown terrain type: {terrain_type}")
            return False

    def emergency_stop_drive(self) -> bool:
        """Emergency stop all drive motors."""
        self.logger.warning("🚨 DRIVE EMERGENCY STOP")
        self.emergency_stop = True
        self.linear_velocity_cmd = 0.0
        self.angular_velocity_cmd = 0.0
        self.wheel_speeds = np.zeros(6)
        return True

    def enable_drive(self) -> bool:
        """Enable drive system."""
        if not self.emergency_stop:
            self.enabled = True
            self.logger.info("Drive system enabled")
            return True
        else:
            self.logger.warning("Cannot enable drive while in emergency stop")
            return False

    def disable_drive(self) -> bool:
        """Disable drive system."""
        self.enabled = False
        self.linear_velocity_cmd = 0.0
        self.angular_velocity_cmd = 0.0
        self.wheel_speeds = np.zeros(6)
        self.logger.info("Drive system disabled")
        return True

    def clear_emergency_stop(self) -> bool:
        """Clear emergency stop condition."""
        if self.emergency_stop:
            self.emergency_stop = False
            self.logger.info("Emergency stop cleared")
        return True

    def _calculate_slip_factor(self, wheel_speed: float, terrain: TerrainProperties) -> float:
        """Calculate wheel slip factor based on terrain."""
        # Simplified slip model based on friction and load
        max_friction_speed = 1.0  # m/s where full friction is available
        if abs(wheel_speed) < max_friction_speed:
            slip_factor = 0.0
        else:
            # Exponential slip increase
            slip_factor = min(0.8, (abs(wheel_speed) - max_friction_speed) * 0.3)

        # Terrain effect
        slip_factor *= (1.0 - terrain.friction_coefficient)

        return slip_factor

    def _update_power_consumption(self, dt: float):
        """Update power consumption model."""
        # Calculate power for each wheel
        total_power = 0.0
        for i, (speed, torque) in enumerate(zip(self.wheel_speeds, self.wheel_torques)):
            # Mechanical power
            mechanical_power = abs(speed * torque)

            # Electrical power (accounting for motor efficiency ~80%)
            electrical_power = mechanical_power / 0.8

            # Add controller power consumption
            controller_power = 5.0  # Watts per controller

            total_power += electrical_power + controller_power

        # Convert to energy (Wh)
        energy_consumption = total_power * dt / 3600.0
        self.total_power_consumption += energy_consumption

        # Update battery level (simplified model)
        battery_capacity_wh = 1000.0  # 1000Wh battery
        self.battery_level = max(0.0, 100.0 - (self.total_power_consumption / battery_capacity_wh * 100.0))

        # Low battery warning
        if self.battery_level < 20.0 and not self.faults["battery_low"]:
            self.faults["battery_low"] = True
            self.logger.warning("⚠️ LOW BATTERY WARNING")

    def _update_thermal_model(self, dt: float):
        """Update motor thermal model."""
        for i in range(6):
            # Heat generation from current
            current = abs(self.wheel_torques[i]) / 10.0  # Estimate current from torque
            heat_generation = current * current * 0.5  # Simplified heating

            # Heat dissipation (convection)
            heat_dissipation = (self.motor_temperatures[i] - 25.0) * 2.0

            # Update temperature
            temp_change = (heat_generation - heat_dissipation) * dt / 10.0  # Thermal mass
            self.motor_temperatures[i] += temp_change

            # Temperature limits
            if self.motor_temperatures[i] > 80.0:
                if f"motor_{i}" not in self.faults["motor_failures"]:
                    self.faults["motor_failures"].append(f"motor_{i}")
                    self.logger.error(f"🚨 MOTOR {i} OVERHEAT")

    def _check_faults(self):
        """Check for system faults."""
        # Motor temperature faults
        for i, temp in enumerate(self.motor_temperatures):
            if temp > 90.0 and f"motor_{i}" not in self.faults["motor_failures"]:
                self.faults["motor_failures"].append(f"motor_{i}")
                self.logger.error(f"🚨 MOTOR {i} CRITICAL TEMPERATURE")

        # Battery fault
        if self.battery_level < 5.0:
            self.emergency_stop = True
            self.logger.critical("🚨 BATTERY CRITICAL - EMERGENCY STOP")

        # Random fault simulation (very low probability for testing)
        if np.random.random() < 0.00001:  # Very rare
            fault_type = np.random.choice(["encoder", "steering"])
            if fault_type == "encoder":
                wheel_idx = np.random.randint(6)
                fault_name = f"encoder_{wheel_idx}"
                if fault_name not in self.faults["encoder_failures"]:
                    self.faults["encoder_failures"].append(fault_name)
                    self.logger.error(f"🚨 ENCODER {wheel_idx} FAILURE SIMULATED")
            elif fault_type == "steering":
                if not self.faults["steering_failures"]:
                    self.faults["steering_failures"].append("steering_actuator")
                    self.logger.error("🚨 STEERING FAILURE SIMULATED")
