"""Base rover interface for simulation.

Provides common interface and functionality for all simulated rovers
including kinematics, dynamics, and control systems.

Author: URC 2026 Autonomy Team
"""

import logging
from abc import ABC, abstractmethod
from typing import Any, Dict, List, Optional, Tuple

import numpy as np


class BaseRover(ABC):
    """Abstract base class for all simulated rovers.

    Provides common functionality for:
    - Kinematic modeling
    - Dynamic simulation
    - Control input processing
    - State tracking
    - Performance monitoring
    """

    def __init__(self, config: Dict[str, Any]):
        """Initialize base rover.

        Args:
            config: Rover configuration dictionary
        """
        self.logger = logging.getLogger(f"{__name__}.{self.__class__.__name__}")

        # Configuration
        self.model_name = config.get("model", "unknown_rover")
        self.mass = config.get("mass", 50.0)  # kg
        self.wheel_count = config.get("wheel_count", 4)
        self.wheelbase = config.get("wheelbase", 0.5)  # meters
        self.track_width = config.get("track_width", 0.4)  # meters

        # Performance limits
        self.max_velocity = config.get("max_velocity", 2.0)  # m/s
        self.max_acceleration = config.get("max_acceleration", 1.0)  # m/s²
        self.max_angular_velocity = config.get("max_angular_velocity", 1.0)  # rad/s

        # Physical properties
        self.friction_coefficient = config.get("friction_coefficient", 0.8)
        self.drag_coefficient = config.get("drag_coefficient", 0.3)
        self.rolling_resistance = config.get("rolling_resistance", 0.05)

        # State
        self.position = np.array([0.0, 0.0, 0.0])  # x, y, z
        self.velocity = np.array([0.0, 0.0, 0.0])  # vx, vy, vz
        self.orientation = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw
        self.angular_velocity = np.array([0.0, 0.0, 0.0])  # ωx, ωy, ωz

        # Control inputs
        self.left_velocity_cmd = 0.0
        self.right_velocity_cmd = 0.0
        self.steering_angle_cmd = 0.0

        # Motor state
        self.motor_temperatures: List[float] = [25.0] * self.wheel_count
        self.motor_currents: List[float] = [0.0] * self.wheel_count
        self.motor_enabled: List[bool] = [True] * self.wheel_count

        # Battery state
        self.battery_voltage = 24.0
        self.battery_current = 0.0
        self.battery_percentage = 100.0

        # Performance tracking
        self.distance_traveled = 0.0
        self.energy_consumed = 0.0
        self.control_commands_processed = 0

        self.logger.info(
            f"Initialized {self.__class__.__name__} ({self.wheel_count} wheels, "
            f"{self.mass}kg, max {self.max_velocity}m/s)"
        )

    @abstractmethod
    def _kinematics_step(self, dt: float, control_inputs: Dict[str, Any]) -> Dict[str, Any]:
        """Execute kinematic simulation step.

        Args:
            dt: Time step in seconds
            control_inputs: Control commands

        Returns:
            Dict with kinematic state updates
        """
        pass

    @abstractmethod
    def _dynamics_step(self, dt: float, environment: Dict[str, Any]) -> Dict[str, Any]:
        """Execute dynamic simulation step.

        Args:
            dt: Time step in seconds
            environment: Environment conditions

        Returns:
            Dict with dynamic state updates
        """
        pass

    def step(self, dt: float, sensor_data: Dict[str, Any]) -> Dict[str, Any]:
        """Execute complete rover simulation step.

        Args:
            dt: Time step in seconds
            sensor_data: Current sensor readings

        Returns:
            Dict containing rover state and outputs
        """
        try:
            # Process control inputs from sensor data
            control_inputs = self._process_sensor_data(sensor_data)

            # Execute kinematic simulation
            kinematic_state = self._kinematics_step(dt, control_inputs)

            # Execute dynamic simulation (with environment effects)
            environment = sensor_data.get("environment", {})
            dynamic_state = self._dynamics_step(dt, environment)

            # Update motor states
            motor_state = self._update_motor_states(dt, control_inputs)

            # Update battery state
            battery_state = self._update_battery_state(dt, control_inputs)

            # Update position tracking
            self._update_position_tracking(dt)

            # Combine all state
            rover_state = {
                "position": self.position.tolist(),
                "velocity": self.velocity.tolist(),
                "orientation": self.orientation.tolist(),
                "angular_velocity": self.angular_velocity.tolist(),
                "motor_temperatures": self.motor_temperatures.copy(),
                "motor_currents": self.motor_currents.copy(),
                "battery_voltage": self.battery_voltage,
                "battery_current": self.battery_current,
                "battery_percentage": self.battery_percentage,
                "distance_traveled": self.distance_traveled,
                "energy_consumed": self.energy_consumed,
                "control_commands_processed": self.control_commands_processed,
                "timestamp": sensor_data.get("timestamp", 0.0),
                "is_simulation": True,
            }

            return rover_state

        except Exception as e:
            self.logger.error(f"Rover simulation step failed: {e}")
            return self._create_error_state(str(e))

    def reset(self):
        """Reset rover to initial state."""
        self.position = np.array([0.0, 0.0, 0.0])
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0])
        self.angular_velocity = np.array([0.0, 0.0, 0.0])

        self.left_velocity_cmd = 0.0
        self.right_velocity_cmd = 0.0
        self.steering_angle_cmd = 0.0

        self.motor_temperatures = [25.0] * self.wheel_count
        self.motor_currents = [0.0] * self.wheel_count
        self.motor_enabled = [True] * self.wheel_count

        self.battery_voltage = 24.0
        self.battery_current = 0.0
        self.battery_percentage = 100.0

        self.distance_traveled = 0.0
        self.energy_consumed = 0.0
        self.control_commands_processed = 0

        self.logger.info(f"Reset {self.model_name}")

    def get_state(self) -> Dict[str, Any]:
        """Get current rover state.

        Returns:
            Dict containing complete rover state
        """
        return {
            "model_name": self.model_name,
            "mass": self.mass,
            "wheel_count": self.wheel_count,
            "position": self.position.tolist(),
            "velocity": self.velocity.tolist(),
            "orientation": self.orientation.tolist(),
            "angular_velocity": self.angular_velocity.tolist(),
            "motor_temperatures": self.motor_temperatures.copy(),
            "motor_currents": self.motor_currents.copy(),
            "motor_enabled": self.motor_enabled.copy(),
            "battery_voltage": self.battery_voltage,
            "battery_current": self.battery_current,
            "battery_percentage": self.battery_percentage,
            "distance_traveled": self.distance_traveled,
            "energy_consumed": self.energy_consumed,
            "control_commands_processed": self.control_commands_processed,
        }

    def set_control_inputs(
        self,
        linear_velocity: float = 0.0,
        angular_velocity: float = 0.0,
        steering_angle: float = 0.0
    ):
        """Set rover control inputs.

        Args:
            linear_velocity: Desired linear velocity (m/s)
            angular_velocity: Desired angular velocity (rad/s)
            steering_angle: Steering angle (radians, for Ackermann steering)
        """
        # Clamp to physical limits
        self.left_velocity_cmd = np.clip(linear_velocity, -self.max_velocity, self.max_velocity)
        self.right_velocity_cmd = np.clip(linear_velocity, -self.max_velocity, self.max_velocity)
        self.steering_angle_cmd = np.clip(steering_angle, -0.5, 0.5)  # ±0.5 radians

        self.control_commands_processed += 1

    def emergency_stop(self):
        """Execute emergency stop."""
        self.logger.warning("Emergency stop activated")

        # Immediately set all velocities to zero
        self.left_velocity_cmd = 0.0
        self.right_velocity_cmd = 0.0
        self.steering_angle_cmd = 0.0

        # Disable motors (simulated)
        self.motor_enabled = [False] * self.wheel_count

    def _process_sensor_data(self, sensor_data: Dict[str, Any]) -> Dict[str, Any]:
        """Process sensor data for control inputs.

        Args:
            sensor_data: Current sensor readings

        Returns:
            Dict with processed control inputs
        """
        # Base implementation - extract basic control commands
        # Subclasses can override for more sophisticated processing

        control_inputs = {
            "linear_velocity_cmd": 0.0,
            "angular_velocity_cmd": 0.0,
            "steering_angle_cmd": 0.0,
        }

        # Look for velocity commands in sensor data (from autonomy system)
        if "velocity_command" in sensor_data:
            vel_cmd = sensor_data["velocity_command"]
            control_inputs["linear_velocity_cmd"] = vel_cmd.get("linear", 0.0)
            control_inputs["angular_velocity_cmd"] = vel_cmd.get("angular", 0.0)

        return control_inputs

    def _update_motor_states(self, dt: float, control_inputs: Dict[str, Any]) -> Dict[str, Any]:
        """Update motor states based on control inputs.

        Args:
            dt: Time step
            control_inputs: Control commands

        Returns:
            Dict with motor state updates
        """
        # Simulate motor heating based on current
        for i in range(self.wheel_count):
            if self.motor_enabled[i]:
                # Calculate current based on commanded velocity
                commanded_vel = control_inputs.get("linear_velocity_cmd", 0.0)
                self.motor_currents[i] = abs(commanded_vel) * 2.0  # Simplified

                # Heat generation (simplified)
                heat_generated = self.motor_currents[i] * 0.1 * dt
                heat_dissipated = (self.motor_temperatures[i] - 25.0) * 0.05 * dt
                self.motor_temperatures[i] += heat_generated - heat_dissipated

                # Temperature limits
                self.motor_temperatures[i] = np.clip(self.motor_temperatures[i], 0, 100)
            else:
                self.motor_currents[i] = 0.0
                self.motor_temperatures[i] = max(25.0, self.motor_temperatures[i] - 0.1 * dt)

        return {
            "temperatures": self.motor_temperatures.copy(),
            "currents": self.motor_currents.copy(),
            "enabled": self.motor_enabled.copy(),
        }

    def _update_battery_state(self, dt: float, control_inputs: Dict[str, Any]) -> Dict[str, Any]:
        """Update battery state based on power consumption.

        Args:
            dt: Time step
            control_inputs: Control commands

        Returns:
            Dict with battery state updates
        """
        # Calculate power consumption
        total_current = sum(self.motor_currents)
        power_consumption = total_current * self.battery_voltage  # Watts

        # Update energy consumption
        energy_used = power_consumption * dt  # Joules
        self.energy_consumed += energy_used

        # Update battery percentage (simplified model)
        battery_capacity_wh = 500  # 500Wh battery
        energy_used_wh = energy_used / 3600  # Convert to Wh
        self.battery_percentage = max(0, self.battery_percentage - (energy_used_wh / battery_capacity_wh) * 100)

        # Update voltage (simplified discharge curve)
        self.battery_voltage = 24.0 * (0.9 + 0.1 * (self.battery_percentage / 100.0))
        self.battery_current = total_current

        return {
            "voltage": self.battery_voltage,
            "current": self.battery_current,
            "percentage": self.battery_percentage,
        }

    def _update_position_tracking(self, dt: float):
        """Update position tracking metrics.

        Args:
            dt: Time step
        """
        # Calculate distance traveled
        velocity_magnitude = np.linalg.norm(self.velocity[:2])  # Ignore vertical velocity
        self.distance_traveled += velocity_magnitude * dt

    def _create_error_state(self, error_msg: str) -> Dict[str, Any]:
        """Create error state response.

        Args:
            error_msg: Error message

        Returns:
            Dict with error state
        """
        return {
            "error": error_msg,
            "position": self.position.tolist(),
            "velocity": self.velocity.tolist(),
            "motor_temperatures": self.motor_temperatures.copy(),
            "battery_percentage": self.battery_percentage,
            "is_simulation": True,
        }

    def get_performance_metrics(self) -> Dict[str, Any]:
        """Get rover performance metrics.

        Returns:
            Dict with performance data
        """
        return {
            "distance_traveled_km": self.distance_traveled / 1000,
            "energy_consumed_wh": self.energy_consumed / 3600,
            "average_velocity_m_s": self.distance_traveled / max(1, self.control_commands_processed * 0.01),
            "efficiency_wh_per_km": (
                (self.energy_consumed / 3600) / max(0.001, self.distance_traveled / 1000)
                if self.distance_traveled > 0
                else 0
            ),
            "motor_temperature_max": max(self.motor_temperatures) if self.motor_temperatures else 0,
            "battery_discharge_rate_percent_per_hour": (
                (100 - self.battery_percentage) / max(0.001, self.distance_traveled / (self.max_velocity * 3600)) * 100
                if self.distance_traveled > 0
                else 0
            ),
        }
