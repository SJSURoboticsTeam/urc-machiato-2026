"""URC 2026 rover simulation implementation.

Provides realistic rover physics and control for URC 2026 competition
scenarios including terrain effects, traction modeling, and performance
characteristics.

Author: URC 2026 Autonomy Team
"""

from typing import Any, Dict

import numpy as np

from simulation.rover.base_rover import BaseRover


class URCRover(BaseRover):
    """URC 2026 rover simulation with realistic physics.

    Implements differential drive kinematics with terrain effects,
    realistic motor characteristics, and URC-specific performance
    parameters.
    """

    def __init__(self, config: Dict[str, Any]):
        """Initialize URC rover.

        Args:
            config: URC rover configuration
        """
        # Set URC-specific defaults
        config.setdefault("model", "urc_rover")
        config.setdefault("mass", 75.0)  # kg
        config.setdefault("wheel_count", 6)
        config.setdefault("wheelbase", 0.8)  # meters (front to rear axle)
        config.setdefault("track_width", 0.6)  # meters (wheel to wheel)
        config.setdefault("max_velocity", 1.5)  # m/s
        config.setdefault("max_acceleration", 0.8)  # m/s²
        config.setdefault("max_angular_velocity", 0.8)  # rad/s

        super().__init__(config)

        # URC-specific parameters
        self.wheel_radius = config.get("wheel_radius", 0.15)  # meters
        self.gear_ratio = config.get("gear_ratio", 50.0)  # motor to wheel
        self.motor_max_torque = config.get("motor_max_torque", 10.0)  # Nm
        self.motor_max_rpm = config.get("motor_max_rpm", 3000)

        # Terrain interaction
        self.ground_clearance = config.get("ground_clearance", 0.2)  # meters
        self.suspension_travel = config.get("suspension_travel", 0.1)  # meters

        # Initialize wheel states (6 wheels for URC rover)
        self.wheel_positions = np.zeros((6, 3))  # x, y, z for each wheel
        self.wheel_velocities = np.zeros(6)  # m/s
        self.wheel_torques = np.zeros(6)  # Nm
        self.wheel_slip_ratios = np.zeros(6)  # 0-1, 0=no slip

        # Initialize wheel positions (simplified)
        self._initialize_wheel_positions()

    def _kinematics_step(
        self, dt: float, control_inputs: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Execute differential drive kinematics step.

        Args:
            dt: Time step in seconds
            control_inputs: Control commands

        Returns:
            Dict with kinematic state updates
        """
        # Get commanded velocities
        linear_cmd = control_inputs.get("linear_velocity_cmd", 0.0)
        angular_cmd = control_inputs.get("angular_velocity_cmd", 0.0)

        # Apply velocity limits
        linear_cmd = np.clip(linear_cmd, -self.max_velocity, self.max_velocity)
        angular_cmd = np.clip(
            angular_cmd, -self.max_angular_velocity, self.max_angular_velocity
        )

        # Differential drive kinematics
        # Left and right wheel velocities
        track_half = self.track_width / 2
        if abs(angular_cmd) < 1e-6:
            # Straight line motion
            v_left = linear_cmd
            v_right = linear_cmd
        else:
            # Turning motion
            turning_radius = linear_cmd / angular_cmd
            v_left = angular_cmd * (turning_radius - track_half)
            v_right = angular_cmd * (turning_radius + track_half)

        # Update wheel velocities with acceleration limits
        max_wheel_accel = self.max_acceleration / self.wheel_radius

        for i in range(self.wheel_count):
            if i < 3:  # Left side wheels
                target_velocity = v_left
            else:  # Right side wheels
                target_velocity = v_right

            # Apply acceleration limit
            velocity_error = target_velocity - self.wheel_velocities[i]
            max_change = max_wheel_accel * dt
            velocity_change = np.clip(velocity_error, -max_change, max_change)

            self.wheel_velocities[i] += velocity_change

            # Calculate effective rover velocity from wheels
            # (accounting for slip and terrain effects - applied in dynamics)

        # Calculate rover velocity (simplified - assumes no slip initially)
        rover_linear_velocity = (
            self.wheel_velocities[:3].mean() + self.wheel_velocities[3:].mean()
        ) / 2
        rover_angular_velocity = (
            self.wheel_velocities[:3].mean() - self.wheel_velocities[3:].mean()
        ) / self.track_width

        # Update position and orientation
        self.orientation[2] += rover_angular_velocity * dt  # Yaw update

        # Velocity in world frame
        heading = self.orientation[2]
        self.velocity[0] = rover_linear_velocity * np.cos(heading)  # vx
        self.velocity[1] = rover_linear_velocity * np.sin(heading)  # vy

        # Update position
        self.position[0] += self.velocity[0] * dt
        self.position[1] += self.velocity[1] * dt

        return {
            "linear_velocity": rover_linear_velocity,
            "angular_velocity": rover_angular_velocity,
            "wheel_velocities": self.wheel_velocities.tolist(),
        }

    def _dynamics_step(self, dt: float, environment: Dict[str, Any]) -> Dict[str, Any]:
        """Execute rover dynamics with terrain and environmental effects.

        Args:
            dt: Time step in seconds
            environment: Environment conditions

        Returns:
            Dict with dynamic state updates
        """
        # Get terrain properties
        terrain_traction = environment.get("traction", 1.0)
        terrain_roughness = environment.get("terrain_difficulty", 0.0)
        slope_angle = environment.get("slope_angle", 0.0)

        # Environmental effects
        wind_speed = environment.get("wind_speed", 0.0)
        wind_direction = environment.get("wind_direction", 0.0)
        temperature = environment.get("temperature", 25.0)

        # Calculate forces on each wheel
        for i in range(self.wheel_count):
            # Base motor torque (simplified)
            commanded_velocity = self.wheel_velocities[i]
            max_torque = self.motor_max_torque * terrain_traction

            # Torque decreases with speed (simplified motor curve)
            speed_ratio = abs(commanded_velocity) / (
                self.motor_max_rpm * 2 * np.pi * self.wheel_radius / 60
            )
            available_torque = max_torque * max(0, 1 - speed_ratio)

            self.wheel_torques[i] = available_torque

            # Calculate wheel slip
            # Simplified: slip increases with low traction and high torque
            slip_factor = (1 - terrain_traction) + (
                available_torque / self.motor_max_torque
            ) * 0.3
            self.wheel_slip_ratios[i] = min(1.0, slip_factor)

            # Apply slip to effective velocity
            effective_velocity = commanded_velocity * (1 - self.wheel_slip_ratios[i])
            self.wheel_velocities[i] = effective_velocity

        # Calculate total forces on rover
        total_force_x = 0.0
        total_force_y = 0.0
        total_torque_z = 0.0

        # Wheel forces
        for i in range(self.wheel_count):
            wheel_force = self.wheel_torques[i] / self.wheel_radius
            wheel_angle = self._get_wheel_angle(i)

            # Force components
            force_x = wheel_force * np.cos(wheel_angle)
            force_y = wheel_force * np.sin(wheel_angle)

            total_force_x += force_x
            total_force_y += force_y

            # Torque from wheel position
            wheel_lever_arm = self._get_wheel_lever_arm(i)
            total_torque_z += wheel_force * wheel_lever_arm

        # Environmental forces
        # Rolling resistance
        rolling_force = -self.velocity[0] * self.rolling_resistance * self.mass * 9.81

        # Air drag
        drag_force = -self.velocity[0] * self.drag_coefficient * abs(self.velocity[0])

        # Wind force
        relative_wind_speed = wind_speed * np.cos(wind_direction - self.orientation[2])
        wind_force = relative_wind_speed * 0.5 * 1.225 * 0.5  # Simplified wind force

        # Gravity on slopes
        gravity_force = self.mass * 9.81 * np.sin(slope_angle)

        # Total forces
        total_force_x += rolling_force + drag_force + wind_force + gravity_force

        # Apply forces to velocity (F = ma)
        acceleration_x = total_force_x / self.mass
        acceleration_y = total_force_y / self.mass

        # Update velocities
        self.velocity[0] += acceleration_x * dt
        self.velocity[1] += acceleration_y * dt
        self.angular_velocity[2] += (
            total_torque_z / (self.mass * self.wheelbase / 2)
        ) * dt

        # Apply velocity limits
        speed = np.sqrt(self.velocity[0] ** 2 + self.velocity[1] ** 2)
        if speed > self.max_velocity:
            self.velocity[0] *= self.max_velocity / speed
            self.velocity[1] *= self.max_velocity / speed

        return {
            "acceleration_x": acceleration_x,
            "acceleration_y": acceleration_y,
            "terrain_traction": terrain_traction,
            "wheel_slip_ratios": self.wheel_slip_ratios.tolist(),
            "total_force_x": total_force_x,
            "total_force_y": total_force_y,
            "rolling_force": rolling_force,
            "drag_force": drag_force,
            "wind_force": wind_force,
        }

    def _initialize_wheel_positions(self):
        """Initialize wheel positions for 6-wheel rover."""
        # Front axle
        self.wheel_positions[0] = [
            -self.wheelbase / 2,
            self.track_width / 2,
            -self.wheel_radius,
        ]  # Front left
        self.wheel_positions[1] = [
            -self.wheelbase / 2,
            0,
            -self.wheel_radius,
        ]  # Front center
        self.wheel_positions[2] = [
            -self.wheelbase / 2,
            -self.track_width / 2,
            -self.wheel_radius,
        ]  # Front right

        # Rear axle
        self.wheel_positions[3] = [
            self.wheelbase / 2,
            self.track_width / 2,
            -self.wheel_radius,
        ]  # Rear left
        self.wheel_positions[4] = [
            self.wheelbase / 2,
            0,
            -self.wheel_radius,
        ]  # Rear center
        self.wheel_positions[5] = [
            self.wheelbase / 2,
            -self.track_width / 2,
            -self.wheel_radius,
        ]  # Rear right

    def _get_wheel_angle(self, wheel_index: int) -> float:
        """Get wheel steering angle.

        Args:
            wheel_index: Index of wheel (0-5)

        Returns:
            float: Wheel angle in radians
        """
        # Simplified: only front wheels steer
        if wheel_index < 3:  # Front wheels
            return self.steering_angle_cmd
        else:  # Rear wheels
            return 0.0

    def _get_wheel_lever_arm(self, wheel_index: int) -> float:
        """Get wheel lever arm for torque calculation.

        Args:
            wheel_index: Index of wheel (0-5)

        Returns:
            float: Lever arm distance from center
        """
        return self.wheel_positions[wheel_index][1]  # Y position (lateral distance)

    def get_urc_specific_metrics(self) -> Dict[str, Any]:
        """Get URC-specific performance metrics.

        Returns:
            Dict with URC-relevant metrics
        """
        base_metrics = self.get_performance_metrics()

        # Add URC-specific metrics
        urc_metrics = {
            "wheel_slip_average": np.mean(self.wheel_slip_ratios),
            "wheel_slip_max": np.max(self.wheel_slip_ratios),
            "motor_utilization": np.mean(self.motor_currents)
            / (self.motor_max_torque * self.gear_ratio),
            "thermal_margin": 100 - max(self.motor_temperatures),  # °C margin to limit
            "battery_range_estimate_km": (
                (
                    self.battery_percentage
                    / 100
                    * 500
                    * self.friction_coefficient
                    / max(0.001, base_metrics["efficiency_wh_per_km"])
                )
                if base_metrics["efficiency_wh_per_km"] > 0
                else 0
            ),
            "terrain_capability_score": (
                self.ground_clearance * 10
                + (1 - np.mean(self.wheel_slip_ratios)) * 50  # cm to score
                + min(  # traction score
                    50, self.suspension_travel * 1000
                )  # suspension score
            ),
        }

        return {**base_metrics, **urc_metrics}
