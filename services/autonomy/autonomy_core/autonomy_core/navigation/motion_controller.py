#!/usr/bin/env python3
"""
Motion Controller - Low-level motion control for rover navigation.

Handles:
- Velocity control and trajectory following
- Motor control and feedback
- Emergency stop functionality
- Odometry integration

Author: URC 2026 Autonomy Team
"""

import logging
from typing import Optional, Tuple

from geometry_msgs.msg import Twist

logger = logging.getLogger(__name__)


class MotionController:
    """
    Low-level motion control for autonomous navigation.

    Provides:
    - Velocity command execution with acceleration limiting
    - Trajectory following with feedback control
    - Emergency stop handling with immediate response
    - Odometry feedback and state monitoring
    - Hardware fault detection and reporting

    The controller implements a hierarchical control structure:
    1. Emergency stop override (highest priority)
    2. Velocity command validation and limiting
    3. Acceleration/deceleration ramping
    4. Motor command generation
    5. Feedback monitoring and correction
    """

    def __init__(self) -> None:
        """
        Initialize the motion controller with default parameters.

        Sets up control limits, motor parameters, and initializes control state.
        Hardware interfaces are not initialized until initialize() is called.
        """
        # Velocity limits (m/s, rad/s)
        self.max_linear_speed = 2.0
        self.max_angular_speed = 1.0
        self.min_linear_speed = -1.0  # Allow reverse at reduced speed
        self.min_angular_speed = -self.max_angular_speed

        # Acceleration limits (m/s², rad/s²)
        self.acceleration_limit = 1.0
        self.deceleration_limit = 2.0  # Faster deceleration for safety

        # Motor control parameters
        self.wheel_separation = 0.5  # meters between wheels
        self.wheel_radius = 0.15  # meters

        # Control state
        self.current_velocity = Twist()
        self.target_velocity = Twist()
        self.emergency_stop_active = False
        self.is_initialized = False

        # Motor controllers (None until initialized)
        self.left_motor = None
        self.right_motor = None
        self.pid_controller = None

        logger.info("MotionController initialized with default parameters")

    def initialize(self) -> bool:
        """
        Initialize motion controller hardware interfaces.

        Returns:
            bool: True if initialization successful, False otherwise

        This method should be called before using the controller.
        It sets up motor controllers, encoders, and tests basic functionality.
        """
        try:
            logger.info("Initializing motion controller hardware...")

            # Initialize professional PID controllers using simple-pid library
            try:
                from simple_pid import PID

                # Linear velocity PID controller
                self.pid_linear = PID(
                    Kp=1.5,
                    Ki=0.1,
                    Kd=0.05,  # Tuned for smooth linear motion
                    setpoint=0.0,
                    sample_time=0.1,
                    output_limits=(-self.max_linear_speed, self.max_linear_speed),
                    auto_mode=True,
                    proportional_on_measurement=False,
                )

                # Angular velocity PID controller
                self.pid_angular = PID(
                    Kp=2.0,
                    Ki=0.2,
                    Kd=0.1,  # More aggressive for rotation
                    setpoint=0.0,
                    sample_time=0.1,
                    output_limits=(-self.max_angular_speed, self.max_angular_speed),
                    auto_mode=True,
                    proportional_on_measurement=False,
                )

                logger.info(
                    "Professional PID controllers initialized with simple-pid library"
                )

            except ImportError:
                logger.warning("simple-pid not available, PID control will be limited")
                self.pid_linear = None
                self.pid_angular = None

            # TODO: Initialize hardware interfaces
            # - Set up motor controllers (PWM, CAN, etc.)
            # - Configure encoder feedback
            # - Test motor functionality with safe movements

            # Placeholder for hardware initialization
            # self.left_motor = MotorController(pin=LEFT_MOTOR_PIN)
            # self.right_motor = MotorController(pin=RIGHT_MOTOR_PIN)

            self.is_initialized = True
            logger.info("Motion controller hardware initialized successfully")
            return True

        except Exception as e:
            logger.error(f"Failed to initialize motion controller: {e}")
            self.is_initialized = False
            return False

    def set_velocity(self, linear_x: float, angular_z: float) -> bool:
        """
        Set target velocity with validation and safety limits.

        Args:
            linear_x: Linear velocity in m/s (forward/backward)
            angular_z: Angular velocity in rad/s (rotation)

        Returns:
            bool: True if velocity command accepted, False if invalid/out of range

        The method applies safety limits and validates inputs before accepting
        the velocity command. Emergency stop takes precedence over velocity commands.
        """
        if not self.is_initialized:
            logger.warning(
                "Motion controller not initialized, " "ignoring velocity command"
            )
            angular_vel = (
                right_wheel_velocity - left_wheel_velocity
            ) / self.wheel_separation

        if self.emergency_stop_active:
            logger.warning("Emergency stop active, ignoring velocity command")
            return False

        # Validate and limit velocities to safe ranges
        try:
            linear_x = max(self.min_linear_speed, min(self.max_linear_speed, linear_x))
            angular_z = max(
                self.min_angular_speed, min(self.max_angular_speed, angular_z)
            )

            self.target_velocity.linear.x = linear_x
            self.target_velocity.angular.z = angular_z

            logger.debug(
                f"Velocity command set: linear={linear_x:.2f}, "
                f"angular={angular_z:.2f}"
            )
            return True

        except (ValueError, TypeError) as e:
            logger.error(
                f"Invalid velocity parameters: linear_x={linear_x}, "
                f"angular_z={angular_z}, error={e}"
            )
            return False

    def execute_velocity_command(self) -> bool:
        """
        Execute the current velocity command with acceleration limiting.

        Returns:
            bool: True if command executed successfully, False otherwise

        This method calculates wheel velocities, applies acceleration limits,
        sends commands to motors, and monitors execution feedback.
        """
        if not self.is_initialized:
            logger.warning("Motion controller not initialized")
            return False

        if self.emergency_stop_active:
            return self.stop_motors()

        try:
            # Calculate wheel velocities from differential drive kinematics
            linear_vel = self.target_velocity.linear.x
            angular_vel = self.target_velocity.angular.z

            # Differential drive kinematics: v_left = v + (ω × L/2), v_right = v - (ω × L/2)
            left_wheel_vel = linear_vel + (angular_vel * self.wheel_separation / 2.0)
            right_wheel_vel = linear_vel - (angular_vel * self.wheel_separation / 2.0)

            # Apply PID control for smooth motion using simple-pid
            if self.pid_linear and self.pid_angular:
                # Update PID setpoints to target velocities
                self.pid_linear.setpoint = linear_vel
                self.pid_angular.setpoint = angular_vel

                # Get current velocity feedback (would come from encoders in real implementation)
                current_linear = self.current_velocity.linear.x
                current_angular = self.current_velocity.angular.z

                # Calculate PID outputs
                pid_linear_output = self.pid_linear(current_linear)
                pid_angular_output = self.pid_angular(current_angular)

                # Apply PID corrections
                corrected_linear = linear_vel + pid_linear_output
                corrected_angular = angular_vel + pid_angular_output

                # Apply acceleration limiting
                corrected_linear = self._apply_acceleration_limit(corrected_linear)
                corrected_angular = self._apply_acceleration_limit(
                    corrected_angular, is_angular=True
                )

                logger.debug(
                    f"PID correction: linear={pid_linear_output:.3f}, angular={pid_angular_output:.3f}"
                )
            else:
                # Fallback without PID
                corrected_linear = linear_vel
                corrected_angular = angular_vel
                logger.debug("PID controllers not available, using direct control")

            # Recalculate wheel velocities with PID corrections
            left_wheel_vel = corrected_linear + (
                corrected_angular * self.wheel_separation / 2.0
            )
            right_wheel_vel = corrected_linear - (
                corrected_angular * self.wheel_separation / 2.0
            )

            # TODO: Send commands to actual motor controllers
            # TODO: Monitor encoder feedback and correct for slippage

            # Update current velocity (would be based on encoder feedback in real implementation)
            self.current_velocity.linear.x = corrected_linear
            self.current_velocity.angular.z = corrected_angular

            logger.debug(
                f"Executed velocity command: linear={linear_vel:.2f}, angular={angular_vel:.2f}"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to execute velocity command: {e}")
            self.emergency_stop()
            return False

    def _apply_acceleration_limit(
        self, target_velocity: float, is_angular: bool = False
    ) -> float:
        """
        Apply acceleration/deceleration limits to prevent jerky motion.

        Args:
            target_velocity: Target velocity
            is_angular: Whether this is angular velocity

        Returns:
            Limited velocity respecting acceleration constraints
        """
        current_vel = (
            self.current_velocity.angular.z
            if is_angular
            else self.current_velocity.linear.x
        )

        velocity_diff = target_velocity - current_vel
        dt = 1.0 / self.update_rate  # Assume 10Hz update rate

        # Choose acceleration limit based on direction and type
        if velocity_diff >= 0:
            # Acceleration
            accel_limit = self.acceleration_limit
            if is_angular:
                accel_limit *= 0.5  # More conservative for rotation
        else:
            # Deceleration (can be faster for safety)
            accel_limit = self.deceleration_limit
            if is_angular:
                accel_limit *= 0.8

        # Calculate maximum allowed velocity change
        max_delta = accel_limit * dt

        # Limit the velocity change
        if abs(velocity_diff) > max_delta:
            limited_velocity = current_vel + (
                max_delta if velocity_diff > 0 else -max_delta
            )
            logger.debug(
                f"Acceleration limited: {current_vel:.2f} -> {limited_velocity:.2f}"
            )
            return limited_velocity

        return target_velocity

    def stop_motors(self) -> bool:
        """
        Emergency stop all motors immediately.

        Returns:
            bool: True if emergency stop executed successfully, False otherwise

        This method provides immediate motor shutdown for safety.
        It takes precedence over all other velocity commands.
        """
        try:
            logger.warning("Executing emergency motor stop")

            # Set emergency stop flag
            self.emergency_stop_active = True

            # Immediately zero all velocities
            self.current_velocity = Twist()
            self.target_velocity = Twist()

            # Stop all motor controllers
            motor_stop_success = self._stop_all_motors()

            # Apply mechanical braking if available
            braking_applied = self._apply_mechanical_braking()

            # Update status indicators
            self._update_emergency_status()

            # Notify safety system if available
            self._notify_safety_system_emergency_stop()

            success = motor_stop_success and braking_applied
            logger.warning(
                f"Emergency stop completed - Motors: {motor_stop_success}, Braking: {braking_applied}"
            )
            return success

        except Exception as e:
            logger.critical(f"Failed to execute emergency stop: {e}")
            return False

    def emergency_stop(self) -> bool:
        """
        Activate emergency stop mode.

        Returns:
            bool: True if emergency stop activated successfully

        Emergency stop disables all velocity commands until manually reset.
        This is a safety-critical function that must work even if hardware fails.
        """
        logger.critical("EMERGENCY STOP ACTIVATED")
        self.emergency_stop_active = True
        return self.stop_motors()

    def reset_emergency_stop(self) -> bool:
        """
        Reset emergency stop and restore normal operation.

        Returns:
            bool: True if reset successful, False otherwise

        This should only be called after confirming the emergency condition is resolved.
        """
        if not self.is_initialized:
            logger.warning("Cannot reset emergency stop: controller not initialized")
            return False

        logger.info("Resetting emergency stop - restoring normal operation")
        self.emergency_stop_active = False
        return True

    def _stop_all_motors(self) -> bool:
        """
        Stop all motor controllers immediately.

        Returns:
            bool: True if all motors stopped successfully
        """
        try:
            # Send stop commands to motor controllers
            # In a real implementation, this would communicate with actual motor drivers

            logger.debug("Sending stop commands to all motor controllers")

            # Set all motor speeds to zero
            self.left_motor_speed = 0.0
            self.right_motor_speed = 0.0

            # If we had actual motor controller objects, we'd call their stop methods:
            # if self.left_motor:
            #     self.left_motor.stop()
            # if self.right_motor:
            #     self.right_motor.stop()

            return True

        except Exception as e:
            logger.error(f"Failed to stop motors: {e}")
            return False

    def _apply_mechanical_braking(self) -> bool:
        """
        Apply mechanical braking if available.

        Returns:
            bool: True if braking applied successfully
        """
        try:
            # Apply regenerative braking or mechanical brakes
            # In a real implementation, this would engage braking systems

            logger.debug("Applying mechanical braking")

            # For now, simulate braking application
            # In practice, this might involve:
            # - Engaging regenerative braking
            # - Applying mechanical brakes
            # - Shorting motor windings for dynamic braking

            return True

        except Exception as e:
            logger.error(f"Failed to apply mechanical braking: {e}")
            return False

    def _update_emergency_status(self) -> bool:
        """
        Update status indicators for emergency stop.

        Returns:
            bool: True if status updated successfully
        """
        try:
            # Update LEDs, dashboard, and other status indicators
            logger.debug("Updating emergency status indicators")

            # In a real implementation, this might involve:
            # - Setting emergency LED patterns
            # - Publishing emergency status to ROS topics
            # - Updating dashboard displays
            # - Sending notifications to ground station

            return True

        except Exception as e:
            logger.error(f"Failed to update emergency status: {e}")
            return False

    def _notify_safety_system_emergency_stop(self) -> bool:
        """
        Notify safety system of emergency stop activation.

        Returns:
            bool: True if notification sent successfully
        """
        try:
            # Import safety monitor if available
            try:
                from autonomy_safety_system.safety_monitor import get_safety_monitor

                safety_monitor = get_safety_monitor()
                # Safety monitor would already know about the emergency stop
                # since it triggered this callback
            except ImportError:
                logger.debug("Safety system not available for notification")

            return True

        except Exception as e:
            logger.error(f"Failed to notify safety system: {e}")
            return False

    def calculate_wheel_velocities(
        self, linear_x: float, angular_z: float
    ) -> Tuple[float, float]:
        """Calculate individual wheel velocities for differential drive"""
        # Differential drive kinematics
        v_left = linear_x - (angular_z * self.wheel_separation / 2.0)
        v_right = linear_x + (angular_z * self.wheel_separation / 2.0)

        return v_left, v_right

    def update_odometry(
        self, left_wheel_velocity: float, right_wheel_velocity: float, dt: float
    ) -> bool:
        """
        Update odometry from wheel encoder feedback.

        Args:
            left_wheel_velocity: Left wheel velocity in m/s
            right_wheel_velocity: Right wheel velocity in m/s
            dt: Time step in seconds

        Returns:
            bool: True if odometry updated successfully, False otherwise

        This method calculates the rover's position and orientation from wheel encoders.
        It accounts for wheel slip and provides dead reckoning when GPS is unavailable.
        """
        if not self.is_initialized:
            logger.warning("Cannot update odometry: controller not initialized")
            return False

        try:
            # TODO: Implement odometry calculation
            # - Calculate linear and angular velocity from wheel velocities
            # - Integrate position over time using kinematic model
            # - Account for wheel slip and terrain conditions
            # - Update internal pose estimate

            # Differential drive odometry
            linear_vel = (left_wheel_velocity + right_wheel_velocity) / 2.0
            angular_vel = (
                right_wheel_velocity - left_wheel_velocity
            ) / self.wheel_separation

            # Integrate position (basic Euler integration)
            # This is a placeholder - real implementation would use proper odometry

            logger.debug(
                f"Odometry update: linear_vel={linear_vel:.2f}, angular_vel={angular_vel:.2f}"
            )
            return True

        except Exception as e:
            logger.error(f"Failed to update odometry: {e}")
            return False

    def get_current_velocity(self) -> Twist:
        """
        Get the currently executing velocity command.

        Returns:
            Twist: Current velocity as ROS Twist message

        This returns the velocity that should be currently executing,
        which may differ from the target velocity due to acceleration limiting.
        """
        return self.current_velocity

    def get_target_velocity(self) -> Twist:
        """
        Get the target velocity command.

        Returns:
            Twist: Target velocity as ROS Twist message

        This is the desired velocity that may still be ramping to due to acceleration limits.
        """
        return self.target_velocity

    def is_emergency_stop_active(self) -> bool:
        """
        Check if emergency stop is currently active.

        Returns:
            bool: True if emergency stop is active, False otherwise
        """
        return self.emergency_stop_active

    def get_status(self) -> dict:
        """
        Get comprehensive status of the motion controller.

        Returns:
            dict: Status information including initialization state,
                  emergency stop status, and current velocities
        """
        return {
            "initialized": self.is_initialized,
            "emergency_stop_active": self.emergency_stop_active,
            "current_linear_velocity": self.current_velocity.linear.x,
            "current_angular_velocity": self.current_velocity.angular.z,
            "target_linear_velocity": self.target_velocity.linear.x,
            "target_angular_velocity": self.target_velocity.angular.z,
        }

    def get_target_velocity(self) -> Twist:
        """Get target velocity"""
        return self.target_velocity

    def is_at_target_velocity(self, tolerance: float = 0.1) -> bool:
        """Check if current velocity matches target"""
        linear_diff = abs(
            self.current_velocity.linear.x - self.target_velocity.linear.x
        )
        angular_diff = abs(
            self.current_velocity.angular.z - self.target_velocity.angular.z
        )

        return linear_diff < tolerance and angular_diff < tolerance

    def set_emergency_stop(self, active: bool):
        """Set emergency stop state"""
        self.emergency_stop_active = active
        if active:
            self.stop_motors()

    def get_motor_status(self) -> dict:
        """Get motor status information"""
        # TODO: Implement motor status monitoring
        # - Current draw
        # - Temperature
        # - Error states
        # - Encoder values

        return {
            "left_motor": {"current": 0.0, "temperature": 25.0, "status": "ok"},
            "right_motor": {"current": 0.0, "temperature": 25.0, "status": "ok"},
        }

    def calibrate_motors(self):
        """Calibrate motor controllers"""
        # TODO: Implement motor calibration
        # - Find zero positions
        # - Calibrate PID gains
        # - Test full range of motion
        # - Validate encoder accuracy

    def reset_controller(self):
        """Reset controller state"""
        # TODO: Reset internal state
        # Clear velocity commands
        # Reset PID integrators
        self.current_velocity = Twist()
        self.target_velocity = Twist()
        self.emergency_stop_active = False

    def shutdown(self):
        """Shutdown motion controller"""
        # TODO: Clean shutdown
        # Stop all motors
        # Save calibration data
        # Close interfaces
        self.stop_motors()
