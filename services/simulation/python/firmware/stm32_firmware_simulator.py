#!/usr/bin/env python3
"""
STM32 Firmware Behavior Simulator

Simulates control-systems submodule firmware behavior for testing
without physical hardware.

Features:
- Motor velocity control loop simulation
- Encoder feedback generation
- Homing sequence simulation
- Emergency stop handling
- Fault injection (overcurrent, timeout, encoder failure)

Author: URC 2026 Simulation Team
"""

import logging
import time
import random
import math
from typing import Dict, Any, List, Optional, Tuple
from dataclasses import dataclass, field
from enum import Enum
import threading

logger = logging.getLogger(__name__)


class MotorFaultType(Enum):
    """Motor fault types."""

    NONE = "none"
    OVERCURRENT = "overcurrent"
    OVERTEMPERATURE = "overtemperature"
    ENCODER_FAILURE = "encoder_failure"
    TIMEOUT = "timeout"
    COMMUNICATION_ERROR = "communication_error"
    HARDWARE_FAULT = "hardware_fault"


class HomingState(Enum):
    """Homing sequence states."""

    IDLE = "idle"
    HOMING = "homing"
    COMPLETE = "complete"
    FAILED = "failed"


@dataclass
class MotorState:
    """State of a single motor."""

    motor_id: int
    velocity_setpoint: float = 0.0  # rad/s
    velocity_actual: float = 0.0  # rad/s
    position: float = 0.0  # rad
    current: float = 0.0  # A
    temperature: float = 25.0  # °C
    voltage: float = 24.0  # V
    fault: MotorFaultType = MotorFaultType.NONE
    enabled: bool = True
    last_update: float = field(default_factory=time.time)


@dataclass
class EncoderState:
    """Encoder sensor state."""

    encoder_id: int
    position: float = 0.0  # rad
    velocity: float = 0.0  # rad/s
    raw_counts: int = 0  # Encoder counts
    resolution: int = 4096  # Counts per revolution
    noise_std: float = 1.0  # Count noise
    valid: bool = True


class STM32FirmwareSimulator:
    """
    Simulates STM32 firmware behavior for rover control.

    Provides realistic motor control, encoder feedback, and fault simulation
    for testing communication interfaces without hardware.
    """

    # Motor parameters (realistic values for rover motors)
    MAX_VELOCITY = 10.0  # rad/s
    MAX_CURRENT = 5.0  # A
    MAX_TEMPERATURE = 80.0  # °C
    NOMINAL_VOLTAGE = 24.0  # V

    # Control loop parameters
    CONTROL_LOOP_HZ = 100  # 100Hz control loop
    VELOCITY_RAMP_RATE = 5.0  # rad/s²
    THERMAL_TIME_CONSTANT = 30.0  # seconds
    FRICTION_COEFFICIENT = 0.1

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """Initialize firmware simulator.

        Args:
            config: Configuration dictionary
        """
        self.logger = logging.getLogger(f"{__name__}.STM32FirmwareSimulator")

        # Configuration
        config = config or {}
        self.num_motors = config.get("num_motors", 6)  # 6 wheel motors
        self.simulate_faults = config.get("simulate_faults", True)
        self.fault_rate = config.get("fault_rate", 0.0001)  # 0.01% per update
        self.control_loop_hz = config.get("control_loop_hz", self.CONTROL_LOOP_HZ)

        # Motor states
        self.motors: Dict[int, MotorState] = {}
        for i in range(self.num_motors):
            self.motors[i] = MotorState(motor_id=i)

        # Encoder states
        self.encoders: Dict[int, EncoderState] = {}
        for i in range(self.num_motors):
            self.encoders[i] = EncoderState(encoder_id=i)

        # System state
        self.emergency_stop_active = False
        self.homing_state = HomingState.IDLE
        self.homing_progress = 0.0
        self.homing_start_time = 0.0
        self.homing_duration = config.get("homing_duration", 5.0)  # seconds

        # Control loop
        self.running = False
        self.control_thread: Optional[threading.Thread] = None
        self.last_control_update = time.time()

        # Statistics
        self.stats = {
            "control_cycles": 0,
            "commands_received": 0,
            "faults_injected": 0,
            "emergency_stops": 0,
            "homing_sequences": 0,
        }

        # Communication timeout
        self.last_command_time = time.time()
        self.command_timeout = config.get("command_timeout", 1.0)  # seconds

        self.logger.info(
            f"STM32 firmware simulator initialized with {self.num_motors} motors"
        )

    def start(self):
        """Start control loop thread."""
        if self.running:
            self.logger.warning("Control loop already running")
            return

        self.running = True
        self.control_thread = threading.Thread(target=self._control_loop, daemon=True)
        self.control_thread.start()
        self.logger.info("Control loop started")

    def stop(self):
        """Stop control loop thread."""
        if not self.running:
            return

        self.running = False
        if self.control_thread:
            self.control_thread.join(timeout=1.0)
        self.logger.info("Control loop stopped")

    def set_velocity_command(self, motor_id: int, velocity: float) -> bool:
        """Set velocity command for a motor.

        Args:
            motor_id: Motor identifier (0-5)
            velocity: Target velocity in rad/s

        Returns:
            bool: True if command accepted
        """
        if motor_id not in self.motors:
            self.logger.error(f"Invalid motor ID: {motor_id}")
            return False

        motor = self.motors[motor_id]

        # Check emergency stop
        if self.emergency_stop_active:
            self.logger.warning("Cannot set velocity: Emergency stop active")
            return False

        # Check for faults
        if motor.fault != MotorFaultType.NONE:
            self.logger.warning(f"Motor {motor_id} has fault: {motor.fault.value}")
            return False

        # Check velocity limits
        if abs(velocity) > self.MAX_VELOCITY:
            self.logger.warning(
                f"Velocity {velocity} exceeds limit {self.MAX_VELOCITY}"
            )
            velocity = max(-self.MAX_VELOCITY, min(self.MAX_VELOCITY, velocity))

        # Set velocity setpoint
        motor.velocity_setpoint = velocity
        self.last_command_time = time.time()
        self.stats["commands_received"] += 1

        self.logger.debug(f"Motor {motor_id} velocity setpoint: {velocity:.3f} rad/s")
        return True

    def set_chassis_velocities(
        self, linear_x: float, linear_y: float, angular_z: float
    ) -> bool:
        """Set chassis velocities (converts to wheel velocities).

        Args:
            linear_x: Forward velocity (m/s)
            linear_y: Lateral velocity (m/s)
            angular_z: Rotational velocity (rad/s)

        Returns:
            bool: True if command accepted
        """
        # Simplified differential drive model
        # For swerve drive, this would be more complex
        wheel_radius = 0.1  # meters
        wheelbase = 0.8  # meters

        # Convert to wheel velocities (simplified)
        # Front left/right
        v_left = (linear_x - angular_z * wheelbase / 2) / wheel_radius
        v_right = (linear_x + angular_z * wheelbase / 2) / wheel_radius

        # Set all wheels (simplified - same for front/mid/rear)
        success = True
        success &= self.set_velocity_command(0, v_left)  # Front left
        success &= self.set_velocity_command(1, v_right)  # Front right
        success &= self.set_velocity_command(2, v_left)  # Mid left
        success &= self.set_velocity_command(3, v_right)  # Mid right
        success &= self.set_velocity_command(4, v_left)  # Rear left
        success &= self.set_velocity_command(5, v_right)  # Rear right

        return success

    def emergency_stop(self) -> bool:
        """Activate emergency stop.

        Returns:
            bool: True if successful
        """
        self.emergency_stop_active = True
        self.stats["emergency_stops"] += 1

        # Stop all motors immediately
        for motor in self.motors.values():
            motor.velocity_setpoint = 0.0
            motor.velocity_actual = 0.0

        self.logger.warning("⚠️ EMERGENCY STOP ACTIVATED")
        return True

    def clear_emergency_stop(self) -> bool:
        """Clear emergency stop.

        Returns:
            bool: True if successful
        """
        self.emergency_stop_active = False
        self.logger.info("Emergency stop cleared")
        return True

    def start_homing_sequence(self) -> bool:
        """Start homing sequence.

        Returns:
            bool: True if started successfully
        """
        if self.homing_state == HomingState.HOMING:
            self.logger.warning("Homing already in progress")
            return False

        if self.emergency_stop_active:
            self.logger.warning("Cannot home: Emergency stop active")
            return False

        self.homing_state = HomingState.HOMING
        self.homing_progress = 0.0
        self.homing_start_time = time.time()
        self.stats["homing_sequences"] += 1

        self.logger.info("Homing sequence started")
        return True

    def get_motor_status(self, motor_id: int) -> Optional[Dict[str, Any]]:
        """Get motor status.

        Args:
            motor_id: Motor identifier

        Returns:
            Dict with motor status or None
        """
        if motor_id not in self.motors:
            return None

        motor = self.motors[motor_id]
        encoder = self.encoders[motor_id]

        return {
            "motor_id": motor_id,
            "velocity_setpoint": motor.velocity_setpoint,
            "velocity_actual": motor.velocity_actual,
            "position": motor.position,
            "current": motor.current,
            "temperature": motor.temperature,
            "voltage": motor.voltage,
            "fault": motor.fault.value,
            "enabled": motor.enabled,
            "encoder_position": encoder.position,
            "encoder_velocity": encoder.velocity,
            "encoder_valid": encoder.valid,
        }

    def get_system_status(self) -> Dict[str, Any]:
        """Get overall system status.

        Returns:
            Dict with system status
        """
        return {
            "emergency_stop": self.emergency_stop_active,
            "homing_state": self.homing_state.value,
            "homing_progress": self.homing_progress,
            "num_motors": self.num_motors,
            "motors_faulted": sum(
                1 for m in self.motors.values() if m.fault != MotorFaultType.NONE
            ),
            "control_loop_running": self.running,
            "control_loop_hz": self.control_loop_hz,
            "statistics": self.stats.copy(),
        }

    def inject_fault(self, motor_id: int, fault_type: MotorFaultType) -> bool:
        """Inject fault for testing.

        Args:
            motor_id: Motor to fault
            fault_type: Type of fault to inject

        Returns:
            bool: True if fault injected
        """
        if motor_id not in self.motors:
            return False

        self.motors[motor_id].fault = fault_type
        self.motors[motor_id].velocity_setpoint = 0.0
        self.motors[motor_id].velocity_actual = 0.0
        self.stats["faults_injected"] += 1

        self.logger.warning(f"Fault injected on motor {motor_id}: {fault_type.value}")
        return True

    def clear_fault(self, motor_id: int) -> bool:
        """Clear motor fault.

        Args:
            motor_id: Motor to clear

        Returns:
            bool: True if fault cleared
        """
        if motor_id not in self.motors:
            return False

        self.motors[motor_id].fault = MotorFaultType.NONE
        self.logger.info(f"Fault cleared on motor {motor_id}")
        return True

    def _control_loop(self):
        """Main control loop (runs in separate thread)."""
        dt = 1.0 / self.control_loop_hz

        while self.running:
            loop_start = time.time()

            # Check communication timeout
            if time.time() - self.last_command_time > self.command_timeout:
                # Safety: Stop motors if no commands received
                for motor in self.motors.values():
                    motor.velocity_setpoint = 0.0

            # Update each motor
            for motor_id, motor in self.motors.items():
                self._update_motor(motor, dt)
                self._update_encoder(motor_id, motor, dt)

            # Update homing sequence
            if self.homing_state == HomingState.HOMING:
                self._update_homing(dt)

            # Random fault injection
            if self.simulate_faults:
                self._maybe_inject_fault()

            self.stats["control_cycles"] += 1

            # Sleep to maintain loop rate
            elapsed = time.time() - loop_start
            sleep_time = max(0, dt - elapsed)
            time.sleep(sleep_time)

    def _update_motor(self, motor: MotorState, dt: float):
        """Update motor physics simulation.

        Args:
            motor: Motor state to update
            dt: Time step
        """
        if motor.fault != MotorFaultType.NONE or self.emergency_stop_active:
            motor.velocity_actual = 0.0
            motor.current = 0.0
            return

        # Velocity control with ramp limiting
        velocity_error = motor.velocity_setpoint - motor.velocity_actual
        max_delta = self.VELOCITY_RAMP_RATE * dt
        velocity_delta = max(-max_delta, min(max_delta, velocity_error))
        motor.velocity_actual += velocity_delta

        # Position integration
        motor.position += motor.velocity_actual * dt

        # Current calculation (simplified motor model)
        torque_current = abs(motor.velocity_actual) * 0.3
        accel_current = abs(velocity_delta / dt) * 0.1
        friction_current = self.FRICTION_COEFFICIENT
        motor.current = torque_current + accel_current + friction_current

        # Check overcurrent
        if motor.current > self.MAX_CURRENT:
            motor.fault = MotorFaultType.OVERCURRENT
            self.logger.error(
                f"Motor {motor.motor_id} overcurrent: {motor.current:.2f}A"
            )

        # Temperature simulation
        power_loss = motor.current * 0.5
        temp_rise = power_loss * dt / self.THERMAL_TIME_CONSTANT
        temp_fall = (motor.temperature - 25.0) * dt / self.THERMAL_TIME_CONSTANT
        motor.temperature += temp_rise - temp_fall

        # Check overtemperature
        if motor.temperature > self.MAX_TEMPERATURE:
            motor.fault = MotorFaultType.OVERTEMPERATURE
            self.logger.error(
                f"Motor {motor.motor_id} overtemperature: {motor.temperature:.1f}°C"
            )

        # Voltage simulation (with ripple)
        motor.voltage = self.NOMINAL_VOLTAGE + random.uniform(-0.5, 0.5)

        motor.last_update = time.time()

    def _update_encoder(self, motor_id: int, motor: MotorState, dt: float):
        """Update encoder simulation.

        Args:
            motor_id: Motor identifier
            motor: Associated motor state
            dt: Time step
        """
        encoder = self.encoders[motor_id]

        # Update position (with noise)
        encoder.position = motor.position + random.gauss(
            0, encoder.noise_std / encoder.resolution * 2 * math.pi
        )
        encoder.velocity = motor.velocity_actual + random.gauss(0, 0.01)

        # Convert to counts
        encoder.raw_counts = int(encoder.position * encoder.resolution / (2 * math.pi))

        # Simulate encoder failures
        if motor.fault == MotorFaultType.ENCODER_FAILURE:
            encoder.valid = False
        else:
            encoder.valid = True

    def _update_homing(self, dt: float):
        """Update homing sequence.

        Args:
            dt: Time step
        """
        elapsed = time.time() - self.homing_start_time
        self.homing_progress = min(1.0, elapsed / self.homing_duration)

        if self.homing_progress >= 1.0:
            self.homing_state = HomingState.COMPLETE
            self.logger.info("Homing sequence complete")

            # Reset all encoder positions to zero
            for encoder in self.encoders.values():
                encoder.position = 0.0
                encoder.raw_counts = 0

    def _maybe_inject_fault(self):
        """Maybe inject random fault for testing."""
        if random.random() < self.fault_rate:
            motor_id = random.randint(0, self.num_motors - 1)
            fault_type = random.choice(
                [
                    MotorFaultType.OVERCURRENT,
                    MotorFaultType.OVERTEMPERATURE,
                    MotorFaultType.ENCODER_FAILURE,
                ]
            )
            self.inject_fault(motor_id, fault_type)


# Convenience function
def create_firmware_simulator(profile: str = "default") -> STM32FirmwareSimulator:
    """Create firmware simulator with predefined configuration.

    Args:
        profile: Configuration profile ('default', 'reliable', 'faulty')

    Returns:
        Configured STM32FirmwareSimulator instance
    """
    profiles = {
        "default": {
            "num_motors": 6,
            "simulate_faults": True,
            "fault_rate": 0.0001,
            "control_loop_hz": 100,
        },
        "reliable": {
            "num_motors": 6,
            "simulate_faults": False,
            "fault_rate": 0.0,
            "control_loop_hz": 100,
        },
        "faulty": {
            "num_motors": 6,
            "simulate_faults": True,
            "fault_rate": 0.001,
            "control_loop_hz": 100,
        },
    }

    config = profiles.get(profile, profiles["default"])
    return STM32FirmwareSimulator(config)
