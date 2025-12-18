#!/usr/bin/env python3
"""
Hardware Interface Simulator for Testing

Provides simulated hardware interfaces when physical hardware is not available.
Integrates with the existing simulation framework to provide realistic testing
environments for hardware interface validation.

Author: URC 2026 Autonomy Team
"""

import os
import sys
import threading
import time
from typing import Any, Callable, Dict, List, Optional
from unittest.mock import Mock

import numpy as np

# Add project paths
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
SIMULATION_ROOT = os.path.join(PROJECT_ROOT, "simulation")
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, SIMULATION_ROOT)

try:
    # Import simulation components
    from simulation.can.can_bus_mock_simulator import (
        CANBusMockSimulator,
        CANMessage,
        CANMessageType,
    )
    from simulation.drive.drive_system_simulator import DriveSystemSimulator
    from simulation.power.power_system_simulator import PowerSystemSimulator
    from simulation.rover.urc_rover import URCRover
    from simulation.sensors.gps_simulator import GPSSimulator
    from simulation.sensors.imu_simulator import IMUSimulator

    SIMULATION_AVAILABLE = True
except ImportError:
    SIMULATION_AVAILABLE = False


class HardwareInterfaceSimulator:
    """
    Hardware Interface Simulator

    Simulates complete hardware stack for testing when physical hardware
    is not available. Provides realistic timing, sensor data, and actuator
    responses.
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        """Initialize hardware simulator."""
        self.config = config or self._get_default_config()
        self.running = False
        self.thread = None

        if not SIMULATION_AVAILABLE:
            raise ImportError(
                "Simulation framework not available. Cannot run hardware simulator."
            )

        # Initialize simulation components
        self._init_simulation_components()

        # ROS2 message publishers (mocked)
        self.publishers = {}

        # Sensor data buffers
        self.sensor_buffers = {
            "imu": [],
            "gps": [],
            "odometry": [],
            "battery": [],
            "temperature": [],
            "joint_states": [],
        }

        # Control command buffers
        self.control_commands = []

        # Callbacks for hardware interface
        self.callbacks = {
            "emergency_stop": None,
            "velocity_command": None,
            "led_command": None,
        }

    def _get_default_config(self) -> Dict[str, Any]:
        """Get default simulator configuration."""
        return {
            "update_rate_hz": 50.0,  # 50Hz control loop
            "sensor_rate_hz": 20.0,  # 20Hz sensor updates
            "can_port": "/dev/ttyACM0",
            "rover_config": {
                "mass": 75.0,
                "wheel_count": 6,
                "max_velocity": 1.5,
                "max_angular_velocity": 0.8,
            },
            "gps_config": {"update_rate_hz": 5.0, "accuracy_m": 2.0},
            "imu_config": {"update_rate_hz": 100.0, "noise_sigma": 0.01},
            "power_config": {"battery_capacity_ah": 10.0, "nominal_voltage": 24.0},
        }

    def _init_simulation_components(self):
        """Initialize simulation components."""
        # CAN bus simulator
        self.can_sim = CANBusMockSimulator(
            port=self.config["can_port"], baudrate=115200, simulate_delays=True
        )

        # Rover physics simulator
        self.rover_sim = URCRover(self.config["rover_config"])

        # Sensor simulators
        self.gps_sim = GPSSimulator(self.config["gps_config"])
        self.imu_sim = IMUSimulator(self.config["imu_config"])
        self.power_sim = PowerSystemSimulator(self.config["power_config"])

        # Drive system simulator
        self.drive_sim = DriveSystemSimulator()

    def start(self):
        """Start the hardware simulator."""
        if self.running:
            return

        self.running = True
        self.thread = threading.Thread(target=self._simulation_loop)
        self.thread.daemon = True
        self.thread.start()

        # Start CAN bus simulation
        self.can_sim.start()

    def stop(self):
        """Stop the hardware simulator."""
        self.running = False
        if self.thread:
            self.thread.join(timeout=1.0)

        # Stop CAN bus simulation
        self.can_sim.stop()

    def _simulation_loop(self):
        """Main simulation loop."""
        control_interval = 1.0 / self.config["update_rate_hz"]
        sensor_interval = 1.0 / self.config["sensor_rate_hz"]

        last_control_time = time.time()
        last_sensor_time = time.time()

        while self.running:
            current_time = time.time()

            # Update control systems
            if current_time - last_control_time >= control_interval:
                self._update_control_systems(current_time - last_control_time)
                last_control_time = current_time

            # Update sensors
            if current_time - last_sensor_time >= sensor_interval:
                self._update_sensors(current_time - last_sensor_time)
                last_sensor_time = current_time

            # Small sleep to prevent busy waiting
            time.sleep(0.001)

    def _update_control_systems(self, dt: float):
        """Update control systems simulation."""
        # Process pending control commands
        while self.control_commands:
            cmd = self.control_commands.pop(0)
            self._process_control_command(cmd)

        # Update rover physics
        control_inputs = self._get_current_control_inputs()
        self.rover_sim.step(dt, control_inputs)

        # Update drive system
        self.drive_sim.update_from_rover(self.rover_sim)

    def _update_sensors(self, dt: float):
        """Update sensor simulation."""
        # Update GPS
        gps_data = self.gps_sim.get_reading()
        self.sensor_buffers["gps"].append(gps_data)

        # Update IMU
        imu_data = self.imu_sim.get_reading()
        self.sensor_buffers["imu"].append(imu_data)

        # Update power system
        battery_data = self.power_sim.get_battery_status()
        self.sensor_buffers["battery"].append(battery_data)

        # Generate odometry from rover state
        odom_data = self._generate_odometry_data()
        self.sensor_buffers["odometry"].append(odom_data)

        # Generate joint states from drive system
        joint_data = self._generate_joint_states()
        self.sensor_buffers["joint_states"].append(joint_data)

        # Keep buffers at reasonable size
        for buffer_name in self.sensor_buffers:
            if len(self.sensor_buffers[buffer_name]) > 10:
                self.sensor_buffers[buffer_name] = self.sensor_buffers[buffer_name][
                    -10:
                ]

    def _process_control_command(self, cmd: Dict[str, Any]):
        """Process a control command."""
        cmd_type = cmd.get("type")

        if cmd_type == "velocity":
            # Send velocity command to CAN bus
            linear_x = cmd.get("linear_x", 0.0)
            angular_z = cmd.get("angular_z", 0.0)

            can_msg = CANMessage(
                message_id=0x100,  # Velocity command ID
                data=f"{linear_x:.3f},{angular_z:.3f}".encode(),
                priority=CANMessageType.MOTOR_COMMAND,
                timestamp=time.time(),
            )
            self.can_sim.send_message(can_msg)

        elif cmd_type == "emergency_stop":
            # Send emergency stop command
            can_msg = CANMessage(
                message_id=0x000,  # Emergency stop ID
                data=b"EMERGENCY_STOP",
                priority=CANMessageType.EMERGENCY_STOP,
                timestamp=time.time(),
            )
            self.can_sim.send_message(can_msg)

            # Trigger callback if registered
            if self.callbacks["emergency_stop"]:
                self.callbacks["emergency_stop"]()

    def _get_current_control_inputs(self) -> Dict[str, Any]:
        """Get current control inputs for rover simulation."""
        # In a real implementation, this would read from CAN bus
        # For simulation, we maintain state
        return {
            "linear_velocity_cmd": getattr(self, "last_linear_cmd", 0.0),
            "angular_velocity_cmd": getattr(self, "last_angular_cmd", 0.0),
        }

    def _generate_odometry_data(self) -> Dict[str, Any]:
        """Generate odometry data from rover simulation."""
        rover_state = self.rover_sim.get_state()

        return {
            "timestamp": time.time(),
            "position": {
                "x": rover_state["position"][0],
                "y": rover_state["position"][1],
                "z": 0.0,
            },
            "orientation": {
                "x": 0.0,
                "y": 0.0,
                "z": rover_state["heading"],
                "w": 1.0,  # Simplified, no roll/pitch
            },
            "linear_velocity": {"x": rover_state["velocity"][0], "y": 0.0, "z": 0.0},
            "angular_velocity": {
                "x": 0.0,
                "y": 0.0,
                "z": rover_state["angular_velocity"],
            },
        }

    def _generate_joint_states(self) -> Dict[str, Any]:
        """Generate joint states from drive system."""
        joint_names = [f"wheel_{i}_joint" for i in range(6)]
        positions = self.drive_sim.get_wheel_positions()
        velocities = self.drive_sim.get_wheel_velocities()
        efforts = self.drive_sim.get_wheel_torques()

        return {
            "timestamp": time.time(),
            "names": joint_names,
            "positions": positions,
            "velocities": velocities,
            "efforts": efforts,
        }

    # Hardware interface methods (called by tests)

    def send_velocity_command(self, linear_x: float, angular_z: float):
        """Send velocity command to simulated hardware."""
        cmd = {
            "type": "velocity",
            "linear_x": linear_x,
            "angular_z": angular_z,
            "timestamp": time.time(),
        }
        self.control_commands.append(cmd)

        # Update last commands for simulation
        self.last_linear_cmd = linear_x
        self.last_angular_cmd = angular_z

    def emergency_stop(self):
        """Trigger emergency stop on simulated hardware."""
        cmd = {"type": "emergency_stop", "timestamp": time.time()}
        self.control_commands.append(cmd)

    def get_latest_sensor_data(self, sensor_type: str) -> Optional[Dict[str, Any]]:
        """Get latest sensor data from simulation."""
        buffer = self.sensor_buffers.get(sensor_type, [])
        return buffer[-1] if buffer else None

    def get_odometry(self) -> Optional[Dict[str, Any]]:
        """Get latest odometry data."""
        return self.get_latest_sensor_data("odometry")

    def get_gps_data(self) -> Optional[Dict[str, Any]]:
        """Get latest GPS data."""
        return self.get_latest_sensor_data("gps")

    def get_imu_data(self) -> Optional[Dict[str, Any]]:
        """Get latest IMU data."""
        return self.get_latest_sensor_data("imu")

    def get_battery_status(self) -> Optional[Dict[str, Any]]:
        """Get latest battery status."""
        return self.get_latest_sensor_data("battery")

    def get_joint_states(self) -> Optional[Dict[str, Any]]:
        """Get latest joint states."""
        return self.get_latest_sensor_data("joint_states")

    def set_emergency_callback(self, callback: Callable):
        """Set emergency stop callback."""
        self.callbacks["emergency_stop"] = callback

    def set_velocity_callback(self, callback: Callable):
        """Set velocity command callback."""
        self.callbacks["velocity_command"] = callback

    def set_led_callback(self, callback: Callable):
        """Set LED command callback."""
        self.callbacks["led_command"] = callback

    # Test utility methods

    def wait_for_stable_state(self, timeout: float = 2.0):
        """Wait for simulation to reach stable state."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.get_odometry() is not None:
                time.sleep(0.1)  # Let simulation run a bit
                return True
            time.sleep(0.01)
        return False

    def simulate_sensor_noise(self, sensor_type: str, noise_level: float = 0.01):
        """Add noise to sensor simulation for testing."""
        if sensor_type == "gps":
            self.gps_sim.set_noise_level(noise_level)
        elif sensor_type == "imu":
            self.imu_sim.set_noise_level(noise_level)

    def simulate_hardware_failure(self, component: str):
        """Simulate hardware failure for testing."""
        if component == "can_bus":
            self.can_sim.simulate_failure()
        elif component == "gps":
            self.gps_sim.simulate_failure()
        elif component == "imu":
            self.imu_sim.simulate_failure()
        elif component == "drive":
            self.drive_sim.simulate_failure()

    def reset_simulations(self):
        """Reset all simulations to initial state."""
        self.rover_sim.reset()
        self.gps_sim.reset()
        self.imu_sim.reset()
        self.power_sim.reset()
        self.drive_sim.reset()
        self.can_sim.reset()

        # Clear buffers
        for buffer_name in self.sensor_buffers:
            self.sensor_buffers[buffer_name].clear()
        self.control_commands.clear()


class HardwareInterfaceTestFixture:
    """
    Test fixture for hardware interface testing with simulator.

    Provides easy setup and teardown for tests that need hardware simulation.
    """

    def __init__(self, config: Optional[Dict[str, Any]] = None):
        self.simulator = None
        self.config = config

    def __enter__(self):
        """Set up test fixture."""
        self.simulator = HardwareInterfaceSimulator(self.config)
        self.simulator.start()

        # Wait for simulation to initialize
        if not self.simulator.wait_for_stable_state():
            raise RuntimeError("Simulator failed to reach stable state")

        return self.simulator

    def __exit__(self, exc_type, exc_val, exc_tb):
        """Clean up test fixture."""
        if self.simulator:
            self.simulator.stop()
            self.simulator = None


# Convenience functions for tests


def create_simulated_hardware_interface(config: Optional[Dict[str, Any]] = None):
    """Create a simulated hardware interface for testing."""
    return HardwareInterfaceSimulator(config)


def with_hardware_simulator(config: Optional[Dict[str, Any]] = None):
    """Decorator to run test with hardware simulator."""

    def decorator(func):
        def wrapper(*args, **kwargs):
            with HardwareInterfaceTestFixture(config) as sim:
                # Add simulator to test arguments
                kwargs["hardware_simulator"] = sim
                return func(*args, **kwargs)

        return wrapper

    return decorator


# Example usage in tests:

"""
Example test using the hardware simulator:

@with_hardware_simulator()
def test_velocity_control(hardware_simulator):
    # Send velocity command
    hardware_simulator.send_velocity_command(1.0, 0.5)

    # Wait for processing
    time.sleep(0.1)

    # Check odometry updated
    odom = hardware_simulator.get_odometry()
    assert odom is not None
    assert odom['linear_velocity']['x'] > 0

def test_emergency_stop():
    with HardwareInterfaceTestFixture() as sim:
        emergency_triggered = False

        def emergency_callback():
            nonlocal emergency_triggered
            emergency_triggered = True

        sim.set_emergency_callback(emergency_callback)
        sim.emergency_stop()

        time.sleep(0.1)
        assert emergency_triggered
"""
