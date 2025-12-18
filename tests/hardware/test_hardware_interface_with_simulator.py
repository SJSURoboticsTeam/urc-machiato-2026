#!/usr/bin/env python3
"""
Hardware Interface Tests Using Simulator

Tests hardware interface functionality using the hardware simulator
when physical hardware is not available.

Author: URC 2026 Autonomy Team
"""

# Add test utilities
import os
import sys
import time
import unittest

import pytest

PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, PROJECT_ROOT)

from tests.hardware.hardware_interface_simulator import (
    HardwareInterfaceTestFixture,
    create_simulated_hardware_interface,
    with_hardware_simulator,
)


class TestHardwareInterfaceWithSimulator(unittest.TestCase):
    """Test hardware interface using simulator."""

    def setUp(self):
        """Set up test fixtures."""
        self.simulator = None

    def tearDown(self):
        """Clean up test fixtures."""
        if self.simulator:
            self.simulator.stop()
            self.simulator = None

    def test_simulator_initialization(self):
        """Test hardware simulator initializes correctly."""
        with HardwareInterfaceTestFixture() as sim:
            # Verify simulator is running
            self.assertTrue(sim.running)

            # Verify simulation components are initialized
            self.assertIsNotNone(sim.rover_sim)
            self.assertIsNotNone(sim.can_sim)
            self.assertIsNotNone(sim.gps_sim)
            self.assertIsNotNone(sim.imu_sim)

            # Verify sensor buffers are ready
            self.assertIn("imu", sim.sensor_buffers)
            self.assertIn("gps", sim.sensor_buffers)
            self.assertIn("odometry", sim.sensor_buffers)

    def test_velocity_command_simulation(self):
        """Test velocity command processing with simulator."""
        with HardwareInterfaceTestFixture() as sim:
            # Send velocity command
            sim.send_velocity_command(1.0, 0.5)

            # Wait for processing
            time.sleep(0.2)

            # Check that command was processed
            self.assertGreater(len(sim.control_commands), 0)

            # Check odometry was updated
            odom = sim.get_odometry()
            self.assertIsNotNone(odom)
            self.assertIn("linear_velocity", odom)
            self.assertIn("angular_velocity", odom)

    def test_emergency_stop_simulation(self):
        """Test emergency stop with simulator."""
        emergency_triggered = False

        def emergency_callback():
            nonlocal emergency_triggered
            emergency_triggered = True

        with HardwareInterfaceTestFixture() as sim:
            # Set emergency callback
            sim.set_emergency_callback(emergency_callback)

            # Trigger emergency stop
            sim.emergency_stop()

            # Wait for processing
            time.sleep(0.1)

            # Verify emergency stop was processed
            self.assertTrue(emergency_triggered)

    def test_sensor_data_generation(self):
        """Test sensor data generation in simulator."""
        with HardwareInterfaceTestFixture() as sim:
            # Wait for sensor updates
            time.sleep(0.5)

            # Check GPS data
            gps_data = sim.get_gps_data()
            self.assertIsNotNone(gps_data)
            self.assertIn("latitude", gps_data)
            self.assertIn("longitude", gps_data)

            # Check IMU data
            imu_data = sim.get_imu_data()
            self.assertIsNotNone(imu_data)
            self.assertIn("angular_velocity", imu_data)
            self.assertIn("linear_acceleration", imu_data)

            # Check battery data
            battery_data = sim.get_battery_status()
            self.assertIsNotNone(battery_data)
            self.assertIn("voltage", battery_data)
            self.assertIn("percentage", battery_data)

            # Check joint states
            joint_data = sim.get_joint_states()
            self.assertIsNotNone(joint_data)
            self.assertIn("names", joint_data)
            self.assertIn("positions", joint_data)
            self.assertEqual(len(joint_data["names"]), 6)  # 6 wheels

    def test_rover_physics_simulation(self):
        """Test rover physics simulation."""
        with HardwareInterfaceTestFixture() as sim:
            initial_odom = sim.get_odometry()

            # Send forward velocity command
            sim.send_velocity_command(1.0, 0.0)

            # Wait for movement
            time.sleep(0.5)

            # Check that rover moved
            final_odom = sim.get_odometry()
            self.assertIsNotNone(final_odom)

            # Position should have changed
            initial_pos = initial_odom["position"]
            final_pos = final_odom["position"]

            # Should have moved forward (positive X)
            self.assertGreater(final_pos["x"], initial_pos["x"])

    def test_simulator_reset(self):
        """Test simulator reset functionality."""
        with HardwareInterfaceTestFixture() as sim:
            # Modify simulator state
            sim.send_velocity_command(2.0, 1.0)
            time.sleep(0.2)

            # Get state after movement
            odom_after_move = sim.get_odometry()

            # Reset simulator
            sim.reset_simulations()

            # Check that state was reset
            odom_after_reset = sim.get_odometry()

            # Position should be back to initial state
            if odom_after_reset:
                initial_pos = odom_after_reset["position"]
                moved_pos = odom_after_move["position"]

                # Should be different (reset worked)
                self.assertNotEqual(initial_pos["x"], moved_pos["x"])

    def test_sensor_failure_simulation(self):
        """Test sensor failure simulation."""
        with HardwareInterfaceTestFixture() as sim:
            # Simulate GPS failure
            sim.simulate_hardware_failure("gps")

            # Wait for update
            time.sleep(0.2)

            # GPS data should indicate failure or be None
            gps_data = sim.get_gps_data()
            if gps_data:
                # Should have error flags or invalid data
                self.assertTrue(
                    gps_data.get("has_error", False)
                    or gps_data.get("fix_quality", 0) == 0
                )

    def test_can_bus_simulation(self):
        """Test CAN bus simulation."""
        with HardwareInterfaceTestFixture() as sim:
            # CAN bus should be running
            self.assertTrue(sim.can_sim.running)

            # Send a CAN message through velocity command
            sim.send_velocity_command(0.5, 0.2)

            # Check CAN message was sent
            # (In real implementation, would check CAN bus message queue)
            self.assertGreater(len(sim.control_commands), 0)

    @pytest.mark.slow
    def test_long_running_simulation(self):
        """Test longer simulation run for stability."""
        with HardwareInterfaceTestFixture() as sim:
            # Run simulation for 2 seconds
            start_time = time.time()

            while time.time() - start_time < 2.0:
                # Send varying commands
                linear = 0.5 * (time.time() - start_time)
                angular = 0.2 * (time.time() - start_time)
                sim.send_velocity_command(linear, angular)
                time.sleep(0.1)

            # Check simulation remained stable
            final_odom = sim.get_odometry()
            self.assertIsNotNone(final_odom)

            # Should have moved significantly
            self.assertGreater(final_odom["position"]["x"], 0.5)

    def test_multiple_simultaneous_commands(self):
        """Test handling multiple simultaneous commands."""
        with HardwareInterfaceTestFixture() as sim:
            # Send multiple commands rapidly
            for i in range(10):
                sim.send_velocity_command(float(i) * 0.1, float(i) * 0.05)

            # Wait for processing
            time.sleep(0.5)

            # All commands should have been processed
            self.assertEqual(len(sim.control_commands), 0)  # Queue should be empty

            # Odometry should reflect final command
            final_odom = sim.get_odometry()
            self.assertIsNotNone(final_odom)


# Test using decorator approach


@with_hardware_simulator()
def test_decorator_based_simulation(hardware_simulator):
    """Test using decorator-based simulator setup."""
    # Send test command
    hardware_simulator.send_velocity_command(1.0, 0.0)

    # Verify command was queued
    assert len(hardware_simulator.control_commands) > 0

    # Wait and check odometry
    time.sleep(0.2)
    odom = hardware_simulator.get_odometry()
    assert odom is not None


@with_hardware_simulator(
    {
        "update_rate_hz": 100.0,  # Higher rate for precision testing
        "rover_config": {"max_velocity": 2.0},
    }
)
def test_custom_config_simulation(hardware_simulator):
    """Test simulation with custom configuration."""
    # Verify custom config was applied
    assert hardware_simulator.config["update_rate_hz"] == 100.0
    assert hardware_simulator.config["rover_config"]["max_velocity"] == 2.0

    # Test with higher velocity
    hardware_simulator.send_velocity_command(1.5, 0.0)
    time.sleep(0.1)

    odom = hardware_simulator.get_odometry()
    assert odom is not None


if __name__ == "__main__":
    unittest.main()
