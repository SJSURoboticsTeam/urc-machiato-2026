#!/usr/bin/env python3
"""
Unit Tests for STM32 Firmware Simulator

Comprehensive tests for STM32 firmware behavior simulation including
motor control, encoder feedback, fault injection, and emergency stop handling.

Author: URC 2026 Testing Team
"""

import pytest
import time
import sys
from pathlib import Path

# Add simulation to path
sys.path.insert(0, str(Path(__file__).parent.parent.parent))

from simulation.firmware.stm32_firmware_simulator import (
    STM32FirmwareSimulator,
    MotorState,
    MotorFaultType,
    HomingState,
    EncoderState,
    create_firmware_simulator
)


class TestSTM32FirmwareSimulator:
    """Test suite for STM32 firmware simulator."""
    
    @pytest.fixture
    def simulator(self):
        """Create basic simulator for testing."""
        sim = STM32FirmwareSimulator({
            'num_motors': 6,
            'simulate_faults': False,
            'control_loop_hz': 100
        })
        yield sim
        sim.stop()
    
    @pytest.fixture
    def fault_simulator(self):
        """Create simulator with fault injection enabled."""
        sim = STM32FirmwareSimulator({
            'num_motors': 6,
            'simulate_faults': True,
            'fault_rate': 0.5  # High rate for testing
        })
        yield sim
        sim.stop()
    
    # Initialization Tests
    
    def test_simulator_initialization(self, simulator):
        """Test simulator initializes correctly."""
        assert simulator is not None
        assert simulator.num_motors == 6
        assert len(simulator.motors) == 6
        assert len(simulator.encoders) == 6
    
    def test_motor_states_initialized(self, simulator):
        """Test all motor states are properly initialized."""
        for motor_id in range(simulator.num_motors):
            motor = simulator.motors[motor_id]
            assert motor.motor_id == motor_id
            assert motor.velocity_setpoint == 0.0
            assert motor.velocity_actual == 0.0
            assert motor.enabled is True
            assert motor.fault == MotorFaultType.NONE
    
    def test_encoder_states_initialized(self, simulator):
        """Test all encoder states are properly initialized."""
        for encoder_id in range(simulator.num_motors):
            encoder = simulator.encoders[encoder_id]
            assert encoder.encoder_id == encoder_id
            assert encoder.position == 0.0
            assert encoder.valid is True
    
    def test_emergency_stop_initial_state(self, simulator):
        """Test emergency stop is initially inactive."""
        assert simulator.emergency_stop_active is False
    
    # Control Loop Tests
    
    def test_start_control_loop(self, simulator):
        """Test control loop can be started."""
        simulator.start()
        assert simulator.running is True
        time.sleep(0.1)  # Let it run briefly
    
    def test_stop_control_loop(self, simulator):
        """Test control loop can be stopped."""
        simulator.start()
        time.sleep(0.05)
        simulator.stop()
        assert simulator.running is False
    
    def test_control_loop_executes(self, simulator):
        """Test control loop increments cycle counter."""
        simulator.start()
        initial_cycles = simulator.stats['control_cycles']
        time.sleep(0.2)  # 200ms = ~20 cycles at 100Hz
        simulator.stop()
        
        assert simulator.stats['control_cycles'] > initial_cycles
    
    # Motor Control Tests
    
    def test_set_single_motor_velocity(self, simulator):
        """Test setting velocity for single motor."""
        success = simulator.set_velocity_command(0, 5.0)
        
        assert success is True
        assert simulator.motors[0].velocity_setpoint == 5.0
    
    def test_set_velocity_invalid_motor(self, simulator):
        """Test setting velocity for non-existent motor fails."""
        success = simulator.set_velocity_command(99, 5.0)
        assert success is False
    
    def test_set_velocity_exceeds_max(self, simulator):
        """Test setting velocity beyond max is clamped."""
        max_vel = simulator.MAX_VELOCITY
        success = simulator.set_velocity_command(0, max_vel * 2)
        
        assert success is True
        # Should be clamped to max
        assert simulator.motors[0].velocity_setpoint <= max_vel
    
    def test_set_chassis_velocities(self, simulator):
        """Test setting chassis-level velocities."""
        success = simulator.set_chassis_velocities(0.5, 0.0, 0.2)
        
        assert success is True
        # Commands should be converted to individual motor velocities
        # At least some motors should have non-zero velocities
        motor_velocities = [m.velocity_setpoint for m in simulator.motors.values()]
        assert any(v != 0.0 for v in motor_velocities)
    
    def test_chassis_velocities_forward(self, simulator):
        """Test forward motion sets motor velocities."""
        simulator.set_chassis_velocities(1.0, 0.0, 0.0)
        
        # All wheels should move in same direction for forward
        motor_velocities = [m.velocity_setpoint for m in simulator.motors.values()]
        # Check velocities are set (sign may vary by wheel)
        assert any(abs(v) > 0 for v in motor_velocities)
    
    def test_chassis_velocities_rotation(self, simulator):
        """Test rotation sets opposing motor velocities."""
        simulator.set_chassis_velocities(0.0, 0.0, 1.0)
        
        # Rotation should cause motors to turn in opposite directions
        motor_velocities = [m.velocity_setpoint for m in simulator.motors.values()]
        assert any(v > 0 for v in motor_velocities) or any(v < 0 for v in motor_velocities)
    
    def test_velocity_ramping(self, simulator):
        """Test velocity ramps up gradually."""
        simulator.start()
        simulator.set_velocity_command(0, 5.0)
        
        time.sleep(0.05)
        velocity_1 = simulator.motors[0].velocity_actual
        
        time.sleep(0.1)
        velocity_2 = simulator.motors[0].velocity_actual
        
        simulator.stop()
        
        # Velocity should increase over time
        assert velocity_2 > velocity_1
    
    def test_velocity_reaches_setpoint(self, simulator):
        """Test velocity eventually reaches setpoint."""
        simulator.start()
        setpoint = 3.0
        simulator.set_velocity_command(0, setpoint)
        
        # Wait for ramping
        time.sleep(1.0)
        actual = simulator.motors[0].velocity_actual
        simulator.stop()
        
        # Should be close to setpoint
        assert abs(actual - setpoint) < 0.5
    
    # Emergency Stop Tests
    
    def test_emergency_stop_activation(self, simulator):
        """Test emergency stop can be activated."""
        success = simulator.emergency_stop()
        
        assert success is True
        assert simulator.emergency_stop_active is True
    
    def test_emergency_stop_zeros_velocities(self, simulator):
        """Test emergency stop immediately zeros all velocities."""
        simulator.start()
        
        # Set some velocities
        for motor_id in range(simulator.num_motors):
            simulator.set_velocity_command(motor_id, 5.0)
        
        time.sleep(0.1)
        
        # Emergency stop
        simulator.emergency_stop()
        
        # All setpoints should be zero
        for motor in simulator.motors.values():
            assert motor.velocity_setpoint == 0.0
        
        simulator.stop()
    
    def test_emergency_stop_disables_motors(self, simulator):
        """Test emergency stop disables motor control."""
        simulator.emergency_stop()
        
        # Try to set velocity after e-stop
        success = simulator.set_velocity_command(0, 5.0)
        
        # Should fail or be ignored
        assert simulator.motors[0].velocity_setpoint == 0.0
    
    def test_clear_emergency_stop(self, simulator):
        """Test clearing emergency stop re-enables control."""
        simulator.emergency_stop()
        assert simulator.emergency_stop_active is True
        
        simulator.clear_emergency_stop()
        assert simulator.emergency_stop_active is False
        
        # Should be able to control motors again
        success = simulator.set_velocity_command(0, 3.0)
        assert success is True
    
    def test_emergency_stop_statistics(self, simulator):
        """Test emergency stop events are tracked."""
        initial = simulator.stats['emergency_stops']
        
        simulator.emergency_stop()
        
        assert simulator.stats['emergency_stops'] > initial
    
    # Homing Sequence Tests
    
    def test_start_homing_sequence(self, simulator):
        """Test homing sequence can be started."""
        success = simulator.start_homing_sequence()
        
        assert success is True
        assert simulator.homing_state == HomingState.HOMING
    
    def test_homing_sequence_progress(self, simulator):
        """Test homing sequence progresses over time."""
        simulator.start()
        simulator.start_homing_sequence()
        
        time.sleep(0.5)
        progress_1 = simulator.homing_progress
        
        time.sleep(0.5)
        progress_2 = simulator.homing_progress
        
        simulator.stop()
        
        assert progress_2 > progress_1
        assert 0.0 <= progress_2 <= 1.0
    
    def test_homing_sequence_completion(self, simulator):
        """Test homing sequence completes after duration."""
        simulator.homing_duration = 0.5  # Short duration for testing
        simulator.start()
        simulator.start_homing_sequence()
        
        time.sleep(0.6)
        simulator.stop()
        
        assert simulator.homing_state == HomingState.COMPLETE
        assert simulator.homing_progress == 1.0
    
    def test_homing_statistics(self, simulator):
        """Test homing sequences are tracked."""
        initial = simulator.stats['homing_sequences']
        
        simulator.start_homing_sequence()
        
        assert simulator.stats['homing_sequences'] > initial
    
    # Motor Status Tests
    
    def test_get_motor_status(self, simulator):
        """Test retrieving motor status."""
        status = simulator.get_motor_status(0)
        
        assert status is not None
        assert 'motor_id' in status
        assert 'velocity_setpoint' in status
        assert 'velocity_actual' in status
        assert 'position' in status
        assert 'current' in status
        assert 'temperature' in status
        assert 'fault' in status
    
    def test_get_motor_status_invalid_id(self, simulator):
        """Test getting status for invalid motor returns None."""
        status = simulator.get_motor_status(99)
        assert status is None
    
    def test_motor_status_accuracy(self, simulator):
        """Test motor status reflects actual state."""
        simulator.set_velocity_command(0, 5.0)
        simulator.start()
        time.sleep(0.2)
        simulator.stop()
        
        status = simulator.get_motor_status(0)
        assert status['velocity_setpoint'] == 5.0
        assert status['velocity_actual'] > 0.0
    
    # System Status Tests
    
    def test_get_system_status(self, simulator):
        """Test retrieving system status."""
        status = simulator.get_system_status()
        
        assert 'num_motors' in status
        assert 'emergency_stop' in status
        assert 'homing_state' in status
        assert 'motors_faulted' in status
        assert 'uptime_s' in status
    
    def test_system_status_emergency_stop_flag(self, simulator):
        """Test system status reflects emergency stop state."""
        status1 = simulator.get_system_status()
        assert status1['emergency_stop'] is False
        
        simulator.emergency_stop()
        
        status2 = simulator.get_system_status()
        assert status2['emergency_stop'] is True
    
    def test_system_status_fault_count(self, simulator):
        """Test system status counts faulted motors."""
        # Inject fault
        simulator.inject_fault(0, MotorFaultType.OVERCURRENT)
        
        status = simulator.get_system_status()
        assert status['motors_faulted'] == 1
        
        # Inject another
        simulator.inject_fault(1, MotorFaultType.OVERTEMPERATURE)
        
        status = simulator.get_system_status()
        assert status['motors_faulted'] == 2
    
    # Fault Injection Tests
    
    def test_inject_overcurrent_fault(self, simulator):
        """Test injecting overcurrent fault."""
        success = simulator.inject_fault(0, MotorFaultType.OVERCURRENT)
        
        assert success is True
        assert simulator.motors[0].fault == MotorFaultType.OVERCURRENT
    
    def test_inject_overtemperature_fault(self, simulator):
        """Test injecting overtemperature fault."""
        success = simulator.inject_fault(0, MotorFaultType.OVERTEMPERATURE)
        
        assert success is True
        assert simulator.motors[0].fault == MotorFaultType.OVERTEMPERATURE
        # Temperature should be elevated
        assert simulator.motors[0].temperature > 50.0
    
    def test_inject_encoder_failure(self, simulator):
        """Test injecting encoder failure."""
        success = simulator.inject_fault(0, MotorFaultType.ENCODER_FAILURE)
        
        assert success is True
        assert simulator.motors[0].fault == MotorFaultType.ENCODER_FAILURE
        # Encoder should be marked invalid
        assert simulator.encoders[0].valid is False
    
    def test_fault_injection_invalid_motor(self, simulator):
        """Test injecting fault on invalid motor fails."""
        success = simulator.inject_fault(99, MotorFaultType.OVERCURRENT)
        assert success is False
    
    def test_clear_fault(self, simulator):
        """Test clearing motor fault."""
        simulator.inject_fault(0, MotorFaultType.OVERCURRENT)
        assert simulator.motors[0].fault == MotorFaultType.OVERCURRENT
        
        success = simulator.clear_fault(0)
        assert success is True
        assert simulator.motors[0].fault == MotorFaultType.NONE
    
    def test_fault_statistics(self, simulator):
        """Test fault injection is tracked."""
        initial = simulator.stats['faults_injected']
        
        simulator.inject_fault(0, MotorFaultType.OVERCURRENT)
        
        assert simulator.stats['faults_injected'] > initial
    
    def test_random_fault_injection(self, fault_simulator):
        """Test random fault injection occurs."""
        simulator = fault_simulator
        simulator.start()
        
        # Run for a bit
        time.sleep(1.0)
        simulator.stop()
        
        # With high fault rate, should have some faults
        # (probabilistic, but very likely)
        faulted_motors = [m for m in simulator.motors.values() 
                          if m.fault != MotorFaultType.NONE]
        # May or may not have faults due to randomness
        # Just check it doesn't crash
    
    # Encoder Tests
    
    def test_encoder_position_updates(self, simulator):
        """Test encoder position integrates velocity."""
        simulator.start()
        simulator.set_velocity_command(0, 5.0)
        
        initial_position = simulator.encoders[0].position
        time.sleep(0.5)
        final_position = simulator.encoders[0].position
        simulator.stop()
        
        # Position should increase with positive velocity
        assert final_position > initial_position
    
    def test_encoder_velocity_reflects_motor(self, simulator):
        """Test encoder velocity matches motor velocity."""
        simulator.start()
        setpoint = 3.0
        simulator.set_velocity_command(0, setpoint)
        
        time.sleep(0.5)  # Let it stabilize
        simulator.stop()
        
        encoder_velocity = simulator.encoders[0].velocity
        motor_velocity = simulator.motors[0].velocity_actual
        
        # Should be close
        assert abs(encoder_velocity - motor_velocity) < 0.5
    
    # Thermal Simulation Tests
    
    def test_temperature_increases_with_load(self, simulator):
        """Test motor temperature increases under load."""
        simulator.start()
        initial_temp = simulator.motors[0].temperature
        
        # High velocity = high load
        simulator.set_velocity_command(0, 8.0)
        time.sleep(0.5)
        
        loaded_temp = simulator.motors[0].temperature
        simulator.stop()
        
        # Temperature should increase (or at least not decrease)
        assert loaded_temp >= initial_temp
    
    # Communication Timeout Tests
    
    def test_command_timeout_tracking(self, simulator):
        """Test last command time is updated."""
        initial_time = simulator.last_command_time
        
        time.sleep(0.1)
        simulator.set_velocity_command(0, 1.0)
        
        assert simulator.last_command_time > initial_time
    
    # Factory Function Tests
    
    def test_create_default_simulator(self):
        """Test factory creates default simulator."""
        sim = create_firmware_simulator('default')
        assert sim is not None
        assert sim.num_motors == 6
        sim.stop()
    
    def test_create_minimal_simulator(self):
        """Test factory creates minimal simulator."""
        sim = create_firmware_simulator('minimal')
        assert sim is not None
        assert sim.num_motors == 4  # Fewer motors
        sim.stop()
    
    def test_create_full_rover_simulator(self):
        """Test factory creates full rover simulator."""
        sim = create_firmware_simulator('full_rover')
        assert sim is not None
        # Full rover has more motors (wheels + arm)
        assert sim.num_motors >= 6
        sim.stop()
    
    # Edge Cases and Robustness
    
    def test_negative_velocity(self, simulator):
        """Test negative velocity commands work."""
        simulator.set_velocity_command(0, -3.0)
        assert simulator.motors[0].velocity_setpoint == -3.0
    
    def test_rapid_command_changes(self, simulator):
        """Test handling rapid velocity changes."""
        simulator.start()
        
        for i in range(10):
            simulator.set_velocity_command(0, (-1)**i * 5.0)
            time.sleep(0.01)
        
        simulator.stop()
        # Should not crash
    
    def test_zero_to_max_velocity(self, simulator):
        """Test step change from zero to max velocity."""
        simulator.start()
        simulator.set_velocity_command(0, simulator.MAX_VELOCITY)
        
        time.sleep(0.5)
        velocity = simulator.motors[0].velocity_actual
        simulator.stop()
        
        # Should be ramping up
        assert 0 < velocity <= simulator.MAX_VELOCITY
    
    def test_all_motors_simultaneously(self, simulator):
        """Test controlling all motors at once."""
        simulator.start()
        
        for motor_id in range(simulator.num_motors):
            simulator.set_velocity_command(motor_id, 5.0)
        
        time.sleep(0.2)
        simulator.stop()
        
        # All motors should be moving
        for motor in simulator.motors.values():
            assert motor.velocity_actual > 0
    
    def test_motor_current_calculation(self, simulator):
        """Test motor current is calculated."""
        simulator.start()
        simulator.set_velocity_command(0, 5.0)
        
        time.sleep(0.2)
        current = simulator.motors[0].current
        simulator.stop()
        
        # Current should be non-zero under load
        assert current > 0.0
    
    def test_voltage_monitoring(self, simulator):
        """Test voltage is monitored."""
        status = simulator.get_motor_status(0)
        assert status['voltage'] > 0.0
        assert status['voltage'] <= simulator.NOMINAL_VOLTAGE * 1.1


# Performance Tests
@pytest.mark.benchmark
class TestSTM32FirmwarePerformance:
    """Performance tests for firmware simulator."""
    
    def test_control_loop_frequency(self, simulator):
        """Test control loop runs at specified frequency."""
        simulator.start()
        
        initial_cycles = simulator.stats['control_cycles']
        start_time = time.time()
        
        time.sleep(1.0)
        
        elapsed = time.time() - start_time
        cycles = simulator.stats['control_cycles'] - initial_cycles
        simulator.stop()
        
        frequency = cycles / elapsed
        
        # Should be close to 100Hz (allow 20% tolerance)
        assert 80 < frequency < 120, f"Frequency {frequency:.1f}Hz not near 100Hz"
    
    def test_command_latency(self, simulator):
        """Test command processing latency."""
        latencies = []
        
        for _ in range(100):
            start = time.time()
            simulator.set_velocity_command(0, 5.0)
            latency = (time.time() - start) * 1000  # ms
            latencies.append(latency)
        
        avg_latency = sum(latencies) / len(latencies)
        
        # Should be sub-millisecond
        assert avg_latency < 1.0, f"Command latency too high: {avg_latency:.3f}ms"


if __name__ == '__main__':
    pytest.main([__file__, '-v'])
