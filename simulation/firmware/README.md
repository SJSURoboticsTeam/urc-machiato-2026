# STM32 Firmware Behavior Simulator

Simulates the control-systems submodule STM32 firmware for testing motor control, encoder feedback, and fault handling without physical hardware.

## Overview

The firmware simulator provides realistic motor dynamics, encoder feedback, thermal simulation, and comprehensive fault injection capabilities. It operates with a real-time control loop (100Hz by default) to accurately represent hardware behavior.

## Features

- **Motor Control Loop**: Real-time velocity control with ramping and limits
- **Encoder Simulation**: Position and velocity feedback with configurable noise
- **Thermal Modeling**: Temperature simulation based on load and cooling
- **Fault Injection**: Overcurrent, overtemperature, encoder failures
- **Emergency Stop**: Immediate safety shutdown
- **Homing Sequence**: Automated homing procedure simulation
- **Multi-Motor**: Supports 4-10 motors (configurable)

## Quick Start

```python
from simulation.firmware.stm32_firmware_simulator import (
    STM32FirmwareSimulator,
    MotorFaultType,
    HomingState,
    create_firmware_simulator
)

# Create simulator
sim = create_firmware_simulator('default')  # 6 motors

# Start control loop
sim.start()

# Set motor velocity
sim.set_velocity_command(motor_id=0, velocity=5.0)  # rad/s

# Wait for response
time.sleep(0.5)

# Check motor status
status = sim.get_motor_status(0)
print(f"Velocity: {status['velocity_actual']:.2f} rad/s")
print(f"Position: {status['position']:.2f} rad")
print(f"Current: {status['current']:.2f} A")
print(f"Temperature: {status['temperature']:.1f} °C")

# Stop simulator
sim.stop()
```

## Configuration Options

```python
config = {
    'num_motors': 6,              # Number of motors to simulate
    'simulate_faults': True,      # Enable random fault injection
    'fault_rate': 0.0001,         # Fault probability per control cycle
    'control_loop_hz': 100,       # Control loop frequency
    'homing_duration': 5.0,       # Homing sequence duration (seconds)
    'command_timeout': 1.0        # Command watchdog timeout
}

sim = STM32FirmwareSimulator(config)
```

## Motor Control

### Individual Motor Control

```python
# Set single motor velocity (rad/s)
success = sim.set_velocity_command(motor_id=0, velocity=5.0)

# Set negative velocity (reverse)
sim.set_velocity_command(motor_id=0, velocity=-3.0)

# Stop motor
sim.set_velocity_command(motor_id=0, velocity=0.0)
```

### Chassis-Level Control

```python
# Set rover-level velocities
success = sim.set_chassis_velocities(
    linear_x=0.5,   # m/s forward
    linear_y=0.0,   # m/s lateral (swerve)
    angular_z=0.2   # rad/s rotation
)

# Commands are automatically distributed to individual wheels
```

### Velocity Limits

Velocities are automatically clamped to safe limits:

```python
# Maximum velocity: 10.0 rad/s
sim.set_velocity_command(0, 20.0)  # Clamped to 10.0

# Motor physical limits respected
print(f"Max velocity: {sim.MAX_VELOCITY} rad/s")
print(f"Max current: {sim.MAX_CURRENT} A")
print(f"Max temperature: {sim.MAX_TEMPERATURE} °C")
```

## Control Loop

### Starting and Stopping

```python
# Start control loop thread
sim.start()
print(f"Running: {sim.running}")

# Stop control loop
sim.stop()
print(f"Running: {sim.running}")
```

### Control Loop Behavior

The simulator runs a real-time control loop that:
1. Updates motor velocities (with ramping)
2. Integrates encoder positions
3. Simulates thermal effects
4. Checks for faults
5. Runs at configured frequency (default 100Hz)

```python
# Check control loop statistics
print(f"Control cycles: {sim.stats['control_cycles']}")
print(f"Commands received: {sim.stats['commands_received']}")
```

## Velocity Ramping

Velocities ramp smoothly to prevent mechanical stress:

```python
sim.start()

# Command step change
sim.set_velocity_command(0, 5.0)

# Velocity ramps up gradually
time.sleep(0.1)
print(f"After 0.1s: {sim.motors[0].velocity_actual:.2f} rad/s")

time.sleep(0.5)
print(f"After 0.6s: {sim.motors[0].velocity_actual:.2f} rad/s")

sim.stop()

# Ramp rate: 5.0 rad/s² (configurable)
```

## Motor Status

### Individual Motor Status

```python
status = sim.get_motor_status(motor_id=0)

# Available fields
print(f"Motor ID: {status['motor_id']}")
print(f"Velocity setpoint: {status['velocity_setpoint']} rad/s")
print(f"Velocity actual: {status['velocity_actual']} rad/s")
print(f"Position: {status['position']} rad")
print(f"Current: {status['current']} A")
print(f"Temperature: {status['temperature']} °C")
print(f"Voltage: {status['voltage']} V")
print(f"Fault: {status['fault']}")
print(f"Enabled: {status['enabled']}")
```

### System Status

```python
status = sim.get_system_status()

# System-wide information
print(f"Total motors: {status['num_motors']}")
print(f"Emergency stop: {status['emergency_stop']}")
print(f"Homing state: {status['homing_state']}")
print(f"Motors faulted: {status['motors_faulted']}")
print(f"Uptime: {status['uptime_s']:.1f}s")
print(f"Control loop frequency: {status['control_loop_hz']} Hz")

# Individual motor summaries
for motor_status in status['motors']:
    print(f"Motor {motor_status['motor_id']}: {motor_status['velocity_actual']:.2f} rad/s")
```

## Encoder Feedback

### Reading Encoders

```python
# Get encoder for specific motor
encoder = sim.encoders[motor_id]

print(f"Position: {encoder.position:.2f} rad")
print(f"Velocity: {encoder.velocity:.2f} rad/s")
print(f"Raw counts: {encoder.raw_counts}")
print(f"Valid: {encoder.valid}")
```

### Encoder Noise

Encoders include configurable noise for realistic simulation:

```python
# Noise standard deviation in counts
encoder = sim.encoders[0]
print(f"Noise std dev: {encoder.noise_std} counts")

# Position will vary slightly from true position
```

## Emergency Stop

### Activating E-Stop

```python
# Trigger emergency stop
success = sim.emergency_stop()

# All motors immediately stop
for motor in sim.motors.values():
    assert motor.velocity_setpoint == 0.0

# System state reflects e-stop
assert sim.emergency_stop_active == True
```

### Clearing E-Stop

```python
# Clear emergency stop
success = sim.clear_emergency_stop()

assert sim.emergency_stop_active == False

# Motors can now be controlled again
sim.set_velocity_command(0, 3.0)
```

### E-Stop Statistics

```python
# Track emergency stop events
print(f"E-stops triggered: {sim.stats['emergency_stops']}")
```

## Homing Sequence

### Starting Homing

```python
# Start homing procedure
success = sim.start_homing_sequence()

print(f"Homing state: {sim.homing_state}")  # HomingState.HOMING
print(f"Progress: {sim.homing_progress*100:.0f}%")
```

### Monitoring Progress

```python
sim.start()
sim.start_homing_sequence()

# Check progress periodically
while sim.homing_state == HomingState.HOMING:
    progress = sim.homing_progress * 100
    print(f"Homing: {progress:.0f}%")
    time.sleep(0.5)

print(f"Final state: {sim.homing_state}")  # HomingState.COMPLETE
sim.stop()
```

### Homing States

```python
from simulation.firmware.stm32_firmware_simulator import HomingState

# Possible states:
# HomingState.IDLE       - Not homing
# HomingState.HOMING     - In progress
# HomingState.COMPLETE   - Successfully completed
# HomingState.FAILED     - Failed (fault occurred)
```

## Fault Injection

### Manual Fault Injection

```python
# Inject overcurrent fault
success = sim.inject_fault(motor_id=0, fault_type=MotorFaultType.OVERCURRENT)

# Check motor status
status = sim.get_motor_status(0)
assert status['fault'] == 'overcurrent'

# Clear fault
sim.clear_fault(motor_id=0)
```

### Fault Types

```python
from simulation.firmware.stm32_firmware_simulator import MotorFaultType

# Available fault types:
MotorFaultType.NONE                  # No fault
MotorFaultType.OVERCURRENT           # Current limit exceeded
MotorFaultType.OVERTEMPERATURE       # Thermal limit exceeded
MotorFaultType.ENCODER_FAILURE       # Encoder malfunction
MotorFaultType.TIMEOUT               # Communication timeout
MotorFaultType.COMMUNICATION_ERROR   # CAN/serial error
MotorFaultType.HARDWARE_FAULT        # General hardware fault
```

### Fault Effects

Different faults have different effects:

```python
# Overcurrent: Motor disabled, high current reading
sim.inject_fault(0, MotorFaultType.OVERCURRENT)
assert sim.motors[0].current > sim.MAX_CURRENT * 1.2

# Overtemperature: Temperature elevated
sim.inject_fault(0, MotorFaultType.OVERTEMPERATURE)
assert sim.motors[0].temperature > 70.0

# Encoder failure: Encoder marked invalid
sim.inject_fault(0, MotorFaultType.ENCODER_FAILURE)
assert sim.encoders[0].valid == False
```

### Random Fault Injection

```python
# Enable automatic random faults
sim = STM32FirmwareSimulator({
    'simulate_faults': True,
    'fault_rate': 0.001  # 0.1% per control cycle
})

sim.start()

# Faults will occur randomly during operation
time.sleep(10.0)

sim.stop()

print(f"Faults injected: {sim.stats['faults_injected']}")
```

## Thermal Simulation

### Temperature Dynamics

Motor temperature changes based on load:

```python
sim.start()

# High load increases temperature
sim.set_velocity_command(0, 8.0)  # High velocity

initial_temp = sim.motors[0].temperature
time.sleep(2.0)
loaded_temp = sim.motors[0].temperature

print(f"Temperature rise: {loaded_temp - initial_temp:.1f}°C")

# Cooling when idle
sim.set_velocity_command(0, 0.0)
time.sleep(5.0)
cooled_temp = sim.motors[0].temperature

print(f"Cooled to: {cooled_temp:.1f}°C")

sim.stop()
```

### Thermal Parameters

```python
# Thermal time constant (cooling rate)
print(f"Thermal TC: {sim.THERMAL_TIME_CONSTANT}s")

# Temperature limits
print(f"Max temperature: {sim.MAX_TEMPERATURE}°C")
```

## Current Simulation

Motor current is simulated based on load:

```python
sim.start()

# No load: Low current
sim.set_velocity_command(0, 0.0)
time.sleep(0.1)
print(f"Idle current: {sim.motors[0].current:.2f} A")

# High load: Higher current
sim.set_velocity_command(0, 8.0)
time.sleep(0.5)
print(f"Loaded current: {sim.motors[0].current:.2f} A")

sim.stop()
```

## Testing Patterns

### Basic Motion Test

```python
def test_basic_motion():
    sim = create_firmware_simulator('default')
    sim.start()
    
    # Command velocity
    sim.set_velocity_command(0, 5.0)
    time.sleep(1.0)
    
    # Verify motion
    status = sim.get_motor_status(0)
    assert status['velocity_actual'] > 4.0  # Close to setpoint
    assert status['position'] > 0  # Position increased
    
    sim.stop()
```

### Emergency Stop Test

```python
def test_emergency_stop():
    sim = create_firmware_simulator('default')
    sim.start()
    
    # Set motion
    for motor_id in range(sim.num_motors):
        sim.set_velocity_command(motor_id, 5.0)
    
    time.sleep(0.2)
    
    # Trigger e-stop
    sim.emergency_stop()
    
    # All motors should stop
    for motor in sim.motors.values():
        assert motor.velocity_setpoint == 0.0
    
    sim.stop()
```

### Fault Recovery Test

```python
def test_fault_recovery():
    sim = create_firmware_simulator('default')
    
    # Inject fault
    sim.inject_fault(0, MotorFaultType.OVERCURRENT)
    assert sim.motors[0].fault == MotorFaultType.OVERCURRENT
    
    # Clear fault
    sim.clear_fault(0)
    assert sim.motors[0].fault == MotorFaultType.NONE
    
    # Motor operational again
    sim.start()
    sim.set_velocity_command(0, 3.0)
    time.sleep(0.5)
    assert sim.motors[0].velocity_actual > 0
    sim.stop()
```

## Profiles

### Predefined Configurations

```python
# Default: 6 motors, standard parameters
sim = create_firmware_simulator('default')

# Minimal: 4 motors, faster for testing
sim = create_firmware_simulator('minimal')

# Full rover: 10+ motors (wheels + arm)
sim = create_firmware_simulator('full_rover')
```

## Statistics

Track simulator activity:

```python
stats = sim.stats

print(f"Control cycles: {stats['control_cycles']}")
print(f"Commands received: {stats['commands_received']}")
print(f"Faults injected: {stats['faults_injected']}")
print(f"Emergency stops: {stats['emergency_stops']}")
print(f"Homing sequences: {stats['homing_sequences']}")
```

## Troubleshooting

### Motors Not Moving

1. Check control loop is running: `sim.running == True`
2. Verify no emergency stop: `sim.emergency_stop_active == False`
3. Check for faults: `sim.motors[id].fault == MotorFaultType.NONE`
4. Ensure command sent: `sim.motors[id].velocity_setpoint != 0`

### Velocity Not Reaching Setpoint

1. Check ramp rate: May take time to reach setpoint
2. Verify no velocity limits: Setpoint within `MAX_VELOCITY`
3. Check for faults that limit motion

### High CPU Usage

1. Lower control loop frequency: `control_loop_hz = 50`
2. Reduce number of motors: `num_motors = 4`
3. Disable fault simulation: `simulate_faults = False`

## Integration Examples

### With SLCAN Protocol

```python
from simulation.can.slcan_protocol_simulator import SLCANProtocolSimulator

slcan_sim = SLCANProtocolSimulator()
firmware_sim = STM32FirmwareSimulator()
firmware_sim.start()

# Decode SLCAN command
frame = 't00C60008000001B4000000C8\r'
cmd = slcan_sim.decode_velocity_command(frame)

# Send to firmware
firmware_sim.set_chassis_velocities(cmd.linear_x, cmd.linear_y, cmd.angular_z)

# Get feedback
status = firmware_sim.get_system_status()

# Encode feedback
feedback_frame = slcan_sim.encode_velocity_command(
    status['motors'][0]['velocity_actual'],
    0.0,
    0.0
)
```

## API Reference

### Main Class

- `STM32FirmwareSimulator(config)`: Create simulator
- `start()`: Start control loop
- `stop()`: Stop control loop
- `set_velocity_command(motor_id, velocity)`: Set motor velocity
- `set_chassis_velocities(linear_x, linear_y, angular_z)`: Set rover velocity
- `emergency_stop()`: Trigger emergency stop
- `clear_emergency_stop()`: Clear emergency stop
- `start_homing_sequence()`: Start homing
- `get_motor_status(motor_id)`: Get motor status
- `get_system_status()`: Get system status
- `inject_fault(motor_id, fault_type)`: Inject fault
- `clear_fault(motor_id)`: Clear fault

### Factory Function

- `create_firmware_simulator(profile)`: Create with profile

### Data Classes

- `MotorState`: Motor state structure
- `EncoderState`: Encoder state structure
- `MotorFaultType`: Fault type enum
- `HomingState`: Homing state enum

## Examples

See `simulation/examples/firmware_simulator_demo.py` for complete usage examples.

## Related

- **SLCAN Protocol**: `simulation/can/slcan_protocol_simulator.py`
- **Full Stack Simulator**: `simulation/integration/full_stack_simulator.py`
- **Control Systems Submodule**: `vendor/control-systems/`
