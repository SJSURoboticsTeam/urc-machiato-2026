# Full Stack Communication Simulator

Orchestrates complete end-to-end communication simulation for testing all paths without hardware: Frontend → WebSocket → ROS2 → SLCAN → Firmware and back.

## Overview

The full stack simulator integrates the WebSocket server, SLCAN protocol, and firmware simulators into a complete system that accurately represents the rover's communication architecture. It provides pre-built test scenarios and comprehensive metrics collection.

## Features

- **Complete E2E Pipeline**: All communication paths in one system
- **Test Scenarios**: Pre-built scenarios for common test cases
- **Metrics Collection**: Comprehensive statistics across all layers
- **Event Tracking**: Complete message flow recording
- **Fault Simulation**: Network failures, firmware faults, high load
- **Test Suite**: Automated test execution with pass/fail tracking

## Quick Start

```python
from simulation.integration.full_stack_simulator import (
    FullStackSimulator,
    ScenarioType,
    create_full_stack_simulator
)
import asyncio

# Create simulator
sim = create_full_stack_simulator('default')

# Run a test scenario
async def test():
    result = await sim.run_scenario(ScenarioType.BASIC_VELOCITY)
    
    print(f"Success: {result.success}")
    print(f"Duration: {result.duration_s:.3f}s")
    print(f"Messages: {result.messages_sent}/{result.messages_received}")
    
    if not result.success:
        print(f"Errors: {result.errors}")
    
    # Cleanup
    sim.shutdown()

asyncio.run(test())
```

## Architecture

The full stack simulator represents this communication flow:

```
Frontend/Operator
       ↕
WebSocket/Socket.IO Server
       ↕
ROS2 System (cmd_vel, feedback)
       ↕
SLCAN Protocol Adapter
       ↕
CAN Bus (Serial)
       ↕
STM32 Firmware (Motors)
```

All components are simulated, allowing complete testing without hardware.

## Test Scenarios

### Available Scenarios

```python
from simulation.integration.full_stack_simulator import ScenarioType

# Basic velocity command propagation
ScenarioType.BASIC_VELOCITY

# Emergency stop through all layers
ScenarioType.EMERGENCY_STOP

# Network failure and recovery
ScenarioType.NETWORK_FAILURE

# Firmware fault handling
ScenarioType.FIRMWARE_FAULT

# High message load (stress test)
ScenarioType.HIGH_LOAD

# Disconnection and reconnection
ScenarioType.RECOVERY
```

### Running Scenarios

```python
# Run single scenario
result = await sim.run_scenario(ScenarioType.BASIC_VELOCITY)

# Run with custom parameters
result = await sim.run_scenario(
    ScenarioType.BASIC_VELOCITY,
    params={'duration': 2.0, 'velocity': 0.8}
)

# Check result
if result.success:
    print(f"✅ Passed in {result.duration_s:.3f}s")
else:
    print(f"❌ Failed: {result.errors}")
```

### Scenario Descriptions

#### BASIC_VELOCITY
Tests basic velocity command flow from frontend to firmware.

```python
result = await sim.run_scenario(ScenarioType.BASIC_VELOCITY, {
    'duration': 1.0,      # Test duration (seconds)
    'velocity': 0.5       # Test velocity (m/s)
})

# Validates:
# - WebSocket receives command
# - ROS2 state updated
# - SLCAN encodes correctly
# - Firmware motors respond
```

#### EMERGENCY_STOP
Tests emergency stop propagation and timing (<100ms requirement).

```python
result = await sim.run_scenario(ScenarioType.EMERGENCY_STOP)

# Validates:
# - E-stop reaches firmware quickly
# - All motors stop
# - System enters safe state
# - Timing requirement met
```

#### NETWORK_FAILURE
Tests behavior under network degradation (high latency, packet loss).

```python
result = await sim.run_scenario(ScenarioType.NETWORK_FAILURE, {
    'packet_loss': 0.3,   # 30% packet loss
    'latency_ms': 200.0   # 200ms latency
})

# Validates:
# - Commands still get through
# - System remains stable
# - Graceful degradation
```

#### FIRMWARE_FAULT
Tests handling of firmware faults.

```python
result = await sim.run_scenario(ScenarioType.FIRMWARE_FAULT, {
    'fault_type': 'overcurrent',  # Type of fault to inject
    'recovery_expected': True      # Should recover?
})

# Validates:
# - Fault detected
# - Error propagates to operator
# - Recovery possible if applicable
```

#### HIGH_LOAD
Tests system under sustained high message rate.

```python
result = await sim.run_scenario(ScenarioType.HIGH_LOAD, {
    'message_rate': 100,  # Messages per second
    'duration': 10.0      # Test duration
})

# Validates:
# - No message drops
# - Performance acceptable
# - No resource exhaustion
```

#### RECOVERY
Tests reconnection and state recovery after disconnect.

```python
result = await sim.run_scenario(ScenarioType.RECOVERY, {
    'disconnect_duration': 2.0  # Seconds disconnected
})

# Validates:
# - Client reconnects
# - State synchronized
# - Operations resume
```

## Test Suite

### Running Full Suite

```python
# Run all scenarios
summary = await sim.run_test_suite()

print(f"Total scenarios: {summary['total_scenarios']}")
print(f"Passed: {summary['passed']}")
print(f"Failed: {summary['failed']}")
print(f"Pass rate: {summary['pass_rate']*100:.1f}%")
print(f"Duration: {summary['duration_s']:.1f}s")

# Individual results
for result in summary['results']:
    status = "✅" if result['success'] else "❌"
    print(f"{status} {result['scenario']}: {result['duration_s']:.3f}s")
```

### Test Suite Output

```
Running Full Test Suite...

✅ BASIC_VELOCITY passed in 0.523s
✅ EMERGENCY_STOP passed in 0.087s
⚠️  NETWORK_FAILURE passed (degraded) in 2.145s
✅ FIRMWARE_FAULT passed in 1.234s
✅ HIGH_LOAD passed in 10.012s
✅ RECOVERY passed in 3.456s

Summary:
- Total: 6 scenarios
- Passed: 6
- Failed: 0
- Pass rate: 100.0%
- Duration: 17.5s
```

## Configuration

### Component Configuration

Configure individual components:

```python
config = {
    'websocket': {
        'network': {
            'latency_ms': 50.0,
            'packet_loss_rate': 0.01
        }
    },
    'slcan': {
        'error_rate': 0.001,
        'serial_delay_ms': 5.0
    },
    'firmware': {
        'num_motors': 6,
        'simulate_faults': True,
        'fault_rate': 0.0001
    },
    'max_queue_size': 1000
}

sim = FullStackSimulator(config)
```

### Predefined Profiles

```python
# Perfect: No delays or errors
sim = create_full_stack_simulator('perfect')

# Default: Realistic conditions
sim = create_full_stack_simulator('default')

# Stressed: High latency, errors
sim = create_full_stack_simulator('stressed')
```

## Communication Paths

### Teleoperation Path

Frontend operator commands to firmware:

```python
# Simulate operator command
client_id = await sim.websocket_sim.connect()

await sim.websocket_sim.receive('driveCommands', {
    'linear': 0.5,
    'angular': 0.2
}, client_id)

# Wait for propagation
await asyncio.sleep(0.1)

# Check it reached firmware
motor_status = sim.firmware_sim.get_motor_status(0)
print(f"Motor velocity: {motor_status['velocity_actual']:.2f} rad/s")
```

### Feedback Path

Firmware status back to frontend:

```python
# Get firmware status
firmware_status = sim.firmware_sim.get_system_status()

# Emit to frontend
await sim.websocket_sim.emit('systemStatus', {
    'motors': firmware_status['motors'],
    'emergency_stop': firmware_status['emergency_stop']
}, client_id)
```

### Emergency Stop Path

E-stop propagation through all layers:

```python
# Operator hits e-stop
await sim.websocket_sim.receive('emergencyStop', {}, client_id)

# Verify it reached firmware
assert sim.firmware_sim.emergency_stop_active == True

# All motors stopped
for motor in sim.firmware_sim.motors.values():
    assert motor.velocity_setpoint == 0.0
```

## Metrics and Statistics

### System-Wide Metrics

```python
metrics = sim._collect_metrics()

# WebSocket metrics
print(f"WS connected: {metrics['websocket']['connected_clients']}")
print(f"WS messages: {metrics['websocket']['stats']['messages_received']}")

# SLCAN metrics
print(f"SLCAN success rate: {metrics['slcan']['success_rate']*100:.1f}%")
print(f"SLCAN frames: {metrics['slcan']['frames_sent']}")

# Firmware metrics
print(f"Firmware motors: {metrics['firmware']['num_motors']}")
print(f"Firmware e-stop: {metrics['firmware']['emergency_stop']}")
```

### Scenario Statistics

```python
# Simulator tracks all scenario results
print(f"Scenarios run: {sim.stats['scenarios_run']}")
print(f"Scenarios passed: {sim.stats['scenarios_passed']}")
print(f"Scenarios failed: {sim.stats['scenarios_failed']}")
print(f"Total messages: {sim.stats['total_messages']}")
print(f"Total errors: {sim.stats['total_errors']}")

# Access individual results
for result in sim.scenario_results:
    print(f"{result.scenario_type.value}: {result.success}")
```

### Event Logging

```python
# Events are logged with full context
for event in sim.event_queue[-10:]:  # Last 10 events
    print(f"{event['timestamp']:.3f}: {event['type']} on {event['path'].value}")
```

## Status Monitoring

### Real-Time Status

```python
status = sim.get_status()

# Component states
for component, state in status['components'].items():
    print(f"{component}: {state}")

# ROS2 state
print(f"Teleop cmd: {status['ros2_state']['cmd_vel_teleop']}")
print(f"E-stop: {status['ros2_state']['emergency_stop']}")

# Recent scenarios
for scenario in status['scenario_history'][-5:]:
    print(f"{scenario['type']}: {'✅' if scenario['success'] else '❌'}")
```

## Reset and Cleanup

### Reset State

```python
# Clear all state and statistics
sim.reset()

# Verify reset
assert sim.stats['scenarios_run'] == 0
assert len(sim.scenario_results) == 0
assert len(sim.event_queue) == 0
```

### Clean Shutdown

```python
# Stop all components
sim.shutdown()

# Firmware control loop stopped
assert sim.firmware_sim.running == False
```

## Testing Patterns

### Basic Integration Test

```python
async def test_basic_integration():
    sim = create_full_stack_simulator('perfect')
    
    # Send command
    client_id = await sim.websocket_sim.connect()
    await sim.websocket_sim.receive('driveCommands', {
        'linear': 0.5, 'angular': 0.0
    }, client_id)
    
    await asyncio.sleep(0.2)
    
    # Verify propagation
    assert sim.ros2_state['cmd_vel_teleop'] is not None
    motor_status = sim.firmware_sim.get_motor_status(0)
    assert motor_status['velocity_actual'] > 0
    
    sim.shutdown()
```

### Performance Test

```python
async def test_throughput():
    sim = create_full_stack_simulator('perfect')
    client_id = await sim.websocket_sim.connect()
    
    start = time.time()
    count = 1000
    
    for i in range(count):
        await sim.websocket_sim.receive('driveCommands', {
            'linear': 0.5, 'angular': 0.0
        }, client_id)
    
    duration = time.time() - start
    throughput = count / duration
    
    print(f"Throughput: {throughput:.0f} msg/s")
    assert throughput > 500  # Should handle >500 msg/s
    
    sim.shutdown()
```

### Fault Injection Test

```python
async def test_fault_handling():
    sim = create_full_stack_simulator('default')
    
    # Inject fault
    sim.firmware_sim.inject_fault(0, MotorFaultType.OVERCURRENT)
    
    # Run scenario
    result = await sim.run_scenario(ScenarioType.FIRMWARE_FAULT)
    
    # Should detect and handle fault
    assert result.success == True
    assert 'fault' in str(result.metrics).lower()
    
    sim.shutdown()
```

## Troubleshooting

### Scenario Fails

1. Check individual component status:
   ```python
   print(sim.websocket_sim.get_statistics())
   print(sim.slcan_sim.get_statistics())
   print(sim.firmware_sim.get_system_status())
   ```

2. Review scenario errors:
   ```python
   result = await sim.run_scenario(ScenarioType.BASIC_VELOCITY)
   if not result.success:
       for error in result.errors:
           print(f"Error: {error}")
   ```

3. Check event log:
   ```python
   for event in sim.event_queue[-20:]:
       print(event)
   ```

### Low Throughput

1. Use perfect profile: `create_full_stack_simulator('perfect')`
2. Check network conditions: `sim.websocket_sim.network_conditions`
3. Verify no error injection: `sim.slcan_sim.simulate_errors = False`

### Messages Not Propagating

1. Verify firmware started: `sim.firmware_sim.running == True`
2. Check no e-stop: `sim.firmware_sim.emergency_stop_active == False`
3. Review ROS2 state: `print(sim.ros2_state)`

## Advanced Usage

### Custom Scenarios

```python
async def custom_scenario(sim, params):
    """Custom test scenario."""
    try:
        # Your test logic here
        client_id = await sim.websocket_sim.connect()
        
        # ... test steps ...
        
        return True  # Success
    except Exception as e:
        return False  # Failure

# Run custom scenario
success = await custom_scenario(sim, {})
```

### Scenario Recording

```python
# Run scenario and capture all events
sim.event_queue.clear()

result = await sim.run_scenario(ScenarioType.BASIC_VELOCITY)

# Export event log
import json
with open('output/scenario_events.json', 'w') as f:
    json.dump(sim.event_queue, f, indent=2, default=str)
```

### Performance Profiling

```python
import cProfile

# Profile test suite
profiler = cProfile.Profile()
profiler.enable()

summary = await sim.run_test_suite()

profiler.disable()
profiler.print_stats(sort='cumtime')
```

## API Reference

### Main Class

- `FullStackSimulator(config)`: Create simulator
- `run_scenario(scenario_type, params)`: Run test scenario
- `run_test_suite()`: Run all scenarios
- `get_status()`: Get complete status
- `reset()`: Clear state and statistics
- `shutdown()`: Stop all components

### Factory Function

- `create_full_stack_simulator(profile)`: Create with profile

### Enums

- `ScenarioType`: Available test scenarios
- `CommunicationPath`: Communication paths

### Data Classes

- `ScenarioResult`: Scenario execution result

## Examples

See `simulation/examples/full_stack_demo.py` for complete usage examples.

## Related

- **WebSocket Simulator**: `simulation/network/websocket_server_simulator.py`
- **SLCAN Simulator**: `simulation/can/slcan_protocol_simulator.py`
- **Firmware Simulator**: `simulation/firmware/stm32_firmware_simulator.py`
- **Integration Tests**: `tests/integration/test_complete_communication_stack.py`
