# SLCAN Protocol Simulator

Complete SLCAN (Serial Line CAN) protocol implementation for testing CAN communication without physical hardware. Simulates the protocol layer between ROS2 and the STM32 firmware.

## Overview

The SLCAN protocol simulator provides byte-accurate encoding/decoding of CAN messages in ASCII hex format, velocity scaling according to the protocol specification, and realistic error conditions including malformed frames and bus overload.

## Features

- **Protocol Encoding/Decoding**: Complete SLCAN ASCII hex format support
- **Velocity Scaling**: Accurate scaling (×4096 linear, ×64 angular)
- **Message ID Routing**: All system message IDs (0x00C, 0x00E, 0x110, etc.)
- **Error Injection**: Configurable error rates and malformed frames
- **Buffer Management**: Serial buffer simulation with overflow handling
- **Statistics**: Frame success rates, errors, and bus load tracking

## Quick Start

```python
from simulation.can.slcan_protocol_simulator import (
    SLCANProtocolSimulator,
    VelocityCommand,
    CANMessageID,
    create_slcan_simulator
)

# Create simulator
sim = create_slcan_simulator('default')

# Encode velocity command
frame = sim.encode_velocity_command(
    linear_x=0.5,    # m/s forward
    linear_y=0.0,    # m/s lateral (swerve)
    angular_z=0.2    # rad/s rotation
)
print(f"Encoded frame: {frame}")
# Output: t00C60008000001B4000000C8\r

# Decode velocity command
cmd = sim.decode_velocity_command(frame)
print(f"Linear X: {cmd.linear_x:.3f} m/s")
print(f"Angular Z: {cmd.angular_z:.3f} rad/s")

# Get statistics
stats = sim.get_statistics()
print(f"Success rate: {stats['success_rate']*100:.1f}%")
```

## SLCAN Protocol Format

### Frame Structure

SLCAN frames follow this format:

```
t<ID><LEN><DATA>\r
```

- `t`: Standard 11-bit ID data frame
- `<ID>`: 3 hex digits (e.g., `00C` for 0x00C)
- `<LEN>`: 1 hex digit (0-8 bytes)
- `<DATA>`: 2 hex digits per byte
- `\r`: Carriage return terminator

### Example Frames

```python
# Velocity command (ID 0x00C, 6 bytes)
't00C6001000000200000300\r'

# Heartbeat request (ID 0x00E, 0 bytes)
't00E0\r'

# Emergency stop (ID 0x1FF, 0 bytes)
't1FF0\r'
```

## Message IDs

The simulator supports all system message IDs:

| ID | Name | Direction | Purpose |
|----|------|-----------|---------|
| 0x00C | SET_CHASSIS_VELOCITIES | ROS2 → Firmware | Set rover velocity |
| 0x00D | FEEDBACK_CHASSIS_VELOCITIES | Firmware → ROS2 | Velocity feedback |
| 0x00E | HEARTBEAT_REQUEST | ROS2 → Firmware | Request heartbeat |
| 0x00F | HEARTBEAT_RESPONSE | Firmware → ROS2 | Heartbeat reply |
| 0x100 | MOTOR_COMMAND | ROS2 → Firmware | Individual motor control |
| 0x101 | MOTOR_STATUS | Firmware → ROS2 | Motor status feedback |
| 0x110 | HOMING_REQUEST | ROS2 → Firmware | Start homing sequence |
| 0x111 | HOMING_RESPONSE | Firmware → ROS2 | Homing status |
| 0x1FF | EMERGENCY_STOP | Either | Emergency stop trigger |
| 0x301 | MAST_GIMBAL | ROS2 → Firmware | Gimbal control |

## Velocity Encoding

### Scaling Factors

The protocol uses fixed-point scaling to fit floating-point velocities into 16-bit integers:

- **Linear Velocity**: Scaled by 4096 (2^12)
  - Range: ±8 m/s with 0.00024 m/s resolution
- **Angular Velocity**: Scaled by 64 (2^6) after rad → deg conversion
  - Range: ±512 deg/s (±8.9 rad/s) with 0.016 deg/s resolution

### Encoding Process

```python
# 1. Scale velocity to integer
linear_scaled = int(linear_x * 4096)

# 2. Convert to signed 16-bit
linear_bytes = linear_scaled.to_bytes(2, 'little', signed=True)

# 3. Encode as hex
hex_str = linear_bytes.hex().upper()

# Result: '0800' for 0.5 m/s
```

### Decoding Process

```python
# 1. Parse hex bytes
data_bytes = bytes.fromhex('0800')

# 2. Convert from signed 16-bit
linear_scaled = int.from_bytes(data_bytes, 'little', signed=True)

# 3. Unscale to float
linear_x = linear_scaled / 4096  # 0.5 m/s
```

## Configuration Options

```python
config = {
    'simulate_errors': True,      # Enable error injection
    'error_rate': 0.001,          # 0.1% of frames will have errors
    'serial_delay_ms': 5.0,       # Serial transmission delay
    'buffer_size': 1024,          # RX/TX buffer size in bytes
    'max_history': 1000           # Maximum frames to keep in history
}

sim = SLCANProtocolSimulator(config)
```

## Encoding Methods

### Velocity Commands

```python
# Full chassis control (most common)
frame = sim.encode_velocity_command(
    linear_x=0.5,   # Forward velocity
    linear_y=0.25,  # Lateral velocity (swerve drive)
    angular_z=0.2   # Rotational velocity
)
```

### Control Messages

```python
# Heartbeat request
frame = sim.encode_heartbeat()

# Heartbeat response
frame = sim.encode_heartbeat_response()

# Homing request
frame = sim.encode_homing_request()

# Homing response
frame = sim.encode_homing_response()

# Emergency stop
frame = sim.encode_emergency_stop()
```

### Custom Frames

```python
# Generic frame with custom data
data = b'\x01\x02\x03\x04'
frame = sim.encode_frame(
    message_id=0x123,
    data=data,
    message_type=SLCANMessageType.STANDARD_DATA
)
```

## Decoding Methods

### Parse Frame

```python
# Parse any SLCAN frame
parsed = sim.parse_frame('t00C6001000000200000300\r')

print(f"Message ID: 0x{parsed.message_id:03X}")
print(f"Data length: {parsed.data_length}")
print(f"Data: {parsed.data.hex()}")
```

### Decode Velocity

```python
# Decode velocity-specific frame
frame = 't00C60008000001B4000000C8\r'
cmd = sim.decode_velocity_command(frame)

if cmd:
    print(f"Linear X: {cmd.linear_x:.3f} m/s")
    print(f"Linear Y: {cmd.linear_y:.3f} m/s")
    print(f"Angular Z: {cmd.angular_z:.3f} rad/s")
```

## Error Injection

### Automatic Errors

Enable random error injection:

```python
sim = SLCANProtocolSimulator({
    'simulate_errors': True,
    'error_rate': 0.05  # 5% error rate
})

# Errors will be injected randomly during encoding/decoding
```

### Manual Errors

```python
original_frame = 't00C6001000000200000300\r'
corrupted_frame = sim.inject_error(original_frame)

# Corrupted frame may have:
# - Wrong checksum
# - Bit flips in data
# - Invalid message type
# - Truncated data
```

### Error Types

The simulator can inject:
- **Bit flips**: Random bits flipped in hex data
- **Truncation**: Frame cut short
- **Invalid hex**: Non-hex characters in data
- **Wrong length**: Data length doesn't match declared length
- **Invalid ID**: Message ID out of range

## Buffer Management

### Writing to Buffer

```python
frame = sim.encode_velocity_command(0.5, 0.0, 0.0)

# Write to TX buffer
success = sim.write_to_buffer(frame)

if not success:
    print("Buffer overflow!")
```

### Reading from Buffer

```python
# Simulate received data in RX buffer
sim.rx_buffer.extend(b't00E0\r')

# Read frame
frame = sim.read_from_buffer()
if frame:
    print(f"Received: {frame}")
```

### Buffer Overflow Handling

```python
# Set small buffer for testing
sim.buffer_size = 100

# Fill buffer
for i in range(100):
    success = sim.write_to_buffer(f't00E0\r')
    if not success:
        print(f"Overflow at message {i}")
        break

# Check statistics
print(f"Overflows: {sim.stats['buffer_overflows']}")
```

## Statistics

### Available Metrics

```python
stats = sim.get_statistics()

# Frame counts
print(f"Frames sent: {stats['frames_sent']}")
print(f"Frames received: {stats['frames_received']}")
print(f"Frames dropped: {stats['frames_dropped']}")

# Error counts
print(f"Encoding errors: {stats['encoding_errors']}")
print(f"Decoding errors: {stats['decoding_errors']}")
print(f"Malformed frames: {stats['malformed_frames']}")
print(f"Buffer overflows: {stats['buffer_overflows']}")

# Success rate
print(f"Success rate: {stats['success_rate']*100:.1f}%")

# Bus load
print(f"Bus load: {stats['bus_load']*100:.1f}%")
```

### Bus Load Tracking

```python
# Send messages
for i in range(100):
    sim.encode_velocity_command(0.5, 0.0, 0.0)

# Check bus load (0.0 = idle, 1.0 = saturated)
load = sim.get_bus_load()
print(f"Bus load: {load*100:.1f}%")
```

### Reset Statistics

```python
sim.reset_statistics()
```

## Testing Patterns

### Encoding Accuracy Test

```python
def test_encoding_accuracy():
    sim = create_slcan_simulator('perfect')
    
    # Test roundtrip
    original = (0.5, 0.25, 0.2618)
    frame = sim.encode_velocity_command(*original)
    decoded = sim.decode_velocity_command(frame)
    
    # Verify accuracy
    assert abs(decoded.linear_x - original[0]) < 0.001
    assert abs(decoded.linear_y - original[1]) < 0.001
    assert abs(decoded.angular_z - original[2]) < 0.001
```

### Error Handling Test

```python
def test_error_handling():
    sim = create_slcan_simulator('default')
    
    malformed_frames = [
        "invalid",
        "t00C",           # Too short
        "t00CZZZZZZ\r",   # Invalid hex
        "x00C6000000\r"   # Wrong message type
    ]
    
    for frame in malformed_frames:
        parsed = sim.parse_frame(frame)
        assert parsed is None  # Should reject gracefully
        
    # Check error statistics
    assert sim.stats['malformed_frames'] > 0
```

### Performance Test

```python
def test_encoding_performance():
    sim = create_slcan_simulator('perfect')
    
    start = time.time()
    count = 10000
    
    for i in range(count):
        sim.encode_velocity_command(0.5, 0.0, 0.1)
    
    duration = time.time() - start
    rate = count / duration
    
    print(f"Encoding rate: {rate:.0f} frames/sec")
    assert rate > 10000  # Should encode >10k frames/sec
```

## Profiles

### Predefined Configurations

```python
# Perfect communication (no errors)
sim = create_slcan_simulator('perfect')
# error_rate = 0.0, serial_delay_ms = 0.0

# Default (realistic)
sim = create_slcan_simulator('default')
# error_rate = 0.001 (0.1%), serial_delay_ms = 5.0

# Stressed (high error rate)
sim = create_slcan_simulator('stressed')
# error_rate = 0.05 (5%), serial_delay_ms = 20.0
```

## Troubleshooting

### Decoding Returns None

**Causes:**
1. Invalid frame format
2. Wrong message ID (not 0x00C for velocity)
3. Malformed hex data
4. Missing terminator (\r)

**Solution:**
```python
# Debug frame parsing
parsed = sim.parse_frame(frame)
if parsed is None:
    print("Frame parsing failed")
    print(f"Frame: {repr(frame)}")
else:
    print(f"Parsed OK, ID: 0x{parsed.message_id:03X}")
```

### Velocity Precision Loss

**Cause:** Integer quantization during scaling

**Solution:**
```python
# Check acceptable error
tolerance = 1.0 / 4096  # ~0.00024 m/s for linear
assert abs(decoded.linear_x - original) < tolerance
```

### Buffer Overflows

**Cause:** Too many frames queued

**Solution:**
```python
# Increase buffer size
sim.buffer_size = 4096

# Or clear buffers periodically
sim.clear_buffers()
```

## Integration Examples

### With CAN Bridge

```python
from src.bridges.can_bridge import CANBridge

# Create simulator
slcan_sim = create_slcan_simulator('default')

# In bridge, use simulator instead of real serial
class SimulatedCANBridge(CANBridge):
    def _write_frame(self, frame):
        slcan_sim.write_to_buffer(frame)
    
    def _read_frame(self):
        return slcan_sim.read_from_buffer()
```

### With Firmware Simulator

```python
from simulation.firmware.stm32_firmware_simulator import STM32FirmwareSimulator

slcan_sim = create_slcan_simulator('perfect')
firmware_sim = STM32FirmwareSimulator()

# Encode command
frame = slcan_sim.encode_velocity_command(0.5, 0.0, 0.2)

# Parse and send to firmware
cmd = slcan_sim.decode_velocity_command(frame)
firmware_sim.set_chassis_velocities(cmd.linear_x, cmd.linear_y, cmd.angular_z)
```

## API Reference

### Main Class

- `SLCANProtocolSimulator(config)`: Create simulator instance
- `encode_velocity_command(linear_x, linear_y, angular_z)`: Encode velocity
- `decode_velocity_command(frame)`: Decode velocity
- `parse_frame(slcan_frame)`: Parse any SLCAN frame
- `encode_frame(message_id, data, message_type)`: Encode generic frame
- `encode_heartbeat()`: Encode heartbeat request
- `encode_emergency_stop()`: Encode e-stop
- `write_to_buffer(frame)`: Write to TX buffer
- `read_from_buffer()`: Read from RX buffer
- `get_statistics()`: Get comprehensive statistics
- `inject_error(frame)`: Manually corrupt frame

### Factory Function

- `create_slcan_simulator(profile)`: Create with predefined profile

### Data Classes

- `SLCANFrame`: Parsed frame structure
- `VelocityCommand`: Decoded velocity command
- `CANMessageID`: Enum of message IDs
- `SLCANMessageType`: Message type enum

## Examples

See `simulation/examples/slcan_protocol_demo.py` for complete usage examples.

## Related

- **WebSocket Simulator**: `simulation/network/websocket_server_simulator.py`
- **Firmware Simulator**: `simulation/firmware/stm32_firmware_simulator.py`
- **CAN Bridge**: `src/bridges/can_bridge.py`
