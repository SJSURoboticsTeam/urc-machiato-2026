# WebSocket/Socket.IO Server Simulator

Simulates the teleoperation WebSocket server for testing frontend/backend integration without requiring physical hardware or the actual `py_server.py`.

## Overview

The WebSocket server simulator provides complete Socket.IO event simulation with realistic network conditions, message validation, and recording capabilities. This allows testing of the teleoperation interface, frontend communication, and command flow without deploying the full system.

## Features

- **Socket.IO Event Handling**: Complete event registration and callback system
- **Network Condition Simulation**: Configurable latency, jitter, packet loss, and disconnections
- **Message Validation**: Automatic validation and metadata injection
- **Connection Management**: Multi-client support with state tracking
- **Message Recording**: Complete history tracking for analysis and replay
- **Statistics**: Comprehensive metrics on messages, connections, and errors

## Quick Start

```python
from simulation.network.websocket_server_simulator import (
    WebSocketServerSimulator,
    create_websocket_simulator
)

# Create simulator with default settings
sim = create_websocket_simulator('default')

# Or create with custom configuration
sim = WebSocketServerSimulator({
    'network': {
        'latency_ms': 50.0,
        'jitter_ms': 10.0,
        'packet_loss_rate': 0.01,
        'enabled': True
    },
    'validate_messages': True,
    'record_messages': True
})

# Register event handlers
async def handle_drive_command(data):
    print(f"Drive command: {data}")

sim.on('driveCommands', handle_drive_command)

# Simulate client connection
client_id = await sim.connect()

# Simulate receiving message from client
await sim.receive('driveCommands', {
    'linear': 0.5,
    'angular': 0.2
}, client_id)

# Emit message to client
await sim.emit('systemStatus', {
    'battery': 95.0,
    'mode': 'autonomous'
}, client_id)

# Get statistics
stats = sim.get_statistics()
print(f"Messages sent: {stats['stats']['messages_sent']}")
print(f"Messages received: {stats['stats']['messages_received']}")
```

## Configuration Options

### Network Conditions

Configure realistic network behavior:

```python
config = {
    'network': {
        'latency_ms': 50.0,        # Average latency in milliseconds
        'jitter_ms': 10.0,          # Latency variance (±jitter)
        'packet_loss_rate': 0.01,   # 1% packet loss
        'disconnect_rate': 0.001,   # 0.1% chance of disconnect per message
        'bandwidth_mbps': 100.0,    # Available bandwidth
        'enabled': True             # Enable/disable simulation
    }
}
```

### Message Validation

Enable automatic message validation:

```python
config = {
    'validate_messages': True,  # Validate all messages
    'record_messages': True,    # Record message history
    'max_history': 10000        # Maximum messages to keep
}
```

## Event Types

### Standard Events

The simulator supports all standard teleoperation events:

- `driveCommands`: Velocity commands from operator
- `emergencyStop`: Emergency stop trigger
- `homingRequest`: Request homing sequence
- `systemStatus`: System status updates
- `cameraFeed`: Camera stream data
- `sensorData`: Sensor readings

### Custom Events

Register handlers for any custom event:

```python
sim.on('customEvent', async_handler_function)
```

### Default Handler

Catch all unregistered events:

```python
async def default_handler(event_name, data):
    print(f"Unhandled event: {event_name}")

sim.set_default_handler(default_handler)
```

## Connection Management

### Multiple Clients

The simulator supports multiple concurrent clients:

```python
# Connect multiple clients
clients = []
for i in range(5):
    client_id = await sim.connect()
    clients.append(client_id)

# Broadcast to all clients
await sim.emit('broadcast', {'message': 'Hello all'})

# Send to specific client
await sim.emit('private', {'message': 'Hello'}, clients[0])

# Disconnect client
await sim.disconnect(clients[0])
```

### Connection States

Monitor connection state:

```python
from simulation.network.websocket_server_simulator import ConnectionState

print(sim.connection_state)  # DISCONNECTED, CONNECTING, CONNECTED, etc.
print(f"Active clients: {len(sim.connected_clients)}")
```

## Message Recording

### Accessing History

```python
# Get recent messages
history = sim.get_message_history(count=10)

# Filter by event type
drive_commands = [msg for msg in sim.get_message_history()
                  if msg['event_name'] == 'driveCommands']

# Get inbound only
inbound = sim.get_message_history(direction='inbound')
```

### Exporting History

```python
# Export to JSON file
sim.export_history('output/message_history.json')
```

### Clearing History

```python
sim.clear_history()
```

## Statistics

### Available Metrics

```python
stats = sim.get_statistics()

# Connection statistics
print(stats['stats']['connections'])
print(stats['stats']['disconnections'])
print(stats['connected_clients'])

# Message statistics
print(stats['stats']['messages_sent'])
print(stats['stats']['messages_received'])
print(stats['stats']['messages_dropped'])
print(stats['stats']['validation_errors'])

# Performance metrics
print(stats['stats']['avg_latency_ms'])
print(stats['stats']['uptime_s'])
```

### Rover State

Track simulated rover state:

```python
# Update rover state
sim.update_rover_state({
    'position': {'x': 10.0, 'y': 20.0},
    'battery': 85.0,
    'mode': 'autonomous'
})

# Get current state
state = sim.get_rover_state()
```

## Network Condition Profiles

### Predefined Profiles

```python
# Perfect network (no delays or errors)
sim = create_websocket_simulator('perfect')

# Default network (typical WiFi)
sim = create_websocket_simulator('default')

# Stressed network (high latency, packet loss)
sim = create_websocket_simulator('stressed')

# Mars simulation (extreme latency)
sim = create_websocket_simulator('mars')
```

### Dynamic Configuration

Change network conditions at runtime:

```python
# Simulate network degradation
sim.set_network_conditions({
    'latency_ms': 200.0,
    'packet_loss_rate': 0.10  # 10% loss
})

# Restore good conditions
sim.set_network_conditions({
    'latency_ms': 10.0,
    'packet_loss_rate': 0.0
})
```

## Testing Patterns

### Basic Communication Test

```python
async def test_basic_communication():
    sim = create_websocket_simulator('perfect')
    
    received = []
    sim.on('testEvent', lambda data: received.append(data))
    
    client_id = await sim.connect()
    await sim.receive('testEvent', {'value': 42}, client_id)
    
    await asyncio.sleep(0.01)
    assert len(received) == 1
    assert received[0]['value'] == 42
```

### Network Failure Test

```python
async def test_network_failure():
    sim = create_websocket_simulator('stressed')
    
    client_id = await sim.connect()
    
    # Send many messages
    for i in range(100):
        await sim.emit('test', {'id': i}, client_id)
    
    stats = sim.get_statistics()
    # Some messages should be dropped
    assert stats['stats']['messages_dropped'] > 0
```

### Latency Measurement

```python
async def test_latency():
    sim = create_websocket_simulator('default')
    sim.set_network_conditions({'latency_ms': 50.0, 'jitter_ms': 0.0})
    
    client_id = await sim.connect()
    
    start = time.time()
    await sim.emit('test', {}, client_id)
    duration_ms = (time.time() - start) * 1000
    
    # Should be close to 50ms
    assert 45 < duration_ms < 55
```

## Troubleshooting

### Messages Not Received

1. Check event handler is registered: `'eventName' in sim.event_handlers`
2. Verify client is connected: `client_id in sim.connected_clients`
3. Check message validation: `sim.validate_messages = False` to bypass
4. Review statistics: `sim.get_statistics()['stats']['validation_errors']`

### High Latency

1. Check network conditions: `sim.network_conditions.latency_ms`
2. Verify simulation enabled: `sim.network_conditions.enabled`
3. Use perfect profile for testing: `create_websocket_simulator('perfect')`

### Dropped Messages

1. Check packet loss rate: `sim.network_conditions.packet_loss_rate`
2. Review statistics: `sim.get_statistics()['stats']['messages_dropped']`
3. Adjust network conditions: `sim.set_network_conditions({'packet_loss_rate': 0.0})`

## Performance Considerations

- **Message History**: Limit with `max_history` to prevent memory growth
- **Network Simulation**: Disable for maximum speed: `network.enabled = False`
- **Validation**: Disable if message format is known good: `validate_messages = False`
- **Recording**: Disable for production: `record_messages = False`

## Integration with Tests

The simulator is designed for use in pytest tests:

```python
@pytest.fixture
def websocket_sim():
    sim = create_websocket_simulator('perfect')
    yield sim
    # Cleanup handled automatically

async def test_drive_command(websocket_sim):
    client_id = await websocket_sim.connect()
    await websocket_sim.receive('driveCommands', {
        'linear': 0.5,
        'angular': 0.2
    }, client_id)
    
    # Verify command processing
    # ...
```

## API Reference

### Main Class

- `WebSocketServerSimulator(config)`: Create simulator instance
- `on(event_name, handler)`: Register event handler
- `connect()`: Simulate client connection
- `disconnect(client_id)`: Simulate client disconnection
- `emit(event_name, data, client_id=None)`: Send message to client(s)
- `receive(event_name, data, client_id)`: Receive message from client
- `get_statistics()`: Get comprehensive statistics
- `get_message_history()`: Get message history
- `export_history(filename)`: Export history to JSON
- `set_network_conditions(conditions)`: Update network simulation

### Factory Function

- `create_websocket_simulator(profile)`: Create with predefined profile
  - `'perfect'`: No delays or errors
  - `'default'`: Typical WiFi conditions
  - `'stressed'`: High latency and packet loss
  - `'mars'`: Extreme latency simulation

## Examples

See `simulation/examples/websocket_bridge_demo.py` for complete usage examples.

## Related

- **SLCAN Protocol Simulator**: `simulation/can/slcan_protocol_simulator.py`
- **Firmware Simulator**: `simulation/firmware/stm32_firmware_simulator.py`
- **Full Stack Simulator**: `simulation/integration/full_stack_simulator.py`
