# URC 2026 Mars Rover - API Documentation

**Complete API Reference for Frontend, Backend, CAN, and ROS2 Integration**

**Version:** 2.0  
**Date:** 2026-01-20  
**Status:** ✅ Complete

---

## Table of Contents

1. [Overview](#overview)
2. [ROS2 API](#ros2-api)
3. [CAN Protocol API](#can-protocol-api)
4. [WebSocket/Socket.IO API](#websocketsocketio-api)
5. [Frontend JavaScript API](#frontend-javascript-api)
6. [Python Backend API](#python-backend-api)
7. [Quick Start Examples](#quick-start-examples)
8. [Testing & Validation](#testing--validation)

---

## Overview

This document provides a complete API reference for all communication interfaces in the URC 2026 Mars Rover system. The system uses a layered architecture:

```
┌─────────────────────────────────────────────────────────┐
│                     Frontend (React)                     │
│              gamepad, joystick, dashboard                │
└───────────────────┬─────────────────────────────────────┘
                    │ Socket.IO (JSON)
                    ↓
┌─────────────────────────────────────────────────────────┐
│          Teleoperation Server (py_server.py)            │
│                  Socket.IO ↔ SLCAN                       │
└───────────────────┬─────────────────────────────────────┘
                    │ SLCAN (Serial)          ↕ Socket.IO
                    ↓                         ↓
┌──────────────────────────────┐  ┌─────────────────────┐
│   STM32 Firmware             │  │  ROS2 Main System   │
│   (control-systems)          │  │  (Autonomy Stack)   │
└──────────────────────────────┘  └─────────────────────┘
         ↑                                   ↑
         └─────── SLCAN (Serial) ────────────┘
              via Protocol Adapter
```

### Communication Protocols

1. **Socket.IO** - Frontend ↔ Teleoperation Server (JSON)
2. **SLCAN** - Serial Line CAN protocol (ASCII hex)
3. **ROS2** - Robot Operating System 2 (DDS)

---

## ROS2 API

### Topics

#### Command Topics (Input to System)

##### `/cmd_vel/teleop` (geometry_msgs/Twist)
**Description:** Teleoperation velocity commands from frontend  
**Priority:** High (500)  
**Rate:** Up to 50 Hz  
**QoS:** Reliable, Volatile

**Message Structure:**
```python
geometry_msgs/Twist:
  linear:
    x: float  # Forward velocity (m/s), positive = forward
    y: float  # Lateral velocity (m/s), positive = left (swerve)
    z: float  # (unused, set to 0.0)
  angular:
    x: float  # (unused, set to 0.0)
    y: float  # (unused, set to 0.0)
    z: float  # Rotational velocity (rad/s), positive = CCW
```

**Example:**
```python
import rclpy
from geometry_msgs.msg import Twist

# Create node and publisher
node = rclpy.create_node('teleop_test')
pub = node.create_publisher(Twist, '/cmd_vel/teleop', 10)

# Publish forward motion
cmd = Twist()
cmd.linear.x = 0.5  # 0.5 m/s forward
cmd.angular.z = 0.0  # No rotation
pub.publish(cmd)
```

---

##### `/cmd_vel/autonomy` (geometry_msgs/Twist)
**Description:** Autonomous navigation commands  
**Priority:** Low (100)  
**Rate:** Up to 50 Hz  
**QoS:** Reliable, Volatile

---

##### `/cmd_vel/safety` (geometry_msgs/Twist)
**Description:** Safety system override commands  
**Priority:** Very High (900)  
**Rate:** As needed  
**QoS:** Reliable, Volatile

---

##### `/emergency_stop` (std_msgs/Bool)
**Description:** Emergency stop trigger  
**Priority:** Critical (1000)  
**Rate:** As needed  
**QoS:** Reliable, Transient Local

**Message Structure:**
```python
std_msgs/Bool:
  data: bool  # true = STOP, false = resume
```

**Example:**
```python
from std_msgs.msg import Bool

stop_msg = Bool()
stop_msg.data = True  # Trigger emergency stop
emergency_pub.publish(stop_msg)
```

---

##### `/hardware/homing_request` (std_msgs/Bool)
**Description:** Request homing sequence for swerve drive  
**Rate:** On demand  
**QoS:** Reliable

---

#### Feedback Topics (Output from System)

##### `/hardware/velocity_feedback` (geometry_msgs/TwistStamped)
**Description:** Actual velocity from encoders  
**Rate:** 10 Hz  
**QoS:** Best Effort, Volatile

**Message Structure:**
```python
geometry_msgs/TwistStamped:
  header:
    stamp: Time  # ROS2 timestamp
    frame_id: string  # "base_link"
  twist:
    linear:
      x: float  # Actual forward velocity (m/s)
      y: float  # Actual lateral velocity (m/s)
      z: float  # 0.0
    angular:
      x: float  # 0.0
      y: float  # 0.0
      z: float  # Actual rotational velocity (rad/s)
```

---

##### `/hardware/battery` (sensor_msgs/BatteryState)
**Description:** Battery status  
**Rate:** 1 Hz  
**QoS:** Best Effort

**Message Structure:**
```python
sensor_msgs/BatteryState:
  voltage: float  # Volts
  current: float  # Amperes (negative = discharging)
  percentage: float  # 0.0 to 100.0
  temperature: float  # Celsius
  power_supply_status: uint8  # DISCHARGING=1, CHARGING=2
  power_supply_health: uint8  # GOOD=1, OVERHEAT=2, DEAD=3
```

---

##### `/hardware/imu` (sensor_msgs/Imu)
**Description:** Inertial measurement unit data  
**Rate:** 50 Hz  
**QoS:** Best Effort

---

##### `/diagnostics` (diagnostic_msgs/DiagnosticArray)
**Description:** System diagnostics and health  
**Rate:** 1 Hz  
**QoS:** Best Effort

---

### Services

##### `/hardware/reset_encoders` (std_srvs/Trigger)
**Description:** Reset wheel encoders to zero

---

##### `/hardware/calibrate_imu` (std_srvs/Trigger)
**Description:** Calibrate IMU (requires stationary robot)

---

### Command Arbitration

The system uses **Twist Mux** for command priority:

1. **Emergency** (Priority 1000) - Overrides everything
2. **Safety** (Priority 900) - Safety system commands
3. **Teleop** (Priority 500) - Manual control
4. **Autonomy** (Priority 100) - Autonomous navigation

**Rules:**
- Higher priority always wins
- Commands timeout after 0.5s (safety feature)
- Zero velocity sent if no active commands

---

## CAN Protocol API

### SLCAN Frame Format

SLCAN (Serial Line CAN) uses ASCII hex encoding:

```
Format: t<ID><DLC><DATA>\r

Where:
  t      = Standard frame indicator
  <ID>   = 3 hex digits (CAN message ID)
  <DLC>  = 1 hex digit (data length in bytes, 0-8)
  <DATA> = 0-16 hex digits (data bytes)
  \r     = Carriage return terminator
```

**Example:**
```
t00C60800000003c0\r

Breakdown:
  t         = Standard frame
  00C       = Message ID 0x00C (SET_CHASSIS_VELOCITIES)
  6         = DLC 6 (6 bytes of data)
  0800      = X velocity (0x0800 = 2048 = 0.5 m/s * 4096)
  0000      = Y velocity (0x0000 = 0)
  03c0      = Rot velocity (0x03C0 = 960 = 15 deg/s * 64)
  \r        = Terminator
```

### Message IDs (Teleoperation Protocol)

#### Send Messages (Main System → Firmware)

| ID    | Name                      | DLC | Description                        |
|-------|---------------------------|-----|------------------------------------|
| 0x00C | SET_CHASSIS_VELOCITIES    | 6   | Drive velocity command             |
| 0x00E | HEARTBEAT                 | 0   | Keepalive ping                     |
| 0x110 | HOMING_SEQUENCE           | 8   | Start swerve homing                |
| 0x112 | GET_OFFSET                | 0   | Request encoder offsets            |
| 0x114 | GET_ESTIMATED_VELOCITIES  | 0   | Request velocity feedback          |
| 0x119 | CONFIG                    | 8   | Configuration update               |
| 0x301 | SET_MAST_GIMBAL          | 2   | Mast gimbal position               |

#### Receive Messages (Firmware → Main System)

| ID    | Name                              | DLC | Description                    |
|-------|-----------------------------------|-----|--------------------------------|
| 0x00D | SET_VELOCITIES_RESPONSE           | 6   | Velocity command acknowledgment|
| 0x00F | HEARTBEAT_REPLY                   | 0   | Heartbeat response             |
| 0x111 | HOMING_SEQUENCE_RESPONSE          | 0   | Homing complete                |
| 0x113 | RETURN_OFFSET                     | 9   | Encoder offset values          |
| 0x115 | RETURN_ESTIMATED_CHASSIS_VELOCITIES | 6 | Actual velocities             |
| 0x11A | CONFIG_ACK                        | 0   | Configuration acknowledged     |

#### 8-Motor Swerve Command IDs (Individual Motor Control)

| ID    | Name           | DLC | Description                          |
|-------|----------------|-----|--------------------------------------|
| 0x200 | DRIVE_MOTOR_0  | 2   | Front-left drive velocity (m/s)      |
| 0x201 | DRIVE_MOTOR_1  | 2   | Front-right drive velocity          |
| 0x202 | DRIVE_MOTOR_2  | 2   | Rear-left drive velocity            |
| 0x203 | DRIVE_MOTOR_3  | 2   | Rear-right drive velocity           |
| 0x210 | STEER_MOTOR_0  | 2   | Front-left steer angle (radians)    |
| 0x211 | STEER_MOTOR_1  | 2   | Front-right steer angle             |
| 0x212 | STEER_MOTOR_2  | 2   | Rear-left steer angle               |
| 0x213 | STEER_MOTOR_3  | 2   | Rear-right steer angle               |

**Scaling:** Velocity and angle use 16-bit signed, big-endian; `scaled = value * 4096` (2^12).

### Velocity Encoding

#### SET_CHASSIS_VELOCITIES (0x00C)

**Data Bytes (6 total):**
- Bytes 0-1: X velocity (16-bit signed, big-endian)
- Bytes 2-3: Y velocity (16-bit signed, big-endian)
- Bytes 4-5: Rotational velocity (16-bit signed, big-endian)

**Scaling:**
- Linear (X, Y): `scaled_value = velocity_m_s * 4096` (2^12)
- Angular (Rot): `scaled_value = velocity_deg_s * 64` (2^6)

**Conversion Functions:**

```python
# Encoding (ROS2 → SLCAN)
def encode_velocity(x_m_s, y_m_s, rot_rad_s):
    # Scale values
    x_scaled = int(x_m_s * 4096)
    y_scaled = int(y_m_s * 4096)
    rot_deg_s = rot_rad_s * 57.2957795131  # rad/s to deg/s
    rot_scaled = int(rot_deg_s * 64)
    
    # Convert to bytes (big-endian, signed)
    x_bytes = x_scaled.to_bytes(2, 'big', signed=True)
    y_bytes = y_scaled.to_bytes(2, 'big', signed=True)
    rot_bytes = rot_scaled.to_bytes(2, 'big', signed=True)
    
    # Create SLCAN frame
    data_hex = x_bytes.hex() + y_bytes.hex() + rot_bytes.hex()
    slcan_frame = f't00C6{data_hex}\r'
    
    return slcan_frame

# Decoding (SLCAN → ROS2)
def decode_velocity(slcan_frame):
    # Extract data bytes (skip 't00C6', take next 12 hex chars)
    data_hex = slcan_frame[5:17]
    
    # Parse velocity values
    x_scaled = int.from_bytes(bytes.fromhex(data_hex[0:4]), 'big', signed=True)
    y_scaled = int.from_bytes(bytes.fromhex(data_hex[4:8]), 'big', signed=True)
    rot_scaled = int.from_bytes(bytes.fromhex(data_hex[8:12]), 'big', signed=True)
    
    # Descale to SI units
    x_m_s = x_scaled / 4096.0
    y_m_s = y_scaled / 4096.0
    rot_deg_s = rot_scaled / 64.0
    rot_rad_s = rot_deg_s / 57.2957795131
    
    return x_m_s, y_m_s, rot_rad_s
```

**Examples:**

```python
# Forward 0.5 m/s, no rotation
>>> encode_velocity(0.5, 0.0, 0.0)
't00C60800000000000\r'

# Rotate 15 deg/s (0.2618 rad/s)
>>> encode_velocity(0.0, 0.0, 0.2618)
't00C6000000000003c0\r'

# Combined: 0.5 m/s forward, 0.25 m/s left, 15 deg/s CCW
>>> encode_velocity(0.5, 0.25, 0.2618)
't00C60800040003c0\r'
```

### Heartbeat

**Request (0x00E):**
```
t00E0\r
```

**Response (0x00F):**
```
t00F0\r
```

**Rate:** Send every 1-2 seconds to detect disconnection

### Homing Sequence

**Request (0x110):**
```
t11080000000000000000\r
```
(8 bytes of zeros)

**Response (0x111):**
```
t1110\r
```

**Duration:** 5-10 seconds (firmware dependent)

---

## WebSocket/Socket.IO API

### Connection

**Server URL:** `http://<raspberry-pi-ip>:5000`  
**Protocol:** Socket.IO v4  
**Transport:** WebSocket (with polling fallback)

### Client Events (Send to Server)

#### `driveCommands`

**Description:** Send drive commands from gamepad/joystick  
**Rate:** Up to 50 Hz  
**Format:** JSON

**Payload:**
```javascript
{
  xVel: number,    // Forward velocity (m/s), positive = forward
  yVel: number,    // Lateral velocity (m/s), positive = left
  rotVel: number   // Rotational velocity (deg/s), positive = CCW
}
```

**Example:**
```javascript
socket.emit('driveCommands', {
  xVel: 0.5,      // 0.5 m/s forward
  yVel: 0.0,      // No lateral movement
  rotVel: 15.0    // 15 deg/s CCW rotation
});
```

---

#### `driveHoming`

**Description:** Request swerve drive homing sequence  
**Rate:** On demand

**Payload:** Empty object `{}`

**Example:**
```javascript
socket.emit('driveHoming', {});
```

---

#### `emergencyStop`

**Description:** Trigger emergency stop  
**Rate:** As needed

**Payload:** Empty object `{}`

**Example:**
```javascript
socket.emit('emergencyStop', {});
```

---

#### `requestStatus`

**Description:** Request immediate status update  
**Rate:** As needed

**Payload:** Empty object `{}`

---

### Server Events (Receive from Server)

#### `systemStatus`

**Description:** System status update  
**Rate:** 10 Hz (automatic)  
**Format:** JSON

**Payload:**
```javascript
{
  timestamp: number,  // Unix timestamp (seconds)
  
  velocity: {
    x: number,        // Actual forward velocity (m/s)
    y: number,        // Actual lateral velocity (m/s)
    rot: number       // Actual rotational velocity (deg/s)
  } | null,
  
  battery: {
    voltage: number,      // Volts
    current: number,      // Amperes
    percentage: number,   // 0-100
    temperature: number   // Celsius
  } | null,
  
  imu: {
    orientation: {x, y, z, w},      // Quaternion
    angular_velocity: {x, y, z},    // rad/s
    linear_acceleration: {x, y, z}  // m/s²
  } | null,
  
  diagnostics: [
    {
      name: string,    // Component name
      level: number,   // 0=OK, 1=WARN, 2=ERROR
      message: string  // Status message
    }
  ],
  
  emergency_stop: boolean,  // true if emergency stop active
  
  bridge_stats: {
    commands_received: number,
    commands_published: number,
    status_sent: number,
    errors: number
  }
}
```

**Example Handler:**
```javascript
socket.on('systemStatus', (data) => {
  console.log('Battery:', data.battery?.percentage + '%');
  console.log('Velocity:', data.velocity?.x + ' m/s');
  
  if (data.emergency_stop) {
    alert('EMERGENCY STOP ACTIVE!');
  }
});
```

---

#### `connect`

**Description:** Fired when connected to server

```javascript
socket.on('connect', () => {
  console.log('Connected to teleoperation server');
});
```

---

#### `disconnect`

**Description:** Fired when disconnected from server

```javascript
socket.on('disconnect', () => {
  console.log('Disconnected from server');
});
```

---

## Frontend JavaScript API

### Socket.IO Client Setup

```javascript
import io from 'socket.io-client';

// Connect to teleoperation server
const socket = io('http://192.168.1.100:5000', {
  reconnection: true,
  reconnectionDelay: 2000,
  transports: ['websocket', 'polling']
});

// Connection events
socket.on('connect', () => {
  console.log('✓ Connected');
});

socket.on('disconnect', () => {
  console.log('✗ Disconnected');
});

socket.on('connect_error', (error) => {
  console.error('Connection error:', error);
});
```

### Gamepad Control Integration

```javascript
class GamepadController {
  constructor(socket) {
    this.socket = socket;
    this.updateRate = 20; // 20 Hz
    this.maxLinearVel = 1.0; // m/s
    this.maxAngularVel = 45.0; // deg/s
    
    this.startLoop();
  }
  
  startLoop() {
    setInterval(() => {
      const gamepad = navigator.getGamepads()[0];
      if (!gamepad) return;
      
      // Left stick: forward/backward, left/right
      const xVel = gamepad.axes[1] * -this.maxLinearVel;  // Inverted
      const yVel = gamepad.axes[0] * this.maxLinearVel;
      
      // Right stick X: rotation
      const rotVel = gamepad.axes[2] * this.maxAngularVel;
      
      // Send to server
      this.socket.emit('driveCommands', {
        xVel: xVel,
        yVel: yVel,
        rotVel: rotVel
      });
      
      // Emergency stop on button 0 (A/X)
      if (gamepad.buttons[0].pressed) {
        this.socket.emit('emergencyStop', {});
      }
    }, 1000 / this.updateRate);
  }
}

// Usage
const controller = new GamepadController(socket);
```

### Status Display

```javascript
// Update UI with system status
socket.on('systemStatus', (status) => {
  // Battery
  document.getElementById('battery-voltage').textContent = 
    status.battery?.voltage.toFixed(1) + ' V';
  document.getElementById('battery-percentage').textContent = 
    status.battery?.percentage.toFixed(0) + '%';
  
  // Velocity
  document.getElementById('velocity-x').textContent = 
    status.velocity?.x.toFixed(2) + ' m/s';
  document.getElementById('velocity-rot').textContent = 
    status.velocity?.rot.toFixed(1) + ' deg/s';
  
  // Emergency stop indicator
  const estopIndicator = document.getElementById('estop-indicator');
  estopIndicator.classList.toggle('active', status.emergency_stop);
});
```

---

## Python Backend API

### Using CAN Bridge

```python
import asyncio
from bridges.can_bridge import CANBridge
from bridges.unified_bridge_interface import BridgeMessage
from geometry_msgs.msg import Twist

async def main():
    # Configure CAN bridge
    config = {
        'protocol': 'teleoperation',
        'device': '/dev/ttyACM0',
        'fallback_devices': ['/dev/ttyAMA10', '/dev/ttyUSB0'],
        'baudrate': 115200
    }
    
    # Create bridge
    bridge = CANBridge(config)
    
    # Connect
    connected = await bridge.connect()
    if not connected:
        print("Failed to connect")
        return
    
    # Send velocity command
    twist = Twist()
    twist.linear.x = 0.5
    twist.angular.z = 0.2618
    
    msg = BridgeMessage(
        message_type='velocity_command',
        data={'twist': twist}
    )
    
    await bridge.send_message(msg)
    
    # Get status
    status = bridge.get_status()
    print(f"Bridge connected: {status.is_connected}")
    print(f"Protocol: {status.additional_info['protocol']}")
    
    await bridge.disconnect()

asyncio.run(main())
```

### Using Protocol Adapter Directly

```python
from bridges.teleop_can_adapter import TeleopCANAdapter
from geometry_msgs.msg import Twist

# Create adapter
adapter = TeleopCANAdapter()

# Encode velocity
twist = Twist()
twist.linear.x = 0.5
twist.angular.z = 0.2618

protocol_msg = adapter.encode_velocity_command(twist)
print(f"SLCAN frame: {protocol_msg.data.decode()}")
# Output: t00C60800000003c0\r

# Decode velocity
slcan_response = b't00D60800000003c0\r'
decoded_twist = adapter.decode_velocity_feedback(slcan_response)
print(f"X: {decoded_twist.linear.x:.3f} m/s")
print(f"Rot: {decoded_twist.angular.z:.3f} rad/s")

# Get stats
stats = adapter.get_stats()
print(f"Encoded: {stats['messages_encoded']}")
print(f"Errors: {stats['encoding_errors']}")
```

---

## Quick Start Examples

### Example 1: Send Drive Command (ROS2)

```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import Twist

rclpy.init()
node = rclpy.create_node('drive_test')
pub = node.create_publisher(Twist, '/cmd_vel/teleop', 10)

# Drive forward
cmd = Twist()
cmd.linear.x = 0.5  # 0.5 m/s
cmd.angular.z = 0.0
pub.publish(cmd)

# Spin in place
cmd = Twist()
cmd.linear.x = 0.0
cmd.angular.z = 0.5236  # 30 deg/s
pub.publish(cmd)

node.destroy_node()
rclpy.shutdown()
```

### Example 2: Monitor Feedback (ROS2)

```python
#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import TwistStamped

def callback(msg):
    print(f"Velocity: {msg.twist.linear.x:.2f} m/s")

rclpy.init()
node = rclpy.create_node('velocity_monitor')
sub = node.create_subscription(
    TwistStamped,
    '/hardware/velocity_feedback',
    callback,
    10
)

rclpy.spin(node)
```

### Example 3: Frontend Gamepad (JavaScript)

```html
<!DOCTYPE html>
<html>
<head>
  <script src="https://cdn.socket.io/4.5.4/socket.io.min.js"></script>
</head>
<body>
  <h1>Rover Teleop</h1>
  <div id="status"></div>
  
  <script>
    const socket = io('http://192.168.1.100:5000');
    
    socket.on('connect', () => {
      document.getElementById('status').textContent = 'Connected';
    });
    
    // Gamepad loop
    setInterval(() => {
      const gamepad = navigator.getGamepads()[0];
      if (!gamepad) return;
      
      socket.emit('driveCommands', {
        xVel: gamepad.axes[1] * -1.0,  // Forward/back
        yVel: gamepad.axes[0] * 1.0,   // Left/right
        rotVel: gamepad.axes[2] * 45.0 // Rotation
      });
    }, 50); // 20 Hz
  </script>
</body>
</html>
```

### Example 4: Emergency Stop (All Methods)

**ROS2:**
```python
from std_msgs.msg import Bool
stop_msg = Bool()
stop_msg.data = True
emergency_pub.publish(stop_msg)
```

**Socket.IO:**
```javascript
socket.emit('emergencyStop', {});
```

**CAN:**
```python
msg = BridgeMessage(message_type='emergency_stop', data={})
await bridge.send_message(msg)
```

---

## Testing & Validation

### Unit Tests

```bash
# Test protocol adapter
cd /home/durian/urc-machiato-2026
python3 -c "
import sys
sys.path.insert(0, 'src')
from geometry_msgs.msg import Twist
from bridges.teleop_can_adapter import TeleopCANAdapter

adapter = TeleopCANAdapter()
twist = Twist()
twist.linear.x = 0.5

msg = adapter.encode_velocity_command(twist)
print(f'✓ Encoded: {msg.data.decode().strip()}')
"
```

### Integration Tests

```bash
# Run integration tests with stubs
python3 tests/integration/test_bridge_integration_stubs.py
```

### Hardware Tests

```bash
# Test with actual teleoperation server
# 1. Start teleoperation server:
cd vendor/teleoperation/server
./run.sh

# 2. In another terminal, run ROS2 node:
ros2 run hardware_interface hardware_interface_node \
  --ros-args -p can_port:=/dev/ttyACM0 \
             -p can_protocol:=teleoperation

# 3. Send test command:
ros2 topic pub /cmd_vel/teleop geometry_msgs/Twist \
  "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

---

## Troubleshooting

### Common Issues

**Issue:** "No module named 'socketio'"  
**Solution:** `pip3 install python-socketio`

**Issue:** "Failed to connect to /dev/ttyACM0"  
**Solution:** Check device permissions: `sudo chmod 666 /dev/ttyACM0`  
Or add user to dialout group: `sudo usermod -a -G dialout $USER`

**Issue:** "Command timeout, zero velocity sent"  
**Solution:** Commands expire after 0.5s. Send commands at >2 Hz rate.

**Issue:** "Emergency stop active, commands ignored"  
**Solution:** Clear emergency stop: `ros2 topic pub /emergency_stop std_msgs/Bool "data: false"`

---

## References

- **Bridge Architecture:** `docs/BRIDGE_INTEGRATION_ARCHITECTURE.md`
- **Protocol Specification:** `docs/SUBMODULE_INTERFACE_SPECIFICATION.md`
- **Implementation Progress:** `IMPLEMENTATION_PROGRESS.md`
- **Test Suite:** `tests/integration/test_bridge_integration_stubs.py`

---

**For questions or issues:**  
Open an issue on GitHub or contact the integration team.

**Last Updated:** 2026-01-20  
**Version:** 2.0
