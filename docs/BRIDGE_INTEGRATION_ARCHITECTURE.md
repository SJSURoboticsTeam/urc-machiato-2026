# Bridge Integration Architecture - CAN & WebSocket

**Main Codebase Adaptation Strategy**  
**Status:** Ready for Implementation  
**Priority:** Make main codebase malleable to prevent submodule conflicts

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [ROS2 Topic Architecture](#ros2-topic-architecture)
3. [CAN Bridge Integration](#can-bridge-integration)
4. [WebSocket Bridge Integration](#websocket-bridge-integration)
5. [Protocol Adaptation Layer](#protocol-adaptation-layer)
6. [Testing Strategy](#testing-strategy)
7. [Implementation Checklist](#implementation-checklist)

---

## Architecture Overview

### Design Principle: Main Codebase is Malleable

**Problem:** Submodules (teleoperation, control-systems) have fixed protocols  
**Solution:** Main codebase adapts to match submodule protocols  
**Benefit:** No conflicts, no breaking changes to working systems

### System Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                   ROS2 MAIN CODEBASE (Malleable)            │
│                                                              │
│  ┌────────────────────────────────────────────────────┐   │
│  │         ROS2 Topic Layer (Standard)                │   │
│  │  /cmd_vel/autonomy, /cmd_vel/teleop, etc.         │   │
│  └────────────────┬───────────────────────────────────┘   │
│                   │                                          │
│  ┌────────────────┴───────────────────────────────────┐   │
│  │       Protocol Adaptation Layer (NEW)              │   │
│  │  - Translate ROS2 ↔ Teleoperation Protocol        │   │
│  │  - Translate ROS2 ↔ WebSocket JSON                │   │
│  │  - Handle velocity scaling (2^12, 2^6)            │   │
│  │  - Message ID mapping (0x100 ↔ 0x00C)             │   │
│  └────────────┬───────────────┬────────────────────────┘   │
│               │               │                             │
│  ┌────────────┴──────┐   ┌───┴────────────────────┐       │
│  │   CAN Bridge      │   │  WebSocket Bridge      │       │
│  │  (SLCAN Format)   │   │  (Socket.IO/JSON)      │       │
│  └────────────┬──────┘   └───┬────────────────────┘       │
└───────────────┼──────────────┼─────────────────────────────┘
                │              │
      ┌─────────┴──────┐       │
      │ /dev/ttyACM0   │       │ ws://0.0.0.0:4000
      │ /dev/ttyAMA10  │       │
      └─────────┬──────┘       │
                │              │
    ┌───────────┴──────────┐   │
    │  Teleoperation Server │   │
    │   (py_server.py)      │   │
    │  - Receives commands  │   │
    │  - Sends to firmware  │   │
    └───────────┬───────────┘   │
                │                │
    ┌───────────┴───────────┐   │
    │  Control-Systems      │   │
    │  STM32 Firmware       │   │
    │  - Drive control      │   │
    │  - Sensor feedback    │   │
    └───────────────────────┘   │
                                 │
                    ┌────────────┴──────────────┐
                    │  Frontend Dashboard       │
                    │  (React/Socket.IO)        │
                    │  - Manual control         │
                    │  - System monitoring      │
                    └───────────────────────────┘
```

---

## ROS2 Topic Architecture

### Command Topics (Subscribed by Main Codebase)

| Topic | Message Type | Rate | Priority | Purpose | Bridge |
|-------|--------------|------|----------|---------|--------|
| `/cmd_vel/emergency` | `geometry_msgs/Twist` | On demand | 1000 | Emergency stop commands | Both |
| `/cmd_vel/safety` | `geometry_msgs/Twist` | 50 Hz | 900 | Safety system overrides | Both |
| `/cmd_vel/teleop` | `geometry_msgs/Twist` | 20 Hz | 500 | Manual teleoperation | WebSocket → CAN |
| `/cmd_vel/autonomy` | `geometry_msgs/Twist` | 10 Hz | 100 | Autonomous navigation | CAN |
| `/cmd_vel` | `geometry_msgs/Twist` | 10 Hz | 100 | Legacy/global commands | CAN |
| `/emergency_stop` | `std_msgs/Bool` | On demand | 1000 | Global emergency stop | Both |
| `/hardware/arm_command` | `std_msgs/String` | On demand | 500 | Arm control commands | CAN |
| `/hardware/led_command` | `autonomy_interfaces/LedCommand` | 5 Hz | 200 | LED status control | CAN |

### Sensor/Feedback Topics (Published by Main Codebase)

| Topic | Message Type | Rate | Purpose | Source |
|-------|--------------|------|---------|--------|
| `/hardware/joint_states` | `sensor_msgs/JointState` | 50 Hz | Motor positions/velocities | CAN |
| `/hardware/chassis_velocity` | `geometry_msgs/TwistStamped` | 50 Hz | Actual chassis velocity | CAN |
| `/hardware/motor_temperatures` | `std_msgs/Float32MultiArray` | 10 Hz | Motor temperatures | CAN |
| `/hardware/battery_state` | `sensor_msgs/BatteryState` | 10 Hz | Battery voltage/current/SOC | CAN |
| `/hardware/imu` | `sensor_msgs/Imu` | 100 Hz | IMU acceleration/gyroscope | CAN |
| `/hardware/gps` | `sensor_msgs/NavSatFix` | 10 Hz | GPS position | CAN |
| `/hardware/system_status` | `std_msgs/String` | 1 Hz | System health (JSON) | Both |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 1 Hz | Diagnostics for all bridges | Both |

### Teleoperation Feedback Topics (Published for WebSocket)

| Topic | Message Type | Rate | Purpose |
|-------|--------------|------|---------|
| `/teleop/drive_status` | `std_msgs/String` | 10 Hz | Drive system status (JSON) |
| `/teleop/arm_status` | `std_msgs/String` | 10 Hz | Arm status (JSON) |
| `/teleop/metrics` | `std_msgs/String` | 1 Hz | System metrics (CPU, SSH, etc.) |

### Message Flow Diagram

```
ROS2 Topics           Protocol Adapter         Bridges              Hardware/Clients
                                                                     
/cmd_vel/teleop  -->  |                    |  -->  WebSocket   -->  Frontend Dashboard
                      | Twist → JSON       |       Socket.IO        (Gamepad control)
                      |                    |             |
                      |                    |             v
                      |                    |       Teleoperation
                      |                    |         Server
                      |                    |       (py_server.py)
                      |                    |             |
                      |                    |             v
/cmd_vel/autonomy --> | Twist → SLCAN     |  -->    CAN Bridge  -->  STM32 Firmware
                      | Velocity scaling   |     /dev/ttyACM0        (Drive control)
                      | (×4096, ×64)      |     /dev/ttyAMA10
                      |                    |
                      |                    |
STM32 Feedback   <--  | SLCAN → ROS2      |  <--    CAN Bridge  <--  STM32 Firmware
(IMU, encoders)       | Parse 0x00D-0x115 |                          (Sensors)
                      |                    |
/hardware/joint_states
/hardware/imu
/hardware/battery_state
```

---

## CAN Bridge Integration

### Current State Analysis

**File:** `src/bridges/can_bridge.py`  
**Protocol:** SocketCAN (native Linux CAN)  
**Message IDs:** 0x100-0xFFF (main codebase convention)

**Problem:** Teleoperation uses 0x00C-0x300 (different convention)

### Solution: Protocol Adaptation Layer

Create adapter that translates between ROS2 and teleoperation protocol:

```python
# src/bridges/teleop_can_adapter.py

class TeleopCANAdapter:
    """
    Adapts main codebase CAN messages to teleoperation protocol.
    
    Makes main codebase malleable - adapts to submodule protocol.
    """
    
    # Teleoperation protocol (from vendor/teleoperation)
    TELEOP_SEND_IDS = {
        "SET_CHASSIS_VELOCITIES": 0x00C,
        "HEARTBEAT": 0x00E,
        "HOMING_SEQUENCE": 0x110,
        "GET_OFFSET": 0x112,
        "GET_ESTIMATED_VELOCITIES": 0x114,
        "CONFIG": 0x119,
        "SET_MAST_GIMBAL": 0x301,  # Changed from 0x300 to avoid conflict
    }
    
    TELEOP_RECEIVE_IDS = {
        "SET_VELOCITIES_RESPONSE": 0x00D,
        "HEARTBEAT_REPLY": 0x00F,
        "HOMING_SEQUENCE_RESPONSE": 0x111,
        "RETURN_OFFSET": 0x113,
        "RETURN_ESTIMATED_CHASSIS_VELOCITIES": 0x115,
        "CONFIG_ACK": 0x11A,
    }
    
    # Main codebase protocol (keep for internal use)
    MAIN_IDS = {
        "MOTOR_COMMAND": 0x100,
        "MOTOR_FEEDBACK": 0x101,
        "IMU_DATA": 0x200,
        "ENCODER_DATA": 0x201,
        "BATTERY_STATUS": 0x300,  # No longer conflicts
        "SYSTEM_STATUS": 0x400,
        "EMERGENCY_STOP": 0xFFF,
    }
    
    def twist_to_slcan(self, twist: Twist) -> str:
        """
        Convert ROS2 Twist to SLCAN frame (teleoperation format).
        
        Args:
            twist: ROS2 Twist message (linear.x, linear.y, angular.z)
        
        Returns:
            str: SLCAN frame 't00C6<x><y><rot>\\r'
        """
        # Extract velocities
        x_vel = twist.linear.x  # m/s forward
        y_vel = twist.linear.y  # m/s lateral (swerve drive)
        rot_vel = twist.angular.z * 57.2958  # rad/s → deg/s
        
        # Teleoperation scaling (from py_server.py)
        x_vel_scaled = int(x_vel * (2 ** 12))      # 16-bit signed, ×4096
        y_vel_scaled = int(y_vel * (2 ** 12))      # 16-bit signed, ×4096
        rot_vel_scaled = int(rot_vel * (2 ** 6))   # 16-bit signed, ×64
        
        # Clamp to 16-bit signed range
        x_vel_scaled = max(-32768, min(32767, x_vel_scaled))
        y_vel_scaled = max(-32768, min(32767, y_vel_scaled))
        rot_vel_scaled = max(-32768, min(32767, rot_vel_scaled))
        
        # Convert to bytes (big-endian, signed)
        x_bytes = x_vel_scaled.to_bytes(2, 'big', signed=True)
        y_bytes = y_vel_scaled.to_bytes(2, 'big', signed=True)
        rot_bytes = rot_vel_scaled.to_bytes(2, 'big', signed=True)
        
        # Create SLCAN frame
        msg_id = self.TELEOP_SEND_IDS["SET_CHASSIS_VELOCITIES"]
        slcan_frame = f't{msg_id:03X}6{x_bytes.hex()}{y_bytes.hex()}{rot_bytes.hex()}\\r'
        
        return slcan_frame
    
    def slcan_to_twist(self, slcan_frame: str) -> Optional[Twist]:
        """
        Parse SLCAN velocity response to ROS2 Twist.
        
        Args:
            slcan_frame: SLCAN frame 't00D6<x><y><rot>\\r'
        
        Returns:
            Twist: ROS2 Twist message or None if parse fails
        """
        try:
            # Parse SLCAN frame
            if not slcan_frame.startswith('t') or len(slcan_frame) < 20:
                return None
            
            msg_id = int(slcan_frame[1:4], 16)
            if msg_id != self.TELEOP_RECEIVE_IDS["SET_VELOCITIES_RESPONSE"]:
                return None
            
            # Extract hex data
            data_hex = slcan_frame[5:17]  # 6 bytes = 12 hex chars
            x_hex = data_hex[0:4]
            y_hex = data_hex[4:8]
            rot_hex = data_hex[8:12]
            
            # Convert hex to signed integers
            x_scaled = int.from_bytes(bytes.fromhex(x_hex), 'big', signed=True)
            y_scaled = int.from_bytes(bytes.fromhex(y_hex), 'big', signed=True)
            rot_scaled = int.from_bytes(bytes.fromhex(rot_hex), 'big', signed=True)
            
            # Reverse teleoperation scaling
            x_vel = x_scaled / (2 ** 12)      # → m/s
            y_vel = y_scaled / (2 ** 12)      # → m/s
            rot_vel = rot_scaled / (2 ** 6)   # → deg/s
            rot_vel_rad = rot_vel / 57.2958   # → rad/s
            
            # Create Twist message
            twist = Twist()
            twist.linear.x = x_vel
            twist.linear.y = y_vel
            twist.angular.z = rot_vel_rad
            
            return twist
            
        except Exception as e:
            logger.error(f"Failed to parse SLCAN frame: {e}")
            return None
```

### CAN Bridge Message Flow

#### Sending Commands (ROS2 → CAN → Teleoperation → Firmware)

```
1. ROS2 publishes:  /cmd_vel/teleop (Twist)
                    linear.x = 0.5 m/s, angular.z = 0.26 rad/s (15 deg/s)

2. Adapter converts:
   - x_vel: 0.5 * 4096 = 2048 = 0x0800
   - y_vel: 0.0 * 4096 = 0 = 0x0000
   - rot_vel: 15.0 * 64 = 960 = 0x03C0
   
3. SLCAN frame generated:
   't00C60800000003c0\r'

4. CAN Bridge sends via serial to /dev/ttyAMA10

5. Teleoperation server (py_server.py) receives and forwards to firmware

6. STM32 firmware executes drive command
```

#### Receiving Feedback (Firmware → Teleoperation → CAN → ROS2)

```
1. STM32 firmware sends: Velocity response (0x00D)
   't00D608000000003c0\r' (echoing commanded values)

2. Teleoperation server forwards to CAN Bridge

3. Adapter parses SLCAN frame:
   - Extracts: x=2048, y=0, rot=960
   - Converts: x=0.5 m/s, y=0.0 m/s, rot=15.0 deg/s

4. ROS2 publishes: /hardware/chassis_velocity (TwistStamped)
```

### Device Configuration

**Flexible device path configuration** (no hardcoding):

```yaml
# config/bridge_config.yaml
can_bridge:
  enabled: true
  protocol: "teleop"  # Use teleoperation protocol
  device_path: "/dev/ttyAMA10"  # Teleoperation drive controller
  fallback_devices:
    - "/dev/ttyACM0"  # Main hardware interface
    - "/dev/ttyUSB0"  # USB adapter fallback
  baudrate: 115200
  auto_discover: true  # Try all devices until connection succeeds
```

---

## WebSocket Bridge Integration

### Current State Analysis

**File:** `src/autonomy/perception/sensor_bridge/autonomy_sensor_bridge/websocket_bridge.py`  
**Protocol:** Socket.IO / WebSocket with JSON  
**Current Use:** Sensor data (IMU, GPS, battery) from external devices

**Goal:** Bidirectional communication with teleoperation frontend

### Solution: Extend WebSocket Bridge

```python
# src/bridges/teleop_websocket_bridge.py

class TeleopWebSocketBridge:
    """
    WebSocket bridge for teleoperation frontend communication.
    
    Bidirectional:
    - ROS2 → WebSocket: Send system status, sensor data
    - WebSocket → ROS2: Receive drive commands, UI events
    """
    
    def __init__(self, ros_node):
        self.node = ros_node
        self.sio = socketio.AsyncClient()
        self.connected = False
        
        # WebSocket endpoint (teleoperation server)
        self.server_url = "http://localhost:4000"
        
        # Setup event handlers
        self._setup_event_handlers()
    
    def _setup_event_handlers(self):
        """Setup Socket.IO event handlers."""
        
        @self.sio.event
        async def connect():
            """Handle connection to teleoperation server."""
            self.connected = True
            self.node.get_logger().info("Connected to teleoperation server")
        
        @self.sio.event
        async def disconnect():
            """Handle disconnection."""
            self.connected = False
            self.node.get_logger().warn("Disconnected from teleoperation server")
        
        @self.sio.event
        async def driveCommands(data):
            """
            Receive drive commands from frontend.
            
            Args:
                data: {xVel: float, yVel: float, rotVel: float}
            """
            # Convert to ROS2 Twist
            twist = Twist()
            twist.linear.x = data.get('xVel', 0.0)
            twist.linear.y = data.get('yVel', 0.0)
            twist.angular.z = data.get('rotVel', 0.0) * (3.14159 / 180.0)  # deg→rad
            
            # Publish to ROS2
            self.node.teleop_cmd_pub.publish(twist)
            self.node.get_logger().debug(
                f"WebSocket drive command: x={twist.linear.x:.2f}, "
                f"y={twist.linear.y:.2f}, rot={twist.angular.z:.2f}"
            )
        
        @self.sio.event
        async def driveHoming(data):
            """Handle drive homing request."""
            # Publish homing command
            msg = String()
            msg.data = json.dumps({"command": "homing", "subsystem": "drive"})
            self.node.homing_cmd_pub.publish(msg)
            self.node.get_logger().info("Drive homing initiated via WebSocket")
    
    async def send_status(self, status_data: Dict):
        """
        Send system status to frontend.
        
        Args:
            status_data: Dictionary with system state
        """
        if not self.connected:
            return
        
        try:
            await self.sio.emit('systemStatus', status_data)
        except Exception as e:
            self.node.get_logger().error(f"Failed to send status: {e}")
    
    async def send_metrics(self, metrics_data: Dict):
        """
        Send metrics to frontend.
        
        Args:
            metrics_data: Dictionary with performance metrics
        """
        if not self.connected:
            return
        
        try:
            await self.sio.emit('metrics', metrics_data)
        except Exception as e:
            self.node.get_logger().error(f"Failed to send metrics: {e}")
```

### WebSocket Message Flow

#### Receiving Commands (Frontend → WebSocket → ROS2 → CAN)

```
1. Frontend sends: driveCommands via Socket.IO
   {xVel: 0.5, yVel: 0.0, rotVel: 15.0}

2. WebSocket Bridge receives and converts to ROS2:
   Twist(linear.x=0.5, linear.y=0.0, angular.z=0.26)

3. ROS2 publishes: /cmd_vel/teleop

4. Hardware Interface Node (Twist Mux) prioritizes command

5. Protocol Adapter converts to SLCAN: 't00C60800000003c0\r'

6. CAN Bridge sends to teleoperation server

7. Teleoperation server forwards to firmware
```

#### Sending Feedback (ROS2 → WebSocket → Frontend)

```
1. ROS2 receives: /hardware/battery_state (BatteryState)
   voltage=24.3V, current=5.2A, soc=78%

2. WebSocket Bridge converts to JSON:
   {
     "battery": {
       "voltage": 24.3,
       "current": 5.2,
       "soc": 78,
       "temperature": 25.4
     },
     "timestamp": 1768974934
   }

3. WebSocket Bridge emits: systemStatus event

4. Frontend displays in dashboard
```

### WebSocket Events Catalog

**Client → Server (Frontend → Main Codebase):**

| Event | Data | Purpose |
|-------|------|---------|
| `driveCommands` | `{xVel, yVel, rotVel}` | Drive control commands |
| `driveHoming` | None | Initiate homing sequence |
| `emergencyStop` | `{active: bool}` | Emergency stop toggle |
| `armCommand` | `{joint, angle, speed}` | Arm control |
| `requestMetrics` | None | Request system metrics |

**Server → Client (Main Codebase → Frontend):**

| Event | Data | Purpose |
|-------|------|---------|
| `systemStatus` | `{battery, motors, sensors}` | System state |
| `metrics` | `{cpu, memory, network}` | Performance metrics |
| `driveStatus` | `{velocity, position, state}` | Drive system state |
| `armStatus` | `{joints, position}` | Arm state |
| `diagnostics` | `{errors, warnings}` | Diagnostic messages |

---

## Protocol Adaptation Layer

### Design Principle

**Main codebase adapts to submodule protocols** (not vice versa):

1. Teleoperation protocol (0x00C-0x300) is fixed
2. Control-systems firmware expects teleoperation protocol
3. Main codebase implements adaptation layer to match

### Adapter Architecture

```python
# src/bridges/protocol_adapter.py

from abc import ABC, abstractmethod
from typing import Optional, Dict, Any
from geometry_msgs.msg import Twist

class ProtocolAdapter(ABC):
    """Base class for protocol adapters."""
    
    @abstractmethod
    def encode_velocity_command(self, twist: Twist) -> bytes:
        """Encode ROS2 Twist to protocol-specific format."""
        pass
    
    @abstractmethod
    def decode_velocity_feedback(self, data: bytes) -> Optional[Twist]:
        """Decode protocol-specific format to ROS2 Twist."""
        pass
    
    @abstractmethod
    def encode_sensor_request(self, sensor_type: str) -> bytes:
        """Encode sensor data request."""
        pass
    
    @abstractmethod
    def decode_sensor_data(self, data: bytes, sensor_type: str) -> Optional[Dict]:
        """Decode sensor data."""
        pass


class TeleopProtocolAdapter(ProtocolAdapter):
    """
    Adapter for teleoperation/control-systems protocol.
    
    Implements SLCAN format with teleoperation message IDs and scaling.
    """
    
    # Message IDs from teleoperation protocol
    MSG_IDS = {
        "SEND": {
            "SET_CHASSIS_VELOCITIES": 0x00C,
            "HEARTBEAT": 0x00E,
            "HOMING_SEQUENCE": 0x110,
            "GET_OFFSET": 0x112,
            "GET_ESTIMATED_VELOCITIES": 0x114,
            "CONFIG": 0x119,
            "SET_MAST_GIMBAL": 0x301,
        },
        "RECEIVE": {
            "SET_VELOCITIES_RESPONSE": 0x00D,
            "HEARTBEAT_REPLY": 0x00F,
            "HOMING_SEQUENCE_RESPONSE": 0x111,
            "RETURN_OFFSET": 0x113,
            "RETURN_ESTIMATED_CHASSIS_VELOCITIES": 0x115,
            "CONFIG_ACK": 0x11A,
        }
    }
    
    def encode_velocity_command(self, twist: Twist) -> bytes:
        """Encode Twist to SLCAN format with teleoperation scaling."""
        # (Implementation from twist_to_slcan above)
        pass
    
    def decode_velocity_feedback(self, data: bytes) -> Optional[Twist]:
        """Decode SLCAN velocity response."""
        # (Implementation from slcan_to_twist above)
        pass
    
    def encode_sensor_request(self, sensor_type: str) -> bytes:
        """Encode sensor request (GET_ESTIMATED_VELOCITIES, etc.)."""
        if sensor_type == "velocity":
            msg_id = self.MSG_IDS["SEND"]["GET_ESTIMATED_VELOCITIES"]
            return f't{msg_id:03X}00\\r'.encode()
        # ... more sensor types
    
    def decode_sensor_data(self, data: bytes, sensor_type: str) -> Optional[Dict]:
        """Decode sensor feedback from teleoperation protocol."""
        # Parse SLCAN frames for sensor data
        pass


class MainProtocolAdapter(ProtocolAdapter):
    """
    Adapter for main codebase's native protocol.
    
    Implements SocketCAN format with main codebase message IDs.
    """
    
    MSG_IDS = {
        "MOTOR_COMMAND": 0x100,
        "MOTOR_FEEDBACK": 0x101,
        "IMU_DATA": 0x200,
        "ENCODER_DATA": 0x201,
        "BATTERY_STATUS": 0x300,
        "SYSTEM_STATUS": 0x400,
        "EMERGENCY_STOP": 0xFFF,
    }
    
    # Implementation for main codebase protocol
    # (Keep for internal testing or future hardware that uses this protocol)
```

### Adapter Selection

```python
# config/bridge_config.yaml
can_bridge:
  protocol_adapter: "teleop"  # Use TeleopProtocolAdapter
  # Options: "teleop", "main", "custom"
  
  # Adapter-specific config
  teleop_adapter:
    velocity_scaling:
      linear: 4096  # 2^12
      angular: 64   # 2^6
    device_paths:
      drive: "/dev/ttyAMA10"
      arm: "/dev/ttyACM1"
  
  main_adapter:
    velocity_scaling:
      linear: 1000  # Different scaling
      angular: 1000
    device_paths:
      primary: "/dev/ttyACM0"
```

---

## Testing Strategy

### Phase 1: Unit Testing (No Hardware Required)

**Goal:** Verify protocol adapters work correctly

```python
# tests/unit/test_protocol_adapter.py

import pytest
from geometry_msgs.msg import Twist
from src.bridges.teleop_can_adapter import TeleopCANAdapter

class TestTeleopCANAdapter:
    """Test teleoperation protocol adapter."""
    
    def setup_method(self):
        self.adapter = TeleopCANAdapter()
    
    def test_twist_to_slcan_forward(self):
        """Test forward velocity encoding."""
        twist = Twist()
        twist.linear.x = 0.5  # 0.5 m/s forward
        twist.linear.y = 0.0
        twist.angular.z = 0.0
        
        slcan = self.adapter.twist_to_slcan(twist)
        
        # Expected: x=2048 (0x0800), y=0, rot=0
        assert slcan == 't00C6080000000000\r'
    
    def test_twist_to_slcan_rotation(self):
        """Test rotation velocity encoding."""
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = 0.2618  # 15 deg/s = 0.2618 rad/s
        
        slcan = self.adapter.twist_to_slcan(twist)
        
        # Expected: rot=960 (0x03C0)
        assert slcan == 't00C6000000000003c0\r'
    
    def test_slcan_to_twist_roundtrip(self):
        """Test encoding/decoding round-trip."""
        # Original twist
        twist_orig = Twist()
        twist_orig.linear.x = 0.5
        twist_orig.linear.y = 0.25
        twist_orig.angular.z = 0.2618
        
        # Encode
        slcan = self.adapter.twist_to_slcan(twist_orig)
        
        # Create response (same ID but 0x00D)
        slcan_response = slcan.replace('t00C', 't00D')
        
        # Decode
        twist_decoded = self.adapter.slcan_to_twist(slcan_response)
        
        # Verify round-trip accuracy
        assert abs(twist_decoded.linear.x - twist_orig.linear.x) < 0.001
        assert abs(twist_decoded.linear.y - twist_orig.linear.y) < 0.001
        assert abs(twist_decoded.angular.z - twist_orig.angular.z) < 0.01
    
    def test_velocity_scaling_limits(self):
        """Test velocity scaling at limits."""
        twist = Twist()
        twist.linear.x = 10.0  # Very high speed
        
        slcan = self.adapter.twist_to_slcan(twist)
        
        # Should clamp to 16-bit signed max (32767)
        assert '7fff' in slcan.lower() or '7FFF' in slcan
    
    def test_negative_velocity(self):
        """Test negative (backward) velocity encoding."""
        twist = Twist()
        twist.linear.x = -0.5  # Backward
        
        slcan = self.adapter.twist_to_slcan(twist)
        
        # Negative values use two's complement
        # -2048 = 0xF800
        assert 'f800' in slcan.lower() or 'F800' in slcan
```

**Run unit tests:**
```bash
cd /home/durian/urc-machiato-2026
pytest tests/unit/test_protocol_adapter.py -v
```

### Phase 2: Integration Testing (Simulated Hardware)

**Goal:** Test complete message flow without physical hardware

```python
# tests/integration/test_bridge_integration.py

import asyncio
import pytest
from unittest.mock import Mock, patch, AsyncMock
from src.bridges.can_bridge import CANBridge
from src.bridges.teleop_websocket_bridge import TeleopWebSocketBridge
from geometry_msgs.msg import Twist

class TestBridgeIntegration:
    """Test integration between ROS2, bridges, and adapters."""
    
    @pytest.mark.asyncio
    async def test_ros2_to_can_flow(self):
        """Test ROS2 → Protocol Adapter → CAN Bridge → Serial."""
        # Mock serial device
        with patch('serial.Serial') as mock_serial:
            mock_serial.return_value.write = Mock()
            
            # Create bridge with teleop adapter
            config = {
                'protocol': 'teleop',
                'device': '/dev/ttyAMA10',
                'baudrate': 115200
            }
            bridge = CANBridge(config)
            await bridge.connect()
            
            # Send ROS2 Twist
            twist = Twist()
            twist.linear.x = 0.5
            twist.angular.z = 0.2618
            
            message = BridgeMessage(
                message_type="motor_command",
                data={"twist": twist}
            )
            
            success = await bridge.send_message(message)
            
            # Verify SLCAN frame was written
            assert success
            assert mock_serial.return_value.write.called
            
            # Check frame format
            call_args = mock_serial.return_value.write.call_args[0][0]
            assert call_args.startswith(b't00C')  # Teleop msg ID
    
    @pytest.mark.asyncio
    async def test_can_to_ros2_feedback(self):
        """Test CAN Bridge → Protocol Adapter → ROS2."""
        # Mock serial device with response data
        with patch('serial.Serial') as mock_serial:
            # Simulate velocity response from teleoperation
            mock_serial.return_value.read_until = Mock(
                return_value=b't00D60800000003c0\r'
            )
            
            config = {'protocol': 'teleop', 'device': '/dev/ttyAMA10'}
            bridge = CANBridge(config)
            await bridge.connect()
            
            # Setup handler to capture parsed message
            received_messages = []
            async def handler(msg):
                received_messages.append(msg)
            
            await bridge.register_handler("velocity_feedback", handler)
            
            # Trigger message processing
            await asyncio.sleep(0.1)  # Allow processing loop to run
            
            # Verify message was parsed and routed
            assert len(received_messages) > 0
            
            # Check decoded values
            twist = received_messages[0].data['twist']
            assert abs(twist.linear.x - 0.5) < 0.001
    
    @pytest.mark.asyncio
    async def test_websocket_to_ros2_flow(self):
        """Test WebSocket → ROS2 topic flow."""
        # Mock Socket.IO client
        with patch('socketio.AsyncClient') as mock_sio:
            mock_sio_instance = AsyncMock()
            mock_sio.return_value = mock_sio_instance
            
            # Create mock ROS2 node
            mock_node = Mock()
            mock_node.teleop_cmd_pub = Mock()
            mock_node.get_logger = Mock(return_value=Mock())
            
            # Create WebSocket bridge
            ws_bridge = TeleopWebSocketBridge(mock_node)
            await ws_bridge.sio.connect(ws_bridge.server_url)
            
            # Simulate frontend sending drive command
            drive_data = {
                'xVel': 0.5,
                'yVel': 0.0,
                'rotVel': 15.0
            }
            
            # Trigger event handler
            await ws_bridge.sio._trigger_event('driveCommands', drive_data)
            
            # Verify ROS2 message was published
            assert mock_node.teleop_cmd_pub.publish.called
            
            # Check published Twist
            published_twist = mock_node.teleop_cmd_pub.publish.call_args[0][0]
            assert abs(published_twist.linear.x - 0.5) < 0.001
```

**Run integration tests:**
```bash
pytest tests/integration/test_bridge_integration.py -v
```

### Phase 3: Hardware-in-Loop Testing

**Goal:** Test with actual hardware devices

#### Test 3.1: CAN Bridge with Teleoperation Server

**Setup:**
1. Start teleoperation server: `cd vendor/teleoperation/server && ./run.sh`
2. Connect STM32 controllers (or use loopback for testing)
3. Launch hardware interface node

**Test Commands:**
```bash
# Terminal 1: Start teleoperation server
cd /home/durian/urc-machiato-2026/vendor/teleoperation/server
./run.sh

# Terminal 2: Launch hardware interface with teleop adapter
cd /home/durian/urc-machiato-2026
ros2 launch src/autonomy/control/hardware_interface/launch/hardware_interface.launch.py \
    protocol:=teleop \
    device:=/dev/ttyAMA10

# Terminal 3: Monitor CAN traffic
candump can0

# Terminal 4: Send test command
ros2 topic pub --once /cmd_vel/teleop geometry_msgs/Twist \
    "{linear: {x: 0.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.2618}}"

# Verify:
# - Teleoperation server logs show received command
# - candump shows SLCAN frame: t00C60800000003c0
# - Firmware responds (if connected)
```

#### Test 3.2: WebSocket Bridge with Frontend

**Setup:**
1. Start teleoperation frontend: `cd vendor/teleoperation && npm run dev`
2. Launch WebSocket bridge node
3. Test gamepad control

**Test Commands:**
```bash
# Terminal 1: Start frontend
cd /home/durian/urc-machiato-2026/vendor/teleoperation
npm run dev

# Terminal 2: Launch WebSocket bridge
cd /home/durian/urc-machiato-2026
ros2 run autonomy_sensor_bridge teleop_websocket_bridge

# Terminal 3: Monitor ROS2 topics
ros2 topic echo /cmd_vel/teleop

# Test in browser:
# 1. Open http://localhost:5173
# 2. Use gamepad to send drive commands
# 3. Verify commands appear in Terminal 3
# 4. Verify system status updates in frontend
```

#### Test 3.3: End-to-End Flow

**Complete system test:**

```bash
# Start all components
cd /home/durian/urc-machiato-2026

# 1. Teleoperation server
gnome-terminal -- bash -c "cd vendor/teleoperation/server && ./run.sh; exec bash"

# 2. Hardware interface with CAN bridge
gnome-terminal -- bash -c "ros2 launch src/autonomy/control/hardware_interface/launch/hardware_interface.launch.py protocol:=teleop; exec bash"

# 3. WebSocket bridge
gnome-terminal -- bash -c "ros2 run autonomy_sensor_bridge teleop_websocket_bridge; exec bash"

# 4. Frontend dashboard
gnome-terminal -- bash -c "cd vendor/teleoperation && npm run dev; exec bash"

# 5. Monitor system
gnome-terminal -- bash -c "ros2 topic echo /hardware/system_status; exec bash"

# Test flow:
# Frontend → WebSocket → ROS2 (/cmd_vel/teleop) → 
# → Adapter → CAN → Teleoperation Server → Firmware →
# → Sensor Feedback → CAN → ROS2 → WebSocket → Frontend
```

### Phase 4: Performance Testing

**Goal:** Measure latency, throughput, reliability

```python
# tests/performance/test_bridge_performance.py

import time
import asyncio
from statistics import mean, stdev

class TestBridgePerformance:
    """Performance and stress testing for bridges."""
    
    @pytest.mark.asyncio
    async def test_can_bridge_latency(self):
        """Measure CAN bridge message latency."""
        bridge = CANBridge({'protocol': 'teleop'})
        await bridge.connect()
        
        latencies = []
        
        for _ in range(100):
            twist = Twist()
            twist.linear.x = 0.5
            
            message = BridgeMessage(
                message_type="motor_command",
                data={"twist": twist}
            )
            
            start_time = time.time()
            await bridge.send_message(message)
            end_time = time.time()
            
            latencies.append((end_time - start_time) * 1000)  # ms
        
        avg_latency = mean(latencies)
        std_latency = stdev(latencies)
        
        print(f"CAN Bridge Latency: {avg_latency:.2f} ± {std_latency:.2f} ms")
        
        # Assert reasonable performance
        assert avg_latency < 10.0  # < 10ms average
        assert max(latencies) < 50.0  # < 50ms worst case
    
    @pytest.mark.asyncio
    async def test_bridge_throughput(self):
        """Measure message throughput."""
        bridge = CANBridge({'protocol': 'teleop'})
        await bridge.connect()
        
        message_count = 1000
        start_time = time.time()
        
        for i in range(message_count):
            twist = Twist()
            twist.linear.x = i * 0.01
            
            message = BridgeMessage(
                message_type="motor_command",
                data={"twist": twist}
            )
            
            await bridge.send_message(message)
        
        end_time = time.time()
        duration = end_time - start_time
        throughput = message_count / duration
        
        print(f"Bridge Throughput: {throughput:.2f} messages/sec")
        
        # Assert minimum throughput
        assert throughput > 100  # > 100 msg/s
```

**Run performance tests:**
```bash
pytest tests/performance/test_bridge_performance.py -v -s
```

### Phase 5: Stress Testing

**Goal:** Test system under extreme conditions

```python
# tests/stress/test_bridge_stress.py

class TestBridgeStress:
    """Stress testing for bridges."""
    
    @pytest.mark.asyncio
    async def test_rapid_command_changes(self):
        """Test rapid velocity changes."""
        bridge = CANBridge({'protocol': 'teleop'})
        await bridge.connect()
        
        # Send 1000 rapid velocity changes
        for i in range(1000):
            twist = Twist()
            twist.linear.x = (-1.0 + 2.0 * (i % 2))  # Alternate ±1.0 m/s
            
            message = BridgeMessage(
                message_type="motor_command",
                data={"twist": twist}
            )
            
            success = await bridge.send_message(message)
            assert success  # All messages should succeed
            
            await asyncio.sleep(0.001)  # 1ms between commands
    
    @pytest.mark.asyncio
    async def test_connection_recovery(self):
        """Test recovery from connection loss."""
        bridge = CANBridge({'protocol': 'teleop'})
        await bridge.connect()
        assert bridge.is_connected
        
        # Simulate connection loss
        await bridge.disconnect()
        assert not bridge.is_connected
        
        # Attempt reconnection
        success = await bridge.connect()
        assert success
        assert bridge.is_connected
        
        # Verify functionality restored
        message = BridgeMessage(
            message_type="motor_command",
            data={"twist": Twist()}
        )
        assert await bridge.send_message(message)
    
    @pytest.mark.asyncio
    async def test_concurrent_bridges(self):
        """Test multiple bridges operating concurrently."""
        can_bridge = CANBridge({'protocol': 'teleop', 'device': '/dev/ttyAMA10'})
        ws_bridge = TeleopWebSocketBridge(Mock())
        
        # Start both bridges
        await asyncio.gather(
            can_bridge.connect(),
            ws_bridge.sio.connect('http://localhost:4000')
        )
        
        # Send messages through both simultaneously
        async def send_can_messages():
            for _ in range(100):
                await can_bridge.send_message(
                    BridgeMessage(message_type="motor_command", data={})
                )
        
        async def send_ws_messages():
            for _ in range(100):
                await ws_bridge.send_status({"test": True})
        
        # Run concurrently
        await asyncio.gather(
            send_can_messages(),
            send_ws_messages()
        )
        
        # Verify no interference
        assert can_bridge.stats.messages_sent > 0
        assert can_bridge.stats.errors == 0
```

---

## Implementation Checklist

### 1. Protocol Adaptation Layer ✅

- [ ] Create `src/bridges/protocol_adapter.py` (base class)
- [ ] Create `src/bridges/teleop_can_adapter.py` (teleoperation protocol)
- [ ] Create `src/bridges/teleop_websocket_bridge.py` (WebSocket bidirectional)
- [ ] Add configuration for adapter selection (`config/bridge_config.yaml`)
- [ ] Update `src/bridges/can_bridge.py` to use adapter
- [ ] Write unit tests for all adapters

### 2. CAN Bridge Integration ✅

- [ ] Modify `src/bridges/can_bridge.py`:
  - [ ] Add adapter injection
  - [ ] Remove hardcoded message IDs
  - [ ] Add device auto-discovery
  - [ ] Add SLCAN protocol support
- [ ] Update `src/autonomy/control/hardware_interface/hardware_interface_node.py`:
  - [ ] Add adapter configuration parameter
  - [ ] Add device path configuration
  - [ ] Update CAN serial initialization
- [ ] Test with teleoperation server (loopback)

### 3. WebSocket Bridge Integration ✅

- [ ] Extend `src/autonomy/perception/sensor_bridge/autonomy_sensor_bridge/websocket_bridge.py`:
  - [ ] Add bidirectional event handlers
  - [ ] Add command reception (driveCommands, driveHoming)
  - [ ] Add status emission (systemStatus, metrics)
  - [ ] Add error handling and reconnection
- [ ] Create ROS2 publishers for WebSocket-originated commands
- [ ] Test with teleoperation frontend

### 4. Testing Infrastructure ✅

- [ ] Write unit tests (`tests/unit/test_protocol_adapter.py`)
- [ ] Write integration tests (`tests/integration/test_bridge_integration.py`)
- [ ] Write performance tests (`tests/performance/test_bridge_performance.py`)
- [ ] Write stress tests (`tests/stress/test_bridge_stress.py`)
- [ ] Create test launch files
- [ ] Document testing procedures

### 5. Configuration & Deployment ✅

- [ ] Create `config/bridge_config.yaml` with all parameters
- [ ] Update launch files to use configuration
- [ ] Add device discovery script
- [ ] Create deployment guide
- [ ] Add troubleshooting documentation

### 6. Documentation ✅

- [ ] Complete this architecture document
- [ ] Create user guide for configuration
- [ ] Create developer guide for adding new protocols
- [ ] Update README with bridge architecture
- [ ] Add diagrams and examples

---

## File Structure

```
/home/durian/urc-machiato-2026/
├── src/
│   ├── bridges/
│   │   ├── __init__.py
│   │   ├── unified_bridge_interface.py          # ✅ Exists
│   │   ├── can_bridge.py                        # ✅ Exists (update)
│   │   ├── protocol_adapter.py                  # ⚠️ Create
│   │   ├── teleop_can_adapter.py                # ⚠️ Create
│   │   └── teleop_websocket_bridge.py           # ⚠️ Create
│   │
│   ├── autonomy/
│   │   ├── control/
│   │   │   └── hardware_interface/
│   │   │       ├── hardware_interface_node.py   # ✅ Exists (update)
│   │   │       └── launch/
│   │   │           └── hardware_interface.launch.py
│   │   │
│   │   └── perception/
│   │       └── sensor_bridge/
│   │           └── autonomy_sensor_bridge/
│   │               └── websocket_bridge.py      # ✅ Exists (extend)
│   │
│   └── config/
│       └── bridge_config.yaml                   # ⚠️ Create
│
├── tests/
│   ├── unit/
│   │   └── test_protocol_adapter.py             # ⚠️ Create
│   ├── integration/
│   │   └── test_bridge_integration.py           # ⚠️ Create
│   ├── performance/
│   │   └── test_bridge_performance.py           # ⚠️ Create
│   └── stress/
│       └── test_bridge_stress.py                # ⚠️ Create
│
└── docs/
    ├── BRIDGE_INTEGRATION_ARCHITECTURE.md       # This document
    ├── SUBMODULE_INTERFACE_SPECIFICATION.md     # ✅ Exists
    └── SUBMODULE_INTEGRATION_SUMMARY.md         # ✅ Exists
```

---

## Next Steps

**Priority Order:**

1. **Create Protocol Adaptation Layer** (Est: 1-2 days)
   - Implement base adapter class
   - Implement teleoperation CAN adapter
   - Write unit tests

2. **Integrate with CAN Bridge** (Est: 1 day)
   - Modify CAN bridge to use adapter
   - Add SLCAN protocol support
   - Test with teleoperation server

3. **Extend WebSocket Bridge** (Est: 1 day)
   - Add bidirectional communication
   - Add command handlers
   - Test with frontend

4. **Testing & Validation** (Est: 2-3 days)
   - Write comprehensive test suite
   - Perform hardware-in-loop testing
   - Performance and stress testing

5. **Documentation & Deployment** (Est: 1 day)
   - Complete user/developer guides
   - Create deployment procedures
   - Train team on new architecture

**Total Estimated Time:** ~1 week

---

## Conclusion

This architecture makes the **main codebase malleable** by:

1. **Protocol Adaptation Layer:** Translates between ROS2 and submodule protocols
2. **Configuration-Driven:** Easy to switch between protocols without code changes
3. **Unified Interface:** Both CAN and WebSocket use same bridge interface
4. **Comprehensive Testing:** Validates all integration points
5. **No Submodule Changes:** Teleoperation and control-systems remain unchanged

The main codebase adapts to match the fixed protocols of the submodules, preventing conflicts and enabling seamless integration.
