# Submodule Interface Specification

## System Overview

The URC 2026 Mars Rover system consists of three integrated components:

1. **Main Codebase** - ROS2 autonomy stack (`/home/durian/urc-machiato-2026/`)
2. **Control Systems** - STM32 embedded firmware (`vendor/control-systems/`)
3. **Teleoperation** - Web-based remote operation interface (`vendor/teleoperation/`)

## Architecture Diagram

```
┌─────────────────────────────────────────────────────────┐
│                  Main ROS2 Autonomy Stack                │
│                                                          │
│  ┌─────────────────────────────────────────────────┐  │
│  │     hardware_interface_node.py                   │  │
│  │  - ROS2 Lifecycle Management                     │  │
│  │  - Twist Mux Command Arbitration                 │  │
│  │  - Emergency > Safety > Teleop > Autonomy        │  │
│  └────────────────┬────────────────────────────────┘  │
│                   │  CAN Serial                         │
│  ┌────────────────┴────────────────────────────────┐  │
│  │          can_bridge.py                           │  │
│  │  - Unified CAN Bus Interface                     │  │
│  │  - Motor Control / Sensor Feedback               │  │
│  └────────────────┬────────────────────────────────┘  │
└───────────────────┼──────────────────────────────────┘
                    │ CAN Serial (/dev/ttyACM0)
      ┌─────────────┴──────────────┬──────────────────┐
      │                            │                  │
┌─────┴────────┐         ┌─────────┴──────┐  ┌───────┴─────────┐
│ Teleoperation │         │ Control Systems │  │ Control Systems │
│   (Python)    │         │    (Drive)      │  │     (Arm)       │
│               │         │                 │  │                 │
│  py_server.py │         │  STM32 Firmware │  │  STM32 Firmware │
│  - Socket.IO  │         │  - CAN Protocol │  │  - CAN Protocol │
│  - CAN Serial │         │  - Swerve Drive │  │  - Arm Control  │
│  /dev/ttyAMA10│         │  - libhal       │  │  - libhal       │
└───────────────┘         └─────────────────┘  └─────────────────┘
```

## Interface 1: Main Codebase → Control Systems

### Communication Protocol: CAN Serial

**File:** `src/bridges/can_bridge.py`  
**Protocol:** SocketCAN over serial UART

#### Configuration
```python
interface = "can0"  # CAN interface name
device = "/dev/ttyACM0"  # Default device path
baudrate = 115200  # CAN-UART bridge baudrate
```

#### CAN Message Format
```python
class CANMessageType(Enum):
    MOTOR_COMMAND = 0x100
    MOTOR_FEEDBACK = 0x101
    IMU_DATA = 0x200
    ENCODER_DATA = 0x201
    BATTERY_STATUS = 0x300
    SYSTEM_STATUS = 0x400
    EMERGENCY_STOP = 0xFFF
```

#### Motor Control Commands

**Message ID:** `0x100` (MOTOR_COMMAND)  
**Format:** 8 bytes
```
Byte 0-1: Motor ID (uint16)
Byte 2-3: Speed command (int16, scaled)
Byte 4-5: Torque/current limit (int16)
Byte 6-7: Reserved
```

#### Motor Feedback

**Message ID:** `0x101` (MOTOR_FEEDBACK)  
**Format:** 8 bytes
```
Byte 0-1: Motor ID (uint16)
Byte 2-3: Actual speed (int16)
Byte 4-5: Current draw (int16)
Byte 6-7: Temperature (int16)
```

### ROS2 Topics Integration

**File:** `src/autonomy/control/hardware_interface/hardware_interface_node.py`

#### Subscribed Topics

| Topic | Message Type | Purpose | Priority |
|-------|--------------|---------|----------|
| `/cmd_vel/emergency` | `Twist` | Emergency stop commands | 1000 |
| `/cmd_vel/safety` | `Twist` | Safety system overrides | 900 |
| `/cmd_vel/teleop` | `Twist` | Manual teleoperation | 500 |
| `/cmd_vel/autonomy` | `Twist` | Autonomous navigation | 100 |
| `/emergency_stop` | `Bool` | Global emergency stop | 1000 |
| `/hardware/arm_command` | `String` | Arm control commands | N/A |
| `/hardware/led_command` | `LedCommand` | LED status control | N/A |

#### Published Topics

| Topic | Message Type | Purpose | Rate |
|-------|--------------|---------|------|
| `/hardware/joint_states` | `JointState` | Motor positions/velocities | 50 Hz |
| `/hardware/chassis_velocity` | `TwistStamped` | Actual chassis velocity | 50 Hz |
| `/hardware/motor_temperatures` | `Float32MultiArray` | Motor temperatures | 10 Hz |
| `/hardware/battery_state` | `BatteryState` | Battery voltage/current | 10 Hz |
| `/hardware/imu` | `Imu` | IMU data | 50 Hz |
| `/hardware/gps` | `NavSatFix` | GPS position | 10 Hz |
| `/hardware/system_status` | `String` | System health JSON | 1 Hz |

### Twist Mux Command Arbitration

**Priority System:**
```python
PRIORITIES = {
    "emergency": 1000,   # Highest - immediate stop
    "safety": 900,       # Safety system overrides
    "teleop": 500,       # Manual control
    "autonomy": 100,     # Autonomous navigation
    "idle": 0            # No active commands
}
```

**Command Timeout:** 0.5 seconds  
**Arbitration Rate:** 50 Hz (20ms control loop)

**Behavior:**
- Commands expire after timeout
- Highest priority active command is selected
- Automatic fallback to lower priority when command expires
- Emergency commands persist until explicitly cleared

## Interface 2: Teleoperation → Control Systems

### Communication Protocol: CAN Serial (Slcan)

**File:** `vendor/teleoperation/server/py_server.py`  
**Protocol:** Serial Line CAN (SLCAN) protocol

#### Configuration
```python
# Device paths
DRIVE_PORT = "/dev/ttyAMA10"  # Drive controller
ARM_PORT = "/dev/ttyACM1"     # Arm controller
BAUDRATE = 115200             # Serial baudrate
```

#### SLCAN Protocol Initialization
```python
# Reset sequence
send: '\r\r\r\r'
send: 'V\r'      # Get version
send: 'S8\r'     # Set CAN speed (500k)
send: 'O\r'      # Open CAN channel
```

#### CAN Message IDs (Teleoperation Protocol)

**Send IDs (Teleoperation → Firmware):**
```python
send_ID = {
    "SET_CHASSIS_VELOCITIES": '00C',   # 0x00C - Drive velocity commands
    "HEARTBEAT": '00E',                # 0x00E - System heartbeat
    "HOMING_SEQUENCE": '110',          # 0x110 - Initiate homing
    "GET_OFFSET": '112',               # 0x112 - Request encoder offset
    "GET_ESTIMATED_VELOCITIES": '114', # 0x114 - Request velocity estimate
    "CONFIG": '119',                   # 0x119 - Configuration command
    "SET_MAST_GIMBAL_OFFSET": '300',   # 0x300 - Mast gimbal offset
}
```

**Receive IDs (Firmware → Teleoperation):**
```python
receive_ID = {
    "SET_VELOCITIES_RESPONSE": '00D',          # 0x00D - Velocity command ACK
    "HEARTBEAT_REPLY": '00F',                  # 0x00F - Heartbeat response
    "HOMING_SEQUENCE_RESPONSE": '111',         # 0x111 - Homing complete
    "RETURN_OFFSET": '113',                    # 0x113 - Encoder offset data
    "RETURN_ESTIMATED_CHASSIS_VELOCITIES": '115', # 0x115 - Velocity feedback
    "CONFIG_ACK": '11A',                       # 0x11A - Config ACK
}
```

#### SET_CHASSIS_VELOCITIES Message (0x00C)

**Format:** SLCAN frame
```
t<ID><DLC><DATA>\r
```

**Example:**
```
t00C6<x_vel><y_vel><rot_vel>\r
```

**Data Encoding:**
- **x_vel** (2 bytes): X velocity, signed int16, scaled by 2^12
- **y_vel** (2 bytes): Y velocity, signed int16, scaled by 2^12  
- **rot_vel** (2 bytes): Rotational velocity, signed int16, scaled by 2^6 (degrees/sec)

**Scaling:**
```python
x_vel_scaled = int(x_vel_m_per_s * (2 ** 12))
y_vel_scaled = int(y_vel_m_per_s * (2 ** 12))
rot_vel_scaled = int(rot_vel_deg_per_s * (2 ** 6))
```

**Example Message:**
```python
# Send 0.5 m/s forward, 0 m/s lateral, 15 deg/s rotation
x_vel = int(0.5 * 4096) = 2048 = 0x0800
y_vel = int(0.0 * 4096) = 0 = 0x0000
rot_vel = int(15.0 * 64) = 960 = 0x03C0

# SLCAN frame:
't00C6080000000003C0\r'
```

#### VELOCITY_RESPONSE Message (0x00D)

**Format:**
```
r<ID><DLC><DATA>\r
```

**Data:**
- Same format as command (x_vel, y_vel, rot_vel)
- Echoes back commanded velocities
- Used for acknowledgment

#### HOMING_SEQUENCE Message (0x110)

**Format:**
```
t110 80000000000000000\r
```

**Purpose:** Initiates swerve module homing sequence
**Response:** 0x111 when homing complete

### WebSocket/Socket.IO Interface

**File:** `vendor/teleoperation/server/py_server.py`  
**Protocol:** Socket.IO over WebSocket

#### Client → Server Events

| Event | Data | Purpose |
|-------|------|---------|
| `driveCommands` | `{xVel, yVel, rotVel}` | Drive velocity commands |
| `driveHoming` | None | Initiate drive homing |
| `connect` | None | Client connection |
| `disconnect` | None | Client disconnection |

#### Server → Client Events

| Event | Data | Purpose |
|-------|------|---------|
| `metrics` | `{...}` | System metrics (SSH, CPU) |
| `armUpdate` | `{armStatus}` | Arm status (future) |

**Server Configuration:**
```python
host = '0.0.0.0'
port = 4000
cors_allowed_origins = '*'
```

### Frontend Interface

**File:** `vendor/teleoperation/src/components/gamepad/Gamepad.jsx`

**Component Hierarchy:**
```
App.jsx
├── DriveView.jsx
│   ├── DriveWidget.jsx
│   │   └── Gamepad.jsx (Drive control)
│   └── CameraPane.jsx
├── ArmView.jsx
├── AutonomyView.jsx
│   └── AutonomyControls.jsx
├── ScienceView.jsx
└── MapView.jsx
    └── Map.jsx
```

## Interface 3: Control Systems Firmware

### Subsystem Architecture

**Repository:** `vendor/control-systems/`  
**Framework:** libhal (Hardware Abstraction Layer)  
**Platform:** STM32F103C8 (ARM Cortex-M3)

#### Subsystems

| Subsystem | Path | Purpose | CAN Messages |
|-----------|------|---------|--------------|
| Drive | `drive/` | Swerve drive control | 0x00C-0x115 |
| Arm | `arm/` | Robotic arm control | 0x200-0x2FF |
| Science | `science/` | Science station | 0x400-0x4FF |
| Perseus | `perseus/` | BLDC motor controller | 0x500-0x5FF |
| Hub | `hub/` | Central hub/router | All messages |
| Drivers | `drivers/` | Sensor drivers | Various |

#### Drive Subsystem API

**File:** `vendor/control-systems/drive/include/drivetrain.hpp`

**Class:** `sjsu::drive::drivetrain`

**Methods:**
```cpp
// Set target chassis velocities
bool set_target_state(chassis_velocities p_target_state, 
                      bool p_resolve_module_conflicts);

// Get current target velocities
chassis_velocities get_target_state();

// Get actual velocities from sensors
chassis_velocities get_actual_state();

// Main control loop update
void periodic();

// Emergency stop
void stop();

// Homing sequence
void hard_home();
```

**Data Structures:**
```cpp
struct vector2d {
    float x;  // X component
    float y;  // Y component
};

struct chassis_velocities {
    vector2d translation;     // Translation velocity (m/s)
    float rotational_vel;     // Rotational velocity (rad/s)
};

struct swerve_module_state {
    float angle;      // Module angle (radians)
    float velocity;   // Wheel velocity (m/s)
};
```

### Build System

**Build Command:**
```bash
cd vendor/control-systems/drive
conan build . -pr stm32f103c8 -pr arm-gcc-14.2 -b missing
```

**Flash Command:**
```bash
stm32loader -e -w -v -B -p /dev/tty.usbserial-10 \
    ./build/stm32f103c8/MinSizeRel/application.elf.bin
```

### CAN Protocol Implementation Status

**⚠️ CRITICAL FINDING:** The control-systems firmware does **NOT** currently implement the teleoperation CAN protocol.

**Current State:**
- `drive/applications/application.cpp` - **STUB** (comments only, no implementation)
- Demo applications exist but use direct API calls, not CAN messages
- No CAN message parsing implementation found
- No message ID definitions matching teleoperation protocol

**Required Implementation:**
1. CAN message parser for messages 0x00C-0x115
2. Integration between CAN messages and drivetrain API
3. Velocity scaling/conversion logic
4. Heartbeat response implementation
5. Homing sequence handler

## Interface Compatibility Analysis

### Protocol Compatibility Matrix

| Feature | Main Codebase | Teleoperation | Control Systems | Status |
|---------|---------------|---------------|-----------------|--------|
| CAN Serial | ✅ can_bridge.py | ✅ can_serial.py | ✅ libhal | ✅ Compatible |
| Message IDs | 0x100-0xFFF | 0x00C-0x300 | ⚠️ Not defined | ⚠️ Needs implementation |
| Velocity Encoding | Generic | 2^12/2^6 scaling | ⚠️ Not implemented | ❌ Mismatch |
| SLCAN Protocol | ❌ Not used | ✅ Full support | ⚠️ Unknown | ⚠️ Unclear |
| Device Paths | /dev/ttyACM0 | /dev/ttyAMA10 | Hardware dependent | ⚠️ Configuration needed |

### Integration Gaps

#### Gap 1: CAN Protocol Implementation in Firmware
**Issue:** Control-systems firmware doesn't implement CAN message handling  
**Impact:** Teleoperation and main codebase cannot control hardware  
**Solution:** Implement CAN parser and message handlers in `drive/applications/application.cpp`

#### Gap 2: Message ID Discrepancy
**Issue:** Main codebase uses 0x100+ IDs, teleoperation uses 0x00C+ IDs  
**Impact:** Systems cannot communicate without translation layer  
**Solution:** 
- Option A: Align main codebase to teleoperation protocol
- Option B: Implement protocol bridge in firmware
- Option C: Create unified protocol specification

#### Gap 3: Velocity Scaling Differences
**Issue:** Teleoperation uses fixed-point scaling (2^12), main codebase uses float  
**Impact:** Velocity commands may be misinterpreted  
**Solution:** Implement consistent scaling layer in hardware interface

#### Gap 4: Device Path Configuration
**Issue:** Different systems expect different device paths  
**Impact:** Hardware connections may fail  
**Solution:** Implement device path discovery and configuration system

## Testing Requirements

### Test 1: CAN Serial Connectivity
**Verify:** Physical CAN connection establishment  
**Tools:** `candump`, `cansend`  
**Success Criteria:**
- CAN interface visible via `ip link`
- Messages can be sent/received via `candump can0`

### Test 2: Protocol Compatibility
**Verify:** Message format compatibility between systems  
**Tools:** Custom test script (see below)  
**Success Criteria:**
- Teleoperation messages decoded correctly
- Main codebase messages understood
- Firmware responds to commands

### Test 3: ROS2 Integration
**Verify:** Hardware interface node communication  
**Tools:** ROS2 topic monitoring  
**Success Criteria:**
- Topics publish at correct rates
- Command arbitration works correctly
- Emergency stop functions properly

### Test 4: End-to-End Control
**Verify:** Complete control loop functionality  
**Tools:** Full system integration test  
**Success Criteria:**
- Gamepad → Teleoperation → Firmware → Motors
- Sensor feedback returns to ROS2
- Both teleop and autonomy can control hardware

## Recommendations

### Immediate Actions (Priority 1)

1. **Implement CAN Protocol in Firmware**
   - Add message parser to `drive/applications/application.cpp`
   - Implement message handlers for 0x00C-0x115
   - Add velocity scaling conversion
   - Implement heartbeat and homing responses

2. **Standardize Message IDs**
   - Create unified protocol specification document
   - Update main codebase to use teleoperation IDs
   - Document all message formats

3. **Fix Device Path Configuration**
   - Add device discovery in hardware interface node
   - Make device paths configurable via ROS2 parameters
   - Add fallback devices

### Short-term Actions (Priority 2)

4. **Create Protocol Bridge**
   - Implement translation layer if needed
   - Add protocol validation and error handling
   - Create debug/monitoring tools

5. **Add Comprehensive Testing**
   - Create integration test suite
   - Add continuous testing for all interfaces
   - Implement hardware-in-loop testing

6. **Improve Documentation**
   - Document complete message catalog
   - Add interface diagrams
   - Create troubleshooting guide

### Long-term Actions (Priority 3)

7. **Refactor for Consistency**
   - Unify protocol across all systems
   - Create shared protocol library
   - Implement versioning

8. **Add Monitoring**
   - Create real-time protocol monitor
   - Add latency and bandwidth measurements
   - Implement automatic diagnostics

## Conclusion

The submodule interfaces are **partially compatible** but require significant implementation work in the control-systems firmware to achieve full integration. The main codebase and teleoperation server have well-defined interfaces, but the control-systems firmware currently lacks the CAN protocol implementation needed to bridge them.

**Critical Path:**
1. Implement CAN message handlers in firmware
2. Test with teleoperation server
3. Integrate with main codebase ROS2 node
4. Validate end-to-end control loop

**Estimated Effort:**
- Firmware CAN implementation: 3-5 days
- Testing and validation: 2-3 days
- Documentation and refinement: 1-2 days
- Total: ~1-2 weeks of focused development
