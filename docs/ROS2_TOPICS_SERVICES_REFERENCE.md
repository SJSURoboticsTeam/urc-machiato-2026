# ROS2 Topics and Services Reference - URC 2026 Autonomy System

## Overview

This document provides a comprehensive reference for all ROS2 topics, services, and message types used in the URC 2026 Mars Rover autonomy system. The system uses a hierarchical architecture with multiple ROS2 nodes communicating through well-defined interfaces.

## Communication Types: Intra-Process vs Inter-Process

ROS2 supports two communication modes that significantly impact performance and system architecture:

### **🔗 Intra-Process Communication (Coupled)**

- **What it is**: Direct shared memory communication between publishers and subscribers within the same process
- **Performance**: 50-70% lower latency, reduced CPU usage, no serialization overhead
- **Use case**: High-frequency topics (>10Hz), real-time control loops, safety-critical data
- **Requirements**: Compatible QoS profiles (same durability), same process
- **Benefits**: Maximum performance, minimal latency, efficient resource usage
- **Table indicator**: ✅ **COUPLED**

### **🌐 Inter-Process Communication (Decoupled)**

- **What it is**: DDS-based network communication between different processes/nodes
- **Performance**: Higher latency, network overhead, serialization required
- **Use case**: Cross-node communication, distributed systems, fault isolation
- **Requirements**: DDS middleware, compatible QoS profiles
- **Benefits**: Fault isolation, network transparency, scalability
- **Table indicator**: ❌ **DECOUPLED**

### **Decision Criteria for Coupling**

- ✅ **Couple** (Intra-process): High-frequency sensors, control commands, safety topics
- ❌ **Decouple** (Inter-process): Cross-node communication, monitoring, status updates

## Architecture Overview

The autonomy system consists of the following ROS2 nodes:

- **State Machine Director** (`autonomy_state_management`)
- **Safety Watchdog** (`autonomy_safety_system`)
- **Computer Vision Node** (`autonomy_computer_vision`)
- **Navigation Node** (`autonomy_navigation`)
- **Sensor Bridge** (`autonomy_sensor_bridge`)
- **LED Status** (`autonomy_led_status`)

---

## 1. STATE MACHINE DIRECTOR (`/state_machine/*`)

### Publishers

| Topic                          | Message Type                          | QoS Profile                | Communication Type | Description                                          | Frequency     |
| ------------------------------ | ------------------------------------- | -------------------------- | ------------------ | ---------------------------------------------------- | ------------- |
| `/state_machine/current_state` | `autonomy_interfaces/SystemState`     | RELIABLE + TRANSIENT_LOCAL | ✅ **COUPLED**     | Current rover state (BOOT, IDLE, NAVIGATION, etc.)   | State changes |
| `/state_machine/transitions`   | `autonomy_interfaces/StateTransition` | RELIABLE                   | ❌ **DECOUPLED**   | State transition notifications                       | Transitions   |
| `/state_machine/safety_status` | `autonomy_interfaces/SafetyStatus`    | RELIABLE + TRANSIENT_LOCAL | ✅ **COUPLED**     | Current safety status from state machine perspective | 1 Hz          |

### Services

| Service                              | Service Type                            | Description                          |
| ------------------------------------ | --------------------------------------- | ------------------------------------ |
| `/state_machine/change_state`        | `autonomy_interfaces/ChangeState`       | Request state change with validation |
| `/state_machine/get_system_state`    | `autonomy_interfaces/GetSystemState`    | Get current system state details     |
| `/state_machine/recover_from_safety` | `autonomy_interfaces/RecoverFromSafety` | Recover from safety violation        |
| `/state_machine/safestop_control`    | `autonomy_interfaces/SafestopControl`   | Engage/disengage safestop            |
| `/state_machine/software_estop`      | `autonomy_interfaces/SoftwareEstop`     | Trigger software emergency stop      |

### Subscribers

| Topic                    | Message Type                       | Communication Type | Description                  |
| ------------------------ | ---------------------------------- | ------------------ | ---------------------------- |
| `/safety/emergency_stop` | `std_msgs/Bool`                    | ✅ **COUPLED**     | Emergency stop signals       |
| `/safety/violations`     | `autonomy_interfaces/SafetyStatus` | ✅ **COUPLED**     | Safety violations            |
| `/imu`                   | `sensor_msgs/Imu`                  | ✅ **COUPLED**     | IMU data for state decisions |
| `/battery/status`        | `sensor_msgs/BatteryState`         | ❌ **DECOUPLED**   | Battery status monitoring    |

---

## 2. SAFETY WATCHDOG (`/safety/*`)

### Publishers

| Topic                     | Message Type                       | QoS Profile | Communication Type | Description                     | Frequency    |
| ------------------------- | ---------------------------------- | ----------- | ------------------ | ------------------------------- | ------------ |
| `/safety/emergency_stop`  | `std_msgs/Bool`                    | RELIABLE    | ✅ **COUPLED**     | Emergency stop signal broadcast | Event-driven |
| `/safety/violations`      | `autonomy_interfaces/SafetyStatus` | RELIABLE    | ✅ **COUPLED**     | Current safety violations       | Event-driven |
| `/safety/watchdog_status` | `std_msgs/String`                  | RELIABLE    | ❌ **DECOUPLED**   | Watchdog operational status     | 1 Hz         |
| `/safety/diagnostics`     | `diagnostic_msgs/DiagnosticArray`  | RELIABLE    | ❌ **DECOUPLED**   | Detailed diagnostic information | 1 Hz         |

### Subscribers

| Topic                             | Message Type                      | Communication Type | Description             |
| --------------------------------- | --------------------------------- | ------------------ | ----------------------- |
| `/state_machine/heartbeat`        | `std_msgs/String`                 | ❌ **DECOUPLED**   | State machine heartbeat |
| `/state_machine/current_state`    | `autonomy_interfaces/SystemState` | ✅ **COUPLED**     | Current system state    |
| `/state_machine/subsystem_status` | `std_msgs/String`                 | ❌ **DECOUPLED**   | Subsystem health status |
| `/battery/status`                 | `sensor_msgs/BatteryState`        | ❌ **DECOUPLED**   | Battery status          |
| `/temperature/data`               | `sensor_msgs/Temperature`         | ❌ **DECOUPLED**   | Temperature monitoring  |

---

## 3. COMPUTER VISION NODE (`vision/*`, `camera/*`)

### Publishers

| Topic                    | Message Type                          | QoS Profile                | Communication Type | Description                        | Frequency  |
| ------------------------ | ------------------------------------- | -------------------------- | ------------------ | ---------------------------------- | ---------- |
| `vision/detections`      | `autonomy_interfaces/VisionDetection` | BEST_EFFORT                | ✅ **COUPLED**     | ArUco marker and object detections | 30 Hz      |
| `vision/debug_image`     | `sensor_msgs/Image`                   | BEST_EFFORT                | ❌ **DECOUPLED**   | Debug visualization image          | 30 Hz      |
| `computer_vision/status` | `std_msgs/String`                     | RELIABLE                   | ✅ **COUPLED**     | Computer vision operational status | 1 Hz       |
| `camera/image_raw`       | `sensor_msgs/Image`                   | BEST_EFFORT                | 🤔 **CONDITIONAL** | Republished camera image           | Camera FPS |
| `camera/depth/image_raw` | `sensor_msgs/Image`                   | BEST_EFFORT                | 🤔 **CONDITIONAL** | Republished depth image            | Camera FPS |
| `camera/camera_info`     | `sensor_msgs/CameraInfo`              | RELIABLE + TRANSIENT_LOCAL | ❌ **DECOUPLED**   | Camera calibration info            | Static     |

### Subscribers

| Topic                    | Message Type             | Communication Type | Description        |
| ------------------------ | ------------------------ | ------------------ | ------------------ |
| `camera/image_raw`       | `sensor_msgs/Image`      | 🤔 **CONDITIONAL** | Raw camera image   |
| `camera/depth/image_raw` | `sensor_msgs/Image`      | 🤔 **CONDITIONAL** | Raw depth image    |
| `camera/camera_info`     | `sensor_msgs/CameraInfo` | ❌ **DECOUPLED**   | Camera calibration |

---

## 4. NAVIGATION NODE (`/navigation/*`, `/cmd_vel`)

### Publishers

| Topic                          | Message Type                | QoS Profile | Communication Type | Description                | Frequency     |
| ------------------------------ | --------------------------- | ----------- | ------------------ | -------------------------- | ------------- |
| `/cmd_vel`                     | `geometry_msgs/Twist`       | RELIABLE    | ✅ **COUPLED**     | Velocity commands to rover | 50 Hz         |
| `/navigation/current_waypoint` | `geometry_msgs/PoseStamped` | RELIABLE    | ✅ **COUPLED**     | Current navigation target  | State changes |

### Subscribers

| Topic                          | Message Type                      | Communication Type | Description              |
| ------------------------------ | --------------------------------- | ------------------ | ------------------------ |
| `/odom`                        | `nav_msgs/Odometry`               | ✅ **COUPLED**     | Odometry data            |
| `/imu`                         | `sensor_msgs/Imu`                 | ✅ **COUPLED**     | IMU data                 |
| `/gps/fix`                     | `sensor_msgs/NavSatFix`           | ❌ **DECOUPLED**   | GPS position             |
| `/state_machine/current_state` | `autonomy_interfaces/SystemState` | ✅ **COUPLED**     | Navigation state control |

---

## 5. SENSOR BRIDGE (`/sensor_bridge/*`)

### Publishers

| Topic                    | Message Type                      | QoS Profile | Communication Type | Description               | Frequency |
| ------------------------ | --------------------------------- | ----------- | ------------------ | ------------------------- | --------- |
| `/sensor_bridge/status`  | `std_msgs/String`                 | RELIABLE    | ✅ **COUPLED**     | Bridge operational status | 1 Hz      |
| `/sensor_bridge/metrics` | `std_msgs/String`                 | RELIABLE    | ❌ **DECOUPLED**   | Performance metrics       | 1 Hz      |
| `/diagnostics`           | `diagnostic_msgs/DiagnosticArray` | RELIABLE    | ❌ **DECOUPLED**   | Diagnostic information    | 1 Hz      |
| `/imu`                   | `sensor_msgs/Imu`                 | BEST_EFFORT | ✅ **COUPLED**     | IMU sensor data           | 100 Hz    |
| `/gps/fix`               | `sensor_msgs/NavSatFix`           | RELIABLE    | ❌ **DECOUPLED**   | GPS position data         | 5 Hz      |
| `/battery/status`        | `sensor_msgs/BatteryState`        | RELIABLE    | ❌ **DECOUPLED**   | Battery status            | 1 Hz      |
| `/odom`                  | `nav_msgs/Odometry`               | RELIABLE    | ✅ **COUPLED**     | Odometry data             | 50 Hz     |
| `/temperature/data`      | `sensor_msgs/Temperature`         | RELIABLE    | ❌ **DECOUPLED**   | Temperature data          | 1 Hz      |

---

## 6. LED STATUS (`/led/*`)

### Publishers

| Topic              | Message Type                      | QoS Profile | Communication Type | Description                    | Frequency     |
| ------------------ | --------------------------------- | ----------- | ------------------ | ------------------------------ | ------------- |
| `/led/status`      | `std_msgs/String`                 | RELIABLE    | ❌ **DECOUPLED**   | Current LED status and pattern | State changes |
| `/led/diagnostics` | `diagnostic_msgs/DiagnosticArray` | RELIABLE    | ❌ **DECOUPLED**   | LED system diagnostics         | 1 Hz          |

### Subscribers

| Topic                         | Message Type      | Communication Type | Description                       |
| ----------------------------- | ----------------- | ------------------ | --------------------------------- |
| `/state_machine/led_info`     | `std_msgs/String` | ❌ **DECOUPLED**   | LED control information           |
| `/state_machine/system_state` | `std_msgs/String` | ❌ **DECOUPLED**   | System state for LED patterns     |
| `/mission_status`             | `std_msgs/String` | ❌ **DECOUPLED**   | Mission status for LED indicators |

---

## 7. MISSION CONTROL (`/mission/*`)

### Publishers/Subscribers (Frontend Integration)

| Topic               | Message Type                          | QoS Profile | Communication Type | Direction                | Description              |
| ------------------- | ------------------------------------- | ----------- | ------------------ | ------------------------ | ------------------------ |
| `/mission/commands` | `std_msgs/String`                     | RELIABLE    | ❌ **DECOUPLED**   | Frontend → State Machine | Mission commands         |
| `/mission/progress` | `autonomy_interfaces/MissionProgress` | RELIABLE    | ❌ **DECOUPLED**   | State Machine → Frontend | Mission progress updates |
| `/mission/status`   | `std_msgs/String`                     | RELIABLE    | ❌ **DECOUPLED**   | State Machine → Frontend | Mission status           |

---

## Message Type Reference

### autonomy_interfaces/SystemState

```ros2
std_msgs/Header header
string current_state              # BOOT, CALIBRATION, IDLE, NAVIGATION, EXECUTION, RECOVERY, SHUTDOWN
string substate                   # SCIENCE, DELIVERY, MAINTENANCE, etc.
string sub_substate              # SAMPLE_COLLECTION, DELIVERY_SETUP, etc.
float64 time_in_state            # Seconds in current state
float64 state_timeout            # Max time allowed in state
string previous_state            # Last state
builtin_interfaces/Time transition_timestamp
bool is_transitioning            # True during transitions
bool preconditions_met           # Autonomous operation prerequisites
string[] active_subsystems       # Currently active subsystems
string[] failed_subsystems       # Failed subsystems
string mission_phase             # Competition mission context
string operator_id               # Who initiated state
string state_reason              # State explanation
```

### autonomy_interfaces/SafetyStatus

```ros2
std_msgs/Header header
bool is_safe                     # Overall safety status
string safety_level              # NORMAL, WARNING, CRITICAL, EMERGENCY
string[] active_triggers         # Active safety triggers
string trigger_type              # Type of trigger
string trigger_source            # Subsystem that triggered
builtin_interfaces/Time trigger_time
string trigger_description       # Human-readable description
bool requires_manual_intervention # Manual recovery required
bool can_auto_recover            # Automatic recovery possible
string[] recovery_steps          # Recovery steps
float64 estimated_recovery_time  # Recovery time estimate
string context_state             # State when triggered
string mission_phase             # Mission context
bool safe_to_retry               # Safe to retry
float64 battery_level            # Battery percentage
float64 temperature              # System temperature
bool communication_ok            # Communication status
bool sensors_ok                  # Sensor status
string[] degraded_capabilities   # Degraded capabilities
```

### autonomy_interfaces/VisionDetection

```ros2
std_msgs/Header header
builtin_interfaces/Time timestamp
string detector_type             # ARUCO, OBJECT, OBSTACLE, etc.
string target_type               # MARKER, SAMPLE, OBSTACLE, etc.
int32 target_id                  # Target identifier
geometry_msgs/PoseWithCovariance pose  # 3D pose with uncertainty
float64 confidence               # Detection confidence [0.0, 1.0]
float64[] bbox                   # 2D bounding box [x,y,w,h]
string[] additional_data         # Extra detection metadata
```

---

## QoS Profile Standards

| QoS Profile         | Reliability | Durability      | Communication Type       | Use Case                                          |
| ------------------- | ----------- | --------------- | ------------------------ | ------------------------------------------------- |
| **RELIABLE**        | RELIABLE    | VOLATILE        | ✅ Intra-process enabled | Safety-critical, state changes, control commands  |
| **BEST_EFFORT**     | BEST_EFFORT | VOLATILE        | ✅ Intra-process enabled | High-frequency sensor data (>10Hz)                |
| **TRANSIENT_LOCAL** | RELIABLE    | TRANSIENT_LOCAL | ✅ Intra-process enabled | State info for new subscribers, persistent status |

### **QoS and Intra-Process Communication**

- **VOLATILE durability**: Enables intra-process communication for real-time data
- **TRANSIENT_LOCAL durability**: Enables intra-process with persistence for late-joining subscribers
- **Automatic coupling**: ROS2 automatically uses intra-process when QoS profiles are compatible and publishers/subscribers are in the same process

---

## Service Interface Reference

### SoftwareEstop Service

**Service:** `autonomy_interfaces/SoftwareEstop`

```ros2
# Request
string operator_id              # Operator triggering ESTOP
string reason                   # Reason for ESTOP
bool acknowledge_criticality    # Operator acknowledges criticality
bool force_immediate            # Force immediate shutdown

# Response
bool success                    # ESTOP triggered successfully
string message                  # Status message
string estop_id                 # Unique ESTOP identifier
float64 timestamp               # Trigger timestamp
string triggered_by             # Who triggered it
bool coordination_started       # Subsystem coordination started
```

### SafestopControl Service

**Service:** `autonomy_interfaces/SafestopControl`

```ros2
# Request
string command                  # "ENGAGE", "DISENGAGE", "TOGGLE"
string operator_id              # Operator initiating command
bool acknowledge_risks          # Risk acknowledgment
string reason                   # Optional reason
string[] affected_subsystems    # Specific subsystems (empty = all)

# Response
bool success                    # Command successful
string message                  # Status message
string current_state            # ENGAGED, DISENGAGED, etc.
string[] affected_subsystems    # Affected subsystems
float64 timestamp               # Command timestamp
string operator_id              # Operator who executed
```

---

## Topic Naming Conventions

1. **Namespace by Component:** `/component_name/topic_name`
   - `/state_machine/*` - State machine topics
   - `/safety/*` - Safety system topics
   - `/navigation/*` - Navigation topics

2. **Standard ROS2 Topics:**
   - `/cmd_vel` - Velocity commands
   - `/odom` - Odometry
   - `/imu` - IMU data
   - `/gps/fix` - GPS data

3. **Message Flow Patterns:**
   - **One-to-Many:** Safety signals (emergency_stop)
   - **Many-to-One:** Sensor data aggregation
   - **Request-Response:** Services for control operations

---

## Monitoring and Diagnostics

### Diagnostic Topics

- `/diagnostics` - Aggregated diagnostic information
- `/safety/diagnostics` - Safety-specific diagnostics
- Component status topics (e.g., `computer_vision/status`)

### Health Monitoring

- Heartbeat mechanisms between critical components
- Timeout monitoring for safety-critical communications
- Automatic fault detection and reporting

---

## Deployment Notes

1. **Node Launch Order:**
   - Safety Watchdog (first for monitoring)
   - State Machine Director (central coordination)
   - Subsystem nodes (Computer Vision, Navigation, etc.)
   - Sensor Bridge (data integration)

2. **QoS Compatibility:**
   - Publishers and subscribers must use compatible QoS profiles
   - Safety-critical topics require RELIABLE QoS
   - High-frequency topics use BEST_EFFORT QoS

3. **Network Configuration:**
   - ROS2 DDS handles automatic discovery
   - Topics work across network boundaries
   - Latency-critical topics use appropriate QoS settings

4. **Intra-Process Communication:**
   - High-performance topics use intra-process communication automatically
   - Coupled topics (✅) bypass DDS for 50-70% latency reduction
   - Decoupled topics (❌) use full DDS networking for fault isolation
   - Conditional topics (🤔) depend on node deployment architecture

---

## Communication Coupling Summary

### **Coupled Topics (Intra-Process) - ✅ 13 topics**

High-performance, low-latency communication for critical systems:

**High-Frequency Sensors (4):**

- `/imu` (100 Hz) - IMU sensor data
- `/odom` (50 Hz) - Odometry data
- `/cmd_vel` (50 Hz) - Velocity commands
- `vision/detections` (30 Hz) - Computer vision detections

**Safety & Control (4):**

- `/safety/emergency_stop` - Emergency stop signals
- `/safety/violations` - Safety violation status
- `/state_machine/current_state` - System state
- `/state_machine/safety_status` - Safety status

**Status & Coordination (5):**

- `/sensor_bridge/status` - Bridge operational status
- `computer_vision/status` - Vision system status
- `/navigation/current_waypoint` - Navigation targets
- `/state_machine/current_state` (subscriber) - State coordination
- `/state_machine/safety_status` (subscriber) - Safety coordination

### **Decoupled Topics (Inter-Process) - ❌ 17 topics**

Network-transparent communication for distributed systems:

- Cross-node monitoring (diagnostics, battery, temperature)
- Frontend integration (mission control topics)
- LED status updates
- GPS positioning data
- Debug and visualization data

### **Conditional Topics (🤔) - 3 topics**

Architecture-dependent coupling:

- Camera image streams (depends on sensor placement)
- Depth image streams (depends on processing architecture)

**Total Topics: 33** | **Coupled: 39%** | **Decoupled: 52%** | **Conditional: 9%**

---

_This document is maintained as part of the URC 2026 autonomy system documentation. Last updated: $(date)_
