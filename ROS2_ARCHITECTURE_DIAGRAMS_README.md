# URC 2026 ROS2 Architecture Diagrams

This directory contains visual UML diagrams showing the complete ROS2 topics and services architecture for the URC 2026 Mars Rover autonomy system.

## Generated Diagrams

### 1. `URC 2026 ROS2 Architecture.png`
**Overview diagram** showing the complete data flow between all ROS2 nodes and external systems.

**Key Components:**
- **Competition Bridge** (`competition_bridge`) - Central telemetry hub
- **State Machine Bridge** (`ros2_state_machine_bridge`) - System state management
- **SLAM Node** (`autonomy_slam`) - RGB-D SLAM with RTAB-Map
- **Navigation Node** (`autonomy_navigation`) - GNSS waypoint navigation
- **Map Data Bridge** (`map_data_bridge`) - SLAM to frontend bridge
- **Hardware Sensors** - Physical sensor data sources
- **Mission Control** - Mission execution and commands
- **Safety Systems** - Emergency stop and boundary monitoring
- **LED Control** - Status indicators
- **C2 Display** - Operator interface highlights
- **Dashboard** - WebSocket telemetry streaming

### 2. `URC 2026 ROS2 Detailed Architecture.png`
**Detailed diagram** with message types and service interfaces.

**Complete Topic List (35+ topics):**

#### Hardware Topics (6):
- `/hardware/battery_state` (BatteryState)
- `/hardware/chassis_velocity` (TwistStamped)
- `/hardware/gps` (NavSatFix)
- `/hardware/imu` (Imu)
- `/hardware/system_status` (String)
- `/odom` (Odometry)

#### Mission Topics (5):
- `/mission/commands` (String)
- `/mission/keyboard_status` (String)
- `/mission/sample_collection_status` (String)
- `/mission/state_request` (String)
- `/mission/status` (String)

#### Safety Topics (2):
- `/emergency_stop` (Bool)
- `/safety/boundary_violation` (Bool)

#### Control Topics (2):
- `/led/command` (LedCommand)
- `/c2/object_highlight` (VisionDetection)

#### State Machine Topics (5):
- `/state_machine/current_state` (String)
- `/state_machine/for_mission_control` (String)
- `/state_machine/metadata` (String)
- `/state_machine/state_transition` (String)
- `/state_machine/transition_request` (String)

#### SLAM Topics (6):
- `slam/pose` (PoseWithCovarianceStamped)
- `slam/pose/fused` (PoseWithCovarianceStamped)
- `slam/system/status` (SlamStatus)
- `slam/system/health` (String)
- `rtabmap/stat` (String)
- `/map` (OccupancyGrid)

#### Navigation Topics (4):
- `/cmd_vel` (Twist)
- `/navigation/current_waypoint` (PoseStamped)
- `/navigation/status` (NavigationStatus)
- `/plan` (Path)

#### Standard ROS2 Topics (1):
- `/parameter_events` (ParameterEvent)

**Complete Service List (16+ services):**

#### Functional Services (4):
- `/state_machine/can_transition_to_autonomous` (Trigger)
- `/state_machine/health_check` (Trigger)
- `navigate_to_pose` (NavigateToPose Action)
- `navigation_integrity_check` (NavigationIntegrityCheck)

#### Parameter Management Services (12):
- Competition Bridge: 6 parameter services
- State Machine Bridge: 6 parameter services

## Data Flow Architecture

```
Hardware Sensors → Competition Bridge → Dashboard (WebSocket)
Mission Control ↔ Competition Bridge ↔ Systems
State Machine Bridge ↔ Mission Control
Safety Systems ↔ Competition Bridge
```

## How to Use

### 1. View the Diagrams
The PNG files can be opened with any image viewer:
```bash
# Simple view
eog "URC 2026 ROS2 Architecture.png"

# Or open with system default
xdg-open "URC 2026 ROS2 Detailed Architecture.png"
```

### 2. Monitor Live Data
```bash
# Start ROS2 environment
source /opt/ros/humble/setup.bash
source install/setup.bash

# Monitor specific topic
ros2 topic echo /state_machine/current_state

# Monitor hardware data
ros2 topic echo /hardware/imu

# Check topic bandwidth
ros2 topic hz /hardware/gps
```

### 3. View System Graph
```bash
# Visual ROS2 computation graph
ros2 run rqt_graph rqt_graph

# Topic monitor GUI
ros2 run rqt_topic rqt_topic
```

### 4. Dashboard Integration
- **WebSocket URL**: `ws://localhost:8080`
- **Dashboard URL**: `http://localhost:3000`
- Real-time telemetry streaming from Competition Bridge

## Regenerating Diagrams

If the ROS2 architecture changes, regenerate the diagrams:

```bash
# 1. Start ROS2 bridges
python3 src/bridges/competition_bridge.py &
python3 src/bridges/ros2_state_machine_bridge.py &
python3 src/bridges/ros2_mission_bridge.py &

# 2. Update the PlantUML files if needed
# Edit: ros2_architecture_diagram.puml
# Edit: ros2_architecture_detailed.puml

# 3. Regenerate PNGs
java -jar plantuml.jar ros2_architecture_diagram.puml
java -jar plantuml.jar ros2_architecture_detailed.puml
```

## Architecture Notes

### Competition Bridge
- **Role**: Central telemetry aggregation and command distribution
- **Subscribers**: 11 topics from hardware, mission, and safety systems
- **Publishers**: 5 topics for control and status
- **WebSocket**: Streams real-time data to dashboard
- **Services**: Parameter management

### State Machine Bridge
- **Role**: System state management and mission coordination
- **Subscribers**: 2 topics for state requests
- **Publishers**: 5 topics for state information
- **Services**: 2 functional services + parameter management
- **Integration**: Works with mission control for state transitions

### SLAM Node
- **Role**: RGB-D SLAM with RTAB-Map for mapping and localization
- **Subscribers**: Camera depth images, IMU data
- **Publishers**: Pose estimates, map data, system status
- **Integration**: Provides localization for navigation system

### Navigation Node
- **Role**: GNSS waypoint navigation with obstacle avoidance
- **Subscribers**: Odometry, IMU, GPS, state commands
- **Publishers**: Velocity commands, waypoint status, navigation progress
- **Services**: Navigate to pose action, integrity checks

### Map Data Bridge
- **Role**: Bridge SLAM map data to frontend visualization
- **Subscribers**: SLAM pose and map topics
- **Publishers**: Processed map data for WebSocket streaming
- **Integration**: Converts ROS map messages to frontend-compatible format

### Data Patterns
- **Sensor Data**: Hardware → Competition Bridge → Dashboard
- **SLAM Data**: Hardware → SLAM Node → Map Data Bridge → Competition Bridge → Dashboard
- **Navigation**: Hardware + State Machine → Navigation Node → Competition Bridge → Hardware
- **Commands**: Dashboard → Competition Bridge → Systems
- **State Management**: Mission Control ↔ State Machine Bridge
- **Safety**: Bidirectional emergency stop coordination
- **Map Visualization**: SLAM → Map Data Bridge → WebSocket → Dashboard

## File Structure
```
.
├── ros2_architecture_diagram.puml          # Simple diagram source
├── ros2_architecture_detailed.puml         # Detailed diagram source
├── URC 2026 ROS2 Architecture.png          # Simple diagram image
├── URC 2026 ROS2 Detailed Architecture.png # Detailed diagram image
└── ROS2_ARCHITECTURE_DIAGRAMS_README.md    # This documentation
```

## System Statistics
- **Total ROS2 Nodes**: 5 active nodes (Competition, State Machine, SLAM, Navigation, Map Data bridges)
- **Total Topics**: 35+ active topics (hardware, mission, safety, SLAM, navigation)
- **Total Services**: 16+ active services (state management, navigation, parameters)
- **Communication Patterns**: 25+ topic connections + WebSocket streaming
- **QoS Profiles**: Mixed (reliable for safety/critical, best-effort for sensors/high-frequency)

---

*Updated on: December 17, 2025*
*Now includes SLAM and Navigation components*
*URC 2026 Mars Rover Autonomy System*
