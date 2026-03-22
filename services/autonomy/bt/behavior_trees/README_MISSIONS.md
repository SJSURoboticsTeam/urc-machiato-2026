# URC 2026 Mission Behavior Trees

This directory contains Behavior Tree (BT) definitions for each URC 2026 competition mission. Each mission has its own BT XML file that implements the required tasks and sequences.

## Mission Templates

### 1. Science Mission (`science_mission.xml`)
**Objective**: Collect samples, perform analysis, cache samples for return

**Key Tasks**:
- Navigate to investigation sites (GNSS waypoints)
- Document sites (panoramas, close-ups, stratigraphic profiles)
- Collect sub-surface samples (≥10cm depth, ≥5g mass)
- Perform onboard analysis (life detection, science capabilities)
- Cache samples for return (spill-proof, contamination-free)

**BT Nodes Used**:
- `NavigateToGNSS`, `NavigateToWaypoint`
- `SampleCollection`
- `SensorCheck` (imu, gps, camera)

### 2. Equipment Servicing Mission (`equipment_servicing_mission.xml`)
**Objective**: Perform dexterous operations on lander, autonomous typing

**Key Tasks**:
- Navigate to lander location
- Pick up sample tube and insert into cache
- Perform autonomous typing of launch key (3-6 characters)
- Connect USB cable and read memory card
- Connect fuel hose (cam lock fitting)
- Turn valve (¼-turn operation)

**BT Nodes Used**:
- `NavigateToWaypoint`
- `PickupObject`, `DeliverObject`
- `PerformTyping`
- `InsertCache`, `ConnectUSB`, `ConnectHose`, `TurnValve`

### 3. Autonomous Navigation Mission (`autonomous_navigation_mission.xml`)
**Objective**: Navigate autonomously to GNSS waypoints, AR tags, and detect objects

**Key Tasks**:
- Navigate to 2 GNSS-only waypoints (3m accuracy)
- Navigate to 2 AR tag posts (2m accuracy)
- Detect 3 objects: mallet, rock pick hammer, water bottle
- Signal arrivals with LED indicators (Red: autonomous, Blue: teleop, Green flash: arrival)
- Handle mission aborts and returns

**BT Nodes Used**:
- `NavigateToGNSS`
- `DetectArUcoPost`
- `DetectObject`
- `SignalArrival`

### 4. Delivery Mission (`delivery_mission.xml`)
**Objective**: Assist astronauts by picking up and delivering objects across terrain

**Key Tasks**:
- Navigate through marked terrain paths
- Find and pickup objects (tools, instruments, containers)
- Open boxes/toolboxes as needed
- Read signs held by astronauts
- Deliver objects to correct locations
- Search areas for equipment

**BT Nodes Used**:
- `NavigateToWaypoint`
- `PickupObject`, `DeliverObject`
- `ReadSign`
- `SearchArea`

## Main Mission (`main_mission.xml`)

The `main_mission.xml` is a basic sample collection mission used for testing. For competition, use the specific mission templates above.

## BT Node Reference

### Navigation Nodes
- `NavigateToWaypoint`: Navigate to local coordinates (x, y)
- `NavigateToGNSS`: Navigate to GNSS coordinates (lat, lon)

### Detection Nodes
- `DetectArUcoPost`: Detect AR tag posts
- `DetectObject`: Detect competition objects
- `ReadSign`: Read astronaut-held signs

### Manipulation Nodes
- `PickupObject`: Pick up objects
- `DeliverObject`: Deliver objects
- `InsertCache`: Insert samples into cache
- `PerformTyping`: Autonomous keyboard typing

### Equipment Nodes
- `ConnectUSB`: Connect USB cable
- `ConnectHose`: Connect hose (cam lock)
- `TurnValve`: Operate valves

### Utility Nodes
- `SampleCollection`: Collect soil samples
- `SensorCheck`: Check sensor health
- `SignalArrival`: Signal mission progress
- `SearchArea`: Search for targets
- `EmergencyStop`: Emergency stopping

## Usage

To use a specific mission:

1. Load the appropriate BT XML file in the BT orchestrator
2. Configure mission-specific parameters (GNSS coordinates, timeouts, etc.)
3. Execute the mission through the `/bt/execute_mission` action server

## Mission Execution Flow

All missions follow this general pattern:
1. **Pre-mission checks**: Sensor validation
2. **Navigation**: Move to mission areas
3. **Task execution**: Perform mission-specific operations
4. **Return**: Navigate back to start position
5. **Completion**: Signal mission success

## Configuration

Mission parameters are specified in the BT XML ports:
- GNSS coordinates for navigation targets
- Timeouts for operations
- Tolerance values for positioning
- Object types for detection/pickup

## Integration with State Machine

The BT system integrates with the adaptive state machine:
- Missions only execute when rover is in `AUTONOMOUS` state
- State machine triggers BT execution on state transitions
- BT reports progress and completion status
- Emergency stops coordinated between systems





