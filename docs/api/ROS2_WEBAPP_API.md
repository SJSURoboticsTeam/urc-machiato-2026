# ROS2 Webapp Integration API Specification

**Version:** 1.0.0  
**Last Updated:** 2026-01-30  
**Target Audience:** Frontend/Webapp Developers

This document specifies how to interface with the URC 2026 Mars Rover ROS2 systems from web applications. It covers topics, services, actions, blackboard access, and communication protocols.

---

## Table of Contents

1. [Communication Protocols](#1-communication-protocols)
2. [System State Management](#2-system-state-management)
3. [Navigation & SLAM](#3-navigation--slam)
4. [Sensor Data](#4-sensor-data)
5. [Camera & Vision](#5-camera--vision)
6. [Safety Systems](#6-safety-systems)
7. [Mission Execution](#7-mission-execution)
8. [Blackboard System](#8-blackboard-system)
9. [Teleoperation](#9-teleoperation)
10. [Monitoring & Diagnostics](#10-monitoring--diagnostics)

---

## 1. Communication Protocols

### Connection Methods

| Protocol | Port | URL | Purpose | Library |
|----------|------|-----|---------|---------|
| ROSBridge WebSocket | 9090 | `ws://localhost:9090` | Direct ROS2 topic/service access | `roslibjs` |
| Socket.IO | 4000 | `http://localhost:4000` | Teleoperation commands & status | `socket.io-client` |
| REST API | 8080 | `http://localhost:8080/api/*` | Dashboard data & emergency controls | `fetch` / `axios` |

### ROSBridge Quick Start

```javascript
import ROSLIB from 'roslib';

const ros = new ROSLIB.Ros({ url: 'ws://localhost:9090' });

// Subscribe to a topic
const topic = new ROSLIB.Topic({
  ros: ros,
  name: '/system/state',
  messageType: 'std_msgs/String'
});

topic.subscribe((message) => {
  const state = JSON.parse(message.data);
  console.log('System state:', state);
});

// Call a service
const service = new ROSLIB.Service({
  ros: ros,
  name: '/system/get_state',
  serviceType: 'autonomy_interfaces/srv/GetSystemState'
});

service.callService(new ROSLIB.ServiceRequest({}), (result) => {
  console.log('Current state:', result.current_state);
});
```

---

## 2. System State Management

### Topics

| Topic | Message Type | Direction | Rate | Description |
|-------|--------------|-----------|------|-------------|
| `/system/state` | `std_msgs/String` | Subscribe | 1 Hz | JSON-encoded system state |
| `/adaptive_state_machine/state` | `autonomy_interfaces/SystemState` | Subscribe | 10 Hz | Full hierarchical state |
| `/adaptive_state_machine/context` | `autonomy_interfaces/ContextUpdate` | Subscribe | 2 Hz | Simplified context for dashboards |
| `/adaptive_state_machine/transition` | `autonomy_interfaces/StateTransition` | Subscribe | Event | State transition events |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/system/get_state` | `GetSystemState` | Query current system state with history |
| `/system/set_state` | `ChangeState` | Request state transition |
| `/adaptive_state_machine/get_context` | `GetContext` | Get current context (battery, mission, etc.) |
| `/adaptive_state_machine/switch_mode` | `SwitchMode` | Switch operating mode |

### System States

| State | Description | Valid Transitions To |
|-------|-------------|---------------------|
| `BOOT` | Initial boot state | `IDLE` |
| `IDLE` | Ready but inactive | `AUTONOMOUS`, `TELEOPERATION`, `SHUTDOWN` |
| `AUTONOMOUS` | Autonomous operation | `IDLE`, `EMERGENCY_STOP`, `ERROR` |
| `TELEOPERATION` | Manual control | `IDLE`, `EMERGENCY_STOP`, `ERROR` |
| `EMERGENCY_STOP` | Emergency stop active | `IDLE`, `SHUTDOWN` |
| `ERROR` | Error state | `IDLE`, `EMERGENCY_STOP` |
| `SHUTDOWN` | System shutdown | None |

### SystemState Message Fields

| Field | Type | Description |
|-------|------|-------------|
| `current_state` | string | Top-level state (BOOT, IDLE, NAVIGATION, etc.) |
| `substate` | string | Mission-specific substate |
| `sub_substate` | string | Detailed substate |
| `time_in_state` | float64 | Seconds in current state |
| `is_transitioning` | bool | Currently transitioning |
| `active_subsystems` | string[] | List of active subsystems |
| `failed_subsystems` | string[] | List of failed subsystems |
| `adaptive_enabled` | bool | Adaptive system active |
| `active_adaptations` | string[] | Current adaptations |

### ContextUpdate Message Fields

| Field | Type | Description |
|-------|------|-------------|
| `battery_level` | float32 | Battery percentage (0-100) |
| `mission_status` | string | Current mission status |
| `mission_progress` | float32 | Mission progress (0.0-1.0) |
| `communication_active` | bool | Communication link active |
| `safety_active` | bool | Safety system active |
| `active_adaptations` | string[] | Current adaptations |
| `alert_level` | string | NONE, WARNING, CRITICAL |
| `available_actions` | string[] | Actions available in current state |

---

## 3. Navigation & SLAM

### Topics

| Topic | Message Type | Direction | Rate | Description |
|-------|--------------|-----------|------|-------------|
| `/odom` | `nav_msgs/Odometry` | Subscribe | 50 Hz | Wheel odometry |
| `/slam/pose/fused` | `geometry_msgs/PoseStamped` | Subscribe | 30 Hz | Fused SLAM pose |
| `/slam/status` | `autonomy_interfaces/SlamStatus` | Subscribe | 5 Hz | SLAM subsystem status |
| `/navigation/status` | `autonomy_interfaces/NavigationStatus` | Subscribe | 10 Hz | Navigation status |
| `/navigation/goal` | `geometry_msgs/PoseStamped` | Publish | Command | Set navigation goal |
| `/cmd_vel` | `geometry_msgs/Twist` | Publish | Command | Velocity commands |
| `/path` | `nav_msgs/Path` | Subscribe | Event | Current planned path |
| `/local_map` | `nav_msgs/OccupancyGrid` | Subscribe | 1 Hz | Local costmap |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/navigation/integrity_check` | `NavigationIntegrityCheck` | Check navigation system health |
| `/subsystem/status` | `GetSubsystemStatus` | Query navigation/SLAM status |

### Actions

| Action | Type | Description |
|--------|------|-------------|
| `/navigate_to_pose` | `NavigateToPose` | Navigate to target pose |

### NavigationStatus Message Fields

| Field | Type | Description |
|-------|------|-------------|
| `state` | string | idle, planning, navigating, avoiding_obstacle, precision_approach, arrived, error |
| `mission_progress` | float32 | Progress (0.0-1.0) |
| `current_waypoint` | int32 | Current waypoint index |
| `total_waypoints` | int32 | Total waypoint count |
| `current_pose` | PoseStamped | Current robot pose |
| `goal_pose` | PoseStamped | Target pose |
| `distance_to_goal` | float32 | Distance remaining (meters) |
| `speed` | float32 | Current speed (m/s) |
| `heading_error` | float32 | Heading error (radians) |

### SlamStatus Message Fields

| Field | Type | Description |
|-------|------|-------------|
| `state` | string | initializing, tracking, lost, relocalizing, mapping |
| `pose` | PoseWithCovarianceStamped | Current pose with covariance |
| `tracking_quality` | float32 | Tracking quality (0.0-1.0) |
| `loop_closure_detected` | bool | Loop closure detected |
| `keyframes_tracked` | int32 | Number of keyframes |
| `landmarks_tracked` | int32 | Number of landmarks |
| `drift_estimate` | float32 | Estimated drift (meters) |

---

## 4. Sensor Data

### Topics

| Topic | Message Type | Direction | Rate | Description |
|-------|--------------|-----------|------|-------------|
| `/imu/data` | `sensor_msgs/Imu` | Subscribe | 100 Hz | IMU data (orientation, angular velocity, linear acceleration) |
| `/gps/fix` | `sensor_msgs/NavSatFix` | Subscribe | 10 Hz | GPS fix (latitude, longitude, altitude) |
| `/battery/state` | `sensor_msgs/BatteryState` | Subscribe | 1 Hz | Battery state |
| `/temperature` | `sensor_msgs/Temperature` | Subscribe | 1 Hz | System temperature |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Subscribe | 1 Hz | System diagnostics |

### IMU Data Fields

| Field | Type | Description |
|-------|------|-------------|
| `orientation` | Quaternion | Orientation (x, y, z, w) |
| `angular_velocity` | Vector3 | Angular velocity (rad/s) |
| `linear_acceleration` | Vector3 | Linear acceleration (m/s^2) |

### BatteryState Fields

| Field | Type | Description |
|-------|------|-------------|
| `voltage` | float32 | Battery voltage (V) |
| `current` | float32 | Current draw (A) |
| `percentage` | float32 | Charge percentage (0.0-1.0) |
| `power_supply_status` | uint8 | 0=UNKNOWN, 1=CHARGING, 2=DISCHARGING, 3=NOT_CHARGING, 4=FULL |

---

## 5. Camera & Vision

### Topics

| Topic | Message Type | Direction | Rate | Description |
|-------|--------------|-----------|------|-------------|
| `/camera/front/image_raw` | `sensor_msgs/Image` | Subscribe | 30 Hz | Front camera raw image |
| `/camera/front/image_compressed` | `sensor_msgs/CompressedImage` | Subscribe | 30 Hz | Front camera compressed (JPEG) |
| `/camera/rear/image_compressed` | `sensor_msgs/CompressedImage` | Subscribe | 30 Hz | Rear camera compressed |
| `/camera/left/image_compressed` | `sensor_msgs/CompressedImage` | Subscribe | 30 Hz | Left camera compressed |
| `/camera/right/image_compressed` | `sensor_msgs/CompressedImage` | Subscribe | 30 Hz | Right camera compressed |
| `/camera/depth` | `sensor_msgs/Image` | Subscribe | 30 Hz | Depth image |
| `/camera/pointcloud` | `sensor_msgs/PointCloud2` | Subscribe | 10 Hz | 3D point cloud |
| `/vision/detections` | `autonomy_interfaces/VisionDetection` | Subscribe | 10 Hz | Object detections |
| `/camera/status` | `autonomy_interfaces/CameraCommand` | Subscribe | 1 Hz | Camera status |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/aruco/detect` | `DetectAruco` | Detect ArUco tags |
| `/aruco/detect_mission` | `DetectMissionAruco` | Mission-specific ArUco detection |
| `/camera/calibrate` | `CalibrateCamera` | Calibrate camera |
| `/camera/validate_calibration` | `ValidateCalibration` | Validate calibration quality |

### VisionDetection Message Fields

| Field | Type | Description |
|-------|------|-------------|
| `class_name` | string | Detected object class |
| `class_id` | int32 | Class identifier |
| `confidence` | float32 | Detection confidence (0.0-1.0) |
| `pose` | PoseStamped | 3D pose of detection |
| `size` | Vector3 | Bounding box size (meters) |
| `track_id` | int32 | Tracking ID (-1 if not tracked) |
| `age` | float32 | Track age (seconds) |

### CameraCommand Fields (for PTZ control)

| Field | Type | Description |
|-------|------|-------------|
| `command_type` | uint8 | 0=absolute, 1=relative, 2=track_target, 3=scan_pattern |
| `pan_angle` | float32 | Pan angle (radians) |
| `tilt_angle` | float32 | Tilt angle (radians) |
| `zoom_level` | float32 | Zoom level (1.0-10.0) |
| `autofocus` | bool | Enable autofocus |

---

## 6. Safety Systems

### Topics

| Topic | Message Type | Direction | Rate | Description |
|-------|--------------|-----------|------|-------------|
| `/safety/status` | `autonomy_interfaces/SafetyStatus` | Subscribe | 10 Hz | Safety system status |
| `/safety/alerts` | `autonomy_interfaces/SafetyAlert` | Subscribe | Event | Safety alerts |
| `/emergency_stop` | `std_msgs/Bool` | Publish | Command | Trigger emergency stop |
| `/emergency_stop_active` | `std_msgs/Bool` | Subscribe | 10 Hz | Emergency stop state |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/safety/get_status` | `GetSafetyStatus` | Get comprehensive safety status |
| `/safety/verify_property` | `VerifySafetyProperty` | Verify specific safety property |
| `/safety/timing_check` | `TimingSafetyCheck` | Check timing constraints |
| `/safety/estop` | `SoftwareEstop` | Trigger software emergency stop |
| `/safety/safestop` | `SafestopControl` | Control safestop (engage/disengage) |
| `/safety/recover` | `RecoverFromSafety` | Initiate recovery from safety state |

### SafetyStatus Message Fields

| Field | Type | Description |
|-------|------|-------------|
| `is_safe` | bool | Overall safety status |
| `safety_level` | string | NORMAL, WARNING, CRITICAL, EMERGENCY |
| `active_triggers` | string[] | Currently active safety triggers |
| `trigger_type` | string | Type of active trigger |
| `trigger_description` | string | Human-readable trigger description |
| `requires_manual_intervention` | bool | Requires operator action |
| `can_auto_recover` | bool | Can auto-recover |
| `recovery_steps` | string[] | Steps to recover |
| `battery_level` | float64 | Battery percentage |
| `temperature` | float64 | System temperature (C) |
| `communication_ok` | bool | Communication link OK |
| `sensors_ok` | bool | Sensors functional |
| `degraded_capabilities` | string[] | Currently degraded capabilities |

### SafetyAlert Message Fields

| Field | Type | Description |
|-------|------|-------------|
| `property` | string | Safety property name |
| `severity` | string | INFO, WARNING, ERROR, CRITICAL |
| `details` | string | Alert details |
| `timestamp` | float64 | Unix timestamp |
| `acknowledged` | bool | Alert acknowledged |

---

## 7. Mission Execution

### Topics

| Topic | Message Type | Direction | Rate | Description |
|-------|--------------|-----------|------|-------------|
| `/mission/status` | `std_msgs/String` | Subscribe | 1 Hz | Mission status (JSON) |
| `/mission/progress` | `std_msgs/Float32` | Subscribe | 1 Hz | Mission progress (0.0-1.0) |
| `/follow_me/status` | `autonomy_interfaces/FollowMeStatus` | Subscribe | 10 Hz | Follow-me mode status |
| `/led/command` | `autonomy_interfaces/LedCommand` | Publish | Command | LED status control |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/mission/configure` | `ConfigureMission` | Configure mission parameters |
| `/follow_me/control` | `FollowMeControl` | Control follow-me mode |
| `/adaptation/history` | `GetAdaptationHistory` | Get adaptation decision history |

### Actions

| Action | Type | Description |
|--------|------|-------------|
| `/execute_mission` | `ExecuteMission` | Execute complete mission |
| `/perform_typing` | `PerformTyping` | Execute autonomous typing task |

### ExecuteMission Action

**Goal:**

| Field | Type | Description |
|-------|------|-------------|
| `mission_type` | string | delivery, sample_collection, follow_me, keyboard, waypoint |
| `mission_id` | string | Unique mission identifier |
| `waypoints` | string[] | Waypoint names or coordinates |
| `timeout` | float32 | Mission timeout (seconds) |

**Feedback:**

| Field | Type | Description |
|-------|------|-------------|
| `current_phase` | string | Current mission phase |
| `progress` | float32 | Overall progress (0.0-1.0) |
| `status_message` | string | Human-readable status |
| `waypoints_completed` | int32 | Waypoints completed |
| `estimated_time_remaining` | float32 | Time remaining (seconds) |

**Result:**

| Field | Type | Description |
|-------|------|-------------|
| `success` | bool | Mission completed successfully |
| `completion_status` | string | Final status message |
| `completed_tasks` | string[] | List of completed tasks |
| `total_time` | float32 | Total mission time (seconds) |
| `waypoints_visited` | int32 | Total waypoints visited |

### FollowMeStatus Message Fields

| Field | Type | Description |
|-------|------|-------------|
| `is_following` | bool | Currently following |
| `target_tag_id` | int32 | ArUco tag ID being followed |
| `target_distance` | float32 | Distance to target (meters) |
| `target_angle` | float32 | Angle to target (radians) |
| `safety_distance` | float32 | Configured safety distance |
| `safety_violation` | bool | Safety distance violated |
| `target_visible` | bool | Target currently visible |
| `current_speed` | float32 | Current following speed (m/s) |

### LedCommand Message Fields

| Field | Type | Description |
|-------|------|-------------|
| `status_code` | int32 | 0=off, 1=ready, 2=running, 3=error, 4=emergency, 5=success |
| `red` | float32 | Red intensity (0.0-1.0) |
| `green` | float32 | Green intensity (0.0-1.0) |
| `blue` | float32 | Blue intensity (0.0-1.0) |
| `pattern` | string | solid, blinking, pulsing, chasing |
| `frequency` | float32 | Blink frequency (Hz) |
| `priority` | int32 | 0=normal, 1=warning, 2=critical |

---

## 8. Blackboard System

The blackboard is a shared key-value store used by the behavior tree system. Access it via ROS2 services.

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/blackboard/get_value` | `GetBlackboardValue` | Read value from blackboard |
| `/blackboard/set_value` | `SetBlackboardValue` | Write value to blackboard |

### GetBlackboardValue Request/Response

**Request:**

| Field | Type | Description |
|-------|------|-------------|
| `key` | string | Blackboard key name |
| `value_type` | string | Optional: "bool", "int", "double", "string" |

**Response:**

| Field | Type | Description |
|-------|------|-------------|
| `success` | bool | Operation succeeded |
| `value` | string | Value as string (JSON for complex types) |
| `value_type` | string | Actual value type |
| `error_message` | string | Error message if failed |

### SetBlackboardValue Request/Response

**Request:**

| Field | Type | Description |
|-------|------|-------------|
| `key` | string | Blackboard key name |
| `value` | string | Value as string |
| `value_type` | string | Value type: "bool", "int", "double", "string" |

**Response:**

| Field | Type | Description |
|-------|------|-------------|
| `success` | bool | Operation succeeded |
| `error_message` | string | Error message if failed |

### Blackboard Keys Reference

#### Robot State

| Key | Type | Description |
|-----|------|-------------|
| `robot_x` | double | Robot X position (meters) |
| `robot_y` | double | Robot Y position (meters) |
| `robot_yaw` | double | Robot heading (radians) |
| `robot_velocity_x` | double | X velocity (m/s) |
| `robot_velocity_y` | double | Y velocity (m/s) |

#### SLAM State

| Key | Type | Description |
|-----|------|-------------|
| `slam_x` | double | SLAM X position (meters) |
| `slam_y` | double | SLAM Y position (meters) |
| `slam_confidence` | double | SLAM confidence (0.0-1.0) |

#### Navigation State

| Key | Type | Description |
|-----|------|-------------|
| `navigation_status` | string | idle, navigating, arrived, failed |
| `path_clear` | bool | Path is clear of obstacles |
| `current_waypoint_index` | int | Current waypoint index |
| `nav_target_x` | double | Navigation target X |
| `nav_target_y` | double | Navigation target Y |
| `distance_to_target` | double | Distance to target (meters) |

#### Perception State

| Key | Type | Description |
|-----|------|-------------|
| `perception_confidence` | double | Overall perception confidence (0.0-1.0) |
| `map_quality` | double | Map quality score (0.0-1.0) |
| `feature_count` | int | Number of tracked features |
| `obstacle_detection_confidence` | double | Obstacle detection confidence |

#### Obstacle Detection

| Key | Type | Description |
|-----|------|-------------|
| `closest_obstacle_distance` | double | Distance to closest obstacle (meters) |
| `obstacle_detected` | bool | Obstacle detected in path |
| `proximity_violation_distance` | double | Proximity violation distance |

#### Mission State

| Key | Type | Description |
|-----|------|-------------|
| `mission_active` | bool | Mission currently active |
| `samples_collected` | int | Samples collected count |
| `waypoints_completed` | int | Waypoints completed count |
| `current_mission_phase` | string | Current mission phase |

#### Safety State

| Key | Type | Description |
|-----|------|-------------|
| `safety_stop_active` | bool | Safety stop active |
| `emergency_stop_active` | bool | Emergency stop active |
| `battery_level` | double | Battery level (0.0-100.0) |

#### System Health

| Key | Type | Description |
|-----|------|-------------|
| `sensors_ok` | bool | All sensors operational |
| `navigation_ok` | bool | Navigation system OK |
| `last_error` | string | Last error message |

---

## 9. Teleoperation

### Socket.IO Events (Port 4000)

#### Client -> Server (Commands)

| Event | Payload | Description |
|-------|---------|-------------|
| `driveCommands` | `{xVel: float, yVel: float, rotVel: float}` | Drive command (rotVel in deg/s) |
| `driveHoming` | `{}` | Request homing sequence |
| `emergencyStop` | `{}` | Trigger emergency stop |

#### Server -> Client (Status)

| Event | Payload | Description |
|-------|---------|-------------|
| `systemStatus` | See below | Periodic system status update |
| `connected` | `{}` | Connection established |
| `error` | `{message: string}` | Error notification |

### systemStatus Payload

```json
{
  "timestamp": 1706659200.0,
  "system_state": "TELEOPERATION",
  "velocity": {
    "linear": {"x": 0.5, "y": 0.0, "z": 0.0},
    "angular": {"x": 0.0, "y": 0.0, "z": 0.1}
  },
  "battery": {
    "voltage": 24.5,
    "current": 5.2,
    "percentage": 0.85
  },
  "imu": {
    "orientation": {"x": 0, "y": 0, "z": 0, "w": 1},
    "angular_velocity": {"x": 0, "y": 0, "z": 0.1},
    "linear_acceleration": {"x": 0.1, "y": 0, "z": 9.8}
  },
  "diagnostics": {
    "level": 0,
    "name": "system",
    "message": "OK"
  },
  "emergency_stop": false,
  "bridge_stats": {
    "messages_sent": 1234,
    "messages_received": 5678,
    "uptime_seconds": 3600
  }
}
```

### REST API Endpoints

| Method | Endpoint | Description |
|--------|----------|-------------|
| GET | `/api/dashboard` | Get full dashboard data |
| POST | `/api/emergency/stop` | Trigger emergency stop |
| POST | `/api/recovery/initiate` | Initiate recovery |
| GET | `/api/alerts/acknowledge/{index}` | Acknowledge alert by index |

---

## 10. Monitoring & Diagnostics

### Topics

| Topic | Message Type | Direction | Rate | Description |
|-------|--------------|-----------|------|-------------|
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Subscribe | 1 Hz | System diagnostics |
| `/aoi/status` | `autonomy_interfaces/AOIStatus` | Subscribe | 1 Hz | Age of Information status |
| `/aoi/metrics` | `autonomy_interfaces/AOIMetrics` | Subscribe | 1 Hz | System-wide AoI metrics |
| `/qos/profile` | `autonomy_interfaces/QoSTopicProfile` | Subscribe | 1 Hz | QoS profile per topic |
| `/qos/network` | `autonomy_interfaces/QoSNetworkStats` | Subscribe | 1 Hz | Network QoS statistics |

### Services

| Service | Type | Description |
|---------|------|-------------|
| `/aoi/get_status` | `GetAOIStatus` | Query AoI for specific sensor |
| `/aoi/get_network_status` | `GetNetworkAOIStatus` | Query network-aware AoI |
| `/qos/get_profile` | `GetQoSProfile` | Get current QoS profiles |
| `/network/get_stats` | `GetNetworkStats` | Get network statistics |

### AOIMetrics Message Fields (System Health)

| Field | Type | Description |
|-------|------|-------------|
| `system_average_aoi` | float64 | Average Age of Information (seconds) |
| `total_sensors` | uint32 | Total sensor count |
| `fresh_sensors` | uint32 | Sensors with fresh data |
| `stale_sensors` | uint32 | Sensors with stale data |
| `critical_sensors` | uint32 | Sensors in critical state |
| `system_healthy` | bool | Overall system health |
| `health_score` | float64 | Health score (0.0-1.0) |
| `health_status` | string | HEALTHY, DEGRADED, CRITICAL |
| `dropped_updates` | uint32 | Dropped update count |
| `processing_latency` | float64 | Processing latency (seconds) |
| `active_alerts` | string[] | Active alert messages |

### QoSNetworkStats Message Fields

| Field | Type | Description |
|-------|------|-------------|
| `bandwidth_up_mbps` | float64 | Upload bandwidth (Mbps) |
| `bandwidth_down_mbps` | float64 | Download bandwidth (Mbps) |
| `latency_ms` | float64 | Network latency (ms) |
| `packet_loss_rate` | float64 | Packet loss rate (0.0-1.0) |
| `current_band` | string | Current network band |
| `signal_strength` | float64 | Signal strength (dBm) |

---

## Appendix A: Message Type Quick Reference

### Standard ROS2 Messages Used

| Message | Package | Common Fields |
|---------|---------|---------------|
| `Twist` | `geometry_msgs` | linear.x/y/z, angular.x/y/z |
| `PoseStamped` | `geometry_msgs` | header, pose.position, pose.orientation |
| `Odometry` | `nav_msgs` | header, pose, twist |
| `Path` | `nav_msgs` | header, poses[] |
| `OccupancyGrid` | `nav_msgs` | header, info, data[] |
| `Image` | `sensor_msgs` | header, height, width, encoding, data |
| `CompressedImage` | `sensor_msgs` | header, format, data |
| `Imu` | `sensor_msgs` | header, orientation, angular_velocity, linear_acceleration |
| `NavSatFix` | `sensor_msgs` | header, latitude, longitude, altitude |
| `BatteryState` | `sensor_msgs` | header, voltage, current, percentage |

---

## Appendix B: Error Codes

| Code | Name | Description |
|------|------|-------------|
| 0 | SUCCESS | Operation completed successfully |
| 1 | INVALID_STATE | Invalid state for requested operation |
| 2 | INVALID_TRANSITION | State transition not allowed |
| 3 | PRECONDITION_FAILED | Preconditions not met |
| 4 | TIMEOUT | Operation timed out |
| 5 | HARDWARE_ERROR | Hardware communication error |
| 6 | SAFETY_VIOLATION | Safety constraint violated |
| 7 | MISSION_ABORTED | Mission aborted |
| 8 | NAVIGATION_FAILED | Navigation failed |
| 9 | PERCEPTION_ERROR | Perception system error |
| 10 | BLACKBOARD_ERROR | Blackboard access error |

---

## Appendix C: JavaScript/TypeScript Helper Functions

```typescript
// Type definitions for common messages
interface SystemState {
  current_state: string;
  substate: string;
  time_in_state: number;
  is_transitioning: boolean;
  active_subsystems: string[];
  failed_subsystems: string[];
}

interface SafetyStatus {
  is_safe: boolean;
  safety_level: 'NORMAL' | 'WARNING' | 'CRITICAL' | 'EMERGENCY';
  active_triggers: string[];
  requires_manual_intervention: boolean;
  recovery_steps: string[];
}

interface NavigationStatus {
  state: 'idle' | 'planning' | 'navigating' | 'avoiding_obstacle' | 'arrived' | 'error';
  mission_progress: number;
  distance_to_goal: number;
  speed: number;
}

// Blackboard helper
async function getBlackboardValue(ros: ROSLIB.Ros, key: string): Promise<string | null> {
  return new Promise((resolve, reject) => {
    const service = new ROSLIB.Service({
      ros,
      name: '/blackboard/get_value',
      serviceType: 'autonomy_interfaces/srv/GetBlackboardValue'
    });
    
    service.callService(
      new ROSLIB.ServiceRequest({ key, value_type: '' }),
      (result: { success: boolean; value: string; error_message: string }) => {
        if (result.success) {
          resolve(result.value);
        } else {
          reject(new Error(result.error_message));
        }
      }
    );
  });
}

// State subscription helper
function subscribeToState(ros: ROSLIB.Ros, callback: (state: SystemState) => void): ROSLIB.Topic {
  const topic = new ROSLIB.Topic({
    ros,
    name: '/adaptive_state_machine/state',
    messageType: 'autonomy_interfaces/msg/SystemState'
  });
  
  topic.subscribe(callback);
  return topic;
}
```

---

## Appendix D: Example Integration Code

### React Hook for ROS2 Connection

```typescript
import { useState, useEffect, useCallback } from 'react';
import ROSLIB from 'roslib';

export function useROS(url = 'ws://localhost:9090') {
  const [ros, setRos] = useState<ROSLIB.Ros | null>(null);
  const [connected, setConnected] = useState(false);
  const [error, setError] = useState<string | null>(null);

  useEffect(() => {
    const rosInstance = new ROSLIB.Ros({ url });

    rosInstance.on('connection', () => {
      setConnected(true);
      setError(null);
    });

    rosInstance.on('error', (err) => {
      setError(err.message);
    });

    rosInstance.on('close', () => {
      setConnected(false);
    });

    setRos(rosInstance);

    return () => {
      rosInstance.close();
    };
  }, [url]);

  const callService = useCallback(
    <T, R>(name: string, serviceType: string, request: T): Promise<R> => {
      return new Promise((resolve, reject) => {
        if (!ros) {
          reject(new Error('ROS not connected'));
          return;
        }

        const service = new ROSLIB.Service({ ros, name, serviceType });
        service.callService(new ROSLIB.ServiceRequest(request), resolve, reject);
      });
    },
    [ros]
  );

  return { ros, connected, error, callService };
}
```

### Dashboard Data Polling Example

```typescript
function useDashboardData() {
  const { ros, connected } = useROS();
  const [state, setState] = useState<SystemState | null>(null);
  const [safety, setSafety] = useState<SafetyStatus | null>(null);
  const [navigation, setNavigation] = useState<NavigationStatus | null>(null);

  useEffect(() => {
    if (!ros || !connected) return;

    const stateTopic = new ROSLIB.Topic({
      ros,
      name: '/adaptive_state_machine/state',
      messageType: 'autonomy_interfaces/msg/SystemState'
    });

    const safetyTopic = new ROSLIB.Topic({
      ros,
      name: '/safety/status',
      messageType: 'autonomy_interfaces/msg/SafetyStatus'
    });

    const navTopic = new ROSLIB.Topic({
      ros,
      name: '/navigation/status',
      messageType: 'autonomy_interfaces/msg/NavigationStatus'
    });

    stateTopic.subscribe(setState);
    safetyTopic.subscribe(setSafety);
    navTopic.subscribe(setNavigation);

    return () => {
      stateTopic.unsubscribe();
      safetyTopic.unsubscribe();
      navTopic.unsubscribe();
    };
  }, [ros, connected]);

  return { state, safety, navigation };
}
```

---

*For additional support, see the ROS2 documentation and the project's internal developer guides.*
