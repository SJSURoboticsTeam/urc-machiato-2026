# Blackboard System: Description, Rover Integration, and URC Alignment

The blackboard is the **centralized, unified state store** for the URC 2026 Mars Rover. It provides a single source of truth for robot state, mission progress, safety flags, and mode so that C++ (Behavior Tree), Python (missions, state machine, perception, navigation, safety), and persistence all share the same state.

---

## 1. What the Blackboard Is

| Aspect | Description |
|--------|-------------|
| **Role** | Key-value store for operational state; "shared memory" for the autonomy stack. |
| **Host** | C++ ROS2 lifecycle node `bt_orchestrator` (`src/autonomy/bt`), using BehaviorTree.CPP `Blackboard`. |
| **Access** | ROS2 services: `/blackboard/get_value`, `/blackboard/set_value` (GetBlackboardValue.srv, SetBlackboardValue.srv). |
| **Keys** | Defined once in C++ (`blackboard_keys.hpp`) and Python (`blackboard_keys.py`); same key strings in both. |
| **Types** | bool, int, double, string (serialized over services). |

---

## 2. Key Namespaces and Ownership

Keys are grouped by domain. Who writes them is noted below.

| Namespace | Keys (examples) | Updated by |
|-----------|-----------------|------------|
| **Robot state** | `robot_x`, `robot_y`, `robot_yaw`, `robot_velocity_x/y` | `bt_orchestrator` (odom callback) |
| **SLAM** | `slam_x`, `slam_y`, `slam_confidence` | `bt_orchestrator` (SLAM pose callback), `slam_node` (perception metrics) |
| **Navigation** | `navigation_status`, `path_clear`, `current_waypoint_index`, `nav_target_x/y`, `distance_to_target` | `navigation_node`, `bt_orchestrator` |
| **Perception** | `perception_confidence`, `map_quality`, `feature_count`, `obstacle_detection_confidence` | `slam_node` |
| **Obstacles** | `closest_obstacle_distance`, `obstacle_detected`, `proximity_violation_distance` | `proximity_monitor` |
| **Mission** | `mission_active`, `samples_collected`, `waypoints_completed`, `current_mission_phase` | BT nodes, mission logic, `bt_orchestrator` (init) |
| **Safety** | `safety_stop_active`, `emergency_stop_active`, `battery_level` | `safety_watchdog`, `bt_orchestrator`, `hardware_interface_node` (direct path) |
| **System** | `sensors_ok`, `navigation_ok`, `last_error` | Watchdogs, BT, error handlers |
| **Hybrid / mode** | `auto.mode`, `auto.mode_request`, `auto.human_override_active`, `auto.assist_enabled` | `adaptive_state_machine` |
| **System (dot)** | `system.emergency_stop`, `system.battery_percent`, `system.last_error` | `safety_watchdog`, `adaptive_state_machine`, `hardware_interface_node` (direct path) |
| **Hardware sensors** | `robot_velocity_x/y`, `battery_level`, `system.battery_percent`, `imu_linear_accel_*`, `imu_angular_vel_*`, `motor_temp_max`, `gps_latitude`, `gps_longitude`, `gps_altitude` | `hardware_interface_node` (direct CAN-to-blackboard path) |

Flat keys (e.g. `robot_x`, `mission_active`) and dot keys (e.g. `auto.mode`, `system.emergency_stop`) live in the same blackboard; naming is for clarity and consistency with URC/mode concepts.

---

## 3. How It Integrates with the Rover

### 3.1 Core Host: bt_orchestrator (C++)

- **Creates** the BehaviorTree.CPP `Blackboard` and initializes all keys to safe defaults on configure.
- **Subscribes** to `/odom`, SLAM/pose topics and **writes** robot/SLAM state into the blackboard.
- **Exposes** `/blackboard/get_value` and `/blackboard/set_value`; BT nodes read/write the same blackboard.
- **Publishes** telemetry (e.g. JSON) derived from blackboard reads so dashboards and logs see current state.

So: one process holds the canonical state; everyone else changes or reads it via services (or via BT inside that process).

### 3.2 Python Access: UnifiedBlackboardClient

- **Location:** `src/core/unified_blackboard_client.py`
- **Role:** Python API to the C++ blackboard via GetBlackboardValue / SetBlackboardValue.
- **Features:** Optional short TTL cache, type-safe get/set, timeout and default handling.
- **Used by:** Adaptive state machine, state management/BehaviorTree wrapper, perception, navigation, safety, persistence.

Any Python node that needs rover-wide state should use this client (and `BlackboardKeys`) instead of local dicts or separate state.

### 3.3 Rover Subsystems That Integrate with the Blackboard

| Subsystem | Component | Integration |
|-----------|------------|-------------|
| **Odometry / SLAM** | `bt_orchestrator` (C++), `slam_node` (Python) | Write `robot_*`, `slam_*`, `perception_*`, `map_quality`, `feature_count`. |
| **Navigation** | `navigation_node` | Writes `navigation_status`, `path_clear`, `distance_to_target`, `current_waypoint_index`, etc. |
| **Perception** | `slam_node` | Writes `slam_confidence`, `perception_confidence`, `map_quality`, `feature_count`. |
| **Obstacles / proximity** | `proximity_monitor` | Writes `closest_obstacle_distance`, `obstacle_detected`, `proximity_violation_distance`. |
| **Safety** | `safety_watchdog` | Writes `safety_stop_active`, `emergency_stop_active`, `battery_level`, `system.emergency_stop`, `system.battery_percent`. |
| **Hardware interface** | `hardware_interface_node` | Direct path: writes `battery_level`, `system.battery_percent`, `robot_velocity_x/y`, `emergency_stop_active`, `safety_stop_active`, `system.emergency_stop`; also `imu_linear_accel_*`, `imu_angular_vel_*`, `motor_temp_max`, `gps_latitude`, `gps_longitude`, `gps_altitude`. Keeps publishing to `/hardware/*` for backward compatibility. |
| **Mode / high-level behavior** | `adaptive_state_machine` | Reads/writes `auto.mode`, `auto.mode_request`, `auto.human_override_active`, `auto.assist_enabled`, `system.emergency_stop`; keeps BT and operator mode in sync. |
| **Missions / BT** | BT in `bt_orchestrator`, Python missions | BT nodes read/write mission and safety keys; Python missions can use `UnifiedBlackboardClient` for same keys. |
| **Persistence** | `BlackboardPersistenceNode` | Uses get/set services to save/load a subset of keys to/from JSON; does not overwrite `system.emergency_stop` on load. |

So the rover’s “brain” is: sensors and nodes push state into the blackboard; BT and adaptive state machine decide behavior from that state; safety and mode are shared so that teleop, autonomous, and emergency are consistent.

### 3.4 Data Flow: CAN to Blackboard (Dual Path)

Two paths deliver hardware sensor data to the blackboard:

- **Direct path (low latency):** CAN Bus -> `hardware_interface_node` -> `blackboard.set()` (via `UnifiedBlackboardClient`) -> BT.CPP Blackboard. Sensor data is written in the same process as the telemetry loop; target latency under 30 ms. The node continues to publish to `/hardware/*` so other subscribers are unchanged.

- **Topic path (fallback / compatibility):** CAN Bus -> `hardware_interface_node` -> `/hardware/battery_state`, `/hardware/chassis_velocity`, etc. -> `can_to_blackboard_bridge` -> `/battery/status`, `/odom` -> `safety_watchdog` and `bt_orchestrator` -> blackboard. This path remains active whenever the bridge and writers are running; useful for backward compatibility and redundancy.

Both paths can run at once (last writer wins per key). To verify direct writes: run `python3 scripts/hardware/validate_can_blackboard_direct.py` (checks source). To test with ROS2: run `hardware_interface_node` (mock or real CAN) and a blackboard host (e.g. bt_orchestrator or a mock service), then read keys via `/blackboard/get_value` or telemetry.

**Runbook (direct path):** Enable: `hardware_interface_node` uses `UnifiedBlackboardClient` by default when `core.unified_blackboard_client` and `core.blackboard_keys` are importable. Disable: Not applicable (optional client; if import fails, node runs without blackboard writes). Bridge: To disable the topic path, do not launch `can_to_blackboard_bridge`. Verify: `validate_can_blackboard_direct.py`; or call `ros2 service call /blackboard/get_value ...` for `battery_level`, `robot_velocity_x`, etc., after running the hardware interface node.

### 3.5 Persistence

- **Node:** `BlackboardPersistenceNode` (`src/core/blackboard_persistence.py`)
- **Services:** `/blackboard/save_snapshot`, `/blackboard/load_snapshot`
- **Persisted keys (conceptually):** Mission progress (`mission_active`, `samples_collected`, `waypoints_completed`, `current_mission_phase`, `current_waypoint_index`), `last_error`, `auto.mode`, `system.battery_percent`, `system.last_error`.
- **Safety:** On load, `system.emergency_stop` is never forced to false.

This supports recovery and restart without losing mission progress or overriding safety.

---

## 4. Alignment with URC (University Rover Challenge)

The blackboard is structured so that competition rules and judging needs map onto keys and consumers.

| URC concern | Blackboard support |
|-------------|--------------------|
| **Autonomous vs teleop** | `auto.mode` / `auto.mode_request` and `auto.human_override_active` drive who is in control; BT and state machine use the same keys so behavior and mode stay aligned. |
| **Emergency stop** | `emergency_stop_active`, `system.emergency_stop` (and related safety keys) are written by the safety watchdog and read by BT and state machine; one place to enforce e-stop. |
| **Mission progress** | `samples_collected`, `waypoints_completed`, `current_mission_phase`, `mission_active` track Sample Return and navigation tasks for scoring and replanning. |
| **Safety / reliability** | `battery_level` / `system.battery_percent`, `sensors_ok`, `navigation_ok`, `last_error` support pre-run checks and in-mission health; obstacle keys support safe navigation. |
| **Observability** | Single blackboard plus telemetry from `bt_orchestrator` gives dashboards and logs one place to show rover state for debugging and post-run analysis. |
| **Recovery / restarts** | Persistence of mission and mode (with safe handling of e-stop) supports reconnecting or restarting without losing progress or violating safety. |

In short: the blackboard is the place where “what the rover believes” (pose, mission, mode, safety) is stored and shared, which is exactly what URC requires for autonomous behavior, safety, and judging.

---

## 5. File and Interface Reference

| Item | Location |
|------|----------|
| C++ key constants | `src/autonomy/bt/include/autonomy_bt/blackboard_keys.hpp` |
| Python key constants | `src/core/blackboard_keys.py` |
| Blackboard host (C++) | `src/autonomy/bt/src/bt_orchestrator.cpp` |
| Python client | `src/core/unified_blackboard_client.py` |
| Persistence node | `src/core/blackboard_persistence.py` |
| Get service | `autonomy_interfaces/srv/GetBlackboardValue.srv` |
| Set service | `autonomy_interfaces/srv/SetBlackboardValue.srv` |

---

## 6. Summary

- **What:** Central key-value state store hosted in the C++ `bt_orchestrator`, accessed via ROS2 services.
- **Rover integration:** Odometry, SLAM, navigation, perception, proximity, and safety all write into it; BT and adaptive state machine read/write for behavior and mode; Python uses `UnifiedBlackboardClient`; persistence saves/loads a subset of keys safely.
- **URC alignment:** One place for mode (autonomous/teleop/emergency), e-stop, mission progress, and health so the rover meets competition rules and remains observable and recoverable.
