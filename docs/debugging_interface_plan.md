# Enhanced Debugging Interface - Implementation Plan

URC 2026 Mars Rover - Comprehensive sensor, blackboard, and behavior tree debugging.

## Overview

Four-phase implementation delivering a unified debugging dashboard with real-time sensor health, blackboard visualization, behavior tree analytics, and time-synced incident replay. Target: reduce time to identify sensor issues by 50%, full coverage of 26+ blackboard keys and 9 sensor types, BT bottleneck identification with <100ms accuracy.

---

## Phase 1: Enhanced Sensor Debugging Interface

### 1.1 Real-Time Sensor Health Dashboard

| Feature | Description | Data Source |
|---------|-------------|-------------|
| Grid status | 9 sensors: health score (0-1), last update time, confidence | `/debugging/sensor_analytics` or `/safety/sensor_health` |
| Time-series charts | Configurable window (30s / 1m / 5m), one chart per sensor or overlay | Same topic, client-side buffer |
| Alerts | Staleness (time since last update), confidence thresholds (<0.5 red, 0.5-0.8 yellow, 0.8+ green), failure root cause | Derived from sensor_analytics |
| Confidence viz | Color-coded indicators: green >=0.8, yellow 0.5-0.8, red <0.5 | Same |

**Sensors (9):** IMU, GPS, Camera, Lidar, SLAM pose, Odom, Battery, Proximity, + one (e.g. Vision or Motor temp) to align with `SensorId` and blackboard hardware keys.

### 1.2 Advanced Sensor Analytics

| Feature | Description |
|---------|-------------|
| Noise analysis | Std dev, outlier count, signal quality metric per sensor |
| Sensor fusion confidence | Visualize contribution of each sensor to fused output (e.g. pose) |
| Correlation matrix | Cross-sensor correlation (e.g. IMU vs Odom) |
| Performance metrics | Update frequency (Hz), latency (ms), success rate (%) |

**Components:** `SensorHealthGrid.jsx`, `SensorTimeSeries.jsx`, `SensorAnalytics.jsx` (noise, correlation, performance).

---

## Phase 2: Comprehensive Blackboard Visualization

### 2.1 Real-Time Blackboard Monitor

| Feature | Description |
|---------|-------------|
| Categorized keys | Group 26+ keys by domain: sensors, navigation, safety, system, mission, perception |
| Change tracking | Highlight recently changed keys, optional value history mini-chart per key |
| Type-aware display | GPS (lat/lon/alt), quaternions, bool, int, double, string with appropriate widgets |
| Search & filter | Text search and domain filter for quick access |

**Data:** ROS service `/blackboard/get_value` (existing); new topic `/blackboard/snapshot` or polling for real-time view; optional `/blackboard/history` for timeline.

### 2.2 Historical Blackboard Analysis

| Feature | Description |
|---------|-------------|
| Timeline view | Select time point, show blackboard state at that time |
| Change patterns | Which keys change most in which mission phase |
| Anomaly detection | Flag unusual value ranges or correlations |
| Export | JSON/CSV of blackboard state or session for offline analysis |

**Components:** `BlackboardRealtime.jsx`, `BlackboardHistory.jsx`, `KeyAnalytics.jsx`.

---

## Phase 3: Advanced Behavior Tree Analytics

### 3.1 Full BT Execution Visualization

| Feature | Description |
|---------|-------------|
| Node timing | Execution duration per node, identify bottlenecks |
| Success/failure patterns | Heat map: nodes that fail under certain conditions |
| Execution path history | Alternative paths taken in different scenarios |
| Resource usage | Memory/CPU per BT node (if exposed by backend) |

### 3.2 BT-Blackboard Integration

| Feature | Description |
|---------|-------------|
| Data dependency mapping | Which nodes read/write which blackboard keys |
| Causality | Trace how blackboard changes affect BT decisions |
| State correlation | Sensor data vs BT node outcomes |

**Data:** New topics e.g. `/debugging/bt_performance`, `/debugging/bt_tick` or existing BT telemetry; blackboard keys from Phase 2.

**Components:** `BTExecutionView.jsx`, `BTPerformanceChart.jsx`, `BTBlackboardLink.jsx`.

---

## Phase 4: Unified Debugging Interface

### 4.1 Integrated Debugging Dashboard

| Feature | Description |
|---------|-------------|
| Tabbed UI | Tabs: Sensor Health, Blackboard, Behavior Tree, System Analytics |
| Time-synced views | Single timeline; all panels show state at selected time |
| Incident replay | Step through a time range with full state reconstruction |
| Export & sharing | Save debugging session (state + events) for team review |

### 4.2 Performance Optimization Tools

| Feature | Description |
|---------|-------------|
| Bottleneck identification | Auto-identify issues across sensors, BT, blackboard |
| Resource monitoring | CPU, memory, network impact on sensor processing |
| Recommendation engine | Suggest config changes based on analysis |

**Components:** `DebuggingDashboard.jsx`, `IncidentReplayer.jsx`, `PerformanceAnalyzer.jsx`.

---

## Component Architecture

```
src/dashboard/src/components/debugging/
  SensorDashboard/
    SensorHealthGrid.jsx      # 9-sensor grid, health/confidence/last update
    SensorTimeSeries.jsx       # Time-series charts, configurable window
    SensorAnalytics.jsx        # Noise, correlation matrix, performance metrics
  BlackboardViewer/
    BlackboardRealtime.jsx     # Categorized keys, type-aware, search/filter
    BlackboardHistory.jsx      # Timeline, change patterns, export
    KeyAnalytics.jsx           # Per-key stats, anomaly flags
  BehaviorTreeAnalytics/
    BTExecutionView.jsx        # Node tree + timing, success/failure
    BTPerformanceChart.jsx     # Duration/latency charts per node
    BTBlackboardLink.jsx       # Node-key read/write mapping, causality
  UnifiedDebugging/
    DebuggingDashboard.jsx     # Tabbed container, time sync
    IncidentReplayer.jsx       # Time scrubber, state replay
    PerformanceAnalyzer.jsx   # Bottlenecks, resources, recommendations
  index.js                     # Public exports
  constants.js                 # Sensor IDs, key categories, thresholds
```

---

## Data Integration

### ROS Topics (extend `rosTopics.js`)

```js
// Debugging topics (new)
DEBUGGING_TOPICS = {
  SENSOR_ANALYTICS: '/debugging/sensor_analytics',   // health, confidence, metrics
  BT_PERFORMANCE: '/debugging/bt_performance',       // node timing, outcomes
  BLACKBOARD_HISTORY: '/blackboard/history',         // optional: snapshot stream
  SYSTEM_ANALYTICS: '/debugging/system_analytics'    // CPU, memory, recommendations
}
```

### Services (existing)

- `GetBlackboardValue` (/blackboard/get_value)
- `SetBlackboardValue` (/blackboard/set_value)
- Optional: `SaveBlackboardSnapshot`, `LoadBlackboardSnapshot` for export/import

### WebSocket / Caching

- **Real-time:** Subscribe to `/debugging/sensor_analytics`, `/debugging/bt_performance`; differential updates where possible.
- **Historical:** Client-side ring buffer (e.g. last 5–10 min) for sensor and BT data; optional server-side aggregation for longer sessions.
- **Blackboard:** Poll or subscribe to snapshot topic; cache last snapshot and diff for "changed" highlighting.

---

## Backend Additions (out of scope for frontend-only)

- **ROS node** publishing `/debugging/sensor_analytics`: aggregate from `SensorHealthTracker`, add noise/outlier metrics if available.
- **ROS node** publishing `/debugging/bt_performance`: from BT orchestrator telemetry (node name, duration, success/failure, timestamp).
- **Blackboard history**: optional publisher or service that records key changes with timestamps for timeline/replay.

---

## Success Metrics

| Metric | Target |
|--------|--------|
| Debugging efficiency | 50% reduction in time to identify sensor issues |
| Coverage | All 26+ blackboard keys + 9 sensor types visible |
| BT timing accuracy | Bottleneck identification with <100ms accuracy |
| Collaboration | Shareable debugging sessions with full state |

---

## Implementation Order

1. **Plan + config:** This document, `DEBUGGING_TOPICS` in `rosTopics.js`, `constants.js` for sensors/keys.
2. **Phase 1:** Sensor dashboard (grid, time-series, analytics) with mock then ROS.
3. **Phase 2:** Blackboard realtime (service get_value + categories), then history + export.
4. **Phase 3:** BT execution view and performance charts (mock then ROS).
5. **Phase 4:** Unified dashboard (tabs, time sync), incident replayer, performance analyzer.
6. **Polish:** Tests (e.g. pytest for backend, Vitest for dashboard components), export/share, docs.
