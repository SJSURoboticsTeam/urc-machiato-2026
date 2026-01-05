# 🤖 Autonomy Stack

This directory contains the core ROS2 packages for the rover's autonomous capabilities. The autonomy stack is organized into functional modules with clear interfaces.

## 📁 Package Structure

### 🎯 `bt/` - Behavior Trees
Mission decision logic using BT.CPP framework.

**Key Components:**
- `bt_orchestrator.cpp` - Main BT execution engine
- `behavior_trees/*.xml` - Mission-specific BT definitions
- Mission trees for sample collection, navigation, equipment service

**Usage:**
```bash
# List available behaviors
ros2 action list | grep bt

# Execute mission
ros2 action send_goal /bt/execute_mission autonomy_interfaces/action/ExecuteMission
```

### 🎮 `control/` - Hardware Control
Low-level hardware interfaces and control systems.

**Subpackages:**
- `hardware_interface/` - Generic hardware abstraction
- `led_status/` - LED status signaling (URC compliance)

**Key Topics:**
- `/hardware/cmd_vel` - Velocity commands
- `/hardware/led_command` - Status LED control
- `/hardware/sensor_data` - Raw sensor readings

### 🧠 `core/` - Core Autonomy
Fundamental autonomous capabilities.

**Subpackages:**
- `navigation/` - Path planning and waypoint following
- `state_management/` - System state and mode management
- `safety_system/` - Emergency response and safety monitors

**Core Nodes:**
- `navigation_node` - GPS + local navigation
- `state_machine` - Mission state coordination
- `safety_monitor` - System health watchdog

### 📡 `interfaces/` - ROS2 Communication
Shared message definitions and interfaces.

**Contents:**
- `msg/` - Data structures (SensorData, NavigationState)
- `srv/` - Request/response services (HealthCheck)
- `action/` - Long-running tasks (ExecuteMission, NavigateToWaypoint)

**Dependencies:** All other autonomy packages depend on this

### 👁️ `perception/` - Sensor Processing
Computer vision, SLAM, and sensor fusion.

**Subpackages:**
- `computer_vision/` - Object detection and scene understanding
- `slam/` - Real-time localization and mapping
- `sensor_bridge/` - IMU, camera, GPS data processing
- `autonomous_typing/` - Computer interaction vision

**Key Capabilities:**
- ArUco marker detection and pose estimation
- Point cloud processing for obstacle avoidance
- Multi-camera calibration and fusion

### 🔧 `utilities/` - Shared Code
Common utilities and helpers used across packages.

## 🚀 System Architecture

### Data Flow
```
Sensors → Perception → Planning → Control → Actuators
    ↑                                        ↓
Dashboard ← Bridges ← State ← Monitoring ← Health
```

### Key Integration Points
- **ROS2 Topics**: Real-time data sharing between packages
- **Behavior Trees**: High-level mission coordination
- **State Machine**: System mode management
- **Safety System**: Emergency response coordination

## 🛠️ Development Guide

### Adding New Autonomy Features

1. **Identify Package**: Choose appropriate subpackage or create new one
2. **Define Interface**: Add messages/services in `interfaces/`
3. **Implement Logic**: Create ROS2 node with proper error handling
4. **Add Configuration**: Update `config/rover.yaml`
5. **Write Tests**: Add unit and integration tests
6. **Update Documentation**: Add to relevant README and docs

### Package Dependencies
```xml
<!-- Example package.xml -->
<depend>rclpy</depend>
<depend>autonomy_interfaces</depend>
<depend>geometry_msgs</depend>
<depend>sensor_msgs</depend>
```

### Launch Integration
Add to `src/autonomy/launch/competition_system.launch.py`:
```python
Node(
    package='autonomy_navigation',
    executable='navigation_node',
    name='navigation'
)
```

## 🧪 Testing Strategy

### Unit Tests
```bash
# Test individual packages
colcon test --packages-select autonomy_navigation
colcon test-result --verbose
```

### Integration Tests
```bash
# Test package interactions
python -m pytest tests/integration/ -v
```

### Hardware-in-the-Loop
```bash
# Test with real hardware interfaces
python -m pytest tests/hardware/ -v
```

## 🔍 Key Configuration Files

### Core Parameters
Located in `config/rover.yaml`:
- Navigation tolerances and timeouts
- Safety system thresholds
- Perception processing parameters
- Control loop frequencies

### Package-Specific Config
Each package may have its own config files in `config/` subdirectory.

## 📊 Monitoring & Debugging

### Key Topics to Monitor
```bash
# System state
ros2 topic echo /autonomy/state

# Navigation status
ros2 topic echo /autonomy/navigation/status

# Safety alerts
ros2 topic echo /autonomy/safety/alerts

# Performance metrics
ros2 topic echo /autonomy/performance/metrics
```

### Logging Levels
```bash
# Set debug logging
ros2 param set /navigation_node log_level DEBUG

# View logs
ros2 logging get-level /navigation_node
```

## 🚨 Common Issues & Solutions

### Build Failures
- Check `package.xml` dependencies are installed
- Verify `CMakeLists.txt` paths are correct
- Ensure all required ROS2 packages are sourced

### Runtime Errors
- Check topic names match between publishers/subscribers
- Verify message types are compatible
- Confirm parameter files are loaded correctly

### Performance Issues
- Profile with `ros2 run tf2_ros tf2_monitor`
- Check CPU usage with system monitors
- Review QoS settings for real-time performance

## 📚 Related Documentation

- **ROS2 Concepts**: `docs/architecture/ros2_system_graphs.rst`
- **Mission System**: `missions/README.md`
- **Testing Guide**: `docs/testing/autonomy_testing.rst`
- **Configuration**: `config/README.md`

