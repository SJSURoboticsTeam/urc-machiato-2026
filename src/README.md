# 🚀 ROS2 Source Packages

This directory contains all ROS2 packages and source code for the URC 2026 rover autonomy system.

## 📁 Package Organization

### 🤖 `autonomy/` - Core Robotics Stack
The heart of the rover's autonomous capabilities.

- **`bt/`** - Behavior Tree implementations for mission logic
- **`control/`** - Hardware control (LED status, motor commands)
- **`core/`** - Core autonomy (navigation, state management, safety)
- **`interfaces/`** - ROS2 messages, services, and actions
- **`perception/`** - Computer vision, SLAM, and sensor processing
- **`utilities/`** - Shared utilities and helpers

### 🌉 `bridges/` - Communication Layer
Inter-system communication and data bridging.

- WebSocket bridges for dashboard communication
- Mission orchestration between components
- Data synchronization across the system

### 💻 `frontend/` - Web Dashboard
Real-time monitoring and control interface.

- React/TypeScript components
- Real-time data visualization
- Mission control and teleoperation

### 🎮 `simulation/` - Gazebo Integration
Simulation environment for testing and development.

- Robot models and world environments
- Sensor simulation and physics
- Testing scenarios and validation

## 🛠️ Development Workflow

### Building ROS2 Packages
```bash
# From project root
colcon build --symlink-install
source install/setup.bash
```

### Running Individual Packages
```bash
# Launch specific package
ros2 launch autonomy_navigation navigation.launch.py

# Check package info
ros2 pkg list | grep autonomy
```

### Testing Packages
```bash
# Test specific package
colcon test --packages-select autonomy_navigation
colcon test-result --verbose
```

## 📋 Key ROS2 Concepts

- **Packages**: Modular units of functionality (like `autonomy_navigation`)
- **Nodes**: Executable processes within packages
- **Topics**: Publish/subscribe communication channels
- **Services**: Request/response communication
- **Actions**: Long-running tasks with feedback

## 🗺️ Finding Your Way Around

| I need to... | Look in... | Files to check |
|-------------|------------|----------------|
| Change navigation behavior | `autonomy/core/navigation/` | `navigation_node.py` |
| Add new ROS2 message | `autonomy/interfaces/` | `msg/`, `srv/`, `action/` |
| Modify web dashboard | `frontend/` | `src/components/` |
| Add communication bridge | `bridges/` | `websocket_bridge.py` |
| Test in simulation | `simulation/` | `worlds/`, `models/` |

## 🔧 Package Dependencies

Most autonomy packages depend on:
- `rclpy` or `rclcpp` - ROS2 client libraries
- `autonomy_interfaces` - Shared message definitions
- `geometry_msgs`, `sensor_msgs` - Standard ROS2 messages

## 🚨 Important Notes

- **Always source setup files** before running ROS2 commands
- **Use colcon build** for ROS2 packages, not pip
- **Check package.xml** for dependencies when adding new packages
- **Test in simulation first** before hardware testing

