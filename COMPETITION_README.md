# URC 2026 Competition-Ready System

This document describes the complete competition-ready software system for URC 2026, integrating ROS2 autonomy with deployed STM32 control systems.

## 🚀 System Overview

The system now includes all critical URC competition components:

### **Hardware Integration Layer**
- **Hardware Interface Node** (`src/autonomy/control/hardware_interface/`)
  - ROS2 ↔ STM32 CAN bus communication
  - Real-time motor control and telemetry
  - Emergency stop coordination
  - Based on teleoperation system CAN protocols

### **Terrain Intelligence**
- **Terrain Analyzer** (`src/autonomy/core/terrain_intelligence/`)
  - Real-time terrain classification (sand, rock, slope, hazard)
  - Traversability cost mapping for navigation
  - Slope analysis and hazard detection
  - Computer vision and point cloud processing

### **Competition Safety**
- **Geofencing System** (`src/autonomy/core/safety_system/competition_safety/`)
  - GPS-based boundary enforcement
  - Competition area monitoring
  - Automatic emergency stops for violations
  - RViz visualization

### **Autonomous Missions**
- **Keyboard Typing Mission** (`missions/autonomous_keyboard_mission.py`)
  - Computer vision keyboard detection
  - Autonomous navigation and positioning
  - Arm control integration with existing typing package
  - Complete URC task implementation

- **Sample Collection Mission** (`missions/sample_collection_mission.py`)
  - Autonomous sample detection and approach
  - Excavation control via STM32 science payload
  - Sample caching and preservation
  - Science data integration

### **Competition Communication**
- **Competition Bridge** (`src/bridges/competition_bridge.py`)
  - WebSocket telemetry streaming for judges
  - Real-time mission status and health monitoring
  - Operator command processing
  - Competition data logging

## 🏁 Quick Start

### **1. Launch Competition System**
```bash
# From project root
cd /home/ubuntu/urc-machiato-2026
source /opt/ros/humble/setup.bash
colcon build --packages-select hardware_interface terrain_intelligence competition_safety competition_bridge
source install/setup.bash

# Launch all competition components
ros2 launch autonomy competition_system.launch.py
```

### **2. Start Dashboard**
```bash
# In another terminal
cd frontend
npm start
```

### **3. Connect Hardware**
```bash
# Connect STM32 via CAN serial
# The hardware interface will automatically detect and connect
# Monitor connection status in logs
```

## 📡 Communication Architecture

### **ROS2 Topics**

#### **Hardware Interface**
```
/hardware/joint_states          # Motor positions/velocities
/hardware/chassis_velocity      # Measured velocity
/hardware/motor_temperatures    # Motor temperatures
/hardware/battery_state         # Battery status
/hardware/imu                   # IMU data
/hardware/gps                   # GPS position
/hardware/system_status         # Overall health
/hardware/arm_command           # Arm control commands
/hardware/excavate_command      # Excavation commands
```

#### **Terrain Intelligence**
```
/terrain/traversability         # OccupancyGrid cost map
/terrain/classification         # Classified terrain image
```

#### **Competition Safety**
```
/safety/boundary_violation      # Boundary violation alerts
/safety/alert                   # Safety alerts
/safety/boundary_visualization  # RViz boundary markers
```

#### **Mission Control**
```
/mission/commands               # Mission commands
/mission/status                 # General mission status
/mission/keyboard_status        # Keyboard mission status
/mission/sample_collection_status # Sample collection status
```

#### **Competition Bridge**
```
/competition/telemetry          # WebSocket telemetry data
/competition/commands           # Operator commands
```

### **WebSocket Interface**
- **Port:** 8080
- **Protocol:** JSON messages
- **Endpoints:**
  - Real-time telemetry streaming
  - Mission control commands
  - Emergency stop coordination
  - System health monitoring

## 🎯 Mission Execution

### **Autonomous Keyboard Typing**
```bash
# Start keyboard mission
ros2 topic pub /mission/keyboard_command std_msgs/String "data: 'start'"

# Or via WebSocket:
{
  "type": "start_mission",
  "mission": "keyboard_typing"
}
```

### **Sample Collection**
```bash
# Start sample collection
ros2 topic pub /mission/sample_collection_command std_msgs/String "data: 'start'"

# Manual sample location
ros2 topic pub /mission/sample_collection_command std_msgs/String "data: 'collect_sample_at:33.05,-117.05'"
```

### **Emergency Controls**
```bash
# Emergency stop
ros2 topic pub /emergency_stop std_msgs/Bool "data: true"

# Via WebSocket:
{
  "type": "emergency_stop"
}
```

## 🔧 Configuration

### **Competition Boundaries**
Create `config/competition_boundary.json`:
```json
{
  "competition_boundary": [
    [33.0, -117.1],
    [33.0, -117.0],
    [33.1, -117.0],
    [33.1, -117.1]
  ],
  "geofenced_zones": [
    {
      "name": "lander_area",
      "boundary": [
        [33.05, -117.05],
        [33.05, -117.04],
        [33.06, -117.04],
        [33.06, -117.05]
      ],
      "severity": "critical"
    }
  ]
}
```

### **Hardware Interface**
```yaml
hardware_interface:
  can_port: "/dev/ttyACM0"
  can_baudrate: 115200
  control_rate_hz: 50.0
  telemetry_rate_hz: 10.0
```

### **Competition Bridge**
```yaml
competition_bridge:
  websocket_port: 8080
  telemetry_rate_hz: 5.0
  max_websocket_clients: 10
  competition_log_file: "competition_telemetry.jsonl"
  enable_data_logging: true
```

## 📊 Monitoring & Telemetry

### **Web Dashboard**
- Real-time system status
- Mission progress tracking
- Sensor data visualization
- Emergency controls

### **WebSocket Telemetry**
Connect to `ws://localhost:8080` for real-time data:
```json
{
  "timestamp": 1640995200.0,
  "mission_time": 45.2,
  "current_mission": "sample_collection",
  "samples_collected": 3,
  "battery_level": 85.0,
  "gps_position": {"lat": 33.05, "lon": -117.05},
  "emergency_stop": false,
  "boundary_violation": false
}
```

### **ROS2 Monitoring**
```bash
# System status
ros2 topic echo /hardware/system_status

# Mission progress
ros2 topic echo /mission/status

# Safety alerts
ros2 topic echo /safety/alert
```

## 🧪 Testing

### **Hardware-in-the-Loop Testing**
```bash
# Start simulation with hardware interface mock
ros2 launch autonomy competition_system.launch.py use_mock_hardware:=true

# Run competition scenario tests
python3 tests/competition/test_competition_scenario.py
```

### **Integration Testing**
```bash
# Test full mission execution
python3 test_end_to_end_integration.py

# Test emergency stop system
python3 tests/competition/test_emergency_stop_system.py
```

### **Performance Testing**
```bash
# Competition load testing
python3 tests/performance/competition_performance_profiler.py

# Latency measurement
python3 tests/performance/test_latency_measurement.py
```

## 🔒 Safety Features

### **Multi-Layer Safety**
1. **Hardware E-stop** - Physical emergency stop buttons
2. **Software E-stop** - ROS2 topic-based emergency stop
3. **Geofencing** - GPS boundary enforcement
4. **Terrain Safety** - Hazard detection and avoidance
5. **Mission Safety** - Automatic mission abort on anomalies

### **Safety Monitoring**
- Real-time health monitoring
- Automatic fault detection
- Redundant safety systems
- Competition boundary enforcement

## 📋 Competition Checklist

### **Pre-Competition Validation**
- [ ] Hardware interface connected and tested
- [ ] CAN communication verified
- [ ] GPS boundaries configured
- [ ] WebSocket telemetry streaming
- [ ] Emergency stop system tested
- [ ] All missions functional
- [ ] Terrain intelligence calibrated
- [ ] Sample collection system ready

### **During Competition**
- [ ] Telemetry streaming to judges
- [ ] Real-time mission monitoring
- [ ] Safety systems active
- [ ] Data logging enabled
- [ ] Operator controls responsive

## 🚨 Troubleshooting

### **Hardware Connection Issues**
```bash
# Check CAN serial connection
dmesg | grep ttyACM

# Test CAN communication
ros2 topic echo /hardware/system_status
```

### **WebSocket Connection**
```bash
# Check WebSocket server
netstat -tlnp | grep 8080

# Test connection
websocat ws://localhost:8080
```

### **Mission Execution**
```bash
# Check mission status
ros2 topic echo /mission/status

# View mission logs
ros2 node list
ros2 node info /autonomous_keyboard_mission
```

## 📈 Performance Metrics

- **Control Loop:** 50Hz (20ms latency)
- **Telemetry:** 5-10Hz streaming
- **Mission Planning:** Real-time path planning
- **Computer Vision:** 10-30Hz processing
- **WebSocket:** Sub-100ms latency

## 🤝 Integration with Vendor Systems

The system integrates with:
- **Control Systems:** STM32 motor controllers via CAN
- **Teleoperation:** WebSocket command interface
- **Science Payload:** Excavation and caching systems
- **Navigation:** Existing SLAM and path planning
- **Dashboard:** Real-time visualization and control

All components follow the established integration guide protocols and maintain compatibility with deployed hardware.
