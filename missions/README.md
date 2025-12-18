# 🚀 URC 2026 Mission Control System

This directory contains the complete mission control system for the URC 2026 Mars rover, featuring enterprise-grade architecture with non-invasive monitoring, comprehensive error handling, and production-ready reliability.

## 🏗️ Architecture Overview

### Core Components

#### **MissionExecutor** (`mission_executor.py`)
The central mission execution engine that orchestrates all mission activities.

**Key Features:**
- **Composition-based Architecture**: Uses specialized components for different concerns
- **ROS2 Integration**: Full ROS2 node with publishers, subscribers, and services
- **Health Monitoring**: Integrated system health checks and emergency response
- **Non-invasive Monitoring**: Event-driven performance tracking
- **Configuration Validation**: Startup validation of environment and configuration

**Responsibilities:**
- Mission lifecycle management (start, pause, resume, stop)
- Real-time navigation and waypoint following
- Emergency response coordination
- Teleoperation data processing integration
- Health monitoring and safety systems

#### **Mission Behaviors** (`mission_behaviors.py`)
Common mission behaviors and utilities shared across different mission types.

**Key Features:**
- **Reusable Behaviors**: Common navigation, detection, and control patterns
- **Safety Integration**: Built-in safety checks and emergency handling
- **Configuration Support**: Environment-aware behavior parameters
- **Logging Integration**: Structured emergency event logging

### Supporting Infrastructure

#### **Non-Invasive Monitoring System** (`monitoring_system.py`)
Event-driven monitoring that doesn't impact real-time performance.

**Key Features:**
- **Event-Driven Collection**: Only triggers on specific events (detections, failures, emergencies)
- **Configurable Sampling**: HIGH/MEDIUM/LOW/OFF sampling rates
- **Resource-Aware**: Memory-bounded with asynchronous processing
- **Performance Tracking**: Monitors its own overhead (< 0.5ms per event)

#### **Configuration Validation** (`config_validator.py`)
Validates configuration and environment variables at startup.

**Key Features:**
- **Environment Validation**: ROS2 domain, discovery server, hardware interfaces
- **Configuration Parsing**: YAML configuration validation
- **Network Connectivity**: ROS2 discovery server reachability checks
- **Graceful Degradation**: Clear error messages for missing dependencies

#### **Custom Exceptions** (`exceptions.py`)
Comprehensive exception hierarchy for robust error handling.

**Key Features:**
- **Specific Exception Types**: RoboticsException, ValidationError, ThermalError, etc.
- **Context Preservation**: Error context maintained for debugging
- **Auto-logging**: Critical errors automatically logged
- **Recovery Patterns**: Exception types designed for appropriate error handling

## 📊 Mission Types

### **Waypoint Navigation** (`waypoint_navigation_mission.py`)
Autonomous GPS waypoint navigation with obstacle avoidance.

**Capabilities:**
- Multi-waypoint route execution
- Real-time position tracking
- Heading and distance control
- Timeout and error handling

### **Object Detection & Approach** (`object_detection_mission.py`)
Computer vision-based object detection and autonomous approach.

**Capabilities:**
- Real-time object detection
- Distance estimation and approach
- Vision-guided navigation
- Target validation and tracking

### **Follow-Me Mission** (`follow_me_mission.py`)
ArUco tag-based following behavior.

**Capabilities:**
- ArUco tag detection and tracking
- Dynamic following behavior
- Distance and heading maintenance
- Tag timeout handling

### **Delivery Mission** (`delivery_mission.py`)
Sample pickup and delivery operations.

**Capabilities:**
- Multi-phase mission execution
- Location-based navigation
- Object manipulation coordination
- Success/failure tracking

### **Return to Operator Mission** (`return_to_operator_mission.py`)
Autonomous return to operator with multi-modal localization and safe navigation.

**Capabilities:**
- GPS-based operator tracking and path planning
- ArUco tag detection for precision final approach
- SLAM-based global localization and obstacle avoidance
- Real-time progress monitoring and safety checks
- Stuck detection and recovery behaviors
- Multi-phase mission execution (localize → plan → navigate → approach)

### **Debug Mission** (`debug_mission.py`)
Testing and validation mission for system checkout.

## 🔧 Usage & Integration

### Starting the Mission Executor

```python
from missions.mission_executor import MissionExecutor
import rclpy

# Initialize ROS2
rclpy.init()

# Create mission executor (validates config automatically)
executor = MissionExecutor()

# Start mission
waypoints = [
    {"x": 10.0, "y": 5.0, "heading": 0.0},
    {"x": 20.0, "y": 10.0, "heading": 45.0}
]
executor.start_waypoint_mission(waypoints)

# Spin ROS2
rclpy.spin(executor)
```

### Mission Commands

```python
# Start mission
ros2 topic pub /mission/commands std_msgs/String "data: '{\"command\": \"start_mission\", \"params\": {\"waypoints\": [{\"x\": 10, \"y\": 5}]}}'"

# Stop mission
ros2 topic pub /mission/commands std_msgs/String "data: '{\"command\": \"stop_mission\"}'"

# Check status
ros2 topic echo /mission/status
```

### Health Monitoring

```python
# Health check
ros2 service call /mission/health_check std_srvs/srv/Trigger

# Monitor mission progress
ros2 topic echo /mission/progress
```

## 📈 Monitoring & Observability

### Event Types
- **DETECTION**: Mission events, object detections, waypoint arrivals
- **FAILURE**: Data quality issues, navigation failures, system errors
- **EMERGENCY**: Thermal overloads, battery critical, motor failures
- **PERFORMANCE_DEGRADATION**: Slow operations, resource constraints

### Sampling Rates
- **HIGH**: All events (debugging)
- **MEDIUM**: Important events only (production default)
- **LOW**: Critical events only (minimal monitoring)
- **OFF**: Disabled (maintenance)

### Performance Metrics
- **Event Processing**: < 0.5ms per event
- **Memory Usage**: < 50MB buffer limit
- **CPU Overhead**: < 1% of system resources
- **Data Volume**: Minimal network traffic

## 🛡️ Safety & Reliability

### Emergency Response
1. **Detection**: Multi-threshold monitoring (warning → critical → emergency)
2. **Assessment**: Severity-based evaluation (LOW, MEDIUM, HIGH, CRITICAL)
3. **Response**: Coordinated multi-system actions
4. **Recovery**: Automatic or manual recovery procedures

### Data Quality Assurance
- **Validation**: Real-time data validation on all inputs
- **Filtering**: Position, velocity, and temperature filtering
- **Monitoring**: Continuous quality assessment with alerting
- **Fallback**: Graceful degradation on data quality issues

### Configuration Management
- **Validation**: Startup configuration validation
- **Environment**: Required environment variable checking
- **Fallback**: Sensible defaults with clear warnings
- **Documentation**: Comprehensive configuration documentation

## 🧪 Testing & Validation

### Automated Test Suite
```bash
# Run all automated tests
python3 tests/run_tests.py --automated

# Run AoI (Age of Information) tests
python3 tests/run_tests.py --aoi
```

### Configuration Validation
```bash
# Validate production configuration
python3 scripts/validate_config.py
```

### Health Checks
```bash
# Production health check
python3 scripts/production_health_check.py
```

## 📚 API Documentation

### MissionExecutor Class
```python
class MissionExecutor(Node):
    def start_waypoint_mission(self, waypoints: List[Dict[str, float]]) -> None:
        """Start autonomous waypoint navigation mission."""

    def stop_mission(self) -> None:
        """Stop current mission execution."""

    def dispatch_command(self, command: str, params: Dict[str, Any]) -> None:
        """Dispatch mission commands to handlers."""
```

### Monitoring System
```python
from missions.monitoring_system import record_detection, record_failure

# Record events
record_detection("mission_executor", {"event": "waypoint_reached"})
record_failure("data_processor", {"error": "quality_threshold_exceeded"})
```

## 🔄 Development Workflow

1. **Feature Development**: Implement in appropriate mission file
2. **Testing**: Run automated test suite
3. **Documentation**: Update docstrings and README
4. **Integration**: Test with full system
5. **Validation**: Configuration and health checks
6. **Deployment**: Ready for rover testing

## 📋 File Structure

```
missions/
├── __init__.py
├── README.md                          # This file
├── mission_executor.py               # Core execution engine
├── teleoperation_data_processor.py   # Data processing & validation
├── system_health_monitor.py          # Health monitoring
├── emergency_response_coordinator.py # Emergency handling
├── monitoring_system.py              # Non-invasive monitoring
├── config_validator.py               # Configuration validation
├── exceptions.py                     # Custom exception hierarchy
├── mission_behaviors.py              # Common behaviors
├── debug_mission.py                  # Debug/testing mission
├── delivery_mission.py               # Delivery mission
├── follow_me_mission.py              # Follow-me mission
├── object_detection_mission.py       # Object detection mission
└── waypoint_navigation_mission.py    # Waypoint navigation mission
```

---

**🎯 This mission control system provides enterprise-grade reliability, comprehensive monitoring, and robust safety systems for autonomous Mars rover operations.**
