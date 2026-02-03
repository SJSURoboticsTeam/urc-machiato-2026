# Quick Reference: System Interfaces & APIs

## Critical APIs at a Glance

### Configuration API
**Module**: `src/infrastructure/config`

```python
from src.infrastructure.config import get_urc_config

# Get system configuration (single source of truth)
config = get_urc_config()

# Access nested configs
config.navigation.max_linear_velocity_ms  # 2.0
config.safety.emergency_stop_enabled  # True
config.network.websocket_port  # 8000
config.environment  # "development" or "production"

# Type-safe access (all values are validated)
if config.safety.emergency_stop_enabled:
    # Emergency stop is active
    pass
```

**Use Case**: Loading configuration in any module
**Key File**: `src/infrastructure/config/schemas.py` (Pydantic models)

### Navigation API
**Module**: `src/autonomy/autonomy_core/navigation`

```python
from src.autonomy.autonomy_core.navigation import PathPlanner

# Create planner with configuration
planner = PathPlanner(config=config.navigation)

# Plan path between points
waypoints = planner.plan_path(
    start=Point(x=0, y=0),
    goal=Point(x=10, y=10),
    obstacles=obstacle_list
)

# Execute motion
controller = MotionController(config)
controller.execute_trajectory(waypoints)
```

**Use Case**: Autonomous navigation
**Test Location**: `tests/unit/test_navigation.py`

### Safety API
**Module**: `src/autonomy/autonomy_core/safety`

```python
from src.autonomy.autonomy_core.safety import EmergencyStop

# Initialize safety system
safety = EmergencyStop(config=config.safety)

# Trigger emergency stop
safety.trigger(reason="Obstacle detected")

# Check status
is_active = safety.is_active()
reason = safety.get_stop_reason()

# Reset after addressing issue
safety.reset()
```

**Use Case**: Emergency stop and recovery
**Test Location**: `tests/critical/test_emergency_stop.py`

### Control API
**Module**: `src/autonomy/autonomy_core/control`

```python
from src.autonomy.autonomy_core.control import MotorDriver, SensorReader

# Motor control
motor = MotorDriver(port="CAN0", motor_id=1)
motor.set_velocity(speed_ms=1.5)  # 1.5 m/s
motor.set_acceleration(accel=0.5)  # 0.5 m/s²

# Sensor reading
sensor = SensorReader(sensor_type="imu", device_id=0)
imu_data = sensor.read()  # Returns IMUData object
print(imu_data.acceleration)  # (x, y, z) tuple
print(imu_data.orientation)    # (roll, pitch, yaw)
```

**Use Case**: Hardware control and sensing
**Test Location**: `tests/hardware/test_motor_control.py`

### Perception API
**Module**: `src/autonomy/autonomy_core/perception`

```python
from src.autonomy.autonomy_core.perception import VisionProcessor, SLAMNode

# Object detection
vision = VisionProcessor(model="yolov8")
detections = vision.detect_objects(frame)

# Filter for ArUco markers
markers = [d for d in detections if d.class_name == "aruco_marker"]
for marker in markers:
    print(f"Marker ID: {marker.class_id}, Position: {marker.bbox}")

# SLAM for localization
slam = SLAMNode(config=config.perception)
slam.process_frame(image_frame)
pose = slam.get_current_pose()  # (x, y, yaw)
```

**Use Case**: Computer vision and SLAM
**Test Location**: `tests/unit/test_perception.py`

### WebSocket Bridge API
**Module**: `src/infrastructure/bridges`

```python
from src.infrastructure.bridges import WebSocketBridge

# Initialize bridge (connects to dashboard)
bridge = WebSocketBridge(host="localhost", port=8000)

# Send data to dashboard
bridge.emit("robot_state", {
    "position": (x, y),
    "orientation": yaw,
    "battery": 85.5,
    "status": "AUTONOMOUS"
})

# Receive commands from dashboard
command = bridge.receive_command(timeout=1.0)
if command.type == "navigate":
    # Execute navigation command
    pass
```

**Use Case**: Real-time communication with web dashboard
**Test Location**: `tests/unit/test_websocket_bridge.py`

### CAN Bridge API
**Module**: `src/infrastructure/bridges`

```python
from src.infrastructure.bridges import CANBridge

# Initialize CAN communication
can = CANBridge(interface="CAN0", bitrate=1000000)

# Send motor command
can.send_message(
    arbitration_id=0x100,
    data=[speed_high, speed_low, accel, direction]
)

# Receive sensor data
msg = can.receive_message(timeout=1.0)
if msg:
    sensor_id, sensor_data = msg
    print(f"Sensor {sensor_id}: {sensor_data}")
```

**Use Case**: CAN bus hardware communication
**Test Location**: `tests/hardware/test_can_bridge.py`

### Monitoring API
**Module**: `src/infrastructure/monitoring`

```python
from src.infrastructure.monitoring import HealthMonitor, MetricsCollector

# Health monitoring
monitor = HealthMonitor()
monitor.register_system("navigation", check_interval=1.0)
monitor.register_system("safety", check_interval=0.5)

# Get system status
status = monitor.get_status()
print(status.overall_health)  # "HEALTHY", "DEGRADED", "CRITICAL"
print(status.components["navigation"])  # Component health

# Metrics collection
metrics = MetricsCollector()
metrics.record("navigation_latency", value=45.3, unit="ms")
metrics.record("sensor_read_time", value=12.1, unit="ms")

# Export metrics
prometheus_metrics = metrics.export_prometheus()
```

**Use Case**: System health tracking and diagnostics
**Test Location**: `tests/unit/test_monitoring.py`

### Mission API
**Module**: `missions`

```python
from missions.sample_collection_mission import SampleCollectionMission

# Create mission instance
mission = SampleCollectionMission(
    config=config,
    start_position=(0, 0),
    sample_locations=[(5, 5), (10, 0)]
)

# Execute mission lifecycle
result = mission.execute()

if result.success:
    print(f"Collected {result.samples_collected} samples")
else:
    print(f"Mission failed: {result.error_reason}")
```

**Use Case**: Mission execution
**Base Class**: `missions/base_mission.py`

## Common Integration Patterns

### Pattern 1: Navigation with Safety
```python
from src.autonomy.autonomy_core.navigation import PathPlanner
from src.autonomy.autonomy_core.safety import EmergencyStop
from src.infrastructure.config import get_urc_config

config = get_urc_config()
planner = PathPlanner(config=config.navigation)
safety = EmergencyStop(config=config.safety)

# Plan path
waypoints = planner.plan_path(start, goal)

# Check safety before execution
if not safety.is_active():
    # Execute trajectory
    execute(waypoints)
else:
    print("Cannot execute: Emergency stop is active")
```

### Pattern 2: Sensor Reading with Fallback
```python
from src.autonomy.autonomy_core.control import SensorReader

primary_sensor = SensorReader(sensor_type="lidar", device_id=0)
fallback_sensor = SensorReader(sensor_type="imu", device_id=1)

try:
    data = primary_sensor.read(timeout=1.0)
except TimeoutError:
    # Fallback to secondary sensor
    data = fallback_sensor.read(timeout=1.0)
    log.warning("Primary sensor timeout, using fallback")

use_sensor_data(data)
```

### Pattern 3: Dashboard Communication
```python
from src.infrastructure.bridges import WebSocketBridge
from src.infrastructure.monitoring import HealthMonitor

bridge = WebSocketBridge()
monitor = HealthMonitor()

# Send periodic updates
while True:
    status = monitor.get_status()
    bridge.emit("health_status", {
        "overall": status.overall_health,
        "components": status.components,
        "timestamp": time.time()
    })
    time.sleep(1.0)
```

### Pattern 4: Error Recovery
```python
from src.autonomy.autonomy_core.safety import EmergencyStop

safety = EmergencyStop(config)

try:
    execute_mission()
except HardwareError as e:
    # Trigger safety shutdown
    safety.trigger(reason=f"Hardware error: {e}")
    
    # Attempt recovery
    try:
        recover_from_error(e)
        safety.reset()
    except RecoveryFailed:
        # Request manual intervention
        request_human_intervention()
```

## Testing Patterns

### Unit Test Pattern
```python
import pytest
from src.autonomy.autonomy_core.navigation import PathPlanner
from tests.factories import NavigationConfigFactory

def test_path_planning_to_goal():
    # Arrange
    config = NavigationConfigFactory.create()
    planner = PathPlanner(config=config)
    
    # Act
    waypoints = planner.plan_path(
        start=Point(0, 0),
        goal=Point(10, 10)
    )
    
    # Assert
    assert len(waypoints) > 0
    assert waypoints[-1].x == pytest.approx(10, abs=0.1)
    assert waypoints[-1].y == pytest.approx(10, abs=0.1)
```

### Integration Test Pattern
```python
def test_full_navigation_mission(mock_ros_node):
    # Setup
    mission = NavigationMission(...)
    
    # Execute
    result = mission.execute()
    
    # Verify
    assert result.success
    assert result.waypoints_reached > 0
```

### Hardware Test Pattern
```python
@pytest.mark.hardware
def test_motor_control_real_hardware():
    motor = MotorDriver(port="CAN0", motor_id=1)
    
    # Test motor response
    motor.set_velocity(0.5)
    time.sleep(0.1)
    
    # Verify (with actual hardware)
    velocity = motor.get_current_velocity()
    assert velocity == pytest.approx(0.5, abs=0.1)
```

## Key Configuration Parameters

```yaml
# src/infrastructure/config - Check schemas.py for all options
navigation:
  max_linear_velocity_ms: 2.0      # Maximum forward speed
  max_angular_velocity_rad_s: 3.14  # Maximum rotation speed
  planning_algorithm: "astar"       # Path planning algorithm
  safety_margin_m: 0.3              # Clearance from obstacles

safety:
  emergency_stop_enabled: true
  watchdog_timeout_s: 2.0
  auto_recovery_enabled: true

perception:
  detection_model: "yolov8"
  confidence_threshold: 0.7
  frame_rate_hz: 30

network:
  websocket_port: 8000
  websocket_host: "0.0.0.0"
  timeout_s: 5.0
  retry_count: 3
```

## Common Debug Commands

```bash
# Check configuration
python -c "from src.infrastructure.config import get_urc_config; print(get_urc_config())"

# Run specific test
python -m pytest tests/unit/test_navigation.py::test_path_planning -v

# Profile performance
python -m cProfile -s cumtime -m pytest tests/performance/ -v

# Check code types
python -m mypy src/ --ignore-missing-imports

# Run with logging
python -m pytest tests/ -v --log-cli-level=DEBUG
```

## Where to Find More Info

| Topic | Location |
|-------|----------|
| Configuration system | `src/infrastructure/config/schemas.py` |
| Navigation | `src/autonomy/autonomy_core/navigation/` |
| Safety | `src/autonomy/autonomy_core/safety/` |
| Control | `src/autonomy/autonomy_core/control/` |
| Perception | `src/autonomy/autonomy_core/perception/` |
| WebSocket | `src/infrastructure/bridges/websocket_bridge.py` |
| CAN | `src/infrastructure/bridges/can_bridge.py` |
| Monitoring | `src/infrastructure/monitoring/` |
| Missions | `missions/base_mission.py` |
| Tests | `tests/conftest.py` for fixtures |
| Examples | `docs/onboarding/examples/` |

## Quick Start Checklist

- [ ] Read `docs/getting_started.rst`
- [ ] Review `docs/onboarding/README.md`
- [ ] Check `AGENTS.md` for commands
- [ ] Run `./scripts/build.sh dev`
- [ ] Run tests: `python -m pytest tests/unit/ -v`
- [ ] Try an example: `python -c "from src.infrastructure.config import get_urc_config; print(get_urc_config())"`
- [ ] Read module docstrings: `help(get_urc_config)`
- [ ] Ask questions!

Good luck with your contributions!
