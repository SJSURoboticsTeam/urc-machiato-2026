# URC 2026 Technical System Overview

## Project At a Glance

**Repository**: University Rover Challenge 2026 Mars Rover
**Language**: Python 3.10+, ROS2 Humble, React/TypeScript
**Build System**: colcon (ROS2) + Python setuptools
**Architecture**: Unified infrastructure + consolidated autonomy core
**Status**: Active development, 85%+ test coverage target

---

## The Four Technical Pillars

```
┌────────────────────────────────────────────────────────────┐
│                    WEB DASHBOARD                           │
│              (React/TypeScript Frontend)                   │
└─────────────────┬──────────────────────────────────────────┘
                  │
        ┌─────────▼──────────┐
        │  WebSocket Bridge   │
        │  (Communication)    │
        └─────────┬──────────┘
                  │
┌─────────────────▼──────────────────────────────────────────┐
│              ROS2 SYSTEM (Main Computer)                   │
│                                                            │
│  ┌──────────────────────────────────────────────────────┐ │
│  │ INFRASTRUCTURE (Unified across all systems)         │ │
│  │ ├─ Config (src/infrastructure/config/)             │ │
│  │ ├─ Bridges (src/infrastructure/bridges/)           │ │
│  │ └─ Monitoring (src/infrastructure/monitoring/)     │ │
│  └──────────────────────────────────────────────────────┘ │
│                                                            │
│  ┌──────────────────────────────────────────────────────┐ │
│  │ AUTONOMY CORE (src/autonomy/autonomy_core/)         │ │
│  │ ├─ PERCEPTION: Vision, SLAM, sensor fusion         │ │
│  │ ├─ COGNITION: State machine, behavior tree         │ │
│  │ ├─ MOTION: Navigation, path planning, control      │ │
│  │ └─ SAFETY: Emergency stop, watchdog                │ │
│  └──────────────────────────────────────────────────────┘ │
│                                                            │
│  ┌──────────────────────────────────────────────────────┐ │
│  │ MISSIONS (missions/*.py)                             │ │
│  │ └─ Sample collection, delivery, waypoint nav, etc.  │ │
│  └──────────────────────────────────────────────────────┘ │
└─────────────────┬──────────────────────────────────────────┘
                  │
        ┌─────────▼──────────┐
        │  CAN Bridge         │
        │  (Hardware comms)   │
        └─────────┬──────────┘
                  │
┌─────────────────▼──────────────────────────────────────────┐
│          HARDWARE (Motors, Sensors, Controllers)           │
│  ┌─────────────────────────────────────────────────────┐  │
│  │ Drive Motors (2x)  │  Arm Motors (3x)  │ E-Stop    │  │
│  │ Encoders (2x)      │  Cameras (2x)     │ Circuit   │  │
│  │ IMU, GPS, LiDAR    │  Sensors (temp, battery)      │  │
│  └─────────────────────────────────────────────────────┘  │
└────────────────────────────────────────────────────────────┘
```

---

## Part 1: Infrastructure (Cross-Cutting Concerns)

### 1.1 Configuration System

**Location**: `src/infrastructure/config/`

**Why It Exists**: Single source of truth for all parameters (speed limits, timeouts, detection thresholds, etc.)

**Design**:
```
rover.yaml (git tracked)
    ↓
Dynaconf (environment-aware loading)
    ↓
Pydantic models (type-safe validation)
    ↓
get_urc_config() (available everywhere)
```

**Key Files**:
- `schemas.py` - Pydantic models defining all config options
- `settings.py` - Dynaconf configuration loading
- `validators.py` - Config validation logic

**How to Access**:
```python
from src.infrastructure.config import get_urc_config

config = get_urc_config()
print(config.navigation.max_linear_velocity_ms)  # 2.0
print(config.safety.emergency_stop_enabled)       # True
```

**Key Configuration Areas**:

| Area | Parameters | Why |
|------|-----------|-----|
| **Navigation** | max velocity, acceleration, PID gains | Ensure safe, smooth motion |
| **Safety** | e-stop enabled, watchdog timeout, recovery limits | Prevent damage, enable safe operation |
| **Perception** | detection model, confidence threshold, update rates | Control accuracy vs. speed tradeoff |
| **Network** | websocket port, CAN interface, retry logic, timeouts | Configure communication hardware |
| **Hardware** | use_mock, motor_count, encoder_ppr, wheel_diameter | Support hardware variations |

**Areas for Improvement**:
- [ ] Add dynamic parameter tuning (adjust during runtime)
- [ ] Implement configuration versioning (track changes)
- [ ] Add configuration profiles (quick switch between setups)
- [ ] Create validation that prevents unsafe parameter combinations

---

### 1.2 Bridge System (Communication Infrastructure)

**Location**: `src/infrastructure/bridges/`

**Why It Exists**: Decouple software from hardware, enable testing without real hardware, handle communication failures gracefully

**Components**:

#### WebSocket Bridge
```
Purpose: Browser ↔ ROS2 bidirectional communication
File: websocket_bridge.py
Port: Configurable (default 8000)

Flow:
  Browser sends JSON → Flask/Socket.IO → Convert to ROS2 topic
  ROS2 topic → Convert to JSON → Send to all browsers
```

**Usage**:
```python
from src.infrastructure.bridges import WebSocketBridge

bridge = WebSocketBridge(host="0.0.0.0", port=8000)

# Send data to dashboard
bridge.emit("robot_status", {
    "position": [1.0, 2.0],
    "mode": "autonomous",
    "battery": 85.5
})

# Receive commands from dashboard
cmd = bridge.receive_command(timeout=1.0)
```

#### CAN Bridge
```
Purpose: ROS2 ↔ Motor controller communication
File: can_bridge.py
Protocol: SLCAN (Serial CAN)

Message Format:
  [Arbitration ID (11-bit)] [Data Length] [8 bytes data] [CRC]

Example - Set motor velocity:
  ID: 0x100
  Data: [motor_id, speed_high, speed_low, ...]
```

**Usage**:
```python
from src.infrastructure.bridges import CANBridge

can = CANBridge(interface="CAN0", bitrate=1000000)

# Send motor command
can.send_message(
    arbitration_id=0x100,
    data=[0x01, 0x05, 0xDC]  # Motor 1, 1500 RPM
)

# Receive sensor data
msg = can.receive_message(timeout=1.0)
if msg:
    arbitration_id, data = msg
    print(f"Motor feedback: {data}")
```

#### Circuit Breaker
```
Purpose: Graceful degradation when services fail
File: circuit_breaker.py
Pattern: CLOSED → OPEN → HALF_OPEN → CLOSED

States:
  CLOSED: Normal operation, try requests
  OPEN: Service down, stop trying, fast-fail
  HALF_OPEN: Try one request cautiously
```

**Usage**:
```python
from src.infrastructure.bridges import get_adaptive_circuit_breaker

breaker = get_adaptive_circuit_breaker("motor_control")
try:
    result = breaker.call(send_motor_command, speed=1500)
except CircuitBreakerOpen:
    print("Motor service is down, using fallback strategy")
```

**Areas for Improvement**:
- [ ] Add persistent logging of all bridge communications
- [ ] Implement message queuing during disconnections
- [ ] Add heartbeat/watchdog at bridge level
- [ ] Improve error recovery and reporting
- [ ] Performance optimization for high-frequency data

---

### 1.3 Monitoring System (Health & Diagnostics)

**Location**: `src/infrastructure/monitoring/`

**Why It Exists**: Real-time system health tracking, enable early failure detection, support diagnostics

**Components**:

**HealthMonitor**:
```python
from src.infrastructure.monitoring import HealthMonitor

monitor = HealthMonitor()

# Register systems to monitor
monitor.register_system("perception", check_interval=1.0)
monitor.register_system("navigation", check_interval=0.5)
monitor.register_system("safety", check_interval=0.2)

# Get overall health
status = monitor.get_status()
print(status.overall_health)  # "HEALTHY", "DEGRADED", "CRITICAL"
print(status.components["navigation"])  # Individual system status
```

**MetricsCollector**:
```python
from src.infrastructure.monitoring import MetricsCollector

metrics = MetricsCollector()

# Record performance metrics
metrics.record("vision_latency", value=45.3, unit="ms")
metrics.record("navigation_cpu_usage", value=62.5, unit="%")
metrics.record("memory_usage", value=285.2, unit="MB")

# Export for monitoring
prometheus = metrics.export_prometheus()
```

**Areas for Improvement**:
- [ ] Add historical data storage (trending)
- [ ] Implement anomaly detection
- [ ] Create performance regression tests
- [ ] Add predictive failure analysis
- [ ] Improve dashboard visualization

---

## Part 2: Autonomy Core (Core Robotics)

**Location**: `src/autonomy/autonomy_core/`

This is a consolidated ROS2 package containing all robotics logic.

### 2.1 Perception Subsystem

**Location**: `autonomy_core/perception/`

**Purpose**: Give the robot "vision" - detect objects, understand position, fuse sensor data

**Architecture**:
```
Raw Sensor Data
  ├─ Camera images
  ├─ Depth data (LiDAR, RealSense)
  ├─ IMU (acceleration, rotation)
  └─ GPS (global position)
         ↓
  ┌─────────────────────┐
  │ Computer Vision     │
  │ - Object detection  │
  │ - ArUco markers     │
  │ - Feature extraction│
  └─────────────────────┘
         ↓
  ┌─────────────────────┐
  │ SLAM                │
  │ - Position tracking │
  │ - Map building      │
  │ - Loop closure      │
  └─────────────────────┘
         ↓
  ┌─────────────────────┐
  │ Sensor Fusion       │
  │ - Kalman filter     │
  │ - Multi-sensor merge│
  │ - Validation        │
  └─────────────────────┘
         ↓
ROS2 Topics Published:
  /vision/detections (objects found)
  /slam/pose (estimated position)
  /sensor_data/processed (fused readings)
  /perception/diagnostics (health info)
```

**Key Files**:

| File | Purpose | Status |
|------|---------|--------|
| `computer_vision_node.py` | ArUco detection, object detection | Core |
| `slam_node.py` | Localization and mapping | Core |
| `sensor_fusion.py` | Multi-sensor data fusion | Core |
| `depth_processor.py` | Filter and process depth data | Enhancement |

**How It Works** (Example: Sample Detection):

```python
# In computer_vision_node.py
def detect_samples(image):
    # 1. Convert to HSV (better for color detection)
    hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV)
    
    # 2. Threshold for red color (samples are red)
    red_mask = cv2.inRange(hsv, (0, 100, 100), (10, 255, 255))
    
    # 3. Find contours (connected regions)
    contours = cv2.findContours(red_mask, ...)
    
    # 4. Filter by size (ignore noise)
    valid_contours = [c for c in contours if area > MIN_SIZE]
    
    # 5. Calculate center and area for each
    detections = []
    for contour in valid_contours:
        center = get_centroid(contour)
        area = cv2.contourArea(contour)
        detections.append({
            "position": center,
            "area": area,
            "confidence": 0.95
        })
    
    return detections
```

**Configuration**:
```python
config.perception.detection_model = "yolov8"  # or "color_detection"
config.perception.confidence_threshold = 0.7
config.perception.frame_rate_hz = 30
config.perception.slam_enabled = True
```

**Testing**:
```bash
python -m pytest tests/unit/test_computer_vision.py -v
python -m pytest tests/unit/test_slam.py -v
python -m pytest tests/integration/test_perception.py -v
```

**ROS2 Interface**:
```python
# Subscribe to camera
/camera/image_raw (sensor_msgs/Image)
/camera/depth (sensor_msgs/Image)

# Publish detections
/vision/detections (autonomy_interfaces/VisionDetection)
/slam/pose (geometry_msgs/PoseWithCovarianceStamped)
```

**Performance Metrics**:
- Vision processing: <67ms per frame (15Hz)
- SLAM update: <50ms
- Sensor fusion: <100ms total latency
- Memory: <200MB for all perception

**Known Limitations**:
- Detection accuracy: 85-90% (depends on lighting)
- SLAM drift: ~5% over 100m
- Depth sensor range: 0.1-3m
- No loop closure under 1m

**Areas for Improvement**:
- [ ] Improve detection accuracy to 95%+ with ML model tuning
- [ ] Add semantic segmentation for terrain understanding
- [ ] Implement better loop closure detection
- [ ] Add sensor validation and outlier rejection
- [ ] Optimize for lower latency (<30ms)
- [ ] Add calibration and self-tuning

**Contribution Ideas**:
1. **Improve detection accuracy**: Tune color thresholds, add ML model
2. **Enhance SLAM**: Implement feature matching algorithms
3. **Sensor fusion**: Add better Kalman filter tuning
4. **Performance**: Profile and optimize bottlenecks
5. **Testing**: Add more integration tests

---

### 2.2 Cognition Subsystem

**Location**: `autonomy_core/state_management.py` + `missions/`

**Purpose**: Make decisions - what mode, what mission, what to do next

**Architecture**:
```
Perception Data (position, obstacles, objects detected)
         ↓
  ┌──────────────────────┐
  │ STATE MACHINE        │
  │ (BOOT → IDLE → MODE) │
  │ Ensures only one mode│
  │ active at a time     │
  └──────────┬───────────┘
             ↓
  ┌──────────────────────┐
  │ BEHAVIOR TREE        │
  │ (Mission logic)      │
  │ Sequences, selectors │
  │ Retry logic          │
  └──────────┬───────────┘
             ↓
  ┌──────────────────────┐
  │ UNIFIED BLACKBOARD   │
  │ (Shared memory)      │
  │ State data for all   │
  │ components           │
  └──────────┬───────────┘
             ↓
Action Decision: "Drive forward 2m, then turn"
```

**State Machine**:

```python
# In state_management.py
class AdaptiveStateMachine:
    STATES = ["BOOT", "IDLE", "AUTONOMOUS", "TELEOPERATION", "EMERGENCY_STOP", "ERROR"]
    
    TRANSITIONS = {
        "BOOT": ["IDLE", "ERROR"],
        "IDLE": ["AUTONOMOUS", "TELEOPERATION", "SHUTDOWN"],
        "AUTONOMOUS": ["IDLE", "EMERGENCY_STOP"],
        "TELEOPERATION": ["IDLE", "EMERGENCY_STOP"],
        "EMERGENCY_STOP": ["IDLE"],  # After reset
        "ERROR": ["BOOT"]
    }
    
    # Only one state active at a time
    # Prevents conflicting commands
```

**Behavior Tree** (Sample Collection Mission):

```
ROOT: SAMPLE_COLLECTION
  ├─ SEQUENCE: MAIN_SEQUENCE
  │   ├─ ACTION: Navigate to sample location
  │   ├─ ACTION: Align arm above sample
  │   ├─ SELECTOR: Collect sample (retry up to 3x)
  │   │   ├─ ACTION: Close gripper
  │   │   ├─ ACTION: Verify sample captured
  │   │   └─ ACTION: Log success
  │   └─ ACTION: Return to home
  └─ ON_FAILURE: Handle collection failure
      ├─ Log error
      ├─ Report to operator
      └─ Continue to next mission
```

**Key Files**:

| File | Purpose |
|------|---------|
| `state_management.py` | State machine implementation |
| `adaptive_state_machine.py` | ROS2 lifecycle integration |
| `unified_blackboard_client.py` | Shared state for BT |
| `missions/sample_collection.py` | Sample mission logic |
| `missions/delivery.py` | Delivery task |
| `missions/base_mission.py` | Base class for all missions |

**How It Works** (Decision Making Loop):

```python
# In main autonomy loop
def cognition_loop(perception_data, state_machine, behavior_tree):
    # 1. Get current state
    state = state_machine.get_current_state()
    
    # 2. Based on state, run appropriate logic
    if state == "AUTONOMOUS":
        # Run behavior tree
        bt_status = behavior_tree.tick()
        
        if bt_status == "SUCCESS":
            state_machine.transition_to("IDLE")
            log("Mission completed!")
        elif bt_status == "FAILURE":
            state_machine.transition_to("IDLE")
            log("Mission failed, returning to idle")
    
    elif state == "TELEOPERATION":
        # Accept manual commands from dashboard
        cmd = receive_dashboard_command()
        execute_command(cmd)
    
    # 3. Always monitor safety
    if perception_data.obstacle_too_close:
        state_machine.transition_to("EMERGENCY_STOP")
```

**Configuration**:
```python
config.cognition.behavior_tree_tick_rate = 10  # Hz
config.cognition.decision_timeout = 5.0  # seconds
config.cognition.max_retries = 3
config.cognition.fallback_strategy = "return_home"
```

**ROS2 Interface**:
```python
# Service to get current state
/adaptive_state_machine/get_state (GetSystemState)

# Topic to monitor state
/adaptive_state_machine/state (SystemStateMsg)

# Topic for commands
/adaptive_state_machine/commands (String, JSON)
```

**Testing**:
```bash
python -m pytest tests/unit/test_state_machine.py -v
python -m pytest tests/integration/test_mission_execution.py -v
python -m pytest tests/critical/test_safety_state_transitions.py -v
```

**Performance**:
- State transition: <1ms
- BT tick: <50ms
- Decision latency: <100ms total

**Known Limitations**:
- No dynamic replanning (uses pre-built missions)
- Limited error recovery options
- Can't learn from failures
- Behavior tree is static (hardcoded)

**Areas for Improvement**:
- [ ] Add dynamic mission planning
- [ ] Implement learning from failures
- [ ] Add more sophisticated error recovery
- [ ] Create mission builder UI
- [ ] Add context-aware behavior selection

**Contribution Ideas**:
1. **Add new mission**: Create new mission class in `missions/`
2. **Improve error recovery**: Add fallback strategies
3. **Enhance BT**: Make behavior trees more flexible
4. **Testing**: Add more edge case tests
5. **Monitoring**: Better diagnostics for decision-making

---

### 2.3 Motion Control Subsystem

**Location**: `autonomy_core/navigation/` and `autonomy_core/control/`

**Purpose**: Execute decisions - actually move the robot

**Architecture**:
```
Cognition Decision: "Drive 2m/s, turn 0.3 rad/s"
         ↓
  ┌──────────────────────┐
  │ Motion Controller    │
  │ - Velocity to wheel  │
  │ - PID control        │
  │ - Acceleration limit │
  └──────────┬───────────┘
             ↓
  ┌──────────────────────┐
  │ Hardware Interface   │
  │ - PWM generation     │
  │ - CAN messages       │
  │ - Encoder reading    │
  └──────────┬───────────┘
             ↓
         Motors Spin
             ↓
  ┌──────────────────────┐
  │ Feedback             │
  │ - Encoder counts     │
  │ - Actual velocity    │
  └──────────┬───────────┘
             ↓
         Adjust Next Command
```

**Key Files**:

| File | Purpose | Status |
|------|---------|--------|
| `motion_controller.py` | Velocity commands → motor speeds | Core |
| `path_planner.py` | Plan path from A to B | Core |
| `trajectory_generator.py` | Generate smooth trajectories | Enhancement |
| `hardware_interface.py` | Motor communication | Core |
| `motor_driver.py` | Individual motor control | Core |

**Differential Drive Math**:

```python
# Convert desired linear and angular velocity to wheel speeds
def velocity_to_wheel_speeds(linear_vel, angular_vel, wheelbase=0.5):
    """
    linear_vel: desired forward speed (m/s)
    angular_vel: desired rotation speed (rad/s)
    wheelbase: distance between wheels (m)
    """
    left_speed = linear_vel - (angular_vel * wheelbase / 2)
    right_speed = linear_vel + (angular_vel * wheelbase / 2)
    
    # Clamp to motor limits
    max_speed = config.navigation.max_linear_velocity_ms
    if abs(left_speed) > max_speed or abs(right_speed) > max_speed:
        scale = max_speed / max(abs(left_speed), abs(right_speed))
        left_speed *= scale
        right_speed *= scale
    
    return left_speed, right_speed

# Example: Go forward and turn
linear = 1.5  # m/s
angular = 0.5  # rad/s
L, R = velocity_to_wheel_speeds(linear, angular)
# Left: 1.375 m/s, Right: 1.625 m/s → Curved forward path
```

**PID Control Loop**:

```python
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp  # Proportional gain
        self.ki = ki  # Integral gain
        self.kd = kd  # Derivative gain
        self.integral = 0
        self.last_error = 0
    
    def update(self, setpoint, actual):
        """Calculate next control output"""
        error = setpoint - actual
        
        # P term: proportional to error
        p_term = self.kp * error
        
        # I term: accumulate errors
        self.integral += error
        i_term = self.ki * self.integral
        
        # D term: proportional to error rate
        d_term = self.kd * (error - self.last_error)
        self.last_error = error
        
        output = p_term + i_term + d_term
        return output

# Usage: Maintain desired speed despite variations
pid = PIDController(kp=1.5, ki=0.1, kd=0.2)

while running:
    target_speed = 1.5  # m/s
    actual_speed = read_encoder()  # Maybe 1.4 m/s
    
    # PID calculates adjustment
    adjustment = pid.update(target_speed, actual_speed)
    
    # New motor power = base + adjustment
    motor_power = motor_power_base + adjustment
```

**How Acceleration Limiting Works**:

```python
def limit_acceleration(target_vel, current_vel, max_accel, dt=0.01):
    """Prevent sudden jerks"""
    max_change = max_accel * dt
    
    if target_vel > current_vel:
        # Accelerating - limit increase
        next_vel = min(target_vel, current_vel + max_change)
    else:
        # Decelerating - limit decrease
        next_vel = max(target_vel, current_vel - max_change)
    
    return next_vel

# Example: 0 → 2 m/s with 1 m/s² acceleration
# Time needed = 2 / 1 = 2 seconds
# Smooth, safe acceleration profile
```

**CAN Motor Command**:

```python
# Send velocity command via CAN bus
def send_motor_command(left_vel, right_vel):
    """
    Encode velocity as CAN message and send to motor controller
    """
    # Scale to 16-bit integer (motor controller expects 0-4000)
    left_scaled = int((left_vel / max_speed) * 2000)  # -2000 to +2000
    right_scaled = int((right_vel / max_speed) * 2000)
    
    # Pack into CAN message
    can_message = {
        'arbitration_id': 0x100,  # Motor command ID
        'data': [
            (left_scaled >> 8) & 0xFF,     # Left high byte
            left_scaled & 0xFF,             # Left low byte
            (right_scaled >> 8) & 0xFF,    # Right high byte
            right_scaled & 0xFF             # Right low byte
        ]
    }
    
    can_bridge.send_message(**can_message)
```

**Configuration**:
```python
config.navigation.max_linear_velocity_ms = 2.0
config.navigation.max_angular_velocity_rad_s = 3.14
config.navigation.max_acceleration_mps2 = 1.0
config.navigation.pid.kp_linear = 1.5
config.navigation.pid.ki_linear = 0.1
config.navigation.pid.kd_linear = 0.2

config.hardware.motor_count = 2
config.hardware.encoder_ppr = 200
config.hardware.wheel_diameter_m = 0.25
```

**ROS2 Interface**:
```python
# Subscribe to commands
/motion/velocity (geometry_msgs/Twist)
  - linear.x: forward speed
  - angular.z: rotation speed

# Publish current state
/motion/current_velocity (geometry_msgs/Twist)
/motion/odometry (nav_msgs/Odometry)

# Services
/motion/set_velocity (SetVelocity service)
```

**Testing**:
```bash
python -m pytest tests/unit/test_motion_controller.py -v
python -m pytest tests/hardware/test_motor_control.py -v
python -m pytest tests/integration/test_motion_integration.py -v
```

**Performance**:
- Control loop: 100 Hz (10ms cycle)
- Latency to motors: <50ms
- Velocity accuracy: ±5%
- Acceleration smoothness: <0.1m/s jerk

**Known Limitations**:
- Wheel slip on sand/mud (encoder inaccuracy)
- No terrain-aware control
- Simple PID (no adaptive tuning)
- Uneven terrain causes tilt
- No active suspension

**Areas for Improvement**:
- [ ] Implement terrain-aware control
- [ ] Add slip detection and compensation
- [ ] Improve PID tuning (auto-tune)
- [ ] Add active suspension control
- [ ] Reduce latency further
- [ ] Add trajectory optimization

**Contribution Ideas**:
1. **Improve PID tuning**: Optimize gains for different terrains
2. **Add terrain detection**: Recognize sand, rock, etc.
3. **Reduce latency**: Profile and optimize control loop
4. **Enhance accuracy**: Better encoder reading
5. **Add features**: Trajectory following, path smoothing

---

### 2.4 Safety Subsystem

**Location**: `autonomy_core/safety/`

**Purpose**: Prevent damage - emergency stop, monitoring, recovery

**Architecture**:
```
Multiple Input Channels:
  ├─ Remote E-stop button
  ├─ Watchdog timeout
  ├─ Obstacle detection
  ├─ Sensor failure
  └─ Power depletion
         ↓
  ┌──────────────────────┐
  │ Safety Monitor       │
  │ - Priority handling  │
  │ - State management   │
  └──────────┬───────────┘
             ↓
  E-Stop Triggered:
  1. Zero all motor commands (0ms)
  2. Activate safety lights (10ms)
  3. Log event (50ms)
  4. Await reset (manual or timeout)
```

**Key Files**:

| File | Purpose |
|------|---------|
| `emergency_stop.py` | E-stop logic and state |
| `watchdog.py` | Heartbeat monitoring |
| `safety_monitor.py` | Central safety coordinator |
| `recovery_system.py` | Auto-recovery logic |

**Emergency Stop State Machine**:

```python
class EmergencyStop:
    states = ["NORMAL", "TRIGGERED", "RECOVERING", "SAFE"]
    
    def trigger(self, reason):
        """Emergency stop activated"""
        if self.state != "TRIGGERED":
            self.state = "TRIGGERED"
            self.log_event(f"E-STOP: {reason}")
            # Immediately zero all motors
            send_motor_command(0, 0)
            # Activate safety circuit (hardware)
            hardware_safety_circuit.activate()
    
    def recover(self):
        """Attempt recovery from E-stop"""
        if self.state == "TRIGGERED":
            self.state = "RECOVERING"
            # Check if safe to recover
            if self.is_safe_to_recover():
                self.state = "NORMAL"
                return True
            else:
                self.state = "TRIGGERED"  # Back to stopped
                return False
```

**Watchdog (Detect Dead Processes)**:

```python
class Watchdog:
    def __init__(self, systems, timeout=2.0):
        self.systems = {name: {"last_heartbeat": 0} for name in systems}
        self.timeout = timeout
    
    def register_heartbeat(self, system_name):
        """Called by system to prove it's alive"""
        self.systems[system_name]["last_heartbeat"] = time.time()
    
    def check_health(self):
        """Monitor all systems"""
        now = time.time()
        for system, data in self.systems.items():
            age = now - data["last_heartbeat"]
            if age > self.timeout:
                # System is dead!
                trigger_emergency_stop(f"{system} watchdog timeout")
                return False
        return True
```

**Configuration**:
```python
config.safety.emergency_stop_enabled = True
config.safety.watchdog_timeout_s = 2.0
config.safety.auto_recovery_enabled = True
config.safety.max_recovery_attempts = 3
config.safety.recovery_timeout_s = 5.0
```

**Testing**:
```bash
python -m pytest tests/critical/test_emergency_stop.py -v
python -m pytest tests/critical/test_watchdog.py -v
python -m pytest tests/critical/test_safety_recovery.py -v
```

**Performance**:
- E-stop activation: <5ms (hardware circuit)
- Watchdog check: <1ms per system
- Recovery attempt: ~5s

**Known Limitations**:
- Watchdog only checks software heartbeats
- No hardware-level faults detected
- Recovery is basic (just retry)
- No predictive failure prevention

**Areas for Improvement**:
- [ ] Add hardware fault detection
- [ ] Implement predictive failure analysis
- [ ] Add distributed safety monitoring
- [ ] Improve recovery strategies
- [ ] Add sensor validation
- [ ] Implement safety certification

**Contribution Ideas**:
1. **Add sensor validation**: Verify sensor data before use
2. **Improve recovery**: More sophisticated recovery logic
3. **Hardware integration**: Detect motor controller failures
4. **Testing**: Add more failure scenarios
5. **Monitoring**: Better diagnostic output

---

## Part 3: How to Contribute

### Getting Started Path

```
1. READ THIS DOCUMENT (you're here!)
   ↓
2. CHOOSE AN AREA OF INTEREST
   ├─ Perception: Computer vision, SLAM, sensors
   ├─ Cognition: Decision making, planning
   ├─ Motion: Motor control, path planning
   └─ Communication: Bridges, protocols
   ↓
3. READ RELEVANT CODE
   ├─ Start with file overview above
   ├─ Look at existing tests
   └─ Understand data flow
   ↓
4. IDENTIFY IMPROVEMENT AREA
   ├─ Check "Areas for Improvement" in each section
   ├─ Review GitHub issues
   └─ Talk to team leads
   ↓
5. CREATE FEATURE/FIX
   ├─ Write tests first (TDD)
   ├─ Implement solution
   ├─ Run quality checks
   └─ Submit PR
```

### Common First Tasks

**Easy (2-4 hours)**:
- Add configuration parameters
- Improve logging/diagnostics
- Add unit tests
- Fix documentation
- Optimize imports

**Medium (4-8 hours)**:
- Tune PID gains for different terrains
- Improve detection accuracy
- Add new mission
- Enhance error messages
- Profile and optimize

**Hard (8-40 hours)**:
- Add semantic segmentation
- Implement new algorithm
- Refactor major component
- Add new sensor support
- Improve architecture

---

## Part 4: Integration Example

**Complete Flow: Sample Collection Mission**

```
TIME: T+0.0s - MISSION STARTS
  COGNITION: State = AUTONOMOUS, Mission = SAMPLE_COLLECTION
  MOTION: Velocity = (0, 0)
  
TIME: T+0.1s - PERCEPTION DETECTS SAMPLE
  PERCEPTION: Found red object at (10m, 45°)
  Publish: /vision/detections
  
TIME: T+0.2s - COGNITION PLANS
  COGNITION: "Navigate to sample, execute collection"
  Calculate target waypoints
  
TIME: T+0.3s - MOTION STARTS
  MOTION: Target velocity (1.5 m/s, 0.3 rad/s turn)
  Send CAN: Motor commands
  COMMUNICATION: Send status to dashboard
  
TIME: T+0.4-2.5s - DRIVING (2+ seconds)
  MOTION: PID control maintains velocity
  PERCEPTION: Continuous monitoring
  COMMUNICATION: Update dashboard every 100ms
  
TIME: T+2.6s - AT SAMPLE LOCATION
  PERCEPTION: Sample now visible in arm camera
  COGNITION: Execute collection behavior
  
TIME: T+2.7-3.5s - COLLECTION
  MOTION: Stop driving
  MOTION: Activate arm gripper
  PERCEPTION: Verify sample in gripper
  
TIME: T+3.6s - COLLECTED
  COGNITION: "Mission phase complete"
  COGNITION: "Navigate back to home"
  MOTION: Reverse motion to home
  
TIME: T+4.0-6.0s - RETURNING (2+ seconds)
  Repeat motion control loop
  
TIME: T+6.1s - HOME REACHED
  COGNITION: Mission complete → State = IDLE
  COMMUNICATION: Send final status
  
SUMMARY:
  ✓ Perception found sample
  ✓ Cognition planned mission
  ✓ Motion executed safely
  ✓ Communication reported status
  ✓ All systems coordinated
```

---

## Part 5: Development Environment

### Build & Test Commands

```bash
# Build
./scripts/build.sh dev                    # Development build
./scripts/build.sh prod --test           # Production with tests

# Test
python -m pytest tests/unit/ -v          # Quick unit tests
python -m pytest tests/integration/ -v   # Full integration
./scripts/check_quality.sh              # All quality checks

# Run rover
./start.py dev dashboard                 # Run with dashboard
./start.py prod autonomy                 # Production autonomy

# Development tools
ros2 topic list                          # See all topics
ros2 topic echo /topic_name              # Monitor topic
ros2 service call /service_name          # Call service
```

### File Locations Quick Reference

```
src/infrastructure/config/       Config system
src/infrastructure/bridges/      Communication
src/infrastructure/monitoring/   Health monitoring
src/autonomy/autonomy_core/perception/    Vision, SLAM
src/autonomy/autonomy_core/cognition/     Decision making
src/autonomy/autonomy_core/navigation/    Motion control
src/autonomy/autonomy_core/safety/        Safety systems
missions/                        Mission implementations
tests/unit/                      Unit tests
tests/integration/               Integration tests
tests/hardware/                  Hardware tests
config/rover.yaml                Main configuration
```

---

## Conclusion

The URC 2026 system is organized around **four technical pillars** working together:

1. **Infrastructure** - Unified configuration, communication, monitoring
2. **Perception** - Seeing the world through sensors
3. **Cognition** - Making intelligent decisions
4. **Motion** - Executing those decisions safely
5. **Safety** - Preventing damage and enabling recovery

Each pillar has:
- Clear interface/API
- Modular design for independent work
- Known limitations and improvement areas
- Existing code to build upon

**Start contributing by**:
1. Reading this document
2. Picking an area of interest
3. Reviewing the code
4. Identifying an improvement
5. Writing tests and implementing
6. Submitting a PR

The team is ready to onboard you. Start small, learn the system, grow your impact!

---

*For questions: Check docs/presentations/ for more details, or ask team members!*
