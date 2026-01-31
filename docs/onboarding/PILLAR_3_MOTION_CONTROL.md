# Pillar 3: Motion Control & Hardware - Educational Presentation

## What is Motion Control in Robotics?

### The Analogy: Your Body Executing Brain Commands

Your brain decides "walk forward", then your **body executes** it:

1. Brain sends signal: "Activate leg muscles"
2. Muscles contract with precise timing
3. Each leg moves at right speed and angle
4. You walk smoothly forward
5. Your body reports back: "Walking at 1 m/s"

A robot's **motion control system** does the same:

1. **Cognition** decides: "Drive forward 2 m/s, turn left 30 degrees"
2. **Motion controller** processes the command
3. **Hardware drivers** activate motors with correct voltages
4. **Wheels rotate** at precise speeds
5. **Sensors report back** actual motion for feedback

---

## 1. Motion Control Basics

### What is Differential Drive?

Most robots (including URC) use **differential drive** - two independent wheels:

```
FORWARD:                BACKWARD:
┌─────────────────┐    ┌─────────────────┐
│  ●────────●     │    │  ●────────●     │
│  ↑        ↑     │    │  ↓        ↓     │
│  L        R     │    │  L        R     │
└─────────────────┘    └─────────────────┘

LEFT TURN:              RIGHT TURN:
┌─────────────────┐    ┌─────────────────┐
│  ↓────────●     │    │  ●────────↓     │
│  L    (slow)  R │    │  L    (fast) R  │
│           (fast)│    │  (slow)         │
└─────────────────┘    └─────────────────┘
```

### The Math

```
If LEFT wheel speed = L m/s
If RIGHT wheel speed = R m/s

Then:
  Forward speed = (L + R) / 2
  Turning speed = (R - L) / wheelbase

Example:
  L = 1.0 m/s, R = 1.0 m/s → Go forward at 1.0 m/s
  L = 0.5 m/s, R = 1.5 m/s → Go forward + turn right
  L = 1.0 m/s, R = 0.0 m/s → Spin left
  L = -1.0 m/s, R = 1.0 m/s → Rotate in place
```

### Velocity Control

The **motion controller** takes a command and figures out wheel speeds:

```
COMMAND:
  Desired forward speed: 1.5 m/s
  Desired turn rate: 45 deg/s

CONTROLLER CALCULATES:
  Left wheel speed = 1.5 - (45° × 0.2) = 0.6 m/s
  Right wheel speed = 1.5 + (45° × 0.2) = 2.4 m/s

SEND TO MOTORS:
  Motor 1: 60% power
  Motor 2: 240% power... WAIT that's over 100%!
  
LIMIT TO MOTOR RANGE:
  Max available is 100% per motor
  So scale both down by 2.4x
  Motor 1: 25% power
  Motor 2: 100% power
  
RESULT:
  Robot moves forward slower than requested, but maintains turn rate
```

---

## 2. Motor Control: From Commands to Motion

### Signal Path

```
┌──────────────────┐
│ Motion Command   │  "Drive 1.5 m/s, turn 45°/s"
└────────┬─────────┘
         ↓
┌──────────────────┐
│ Velocity to      │  Calculate left/right wheel speeds
│ Wheel Speed      │
└────────┬─────────┘
         ↓
┌──────────────────┐
│ Motor Control    │  Convert speeds to PWM (pulse width mod)
│ (PWM Output)     │  Example: 200/255 = 78% power
└────────┬─────────┘
         ↓
┌──────────────────┐
│ Motor Driver     │  Amplify signal, handle current
│ (H-Bridge)       │  Supply correct voltage/current to motor
└────────┬─────────┘
         ↓
┌──────────────────┐
│ Motor Spins      │  ● ● ●  Wheels move!
└────────┬─────────┘
         ↓
┌──────────────────┐
│ Feedback Sensors │  Encoder tells us actual speed
│ (Encoders)       │  "Left wheel: 1.0 m/s, Right: 1.1 m/s"
└──────────────────┘
```

### Closed-Loop Control (PID)

```
GOAL: Robot should drive at exactly 1.5 m/s

REALITY: Robot only drives at 1.4 m/s (slightly too slow)

PID CONTROLLER:
  Error = Goal - Actual = 1.5 - 1.4 = 0.1 m/s (too slow)
  
  Proportional: Increase power by proportional to error
  Integral: If error persists, keep increasing
  Derivative: If error is changing fast, smooth it out
  
  New command = Proportional × error + Integral × sum(errors) + Derivative × error_rate
  
RESULT: Power adjusted upward, now robot goes 1.49 m/s

NEXT CYCLE:
  Error = 1.5 - 1.49 = 0.01 m/s (very close!)
  Adjust again slightly
  
STEADY STATE: Robot maintains 1.5 m/s ± 0.01 m/s
```

This **feedback loop** runs 100+ times per second to keep motion smooth and accurate.

---

## 3. Acceleration Control: Smooth Motion

### The Problem

```
WITHOUT acceleration limiting:
  Command: 0 → 5 m/s instantly
  Robot: LURCH! (jerky, dangerous)
  Wheels: Might slip or spin

WITH acceleration limiting:
  Command: 0 → 5 m/s over 2 seconds
  Ramp: 0 → 1 → 2 → 3 → 4 → 5 m/s
  Robot: Smooth acceleration
  Wheels: No slip, no lurch
```

### Configuration

```python
config.navigation.max_acceleration_mps2 = 1.0  # Max 1 m/s²
config.navigation.max_deceleration_mps2 = 2.0  # Can brake harder

# If robot wants to go from 0 to 5 m/s:
# Time needed = 5.0 / 1.0 = 5 seconds
# Actual motion: gradual, smooth
```

---

## 4. Hardware Architecture

### Sensor Feedback Loop

```
┌─────────────────────────────────────┐
│ MOTOR with ENCODER (Feedback)       │
│                                     │
│  ┌──────────────────┐               │
│  │ Motor            │               │
│  │ (gets voltage)   │───┐           │
│  └──────────────────┘   │           │
│                         ↓           │
│  ┌──────────────────────────────┐  │
│  │ Wheel rotates, encoder       │  │
│  │ counts rotations             │  │
│  └──────────────────────────────┘  │
│                 ↓                   │
│  ┌──────────────────────────────┐  │
│  │ Encoder sends pulses:        │  │
│  │ "I've rotated X times"       │  │
│  └──────────────────────────────┘  │
│                 ↓                   │
│  "Actual speed = rotations × wheel_circumference"
│                 ↑
└─────────────────┼──────────────────┘
                  │
            Used by PID control
            to adjust motor voltage
```

### Motor Specifications

| Property | Value | Meaning |
|----------|-------|---------|
| **Rated Voltage** | 12V | Nominal operating voltage |
| **Rated Current** | 5A | Current at full power |
| **Rated Power** | 60W | Power output at rated specs |
| **No-Load RPM** | 300 RPM | Max speed with no load |
| **Stall Torque** | 2 N·m | Pushing force when stalled |
| **Encoder** | 200 PPR | 200 pulses per rotation |

### Emergency Stop (E-Stop) Circuit

```
┌─────────────────────────────────────┐
│ Safety Circuit (Highest Priority)   │
│                                     │
│  E-Stop Button Pressed              │
│         ↓                           │
│  Safety Watchdog Triggered          │
│         ↓                           │
│  IMMEDIATELY zero all motor outputs │
│         ↓                           │
│  Motors stop within 50ms            │
│                                     │
│  Note: This MUST be hardware-level  │
│  Can't rely on software being slow  │
└─────────────────────────────────────┘
```

---

## 5. CAN Bus Communication

### What is CAN Bus?

**CAN = Controller Area Network** - a protocol for devices to talk to each other

```
Traditional Approach:
  Motor 1 ←──────────── Control System
  Motor 2 ←──────────────
  Motor 3 ←──────────────
  (Lots of wires!)

CAN Bus Approach:
  Motor 1 ─┐
  Motor 2 ├─ Single CAN Bus ←──── Control System
  Motor 3 ─┘
  (Cleaner, easier to scale)
```

### CAN Message Structure

```
Message ID: 0x100           (Address: which device?)
Data: [Speed_H, Speed_L, ... ]  (Command: what to do?)
```

### Example: Motor Speed Command

```
CAN Message:
  ID: 0x120
  Data: [0x0F, 0x9F, ...]
  
Meaning: "Motor #32, set speed to 4000 RPM"
  0x0F = Motor ID 15 (hex for 32 in firmware)
  0x9F = Speed value (encoded)
```

---

## 6. Motion Control in URC 2026

### File Structure

```
src/autonomy/autonomy_core/
├── autonomy_core/
│   ├── navigation/
│   │   ├── motion_controller.py       # Velocity → motor commands
│   │   ├── path_planner.py            # Plan path to target
│   │   └── trajectory_generator.py    # Smooth motion profile
│   │
│   └── control/
│       ├── motor_driver.py            # Talk to motors
│       ├── hardware_interface.py      # CAN/serial communication
│       └── encoder_reader.py          # Read wheel encoders

src/infrastructure/bridges/
├── can_bridge.py                      # CAN communication protocol
└── motor_command_handler.py           # Convert ROS2 to CAN
```

### Motion Controller API

```python
from src.autonomy.autonomy_core.navigation import MotionController

# Create controller
controller = MotionController(config=config.navigation)

# Set desired velocity
controller.set_velocity(
    linear_speed_mps=1.5,      # 1.5 m/s forward
    angular_speed_rads=0.5     # 0.5 rad/s (rotating left)
)

# Controller figures out motor speeds and sends commands
# (internally handles PID, acceleration limits, etc.)

# Get actual motion
actual = controller.get_current_velocity()
print(f"Actual: {actual.linear} m/s forward, {actual.angular} rad/s rotation")
```

### CAN Bridge API

```python
from src.infrastructure.bridges import CANBridge

# Initialize CAN
can = CANBridge(interface="CAN0", bitrate=1000000)  # 1 Mbps

# Send motor command
can.send_message(
    arbitration_id=0x120,  # Motor ID
    data=[0x0F, 0x9F]      # Speed command
)

# Receive feedback
msg = can.receive_message(timeout=1.0)
if msg:
    motor_id, feedback_data = msg
    speed = decode_speed(feedback_data)
    print(f"Motor {motor_id}: {speed} RPM")
```

### Motor Command Sequence

```python
# User wants to drive forward 2 meters

path_planner.plan_path(current_pos, goal_pos=(2, 0, 0))
# Returns waypoints: [(0.5, 0), (1.0, 0), (1.5, 0), (2.0, 0)]

for waypoint in waypoints:
    while not reached(waypoint):
        # Continuously adjust motion
        error = calculate_error_to_waypoint(waypoint)
        
        if error.forward_distance > 0.1:
            controller.set_velocity(linear=1.0)  # 1 m/s
        else:
            controller.set_velocity(linear=0.2)  # Slow down
        
        if error.lateral_error > 0.05:
            controller.set_velocity(angular=0.3)  # Correct course
        
        sleep(0.01)  # 100 Hz control loop
    
    # Reached waypoint, continue to next
```

---

## 7. Configuration: Tuning Motion

### Key Parameters

```python
config.navigation:
  max_linear_velocity_ms = 2.0        # 2 m/s max forward
  max_angular_velocity_rad_s = 3.14   # ~1 rotation/sec max
  max_acceleration_mps2 = 1.0         # Gradual acceleration
  max_deceleration_mps2 = 2.0         # Harder braking

config.navigation.pid:
  kp_linear = 1.5                     # Proportional gain
  ki_linear = 0.1                     # Integral gain
  kd_linear = 0.2                     # Derivative gain
  # Same for angular motion (kp_angular, etc.)

config.hardware:
  motor_count = 2                     # Two drive motors
  encoder_ppr = 200                   # Pulses per rotation
  wheel_diameter_m = 0.25             # 25 cm wheels
```

### Testing Motion

```bash
# Test motion controller
python -m pytest tests/unit/test_motion_controller.py -v

# Test CAN communication
python -m pytest tests/hardware/test_can_bridge.py -v

# Test integrated motion
python -m pytest tests/integration/test_motion_integration.py -v
```

---

## 8. Safety in Motion Control

### Watchdog: "Are You Still Responsive?"

```
Main computer: "Motor, are you there?"
  ↓
Motor: "Yes! Still here"
  ↓
Watchdog: "Good, motor is responsive"

(If motor doesn't respond for 1 second)
  ↓
Watchdog: "Motor is DEAD! EMERGENCY STOP!"
  ↓
All motors immediately stop
```

### Velocity Limits: Don't Go Too Fast

```python
# Configuration-enforced limits
max_velocity = config.navigation.max_linear_velocity_ms  # 2.0 m/s

# Even if mission asks for 5 m/s
requested = 5.0
actual = min(requested, max_velocity)  # Clipped to 2.0
```

### Acceleration Limits: Prevent Jerking

```python
# Don't jump from 0 to full speed instantly
max_accel = config.navigation.max_acceleration_mps2  # 1.0

speed_change_per_second = max_accel * 1.0 m/s²
# To reach max speed (2.0 m/s): needs 2 seconds minimum
```

---

## 9. Real-World Challenges in Motion Control

| Challenge | Problem | Solution |
|-----------|---------|----------|
| **Wheel Slip** | Wheels spin but don't move (sand/mud) | Use encoders to detect, compensate |
| **Uneven Terrain** | Robot tilts, motion is jerky | Suspension, smooth acceleration |
| **Battery Drain** | Voltage drops, motors slow | Monitor voltage, reduce speed |
| **Motor Lag** | Command arrives but motor takes time | PID control compensates |
| **Wheel Imbalance** | One wheel faster than other | Separate PID for each wheel |
| **Turning Circle** | Can't rotate in place on rough terrain | Accept rotation limitations |

---

## 10. Integration with Other Pillars

### Perception → Motion

```
Perception detects obstacle ahead
  ↓
Cognition decides: "Stop and turn right"
  ↓
Motion Control receives command: turn_rate = 0.5 rad/s
  ↓
Motors activate, robot turns
  ↓
Encoders report: "Turned 45 degrees"
  ↓
Cognition: "Obstacle cleared, resume forward"
```

---

## 11. Knowledge Check

1. **How does differential drive work?**
   - Two wheels move independently, speeds control forward/turn motion

2. **Why do we need PID control?**
   - To maintain desired speed despite variations (load, friction, etc.)

3. **What's an encoder?**
   - Sensor that counts wheel rotations to measure actual motion

4. **Why is acceleration limiting important?**
   - Prevents jerking, wheel slip, and improves control accuracy

5. **What happens in an E-stop?**
   - All motors immediately receive zero command, robot stops

---

## 12. Next Steps

- **Read the code**: `src/autonomy/autonomy_core/navigation/motion_controller.py`
- **Understand PID**: Study proportional-integral-derivative control
- **Test motion**: Run motion control tests locally
- **Tune parameters**: Adjust PID gains in configuration
- **Contribute**: Improve motion smoothness or add new control features

---

## Key Takeaways

1. **Motion control translates decisions to motion** - Bridge between cognition and hardware
2. **PID feedback is essential** - Maintains accurate motion despite variations
3. **Acceleration limits prevent problems** - Smoother, safer motion
4. **Sensor feedback is critical** - Must know actual motion to correct errors
5. **Safety is built-in** - Watchdogs, limits, emergency stop circuits

---

*Next Pillar: [Communication Systems](PILLAR_4_COMMUNICATION.md)*
