# Pillar 2: Cognition & Decision Making - Educational Presentation

## What is Cognition in Robotics?

### The Analogy: Your Brain Making Decisions

Think about how YOU solve a problem:

1. **You perceive** the situation (see an object blocking your path)
2. **You think** about options (go around it, jump over it, ask for help)
3. **You decide** what to do (choose the best option)
4. **You act** on your decision (go around)
5. **You monitor** the result (did it work? adjust if needed)

A robot's **cognition system** does the same thing:
- **Perceive**: Get information from sensors (already covered in Pillar 1)
- **Think**: Process information, make decisions
- **Decide**: Choose the next action based on goals and constraints
- **Act**: Execute the chosen action (covered in Pillar 3)
- **Monitor**: Track if the action succeeded

---

## 1. What is a State Machine?

### Real-World Analogy: A Light Switch

```
         ┌─────────────────┐
         │                 │
         ↓                 │
    ┌─────────┐    OFF    ┌─────────┐
    │         │◄─────────►│         │
    │   ON    │    ON     │  OFF    │
    │         │◄─────────►│         │
    └─────────┘          └─────────┘
         ↑                 ↓
         │                 │
         └─────────────────┘
```

A state machine has **states** (ON, OFF) and **transitions** between them based on **events** (switch flipped).

### Robot State Machine Example

```
                    POWER UP
                       ↓
               ┌─────────────────┐
               │     BOOT        │
               │ - Initialize    │
               │ - Check systems │
               └────────┬────────┘
                        ↓
               ┌─────────────────┐
               │     IDLE        │ ◄─────────┐
               │ - Ready to act  │           │
               │ - Wait for cmd  │           │
               └────┬─────┬──────┘           │
                    │     │                  │
                    │     └─ MISSION DONE ──→┤
        MANUAL CMD  │
                    │     ┌─ MANUAL COMMAND ─┐
                    ↓     │                   ↓
            ┌────────────────────┐    ┌──────────────────────┐
            │  TELEOPERATION     │    │  AUTONOMOUS MISSION  │
            │ - User controls    │    │ - Execute mission    │
            │ - Real-time        │    │ - Follow plan        │
            └───────┬────────────┘    └──────┬───────────────┘
                    │                       │
                    │  E-STOP PRESSED       │
                    ↓       ↓               ↓
               ┌─────────────────────┐
               │  EMERGENCY STOP     │
               │ - All motors stop   │
               │ - Wait for reset    │
               └─────────┬───────────┘
                         │
                    RESET COMMAND
                         │
                         ↓
                      (back to IDLE)
```

### States in URC

| State | Meaning | What Happens | Next States |
|-------|---------|-------------|-------------|
| **BOOT** | System starting | Initialize hardware, run diagnostics | → IDLE or ERROR |
| **IDLE** | Waiting for command | Ready but not doing anything | → TELEOPERATION, AUTONOMOUS, SHUTDOWN |
| **AUTONOMOUS** | Running mission | Execute pre-planned tasks | → IDLE, EMERGENCY_STOP |
| **TELEOPERATION** | Human control | Operator drives via remote | → IDLE, EMERGENCY_STOP |
| **EMERGENCY_STOP** | All stop | Motors zero, await reset | → IDLE (after reset) |
| **ERROR** | Something failed | Diagnostic mode, log error | → BOOT (to restart) |

### Why State Machines Matter

A state machine **ensures the robot never does conflicting things**:

```
BAD (No state machine):
- User asks to drive forward manually
- Mission system asks to drive left autonomously
- Motors get conflicting commands → CRASH!

GOOD (With state machine):
- State = TELEOPERATION → only manual commands work
- State = AUTONOMOUS → only mission commands work
- Cannot be in two states at once → no conflict
```

---

## 2. What is a Behavior Tree?

### Thinking Beyond Simple States

State machines are great for *mode* (teleoperation vs. autonomous), but what about *mission logic*?

**Challenge**: How do you decide between:
- "Drive to waypoint 1"
- "Drive to waypoint 2"
- "If waypoint 1 fails, try waypoint 2"
- "Keep trying until timeout"

Answer: **Behavior Trees** (BTs)

### What is a Behavior Tree?

A behavior tree is a **flowchart for robot tasks** with three types of nodes:

```
┌──────────────────────────────────────────┐
│ SELECTOR (try one option)                │
│ Returns SUCCESS if ANY child succeeds    │
│                                          │
│  ┌──────────┐  ┌──────────┐              │
│  │ Option A │  │ Option B │              │
│  └──────────┘  └──────────┘              │
└──────────────────────────────────────────┘
         (Try Option A. If fails, try Option B)


┌──────────────────────────────────────────┐
│ SEQUENCE (do all in order)               │
│ Returns SUCCESS only if ALL children     │
│ succeed in order                         │
│                                          │
│  ┌──────────┐  ┌──────────┐              │
│  │ Step 1   │→ │ Step 2   │              │
│  └──────────┘  └──────────┘              │
└──────────────────────────────────────────┘
         (Do Step 1, then Step 2. Both must work.)


┌──────────────────────────────────────────┐
│ ACTION (do something)                    │
│ Performs actual robot action             │
│                                          │
│  "Move forward 5 meters"                 │
│  "Pick up object"                        │
│  "Check if object is red"                │
└──────────────────────────────────────────┘
```

### Building a Mission with BT

```
MISSION: Collect sample and bring it back home

                    ROOT
                     │
        ┌────────────┴────────────┐
        │   COLLECT & RETURN      │
        │     (SEQUENCE)          │
        └────────────┬────────────┘
                     │
        ┌────────────┼────────────┐
        │            │            │
        ↓            ↓            ↓
    MOVE TO       COLLECT       MOVE TO
    SAMPLE     (SEQUENCE)       HOME
     │             │             │
     │        ┌────┴────┐        │
     │        │         │        │
     │        ↓         ↓        │
     │      ARM      VERIFY      │
     │      UP       SAMPLE      │
     │                           │
     └───────────────┬───────────┘
                     │
                  SUCCESS!
```

### How BT Works

```python
def run_tree(node):
    if node is SELECTOR:
        for child in node.children:
            result = run_tree(child)
            if result == SUCCESS:
                return SUCCESS  # Got one success, we're done
            # Otherwise try next child
        return FAILURE  # All children failed
        
    elif node is SEQUENCE:
        for child in node.children:
            result = run_tree(child)
            if result != SUCCESS:
                return FAILURE  # One child failed, stop sequence
        return SUCCESS  # All succeeded
        
    elif node is ACTION:
        return execute_action(node)  # Do the actual work
```

### Behavior Tree Advantages

- **Flexible**: Easy to add "try alternative" logic
- **Readable**: Visual representation matches task logic
- **Reusable**: Build complex missions from simple blocks
- **Debuggable**: Can see exactly where mission fails

---

## 3. How Do State Machine and Behavior Tree Work Together?

### The Hierarchy

```
┌─────────────────────────────────────┐
│   STATE MACHINE (System Mode)       │
│                                     │
│   IDLE │ TELEOPERATION │ AUTONOMOUS │
│                                     │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│   BEHAVIOR TREE (Task Logic)        │
│                                     │
│   Within AUTONOMOUS mode:           │
│   - Select mission                  │
│   - Execute steps                   │
│   - Handle failures                 │
│                                     │
└─────────────────────────────────────┘
              ↓
┌─────────────────────────────────────┐
│   MOTION CONTROL (Actions)          │
│                                     │
│   Drive motor, read sensor, etc.    │
│                                     │
└─────────────────────────────────────┘
```

### Real Example: Sample Collection

```
State Machine: AUTONOMOUS MISSION
  ↓
Behavior Tree:
  SEQUENCE:
    1. Navigate to sample location
       ├─ Get sample position from perception
       ├─ Plan path to sample
       └─ Execute path (drive toward it)
    
    2. Collect sample
       ├─ Position arm above sample
       ├─ Close gripper
       └─ Verify sample captured
    
    3. Return home
       ├─ Plan path back to start
       └─ Execute path
  ↓
Motion Control:
  - Send velocity commands to motors
  - Read IMU for orientation
  - Read distance sensors for obstacles
```

---

## 4. The Perception → Cognition → Action Loop

### Real-Time Decision Loop

```
Cycle Time: ~100 ms (10 Hz decision rate)

START OF CYCLE
  │
  ├─ PERCEPTION (30 ms)
  │  └─ Read sensors, get robot position, detect obstacles
  │
  ├─ COGNITION (40 ms)
  │  ├─ Where am I?
  │  ├─ Where do I want to go?
  │  ├─ What obstacles are in the way?
  │  └─ What should I do next?
  │
  ├─ ACTION (20 ms)
  │  ├─ Send velocity commands
  │  ├─ Move gripper
  │  └─ Publish status
  │
  └─ REPEAT
```

This loop runs continuously. Every 100ms, the robot **senses, thinks, and acts**.

---

## 5. Decision Making Example: Obstacle Avoidance

### Simple Decision Tree

```
Sensor reports: "Obstacle 1 meter ahead"

          Obstacle ahead?
                 │
          ┌──────┴──────┐
        YES              NO
         │               │
         ↓               ↓
    Can stop   Continue forward
    in time?
         │
    ┌────┴────┐
   YES        NO
    │          │
    ↓          ↓
  STOP    Try alternative path
              │
          ┌───┴────┐
        left        right
         │          │
    ┌────┴──┐   ┌───┴────┐
  Can go?  No Can go?   No
    │       │   │        │
    ↓       ↓   ↓        ↓
  Go L.  REVERSE Go R.  REVERSE
```

### Code Example

```python
class ObstacleAvoider:
    def decide_next_action(self, sensor_data):
        if not sensor_data.obstacle_ahead:
            return "MOVE_FORWARD"
        
        # Obstacle detected
        stopping_distance = calculate_stopping_distance()
        if sensor_data.distance_to_obstacle > stopping_distance:
            return "MOVE_FORWARD"  # Can still go
        
        # Must avoid
        left_clear = sensor_data.left_distance > MIN_CLEARANCE
        right_clear = sensor_data.right_distance > MIN_CLEARANCE
        
        if left_clear:
            return "TURN_LEFT"
        elif right_clear:
            return "TURN_RIGHT"
        else:
            return "REVERSE"  # Last resort
```

---

## 6. Cognition in the URC 2026 Codebase

### File Structure

```
src/autonomy/autonomy_core/
├── autonomy_core_node.py           # Main ROS2 node
├── state_management.py             # State machine
├── behavior_tree_executor.py       # BT engine
└── missions/
    ├── sample_collection.py        # Sample mission logic
    ├── delivery.py                 # Delivery mission
    └── navigation.py               # Waypoint mission

src/core/
├── state_management.py             # State machine classes
├── adaptive_state_machine.py       # ROS2-aware state machine
└── unified_blackboard_client.py    # Shared memory for BT
```

### State Management API

```python
from src.core.state_management import StateManager

# Create state machine
sm = StateManager()
sm.add_state("IDLE")
sm.add_state("MISSION")
sm.add_state("EMERGENCY_STOP")

# Register transitions
sm.add_transition("IDLE", "MISSION", trigger="start_mission")
sm.add_transition("MISSION", "IDLE", trigger="mission_done")
sm.add_transition("*", "EMERGENCY_STOP", trigger="estop")  # From any state

# Use state machine
sm.trigger("start_mission")  # IDLE → MISSION
current = sm.get_state()    # "MISSION"
```

### Behavior Tree API

```python
from src.autonomy.bt import BehaviorTree

# Create BT from XML or programmatically
bt = BehaviorTree("sample_collection")

# Add nodes
root = Sequence("Main")
root.add_child(Action("navigate_to_sample"))
root.add_child(Action("collect_sample"))
root.add_child(Action("return_home"))

# Execute
status = bt.execute()  # SUCCESS, FAILURE, or RUNNING
```

### Decision Making Pseudocode

```python
def mission_loop():
    while robot_running:
        # PERCEIVE
        perception = get_perception()  # Position, obstacles, objects
        
        # COGNITION
        state = state_machine.current_state
        
        if state == "AUTONOMOUS":
            # Run behavior tree
            bt_status = behavior_tree.execute()
            
            if bt_status == "SUCCESS":
                state_machine.trigger("mission_complete")
            elif bt_status == "FAILURE":
                state_machine.trigger("mission_failed")
        
        # ACTION
        execute_next_move()
        
        # Wait for next cycle
        sleep(0.1)  # 100 ms cycle
```

### Configuration

```python
from src.infrastructure.config import get_urc_config

config = get_urc_config()

# Decision-making parameters
config.cognition.behavior_tree_tick_rate  # Hz
config.cognition.decision_timeout         # seconds
config.cognition.max_retries             # times to retry failed action
config.cognition.fallback_strategy       # What to do if mission fails
```

---

## 7. Common Decision-Making Patterns

### Pattern 1: Retry on Failure

```
Try action
  ↓
Success? → YES → Move on
  ↓
  NO
  ↓
Retries left? → NO → Fail
  ↓
  YES
  ↓
Try again (go back to "Try action")
```

### Pattern 2: Fallback Option

```
Try first approach
  ↓
Success? → YES → Done
  ↓
  NO
  ↓
Try alternative approach
  ↓
Success? → YES → Done
  ↓
  NO
  ↓
Fail
```

### Pattern 3: Conditional Logic

```
Check condition
  ↓
Condition true? → YES → Do action A
  ↓
  NO
  ↓
Do action B
```

### Pattern 4: Parallel Tasks

```
Do A
└─ Do B at the same time
  └─ Do C at the same time
```

---

## 8. Testing Decision-Making

### Test a Decision

```bash
# Test state machine
python -m pytest tests/unit/test_state_machine.py -v

# Test behavior tree
python -m pytest tests/unit/test_behavior_tree.py -v

# Test mission logic
python -m pytest tests/integration/test_sample_collection_mission.py -v
```

### Example Test

```python
def test_obstacle_avoidance_decision():
    # Setup
    robot = RobotSimulator()
    obstacle_data = SensorData(obstacle_ahead=True, distance=0.5)
    
    # Execute
    decision = robot.decide_next_action(obstacle_data)
    
    # Verify
    assert decision in ["TURN_LEFT", "TURN_RIGHT", "REVERSE"]
    assert decision != "MOVE_FORWARD"  # Don't drive into obstacle!
```

---

## 9. Real-World Challenges in Decision-Making

| Challenge | Problem | URC Solution |
|-----------|---------|------------|
| **Incomplete Info** | Don't know what's beyond next hill | Use SLAM, explore cautiously |
| **Time Pressure** | Mission must complete in 30 minutes | Plan efficiently, parallelize tasks |
| **Uncertainty** | Sensor might be wrong | Use multiple sensors, require confirmation |
| **Dynamic Env** | Another robot moved the sample | Re-plan when initial plan fails |
| **Compute Limits** | Can't run complex algorithm every 10ms | Pre-compute plans, use approximations |

---

## 10. Knowledge Check

1. **What's the purpose of a state machine?**
   - Ensure robot doesn't do conflicting actions simultaneously

2. **How is a behavior tree different from if/else statements?**
   - BT is visual, hierarchical, and easier to modify than code

3. **Why does cognition need perception?**
   - Can't make good decisions without knowing what's happening

4. **What happens if cognition is too slow?**
   - Robot can't react to unexpected events, crashes into obstacles

5. **How do you add a new decision type to the system?**
   - Add new BT node type or state machine transition

---

## 11. Next Steps

- **Read the code**: `src/core/state_management.py`, `src/autonomy/autonomy_core/`
- **Understand BTs**: Study `src/autonomy/bt/` XML definitions
- **Write a mission**: Create new mission in `missions/`
- **Test decisions**: Add unit tests for your decision logic
- **Contribute**: Improve mission planning or add new behaviors

---

## Key Takeaways

1. **State machine defines MODE** - What capability is active (auto vs. manual)
2. **Behavior tree defines MISSION** - What specific task is being done
3. **Both enable safe operation** - No conflicting commands
4. **Decisions happen in real-time** - Must be fast enough to react
5. **Perception informs cognition** - Bad sensing leads to bad decisions

---

*Next Pillar: [Motion Control & Hardware](PILLAR_3_MOTION_CONTROL.md)*
