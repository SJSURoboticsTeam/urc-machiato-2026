# Pillar 4: Communication Systems - Educational Presentation

## What is Communication in Robotics?

### The Analogy: How People Communicate

```
Person A wants to tell Person B something:

1. Person A has a thought
2. Person A speaks (encodes message into sound waves)
3. Sound travels through air
4. Person B hears (receives sound)
5. Person B understands (decodes message)
6. Person B acts on information
```

A robot's **communication system** does the same:

```
System A (e.g., motor controller) wants to send data to System B (main computer):

1. System A has data (motor speed: 1000 RPM)
2. System A encodes message (converts to CAN format)
3. Message travels on physical medium (CAN bus wires)
4. System B receives (listens on bus)
5. System B decodes (parses CAN message)
6. System B uses data (adjusts main controller)
```

---

## 1. Why Communication Matters in Robotics

### The Distributed System

A robot isn't one computer - it's **many systems working together**:

```
┌──────────────────────────────────────────────────┐
│              Main Computer (ROS2)                │
│   - Perception processing                       │
│   - Mission planning                            │
│   - Decision making                             │
└────────┬─────────────┬──────────────┬───────────┘
         │             │              │
   CAN BUS             WebSocket      WiFi
         │             │              │
         ↓             ↓              ↓
    ┌────────┐   ┌──────────┐   ┌─────────────┐
    │ Motor  │   │Dashboard │   │Remote ops   │
    │Control │   │(web app) │   │(field base) │
    │(STM32) │   │(browser) │   │(laptop)     │
    └────────┘   └──────────┘   └─────────────┘
```

**Each device needs to talk to others:**
- Motor controller → Main computer: "Current speed is X"
- Main computer → Motor controller: "Set speed to Y"
- Main computer → Dashboard: "Rover status"
- Dashboard → Main computer: "User wants to start mission"

### Communication Challenges

```
IDEAL:
  Send message → Received instantly and correctly

REALITY:
  Send message → Might be delayed
             → Might arrive corrupted
             → Might not arrive at all
             → Might arrive multiple times
```

---

## 2. Communication Layers

### OSI Model Simplified for Robotics

```
Layer 7: APPLICATION   What data is being sent?
         (ROS2 Topics, WebSocket messages)
              ↓
Layer 4: TRANSPORT     How is it reliably delivered?
         (TCP, UDP, guaranteed delivery)
              ↓
Layer 2: DATA LINK     Physical connection format
         (CAN, Serial, Ethernet)
              ↓
Layer 1: PHYSICAL      Actual wires/wireless
         (Copper wire, radio signal)
```

### URC Communication Stack

```
┌───────────────────────────────────────────────┐
│ Layer 7: Application Messages                 │
│ "Motor speed command: 1500 RPM"               │
│ "Sensor reading: 45.2°C"                      │
└────────────────┬────────────────────────────┘
                 │
┌────────────────▼────────────────────────────┐
│ Layer 4: ROS2 Topics/Services                │
│ "/motor/set_velocity"                        │
│ "/sensor/temperature"                        │
└────────────────┬────────────────────────────┘
                 │
┌────────────────▼────────────────────────────┐
│ Layer 2: CAN/WebSocket Protocol              │
│ Frame: [ID][Length][Data][Checksum]          │
│ Example: 0x120 | 2 | 0x0F9F | CRC            │
└────────────────┬────────────────────────────┘
                 │
┌────────────────▼────────────────────────────┐
│ Layer 1: Physical                            │
│ CAN Bus: Two wires                           │
│ WebSocket: Ethernet cable or WiFi            │
└───────────────────────────────────────────────┘
```

---

## 3. CAN Bus Communication

### What is CAN?

**CAN = Controller Area Network** - Designed for vehicles (cars use it!)

Advantages:
- **Robust**: Noise immunity (twisted pair wires)
- **Efficient**: Cheap, simple protocol
- **Widespread**: Used in automotive, robotics, industrial
- **Real-time**: Deterministic message delivery

### CAN Message Structure

```
CAN Frame:
┌─────────┬──────────┬──────────┬──────────┬─────────┐
│ ID      │ DLC      │ Data     │ CRC      │ Flags   │
│ (11 bits)│ (4 bits) │ (8 bytes)│ (16 bits)│         │
└─────────┴──────────┴──────────┴──────────┴─────────┘

Example:
┌─────────────────────────────────────────────────────┐
│ ID: 0x120 (which device/message)                   │
│ DLC: 2 (2 data bytes following)                    │
│ Data: [0x0F, 0x9F] (motor speed: 3999)             │
│ CRC: 0x42AC (error detection)                      │
└─────────────────────────────────────────────────────┘
```

### CAN Bus Addressing

```
Different message types on same bus:

ID 0x100: Motor 1 commands
ID 0x101: Motor 2 commands
ID 0x102: Motor 3 commands
...
ID 0x200: Sensor readings
ID 0x201: Status updates

Receiver listens for messages it cares about:
┌──────────────────────────┐
│ Motor 1 Listens for:     │
│ - 0x100 (my commands)    │
│ - 0x1FF (broadcast stop) │
└──────────────────────────┘

┌──────────────────────────┐
│ Sensor reads:            │
│ - 0x200 (my data type)   │
└──────────────────────────┘
```

### CAN Message Example: Speed Command

```
Real-world: "Set motor 1 to 1500 RPM going forward"

Encoded as CAN message:
┌────────────────────────────────────┐
│ Arbitration ID: 0x120              │ ← Motor 1 command
│ Data Length Code: 8                │ ← 8 bytes
│ Byte 0: 0x01                       │ ← Motor ID (1)
│ Byte 1: 0x00                       │ ← Command type (SET_SPEED)
│ Byte 2: 0x05                       │ ← Speed high byte
│ Byte 3: 0xDC                       │ ← Speed low byte (0x05DC = 1500)
│ Byte 4: 0x01                       │ ← Direction (forward)
│ Byte 5: 0x00                       │
│ Byte 6: 0x00                       │
│ Byte 7: 0x00                       │
│ CRC: 0x7E                          │ ← Checksum
└────────────────────────────────────┘

Motor 1 receives, decodes:
Speed = (0x05DC) = 1500 RPM
Direction = forward
→ Spins motor at 1500 RPM forward
```

### URC CAN Protocol

```python
# In src/infrastructure/bridges/can_bridge.py

# Standard CAN message IDs:
MESSAGE_SET_CHASSIS_VELOCITIES = 0x100  # Main drive command
MESSAGE_HEARTBEAT = 0x101                # "I'm still here"
MESSAGE_HOMING_SEQUENCE = 0x102          # Arm initialization
MESSAGE_GET_ESTIMATED_VELOCITIES = 0x103 # Feedback request

# Message structure:
# [Motor Command] → CAN → [Motor Receives] → [Acknowledges]
```

---

## 4. ROS2 Communication

### What is ROS2?

**ROS2 = Robot Operating System 2** - Middleware for robot software

Think of it as **"the central nervous system"** of the robot:

```
┌────────────┐
│ Publisher  │  "I have sensor data"
│ (Publishes)│
└──────┬─────┘
       │
    ROS2 Topic: /sensor/temperature
       │
       ├─ Subscriber 1: Main controller (cares about temp)
       ├─ Subscriber 2: Safety system (alert if overheating)
       └─ Subscriber 3: Dashboard (display for operator)
```

### Topics: One-Way Streaming Data

```
Topic: /motor/velocity

┌──────────────────────┐
│ Motor Driver Node    │
│ (Publisher)          │
│ "My speed is 2.5 m/s"│
└──────┬───────────────┘
       │
   ROS2 Topic: /motor/velocity
   Publishes every 10ms
       │
       ├─ Navigation Node (subscribes)
       │  "I need to know actual speed for path control"
       │
       └─ Dashboard (subscribes)
          "Show speed to user"
```

### Services: Request-Response

```
Service: /motion/plan_path

┌──────────────────────┐
│ Navigation Client    │
│ (Requests)           │
│ "Plan path from      │
│  A to B"             │
└──────┬───────────────┘
       │
    ROS2 Service
       │
       ↓
┌──────────────────────┐
│ Path Planner Server  │
│ (Responds)           │
│ "Path is: [A1, A2,   │
│  A3, B]"             │
└──────────────────────┘
```

### QoS (Quality of Service) Settings

Different types of data need different reliability:

```
High-Frequency Sensor Data (IMU at 100 Hz):
  ├─ BEST_EFFORT reliability (fast, not guaranteed)
  ├─ VOLATILE durability (old data discarded)
  └─ Bandwidth: OK to lose occasional packets

Critical Commands (Motor stop):
  ├─ RELIABLE delivery (guaranteed arrival)
  ├─ TRANSIENT_LOCAL durability (last message saved)
  └─ Bandwidth: Must not lose messages

Balancing Act:
  Send too much data → Network congestion → Latency
  Send too reliably → Overhead, delays
  Send unreliably → Missed critical messages
```

---

## 5. WebSocket Communication (Frontend ↔ Backend)

### What is WebSocket?

**WebSocket** - Persistent connection for real-time communication between web browser and server

```
Traditional HTTP:
  Browser: "Give me data" → Server
  Server: [Sends data] → Browser
  → Connection closes
  → To get new data, must ask again

WebSocket:
  Browser ←→ Server (connection stays open)
  Server can send data anytime
  Browser can send data anytime
  Real-time bidirectional
```

### URC Dashboard Communication

```
┌─────────────────────────────────────────────────┐
│ Web Browser (JavaScript)                        │
│                                                 │
│ Dashboard UI:                                   │
│ - Mission status                                │
│ - Robot position on map                         │
│ - Sensor readings                               │
│ - Video feed                                    │
└─────────────────┬───────────────────────────────┘
                  │
         WebSocket Connection
         (Socket.IO library)
                  │
┌─────────────────▼───────────────────────────────┐
│ ROS2 Main Computer (Python)                     │
│                                                 │
│ WebSocket Bridge Node:                          │
│ - Listens for /robot/status topic               │
│ - Sends to all connected browsers               │
│ - Receives commands from browsers               │
│ - Publishes as ROS2 topics                      │
└─────────────────────────────────────────────────┘
```

### Example WebSocket Message

```javascript
// Browser sends command to start mission:
socket.emit("command", {
  type: "start_mission",
  mission_name: "sample_collection",
  params: { sample_location: [10, 5] }
});

// Server (ROS2) receives, publishes:
rospy.publish("/mission/command", {
  type: "start_mission",
  mission_name: "sample_collection",
  params: { sample_location: [10, 5] }
});

// Server (ROS2) receives status update:
status = subscribe_to("/robot/status")  # Position, mode, health

// Server sends to all browsers:
socket.emit("status_update", {
  position: [8.3, 4.2],
  mode: "autonomous",
  health: "healthy",
  timestamp: 1643234567
});

// Browser displays:
"Robot at (8.3, 4.2), mode: autonomous, health: healthy"
```

---

## 6. Resilience: When Communication Fails

### The Problem

```
Ideal:
  Send message → Delivered → Received → Response
  
Real world:
  Send message → LOST → No response
             OR
  Send message → DELAYED → Response too late
             OR
  Send message → CORRUPTED → Garbage data received
             OR
  Receive message → MULTIPLE COPIES → Process twice
```

### Circuit Breaker Pattern

```python
# src/infrastructure/bridges/circuit_breaker.py

class CircuitBreaker:
    states = ["CLOSED", "OPEN", "HALF_OPEN"]
    
    def __init__(self, failure_threshold=5, timeout=60):
        self.state = "CLOSED"
        self.failures = 0
        
    def call(self, function):
        if self.state == "CLOSED":
            try:
                result = function()
                self.failures = 0  # Reset on success
                return result
            except Exception as e:
                self.failures += 1
                if self.failures >= self.failure_threshold:
                    self.state = "OPEN"  # Stop trying
                raise
        
        elif self.state == "OPEN":
            # Stop sending requests, wait
            if time.time() > self.open_time + self.timeout:
                self.state = "HALF_OPEN"  # Try again cautiously
            else:
                raise CircuitBreakerOpen("Service unavailable")
        
        elif self.state == "HALF_OPEN":
            try:
                result = function()
                self.state = "CLOSED"  # Back to normal
                return result
            except Exception:
                self.state = "OPEN"  # Nope, still broken
                raise

# Usage:
breaker = CircuitBreaker()
for i in range(100):
    try:
        breaker.call(send_motor_command)
    except CircuitBreakerOpen:
        print("Motor service is down, using fallback")
```

Workflow:

```
CLOSED: Normal operation
  ↓ (5 failures in a row)
OPEN: Stop trying, wait
  ↓ (wait 60 seconds)
HALF_OPEN: Try one request
  ├─ Success → CLOSED (normal again)
  └─ Failure → OPEN (still broken)
```

### Retry Logic

```python
def send_command_with_retry(command, max_retries=3):
    for attempt in range(1, max_retries + 1):
        try:
            return send_command(command)
        except CommunicationError as e:
            if attempt == max_retries:
                raise  # Give up
            
            wait_time = 2 ** attempt  # 2, 4, 8 seconds
            print(f"Failed, retry {attempt}/{max_retries} in {wait_time}s")
            time.sleep(wait_time)

# Usage:
try:
    send_command_with_retry({"speed": 1500})
except CommunicationError:
    print("Motor command failed after retries")
```

### Message Queuing

```
Normal operation:
  Message → Send immediately → Delivered

Connection lost:
  Message 1 → Queue
  Message 2 → Queue
  Message 3 → Queue
  (Can't send, no connection)

Connection restored:
  → Send Queue[1] → Delivered
  → Send Queue[2] → Delivered
  → Send Queue[3] → Delivered
  (Catch up once reconnected)
```

---

## 7. Communication in URC 2026 Codebase

### File Structure

```
src/infrastructure/bridges/
├── can_bridge.py              # CAN bus protocol
├── websocket_bridge.py        # Web dashboard communication
├── circuit_breaker.py         # Fault tolerance
└── message_handler.py         # Encode/decode messages

src/autonomy/autonomy_core/
├── communication_node.py      # ROS2 node for all comms
└── network_manager.py         # Connection monitoring
```

### Configuration

```python
config.network:
  websocket_port = 8000                    # Port for dashboard
  websocket_host = "0.0.0.0"              # Listen on all IPs
  can_interface = "CAN0"                   # CAN port
  can_bitrate = 1000000                    # 1 Mbps
  retry_attempts = 3                       # Retry count
  retry_wait_base = 1.0                    # Seconds
  circuit_breaker_threshold = 5            # Failures before break
  circuit_breaker_timeout = 60             # Wait before retry
  heartbeat_interval = 1.0                 # Check connection every 1s
```

### Communication APIs

```python
# CAN Communication
from src.infrastructure.bridges import CANBridge

can = CANBridge(interface="CAN0")
can.send_message(arbitration_id=0x120, data=[...])
msg = can.receive_message(timeout=1.0)

# WebSocket Communication
from src.infrastructure.bridges import WebSocketBridge

ws = WebSocketBridge(host="localhost", port=8000)
ws.emit("robot_status", {"position": [1.0, 2.0]})
cmd = ws.receive_command(timeout=0.1)

# Resilience
from src.infrastructure.bridges import get_adaptive_circuit_breaker

breaker = get_adaptive_circuit_breaker("motor_control")
result = breaker.call(send_motor_command, speed=1500)
```

### Message Flow

```
DASHBOARD (Browser)
    ↓ (JSON over WebSocket)
WebSocket Bridge Node
    ↓ (Convert to ROS2)
ROS2 Topic: /commands
    ↓
Motion Controller Node
    ↓ (Convert to CAN)
CAN Bridge
    ↓ (CAN protocol)
Motor Controller (STM32)
    ↓ (Motor spins)
Feedback comes back:
    ↓ (Encoder data)
Motor Controller
    ↓ (CAN protocol)
CAN Bridge
    ↓ (Convert to ROS2)
ROS2 Topic: /status
    ↓
WebSocket Bridge
    ↓ (Convert to JSON)
DASHBOARD (Browser displays)
```

---

## 8. Testing Communication

### Test CAN Communication

```bash
python -m pytest tests/hardware/test_can_bridge.py -v
```

### Test WebSocket

```bash
python -m pytest tests/unit/test_websocket_bridge.py -v
```

### Test Resilience

```bash
python -m pytest tests/unit/test_circuit_breaker.py -v
```

---

## 9. Common Communication Challenges

| Challenge | Problem | Solution |
|-----------|---------|----------|
| **Latency** | Commands delayed reaching motor | Use real-time protocols, prioritize |
| **Packet Loss** | Message arrives corrupted | Use error detection (CRC), retry |
| **Bandwidth** | Too much data, network congestion | Compress data, lower frequency |
| **Timing Jitter** | Inconsistent delivery timing | Use priorities, reserve bandwidth |
| **Scale** | More robots = more traffic | Use IDs, multicast protocols |

---

## 10. Real-Time Constraints

### Critical Paths (Must Be Fast)

```
E-Stop Signal:
  Button → CAN → Motor controller → Motor stops
  MAX TIME: 50ms (depends on safety standards)

Obstacle Detection → Avoidance:
  LiDAR reads → Vision processes → Decision → Motor stops
  MAX TIME: 300ms (must react before hitting obstacle)

Heartbeat/Watchdog:
  Send "I'm alive" signal every 100ms
  If missed 2x, assume dead, emergency stop
```

### Non-Critical Paths (Can Be Slower)

```
Status to Dashboard:
  Update once per second (100ms latency is OK)

Data Logging:
  Once per minute to memory card (can be slow)
```

---

## 11. Knowledge Check

1. **Why does a robot need multiple communication channels?**
   - Different purposes need different protocols (speed vs. reliability)

2. **What's the purpose of a CAN ID?**
   - Address specific device/message type on the bus

3. **Why use Circuit Breaker pattern?**
   - Stop wasting resources on failing services

4. **How does WebSocket differ from HTTP?**
   - WebSocket: bidirectional, persistent connection; HTTP: one-way, per-request

5. **What happens if a critical message is lost?**
   - Circuit breaker detects, triggers fallback, system stays safe

---

## 12. Next Steps

- **Read the code**: `src/infrastructure/bridges/`
- **Understand CAN**: Study CAN protocol documentation
- **Test communication**: Run communication tests
- **Monitor connections**: Use ROS2 tools to see message flow
- **Contribute**: Improve resilience or add new protocols

---

## Key Takeaways

1. **Communication is the robot's nervous system** - Without it, systems can't coordinate
2. **Different data needs different protocols** - Speed vs. reliability tradeoff
3. **Failures WILL happen** - Design for resilience (retry, fallback, circuit breaker)
4. **Real-time matters** - Some commands must complete in milliseconds
5. **Validation is essential** - Check messages aren't corrupted before using

---

## System Integration: Putting It All Together

```
┌─────────────────────────────────────────────────────────────┐
│                    COMPLETE ROBOT SYSTEM                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  PILLAR 1: PERCEPTION (I see...)                           │
│  └─ Sensors provide information about the world            │
│     ↓                                                       │
│  PILLAR 2: COGNITION (I think...)                          │
│  └─ Decides what to do based on perception                 │
│     ↓                                                       │
│  PILLAR 3: MOTION (I move...)                              │
│  └─ Executes the decision with motors                      │
│     ↓                                                       │
│  PILLAR 4: COMMUNICATION (I tell others...)                │
│  └─ Sends status/receives commands                         │
│     ↓ (Loops back to Pillar 1)                             │
│                                                             │
│  All four pillars work together continuously:              │
│  Sense → Think → Move → Report → Sense again              │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

*Complete Pillar Overview finished! You now understand:*
- **Perception**: How robots see and understand the world
- **Cognition**: How robots make decisions
- **Motion**: How robots execute those decisions
- **Communication**: How all systems work together

*These are the four fundamental pillars of any autonomous robot!*
