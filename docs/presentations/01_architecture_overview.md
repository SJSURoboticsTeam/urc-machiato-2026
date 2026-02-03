# URC 2026 Architecture Presentation

## System Overview

The URC 2026 Mars Rover is a complete autonomous robotics system featuring ROS2-based navigation, computer vision, mission execution, and real-time monitoring via a modern web dashboard.

## High-Level Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    WEB DASHBOARD (React)                        │
│                    (localhost:5173)                              │
│  ┌────────────────┬──────────────┬──────────────┬──────────────┐│
│  │ Mission Control│ Real-time    │ System       │ Telemetry    ││
│  │ & Monitoring   │ Monitoring   │ Status       │ Analytics    ││
│  └────────────────┴──────────────┴──────────────┴──────────────┘│
└────────────────┬────────────────────────────────────────────────┘
                 │
    ┌────────────▼────────────┐
    │  WebSocket Bridges      │
    │  (Communication Layer)   │
    └────────────┬────────────┘
                 │
┌────────────────▼──────────────────────────────────────────────────┐
│          ROS2 AUTONOMY STACK (Humble)                             │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │ Infrastructure (Unified Systems)                            │ │
│  │ ┌──────────────┬─────────────┬──────────────────────────┐   │ │
│  │ │ Configuration│ Bridges     │ Monitoring System       │   │ │
│  │ │ System       │ (CAN, WS)   │ (Health, Metrics)       │   │ │
│  │ └──────────────┴─────────────┴──────────────────────────┘   │ │
│  └──────────────────────────────────────────────────────────────┘ │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │ Autonomy Core (consolidated ROS2 package)                   │ │
│  │ ┌──────────────┬──────────────┬───────────┬──────────────┐  │ │
│  │ │ Navigation   │ Safety       │ Control   │ Perception   │  │ │
│  │ │ & Planning   │ (E-stop)     │ (Motors)  │ (Vision)     │  │ │
│  │ └──────────────┴──────────────┴───────────┴──────────────┘  │ │
│  └──────────────────────────────────────────────────────────────┘ │
│                                                                    │
│  ┌──────────────────────────────────────────────────────────────┐ │
│  │ Mission Execution                                            │ │
│  │ ┌──────────────┬──────────────┬───────────────────────────┐ │ │
│  │ │ Behavior     │ State        │ Task Coordination         │ │ │
│  │ │ Trees        │ Management   │                           │ │ │
│  │ └──────────────┴──────────────┴───────────────────────────┘ │ │
│  └──────────────────────────────────────────────────────────────┘ │
└────────────────┬──────────────────────────────────────────────────┘
                 │
    ┌────────────▼────────────┐
    │ Hardware Interfaces      │
    │ ┌──────────────────────┐ │
    │ │ Motors, Sensors,     │ │
    │ │ Cameras, IMU, GPS    │ │
    │ └──────────────────────┘ │
    └─────────────────────────┘
```

## Core Systems (Unified Architecture)

### 1. Unified Infrastructure (`src/infrastructure/`)
Single source of truth for system-wide functionality:

**Configuration System** (`infrastructure/config/`)
- Dynaconf + Pydantic for type-safe configuration
- Environment-aware settings (dev, prod, competition)
- Dynamic parameter management
- Single entry point: `get_urc_config()`

**Bridge System** (`infrastructure/bridges/`)
- WebSocket bridges for dashboard communication
- CAN bus bridges for hardware control
- Circuit breaker patterns for resilience
- Network retry logic and auto-recovery

**Monitoring System** (`infrastructure/monitoring/`)
- Real-time health status tracking
- Performance metrics collection
- System diagnostics and alerts
- Integration with Prometheus/Grafana

### 2. Autonomy Core (`src/autonomy/autonomy_core/`)
Consolidated ROS2 package with all core robotics functionality:

**Navigation Module** (`autonomy_core/navigation/`)
- Path planning (A*, Dijkstra)
- Motion control and trajectory generation
- SLAM integration
- Waypoint following

**Safety Module** (`autonomy_core/safety/`)
- Emergency stop system (E-stop)
- Watchdog monitoring
- Sensor validation
- Auto-recovery mechanisms


## PLACEHOLDER SINCE WE'LL USE FIRMWARE TEAM CODE
**Control Module** (`autonomy_core/control/`)
- Hardware interface abstraction
- Motor control drivers
- Sensor readers
- PWM/CAN command generation

**Perception Module** (`autonomy_core/perception/`)
- Computer vision processing
- Object detection (ArUco markers, samples)
- SLAM algorithms
- Sensor fusion

### 3. Mission System (`missions/`)
Task execution and sequencing:
- Mission orchestration
- Behavior tree execution
- Task completion tracking
- Error recovery


## PLACEHOLDER FOR MISSION CONTROL TEAM
### 4. Web Dashboard (`src/dashboard/`)
Modern React interface:
- Real-time monitoring (WebSocket)
- Mission control and teleoperation
- System health visualization
- Telemetry analytics

## Data Flow Architecture

```
Input Layer:
  - Sensors (IMU, GPS, Cameras, LiDAR)
  - Remote Commands (Web Dashboard)
  - Mission Plans

        ↓ ↓ ↓

Processing Layer:
  - Perception: Computer Vision, SLAM
  - Planning: Navigation, Trajectory Generation
  - State Management: Current state tracking
  - Cognition: Behavior Trees, Decision Making

        ↓ ↓ ↓

Action Layer:
  - Control Output: Motor commands via CAN/PWM
  - Hardware Actuation: Drive motors, Arm control
  - Telemetry Output: System state to dashboard

        ↓ ↓ ↓

Feedback Loop:
  - System status back to Monitoring
  - Telemetry back to Dashboard
  - State updates to Planning system
```

## Communication Patterns

### ROS2 Topics (Asynchronous Streaming)
- `/sensors/*` - Sensor data streams
- `/state/*` - System state updates
- `/control/*` - Control commands
- `/diagnostics` - System health

### ROS2 Services (Synchronous Requests)
- `/navigation/plan_path` - Path planning
- `/safety/emergency_stop` - Emergency stop
- `/mission/execute` - Start mission

### WebSocket (Frontend Communication)
- Real-time dashboard updates
- Mission commands
- System status polling
- Telemetry streaming

## Integration Points

### Hardware Integration
```
Hardware Layer (Motors, Sensors, Controllers)
        ↑
    CAN Bridge / PWM Control
        ↑
    Control Module (autonomy_core/control)
        ↑
    ROS2 Topics/Services
        ↑
    Autonomy Stack
        ↑
    WebSocket Bridge
        ↑
    Web Dashboard
```

### Configuration Flow
```
rover.yaml (git tracked)
        ↓
Dynaconf (environment aware)
        ↓
RoverConfig Pydantic Model
        ↓
Type-safe config throughout system
```

## Key Technologies

| Layer | Technology | Version |
|-------|-----------|---------|
| **Backend** | ROS2 | Humble |
| **Language** | Python | 3.10+ |
| **Config** | Dynaconf + Pydantic | Latest |
| **Frontend** | React | 18+ |
| **Frontend Lang** | TypeScript | 4.9+ |
| **Messaging** | Protocol Buffers / JSON | - |
| **Simulation** | Gazebo | 11+ |
| **Build** | colcon | Latest |
| **Testing** | pytest | 7.0+ |

## File Structure

```
src/
├── infrastructure/           # UNIFIED systems (config, bridges, monitoring)
│   ├── config/              # Type-safe configuration
│   ├── bridges/             # WebSocket, CAN, communication
│   └── monitoring/          # Health, metrics, diagnostics
│
├── autonomy/
│   ├── autonomy_core/       # CONSOLIDATED ROS2 package
│   │   ├── navigation/      # Path planning, SLAM
│   │   ├── safety/          # E-stop, watchdog
│   │   ├── control/         # Hardware interfaces
│   │   └── perception/      # Vision, object detection
│   ├── interfaces/          # ROS2 messages & services
│   └── bt/                  # Behavior tree definitions
│
├── dashboard/               # React web interface
│   ├── src/components/      # React components
│   ├── src/services/        # API/WebSocket clients
│   └── src/pages/           # Main pages
│
└── simulation/              # Gazebo integration
    ├── worlds/              # .world files
    └── models/              # Robot URDF/DAE models

missions/                    # URC mission implementations
tests/                       # Test suites (unit, integration)
config/                      # Configuration files (rover.yaml)
```

## Development Workflow

```
Feature Development:
1. Create feature branch
2. Build: ./scripts/build.sh dev
3. Make changes (follow code standards)
4. Test: python -m pytest tests/
5. Quality: pre-commit run --all-files
6. Push & create PR
7. Code review
8. Merge to main

Production Deployment:
1. Tag release: git tag -a v1.0.0
2. Build: ./scripts/build.sh prod --test
3. Package: Docker build
4. Deploy: Docker push + orchestration
```

## Next Steps

See **CONTRIBUTING.md** and **development/workflow.rst** for detailed guidance on extending the system.
