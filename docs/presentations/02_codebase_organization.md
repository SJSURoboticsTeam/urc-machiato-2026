# Codebase Organization Presentation

## Repository Structure Overview

The URC 2026 codebase is organized by **functional domains** rather than traditional layered architecture, enabling independent team work while maintaining tight integration.

```
urc-machiato-2026/
├── src/                          # ROS2 source code
│   ├── infrastructure/           # CORE: Unified systems
│   │   ├── config/              # Pydantic models + Dynaconf (single source of truth)
│   │   ├── bridges/             # WebSocket, CAN communication
│   │   └── monitoring/          # Health checks, metrics
│   │
│   ├── autonomy/                # ROBOTICS: Autonomy stack
│   │   ├── autonomy_core/       # Main ROS2 package (navigation, safety, control, perception)
│   │   ├── interfaces/          # ROS2 messages & services
│   │   └── bt/                  # Behavior trees (XML definitions)
│   │
│   ├── dashboard/               # UI: Web interface
│   │   ├── src/
│   │   │   ├── components/      # React components
│   │   │   ├── services/        # WebSocket, API clients
│   │   │   └── pages/           # Route pages
│   │   └── package.json         # Node dependencies
│   │
│   └── simulation/              # TESTING: Gazebo integration
│       ├── worlds/              # .world files
│       └── models/              # Robot models (URDF, DAE)
│
├── missions/                     # TASKS: URC mission implementations
│   ├── sample_collection_mission.py
│   ├── delivery_mission.py
│   ├── waypoint_navigation_mission.py
│   └── ... (other mission files)
│
├── config/                       # SETTINGS: Configuration files
│   ├── rover.yaml               # Main rover configuration
│   ├── development.yaml          # Dev overrides
│   ├── competition.yaml          # Competition settings
│   └── calibration_manager.yaml  # Calibration data
│
├── tests/                        # VALIDATION: Test suites
│   ├── unit/                    # Unit tests (individual components)
│   ├── integration/             # Integration tests (component interactions)
│   ├── hardware/                # Hardware-in-the-loop tests
│   ├── performance/             # Performance benchmarks
│   ├── critical/                # Safety-critical tests
│   └── conftest.py              # Shared fixtures
│
├── scripts/                      # AUTOMATION: Build & deployment
│   ├── build.sh                 # Unified build system
│   ├── run_tests.py             # Smart test runner
│   ├── check_quality.sh         # Quality assurance
│   ├── deployment/              # Deployment scripts
│   └── ... (utility scripts)
│
├── docker/                       # CONTAINERS: Docker configuration
│   ├── Dockerfile.prod          # Production image
│   ├── Dockerfile.universal     # Multi-use image
│   ├── docker-compose.yml       # Service orchestration
│   └── ... (other configs)
│
├── docs/                         # DOCUMENTATION: Sphinx + markdown
│   ├── onboarding/              # Onboarding by specialization
│   ├── architecture/            # System design
│   ├── api/                     # API reference
│   ├── operations/              # Deployment & ops
│   ├── presentations/           # This folder (overview materials)
│   └── index.rst                # Sphinx TOC
│
├── tools/                        # UTILITIES: Development tools
│   ├── calibration/             # Camera/hardware calibration
│   ├── monitoring/              # System monitoring tools
│   ├── diagnostics/             # Diagnostic utilities
│   └── ...
│
├── vendor/                       # SUBMODULES: External components
│   ├── teleoperation/           # Web teleoperation interface
│   └── control-systems/         # STM32 control firmware
│
├── pyproject.toml               # Python project config
├── setup.cfg                    # Setup configuration
├── requirements.txt             # Python dependencies
├── AGENTS.md                    # AI agent guidelines
├── README.md                    # Project README
└── DEPLOYMENT.md                # Deployment guide
```

## Key Directories Explained

### `src/infrastructure/` - Unified Systems

**Purpose**: Single source of truth for cross-cutting concerns

**Contents**:
```
infrastructure/
├── config/
│   ├── schemas.py           # Pydantic models defining all config
│   ├── settings.py          # Dynaconf configuration loading
│   ├── validators.py        # Config validation logic
│   └── __init__.py
│
├── bridges/
│   ├── websocket_bridge.py  # Dashboard ↔ backend communication
│   ├── can_bridge.py        # Hardware ↔ backend communication
│   ├── circuit_breaker.py   # Network resilience patterns
│   └── __init__.py
│
└── monitoring/
    ├── health_monitor.py    # System health tracking
    ├── metrics.py           # Performance metrics
    ├── diagnostics.py       # System diagnostics
    └── __init__.py
```

**Usage**: Import from this package for system-wide configuration and bridging
```python
from src.infrastructure.config import get_urc_config
from src.infrastructure.bridges import WebSocketBridge
```

### `src/autonomy/autonomy_core/` - Core Robotics

**Purpose**: All robotics logic in one consolidated ROS2 package

**Structure**:
```
autonomy_core/
├── autonomy_core/              # Python package
│   ├── navigation/             # Path planning, motion control
│   │   ├── path_planner.py
│   │   ├── motion_controller.py
│   │   └── ...
│   │
│   ├── safety/                 # Emergency stop, watchdog
│   │   ├── emergency_stop.py
│   │   ├── watchdog.py
│   │   └── ...
│   │
│   ├── control/                # Hardware interfaces
│   │   ├── motor_driver.py
│   │   ├── sensor_reader.py
│   │   └── ...
│   │
│   ├── perception/             # Vision, SLAM, sensors
│   │   ├── vision_processor.py
│   │   ├── slam_node.py
│   │   └── ...
│   │
│   ├── navigation_node.py      # Main ROS2 node
│   ├── safety_node.py
│   └── __init__.py
│
├── launch/                     # ROS2 launch files
│   ├── integrated_system.launch.py
│   └── ...
│
└── package.xml                 # ROS2 package definition
```

**Key Files**:
- `autonomy_core_node.py` - Entry point, node initialization
- `package.xml` - ROS2 package metadata
- `launch/*.py` - Launch file definitions

### `src/dashboard/` - Web Frontend

**Purpose**: React/TypeScript web interface for monitoring and control

**Structure**:
```
dashboard/
├── src/
│   ├── components/             # React components
│   │   ├── MissionControl.tsx
│   │   ├── SystemStatus.tsx
│   │   ├── TelemetryChart.tsx
│   │   └── ...
│   │
│   ├── services/               # API clients
│   │   ├── websocket_client.ts
│   │   ├── api_client.ts
│   │   └── ...
│   │
│   ├── pages/                  # Route pages
│   │   ├── Dashboard.tsx
│   │   ├── Monitor.tsx
│   │   └── ...
│   │
│   ├── App.tsx                 # Root component
│   ├── index.tsx               # Entry point
│   └── ...
│
├── package.json                # Node.js dependencies
├── tsconfig.json               # TypeScript configuration
├── vite.config.ts              # Vite build config
└── ...
```

### `missions/` - Mission Implementations

**Purpose**: Task-specific logic for URC challenges

**Files**:
```
missions/
├── sample_collection_mission.py      # Science task
├── delivery_mission.py               # Delivery task
├── waypoint_navigation_mission.py    # Navigation task
├── autonomous_keyboard_mission.py    # Equipment service
├── follow_me_mission.py              # Follow task
├── base_mission.py                   # Mission base class
└── README.md
```

**Pattern**: Each mission inherits from `BaseMission` and implements:
- `plan()` - Generate task plan
- `execute()` - Execute the task
- `recover()` - Handle errors

### `tests/` - Test Organization

**Organization**:
```
tests/
├── unit/                       # 60+ test files
│   ├── test_navigation.py
│   ├── test_safety.py
│   ├── test_config.py
│   └── ...
│
├── integration/                # 74+ test files
│   ├── test_system_integration.py
│   ├── test_mission_execution.py
│   └── ...
│
├── hardware/                   # Hardware-in-the-loop
│   ├── test_motor_control.py
│   ├── test_sensor_reading.py
│   └── ...
│
├── performance/                # Performance benchmarks
│   ├── test_navigation_performance.py
│   └── ...
│
├── critical/                   # Safety-critical
│   ├── test_emergency_stop.py
│   ├── test_safety_recovery.py
│   └── ...
│
├── conftest.py                 # Pytest fixtures
├── factories/                  # Test data factories
├── fixtures/                   # Test fixtures
└── run_comprehensive_testing.py
```

**Minimum Coverage**: 85% for CI/CD pipeline

### `config/` - Configuration Files

**Files**:
```
config/
├── rover.yaml                  # Main config (git tracked)
├── development.yaml            # Dev overrides
├── competition.yaml            # Competition mode settings
├── calibration_manager.yaml    # Calibration data
└── ...
```

**Loading**: Via `infrastructure/config` - Dynaconf + Pydantic

## Code Navigation Guide

### Finding Code by Task

| I want to... | Look in... | Key file |
|--------------|-----------|----------|
| **Change robot behavior** | `src/autonomy/autonomy_core/` | `navigation_node.py`, `safety_node.py` |
| **Add a new mission** | `missions/` | Create `new_mission.py` |
| **Modify web UI** | `src/dashboard/src/` | Component `.tsx` files |
| **Configure system** | `src/infrastructure/config/` | `schemas.py` for types, `settings.py` for loading |
| **Add hardware interface** | `src/autonomy/autonomy_core/control/` | Add driver file |
| **Write test** | `tests/` | Match the feature location |
| **Add ROS2 message** | `src/autonomy/interfaces/msg/` | Create `.msg` file |
| **Modify system monitoring** | `src/infrastructure/monitoring/` | `health_monitor.py` |

### Import Patterns

```python
# Infrastructure (unified systems)
from src.infrastructure.config import get_urc_config
from src.infrastructure.bridges import WebSocketBridge
from src.infrastructure.monitoring import HealthMonitor

# Autonomy core
from src.autonomy.autonomy_core.navigation import PathPlanner
from src.autonomy.autonomy_core.safety import EmergencyStop
from src.autonomy.autonomy_core.control import MotorDriver
from src.autonomy.autonomy_core.perception import VisionProcessor

# Missions
from missions.base_mission import BaseMission
from missions.sample_collection_mission import SampleCollectionMission

# Testing
from tests.factories import NavigationConfigFactory
from tests.conftest import mock_ros_node
```

## Dependency Flow

```
External Layer (Hardware, Sensors, WebSocket Clients)
        ↓
Infrastructure Layer (Configuration, Bridges, Monitoring)
        ↓
Autonomy Core Layer (Navigation, Safety, Control, Perception)
        ↓
Mission Layer (Task Sequencing)
        ↓
Dashboard Layer (Web UI)
```

**Key Principle**: Infrastructure provides cross-cutting services; Autonomy Core handles robotics; Missions orchestrate tasks; Dashboard visualizes state.

## Development Principles

1. **Modularity**: Each package has a single, clear responsibility
2. **Interface Segregation**: Minimal dependencies between packages
3. **Configuration-Driven**: Use unified config for all parameters
4. **Testing-First**: Write tests before implementation
5. **Type Safety**: Use Python type hints throughout
6. **Documentation**: Docstrings for all public functions/classes

## Next Steps

- Review the codebase: Start in `src/autonomy/autonomy_core/`
- Check examples: See `docs/onboarding/examples/`
- Run tests: `python -m pytest tests/unit/ -v`
- Build: `./scripts/build.sh dev`
