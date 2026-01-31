# URC Machiato 2026 - University Rover Challenge

An autonomous Mars rover system for the University Rover Challenge 2026, featuring ROS2-based navigation, computer vision, SLAM, and mission execution with enterprise-grade unified systems architecture.

## 🗺️ System Overview

**New to the project?** Start here to understand how everything fits together:

![System Architecture](docs/architecture/diagrams/high/04_system_component_architecture.png)

### Unified Systems Architecture

The URC 2026 system is built on 5 unified, enterprise-grade systems that provide consistent APIs across all components:

#### 🧪 **Unified Test Suite** (`src/core/test_suite.py`)
- 140+ test files consolidated into 1 comprehensive testing framework
- Test data factories, mock management, performance benchmarking
- **75% code reduction** from scattered test files

#### 📊 **Unified Data Manager** (`src/core/data_manager.py`)
- High-performance data processing, validation, and analytics
- JSON schema validation, statistical analysis, coordinate transformations
- **60% code reduction** from 6 separate data systems

#### 🛠️ **Unified Utilities** (`src/core/utilities.py`)
- Safety monitoring, hardware validation, recovery coordination
- Network resilience, system utilities, emergency handling
- **50% code reduction** from 4 separate utility systems

#### 🤖 **Unified State Management** (`src/core/state_management.py`)
- Hierarchical state machines and behavior trees
- Real-time state synchronization and persistence
- **45% code reduction** from 3 separate state systems

#### 📈 **Unified Observability** (`src/core/observability.py`)
- Structured logging, Prometheus metrics, health monitoring
- Performance profiling and real-time dashboards
- **50% code reduction** from 4 separate monitoring systems

**Total Code Reduction: 56%** across all unified systems!

📖 **[Read the Unified Systems Guide](docs/unified_systems.rst)** for detailed documentation

The system consists of 6 interconnected layers:
- **🎯 Missions**: Sample collection, delivery, autonomous keyboard
- **🤖 Autonomy**: Navigation, state machines, safety systems
- **👁️ Perception**: Computer vision, SLAM, sensor processing
- **🎮 Control**: Motor control, hardware interfaces
- **🌉 Communication**: ROS2 messaging, network resilience
- **💻 Interface**: Web dashboard, real-time monitoring

📖 **[Read the Big Picture Guide](docs/big_picture.rst)** for a detailed overview

**Team Member?** Jump to your specialization (see [Documentation](#-documentation) below):
- [Onboarding overview](docs/onboarding/README.md) and pillar guides (Perception, Cognition, Motion Control, Communication)
- [Network Guide](docs/network_guide.rst) | [SLAM/NAV Guide](docs/slam_nav_guide.rst) | [ARM Guide](docs/arm_guide.rst) | [Testing Guide](docs/testing_guide.rst)

## 🚀 Quick Start

### Prerequisites
- Python 3.10+
- ROS2 Humble/Jazzy
- Ubuntu 22.04 LTS (recommended)

### One-Command Launch

```bash
# Development frontend only
./start.py dev frontend

# Testing dashboard (backend + frontend)
./start.py dev dashboard

# Full autonomy system
./start.py prod autonomy

# Gazebo simulation
./start.py dev simulation
```

### Build & Test
```bash
# Development build
./scripts/build.sh dev --test

# Quality check (format + lint + test)
./scripts/check_quality.sh

# Run specific tests
python -m pytest tests/unit/ -v
python -m pytest path/to/test_file.py::test_name
```

## 📋 Requirements

- **Python**: 3.10+
- **ROS2**: Humble/Jazzy
- **Node.js**: 18+ (for dashboard)
- **Docker**: 24.0+ (optional, recommended)
- **System**: Ubuntu 22.04 LTS or equivalent

## 🏗️ Architecture

### Core Components
- **Autonomy System**: ROS2-based navigation and control with safety layers
- **Computer Vision**: Object detection, ArUco tracking, scene understanding
- **SLAM**: Real-time localization and mapping
- **Mission Control**: Behavior trees for task execution and coordination
- **Web Dashboard**: React/TypeScript interface for monitoring and control
- **Safety Systems**: Multi-layer redundant safety monitoring
- **Hardware Interfaces**: STM32-based control systems (via submodules)

### Unified Systems
56% code reduction through enterprise-grade unified components:
- 🧪 **Unified Test Suite** - 140+ tests consolidated
- 📊 **Unified Data Manager** - High-performance data processing
- 🛠️ **Unified Utilities** - Safety, hardware validation, recovery
- 🤖 **Unified State Management** - Hierarchical state machines
- 📈 **Unified Observability** - Structured logging and metrics

## 📦 Installation

### Quick Install (Recommended)

```bash
# Clone repository with submodules
git clone --recurse-submodules https://github.com/SJSURoboticsTeam/urc-machiato-2026.git
cd urc-machiato-2026

# If not cloned with --recurse-submodules, initialize submodules
git submodule update --init --recursive

# Install Python dependencies
pip install -e .

# For development (includes testing tools)
pip install -e ".[dev]"

# For documentation
pip install -e ".[docs]"

# Build ROS2 packages
./scripts/build.sh dev

# Setup submodules
./scripts/manage_submodules.sh update
```

### Development Commands

```bash
# Format and lint code
black . && ruff check --fix . && mypy .

# Frontend development (in src/dashboard/)
npm run dev      # Development server
npm run test     # Run tests
npm run build    # Production build

# Docker deployment
docker-compose -f docker-compose.prod.yml up -d
docker-compose ps
```

## 🚀 Usage

### Basic Operation

#### Unified Launcher

Use the single `./start.py` command for all components:

```bash
# Frontend development server
./start.py dev frontend

# Testing dashboard (includes backend WebSocket server)
./start.py dev dashboard

# Full autonomy system with ROS2
./start.py prod autonomy

# Gazebo simulation
./start.py dev simulation
```

#### Manual ROS2 Commands (Advanced)

```bash
# Validate configuration
python scripts/validate_config.py

# Direct ROS2 launches (for advanced users)
ros2 launch autonomy/launch/integrated_system.launch.py
ros2 launch autonomy/launch/mission_system.launch.py
ros2 launch autonomy/launch/rover_simulation.launch.py

# Monitor system
ros2 topic echo /mission/status
```

### Web Interface

Access the web interface at `http://localhost:5173` for real-time monitoring and control.

### Submodule Management

```bash
# Update submodules to latest upstream
./scripts/manage_submodules.sh update

# Check submodule status
./scripts/manage_submodules.sh status

# View integration instructions
./scripts/manage_submodules.sh integration
```

## 📁 Directory Structure

### 🔧 Core Development Areas

| Directory | Purpose | Key Contents |
|-----------|---------|--------------|
| **`src/`** | ROS2 packages & source code | `autonomy/`, `bridges/`, `src/dashboard/`, `simulation/` |
| **`missions/`** | URC mission implementations | Sample collection, delivery, navigation missions |
| **`simulation/`** | Gazebo simulation environment | World files, robot models, simulation tools |
| **`config/`** | Configuration files | `rover.yaml`, environment configs |
| **`tests/`** | Test suites | Unit tests, integration tests, fixtures |

### 🏗️ ROS2 Source Code (`src/`)

#### `src/autonomy/` - Core Robotics Stack
```
autonomy/
├── bt/                 # Behavior Tree implementations
├── control/            # Hardware control (LED, motors)
├── core/               # Core autonomy (navigation, state, safety)
├── interfaces/         # ROS2 messages, services, actions
├── perception/         # Computer vision, SLAM, sensors
└── utilities/          # Shared utilities
```

#### `src/bridges/` - Communication Layer
- **WebSocket bridges** for dashboard communication
- **Mission orchestration** between components
- **Data synchronization** across systems

#### `src/dashboard/` - Web Dashboard
- **React/TypeScript** monitoring interface
- **Real-time visualization** of rover state
- **Mission control** and teleoperation
- **Vite** build system with fast development

#### `src/simulation/` - Gazebo Integration
- **Robot models** and world environments
- **Sensor simulation** and physics
- **Testing scenarios** for validation

### 🎯 Mission-Specific Code (`missions/`)

| Mission File | URC Challenge | Description |
|--------------|---------------|-------------|
| `sample_collection_mission.py` | Science | Collect and analyze samples |
| `delivery_mission.py` | Delivery | Transport objects between locations |
| `waypoint_navigation_mission.py` | Navigation | GPS waypoint following |
| `autonomous_keyboard_mission.py` | Equipment Service | Type on computer autonomously |
| `follow_me_mission.py` | Follow-Me | Track and follow ArUco markers |

### 🧪 Testing & Validation (`tests/`)

```
tests/
├── unit/               # Individual component tests
├── integration/        # Cross-component validation
├── simulation/         # Gazebo-based testing
└── hardware/           # Hardware-in-the-loop tests
```

### 🛠️ Development Tools

| Directory | Purpose |
|-----------|---------|
| **`tools/`** | Development utilities, calibration, deployment |
| **`scripts/`** | Build scripts, validation, health checks |
| **`docker/`** | Container definitions for development/deployment |
| **`docs/`** | Sphinx documentation (run `make html` to build) |

### 🏃‍♂️ Quick Developer Orientation

**New to the project? Start here:**

1. **📖 Read** [`docs/getting_started.rst`](docs/getting_started.rst) - Setup guide
2. **🔧 Install** dependencies: `pip install -e .`
3. **🏗️ Build** ROS2 packages: `colcon build`
4. **🚀 Launch** development environment: `./start.py dev dashboard`
5. **🧪 Test** your changes: `python -m pytest tests/unit/`

**Need to find something?**
- **ROS2 packages**: Look in `src/autonomy/`
- **Mission logic**: Check `missions/`
- **Configuration**: See `config/rover.yaml`
- **Documentation**: `docs/` directory

### `/vendor/`

Git submodules for external components:

- **`teleoperation/`**: Web-based teleoperation interface
- **`control-systems/`**: STM32-based control systems for drive and arm motion

**Note**: Submodules are readonly. See [`vendor/README.md`](vendor/README.md) for details.

## 🤝 Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for development guidelines. For AI assistants, see [`AGENTS.md`](AGENTS.md) for detailed development commands and code style guidelines.

### Development Workflow
1. Run `./scripts/check_quality.sh` before committing
2. Ensure >80% test coverage for new code
3. Follow ROS2 best practices and safety protocols
4. Update documentation for API changes

## 📄 License

This project is licensed under the MIT License - see the [`LICENSE`](LICENSE) file for details.

## 📚 Documentation

### Essential Reading
- **[Getting Started](docs/getting_started.rst)** - Setup and codebase orientation
- **[Quick Start](docs/quickstart.rst)** - Run the system in minutes
- **[Development Guide](AGENTS.md)** - Commands, code style, and workflows (for AI agents)
- **[Unified Systems](docs/unified_systems.rst)** - Enterprise architecture components

### Specialized Guides
- **[Onboarding by role](docs/onboarding/README.md)** - Perception, Cognition, Motion Control, Communication
- **[Build and test](docs/development/BUILD_AND_TEST.md)** - ROS2 build and testing
- **[Hardware Testing](docs/hardware/HOW_TO_TEST_ON_HARDWARE.md)** - CAN and hardware validation
- **[Operations](docs/operations/)** - Deployment, troubleshooting, [deployment guide](docs/operations/DEPLOYMENT_GUIDE.md)
- **[API Reference](docs/api/index.rst)** - Python, C++, JavaScript APIs

### Competition Context
- **URC 2026 Challenges**: Sample collection, equipment service, navigation, autonomous traversal
- **Safety Requirements**: Multi-layer redundant safety systems
- **Performance Targets**: Real-time processing under resource constraints
