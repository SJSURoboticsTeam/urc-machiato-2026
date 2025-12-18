# URC 2026 Robotics Platform

A complete autonomous robotics system for the University Rover Challenge 2026, featuring ROS2-based autonomy, computer vision, SLAM, and mission execution capabilities.

## 🚀 Quick Start

### One-Command Launch
```bash
# Development frontend only
./start.py dev frontend

# Testing dashboard (backend + frontend)
./start.py dev dashboard

# Full autonomy system
./start.py prod autonomy

# Simulation environment
./start.py dev simulation
```

### For Production Deployment
See [`DEPLOYMENT.md`](DEPLOYMENT.md) for production deployment instructions.

## 📋 Requirements

- **Python**: 3.10+
- **ROS2**: Humble Hawksbill
- **Docker**: 24.0+ (recommended)
- **System**: Ubuntu 22.04 LTS or equivalent

## 🏗️ Architecture

- **Autonomy System**: ROS2-based navigation and control
- **Computer Vision**: Object detection and scene understanding
- **SLAM**: Real-time localization and mapping
- **Mission Control**: Task execution and coordination
- **Web Interface**: Real-time monitoring and control
- **Teleoperation**: Web-based remote control interface (submodule)
- **Control Systems**: STM32-based drive and arm control (submodule)

## 📦 Installation

### Quick Install (Recommended)
```bash
# Clone repository with submodules
git clone --recurse-submodules https://github.com/your-org/urc-machiato-2026.git
cd urc-machiato-2026

# If not cloned with --recurse-submodules, initialize submodules
git submodule update --init --recursive

# Install Python dependencies
pip install -e .

# For development
pip install -e ".[dev]"

# For documentation
pip install -e ".[docs]"

# Setup submodules (optional)
./scripts/manage_submodules.sh update
```

### Docker Deployment
```bash
# Build and start services
docker-compose -f docker-compose.prod.yml up -d

# Check status
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

### `/autonomy/`
Main robotics autonomy codebase and development environment.
- **`docs/`** - Technical documentation and guides
- **`code/`** - Core autonomy subsystems (navigation, SLAM, computer vision, etc.)
- **`config/`** - Autonomy-specific configuration files
- **`launch/`** - ROS2 launch files for autonomy components
- **`scripts/`** - Development and utility scripts
- **`tests/`** - Unit and integration tests

### `/bridges/`
Data bridge components for inter-system communication:
- `map_data_bridge.py` - Map data bridging
- `slam_data_bridge.py` - SLAM data processing
- `websocket_mission_bridge.py` - Mission control WebSocket bridge
- `websocket_slam_bridge.py` - SLAM WebSocket bridge

### `/config/`
Configuration files:
- `rover.yaml` - Single unified configuration file with environment overrides

### `/docs/`
Sphinx documentation system (regenerable from source).

### `/frontend/`
Web interface for monitoring and control.

### `/launch/`
ROS2 launch files for system components:
- `mission_system.launch.py` - Complete mission system
- `rover_simulation.launch.py` - Simulation environment
- `slam_bridge.launch.py` - SLAM and mapping

### `/missions/`
Mission implementations for URC competition:
- `debug_mission.py` - Debug and testing mission
- `delivery_mission.py` - Sample delivery mission
- `follow_me_mission.py` - Follow-me mission
- `object_detection_mission.py` - Object detection mission
- `waypoint_navigation_mission.py` - GPS waypoint navigation

### `/robot_definition/`
Robot hardware definitions, URDF files, and BOM.

### `/scripts/`
Utility and management scripts:
- `validate_config.py` - Configuration validation
- `production_health_check.py` - System health checks
- `extract-todos-to-issues.py` - GitHub issue management

### `/tests/`
Comprehensive test suite with mocks and fixtures.

### `/vendor/`
Git submodules for external components:
- **`teleoperation/`**: Web-based teleoperation interface
- **`control-systems/`**: STM32-based control systems for drive and arm motion

**Note**: Submodules are readonly. See [`vendor/README.md`](vendor/README.md) for details.

## 🤝 Contributing

See [`CONTRIBUTING.md`](CONTRIBUTING.md) for development guidelines and contribution process.

## 📄 License

This project is licensed under the MIT License - see the [`LICENSE`](LICENSE) file for details.

## 📚 Documentation

- **[Deployment Guide](DEPLOYMENT.md)** - Production deployment instructions
- **[API Documentation](docs/)** - Complete technical documentation
- **[Development Guide](autonomy/QUICKSTART.md)** - Getting started with development
