# URC 2026 Mars Rover Platform

Welcome to the Machiato 2026 Robotics Platform repository. This repository implements a robust, microservice-oriented autonomy software stack for the University Rover Challenge.

## Repository Structure

The system is organized into decoupled services for maintainability and scalability:

- [**Autonomy Service**](./services/autonomy/) — Core ROS2 stack (navigation, perception, control).
- [**Dashboard Service**](./services/dashboard/) — React-based interface for teleoperation and real-time monitoring.
- [**Simulation Service**](./services/simulation/) — Gazebo and Python-based simulation environments for testing.
- [**Missions Service**](./services/missions/) — High-level Behavior Tree scripts coordinating mission execution.
- [**Hardware Service**](./services/hardware/) — Utilities for hardware integration and calibration.
- [**Shared Components**](./shared/) — Common libraries, interfaces, and core infrastructure used across services.

## Getting Started

1. **Clone the repository:**
   ```bash
   git clone --recursive https://github.com/SJSURoboticsTeam/urc-machiato-2026.git
   cd urc-machiato-2026
   ```

2. **Run in Docker (Recommended):**
   ```bash
   docker-compose -f docker/docker-compose.dev.yml up
   ```

3. **Explore the Docs:**
   See the `docs/` folder for comprehensive onboarding and architecture documentation.

## Native ROS 2 development (lab machines)

Use **Ubuntu’s system `python3`** for `colcon build` and any `rclpy` scripts so generated
interfaces match the distro (Jazzy: Python 3.12, Humble: 3.10). Deactivate conda/venv before
building; pin CMake with `PYTHON_EXECUTABLE=$(command -v python3)`.

- **Onboarding:** [docs/onboarding/ros2_workspace_and_bridges.md](docs/onboarding/ros2_workspace_and_bridges.md)
- **Python / ROS alignment:** [docs/development/ros2_python_environment.rst](docs/development/ros2_python_environment.rst) (Sphinx)
- **Build + tests:** [docs/development/BUILD_AND_TEST.md](docs/development/BUILD_AND_TEST.md)
