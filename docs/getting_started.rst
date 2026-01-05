.. _getting_started:

==================
Getting Started
==================

Welcome to the URC 2026 Mars Rover project! This guide will help you understand the codebase structure and get up to speed quickly.

Project Overview
================

The URC 2026 rover is a complete autonomous robotics system with:

- **ROS2-based autonomy stack** for navigation and control
- **Computer vision** for object detection and scene understanding
- **Mission execution** for URC competition challenges
- **Web dashboard** for monitoring and control
- **Simulation environment** for testing and validation

Quick Start Checklist
=====================

.. checklist::

   [ ] Clone repository: ``git clone --recurse-submodules <repo-url>``
   [ ] Install dependencies: ``pip install -e .``
   [ ] Setup ROS2 workspace: ``source /opt/ros/humble/setup.bash``
   [ ] Build packages: ``colcon build``
   [ ] Launch development environment: ``./start.py dev dashboard``
   [ ] Run basic tests: ``python -m pytest tests/unit/ -v``

Understanding the Codebase
===========================

The project follows a **layered architecture** with clear separation of concerns:

Code Organization Map
----------------------

.. code-block:: text

   urc-machiato-2026/
   ├── src/                    # 🚀 ROS2 Source Packages
   │   ├── autonomy/          # 🤖 Core Robotics Stack
   │   │   ├── bt/            # Behavior Tree logic
   │   │   ├── control/       # Hardware control (LEDs, motors)
   │   │   ├── core/          # Navigation, state, safety
   │   │   ├── interfaces/    # ROS2 messages/services
   │   │   ├── perception/    # Vision, SLAM, sensors
   │   │   └── utilities/     # Shared code
   │   ├── bridges/           # 🌉 Communication layer
   │   ├── frontend/          # 💻 Web dashboard
   │   └── simulation/        # 🎮 Gazebo integration
   ├── missions/              # 🎯 URC Mission implementations
   ├── simulation/            # 🏗️ Additional simulation tools
   ├── config/                # ⚙️ Configuration files
   ├── tests/                 # 🧪 Test suites
   └── tools/                 # 🔧 Development utilities

Where to Find What You Need
---------------------------

+----------------+------------------+-----------------------------------+
| I want to...   | Look in...       | Example files                     |
+================+==================+===================================+
| Change robot   | ``src/autonomy/``| ``navigation_node.py``            |
| behavior       |                  | ``state_machine.py``              |
+----------------+------------------+-----------------------------------+
| Add new        | ``missions/``     | ``sample_collection_mission.py``  |
| mission        |                  |                                   |
+----------------+------------------+-----------------------------------+
| Modify web UI  | ``src/frontend/``| ``Dashboard.jsx``                 |
|                |                  | ``MissionControl.tsx``            |
+----------------+------------------+-----------------------------------+
| Test in        | ``simulation/``   | ``worlds/mars_yard.world``        |
| simulation     |                  | ``rover_model.urdf``              |
+----------------+------------------+-----------------------------------+
| Add ROS2       | ``src/autonomy/``| ``interfaces/msg/SensorData.msg`` |
| messages       | ``interfaces/``   |                                   |
+----------------+------------------+-----------------------------------+
| Write tests    | ``tests/``        | ``test_navigation.py``            |
+----------------+------------------+-----------------------------------+

Key Concepts to Understand
===========================

ROS2 Architecture
------------------

The system uses **ROS2 Humble** with a distributed architecture:

- **Nodes**: Individual processes (navigation, perception, control)
- **Topics**: Data streams between nodes
- **Services**: Request/response communication
- **Actions**: Long-running tasks with feedback

Behavior Trees
--------------

Mission logic is implemented using **Behavior Trees** (BT.CPP):

- **Sequences**: Execute tasks in order
- **Selectors**: Try alternatives until one succeeds
- **Decorators**: Modify behavior (retry, timeout, etc.)
- **Actions**: Leaf nodes that perform work

Mission Structure
-----------------

Each URC mission follows a standard pattern:

1. **Planning**: Generate waypoints or trajectories
2. **Execution**: Navigate and perform tasks
3. **Monitoring**: Track progress and handle failures
4. **Recovery**: Handle errors and edge cases

Development Workflow
=====================

Daily Development
------------------

1. **Pull latest changes**: ``git pull --recurse-submodules``
2. **Create feature branch**: ``git checkout -b feature/my-feature``
3. **Make changes** following the architecture above
4. **Test locally**: ``./start.py dev dashboard``
5. **Run tests**: ``python -m pytest tests/unit/ -v``
6. **Commit changes**: ``git commit -m "Add feature"``
7. **Create PR** for review

Code Standards
--------------

- **Python**: Black formatting, type hints, docstrings
- **C++**: Follow ROS2 style guidelines
- **JavaScript**: ESLint, Prettier formatting
- **Tests**: pytest with coverage >80%
- **Documentation**: Sphinx docs for public APIs

Common Development Tasks
=========================

Adding a New Mission
---------------------

1. Create ``missions/new_mission.py``
2. Implement mission class inheriting from base
3. Add BT XML in ``src/autonomy/bt/behavior_trees/``
4. Add ROS2 action server in autonomy stack
5. Update launch files and dashboard

Modifying Robot Behavior
-------------------------

1. Find relevant autonomy package in ``src/autonomy/``
2. Modify behavior logic (Python/C++)
3. Update ROS2 interfaces if needed
4. Test in simulation first
5. Validate on hardware

Adding Web Dashboard Features
------------------------------

1. Modify React components in ``src/frontend/``
2. Update WebSocket communication in ``src/bridges/``
3. Add backend endpoints if needed
4. Test real-time updates

Troubleshooting
===============

Common Issues
--------------

**ROS2 build fails:**
- Check dependencies in ``package.xml``
- Ensure all required packages are installed
- Look for missing includes or linking errors

**Tests failing:**
- Run ``python -m pytest tests/ -v`` for details
- Check ROS2 nodes are running
- Verify configuration files

**Simulation not working:**
- Check Gazebo installation
- Verify URDF files are valid
- Look at simulation logs

Getting Help
=============

- **📖 Documentation**: ``docs/`` directory (run ``make html``)
- **🧪 Testing Guide**: ``docs/testing/``
- **🚀 Deployment**: ``DEPLOYMENT.md``
- **💬 Team Chat**: Ask questions in development channels
- **📋 Issues**: Check existing GitHub issues first

Remember: This is a complex robotics system. Don't hesitate to ask questions - the codebase has evolved over time and some complexity is inherent to the domain!