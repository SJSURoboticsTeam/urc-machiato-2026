.. _big_picture:

============
Big Picture
============

Welcome to the URC 2026 Mars Rover! This page gives you the **30-second overview** of how everything fits together. Think of this as the "elevator pitch" for our entire robotics system.

.. image:: architecture/diagrams/high/04_system_component_architecture.png
   :alt: URC 2026 System Architecture
   :align: center
   :width: 80%

What's Happening Here?
======================

Imagine you're commanding a Mars rover from Earth. You need to:

1. **Tell the rover what to do** (missions)
2. **Make sure it knows where it is** (navigation/SLAM)
3. **Help it see and understand its surroundings** (vision/perception)
4. **Keep it safe and working** (safety systems)
5. **Control its movements precisely** (motion control)

Our system does all of this automatically, with you supervising from a web dashboard.

The 6 Layers of Our Rover
========================

.. image:: architecture/diagrams/high/05_ros2_communication_architecture.png
   :alt: ROS2 Communication Architecture
   :align: center
   :width: 80%

**🎯 Mission Layer** (What the rover does)
   - Sample collection, delivery, autonomous keyboard
   - High-level task planning and execution
   - **Files**: ``missions/``, ``src/autonomy/bt/``

**🤖 Autonomy Layer** (How the rover thinks)
   - Navigation, obstacle avoidance, path planning
   - State machines, decision making, safety
   - **Files**: ``src/autonomy/core/``

**👁️ Perception Layer** (How the rover sees)
   - Computer vision, SLAM, sensor processing
   - Understanding the environment around the rover
   - **Files**: ``src/autonomy/perception/``

**🎮 Control Layer** (How the rover moves)
   - Motor control, actuator management
   - Hardware interfaces and feedback
   - **Files**: ``src/autonomy/control/``

**🌉 Communication Layer** (How parts talk to each other)
   - ROS2 messaging, WebSocket bridges
   - Network resilience, data routing
   - **Files**: ``src/bridges/``, ``src/core/``

**💻 User Interface Layer** (How you control the rover)
   - Web dashboard, real-time monitoring
   - Mission planning, system health
   - **Files**: ``src/frontend/``, ``src/dashboard/``

Key Data Flows
==============

**Mission Commands** flow down:
   Missions → Autonomy → Control → Hardware

**Sensor Data** flows up:
   Hardware → Perception → Autonomy → Missions → User

**Safety** is everywhere:
   Every layer can trigger emergency stops

Quick Start for New Developers
==============================

1. **Understand your role**: Pick Network, SLAM/NAV, ARM, or Testing
2. **Read your guide**: Each role has a dedicated onboarding document
3. **Find your code**: Use the directory map below
4. **Run a test**: Get something working quickly

Directory Map for New People
============================

.. code-block:: text

   urc-machiato-2026/
   ├── 🎯 missions/              # What the rover does
   │   ├── sample_collection_mission.py
   │   ├── delivery_mission.py
   │   └── waypoint_navigation_mission.py
   ├── 🤖 src/autonomy/          # How the rover works
   │   ├── core/                 # Navigation, state, safety
   │   ├── perception/           # Vision, SLAM, sensors
   │   ├── control/              # Motors, hardware
   │   └── interfaces/           # ROS2 messages
   ├── 🌐 src/bridges/           # Communication between parts
   ├── 💻 src/frontend/          # Web dashboard
   ├── 🧪 tests/                 # Making sure it works
   └── ⚙️ config/                # Configuration files

Your First Hour
===============

**Don't try to understand everything!** Focus on:

1. **Run the system**: ``./start.py dev dashboard``
2. **See the dashboard**: Open http://localhost:3000
3. **Run a simple test**: ``python -m pytest tests/unit/ -v``
4. **Find your role's guide**: See the navigation below

.. image:: onboarding_flow.png
   :alt: Developer Onboarding Flow
   :align: center
   :width: 80%

**Onboarding Process:**
- **Week 1**: Understand big picture, get development environment running
- **Week 1-2**: Master your role's systems and tools
- **Week 2+**: Contribute to your team's area of expertise

Next Steps
==========

Now that you understand the big picture, choose your role and dive deep:

**Team-Specific Onboarding:**

.. toctree::
   :maxdepth: 1

   Network Team Guide <network_guide>
   SLAM/NAV Team Guide <slam_nav_guide>
   ARM Team Guide <arm_guide>
   Testing Team Guide <testing_guide>

**General Development:**

.. toctree::
   :maxdepth: 1

   getting_started
   architecture/diagrams
   quickstart

Questions? Ask in team chat or check the detailed documentation!
