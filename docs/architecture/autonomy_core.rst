==================
Autonomy Core
==================

The consolidated ROS2 package ``autonomy_core`` combines navigation, safety, control, and perception in one package.

Package Structure
=================

::

    src/autonomy/autonomy_core/
    ├── autonomy_core/
    │   ├── navigation/   # Path planning, motion control, GNSS
    │   ├── safety/       # Emergency stop, watchdog, proximity
    │   ├── control/      # Hardware interfaces
    │   └── perception/   # Computer vision, SLAM, sensor sim
    ├── launch/
    │   ├── unified.launch.py   # mode:=simulation|hardware|competition
    │   └── simulation.launch.py
    ├── package.xml
    ├── setup.py
    └── CMakeLists.txt

Launch
======

Unified launch (single entry point)::

    ros2 launch autonomy_core unified.launch.py mode:=simulation
    ros2 launch autonomy_core unified.launch.py mode:=competition

Simulation-only::

    ros2 launch autonomy_core simulation.launch.py

Nodes
=====

- **Navigation**: ``navigation_node``, ``path_planner``, ``motion_controller``, ``gnss_processor``
- **Safety**: ``safety_monitor``, ``safety_watchdog``, ``proximity_monitor``, ``emergency_coordinator``
- **Perception**: ``computer_vision_node``, ``slam_node``, ``sensor_simulator``

Legacy packages (``autonomy_navigation``, ``autonomy_safety_system``, etc.) remain available; new development should use ``autonomy_core`` and the unified launch.

See :doc:`overview` and onboarding pillar docs under :doc:`../onboarding/README`.
