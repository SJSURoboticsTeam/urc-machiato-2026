================
Deployment
================

Deployment procedures for URC 2026 rover system.

Deploy Script
=============

Use the unified deploy script::

    ./scripts/deploy.sh simulation   # Default: simulation mode
    ./scripts/deploy.sh hardware
    ./scripts/deploy.sh competition

The script runs pre-deployment checks, starts the system, and runs a health check.

Pre-Deployment
==============

1. Build the workspace: ``./scripts/build.sh prod --test``
2. Source ROS2: ``source /opt/ros/humble/setup.bash``
3. Source workspace: ``source install/setup.bash``
4. For simulation: set ``GAZEBO_MODEL_PATH`` to include ``src/simulation/models``

Launch
======

Unified launch::

    ros2 launch autonomy_core unified.launch.py mode:=simulation
    ros2 launch autonomy_core unified.launch.py mode:=competition

See :doc:`build_system` for build commands.
