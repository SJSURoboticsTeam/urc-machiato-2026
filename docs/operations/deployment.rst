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

Additional modes (see ``scripts/deploy.sh``): ``cognitive`` (competition-oriented stack),
``slam_only`` (SLAM launch only).

``deploy.sh`` modes vs ROS launch
==================================

After sourcing ROS and ``install/setup.bash``, ``deploy.sh`` starts the following:

.. list-table::
   :header-rows: 1
   :widths: 18 52 30

   * - Mode
     - Command
     - Notes
   * - ``simulation`` (default)
     - ``ros2 launch autonomy_core unified.launch.py mode:=simulation``
     - Includes simulated sensors when simulation condition matches unified launch.
   * - ``hardware``
     - ``ros2 launch autonomy_core unified.launch.py mode:=hardware``
     - Real-robot profile; ensure hardware drivers and network are configured.
   * - ``competition``
     - ``ros2 launch autonomy_core unified.launch.py mode:=competition``
     - Sets strict safety-related environment in unified launch path.
   * - ``cognitive``
     - ``ros2 launch autonomy_core competition_system.launch.py``
     - Optional packages (e.g. ``vision_processing``, ``missions``, ``hardware_interface``) must be built; includes SLAM via ``slam.launch.py``.
   * - ``slam_only``
     - ``ros2 launch autonomy_core slam.launch.py``
     - SLAM / RealSense-oriented pipeline only.

Extras not started by ``unified.launch.py``
===========================================

Start these when you need blackboard services, LiDAR bridging, or the web dashboard:

.. list-table::
   :header-rows: 1
   :widths: 28 52

   * - Capability
     - Typical command
   * - BT blackboard services (``/blackboard/get_value``, etc.)
     - ``ros2 run autonomy_bt bt_orchestrator`` then lifecycle ``configure`` and ``activate`` (see :doc:`../development/BUILD_AND_TEST`).
   * - Unitree L2 PointCloud2 to ``/scan``
     - ``ros2 launch autonomy_core unitree_l2_pc2_to_scan.launch.py``
   * - rosbridge + rosapi (dashboard WebSocket)
     - ``ros2 launch autonomy_core rosbridge_stack.launch.py`` (default port **9090**, matches ``services/dashboard``). Change ``rosbridge_port`` only if you point the UI at the same URL.
   * - HIL: WebSocket + serial CAN bridge
     - ``ros2 launch autonomy_core hil_serial_bridges.launch.py`` (optional ``can_port:=/dev/ttyACM0`` or env ``URC_STM32_SERIAL``).

See :doc:`../hardware/l2_lidar_blackboard_e2e` for LiDAR-to-blackboard verification.

Pre-Deployment
==============

1. Build the workspace: ``./scripts/build.sh prod --test``
2. Source ROS2: ``source /opt/ros/humble/setup.bash`` (or Jazzy if applicable)
3. Source workspace: ``source install/setup.bash``
4. For simulation: extend ``GAZEBO_MODEL_PATH`` with repo assets, e.g. ``services/simulation/models`` (``deploy.sh`` sets this for simulation mode)

Launch (manual)
===============

Unified launch::

    ros2 launch autonomy_core unified.launch.py mode:=simulation
    ros2 launch autonomy_core unified.launch.py mode:=competition

See :doc:`build_system` for build commands.

Other entrypoints
=================

- **Docker / compose:** ``docs/unified-docker-system.md`` and ``scripts/docker_deploy.sh``
- **Developer launcher:** ``scripts/start.py`` (frontend, dashboard script, autonomy via ``autonomy_core``, simulation via ``scripts/launch/``)
- **Legacy ROS launches:** ``scripts/launch/`` targets an older package layout; prefer ``autonomy_core`` launches above unless you maintain matching packages.
