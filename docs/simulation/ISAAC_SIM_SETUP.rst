.. _isaac_sim_setup:

Isaac Sim 5.0 Setup
===================

Full step-by-step guide for Isaac Sim 5.0 with ROS2 and the URC 2026 rover:

- **Scripts**: ``scripts/setup_fastdds.sh``, ``scripts/isaac_sim_launch.sh``
- **Bridge**: ``ros2 launch isaac_sim_bridge isaac_sim_bridge.launch.py``
- **Detailed instructions**: See the Markdown guide in the repo: ``docs/simulation/ISAAC_SIM_SETUP.md``

Quick start:

1. Run ``./scripts/setup_fastdds.sh`` once.
2. Launch Isaac Sim with ``./scripts/isaac_sim_launch.sh``.
3. Import the rover URDF (File → Import → URDF) from ``robot_definition/rover2025/urdf/rover2025.urdf``.
4. Add cameras and LiDAR via Tools → Robotics → ROS 2 OmniGraphs, then press PLAY.
5. In another terminal: ``source install/setup.bash`` and ``ros2 launch isaac_sim_bridge isaac_sim_bridge.launch.py``.

**SLAM + Navigation (after setup):** With Isaac Sim running and PLAY pressed, run the full stack in one go: ``ros2 launch isaac_sim_bridge isaac_sim_slam_nav.launch.py``. This starts the bridge (if needed), camera_info relay, SLAM pipeline (odom → slam/pose), and navigation_node. See ``docs/simulation/ISAAC_SIM_SETUP.md`` Step 6.
