.. _l2-lidar-blackboard-e2e:

L2 LiDAR, blackboard, and dashboard (end-to-end)
=================================================

This guide matches the on-robot verification path for **Unitree L2** data flowing into
``proximity_monitor``, the **unified blackboard** (via ``bt_orchestrator``), and the
**dashboard** (rosbridge + ``/blackboard/get_value``).

Data path (summary)
--------------------

1. L2 driver publishes ``sensor_msgs/PointCloud2`` (typical topic: ``/unilidar/cloud``).
2. ``pointcloud_to_laserscan`` republishes ``sensor_msgs/LaserScan`` on ``/scan``.
3. ``proximity_monitor`` (``autonomy_core``) subscribes to ``/scan`` and writes obstacle keys
   through ``UnifiedBlackboardClient`` (Python ``shared/core/``).
4. The C++ **BT orchestrator** (``autonomy_bt``, node ``bt_orchestrator``) hosts the
   blackboard and exposes ``/blackboard/get_value`` and ``/blackboard/set_value``.
5. The dashboard uses rosbridge and polls those keys (e.g. Perception / Blackboard views).

Obstacle-related blackboard keys (written by proximity logic) include:

- ``closest_obstacle_distance``
- ``obstacle_detected``
- ``proximity_violation_distance``

See ``shared/core/blackboard_keys.py`` and ``docs/architecture/BLACKBOARD_SYSTEM.md``.

Python import path (``shared/core``)
------------------------------------

Nodes such as ``proximity_monitor`` import ``core.unified_blackboard_client`` and
``core.blackboard_keys`` from the repo ``shared/`` tree. At runtime, either:

- Run from a layout where an ancestor of the installed package contains ``shared/core/``, or
- Set ``URC_REPO_ROOT`` to the repository root so ``shared`` is added to ``sys.path``.

If the client fails to import, check logs for missing blackboard writes.

Phase A: Network and L2 driver
------------------------------

1. Put the host on the LiDAR subnet (example: ``192.168.1.x/24``), ping the LiDAR IP.
2. Build and run ``unitree_lidar_ros2`` (and SDK) per Unitree documentation; ``source`` your
   workspace.
3. Verify ROS 2 traffic:

   - ``ros2 topic list`` includes the cloud topic.
   - ``ros2 topic hz <cloud_topic>`` shows a stable rate.
   - ``ros2 topic echo <cloud_topic> --once`` shows non-zero dimensions where applicable.
   - RViz2: display PointCloud2 with the correct ``fixed frame``.

Troubleshooting: ``unitree_lidar_ros2`` not found
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

- **``No such file or directory`` on ``source .../install/setup.bash``:** that path was a
  **placeholder**. You must **clone, build, then source the real directory** where ``colcon``
  created ``install/`` (see below). There is no driver install inside the Machiato repo unless
  you add it yourself.

- If the error shows **only** ``searching: ['/opt/ros/jazzy']`` (or your distro path), you did
  not ``source`` an **install** space that contains the driver. Typical order:

  .. code-block:: bash

     source /opt/ros/$ROS_DISTRO/setup.bash
     source "$HOME/unilidar_sdk2/unitree_lidar_ros2/install/setup.bash"   # example: adjust to your clone

- **One-time build (upstream layout):** Unitree ships the ROS 2 package inside
  `unilidar_sdk2 <https://github.com/unitreerobotics/unilidar_sdk2>`__ (see their README,
  section *How to Use the ROS2 Package*). After cloning:

  .. code-block:: bash

     git clone https://github.com/unitreerobotics/unilidar_sdk2.git "$HOME/unilidar_sdk2"
     cd "$HOME/unilidar_sdk2/unitree_lidar_ros2"
     colcon build
     source install/setup.bash
     ros2 launch unitree_lidar_ros2 launch.py

  Their docs reference an older folder name (``unilidar_sdk/...``); use ``unilidar_sdk2`` if
  that is what you cloned. Newer ROS distros (e.g. Jazzy) may need extra dependency or CMake
  fixes compared to the Foxy-era instructions; follow upstream issues/README if the build fails.

- The package is **not** built by the default Machiato colcon instructions for
  ``autonomy_interfaces`` / ``autonomy_core`` / ``autonomy_bt``.

- If ``ros2`` commands behave oddly, ensure you are not shadowing the ROS Python with another
  environment (e.g. ``conda deactivate`` and a clean ``PATH`` for that shell).

Phase B: PointCloud2 to ``/scan``
----------------------------------

Install the bridge package (Ubuntu package name includes the distro), e.g.:

.. code-block:: bash

   sudo apt install ros-${ROS_DISTRO}-pointcloud-to-laserscan

If ``ros2 launch autonomy_core unitree_l2_pc2_to_scan.launch.py`` fails with
``package 'pointcloud_to_laserscan' not found``, the Debian package above is missing; install
it and re-source your ROS workspace.

Launch the provided bridge (remap if your cloud topic differs):

.. code-block:: bash

   ros2 launch autonomy_core unitree_l2_pc2_to_scan.launch.py

Optional arguments: ``cloud_topic:=/unilidar/cloud``, ``scan_topic:=/scan``,
``use_sim_time:=true``.

Tune ``config/unitree_l2_pc2_to_scan.yaml`` (height slice, angles, range) so the forward arc
contains valid ranges for your mount.

Verify:

.. code-block:: bash

   ros2 topic info /scan
   ros2 topic hz /scan

Phase C: Autonomy stack and blackboard
--------------------------------------

1. Build and source the workspace (``autonomy_interfaces``, ``autonomy_core``, ``autonomy_bt``).
2. Start **bt_orchestrator** and bring the lifecycle node to an active state, for example:

   .. code-block:: bash

      ros2 run autonomy_bt bt_orchestrator
      # In another terminal (same ROS_DOMAIN_ID, sourced):
      ros2 lifecycle set /bt_orchestrator configure
      ros2 lifecycle set /bt_orchestrator activate

   Alternatively use ``scripts/launch/integrated_system.launch.py`` if that matches your profile.

3. Start ``proximity_monitor`` (included in ``ros2 launch autonomy_core unified.launch.py`` or
   run the executable standalone). Confirm it subscribes to ``/scan`` and that the unified
   blackboard client initialized in logs.

4. Without the UI, call the service:

   .. code-block:: bash

      ros2 service call /blackboard/get_value autonomy_interfaces/srv/GetBlackboardValue "{key: 'closest_obstacle_distance', value_type: ''}"

   Repeat for ``obstacle_detected``. Moving an obstacle in front of the L2 should change values
   when the full pipeline is healthy.

**Smoke script** (after sourcing the workspace):

.. code-block:: bash

   python3 scripts/hardware/lidar_blackboard_smoke_test.py

Phase D: Dashboard (rosbridge)
------------------------------

1. Run ``rosbridge_server`` / ``rosapi`` on the same ``ROS_DOMAIN_ID`` as the robot stack.
2. Start the dashboard (project entrypoint or ``npm run dev`` under ``services/dashboard``).
3. Connect the UI to the bridge (default often ``ws://localhost:9090``).
4. Open **Perception** or **Blackboard** debugging views; summarized obstacle fields come from
   blackboard polling, not raw point clouds. For raw cloud, use RViz2 or add a dedicated
   subscription in the UI.

Phase E: Deployment / HIL notes
-------------------------------

- **Order** (typical): L2 driver → ``unitree_l2_pc2_to_scan`` → ``bt_orchestrator``
  (configured+activated) → ``proximity_monitor`` (or full ``unified.launch.py``) → rosbridge →
  dashboard.
- **Docker**: LiDAR UDP usually needs host networking or dedicated NIC access to the LiDAR
  subnet.
- **Tests**: Hardware-style checks can use ``@pytest.mark.hardware`` patterns; this E2E path is
  primarily manual HIL unless wrapped in launch + service-call automation.

References in-tree
------------------

- Launch: ``services/autonomy/autonomy_core/launch/unitree_l2_pc2_to_scan.launch.py``
- Params: ``services/autonomy/autonomy_core/config/unitree_l2_pc2_to_scan.yaml``
- Proximity: ``services/autonomy/autonomy_core/autonomy_core/safety/proximity_monitor.py``
- Unified launch notes: ``services/autonomy/autonomy_core/launch/unified.launch.py`` (docstring)
- BT + blackboard testing: ``docs/development/BUILD_AND_TEST.md``
