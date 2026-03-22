.. _skeletal-hil-checklist:

Skeletal HIL checklist (sensors to computer)
============================================

Use this page for **minimal hardware-in-the-loop (HIL) bring-up**: prove each sensor reaches ROS 2 on the host before wiring full autonomy, blackboard, or missions.

Related overview: :doc:`hardware_integration`.

Layers (order matters)
----------------------

1. **Physical** – Power, cables, Ethernet subnet (e.g. LiDAR), USB3 for cameras, CAN/USB for chassis.
2. **OS / permissions** – Devices visible; user in ``dialout``; udev rules where needed.
3. **ROS topics** – Driver nodes publish; stable ``ros2 topic hz``; sane ``frame_id`` in headers.
4. **TF** – Sensor frames chain to ``base_link`` (or your robot root).
5. **Autonomy / blackboard / UI** – Only after topics and TF are trustworthy.

Environment variables (lab profile)
------------------------------------

Set these in your shell profile or systemd unit when running HIL tests and tooling:

.. list-table::
   :header-rows: 1
   :widths: 28 52

   * - Variable
     - Purpose
   * - ``ROS_DOMAIN_ID``
     - Same value in every terminal, rosbridge host, and robot processes.
   * - ``URC_CAN_INTERFACE``
     - Real CAN interface name for pytest HIL fixtures (see below).
   * - ``URC_STM32_SERIAL``
     - Serial device path (e.g. ``/dev/ttyACM0``) for pytest HIL and default ``can_port`` on ``hil_serial_bridges.launch.py``.
   * - ``URC_HIL_TIMEOUT_S``
     - Optional timeout (seconds) for hardware operations in tests (default ``30``).
   * - ``URC_REPO_ROOT``
     - Repository root; helps Python nodes find ``shared/`` after install (see :doc:`hardware/l2_lidar_blackboard_e2e`).

Pytest hardware fixtures read ``URC_CAN_INTERFACE`` and ``URC_STM32_SERIAL`` from :file:`tests/integration/hardware/conftest.py`.

Rosbridge WebSocket and serial CAN (installed launches)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

The dashboard defaults to ``ws://localhost:9090`` (``services/dashboard``). Use the colcon-installed launches so the port matches the UI:

.. code-block:: bash

   source install/setup.bash
   ros2 launch autonomy_core rosbridge_stack.launch.py

WebSocket + ``hardware_interface`` (serial ``can_port``, default from ``URC_STM32_SERIAL`` or ``/dev/ttyACM0``):

.. code-block:: bash

   export URC_STM32_SERIAL=/dev/ttyACM0
   ros2 launch autonomy_core hil_serial_bridges.launch.py

If you change ``rosbridge_port``, set the dashboard URL accordingly (or ``VITE_`` / runtime config if your build supports it). For production, consider binding ``address:=127.0.0.1`` on ``rosbridge_stack.launch.py``.

``sensor_topic_smoke.sh`` (optional)
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

From a sourced ROS 2 workspace, checks that topics exist and (optionally) meet a minimum average rate:

.. code-block:: bash

   source /opt/ros/humble/setup.bash
   source install/setup.bash
   ./scripts/hardware/sensor_topic_smoke.sh --min-hz 0.5 /scan /imu/data

Or pass a topic list via environment (space-separated):

.. code-block:: bash

   URC_SMOKE_TOPICS="/scan /odom" URC_SMOKE_MIN_HZ=0.1 ./scripts/hardware/sensor_topic_smoke.sh

.. list-table::
   :header-rows: 1
   :widths: 28 52

   * - Variable / flag
     - Purpose
   * - ``URC_SMOKE_TOPICS``
     - Space-separated topics if no CLI topics are given.
   * - ``URC_SMOKE_MIN_HZ``
     - Minimum average rate from ``ros2 topic hz`` (default ``0.1``). Set to ``0`` to skip the hz check (presence only).
   * - ``URC_SMOKE_HZ_SEC``
     - Seconds to run ``ros2 topic hz`` per topic (default ``4``).
   * - ``--echo-once``
     - Require at least one message via ``ros2 topic echo --once`` before the hz check.

Ordered backlog checklist
-------------------------

1. **Inventory** – For each sensor: bus (USB / Ethernet / UART / CAN), expected topic names, message type, ``frame_id``.
2. **Physical** – Verify link (ping for IP LiDAR; ``lsusb``; ``/dev/ttyACM*``; ``ip addr``).
3. **Permissions** – CAN/serial: ``docs/hardware/HARDWARE_TESTING_GUIDE.md`` and ``scripts/hardware/setup_usbcan_pi5.sh`` where applicable.
4. **Per-sensor ROS proof** – ``ros2 topic list``, ``ros2 topic hz <topic>``, ``ros2 topic echo <topic> --once``. Optional automation: ``scripts/hardware/sensor_topic_smoke.sh`` (see env vars below).
5. **TF** – ``ros2 run tf2_tools view_frames`` (or echo specific transforms); fix URDF/xacro and ``robot_state_publisher`` if transforms are missing.
6. **Autonomy hooks** – ``ros2 launch autonomy_core unified.launch.py`` does not start every driver; add driver launches or parameters as needed.
7. **Blackboard / dashboard** – Follow the same pattern as LiDAR: writers into the unified blackboard and/or ``/blackboard/get_value`` via ``bt_orchestrator`` (:doc:`hardware/l2_lidar_blackboard_e2e`).
8. **Record** – ``ros2 bag record`` on your skeletal topic set for regression.

Modality-specific references
----------------------------

.. list-table::
   :header-rows: 1
   :widths: 22 58

   * - Modality
     - Repo docs / scripts
   * - LiDAR (Unitree L2) + blackboard
     - :doc:`hardware/l2_lidar_blackboard_e2e`; ``scripts/hardware/lidar_blackboard_smoke_test.py``; ``scripts/run_hil_lidar_websocket_test.sh``. If smoke tests fail importing ``autonomy_interfaces``, see :doc:`development/ros2_python_environment` (system Python vs ``install/`` bindings).
   * - CAN / chassis / hardware interface
     - ``ros2 launch autonomy_core hil_serial_bridges.launch.py``; ``docs/hardware/HARDWARE_TESTING_GUIDE.md``; ``scripts/hardware/can_message_monitor.py``
   * - Cameras / calibration
     - :doc:`hardware/calibration`; :doc:`calibration/camera_calibration`
   * - Simulation vs hardware mindset
     - :doc:`simulation_testing_guide` (hardware phases)
   * - Deployment / launch entrypoints
     - :doc:`operations/deployment`

Docker note
-----------

Container runs usually need **host network** or **device passthrough** for USB/Ethernet sensors. For skeletal HIL, running drivers on the **host** with ROS sourced is often simpler than debugging Docker first.

Safety
------

Verify **E-stop** and safe commanding policy before ``cmd_vel`` or mission tests. Competition-specific launches (e.g. ``competition_system.launch.py``) assume extra packages and policies; do not use them for first sensor smoke tests unless intentional.
