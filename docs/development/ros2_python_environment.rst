.. _ros2-python-environment:

================================
ROS 2 and Python (cross-machine)
================================

ROS 2 message and service **Python bindings** under ``install/<pkg>/lib/pythonX.Y/`` must match
the **same** ``python3`` you use to run ``rclpy`` scripts, pytest with ROS, and tools such as
``lidar_blackboard_smoke_test.py``. If they differ, you may see:

- ``ImportError: libpython3.N.so: cannot open shared object file``
- ``UnsupportedTypeSupport: Could not import 'rosidl_typesupport_c' for package 'autonomy_interfaces'``

Distro defaults (use **system** Python tied to ROS, not a separate venv/conda Python):

+------------------+----------------------+------------------------+
| ROS 2 distro     | Typical Ubuntu       | System ``python3``     |
+==================+======================+========================+
| **Humble**       | 22.04                | 3.10                   |
+------------------+----------------------+------------------------+
| **Jazzy**        | 24.04                | 3.12                   |
+------------------+----------------------+------------------------+

Before building
===============

1. **Source the distro** before ``colcon``:

   .. code-block:: bash

      source /opt/ros/$ROS_DISTRO/setup.bash

2. **Do not** let conda/venv override ``python3`` during ``colcon build``:

   .. code-block:: bash

      conda deactivate
      deactivate 2>/dev/null || true
      export PATH="/usr/bin:/bin:/usr/local/bin:$PATH"
      unset VIRTUAL_ENV

3. **Pin the interpreter** for CMake (recommended on shared machines):

   .. code-block:: bash

      export PYTHON_EXECUTABLE="$(command -v python3)"
      echo "Using $PYTHON_EXECUTABLE — $(python3 --version)"

Canonical ``colcon`` invocation (this repository)
=================================================

From the repository root (step-by-step onboarding: :doc:`onboarding/ros2_workspace_and_bridges`):

.. code-block:: bash

   cd /path/to/urc-machiato-2026
   source /opt/ros/jazzy/setup.bash   # or humble

   export PYTHON_EXECUTABLE="$(command -v python3)"
   colcon build --symlink-install \
     --base-paths \
       shared/interfaces/autonomy_interfaces \
       services/autonomy/autonomy_core \
       services/autonomy/bt \
     --cmake-args "-DPYTHON_EXECUTABLE=${PYTHON_EXECUTABLE}"

   source install/setup.bash

Verify alignment
================

.. code-block:: bash

   python3 --version
   ls -d install/autonomy_interfaces/lib/python*

The directory name (e.g. ``python3.12``) must match ``python3``\ ’s major.minor.

Clean rebuild after a mismatch
==============================

.. code-block:: bash

   rm -rf build install log
   # then repeat the build steps above with ROS sourced and conda/venv off

**Colcon warnings** ``The path '.../install/autonomy_core' ... doesn't exist`` **in** ``AMENT_PREFIX_PATH``:
your shell still has a **previous** ``source install/setup.bash`` while ``install/`` was deleted. Either run
``./scripts/clean_ros_build.sh`` (it strips those entries), or open a **new terminal**, ``source
/opt/ros/$ROS_DISTRO/setup.bash`` only, then ``colcon build ...``. After a successful build, run
``source install/setup.bash`` again.

Running ROS Python tools
========================

- Use **system** ``python3`` (or the same path you passed as ``PYTHON_EXECUTABLE``) after
  ``source install/setup.bash``.
- For ``ros2 launch``, ensure ``python3-lark`` is installed if launch fails with
  ``ModuleNotFoundError: No module named 'lark'`` (see :doc:`../onboarding/ros2_workspace_and_bridges`).
- A **project venv** is fine for non-ROS tooling (formatters, some unit tests); **do not** build
  ROS interfaces or run ``rclpy`` against ``install/`` using a venv interpreter unless that venv
  intentionally matches the ROS distro Python.

Related
=======

- ``docs/development/BUILD_AND_TEST.md`` — build and blackboard test flow
- :doc:`hardware/l2_lidar_blackboard_e2e` — LiDAR bridge apt dependencies
