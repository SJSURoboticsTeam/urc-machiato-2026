# ROS 2 workspace, blackboard, and WebSocket bridges (onboarding)

This guide completes the steps that are only summarized elsewhere (for example: “build workspace”, “start bt_orchestrator, rosbridge”). Follow it on a lab machine after cloning the repo and installing ROS 2 (Humble or Jazzy).

## Prerequisites

| Requirement | Notes |
|-------------|--------|
| ROS 2 | `source /opt/ros/$ROS_DISTRO/setup.bash` (Humble on 22.04, Jazzy on 24.04) |
| **Python = distro Python** | **Jazzy uses Python 3.12; Humble uses 3.10.** Build and run `rclpy` with **system** `python3` (not conda/venv), or bindings under `install/.../lib/pythonX.Y` will not load. See `docs/development/ros2_python_environment.rst`. |
| Python build tools | `pip install empy catkin_pkg` if `colcon build` fails on `em` or `catkin_pkg` (use `python3 -m pip install --user` with the same interpreter as ROS). |
| BT orchestrator build | `sudo apt install ros-$ROS_DISTRO-behaviortree-cpp` before building `autonomy_bt` |
| Rosbridge stack | `sudo apt install ros-$ROS_DISTRO-rosbridge-server ros-$ROS_DISTRO-rosapi` |
| Launch (`ros2 launch`) | If you see `No module named 'lark'`: `sudo apt install python3-lark` (and avoid venv shadowing `python3`). |
| Optional: LiDAR bridge | `sudo apt install ros-$ROS_DISTRO-pointcloud-to-laserscan` |

Use the **same** `ROS_DOMAIN_ID` in every terminal (default is fine if unset everywhere, or set e.g. `export ROS_DOMAIN_ID=42` in all shells and in the dashboard host).

## 1. Build the workspace (canonical paths for this repository)

From the **repository root** (where `shared/` and `services/` live):

```bash
cd /path/to/urc-machiato-2026
source /opt/ros/jazzy/setup.bash   # or humble

colcon build --symlink-install \
  --base-paths \
    shared/interfaces/autonomy_interfaces \
    services/autonomy/autonomy_core \
    services/autonomy/bt

source install/setup.bash
```

**Verify:**

```bash
ros2 pkg list | grep -E 'autonomy_(core|interfaces|bt)'
```

You should see `autonomy_core`, `autonomy_interfaces`, and `autonomy_bt`.

**Note:** `scripts/build_ros_for_bt_tests.sh` and `scripts/clean_ros_build.sh` use the same `--base-paths` as above and set `PYTHON_EXECUTABLE` for a consistent build.

## 2. Start core services: BT orchestrator (blackboard)

The dashboard and Python smoke tests call **`/blackboard/get_value`** and **`/blackboard/set_value`**, which are provided by the C++ **`bt_orchestrator`** lifecycle node, not by `proximity_monitor` alone.

**Terminal A:**

```bash
cd /path/to/urc-machiato-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42   # optional but must match other terminals

ros2 run autonomy_bt bt_orchestrator
```

**Terminal B** (same sourcing and `ROS_DOMAIN_ID`):

```bash
ros2 lifecycle set /bt_orchestrator configure
ros2 lifecycle set /bt_orchestrator activate
```

**Verify:**

```bash
ros2 service list | grep blackboard
# Expect: /blackboard/get_value and /blackboard/set_value

ros2 service call /blackboard/get_value autonomy_interfaces/srv/GetBlackboardValue "{key: 'mission_active', value_type: ''}"
```

If services are missing, the lifecycle node is not configured/activated yet.

More detail: `docs/development/BUILD_AND_TEST.md` (live blackboard steps).

## 3. Start core services: WebSocket bridge (rosbridge + rosapi)

The **dashboard** defaults to **`ws://localhost:9090`**. Use the packaged launch so the port matches the UI.

**Terminal C:**

```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch autonomy_core rosbridge_stack.launch.py
```

Optional: bind locally only: `address:=127.0.0.1`. To use another port, change **`rosbridge_port`** and point the dashboard at the same URL.

**Verify:**

```bash
nc -z 127.0.0.1 9090 && echo "rosbridge port open"
```

**Legacy note:** `scripts/launch/integrated_system.launch.py` uses port **9091**, which does not match the dashboard default. Prefer `rosbridge_stack.launch.py` unless you intentionally reconfigure the UI.

## 4. Optional: HIL serial bridges (WebSocket + CAN serial in one launch)

For **dashboard WebSocket + `hardware_interface`** (serial device, e.g. SLCAN) in one command:

```bash
export URC_STM32_SERIAL=/dev/ttyACM0   # optional; default in launch is /dev/ttyACM0
ros2 launch autonomy_core hil_serial_bridges.launch.py
```

You still need **`bt_orchestrator`** (above) if you want blackboard services for the dashboard.

## 5. Optional: Autonomy nodes and LiDAR path

- **Unified stack:** `ros2 launch autonomy_core unified.launch.py mode:=simulation` (does not start rosbridge or BT by default).
- **LiDAR → `/scan`:** `ros2 launch autonomy_core unitree_l2_pc2_to_scan.launch.py` after the L2 driver publishes a cloud.
- **End-to-end LiDAR → blackboard:** `docs/hardware/l2_lidar_blackboard_e2e.rst`.

## 6. Optional: Dashboard and smoke tests

**Dashboard** (`services/dashboard/`):

```bash
cd services/dashboard
npm install
npm run dev
```

Connect the UI to **`ws://localhost:9090`** (or your `rosbridge_port`).

**Blackboard smoke test** (with `bt_orchestrator` active):

Use **system** `python3` (same version the workspace was built with — see `docs/development/ros2_python_environment.rst`).

```bash
cd /path/to/urc-machiato-2026
source install/setup.bash
python3 scripts/hardware/lidar_blackboard_smoke_test.py
```

**Skeletal HIL checklist:** `docs/hardware/skeletal_hil_checklist.rst`.

## 7. Troubleshooting

| Symptom | Check |
|---------|--------|
| `colcon` ignores `autonomy_core` | Use `--base-paths` including `services/autonomy/autonomy_core` |
| No `/blackboard/*` services | Start `bt_orchestrator` and run lifecycle **configure** then **activate** |
| Dashboard never connects | Same `ROS_DOMAIN_ID`; rosbridge running; URL matches port (default 9090) |
| `get_value` timeouts | Duplicate `bt_orchestrator` processes; see `BUILD_AND_TEST.md` |
| `libpython3.N.so` / `UnsupportedTypeSupport` for `autonomy_interfaces` | Workspace built with wrong Python (e.g. 3.13 venv). `rm -rf build install log`, deactivate conda/venv, rebuild with `PYTHON_EXECUTABLE=$(command -v python3)` — see `docs/development/ros2_python_environment.rst` |

## Related documentation

- `docs/operations/deployment.rst` – deploy modes and extras
- `docs/hardware/skeletal_hil_checklist.rst` – sensor and TF bring-up
- `docs/hardware/l2_lidar_blackboard_e2e.rst` – LiDAR and blackboard
- `test_reports/launch_websocket_blackboard_summary_report.md` – infrastructure validation summary
