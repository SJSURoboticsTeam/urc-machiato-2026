# ROS2 Build and Test - Quick Start

## Build ROS2 Packages

The workspace root contains `COLCON_IGNORE`, so packages are built using explicit `--base-paths`. From the project root:

```bash
cd /path/to/urc-machiato-2026
source /opt/ros/jazzy/setup.bash   # or /opt/ros/humble/setup.bash

colcon build --base-paths \
  src/autonomy/interfaces/autonomy_interfaces \
  src/autonomy/autonomy_core \
  src/autonomy/bt \
  src/simulation/gazebo_simulation \
  src/vision_processing \
  --symlink-install
source install/setup.bash
```

### System dependency for autonomy_bt (BT/blackboard tests)

If you want to build the BT orchestrator (for live blackboard and BT+state machine tests), install the behaviortree_cpp package first:

- **ROS2 Jazzy (Ubuntu 24.04)**: `sudo apt install ros-jazzy-behaviortree-cpp`
- **ROS2 Humble (Ubuntu 22.04)**: `sudo apt install ros-humble-behaviortree-cpp` (if available; otherwise see [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP) for building from source)

Then use the helper script (checks ROS and behaviortree_cpp, then builds):

```bash
source /opt/ros/jazzy/setup.bash
./scripts/build_ros_for_bt_tests.sh
source install/setup.bash
```

## Clean rebuild

To remove all colcon artifacts and rebuild from scratch:

```bash
rm -rf build install log
source /opt/ros/jazzy/setup.bash   # or humble
colcon build --base-paths \
  src/autonomy/interfaces/autonomy_interfaces \
  src/autonomy/autonomy_core \
  src/autonomy/bt \
  src/simulation/gazebo_simulation \
  src/vision_processing \
  --symlink-install
source install/setup.bash
```

Or run the script:

```bash
./scripts/clean_ros_build.sh
source install/setup.bash
```

## Verify Build

```bash
# Check packages are available
ros2 pkg list | grep -E 'autonomy|gazebo|vision'

# Should show (depending on what you built):
# autonomy_bt
# autonomy_core
# autonomy_interfaces
# gazebo_simulation   (if built)
# vision_processing  (if built)
```

## Testing plan order

Run steps in order: **1 -> 2 -> 3 -> 4**.

**Single script (easiest):** From project root (no venv), run:

- `./scripts/test_all_blackboard_bt.sh` – Step 1 + 2; Step 3+4 only if bt_orchestrator (and adaptive_state_machine) are already running.
- `./scripts/test_all_blackboard_bt.sh --all` – Step 1 + 2 + 3: starts bt_orchestrator, runs live blackboard tests, stops orchestrator (no separate terminals).
- `./scripts/test_all_blackboard_bt.sh --all --with-step4` – Step 1–4: also starts adaptive_state_machine and runs Step 4, then stops both nodes.

| Step | What | Prereqs | Command |
|------|------|---------|--------|
| 1 | Unit tests (blackboard keys, persistence, hybrid logic, state machine) | None (no ROS build) | `./scripts/run_blackboard_bt_state_machine_tests.sh` |
| 2 | Contract integration tests (BT/state machine source checks, no live ROS) | None (no live ROS) | `./scripts/run_bt_blackboard_state_machine_validation.sh` (runs Step 1 then Step 2) |
| 3 | Live blackboard (unified blackboard client vs bt_orchestrator) | ROS built, `source install/setup.bash`, bt_orchestrator running **and** lifecycle configured+activated | In a test terminal: **do not use the project venv** (run `deactivate` if active). Then: `source install/setup.bash`, `export PYTHONPATH=$(pwd)/src:$(pwd):${PYTHONPATH:-}` (prepend only; do not overwrite PYTHONPATH or rclpy is lost), then `python3 -m pytest tests/integration/core/test_unified_blackboard.py -v --tb=short` |
| 4 | BT + state machine runtime | Same as Step 3 plus adaptive_state_machine running | Same terminal: `python3 -m pytest tests/integration/autonomy/test_bt_state_machine_runtime.py -v --tb=short` |

**Step 2 only (contract tests):** `PYTHONPATH=src:tests python3 -m pytest tests/integration/autonomy/test_bt_state_machine_integration.py tests/integration/autonomy/test_complete_bt_state_machine_flow.py -v --tb=line -p no:launch_testing -p no:launch_testing_ros_pytest_entrypoint`

**Quick full sequence:** Run `./scripts/test_all_blackboard_bt.sh` (Steps 1-2; Step 3+4 if nodes already running), or `./scripts/test_all_blackboard_bt.sh --all` to run Steps 1-3 in one go (script starts bt_orchestrator). For Steps 1-2 only without ROS: `./scripts/run_bt_blackboard_state_machine_validation.sh`. To run Step 3-4 manually: build ROS (`./scripts/build_ros_for_bt_tests.sh`), start bt_orchestrator (and lifecycle configure/activate), then run the Step 3 and Step 4 pytest commands above.

## Testing BT and blackboard (Steps 3-4)

Steps 3 and 4 require `rclpy` (ROS2 Python). Use a terminal where the **project venv is not activated** so that `python3` is the ROS2-provided interpreter. If you use a venv, run `deactivate` first, then source the workspace and run the pytest commands below.

**Preconditions**

- bt_orchestrator must be running in one terminal.
- In a second terminal (with workspace sourced): run `ros2 lifecycle set /bt_orchestrator configure` then `ros2 lifecycle set /bt_orchestrator activate`.
- **Same ROS_DOMAIN_ID** in all terminals (if set). If you use `ROS_DOMAIN_ID`, set it the same in the terminal where you run bt_orchestrator and in the terminal where you run pytest; otherwise the test cannot see the service.

After building with autonomy_bt and sourcing the workspace:

1. Start the BT orchestrator (provides blackboard services). **bt_orchestrator is a lifecycle node**; the `/blackboard/get_value` and `/blackboard/set_value` services are only created after configure. You must configure and activate before running Step 3:
   ```bash
   ros2 run autonomy_bt bt_orchestrator
   ```
   In another terminal (with the workspace sourced):
   ```bash
   ros2 lifecycle set /bt_orchestrator configure
   ros2 lifecycle set /bt_orchestrator activate
   ```

2. **Verify service**: Before running Step 3, check that blackboard services are visible: `ros2 service list | grep blackboard` should show `/blackboard/get_value` and `/blackboard/set_value`. If not, ensure bt_orchestrator is running and configured+activated (and check ROS_DOMAIN_ID in all terminals).

3. In another terminal **without the project venv** (run `deactivate` if needed), then `source install/setup.bash` and `export PYTHONPATH=$(pwd)/src:$(pwd):${PYTHONPATH:-}` (prepend; do not overwrite PYTHONPATH):
   - **Step 3 – Live blackboard**: `python3 -m pytest tests/integration/core/test_unified_blackboard.py -v --tb=short`
   - **Step 4 – BT + state machine runtime**: Start `adaptive_state_machine` as well, then run `python3 -m pytest tests/integration/autonomy/test_bt_state_machine_runtime.py -v --tb=short`

**Why get_value times out with real bt_orchestrator:** bt_orchestrator uses a multi-threaded executor so blackboard service callbacks run alongside BT/telemetry timers. If you see "failed to send response to /blackboard/get_value (timeout)" on the server, rebuild autonomy_bt (the fix is in main()).

**Duplicate bt_orchestrator:** If you see "Publisher already registered for node name: 'bt_...'" and get_value timeouts, two bt_orchestrator instances are running. Stop all: `pkill -f bt_orchestrator`, then run the script again. The script with `--all` stops any existing bt_orchestrator before starting its own.

**Blackboard-only mode:** When you run `test_all_blackboard_bt.sh --all`, the script starts bt_orchestrator with `run_bt_tick:=false` so the BT execution and telemetry timers are disabled. Only blackboard services run, so get_value/set_value are not starved by the BT. For full BT behavior (e.g. manual testing), run bt_orchestrator without that parameter.

### Simpler way to test blackboard (no bt_orchestrator)

To run the blackboard integration tests **without** building or running the C++ bt_orchestrator, use the mock blackboard service. The script uses `ROS_DOMAIN_ID=42` to isolate the mock from other nodes; run with no other ROS nodes in that domain (or in a clean terminal).

**One command (mock + pytest):**
   ```bash
   source install/setup.bash
   ./scripts/run_blackboard_integration_with_mock.sh
   ```

**Or manually (two terminals, same env):**
1. **Terminal 1:** `export ROS_DOMAIN_ID=42 && source install/setup.bash && export PYTHONPATH=$(pwd)/src:$(pwd):${PYTHONPATH:-} && python3 scripts/hardware/mock_blackboard_service.py`
2. **Terminal 2:** `export ROS_DOMAIN_ID=42 && source install/setup.bash && export PYTHONPATH=$(pwd)/src:$(pwd):${PYTHONPATH:-} && python3 -m pytest tests/integration/core/test_unified_blackboard.py -v --tb=short`

If get_value still times out with the mock, ensure no other process (e.g. CAN bridge, bt_orchestrator) is using the same domain or writing to the mock.

## Test CAN → Blackboard Connection

### Option 1: Automated Test Suite (Recommended)

```bash
./scripts/hardware/run_can_tests.sh
```

Select option 1 for software testing.

### Option 2: Manual Testing

**Terminal 1** - Start hardware_interface:
```bash
./scripts/hardware/start_hardware_interface.sh
```

**Terminal 2** - Monitor messages:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/hardware/can_testing_dashboard.py
```

## What You Should See

The dashboard will show:
- Battery: 24.0V, 85% (mock data)
- Velocity: 0.0 m/s
- Message rate: 10 Hz
- Blackboard: AVAILABLE

In the hardware_interface terminal, look for:
```
[INFO] [hardware_interface]: Hardware Interface Node initialized
[INFO] [hardware_interface]: Blackboard client initialized - direct CAN->blackboard writes enabled
```

## Build Issues

If colcon can't find packages, ensure you are in the repo root and use the explicit `--base-paths` above (workspace root has `COLCON_IGNORE`).

If you get import or stale-build errors, clean and rebuild using `./scripts/clean_ros_build.sh` or the clean rebuild commands above.

If autonomy_bt fails with "behaviortree_cpp not found", install it: `sudo apt install ros-jazzy-behaviortree-cpp` (or ros-humble-behaviortree-cpp).

## Testing Checklist

- [ ] ROS2 packages built successfully
- [ ] `ros2 pkg list` shows autonomy_core, autonomy_interfaces, autonomy_bt (and optionally gazebo_simulation, vision_processing)
- [ ] hardware_interface starts without errors
- [ ] Dashboard shows messages at 10 Hz
- [ ] "Blackboard client initialized" appears in logs
- [ ] No import errors

## Next Steps

Once software testing works:
1. Test with real hardware (tomorrow)
2. Verify blackboard writes in logs
3. Test velocity commands
4. Test emergency stop
5. Full autonomy integration

---

**Quick command**: `./scripts/hardware/run_can_tests.sh` (select option 1)
