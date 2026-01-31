# URC 2026 Test Suite

## Running tests

**Without full ROS2 workspace (recommended for quick checks):**

```bash
# From project root, with src on path
PYTHONPATH=src:$PYTHONPATH python3 -m pytest tests/test_critical_systems.py -v
```

Critical systems tests will pass or skip based on available dependencies (CAN bridge and sensor fusion run; RPi.GPIO, motor_controller, IntegratedCriticalSystems skip when missing).

**With ROS2 workspace (after `colcon build`):**

```bash
source install/setup.bash
python3 -m pytest tests/ -v --tb=short
```

**Unit tests under `tests/unit/`:** The ROS2 Jazzy `launch_testing` plugin can take over collection for `tests/unit/`; tests that need `autonomy_utilities` or `geometry_msgs` skip when those packages are not built/sourced. To run unit tests without launch_testing, use:

```bash
PYTHONPATH=src:$PYTHONPATH python3 -m pytest tests/unit/ -v -p no:launch_testing -p no:launch_testing_ros_pytest_entrypoint
```

(On some environments this can trigger a plugin hook error; in that case run after sourcing the ROS2 install.)

## Test categories

- **tests/test_critical_systems.py** – CAN bridge, sensor fusion, performance; optional RPi/motor/graceful-degradation/integrated (skip when deps missing).
- **tests/unit/** – Unit tests; some require `autonomy_utilities`, `geometry_msgs`, or ROS2.
- **tests/integration/** – Integration tests; many require ROS2 and simulation.
- **tests/performance/**, **tests/critical/**, **tests/extreme/** – Performance and stress tests.

## Skipped vs removed

Tests that require missing dependencies (RPi.GPIO, `motor_controller`, `autonomy_utilities`, `geometry_msgs`, etc.) are **skipped** via `pytest.importorskip` or try/except, not removed, so they run when the workspace or hardware is available.
