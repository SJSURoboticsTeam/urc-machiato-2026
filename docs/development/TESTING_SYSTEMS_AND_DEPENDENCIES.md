# Testing systems, compatibility, and dependency chain

This document maps **which systems must be tested**, how they depend on each other (**upstream → downstream**), and what must be verified **before** downstream tests are meaningful.

## 1. Layered system map (what we are testing)

| Layer | Systems | Role |
|-------|---------|------|
| **L0 – Tooling** | Python 3.10, pytest, plugins | Runs all other tests |
| **L1 – Workspace layout** | `PYTHONPATH` = repo root; imports resolve `services/`, `shared/` | Without this, collection and imports fail |
| **L2 – ROS2 base** | `rclpy`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `launch_ros`, sourced `install/` | Real ROS integration and some launch tests |
| **L3 – Custom interfaces** | `autonomy_interfaces` (built with colcon, workspace sourced) | Blackboard services, mission types, safety msgs |
| **L4 – Python stack** | `websockets`, `aiohttp`, `py_trees`, `tenacity`, `serial` (pyserial), `cv_bridge` | Bridges, BT, resilience, CAN, vision nodes |
| **L5 – Simulation Python** | `services/simulation/python/**` (WebSocket stub, SLCAN, firmware, ROS2 adapter, full stack) | End-to-end *logical* pipeline without Gazebo |
| **L6 – ROS2 C++ / BT** | `autonomy_bt`, BT orchestrator, behavior tree XML | BT runtime tests; must be built and on `PATH`/`AMENT_PREFIX_PATH` |
| **L7 – Gazebo / graphics** | Gazebo, `DISPLAY`, GPU/GL (often absent in CI) | Full physics sim; optional for most pytest suites |
| **L8 – Hardware / HIL** | CAN, STM32, real sensors | Marked `hardware`; excluded in Docker `-k 'not hardware'` |

**Compatibility rule:** downstream layers assume upstream layers are satisfied. A failure at L3 (e.g. missing `autonomy_interfaces`) is an **upstream** problem; failing “mission workflow” tests are often **downstream symptoms**.

## 2. Upstream requirements checklist (verify these first)

### 2.1 Docker test runners (`docker/docker-compose.test.yml`)

| Runner | Upstream must include | Common gap (your logs) |
|--------|------------------------|-------------------------|
| **unit-tests** | `pytest`, `pytest-cov`, `pytest-asyncio`, `pytest-mock` | OK if pip line matches needs |
| **integration-tests** | Above + `websockets`, `aiohttp` | Was missing; now added in compose command |
| **simulation-tests** | Above + `websockets`, `aiohttp` | Same; **Gazebo not started** by compose – tests are mostly Python sim |
| **performance-tests** | Above + **`pytest-benchmark`** | **Was missing** → `unrecognized arguments: --benchmark-only` |
| **all-tests** | Full dev dependencies | Should align with `pyproject.toml` optional/dev deps |

**Action:** Keep compose `pip install` lines in sync with `pyproject.toml` `[project.optional-dependencies]` / dev deps, or bake them into `autonomy.Dockerfile`.

### 2.2 `pyproject.toml` pytest `addopts`

Global `addopts` includes `--cov=...`. That applies to every pytest invocation unless overridden.

- **Upstream:** `pytest-cov` installed in the container.
- **Downstream:** Slightly slower runs; not usually a failure cause.

### 2.3 ROS2 “real” vs “mock”

| Mode | Upstream | Downstream tests that work |
|------|----------|----------------------------|
| **Mock** | `shared.core.ros2_mock`, no `install/setup.bash` | Many unit tests; adapter tests that use mock `Twist` |
| **Real** | Humble/Jazzy sourced + `colcon build` + `source install/setup.bash` | Nodes, `launch_ros`, real `sensor_msgs` types, `autonomy_interfaces` |

**Compatibility:** code that imports `LaserScan` / `PoseWithCovarianceStamped` needs **`ros-humble-sensor-msgs`** (and friends), not only `ros-core`. `ros:humble-ros-core` is a **narrow** upstream; many integration tests will skip or fail until the image or install step adds message packages.

### 2.4 Async tests

| Upstream | Downstream |
|----------|------------|
| `pytest-asyncio` installed + correct mode/markers | `async def test_*` in `test_ros2_simulation_integration.py` actually runs coroutines |

If async tests are not marked (depending on `asyncio_mode`), handlers may never run → `ros2_state` stays `None`.

## 3. Downstream test suites (what each validates)

| Suite | Primary downstream concern |
|-------|----------------------------|
| **Unit** | Single modules; mocks for ROS/hardware |
| **Integration** | Cross-package imports, bridges, state, data flow |
| **Simulation (pytest)** | Python full-stack sim, Mars scenario *logic*, ROS2 adapter in process |
| **Performance** | Benchmarks; requires `pytest-benchmark` |
| **colcon test** | ROS packages, launch_testing, C++ BT – separate from Docker pytest |

## 4. Failure interpretation guide (from recent logs)

| Symptom | Likely upstream cause | Downstream fix |
|---------|----------------------|----------------|
| `unrecognized arguments: --benchmark-only` | `pytest-benchmark` not installed in container | Add to compose pip install or Dockerfile |
| `MockRCLPY` / `launch_ros` AttributeError | Test env mocks or incomplete `rclpy` | Use real sourced workspace or skip launch tests in slim images |
| `cannot import name 'LaserScan'` | `ros-core` only image | Install `sensor-msgs` / use `ros-base` or apt packages in Dockerfile |
| Mars / mission tests flaky | `random` in comms + missions + strict thresholds | Deterministic seeds; reset robot state between missions; fix timing assertions |
| Full-stack ROS2 tests fail | Async not awaited; handler not run | `@pytest.mark.asyncio` on async tests |

## 5. Recommended order of operations (when triaging CI)

1. **Image / pip:** Does the container have every plugin and library the test file imports?
2. **ROS profile:** Is this run intended to be mock-only or real ROS? Match `Dockerfile`/`compose` to that decision.
3. **Workspace:** `PYTHONPATH` and `colcon` `install` sourced if testing real messages.
4. **Test design:** Determinism (seed, state reset), async markers, and assertions that match scaled delays (e.g. comms sleep vs minimum delay).

## 6. Maintaining compatibility over time

- When moving packages under `services/` or `shared/`, update **imports and `patch()` targets** in the same change set.
- Treat **compose pip one-liners** as part of the public test contract; bump them when `pyproject.toml` dev deps change.
- Prefer **one** ROS strategy per job: either a **fat** ROS image + sourced install, or a **slim** image with skips and mocks – mixing without documenting causes false “downstream” failures.

---

**Related:** `INTEGRATION_TEST_FIX_PLAN.md`, `docker/docker-compose.test.yml`, `scripts/fix_integration_test_paths.sh`.
