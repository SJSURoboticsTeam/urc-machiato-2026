# Test Suite - URC Machiato 2026

Tests are organized by **type** (unit, integration, critical, performance) and by **component** under unit and integration.

## Layout

```
tests/
  unit/                    # Fast, isolated tests
    core/                  # src/core: state machine, blackboard, config, safety
    autonomy/              # BT, missions, navigation, perception
    infrastructure/        # Bridges, protocol, websocket, STM32
    simulation/            # Simulator unit tests
  integration/             # Cross-component tests
    core/                  # State machine, blackboard, recovery, dataflow
    autonomy/              # BT runtime, missions, navigation
    infrastructure/        # Network, bridges, websocket, ROS topics
    simulation/            # Simulation integration
    hardware/              # Hardware-in-the-loop, arm, LED
    system/                # Full system, E2E, architecture
    deployment/            # CI/CD, Docker, deployment readiness
    frontend/              # Dashboard/frontend integration
  critical/                 # Safety and recovery (must pass)
  performance/              # Latency, load, stress
  hardware/                  # Hardware validation (optional)
  communication/            # Communication stress
  slam/                      # SLAM-specific
  factories/                 # Test data factories
  fixtures/                  # Shared fixtures (JSON, etc.)
  conftest.py                # Shared pytest fixtures
```

## Run by type

```bash
# Unit only (fast)
python -m pytest tests/unit/ -v

# Integration only
python -m pytest tests/integration/ -v

# Critical systems
python -m pytest tests/critical/ -v

# Performance
python -m pytest tests/performance/ -v
```

## Run by component

```bash
# Unit: core (state machine, blackboard, config, safety)
python -m pytest tests/unit/core/ -v

# Unit: autonomy (BT, missions, navigation)
python -m pytest tests/unit/autonomy/ -v

# Unit: infrastructure (bridges, protocol, websocket)
python -m pytest tests/unit/infrastructure/ -v

# Integration: core
python -m pytest tests/integration/core/ -v

# Integration: autonomy (BT flow, missions)
python -m pytest tests/integration/autonomy/ -v

# Integration: system (full system, E2E)
python -m pytest tests/integration/system/ -v
```

## Run by marker

See `pytest.ini` for full marker list. Examples:

```bash
python -m pytest -m "unit" -v
python -m pytest -m "integration" -v
python -m pytest -m "state_machine" -v
python -m pytest -m "not slow" -v
python -m pytest -m "plan_5_1" -v
```

## Single test or file

```bash
python -m pytest tests/unit/core/test_blackboard_persistence.py -v
python -m pytest tests/integration/autonomy/test_complete_bt_state_machine_flow.py::test_name -v
```

## Coverage

```bash
python -m pytest tests/unit/ tests/integration/ --cov=src --cov-report=html --cov-report=term-missing
```

## Shared resources

- **conftest.py** – Session/module fixtures, ROS2 setup, path setup.
- **factories/** – Test data builders.
- **fixtures/** – Static fixtures (e.g. JSON).

**Run pytest from project root** with `python -m pytest tests/...` so conftest path setup applies and `src` resolves correctly:

```bash
cd /path/to/urc-machiato-2026
python -m pytest tests/unit/ -v
```

## Environment requirements

- **Unit tests**: No ROS2 required; PYTHONPATH is set by conftest when run from project root.
- **Integration (non-live)**: May require `autonomy_interfaces` built and on path (e.g. `source install/setup.bash` after colcon build).
- **Live blackboard / BT**: Require `bt_orchestrator` running and lifecycle configured (`ros2 lifecycle set /bt_orchestrator configure` then `activate`). Exclude in CI with: `python -m pytest -m "not live_blackboard" ...`.

When running the full suite without a ROS2 workspace (or without sourcing it), many integration tests skip (e.g. ~100+). For a fast, no-ROS2 subset run: `python -m pytest tests/unit/ -v`.

**Stable subset (unit tests, no ROS2):** From project root, run unit tests with the same ignores used by quality checks:

```bash
python -m pytest tests/unit/ -v --tb=short \
  --ignore=tests/unit/simulation/test_full_stack_simulator_unit.py \
  --ignore=tests/unit/infrastructure/test_slcan_protocol_simulator.py \
  --ignore=tests/unit/infrastructure/test_stm32_firmware_simulator.py
```

Or use the script: `./scripts/run_stable_tests.sh`.
