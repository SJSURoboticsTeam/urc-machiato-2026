# ROS2 Software Test Assessment

Full pytest run (excluding `tests/factories` and `tests/performance` due to collection errors). ROS2 Jazzy sourced; simulated data / no hardware.

**Run date:** 2026-01-31  
**Command:** `python3 -m pytest tests/ -v --tb=short --ignore=tests/factories --ignore=tests/performance`

---

## Summary

| Result   | Count |
|----------|--------|
| Passed   | 13    |
| Failed   | 37    |
| Skipped  | 26    |
| Errors   | 18    |

**Collection errors (not run):**
- `tests/factories` – `NameError: name 'FactoryFaker' is not defined` in `test_data_factories.py`
- `tests/performance` – `ModuleNotFoundError: No module named 'src.comms'` (test_performance_baseline.py)

---

## What Works (Passed)

| Test | File |
|------|------|
| test_available_topics | tests/test_debug_topics.py |
| test_integration_with_missing_dependencies | tests/test_fallback_functionality.py |
| test_configuration_validation | tests/test_mock_simulation_validation.py |
| test_error_handling_robustness | tests/test_mock_simulation_validation.py |
| test_mission_executor_invalid_mission_type | tests/test_mock_simulation_validation.py |
| test_mission_executor_navigation_mission | tests/test_mock_simulation_validation.py |
| test_mission_executor_status_tracking | tests/test_mock_simulation_validation.py |
| test_safety_manager_multiple_sensor_failures | tests/test_mock_simulation_validation.py |
| test_safety_manager_normal_operation | tests/test_mock_simulation_validation.py |
| test_safety_manager_single_sensor_failure | tests/test_mock_simulation_validation.py |
| test_sensor_data_validation | tests/test_mock_simulation_validation.py |
| test_state_machine_invalid_transitions | tests/test_mock_simulation_validation.py |
| test_state_machine_valid_transitions | tests/test_mock_simulation_validation.py |

**Strong area:** `test_mock_simulation_validation.py` – mission executor, safety manager, state machine, and sensor validation all pass with mocked/simulated data.

---

## What Fails (by cause)

### 1. Missing modules (src.bridges, src.autonomy.perception, src.autonomy.core, src.comms)

- **src.bridges:** Communication stress (test_communication_stress.py), critical systems CAN bridge and performance (test_critical_systems.py). Tests expect a `src.bridges` package.
- **src.autonomy.perception:** Sensor fusion and perception fallback (test_critical_systems.py, test_fallback_functionality.py).
- **src.autonomy.core:** SLAM performance (test_slam_performance.py), terrain fallback (test_fallback_functionality.py).
- **src.comms:** Performance baseline (test_performance_baseline.py) – entire `tests/performance` not run due to collection error.

**Action:** Either add/restore these packages or adjust tests to skip/use stubs when the package is absent.

### 2. CANBusMockSimulator API (port argument)

- **tests/hardware/test_hardware_interface_with_simulator.py** – 12 failures: `CANBusMockSimulator.__init__() got an unexpected keyword argument 'port'` (and one `KeyError: 'can_port'`).

**Action:** Align simulator constructor with tests: add `port` (and config `can_port`) to the mock or stop passing them in tests.

### 3. feature_flags.py syntax error (line 379)

- **test_fallback_functionality.py:** test_feature_flags_fallback, test_integration_with_missing_dependencies (feature_flags).
- **test_integration.py:** test_concurrent_access_safety, test_cross_component_state_consistency, test_end_to_end_mission_profile_switching, test_mission_resource_manager_feature_flags_integration, test_system_stability_during_transitions.

**Action:** Fix syntax at `src/core/feature_flags.py` line 379.

### 4. RoverConfig has no attribute 'get'

- **test_recovery_mechanisms.py:** all 5 tests (e.g. test_cpu_pressure_recovery, test_memory_pressure_recovery).
- **test_fallback_functionality.py:** test_resource_manager_fallback.

**Action:** Use a dict-like config in tests or add a `get()` method (or compatible interface) to `RoverConfig`.

### 5. Other import/API mismatches

- **test_integration.py::test_behavior_tree_resource_manager_integration** – `cannot import name 'circuitbreaker' from 'src.core.error_handling'`.
- **test_integration.py::test_monitoring_resource_manager_integration** – `No module named 'simulation.tools.monitoring_dashboard'`.

**Action:** Fix exports in `src/core/error_handling.py` and add or stub `simulation.tools.monitoring_dashboard` (or skip if not implemented).

---

## Skipped (26)

Many skips are intentional (e.g. ROS2/launch not available, optional deps, hardware). Examples:

- test_critical_systems.py: TestHardwareEmergencyStop (5), TestAdvancedMotorControl (6), TestGracefulDegradation (4), TestIntegratedCriticalSystems (5), TestCompetitionScenarios (2), TestCompleteCommunicationStack (1).
- Others: launch_testing, state_synchronization_manager removed, BT action server not available, etc.

---

## ROS2 simulated-data flow (separate from pytest)

The **CAN software test** (simulated CAN, no hardware) is not part of pytest. Run it manually:

```bash
cd /home/durian/urc-machiato-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./scripts/hardware/run_can_tests.sh
# Choose option 1 (Software Test)
```

This starts `hardware_interface` with mock CAN, runs the CAN testing dashboard, and exercises CAN-to-blackboard with simulated data. Success here is independent of the pytest results above.

---

## Recommended next steps

1. **Fix collection:** Add `FactoryFaker` (or correct import) in `tests/factories/test_data_factories.py`; fix or stub `src.comms` for `tests/performance`.
2. **Fix feature_flags.py** syntax at line 379 so fallback and integration tests can run.
3. **Align RoverConfig** with tests (e.g. support `get`) for recovery and resource-manager fallback tests.
4. **Update CAN hardware simulator tests** to match `CANBusMockSimulator` API (port/can_port).
5. **Decide on src.bridges / perception / core:** Either restore packages or make tests skip when the module is missing and document the intended layout.

---

## How to re-run this assessment

```bash
cd /home/durian/urc-machiato-2026
source /opt/ros/jazzy/setup.bash   # or humble
source install/setup.bash
python3 -m pytest tests/ -v --tb=short \
  --ignore=tests/factories --ignore=tests/performance \
  2>&1 | tee /tmp/pytest_assessment.log
```

Full log was saved to `/tmp/pytest_full.log` (from the run that produced this assessment).
