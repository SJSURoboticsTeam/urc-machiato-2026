# Traceability: Blackboard, BT, State Machine Plan

Maps each test to plan requirements. **Testing plan order (Step 1 -> 2 -> 3 -> 4):** see [BUILD_AND_TEST.md - Testing plan order](../../docs/development/BUILD_AND_TEST.md#testing-plan-order).

Run real tests with: `./scripts/run_blackboard_bt_state_machine_tests.sh` (emits JUnit XML to this directory). For full order use `./scripts/run_full_testing_plan_order.sh`. For live blackboard and BT+state machine tests (Steps 3-4), build ROS first: `./scripts/build_ros_for_bt_tests.sh` (see docs/development/BUILD_AND_TEST.md).

## Plan sections

| Section | Title |
|--------|--------|
| 1.1 | Blackboard hierarchical (dot) keys; C++/Python contract |
| 1.2 | Blackboard constants everywhere (no raw strings) |
| 1.3 | Blackboard persistence (save/load, safety: do not clear emergency_stop on load) |
| 2.1 | State machine hybrid flags (mode_request, human_override_active, auto_assist_enabled) |
| 2.2 | State machine safety checks before mode switch (battery, EMERGENCY_STOP from any state) |
| 3 | BT emergency-first and mode-based execution; shared control (command source priority) |
| 5.1 | Unit tests for blackboard, state machine, hybrid logic |

## Test-to-requirement matrix

| Test ID | Plan | Description |
|---------|------|--------------|
| unit/test_blackboard_keys_schema.py::TestBlackboardKeysSchema::test_flat_keys_exist | 1.1, 1.2 | Flat keys exist; constants contract for bridge nodes |
| unit/test_blackboard_keys_schema.py::TestBlackboardKeysSchema::test_dot_keys_auto_namespace | 1.1 | auto.* keys (mode, mode_request, human_override_active, assist_enabled) |
| unit/test_blackboard_keys_schema.py::TestBlackboardKeysSchema::test_dot_keys_system_namespace | 1.1 | system.* keys (emergency_stop, battery_percent, last_error) |
| unit/test_blackboard_persistence.py::TestBlackboardPersistenceLogic::test_persisted_keys_include_mission_and_mode | 1.3, 5.1 | Persisted key list includes mission and mode |
| unit/test_blackboard_persistence.py::TestBlackboardPersistenceLogic::test_load_skips_emergency_stop | 1.3 | Load does not overwrite system.emergency_stop (safety) |
| unit/test_blackboard_persistence.py::TestBlackboardPersistenceLogic::test_save_format_json | 1.3 | Save produces valid JSON |
| unit/test_hybrid_controller_logic.py::TestHybridControllerLogic::test_emergency_wins | 3, 5.1 | Command source: emergency when system.emergency_stop |
| unit/test_hybrid_controller_logic.py::TestHybridControllerLogic::test_override_over_mode | 3 | Command source: teleop when human_override_active |
| unit/test_hybrid_controller_logic.py::TestHybridControllerLogic::test_mode_when_no_override | 3 | Command source follows auto.mode when no override |
| unit/test_hybrid_controller_logic.py::TestHybridControllerLogic::test_emergency_stop_active_flat_key | 3 | Flat key emergency_stop_active also triggers emergency |
| unit/test_adaptive_state_machine_hybrid.py::TestAdaptiveStateMachineHybrid::test_emergency_stop_allowed_from_any_state | 2.2, 5.1 | EMERGENCY_STOP allowed from any state |
| unit/test_adaptive_state_machine_hybrid.py::TestAdaptiveStateMachineHybrid::test_battery_threshold_contract | 2.2 | Battery threshold for autonomous (e.g. 15%) |
| unit/test_adaptive_state_machine_hybrid.py::TestAdaptiveStateMachineHybrid::test_valid_transitions_idle_to_autonomous | 2.1 | Idle to autonomous/teleoperation transitions |
| unit/test_adaptive_state_machine_hybrid.py::TestAdaptiveStateMachineHybrid::test_mode_request_and_flags_contract | 2.1 | mode_request, human_override_active, auto_assist_enabled |

## Artifacts

- **JUnit XML**: `tests/reports/junit_blackboard_bt_state_machine.xml` (created by runner script; CI-parseable). Each `<testcase>` has `classname` and `name` for traceability.
- **Markers**: Tests are tagged with `@pytest.mark.plan_1_1`, `plan_1_2`, etc.

## Run by plan (real tests, traceable)

Use the runner script so JUnit XML is produced and ROS plugin is avoided:

```bash
./scripts/run_blackboard_bt_state_machine_tests.sh
```

To run only tests for a specific plan section (same runner env; from repo root with PYTHONPATH=src and ROS path stripped):

```bash
# Example: only Plan 1.3 (blackboard persistence)
python3 -m pytest tests/unit/test_blackboard_persistence.py -v -m plan_1_3 --junitxml=tests/reports/junit_plan_1_3.xml -p no:launch_testing -p no:launch_testing_ros_pytest_entrypoint
```

Marker selection: `-m plan_1_1`, `-m plan_1_2`, `-m plan_1_3`, `-m plan_2_1`, `-m plan_2_2`, `-m plan_3`, `-m plan_5_1`.
