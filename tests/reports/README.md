# Test reports

This directory holds traceability and CI artifacts for the Blackboard/BT/State Machine plan.

- **TRACEABILITY_BLACKBOARD_BT_STATE_MACHINE.md** – Test-to-requirement matrix; updated when tests or plan change.
- **junit_blackboard_bt_state_machine.xml** – JUnit XML produced by `./scripts/run_blackboard_bt_state_machine_tests.sh` (real test run; CI can parse this).

**Full testing plan order (Step 1 -> 2 -> 3 -> 4):** see [BUILD_AND_TEST.md - Testing plan order](../../docs/development/BUILD_AND_TEST.md#testing-plan-order). Single entry point: `./scripts/run_full_testing_plan_order.sh` (runs 1-2, then 3-4 if nodes are available).

Run the traceable test suite (Step 1 only):

```bash
./scripts/run_blackboard_bt_state_machine_tests.sh
```

Run Steps 1-2 (validation ladder):

```bash
./scripts/run_bt_blackboard_state_machine_validation.sh
```

To build ROS for live blackboard and BT+state machine tests (Steps 3-4): `./scripts/build_ros_for_bt_tests.sh` (see [docs/development/BUILD_AND_TEST.md](../../docs/development/BUILD_AND_TEST.md)).
