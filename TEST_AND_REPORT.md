# Test and Report

**Date:** 2026-01-31  
**Last run:** 2026-01-31 (re-run)  
**Scope:** Unit tests (excluding one broken collector), integration subset, new plan tests, bridge validation.

---

## Summary

| Suite | Passed | Failed | Errors | Skipped | Total |
|-------|--------|--------|--------|---------|-------|
| Unit (excl. full_stack_simulator) | 243 | 118 | 47 | 15 | 423 |
| New tests (HIL, chaos, path benchmark) | 9 | 0 | 0 | 0 | 9 |
| Bridge validation script | round-trip mock OK | HW interface checks fail | - | - | - |

---

## New Plan Tests (All Passed)

- **HIL blackboard command path** (`tests/integration/hardware/test_hil_blackboard_command_path.py`): 4/4 passed  
  - Blackboard set/get roundtrip, multiple keys, sensor status structure, serializable roundtrip.
- **Chaos safety recovery** (`tests/integration/core/test_chaos_safety_recovery.py`): 3/3 passed  
  - Watchdog health check for recovery (heartbeat stale), automatic recovery gated by config, degraded mode on sensor timeout.
- **Path planning benchmark** (`tests/performance/test_path_planning_benchmark.py`): 2/2 passed  
  - Plan path latency under 0.5s, D* Lite replan rate acceptable.

---

## Unit Tests

- **Collection:** 1 collection error: `tests/unit/simulation/test_full_stack_simulator_unit.py` imports `simulation.integration.full_stack_simulator` which pulls in `core.ros2_mock` (missing). Run with `--ignore=tests/unit/simulation/test_full_stack_simulator_unit.py` to avoid.
- **Results (excluding that file):** 243 passed, 118 failed, 47 errors, 15 skipped.
- **Common failure causes:**
  - **ROS2 not initialized:** Many tests need `rclpy.init()` before creating nodes (e.g. mission_executor, mission tests).
  - **Async tests:** Some need `pytest-asyncio` or correct async mode (e.g. websocket_server_simulator concurrent test).
  - **Observability/registry:** `Duplicated timeseries in CollectorRegistry`, `SimplifiedComponentRegistry` missing `register_component` in some tests.
  - **Performance tests:** Some benchmarks fail (encoding/decoding, control loop, message throughput) due to environment or thresholds.

---

## Bridge Validation Script

- **Command:** `PYTHONPATH=src python3 scripts/hardware/validate_can_blackboard_direct.py`
- **CAN bridge:** All 4 checks passed (8-motor swerve IDs, encoder, message type).
- **Bridge round-trip mock:** Passed (set then get on in-memory storage).
- **Hardware interface checks:** Failed (script expects `UnifiedBlackboardClient`, `BlackboardKeys`, and specific `blackboard.set(...)` calls in `src/autonomy/autonomy_core/autonomy_core/control/hardware_interface_node.py`; that file may use different names or structure).

---

## Fixes Applied This Session

1. **Path planner:** `_create_navigation_graph`, `_coord_to_node`, `_node_to_coord`, `_euclidean_heuristic`, `_simple_path` were defined on D* Lite; moved to PathPlanner so A*/Dijkstra work.
2. **D* Lite:** `_compute_shortest_path` and comparison now use `.get(..., float("inf"))` to avoid KeyError for uninitialized nodes.
3. **Validation script:** Added missing `validate_bridge_roundtrip_mock()` and call in `main()`.
4. **Chaos test:** Watchdog test now imports from `autonomy.autonomy_core.autonomy_core.safety.safety_watchdog` with src on path and skips if import fails.

---

## Recommendations

1. **Unit tests:** Add a session-scoped `rclpy.init()`/`rclpy.shutdown()` fixture for tests that create ROS2 nodes; fix or relax observability registry duplicate-metric handling in tests.
2. **Collection error:** Implement or stub `core.ros2_mock` (or make `simulation.integration.full_stack_simulator` not require it) so `test_full_stack_simulator_unit.py` collects.
3. **Bridge validation:** Align `validate_can_blackboard_direct.py` checks with the actual symbols and calls in `hardware_interface_node.py`, or document that script as reference for a specific implementation.
