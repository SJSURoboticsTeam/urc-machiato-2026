# Test Coverage and Archived Code Cleanup

Summary of tests that still need to be conducted or fixed, and which archived code under `src/core` (and related) can be removed.

---

## 1. Other Tests That Need to Be Conducted

### Already in place (post-migration)

- **Critical systems:** `tests/critical/test_critical_systems_integration.py` (blackboard, state machine, BT with mocks).
- **System validation runner:** `scripts/run_system_validation.py` (smoke, by category, with-simulation).
- **Simulation-validated:** `tests/integration/test_simulation_validated_systems.py` (when simulation stack is available).
- **Realistic test data:** `tests/fixtures/realistic_test_data.py` (config/waypoints/safety limits).

### Recommended test runs (by area)

| Area | What to run | Notes |
|------|-------------|--------|
| **Critical** | `pytest tests/critical/test_critical_systems_integration.py -v` | No ROS2/hardware required. |
| **State machine (ROS2)** | `pytest tests/integration/test_state_transitions.py -v` | Needs ROS2 + autonomy_interfaces. |
| **Blackboard (ROS2)** | `pytest tests/integration/test_unified_blackboard.py -v` | Needs BT orchestrator or mock services. |
| **Component registry** | `pytest tests/unit/test_component_registry.py -v` | API mismatch with simplified registry (see below). |
| **Communication** | `scripts/run_system_validation.py --category communication` | WebSocket, bridges, network. |
| **Performance** | `pytest tests/performance/ -v` | Some tests import archived modules (fix or skip). |
| **Full validation** | `scripts/run_system_validation.py --all` | All categories. |

### Tests that need fixes before they are useful

These reference **archived or wrong** modules and will fail on import or at runtime:

| Test file | Issue | Action |
|-----------|--------|--------|
| `tests/unit/test_component_registry.py` | Uses `register_component(name, comp, deps)`, `.components`, `.dependencies`, `.health_status`, async `start_component`/`stop_component`. `SimplifiedComponentRegistry` has `register(name, class, priority)`, `get_component` (returns `ComponentInfo`), no dependencies dict, no async lifecycle. | Update tests to simplified API or mark as legacy and skip. |
| `tests/integration/test_state_machine_optimization.py` | Imports `AdaptiveStateMachine` from `src.core.simplified_state_manager`; only `UnifiedStateManager` and `get_state_manager()` exist there. | Change to `UnifiedStateManager`/`get_state_manager()` or skip. |
| `tests/performance/test_performance_regression.py` | Imports `core.state_synchronization_manager.Distributedget_state_manager()` (typo and archived). | Remove or rewrite to use current APIs. |
| `tests/performance/test_memory_leak_detection.py` | Imports `autonomy.code.state_management.autonomy_state_machine.states` (path may not exist). | Fix path or skip. |
| `tests/integration/test_advanced_systems_*.py` (several) | Import `core.state_synchronization_manager` / `Distributedget_state_manager()`. | Switch to simplified state/blackboard or skip. |
| `tests/integration/test_competition_bridge_advanced_systems.py` | Same `state_synchronization_manager` import. | Same as above. |
| `tests/integration/test_deployment_readiness.py` | Uses `src.core.component_registry` (old). Should use `src.core.simplified_component_registry`. | Update imports and API usage. |
| `tests/integration/test_redundancy_failover.py` | Imports `autonomy.code.state_management.autonomy_state_machine.states`. | Fix path or skip. |
| `tests/integration/test_basic_integration.py` | Imports from `src.autonomy.core.state_management` / `autonomy.code.state_management`. | Fix paths to current packages. |

### Scripts/entrypoints that reference old core modules

- **`src/cli/urc_cli.py`:** Imports `core.state_management`, `core.config_models`, `core.json_processor`, `core.statistics_processor`, `core.data_structures`, `core.transforms`. These top-level `core.*` modules do not exist (state lives in `simplified_state_manager`; config in `config_manager`/config package). CLI will fail or use stubs unless those imports are updated.
- **`src/run_full_test.py`:** Imports `core.state_management.create_state_machine`. Same as above; update to `simplified_state_manager` or remove.

---

## 2. Archived Code: What Can Be Removed

### Safe to remove (no active imports from production code)

| Path | Reason |
|------|--------|
| `src/core/archived_state_management/` | No active `src/` code imports it. Only `state_synchronization_manager.py` (inside the archive) imports `core.state_management`. Replaced by `simplified_state_manager.py`. |
| `src/core/archived_component_registry/` | No active code imports it. All use `simplified_component_registry`. |

**Before deleting:**

1. Fix or remove tests that still import archived state or old component registry (see table above).
2. Fix `src/cli/urc_cli.py` and `src/run_full_test.py` to use `simplified_state_manager` and current config/data modules (or drop unused imports).
3. Remove any references to `core.state_synchronization_manager` and `core.state_management` from tests and scripts.
4. Then run:

```bash
rm -rf src/core/archived_state_management/
rm -rf src/core/archived_component_registry/
```

### Do not remove yet (still used as fallback)

| Path | Reason |
|------|--------|
| `src/infrastructure/config/archived_config/` | `src/infrastructure/config/__init__.py` falls back to it when `simplified_config` fails. Remove only after simplified config is the only path (or fallback is removed). |
| `src/infrastructure/bridges/archived_circuit_breaker/` | `src/infrastructure/bridges/__init__.py` falls back to it when simplified circuit breaker fails. Same as above. |

### Optional cleanup after removing core archives

- **`tests/conftest.py`** (and `conftest_new.py`): Remove `"core.state_synchronization_manager"` from `modules_to_clear` once that module no longer exists.
- **`scripts/verify_simplifications.py`** and **`scripts/simplification_summary.py`**: Update or remove references to `archived_state_management` and `archived_component_registry` after deletion.

---

## 3. Summary

- **Tests to run:** Use `scripts/run_system_validation.py` (smoke/category/all) and the critical/simulation tests above; then add state-transition, blackboard, and communication runs as needed.
- **Tests to fix:** Component registry unit test (API), state machine optimization (import), performance and advanced_systems tests (state_synchronization_manager / wrong paths), deployment readiness and basic_integration (imports/paths).
- **Can remove now (after fixing tests/CLI):** `src/core/archived_state_management/`, `src/core/archived_component_registry/`.
- **Keep for now:** `src/infrastructure/config/archived_config/`, `src/infrastructure/bridges/archived_circuit_breaker/` until simplified versions are the only path.
