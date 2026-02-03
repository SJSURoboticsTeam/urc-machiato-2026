# URC 2026 System Test Report

**Generated**: 2026-02-01T00:58:39.438335
**Environment**: ROS2 Jazzy, Python 3.12.3

## Executive Summary

- **Test Suites**: 4
- **Total Tests**: 19
- **Passed**: 0 (0%)
- **Failed**: 19
- **Skipped**: 2

⚠️  **19 tests failed**

## Test Results by Category

### Communication Stack

**Status**: ❌ FAIL (7 failures)
- Passed: 0
- Failed: 7
- Skipped: 0
- Total: 7

**Failed Tests**:

- `tests/unit/test_websocket_server_simulator.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.
- `tests/unit/test_bridge_communications.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.
- `tests/unit/test_simple_websocket_bridge.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.
- `tests/integration/test_websocket_bridge_integration.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.
- `tests/integration/test_complete_communication_stack.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.
- `tests/integration/test_network_integration.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.
- `tests/integration/test_network_resilience.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.

### Cognition System

**Status**: ❌ FAIL (7 failures)
- Passed: 0
- Failed: 7
- Skipped: 0
- Total: 7

**Failed Tests**:

- `tests/unit/test_bt_orchestrator_implementation.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.
- `tests/unit/test_behavior_tree_failures.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.
- `tests/unit/test_mission_executor.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.
- `tests/integration/test_complete_bt_state_machine_flow.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.
- `tests/integration/test_bt_state_machine_runtime.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.
- `tests/integration/test_mission_system.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.
- `tests/integration/test_bt_runtime_integration.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.

### Full System Integration

**Status**: ❌ FAIL (4 failures)
- Passed: 0
- Failed: 4
- Skipped: 0
- Total: 4

**Failed Tests**:

- `tests/integration/test_bridges.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.
- `tests/integration/test_mission_validation.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.
- `tests/integration/test_comprehensive_bt_integration.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.
- `tests/integration/test_ros2_state_machine_bridge.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.

### Performance & Stress Testing

**Status**: ❌ FAIL (1 failures)
- Passed: 0
- Failed: 1
- Skipped: 2
- Total: 1

**Failed Tests**:

- `tests/performance/stress_test_network_communication.py`
  Error: ============================= test session starts ==============================
platform linux -- Python 3.12.3, pytest-9.0.2, pluggy-1.6.0 -- /usr/bin/python3
cachedir: .pytest_cache
benchmark: 5.2.

## Recommendations for Hardware-in-the-Loop Testing

⚠️  **19 systems need attention before hardware testing**

### Issues to Address:
- Communication Stack: Fix 7 failing tests
- Cognition System: Fix 7 failing tests
- Full System Integration: Fix 4 failing tests
- Performance & Stress Testing: Fix 1 failing tests

---
*Report Generated: 2026-02-01T00:58:39.438379*
