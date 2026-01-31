# Removed Performance Test Files

## Removed on 2026-01-30

### Duplicate/Unused Performance Tests

These files were removed as they were either duplicates or not referenced by any scripts:

1. **simple_performance_test.py** (154 lines)
   - Near-duplicate of `performance_test.py` (only formatting differences)
   - Not referenced by any scripts
   - Functionality: Tests lightweight_core startup and component loading
   - **Keep:** `performance_test.py` (referenced by `tests/run_comprehensive_testing.py`)

2. **performance_test_suite.py** (154 lines)
   - Exact duplicate of `simple_performance_test.py`
   - Not referenced by any scripts
   - Functionality: Same as simple_performance_test.py

3. **quick_performance_test.py** (436 lines)
   - Not referenced by any scripts
   - Functionality: Tests ROS2 intra-process communication performance
   - Note: Different from other tests (ROS2-specific), but unused

**Active Performance Tests (kept):**
- `performance_test.py` - Core performance testing (referenced by run_comprehensive_testing.py)
- `test_performance_baseline.py` - Baseline measurements
- `test_performance_regression.py` - Regression tracking
- `stress_test.py` - Stress testing
- And 30+ other specialized performance tests

**Rationale:** These files were duplicates or unused, adding confusion without benefit. The remaining test suite provides comprehensive coverage.

**Risk Mitigation:** Files backed up in git history. Can be restored with:
```bash
git show HEAD~1:tests/performance/simple_performance_test.py > tests/performance/simple_performance_test.py
```

## Recovery

If you need to restore these files:
```bash
# Restore from git history (before this commit)
git log --all --full-history -- tests/performance/simple_performance_test.py
git checkout <commit-hash> -- tests/performance/simple_performance_test.py
```
