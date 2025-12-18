# Simulation Integration Issues - CRITICAL

## 🚨 Major Problem Identified

**Tests are importing from non-existent modules!**

### The Issue

1. **Tests import from wrong location**:

   ```python
   # Tests are doing this:
   sys.path.insert(0, os.path.join(PROJECT_ROOT, "tests", "simulation"))
   from environment_tiers import EnvironmentSimulator, EnvironmentTier
   from network_emulator import NetworkEmulator, NetworkProfile
   ```

2. **But these files don't exist**:
   - ❌ `tests/simulation/environment_tiers.py` - DOES NOT EXIST
   - ❌ `tests/simulation/network_emulator.py` - DOES NOT EXIST

3. **Actual simulation framework is here**:
   - ✅ `simulation/environments/environment_factory.py` - EXISTS
   - ✅ `simulation/network/network_emulator.py` - EXISTS
   - ✅ `simulation/environments/environment_factory.py` - EXISTS

### Affected Tests

All these tests have **broken imports**:

1. ❌ `test_state_machine_comprehensive.py`
2. ❌ `test_navigation_comprehensive.py`
3. ❌ `test_arm_control.py`
4. ❌ `test_mission_execution_comprehensive.py`
5. ❌ `test_advanced_safety.py`
6. ❌ `test_dataflow_consistency.py`
7. ❌ `test_vision_degradation.py` (partial - has its own simulator)

### Working Test

✅ **`test_ros_topic_comprehensive.py`** - Uses correct imports:

```python
from simulation.environments.environment_factory import EnvironmentFactory
from simulation.network.network_factory import NetworkFactory
```

## Root Cause

Tests were written expecting a `tests/simulation/` directory with:

- `environment_tiers.py` (with `EnvironmentSimulator` class)
- `network_emulator.py` (with `NetworkEmulator` class)

But the actual simulation framework uses:

- Factory pattern (`EnvironmentFactory`, `NetworkFactory`)
- Different class names and structure
- Located in `simulation/` (root level), not `tests/simulation/`

## Impact

**ALL comprehensive integration tests will fail to import simulation components!**

Tests will:

- Skip simulation gracefully (due to try/except)
- Run without actual simulation/degradation
- Not test across environment tiers properly
- Not test network conditions properly

## Solution Required

### Option 1: Create Bridge Module (Recommended)

Create `tests/simulation/environment_tiers.py` and `tests/simulation/network_emulator.py` that wrap the actual simulation framework:

```python
# tests/simulation/environment_tiers.py
from simulation.environments.environment_factory import EnvironmentFactory

class EnvironmentTier(Enum):
    PERFECT = "perfect"
    REAL_LIFE = "real_life"
    EXTREME = "extreme"

class EnvironmentSimulator:
    def __init__(self, tier: EnvironmentTier):
        factory = EnvironmentFactory()
        self.env = factory.create({"tier": tier.value})

    def apply_gps_degradation(self, gps_data):
        return self.env.apply_gps_degradation(gps_data)
    # ... wrap other methods
```

### Option 2: Update All Tests

Update all tests to use the actual simulation framework:

- Change imports to use factories
- Update code to use factory-created objects
- More work but uses actual framework

### Option 3: Move Simulation to tests/simulation/

Move simulation framework to `tests/simulation/` and update imports.

## Current Status

**Status**: ❌ **BROKEN** - Tests cannot import simulation components

**Tests using simulation correctly**: 1/8 (12.5%)
**Tests with broken imports**: 7/8 (87.5%)

## Immediate Action Required

1. ✅ Create bridge modules in `tests/simulation/`
2. ✅ Or update all test imports to use actual framework
3. ✅ Verify tests can actually import and use simulation
4. ✅ Ensure tests apply simulation degradation to data
