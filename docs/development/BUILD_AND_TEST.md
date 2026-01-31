# ROS2 Build and Test - Quick Start

## Build ROS2 Packages

The workspace uses a nested package structure. Build with explicit paths:

```bash
cd /home/durian/urc-machiato-2026
source /opt/ros/jazzy/setup.bash

# Build autonomy packages
colcon build --base-paths src/autonomy/autonomy_core src/autonomy/interfaces/autonomy_interfaces --symlink-install

# Source the workspace
source install/setup.bash
```

## Verify Build

```bash
# Check packages are available
ros2 pkg list | grep autonomy

# Should show:
# autonomy_core
# autonomy_interfaces
```

## Test CAN → Blackboard Connection

### Option 1: Automated Test Suite (Recommended)

```bash
./scripts/hardware/run_can_tests.sh
```

Select option 1 for software testing.

### Option 2: Manual Testing

**Terminal 1** - Start hardware_interface:
```bash
./scripts/hardware/start_hardware_interface.sh
```

**Terminal 2** - Monitor messages:
```bash
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/hardware/can_testing_dashboard.py
```

## What You Should See

The dashboard will show:
- Battery: 24.0V, 85% (mock data)
- Velocity: 0.0 m/s
- Message rate: 10 Hz
- Blackboard: AVAILABLE

In the hardware_interface terminal, look for:
```
[INFO] [hardware_interface]: Hardware Interface Node initialized
[INFO] [hardware_interface]: Blackboard client initialized - direct CAN->blackboard writes enabled
```

## Build Issues

If colcon can't find packages:

```bash
# Use explicit base paths
colcon build \
  --base-paths src/autonomy/autonomy_core src/autonomy/interfaces/autonomy_interfaces \
  --symlink-install
```

If you get import errors:

```bash
# Clean and rebuild
rm -rf build/ install/ log/
colcon build --base-paths src/autonomy/autonomy_core src/autonomy/interfaces/autonomy_interfaces --symlink-install
source install/setup.bash
```

## Testing Checklist

- [ ] ROS2 packages built successfully
- [ ] `ros2 pkg list` shows autonomy_core and autonomy_interfaces
- [ ] hardware_interface starts without errors
- [ ] Dashboard shows messages at 10 Hz
- [ ] "Blackboard client initialized" appears in logs
- [ ] No import errors

## Next Steps

Once software testing works:
1. Test with real hardware (tomorrow)
2. Verify blackboard writes in logs
3. Test velocity commands
4. Test emergency stop
5. Full autonomy integration

---

**Quick command**: `./scripts/hardware/run_can_tests.sh` (select option 1)
