# CAN Testing Suite - Complete Guide

**Test CAN → hardware_interface → blackboard communication**

Software testing first, then hardware validation.

---

## Quick Start (Software Testing - No Hardware)

Run the complete testing suite:

```bash
./scripts/hardware/run_can_tests.sh
```

Select option **1** for software testing.

This will:
1. Start `hardware_interface_node` with mock CAN data
2. Launch unified dashboard showing real-time messages
3. Display CAN data flow and blackboard status

**Expected Results**:
- Battery: 24.0V, 85%
- Velocity: 0.0 m/s (mock data)
- Message rate: ~10 Hz
- Status: "Blackboard client initialized"

---

## What You'll See

```
╔════════════════════════════════════════════════════════════════╗
║         URC 2026 - Unified CAN Testing Dashboard               ║
╚════════════════════════════════════════════════════════════════╝

  💻 SIMULATION MODE
  Runtime: 0m 15s

┌─ Connection Status ──────────────────────────────────────────┐
│  Hardware:   🔴 MOCK/DISCONNECTED                            │
│  Blackboard: 🟢 AVAILABLE                                    │
└──────────────────────────────────────────────────────────────┘

┌─ Message Statistics ─────────────────────────────────────────┐
│  Total Messages: 150                                         │
│                                                              │
│  battery        : 15       messages  (10.0 Hz)              │
│  velocity       : 45       messages  (10.0 Hz)              │
│  temperatures   : 15       messages  (10.0 Hz)              │
│  status         : 75       messages  (10.0 Hz)              │
└──────────────────────────────────────────────────────────────┘

┌─ Latest CAN Data ────────────────────────────────────────────┐
│  🔋 Battery:
│     Voltage:     24.00 V
│     Current:     -5.00 A
│     Percentage:  85.0 %
│
│  🏎️  Chassis Velocity:
│     Linear X:    0.000 m/s
│     Linear Y:    0.000 m/s
│     Angular Z:   0.000 rad/s
│
│  🌡️  Motor Temperatures:
│     M0: 25.0°C  M1: 25.0°C  M2: 25.0°C  M3: 25.0°C
└──────────────────────────────────────────────────────────────┘

✅ System is receiving CAN messages
✅ Blackboard writes are active (Option 1 working!)
```

---

## Hardware Testing (Real CAN Device)

### Prerequisites

1. **STM32 CAN device** connected via USB
2. **Device permissions** configured

### Setup (One-time)

```bash
# Configure device access
./scripts/hardware/setup_usbcan_pi5.sh

# Verify device appears
ls -l /dev/ttyACM*
```

### Run Hardware Test

```bash
./scripts/hardware/run_can_tests.sh
```

Select option **2** for hardware testing.

You'll be prompted to select the CAN device (default: `/dev/ttyACM0`).

**Expected Results**:
- Hardware: 🟢 CONNECTED
- Real battery voltage/current from CAN
- Real motor velocities and temperatures
- Message rate: 10-50 Hz (depends on firmware)

---

## Manual Testing (Advanced)

### Start Components Separately

**Terminal 1** - Hardware Interface (Simulation):
```bash
source /opt/ros/humble/setup.bash  # or jazzy
source install/setup.bash
ros2 run autonomy_core hardware_interface
```

**Terminal 2** - Dashboard:
```bash
python3 scripts/hardware/can_testing_dashboard.py
```

**Terminal 1** - Hardware Interface (Real Hardware):
```bash
ros2 run autonomy_core hardware_interface \
  --ros-args \
  -p can_port:=/dev/ttyACM0 \
  -p can_baudrate:=115200
```

**Terminal 2** - Dashboard (Hardware Mode):
```bash
python3 scripts/hardware/can_testing_dashboard.py --hardware
```

### Monitor Only (hardware_interface already running)

```bash
./scripts/hardware/run_can_tests.sh
```

Select option **3**.

---

## Available Testing Tools

### 1. Complete Testing Suite (Recommended)
```bash
./scripts/hardware/run_can_tests.sh
```
Interactive menu for software/hardware testing.

### 2. Unified Dashboard
```bash
python3 scripts/hardware/can_testing_dashboard.py          # Simulation mode
python3 scripts/hardware/can_testing_dashboard.py --hardware  # Hardware mode
```
Real-time CAN message monitoring.

### 3. Validation Scripts
```bash
python3 scripts/hardware/validate_can_blackboard_direct.py  # Code validation
python3 scripts/hardware/test_option1_e2e.py                 # Integration test
```
Verify implementation without ROS2 build.

### 4. Blackboard Visualizer
```bash
python3 scripts/hardware/blackboard_visualizer.py
```
Monitor blackboard key-value store (requires blackboard service).

### 5. Hardware Setup
```bash
./scripts/hardware/setup_usbcan_pi5.sh          # Configure device access
./scripts/hardware/flash_control_systems.sh     # Flash STM32 firmware
```

---

## Troubleshooting

### No Messages Received

**Check if hardware_interface is running**:
```bash
ros2 node list | grep hardware_interface
```

**Start it manually**:
```bash
ros2 run autonomy_core hardware_interface
```

**Check logs**:
```bash
tail -f /tmp/hw_interface.log
```

### CAN Device Not Found

**List devices**:
```bash
ls -l /dev/ttyACM* /dev/ttyUSB*
```

**Check permissions**:
```bash
groups | grep dialout
```

If not in dialout group:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in
```

### ROS2 Build Errors

**Clean and rebuild**:
```bash
rm -rf build/ install/ log/
colcon build --packages-select autonomy_interfaces autonomy_core
source install/setup.bash
```

### Low Message Rate

**Check telemetry_rate_hz parameter**:
```bash
ros2 run autonomy_core hardware_interface \
  --ros-args \
  -p telemetry_rate_hz:=50.0
```

**Monitor specific topic**:
```bash
ros2 topic hz /hardware/battery_state
```

---

## Understanding the Data Flow

```
┌─────────────────┐
│  CAN Hardware   │  (STM32 via /dev/ttyACM0)
│  (or Mock)      │
└────────┬────────┘
         │ SLCAN Protocol
         │
         ▼
┌─────────────────────────┐
│  hardware_interface_   │
│  node                   │
│                         │
│  Read CAN:              │
│  - Battery state        │
│  - Velocity feedback    │
│  - Motor temps          │
│                         │
│  Write to:              │
│  ├─> ROS2 topics ──────┼──> Dashboard sees this
│  │   (/hardware/*)     │
│  │                     │
│  └─> Blackboard ───────┼──> BT.CPP reads this
│      (direct write)    │    (<1ms latency)
└─────────────────────────┘
```

**Key Points**:
- Dashboard subscribes to `/hardware/*` topics
- Blackboard gets direct writes (Option 1)
- Zero ROS2 hops for blackboard updates
- Both paths updated simultaneously

---

## Files Overview

### Essential Testing Files
- `run_can_tests.sh` - Main testing suite (start here)
- `can_testing_dashboard.py` - Unified real-time dashboard

### Validation Tools
- `validate_can_blackboard_direct.py` - Code inspection validator
- `test_option1_e2e.py` - Integration test
- `blackboard_visualizer.py` - Blackboard monitor

### Hardware Tools
- `setup_usbcan_pi5.sh` - Device setup (one-time)
- `flash_control_systems.sh` - STM32 firmware flash

### Support Files
- `can_to_blackboard_bridge.py` - Topic remapping bridge (not needed for Option 1)
- `run_swerve_hardware_validation.sh` - Swerve-specific testing

---

## Success Criteria

### Software Test (Simulation)
- ✅ Dashboard shows 10+ Hz message rate
- ✅ Battery voltage = 24.0V, 85%
- ✅ Velocity values present (0.0 m/s mock)
- ✅ "Blackboard client initialized" in logs
- ✅ No errors in dashboard

### Hardware Test (Real CAN)
- ✅ Hardware: 🟢 CONNECTED
- ✅ Real voltage/current from CAN bus
- ✅ Motor feedback (velocity, temperature)
- ✅ Message rate matches firmware (10-50 Hz)
- ✅ No serial errors or timeouts

---

## Next Steps After Testing

1. **Verify Blackboard Writes**
   - Check logs for "blackboard.set()" calls
   - Run `blackboard_visualizer.py` to see values

2. **Test Command Sending**
   ```bash
   ros2 topic pub /cmd_vel geometry_msgs/Twist \
     "{linear: {x: 0.5}, angular: {z: 0.1}}" --once
   ```

3. **Test Emergency Stop**
   ```bash
   ros2 topic pub /emergency_stop std_msgs/Bool "data: true" --once
   ```

4. **Integration with Autonomy**
   - Verify behavior trees can read blackboard
   - Test navigation commands
   - Validate safety systems

---

## Quick Command Reference

```bash
# Complete test suite (recommended)
./scripts/hardware/run_can_tests.sh

# Software test only
./scripts/hardware/run_can_tests.sh  # Select option 1

# Hardware test only  
./scripts/hardware/run_can_tests.sh  # Select option 2

# Monitor existing node
./scripts/hardware/run_can_tests.sh  # Select option 3

# Direct dashboard
python3 scripts/hardware/can_testing_dashboard.py

# Validate code
python3 scripts/hardware/validate_can_blackboard_direct.py

# Check device
ls -l /dev/ttyACM*

# View logs
tail -f /tmp/hw_interface.log

# ROS2 topics
ros2 topic list | grep hardware
ros2 topic hz /hardware/battery_state
ros2 topic echo /hardware/chassis_velocity
```

---

**Ready to test!** Start with `./scripts/hardware/run_can_tests.sh`
