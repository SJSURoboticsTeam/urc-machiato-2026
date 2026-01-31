# How to Test CAN to Blackboard on Hardware

Steps to test with real CAN hardware after simulation passes.

---

## 1. One-Time Setup

### 1.1 Device Permissions

```bash
cd /home/durian/urc-machiato-2026
./scripts/hardware/setup_usbcan_pi5.sh
```

- Adds your user to the `dialout` group (you must **log out and back in** for this to apply).
- Creates udev rules so `/dev/ttyACM*` is readable without sudo.

### 1.2 Verify Device

After plugging in the STM32 CAN adapter (e.g. USB-CAN):

```bash
ls -l /dev/ttyACM*
# Expect: crw-rw---- 1 root dialout ... /dev/ttyACM0

groups | grep dialout
# Should list dialout (if not, log out and back in)
```

If you see `/dev/ttyUSB0` instead, use that as the port below.

### 1.3 (Optional) Flash STM32 Firmware

If the control board needs the latest firmware:

```bash
./scripts/hardware/flash_control_systems.sh
```

---

## 2. Run Hardware Test with Blackboard

You need three pieces: **mock blackboard** (or real BT), **hardware_interface** on real CAN, and the **dashboard**.

### 2.1 Source ROS2 and Workspace

```bash
cd /home/durian/urc-machiato-2026
source /opt/ros/jazzy/setup.bash   # or humble
source install/setup.bash
```

### 2.2 Terminal 1: Mock Blackboard Service

So the dashboard shows "Blackboard: AVAILABLE" and you can verify writes:

```bash
python3 scripts/hardware/mock_blackboard_service.py
```

Leave this running. You should see `[INFO] Mock blackboard service started` and then `SET battery_level = ...` etc. when hardware_interface writes.

### 2.3 Terminal 2: Hardware Interface (Real CAN)

Replace `/dev/ttyACM0` with your device if different (e.g. `/dev/ttyUSB0`).

**Option A – Use the full test script (starts mock + hw + dashboard):**

Edit `scripts/hardware/test_can_blackboard_full.sh` to pass the CAN port, or run hardware_interface manually with parameters.

**Option B – Run hardware_interface with real port:**

ROS2 parameters must be set before the node starts. Use a launch file or run with parameters:

```bash
# From workspace root, with install/setup.bash already sourced:
python3 -c "
import sys
sys.path.insert(0, 'src')
import os
os.environ['ROS_PARAMETER_OVERRIDES'] = 'can_port:=/dev/ttyACM0 can_baudrate:=115200'
from autonomy.autonomy_core.autonomy_core.control.hardware_interface_node import main
main()
"
```

Or create a small wrapper script that sets `can_port` (e.g. via a YAML parameter file or a launch file that passes the parameter).

**Option C – Parameter file:**

Create `config/hardware_can.yaml`:

```yaml
hardware_interface:
  ros__parameters:
    can_port: "/dev/ttyACM0"
    can_baudrate: 115200
```

Then run:

```bash
ros2 run autonomy_core hardware_interface --ros-args --params-file config/hardware_can.yaml
```

(Only works if `ros2 run autonomy_core hardware_interface` is available; otherwise use the Python invocation above with the parameter file loaded by your launch.)

### 2.4 Terminal 3: Dashboard

```bash
python3 scripts/hardware/can_testing_dashboard.py --hardware
```

You should see:

- **Hardware: CONNECTED** (once the CAN serial connects).
- **Blackboard: AVAILABLE** (with mock or real BT running).
- Real battery, velocity, and temperatures from CAN.

---

## 3. What to Verify

- **hardware_interface logs:** `Connected to CAN serial on /dev/ttyACM0`, `Blackboard client initialized`, no serial/timeout errors.
- **Mock blackboard logs:** `SET battery_level = ...`, `SET robot_velocity_x = ...`, etc., with values that match real hardware.
- **Dashboard:** Battery and velocity updating from hardware; Blackboard: AVAILABLE.

---

## 4. Quick Reference

| Step              | Command / action |
|-------------------|------------------|
| One-time setup   | `./scripts/hardware/setup_usbcan_pi5.sh` then log out/in |
| Check device     | `ls -l /dev/ttyACM*` |
| Mock blackboard  | `python3 scripts/hardware/mock_blackboard_service.py` |
| HW interface     | Run with `can_port:=/dev/ttyACM0` (script, launch, or params file) |
| Dashboard        | `python3 scripts/hardware/can_testing_dashboard.py --hardware` |
| Full sim test    | `./scripts/hardware/test_can_blackboard_full.sh` (simulation only; for hardware use the three terminals above) |

---

## 5. Troubleshooting

- **No `/dev/ttyACM*`:** Plug in USB-CAN; run `ls /dev/tty*`; if needed run setup script and log out/in.
- **Permission denied:** Ensure you are in `dialout`: `groups \| grep dialout`.
- **"No module named 'can_serial'":** Expected if teleoperation vendor code is not installed; node falls back to mock CAN and still publishes/blackboard-writes. For real CAN, fix the `can_serial` import path in `connect_can_serial()` (e.g. to your vendor path).
- **Blackboard: NOT DETECTED:** Start mock blackboard (or real BT) before or with the dashboard; wait a few seconds for the dashboard to detect the service.
