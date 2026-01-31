# CAN Testing on Raspberry Pi 5

Set up and run CAN-to-blackboard testing on a Pi 5 using only the main repo. No submodule updates or vendor changes required.

---

## 1. Pi 5 and OS

- **Hardware**: Raspberry Pi 5 (4GB+ RAM recommended).
- **OS**: Raspberry Pi OS (64-bit), e.g. Bookworm.
- **USB-CAN**: STM32 or other SLCAN-compatible adapter connected via USB.

---

## 2. Install ROS2 (on the Pi)

Pick the distro that matches your OS:

**Raspberry Pi OS Bookworm (Debian 12):**

```bash
# Jazzy (ROS2 10) – recommended on Bookworm
sudo apt update
sudo apt install -y ros-jazzy-ros-base python3-colcon-common-extensions

# Or Humble (ROS2 8) if you use the Humble repo for Debian 12
# sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions
```

**Raspberry Pi OS Bullseye (Debian 11):**

```bash
sudo apt update
sudo apt install -y ros-humble-ros-base python3-colcon-common-extensions
```

Add to `~/.bashrc` (use your distro: `jazzy` or `humble`):

```bash
source /opt/ros/jazzy/setup.bash
```

Then: `source ~/.bashrc` or open a new terminal.

---

## 3. Clone Repo (no submodule changes)

Clone without initializing or updating submodules so vendor/ is not touched:

```bash
cd ~
git clone https://github.com/your-org/urc-machiato-2026.git
cd urc-machiato-2026
# Do not run: git submodule update --init
```

If the repo was already cloned with submodules, leave them as-is; do not run `git submodule update` for this setup.

---

## 4. Python Dependencies

From the repo root:

```bash
cd ~/urc-machiato-2026
pip install --user -r requirements.txt
pip install --user pyserial
```

(Use a venv if you prefer; then activate it before the steps below.)

---

## 5. One-Time CAN Device Setup

Run the Pi 5 USB-CAN setup script (no submodules involved):

```bash
cd ~/urc-machiato-2026
./scripts/hardware/setup_usbcan_pi5.sh
```

- Adds your user to the `dialout` group (you must **log out and log back in** for it to apply).
- Creates udev rules for `/dev/ttyACM*`.

After re-login, plug in the USB-CAN adapter and check:

```bash
ls -l /dev/ttyACM*
groups | grep dialout
```

Use `/dev/ttyUSB0` instead if that is what appears.

---

## 6. Build ROS2 Packages (main repo only)

Build only the autonomy packages used for CAN testing (no vendor/submodule build):

```bash
cd ~/urc-machiato-2026
source /opt/ros/jazzy/setup.bash   # or humble

colcon build \
  --base-paths src/autonomy/autonomy_core src/autonomy/interfaces/autonomy_interfaces \
  --symlink-install

source install/setup.bash
```

Check:

```bash
ros2 pkg list | grep autonomy
# Should show: autonomy_core, autonomy_interfaces
```

---

## 7. Run CAN Test (3 terminals)

All commands from repo root. Use your device if different (e.g. `/dev/ttyUSB0`).

**Terminal 1 – Mock blackboard**

```bash
cd ~/urc-machiato-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/hardware/mock_blackboard_service.py
```

Leave running. You should see `Mock blackboard service started` and later `SET battery_level = ...` when the hardware interface writes.

**Terminal 2 – Hardware interface (real CAN)**

```bash
cd ~/urc-machiato-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 run autonomy_core hardware_interface \
  --ros-args -p can_port:=/dev/ttyACM0 -p can_baudrate:=115200
```

Leave running. Look for `Connected to CAN serial on /dev/ttyACM0`.

**Terminal 3 – Dashboard**

```bash
cd ~/urc-machiato-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
python3 scripts/hardware/can_testing_dashboard.py --hardware
```

You should see live CAN data and **Blackboard: AVAILABLE**.

---

## 8. Optional: Software-Only Test (no CAN device)

To confirm the stack works without hardware:

```bash
cd ~/urc-machiato-2026
source /opt/ros/jazzy/setup.bash
source install/setup.bash
./scripts/hardware/run_can_tests.sh
```

Choose option **1** (Software Test). No USB-CAN or submodules needed.

---

## 9. Flashing STM32 Firmware (optional; uses submodule)

Flashing the control board uses `vendor/control-systems` and therefore submodule content. If you want to avoid submodules, skip flashing and use the board with its existing firmware.

If you later decide to flash:

```bash
git submodule update --init vendor/control-systems
./scripts/hardware/flash_control_systems.sh
```

---

## 10. Quick Checklist (no submodules)

- [ ] Pi 5 with Raspberry Pi OS 64-bit
- [ ] ROS2 (Humble or Jazzy) installed and sourced
- [ ] Repo cloned; **no** `git submodule update`
- [ ] `pip install -r requirements.txt` and `pyserial`
- [ ] `./scripts/hardware/setup_usbcan_pi5.sh` run; re-login; device visible at `/dev/ttyACM*` or `/dev/ttyUSB*`
- [ ] `colcon build --base-paths src/autonomy/autonomy_core src/autonomy/interfaces/autonomy_interfaces --symlink-install` and `source install/setup.bash`
- [ ] Three terminals: mock blackboard, `ros2 run autonomy_core hardware_interface --ros-args -p can_port:=/dev/ttyACM0 -p can_baudrate:=115200`, dashboard with `--hardware`

---

## Troubleshooting

- **Permission denied on /dev/ttyACM0**: Ensure you are in group `dialout` and have logged out and back in after running `setup_usbcan_pi5.sh`.
- **No such executable 'hardware_interface'**: Run `source install/setup.bash` and ensure `colcon build` completed for `autonomy_core`.
- **Blackboard: NOT DETECTED**: Start the mock blackboard (Terminal 1) before starting the dashboard.
- **Import errors (autonomy_interfaces)**: Build the interfaces package:  
  `colcon build --packages-select autonomy_interfaces --symlink-install`
