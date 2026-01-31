# Hardware CAN Testing Guide

Quick guide for testing CAN communication with real hardware.

## Test in Real ROS2 Environment (Simulation)

This tests the full ROS2 stack with mock CAN data:

```bash
# Automated test (recommended)
./scripts/hardware/test_ros2_environment.sh
```

This will:
1. Check ROS2 is sourced
2. Build packages if needed
3. Launch hardware_interface_node (mock CAN)
4. Monitor CAN messages for 10 seconds
5. Show results and options

### Manual Testing

```bash
# 1. Source ROS2
source /opt/ros/humble/setup.bash  # or jazzy
source install/setup.bash

# 2. Terminal 1 - Hardware interface
ros2 run autonomy_core hardware_interface

# 3. Terminal 2 - Monitor messages
python3 scripts/hardware/can_message_monitor.py

# 4. Terminal 3 - Check topics
ros2 topic list | grep hardware
ros2 topic echo /hardware/battery_state
```

Expected output in monitor:
- Battery: 24.0V, 85%
- Velocity: 0.0 m/s
- Messages updating at 10 Hz

---

## Test with Real Hardware

### Prerequisites

1. STM32 CAN device connected to `/dev/ttyACM0`
2. USBcan Pi 5 setup completed

### Setup (One-time)

```bash
# Configure device permissions
./scripts/hardware/setup_usbcan_pi5.sh

# Verify device
ls -l /dev/ttyACM*
# Should show: crw-rw---- 1 root dialout ...

# Test user is in dialout group
groups | grep dialout
# If not, log out and back in after running setup script
```

### Flash STM32 Firmware (If needed)

```bash
# Build and flash control-systems firmware
./scripts/hardware/flash_control_systems.sh
```

### Run Hardware Test

```bash
# 1. Source ROS2
source /opt/ros/humble/setup.bash  # or jazzy
source install/setup.bash

# 2. Terminal 1 - Hardware interface with real CAN
ros2 run autonomy_core hardware_interface \
  --ros-args \
  -p can_port:=/dev/ttyACM0 \
  -p can_baudrate:=115200

# 3. Terminal 2 - Monitor CAN messages
python3 scripts/hardware/can_message_monitor.py

# 4. Terminal 3 - Send test commands
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.1}}" --once
```

### What to Verify

Monitor should show:
- Battery state from real CAN (actual voltage/percentage)
- Chassis velocity feedback from motors
- Motor temperatures (if available)
- System status updates

Hardware interface logs should show:
- "Connected to CAN serial on /dev/ttyACM0"
- "Blackboard client initialized"
- No serial errors or timeouts

---

## Troubleshooting

### "No messages received"

```bash
# Check if hardware_interface is running
ros2 node list | grep hardware_interface

# Check logs
ros2 run autonomy_core hardware_interface 2>&1 | tee /tmp/hw.log

# Look for:
# - "Hardware Interface Node initialized"
# - "Connected to CAN serial" or "Using mock CAN interface"
# - "Blackboard client initialized"
```

### "CAN serial connection failed"

```bash
# Verify device exists
ls -l /dev/ttyACM*

# Check permissions
groups | grep dialout

# Try with sudo (temporary test only)
sudo ros2 run autonomy_core hardware_interface --ros-args -p can_port:=/dev/ttyACM0

# If sudo works, fix permissions:
sudo usermod -a -G dialout $USER
# Then log out and back in
```

### "Blackboard client not available"

```bash
# Check if blackboard service is running
ros2 service list | grep blackboard

# If not found, the hardware_interface will still publish to topics
# but won't write directly to blackboard

# To test blackboard writes, start a BT.CPP node with blackboard service
```

### STM32 not responding

```bash
# Check device
ls /dev/ttyACM* /dev/ttyUSB*

# Test serial communication
screen /dev/ttyACM0 115200
# Or
minicom -D /dev/ttyACM0 -b 115200

# If no devices, check USB connection:
dmesg | grep tty
lsusb
```

---

## Hardware Test Parameters

You can customize hardware_interface parameters:

```bash
ros2 run autonomy_core hardware_interface \
  --ros-args \
  -p can_port:=/dev/ttyACM0 \        # CAN device path
  -p can_baudrate:=115200 \          # Serial baud rate
  -p control_rate_hz:=50.0 \         # Control loop frequency
  -p telemetry_rate_hz:=10.0         # Sensor data frequency
```

---

## Expected Data Flow

```
STM32 CAN Device
  |
  | /dev/ttyACM0 (SLCAN protocol)
  |
  v
hardware_interface_node
  |
  ├─> Publish to /hardware/* topics
  |   (for other ROS2 nodes)
  |
  └─> blackboard.set(key, value)
      (direct write, <1ms)
```

Monitor shows messages from `/hardware/*` topics.
Blackboard gets direct writes (verified in logs).

---

## Quick Reference

### Start Hardware Interface (Simulation)
```bash
ros2 run autonomy_core hardware_interface
```

### Start Hardware Interface (Real CAN)
```bash
ros2 run autonomy_core hardware_interface --ros-args -p can_port:=/dev/ttyACM0
```

### Monitor CAN Messages
```bash
python3 scripts/hardware/can_message_monitor.py
```

### Check ROS2 Topics
```bash
ros2 topic list | grep hardware
ros2 topic hz /hardware/battery_state
ros2 topic echo /hardware/chassis_velocity
```

### Check Blackboard (if service available)
```bash
python3 scripts/hardware/blackboard_visualizer.py
```

---

## Next Steps After Hardware Test

Once hardware CAN communication is working:

1. Verify blackboard writes in logs
2. Test sending velocity commands via `/cmd_vel`
3. Monitor feedback from hardware
4. Test emergency stop
5. Integrate with full autonomy stack
