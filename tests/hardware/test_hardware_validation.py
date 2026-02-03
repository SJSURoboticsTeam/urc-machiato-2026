#!/usr/bin/env python3
"""
CAN Hardware Validation for URC 2026 Swerve Drive

Validates CAN protocol encoding and optionally tests real hardware.
Run from workspace root.

Usage:
  python tests/hardware/test_hardware_validation.py                    # Protocol tests only
  python tests/hardware/test_hardware_validation.py --device /dev/ttyACM0   # With hardware
"""

import argparse
import sys
import time
from pathlib import Path

# Add src to path for can_bridge (script lives in tests/hardware/)
WORKSPACE_ROOT = Path(__file__).resolve().parent.parent.parent
sys.path.insert(0, str(WORKSPACE_ROOT / "src" / "infrastructure" / "bridges"))


def run_protocol_tests() -> bool:
    """Run CAN message encoding/decoding tests (no hardware)."""
    print("=== CAN Protocol Validation (no hardware) ===")
    try:
        from can_bridge import (
            encode_swerve_motor_commands,
            _encode_can_message_2byte,
            SWERVE_DRIVE_IDS,
            SWERVE_STEER_IDS,
            LINEAR_SCALE,
            ANGULAR_SCALE,
        )
    except ImportError as e:
        print(f"  [FAIL] Import error: {e}")
        return False

    # Test 8-motor swerve encoding
    motor_states = [
        {"velocity": 0.5},
        {"velocity": -0.3},
        {"velocity": 0.0},
        {"velocity": 0.2},
        {"angle": 0.0},
        {"angle": 1.57},
        {"angle": -0.78},
        {"angle": 0.0},
    ]
    frames = encode_swerve_motor_commands(motor_states)
    if len(frames) != 8:
        print(f"  [FAIL] Expected 8 SLCAN frames, got {len(frames)}")
        return False
    for i, frame in enumerate(frames):
        s = frame.decode("ascii")
        if not s.startswith("t") or "\r" not in s or len(s) < 6:
            print(f"  [FAIL] Frame {i} invalid format: {s!r}")
            return False
    print("  [OK] 8-motor swerve encoding: 8 frames generated")

    # Test chassis velocity scaling (from test_submodule_interfaces)
    x_vel, y_vel, rot_vel = 0.5, 0.0, 15.0  # m/s, m/s, deg/s
    x_scaled = int(x_vel * LINEAR_SCALE)
    y_scaled = int(y_vel * LINEAR_SCALE)
    rot_deg = rot_vel  # already deg/s in teleop
    rot_scaled = int(rot_deg * (ANGULAR_SCALE))
    if x_scaled != 2048 or abs(rot_scaled - 960) > 1:
        print(f"  [FAIL] Velocity scaling: x_scaled={x_scaled} rot_scaled={rot_scaled}")
        return False
    print("  [OK] Chassis velocity scaling (0x00C)")

    print("  [OK] All protocol tests passed")
    return True


def run_hardware_test(
    device: str, baudrate: int = 115200, timeout_s: float = 2.0
) -> bool:
    """Send SLCAN frames to device and check for no errors (optional reply)."""
    print(f"=== CAN Hardware Validation: {device} ===")
    try:
        import serial
    except ImportError:
        print("  [FAIL] pyserial not installed: pip install pyserial")
        return False

    try:
        port = serial.Serial(
            port=device, baudrate=baudrate, timeout=0.5, write_timeout=1.0
        )
    except (serial.SerialException, OSError) as e:
        print(f"  [FAIL] Cannot open {device}: {e}")
        return False

    try:
        # Send heartbeat (0x00E, DLC 0)
        port.write(b"t00E0\r")
        port.flush()
        start = time.monotonic()
        reply = b""
        while (time.monotonic() - start) < timeout_s:
            if port.in_waiting:
                reply += port.read(port.in_waiting)
                if b"\r" in reply:
                    break
            time.sleep(0.05)
        if reply:
            print(f"  [OK] Heartbeat sent; received {len(reply)} bytes")
        else:
            print(
                "  [OK] Heartbeat sent (no reply within timeout - device may not echo)"
            )

        # Send single chassis velocity (0x00C) zero command
        port.write(b"t00C6000000000000\r")
        port.flush()
        print("  [OK] Chassis velocity (zero) sent")

        # Send one swerve drive motor command (0x200)
        port.write(b"t20020800\r")  # velocity scaled 2048 = 0.5 m/s
        port.flush()
        print("  [OK] Swerve drive motor command (0x200) sent")

    except Exception as e:
        print(f"  [FAIL] Hardware test error: {e}")
        return False
    finally:
        port.close()

    print("  [OK] Hardware validation complete")
    return True


def main() -> int:
    parser = argparse.ArgumentParser(
        description="CAN hardware validation for swerve drive"
    )
    parser.add_argument(
        "--device", type=str, default="", help="Serial device e.g. /dev/ttyACM0"
    )
    parser.add_argument("--baudrate", type=int, default=115200, help="Serial baud rate")
    args = parser.parse_args()

    ok = run_protocol_tests()
    if not ok:
        return 1

    if args.device:
        ok = run_hardware_test(args.device, args.baudrate)
        if not ok:
            return 1

    print("\n=== Validation complete ===")
    return 0


if __name__ == "__main__":
    sys.exit(main())
