#!/usr/bin/env bash
# USBcan Pi 5 Configuration for URC 2026 Swerve Drive
# Run from workspace root. Some steps require sudo.
# Phase 1 of hardware integration - run before connecting STM32.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
UDEV_RULES="/etc/udev/rules.d/99-usbcans.rules"

echo "=== USBcan Pi 5 Configuration ==="
echo "Workspace: $WORKSPACE_ROOT"
echo ""

# Step 1: Add user to dialout (requires manual logout/login to take effect)
echo "Step 1: Add user to dialout group for serial access"
if groups | grep -q dialout; then
    echo "  [OK] User already in dialout group"
else
    echo "  Run this command (requires sudo), then LOG OUT and log back in:"
    echo "    sudo usermod -a -G dialout \$USER"
    echo ""
    read -p "  Run it now? [y/N] " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo usermod -a -G dialout "$USER"
        echo "  [OK] Added to dialout. You must log out and back in for it to take effect."
    fi
fi
echo ""

# Step 2: Create udev rules for USBcan devices
echo "Step 2: Create udev rules for ttyACM devices"
if [[ -f "$UDEV_RULES" ]]; then
    echo "  [OK] Rules file already exists: $UDEV_RULES"
    cat "$UDEV_RULES"
else
    echo 'KERNEL=="ttyACM*" SUBSYSTEM=="tty" GROUP="dialout", MODE="0660"' | sudo tee "$UDEV_RULES"
    echo "  [OK] Created $UDEV_RULES"
fi
echo ""

# Step 3: Reload udev
echo "Step 3: Reload udev rules"
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "  [OK] Udev rules reloaded"
echo ""

# Step 4: Verify Python serial library
echo "Step 4: Verify Python serial library"
if python3 -c "import serial; print('  [OK] pyserial available')" 2>/dev/null; then
    :
else
    echo "  [WARN] pyserial not found. Install with: pip install pyserial"
fi
echo ""

# Step 5: Check for devices (may be none if STM32 not connected)
echo "Step 5: Check for serial devices"
if ls /dev/ttyACM* 2>/dev/null; then
    echo "  [OK] Found ttyACM device(s)"
    ls -l /dev/ttyACM*
else
    echo "  [INFO] No /dev/ttyACM* devices found. Connect STM32/USBcan and run again."
fi
echo ""

echo "=== Configuration complete ==="
echo "Verification checklist:"
echo "  - Run 'groups' and confirm 'dialout' is listed (after re-login)"
echo "  - Connect STM32 and run: ls -l /dev/ttyACM*"
echo "  - Test serial: python3 -c \"import serial; s=serial.Serial('/dev/ttyACM0',115200); s.close(); print('OK')\""
echo ""
