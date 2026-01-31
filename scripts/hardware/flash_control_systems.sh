#!/usr/bin/env bash
# Flash STM32 control-systems (drive) firmware for URC 2026 Swerve Drive
# Run from workspace root. Requires: conan, cmake, arm-none-eabi-gcc, stm32flash
# Phase 2 of hardware integration.

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
CONTROL_SYSTEMS="${WORKSPACE_ROOT}/vendor/control-systems/drive"
BUILD_DIR="${CONTROL_SYSTEMS}/build"
DEVICE="${1:-/dev/ttyACM0}"
FLASH_ONLY="${FLASH_ONLY:-0}"

echo "=== STM32 Control-Systems Flash ==="
echo "Workspace: $WORKSPACE_ROOT"
echo "Drive package: $CONTROL_SYSTEMS"
echo "Device: $DEVICE"
echo ""

if [[ ! -d "$CONTROL_SYSTEMS" ]]; then
    echo "Error: vendor/control-systems/drive not found. Update submodules: git submodule update --init vendor/control-systems"
    exit 1
fi

cd "$CONTROL_SYSTEMS"

if [[ "$FLASH_ONLY" != "1" ]]; then
    echo "Step 1: Conan install"
    if command -v conan &>/dev/null; then
        conan install . --output-folder=build --build=missing || true
    else
        echo "  [WARN] conan not found. Skip conan install. Build may fail."
    fi
    echo ""

    echo "Step 2: CMake configure"
    mkdir -p "$BUILD_DIR"
    cd "$BUILD_DIR"
    if cmake .. 2>/dev/null; then
        echo "  [OK] CMake configured"
    else
        echo "  [WARN] CMake configure failed (may need conan or preset). Try building from vendor/control-systems per their README."
    fi
    echo ""

    echo "Step 3: Build"
    if cmake --build . 2>/dev/null; then
        echo "  [OK] Build complete"
    else
        echo "  [WARN] Build failed. Ensure arm-none-eabi-gcc and libhal are installed. Flash step will use existing binary if present."
    fi
    cd "$CONTROL_SYSTEMS"
else
    echo "FLASH_ONLY=1: skipping build"
fi

# Locate firmware binary (common names/locations)
FIRMWARE=""
for candidate in \
    "$BUILD_DIR/application.bin" \
    "$BUILD_DIR/drive.bin" \
    "$BUILD_DIR/bin/application.bin" \
    "$BUILD_DIR/application" \
    "$CONTROL_SYSTEMS/build/application.bin"; do
    if [[ -f "$candidate" ]]; then
        FIRMWARE="$candidate"
        break
    fi
done

echo "Step 4: Flash firmware"
if [[ -z "$FIRMWARE" ]]; then
    echo "  [WARN] No firmware binary found in $BUILD_DIR."
    echo "  Build the drive package per vendor/control-systems README, then run this script with FLASH_ONLY=1."
    echo "  Example: stm32flash -w build/application.bin -v -g 0x0 $DEVICE"
    exit 0
fi

if ! command -v stm32flash &>/dev/null; then
    echo "  [WARN] stm32flash not found. Install with: sudo apt install stm32flash"
    echo "  Manual flash: stm32flash -w $FIRMWARE -v -g 0x0 $DEVICE"
    exit 0
fi

if [[ ! -e "$DEVICE" ]]; then
    echo "  [WARN] Device $DEVICE not found. Connect STM32 and run: ls -l /dev/ttyACM*"
    echo "  Then: $0 $DEVICE"
    exit 1
fi

echo "  Flashing $FIRMWARE to $DEVICE"
if stm32flash -w "$FIRMWARE" -v -g 0x0 "$DEVICE"; then
    echo "  [OK] Flash complete. Device will reset."
else
    echo "  [FAIL] Flash failed. Check device, boot pins, and permissions (dialout group)."
    exit 1
fi
