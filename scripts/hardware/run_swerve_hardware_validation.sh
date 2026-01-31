#!/usr/bin/env bash
# End-to-end swerve drive validation: terminal dashboard, simulator, CAN hardware tests.
# Run from workspace root. Optional: pass device for hardware test (e.g. /dev/ttyACM0).

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$SCRIPT_DIR/../.." && pwd)"
DEVICE="${1:-}"
cd "$WORKSPACE_ROOT"

DASH_PID=""
SIM_PID=""
cleanup() {
    if [[ -n "$DASH_PID" ]] && kill -0 "$DASH_PID" 2>/dev/null; then
        kill "$DASH_PID" 2>/dev/null || true
    fi
    if [[ -n "$SIM_PID" ]] && kill -0 "$SIM_PID" 2>/dev/null; then
        kill "$SIM_PID" 2>/dev/null || true
    fi
}
trap cleanup EXIT

echo "=== Swerve Drive End-to-End Validation ==="
echo "Workspace: $WORKSPACE_ROOT"
echo ""

# Start terminal dashboard in background (stdin from /dev/null so it doesn't block)
echo "Starting terminal dashboard..."
python3 terminal_dashboard.py </dev/null &>/tmp/urc_dashboard.log &
DASH_PID=$!
sleep 1
if ! kill -0 "$DASH_PID" 2>/dev/null; then
    echo "  [WARN] Dashboard exited; continuing without it"
    DASH_PID=""
else
    echo "  [OK] Dashboard PID $DASH_PID"
fi

# Start swerve simulator in background
echo "Starting swerve simulator..."
python3 swerve_simulator.py </dev/null &>/tmp/urc_simulator.log &
SIM_PID=$!
sleep 1
if ! kill -0 "$SIM_PID" 2>/dev/null; then
    echo "  [WARN] Simulator exited; continuing without it"
    SIM_PID=""
else
    echo "  [OK] Simulator PID $SIM_PID"
fi

# Run CAN hardware validation (protocol always; hardware if device given)
echo ""
if [[ -n "$DEVICE" ]]; then
    echo "Running validation with device: $DEVICE"
    python3 test_hardware_validation.py --device "$DEVICE"
else
    echo "Running protocol validation only (no --device). Pass /dev/ttyACM0 for hardware test."
    python3 test_hardware_validation.py
fi
RESULT=$?

echo ""
echo "=== Validation script complete (exit $RESULT) ==="
exit $RESULT
