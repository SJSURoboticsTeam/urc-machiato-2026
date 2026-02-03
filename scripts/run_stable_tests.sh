#!/usr/bin/env bash
# Run stable subset of unit tests (no ROS2 required).
# Uses same ignores as check_quality.sh. Exit 0 if all pass, 1 otherwise.

set -e

cd "$(dirname "$0")/.."

echo "Running stable unit tests (no ROS2)..."

python3 -m pytest tests/unit/ -v --tb=short \
    --ignore=tests/unit/simulation/test_full_stack_simulator_unit.py \
    --ignore=tests/unit/infrastructure/test_slcan_protocol_simulator.py \
    --ignore=tests/unit/infrastructure/test_stm32_firmware_simulator.py
