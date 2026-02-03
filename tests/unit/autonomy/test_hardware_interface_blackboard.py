"""
Unit tests for hardware_interface_node direct blackboard writes (Phase 1).

Asserts that when telemetry and emergency stop run, the blackboard client
receives set() calls with expected keys (battery_level, robot_velocity_x/y,
emergency_stop_active, safety_stop_active, system.emergency_stop).
"""

import sys
from pathlib import Path
from unittest.mock import MagicMock, patch

import pytest

# Ensure src is on path for core.* imports (file is tests/unit/autonomy/... so parents[3] = repo root)
PROJECT_ROOT = Path(__file__).resolve().parents[3]
SRC_ROOT = str(PROJECT_ROOT / "src")
if SRC_ROOT not in sys.path:
    sys.path.insert(0, SRC_ROOT)


@pytest.fixture
def mock_blackboard():
    """Mock UnifiedBlackboardClient; records set(key, value) calls."""
    m = MagicMock()
    m.set = MagicMock(return_value=True)
    return m


@pytest.fixture
def mock_blackboard_keys():
    """Provide BlackboardKeys-like attributes for assertions."""

    class Keys:
        BATTERY_LEVEL = "battery_level"
        System = type(
            "System",
            (),
            {
                "BATTERY_PERCENT": "system.battery_percent",
                "EMERGENCY_STOP": "system.emergency_stop",
            },
        )()
        ROBOT_VELOCITY_X = "robot_velocity_x"
        ROBOT_VELOCITY_Y = "robot_velocity_y"
        EMERGENCY_STOP_ACTIVE = "emergency_stop_active"
        SAFETY_STOP_ACTIVE = "safety_stop_active"

    return Keys()


@pytest.mark.unit
def test_hardware_interface_code_has_blackboard_writes():
    """Source-inspection: hardware_interface_node contains required blackboard usage."""
    hw_path = (
        PROJECT_ROOT
        / "src/autonomy/autonomy_core/autonomy_core/control/hardware_interface_node.py"
    )
    if not hw_path.exists():
        pytest.skip("hardware_interface_node.py not found")
    code = hw_path.read_text()
    assert "UnifiedBlackboardClient" in code
    assert "BlackboardKeys" in code
    assert "self.blackboard = UnifiedBlackboardClient" in code
    assert "BlackboardKeys.BATTERY_LEVEL" in code
    assert "BlackboardKeys.ROBOT_VELOCITY_X" in code
    assert "BlackboardKeys.EMERGENCY_STOP_ACTIVE" in code
    assert "BlackboardKeys.SAFETY_STOP_ACTIVE" in code
    assert "BlackboardKeys.System.EMERGENCY_STOP" in code


@pytest.mark.unit
@pytest.mark.skipif(
    not pytest.importorskip("rclpy", reason="rclpy not available"),
    reason="rclpy required for node test",
)
def test_hardware_interface_blackboard_set_called_on_telemetry_and_emergency(
    mock_blackboard, mock_blackboard_keys
):
    """When read_battery_state, read_chassis_velocity, and emergency_stop_callback run, blackboard.set is called with expected keys."""
    import rclpy
    from std_msgs.msg import Bool

    mod = "autonomy.autonomy_core.autonomy_core.control.hardware_interface_node"
    with patch(mod + ".UnifiedBlackboardClient", return_value=mock_blackboard), patch(
        mod + ".BlackboardKeys", mock_blackboard_keys
    ):
        from autonomy.autonomy_core.autonomy_core.control.hardware_interface_node import (
            HardwareInterfaceNode,
        )

        try:
            try:
                rclpy.init()
            except RuntimeError:
                pass
            node = HardwareInterfaceNode()
            node.blackboard = mock_blackboard
            mock_blackboard.set.reset_mock()

            # Telemetry path: read_* populate _blackboard_updates; telemetry_loop flushes via _update_blackboard_batch
            node.telemetry_loop()
            node.emergency_stop_callback(Bool(data=True))
            node.emergency_stop_callback(Bool(data=False))

            calls = mock_blackboard.set.call_args_list
            keys = [c[0][0] for c in calls]
            assert any(
                "battery" in str(k).lower() for k in keys
            ), f"Expected at least one battery-related set(); keys: {keys}"
            assert any(
                "velocity" in str(k).lower() for k in keys
            ), f"Expected velocity set(); keys: {keys}"
            assert any(
                "emergency" in str(k).lower() or "safety" in str(k).lower()
                for k in keys
            ), f"Expected emergency/safety set(); keys: {keys}"
            assert any(
                c[0][1] is False for c in calls
            ), "Expected at least one False (emergency clear)"
        finally:
            node.destroy_node()
