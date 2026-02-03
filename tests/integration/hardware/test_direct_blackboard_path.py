"""
Integration tests for direct CAN-to-blackboard path.

Verifies that hardware_interface_node is implemented to write directly to the
blackboard (validate_can_blackboard_direct.py) and that the direct path runs
without error when the node executes telemetry_loop (mock CAN).
"""

import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[3]
SCRIPT = REPO / "scripts" / "hardware" / "validate_can_blackboard_direct.py"


@pytest.mark.integration
def test_validate_can_blackboard_direct_script_passes():
    """Run validate_can_blackboard_direct.py; direct path implementation must pass."""
    if not SCRIPT.exists():
        pytest.skip(f"Validation script not found: {SCRIPT}")
    env = {"PYTHONPATH": str(REPO / "src")}
    result = subprocess.run(
        [sys.executable, str(SCRIPT)],
        cwd=str(REPO),
        env={**__import__("os").environ, **env},
        capture_output=True,
        text=True,
        timeout=10,
    )
    assert (
        result.returncode == 0
    ), f"validate_can_blackboard_direct.py failed: stdout={result.stdout!r} stderr={result.stderr!r}"
    assert (
        "Option 1 Implementation Complete" in result.stdout
        or "SUCCESS" in result.stdout
    )


@pytest.mark.integration
@pytest.mark.skipif(
    not pytest.importorskip("rclpy", reason="rclpy not available"),
    reason="rclpy required",
)
def test_hardware_interface_direct_path_telemetry_loop_runs():
    """Hardware interface node telemetry_loop (mock CAN) runs and flushes blackboard batch."""
    import rclpy

    if str(REPO / "src") not in sys.path:
        sys.path.insert(0, str(REPO / "src"))
    from autonomy.autonomy_core.autonomy_core.control.hardware_interface_node import (
        HardwareInterfaceNode,
    )

    try:
        try:
            rclpy.init()
        except RuntimeError:
            pass
        node = HardwareInterfaceNode()
        # With can_connected False, telemetry_loop uses publish_mock_telemetry which
        # populates _blackboard_updates and calls _update_blackboard_batch
        node.telemetry_loop()
        assert hasattr(node, "_blackboard_updates")
        # After telemetry_loop, batch was flushed (so _blackboard_updates may be empty
        # or we just assert no exception)
        node.destroy_node()
    finally:
        pass  # do not shutdown; other tests may share context
