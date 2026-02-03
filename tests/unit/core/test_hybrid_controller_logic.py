"""
Unit tests for hybrid controller logic: emergency wins, then override, then mode.
Pure Python, no ROS2; uses dict as blackboard stand-in.
Traceability: Plan 3 (shared control / command source), Plan 5.1.
"""

import pytest


def command_source(bb: dict) -> str:
    """
    Decide command source from blackboard (stand-in for real mixer).
    Returns "emergency" | "teleop" | "autonomous".
    """
    if bb.get("system.emergency_stop") or bb.get("emergency_stop_active"):
        return "emergency"
    if bb.get("auto.human_override_active"):
        return "teleop"
    mode = bb.get("auto.mode", "idle")
    if mode == "teleop":
        return "teleop"
    if mode == "autonomous":
        return "autonomous"
    return "teleop"  # default safe


class TestHybridControllerLogic:
    """Test priority: emergency > human override > mode."""

    @pytest.mark.unit
    @pytest.mark.plan_3
    @pytest.mark.plan_5_1
    def test_emergency_wins(self):
        """Plan 3/5.1: When system.emergency_stop is true, command source is emergency."""
        bb = {
            "auto.mode": "autonomous",
            "auto.human_override_active": True,
            "system.emergency_stop": True,
        }
        assert command_source(bb) == "emergency"

    @pytest.mark.unit
    @pytest.mark.plan_3
    def test_override_over_mode(self):
        """Plan 3: When human_override_active and autonomous, command source is teleop."""
        bb = {
            "auto.mode": "autonomous",
            "auto.human_override_active": True,
            "system.emergency_stop": False,
        }
        assert command_source(bb) == "teleop"

    @pytest.mark.unit
    @pytest.mark.plan_3
    def test_mode_when_no_override(self):
        """Plan 3: When no override, command source follows auto.mode."""
        bb = {
            "auto.mode": "autonomous",
            "auto.human_override_active": False,
            "system.emergency_stop": False,
        }
        assert command_source(bb) == "autonomous"
        bb["auto.mode"] = "teleop"
        assert command_source(bb) == "teleop"

    @pytest.mark.unit
    @pytest.mark.plan_3
    def test_emergency_stop_active_flat_key(self):
        """Plan 3: emergency_stop_active (flat) also triggers emergency."""
        bb = {"emergency_stop_active": True, "auto.mode": "autonomous"}
        assert command_source(bb) == "emergency"
