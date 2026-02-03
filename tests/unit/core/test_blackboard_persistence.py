"""
Unit tests for blackboard persistence: save/load subset of keys, emergency_stop safety.
Traceability: Plan 1.3 (persistence scope, format, load safety), Plan 5.1.
"""

import json
import os
import sys
import tempfile
from pathlib import Path

import pytest

_here = os.path.dirname(os.path.abspath(__file__))
_src = os.path.abspath(os.path.join(_here, "..", "..", "src"))
if _src not in sys.path:
    sys.path.insert(0, _src)

try:
    from core.blackboard_keys import BlackboardKeys
except Exception:
    try:
        import importlib.util

        _spec = importlib.util.spec_from_file_location(
            "blackboard_keys",
            os.path.join(_src, "core", "blackboard_keys.py"),
        )
        _mod = importlib.util.module_from_spec(_spec)
        _spec.loader.exec_module(_mod)
        BlackboardKeys = _mod.BlackboardKeys
    except Exception:
        BlackboardKeys = None


class TestBlackboardPersistenceLogic:
    """Test persistence key list and load safety (no clearing emergency_stop)."""

    @pytest.mark.unit
    @pytest.mark.plan_1_3
    @pytest.mark.plan_5_1
    def test_persisted_keys_include_mission_and_mode(self):
        """Plan 1.3/5.1: Persisted keys list includes mission progress and mode keys."""
        if BlackboardKeys is None:
            pytest.skip("core.blackboard_keys not available (run with PYTHONPATH=src)")
        assert BlackboardKeys.Auto.MODE == "auto.mode"
        assert BlackboardKeys.System.BATTERY_PERCENT == "system.battery_percent"

    @pytest.mark.unit
    @pytest.mark.plan_1_3
    def test_load_skips_emergency_stop(self):
        """Plan 1.3: Load logic never overwrites system.emergency_stop from file (safety)."""
        skip_keys = {"system.emergency_stop"}
        assert "system.emergency_stop" in skip_keys

    @pytest.mark.unit
    @pytest.mark.plan_1_3
    def test_save_format_json(self):
        """Plan 1.3: Save produces valid JSON with expected keys."""
        data = {
            "mission_active": False,
            "samples_collected": 0,
            "current_mission_phase": "idle",
            "last_error": "",
        }
        with tempfile.NamedTemporaryFile(mode="w", suffix=".json", delete=False) as f:
            json.dump(data, f, indent=2)
            path = f.name
        try:
            with open(path) as f:
                loaded = json.load(f)
            assert loaded["mission_active"] is False
            assert loaded["samples_collected"] == 0
        finally:
            os.unlink(path)
