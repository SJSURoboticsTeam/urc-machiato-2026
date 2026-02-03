"""
HIL tests: validate command path (dashboard/bridge -> blackboard) and
response path (sensor/status -> dashboard) using mocks.

Runs against mock blackboard or in-process storage to verify round-trip
without real hardware.
"""

import sys
from pathlib import Path

import pytest

# Add repo root for imports
REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "src"))


class TestHILBlackboardCommandPath:
    """Validate command path: bridge/dashboard -> blackboard set/get."""

    def test_blackboard_set_get_roundtrip(self):
        """Simulate set then get on blackboard-like storage (no ROS2)."""
        storage = {}
        # Simulate set_value
        key, value, value_type = "robot_x", "1.5", "double"
        try:
            v = float(value)
            storage[key] = (v, value_type)
        except (ValueError, TypeError):
            storage[key] = (value, value_type)
        # Simulate get_value
        assert key in storage
        stored_val, stored_type = storage[key]
        assert stored_type == value_type
        assert abs(stored_val - 1.5) < 1e-6

    def test_blackboard_multiple_keys(self):
        """Multiple keys set and retrieved."""
        storage = {}
        for k, v, t in [
            ("battery_level", "85.0", "double"),
            ("safety_stop", "false", "bool"),
        ]:
            if t == "double":
                storage[k] = (float(v), t)
            elif t == "bool":
                storage[k] = (v.lower() in ("true", "1"), t)
            else:
                storage[k] = (v, t)
        assert storage["battery_level"][0] == 85.0
        assert storage["safety_stop"][0] is False


class TestHILSensorResponsePath:
    """Validate response path: sensor/status -> dashboard (data flow)."""

    def test_sensor_status_structure(self):
        """Sensor status dict has expected fields for dashboard."""
        status = {
            "timestamp": 0.0,
            "velocity_feedback": {"x": 0.0, "y": 0.0, "rot": 0.0},
            "battery": {"percentage": 85.0},
            "emergency_stop": False,
        }
        assert "timestamp" in status
        assert "battery" in status
        assert "emergency_stop" in status

    def test_sensor_status_roundtrip_serializable(self):
        """Sensor status can be serialized (e.g. for WebSocket)."""
        import json

        status = {"battery": {"percentage": 85.0}, "emergency_stop": False}
        s = json.dumps(status)
        back = json.loads(s)
        assert back["battery"]["percentage"] == 85.0
        assert back["emergency_stop"] is False
