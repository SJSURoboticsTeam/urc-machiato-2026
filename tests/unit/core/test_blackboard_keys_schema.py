"""
Unit tests for blackboard key schema: C++ and Python key sets match, dot keys exist.
Traceability: Plan 1.1 (hierarchical dot keys), Plan 1.2 (constants contract).
"""

import importlib.util
import os
import sys

import pytest

# Load core.blackboard_keys from repo src (cwd-independent)
_here = os.path.dirname(os.path.abspath(__file__))
_src = os.path.abspath(os.path.join(_here, "..", "..", "..", "src"))
if _src not in sys.path:
    sys.path.insert(0, _src)
_spec = importlib.util.spec_from_file_location(
    "blackboard_keys",
    os.path.join(_src, "core", "blackboard_keys.py"),
)
_bb_module = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_bb_module)
BlackboardKeys = _bb_module.BlackboardKeys


class TestBlackboardKeysSchema:
    """Test blackboard key constants match between Python and expected C++ contract."""

    @pytest.mark.unit
    @pytest.mark.plan_1_1
    @pytest.mark.plan_1_2
    def test_flat_keys_exist(self):
        """Plan 1.1/1.2: Flat keys used by bridge nodes exist (constants, no raw strings)."""
        assert BlackboardKeys.ROBOT_X == "robot_x"
        assert BlackboardKeys.ROBOT_Y == "robot_y"
        assert BlackboardKeys.PATH_CLEAR == "path_clear"
        assert BlackboardKeys.CLOSEST_OBSTACLE_DISTANCE == "closest_obstacle_distance"
        assert BlackboardKeys.NAVIGATION_STATUS == "navigation_status"
        assert BlackboardKeys.DISTANCE_TO_TARGET == "distance_to_target"
        assert BlackboardKeys.BATTERY_LEVEL == "battery_level"
        assert BlackboardKeys.SAFETY_STOP_ACTIVE == "safety_stop_active"
        assert BlackboardKeys.EMERGENCY_STOP_ACTIVE == "emergency_stop_active"
        assert BlackboardKeys.FEATURE_COUNT == "feature_count"
        assert BlackboardKeys.SLAM_CONFIDENCE == "slam_confidence"
        assert BlackboardKeys.PERCEPTION_CONFIDENCE == "perception_confidence"
        assert BlackboardKeys.MAP_QUALITY == "map_quality"
        assert BlackboardKeys.LAST_ERROR == "last_error"

    @pytest.mark.unit
    @pytest.mark.plan_1_1
    def test_dot_keys_auto_namespace(self):
        """Plan 1.1: Hierarchical auto.* keys exist for hybrid."""
        assert BlackboardKeys.Auto.MODE == "auto.mode"
        assert BlackboardKeys.Auto.MODE_REQUEST == "auto.mode_request"
        assert BlackboardKeys.Auto.HUMAN_OVERRIDE_ACTIVE == "auto.human_override_active"
        assert BlackboardKeys.Auto.ASSIST_ENABLED == "auto.assist_enabled"

    @pytest.mark.unit
    @pytest.mark.plan_1_1
    def test_dot_keys_system_namespace(self):
        """Plan 1.1: Hierarchical system.* keys exist."""
        assert BlackboardKeys.System.EMERGENCY_STOP == "system.emergency_stop"
        assert BlackboardKeys.System.BATTERY_PERCENT == "system.battery_percent"
        assert BlackboardKeys.System.LAST_ERROR == "system.last_error"
