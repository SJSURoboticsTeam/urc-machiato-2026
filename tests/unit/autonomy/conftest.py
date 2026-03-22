"""Autonomy unit-test fixtures (navigation, BT, missions)."""

from __future__ import annotations

import os
from typing import Any, Dict
from unittest.mock import MagicMock

import pytest


@pytest.fixture
def mock_nav2_costmap() -> MagicMock:
    """Lightweight stand-in for costmap / occupancy interfaces."""
    m = MagicMock()
    m.get_size_in_cells_x.return_value = 64
    m.get_size_in_cells_y.return_value = 64
    m.get_resolution.return_value = 0.05
    return m


@pytest.fixture
def sample_rover_yaml_path() -> str:
    """Path to main rover config if present (for parsers that read YAML)."""
    root = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
    path = os.path.join(root, "config", "rover.yaml")
    if not os.path.isfile(path):
        pytest.skip("config/rover.yaml not found")
    return path


@pytest.fixture
def mission_params() -> Dict[str, Any]:
    """Minimal mission-like parameter bag for BT / executor unit tests."""
    return {
        "mission_id": "unit_test_mission",
        "timeout_s": 5.0,
        "retry_limit": 2,
    }
