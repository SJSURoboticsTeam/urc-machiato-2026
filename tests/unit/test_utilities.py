#!/usr/bin/env python3
"""
Unit tests for simplified autonomy utilities.

Tests the core utility functions without ROS2 dependencies.
"""

import os
import sys
import tempfile
from typing import Any, Dict

import pytest
import yaml

# Import from autonomy utilities
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))
from autonomy.utilities.autonomy_utilities import safe_execute, load_config_file


class TestCoreUtilities:
    """Test simplified utility functions."""

    def test_safe_execute_success(self):
        """Test safe_execute with successful operation."""
        result, error = safe_execute(lambda: 42)
        assert result == 42
        assert error is None

    def test_safe_execute_failure(self):
        """Test safe_execute with failed operation."""
        result, error = safe_execute(lambda: 1 / 0)
        assert result is None
        assert isinstance(error, str)
        assert "division by zero" in error


    def test_load_config_file_success(self):
        """Test loading valid YAML config file."""
        config_data = {"test_key": "test_value", "number": 42}

        # Create temporary config file
        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            yaml.dump(config_data, f)
            config_file = f.name

        try:
            result = load_config_file(config_file)
            assert result == config_data
        finally:
            os.unlink(config_file)

    def test_load_config_file_missing_required_key(self):
        """Test loading config with missing file (should return empty dict)."""
        config_file = "/nonexistent/config.yaml"

        result = load_config_file(config_file)
        assert result == {}  # Should return empty dict for missing file
