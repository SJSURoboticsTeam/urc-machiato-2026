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

# Import from simplified utilities
sys.path.insert(
    0, os.path.join(os.path.dirname(__file__), "..", "..", "src", "autonomy", "code")
)
from utilities import get_validated_parameter, load_config_file, safe_execute


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
        assert isinstance(error, ZeroDivisionError)
        assert "division by zero" in str(error)

    def test_get_validated_parameter_success(self):
        """Test parameter validation with valid value."""
        # Use a dictionary as expected by the function
        params = {"test_param": 42}
        result = get_validated_parameter(params, "test_param", int)
        assert result == 42

    def test_get_validated_parameter_validation(self):
        """Test parameter validation with invalid value."""


class MockNode:
    def get_parameter(self, name):
        return type("Param", (), {"value": -5})()

    def get_logger(self):
        return type("Logger", (), {"warn": lambda *args: None})()

        node = MockNode()
        result = get_validated_parameter(node, "test_param", 10, lambda x: x > 0)
        assert result == 10  # Should return default for invalid value

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
        """Test loading config with missing required key."""
        config_data = {"existing_key": "value"}

        with tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False) as f:
            yaml.dump(config_data, f)
            config_file = f.name

        try:
            result = load_config_file(config_file, ["missing_key"])
            assert result is None  # Should return None for missing required key
        finally:
            os.unlink(config_file)
