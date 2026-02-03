#!/usr/bin/env python3
"""
Pytest Configuration and Shared Fixtures

Provides shared test fixtures and configuration for the autonomy test suite.
Ensures tests use newest implementation and start from clean state.
"""

import importlib
import os
import sys
import time
from unittest.mock import Mock

import numpy as np
import pytest

# Ensure we use the absolute latest code - clear any cached imports
modules_to_clear = [
    "bridges.competition_bridge",
    "bridges.telemetry_manager",
    "bridges.websocket_manager",
    "bridges.mission_orchestrator",
    "core.dds_domain_redundancy_manager",
    "core.recovery_coordinator",
    "config.config_manager",
]

for module in modules_to_clear:
    if module in sys.modules:
        del sys.modules[module]

# Set up project paths to ensure latest code
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), ".."))
SRC_ROOT = os.path.join(PROJECT_ROOT, "src")

# Clear and reset Python path for clean imports
original_path = sys.path[:]
sys.path.clear()
sys.path.append(PROJECT_ROOT)
sys.path.append(SRC_ROOT)
sys.path.extend(original_path)

# ROS2 testing imports and setup
try:
    import rclpy

    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    rclpy = None


@pytest.fixture(scope="session", autouse=True)
def ros2_setup_teardown():
    """Initialize and shutdown ROS2 context for tests that need it."""
    if ROS2_AVAILABLE:
        # Initialize ROS2 if not already initialized
        if not rclpy.utilities.ok():
            rclpy.init()

        yield

        # Shutdown ROS2 after all tests
        if rclpy.utilities.ok():
            rclpy.shutdown()
    else:
        yield


@pytest.fixture
def ros2_node():
    """Provide a ROS2 node for tests that need it."""
    if not ROS2_AVAILABLE:
        pytest.skip("ROS2 not available")

    from rclpy.node import Node

    node = Node("test_node")
    yield node
    node.destroy_node()


@pytest.fixture
def mock_config():
    """Provide mock configuration for testing."""
    return {
        "system": {
            "name": "URC 2026 Test Rover",
            "version": "1.0.0",
            "environment": "testing",
        },
        "navigation": {
            "gps_timeout": 5.0,
            "imu_timeout": 1.0,
            "waypoint_tolerance": 0.5,
        },
        "communication": {"websocket_port": 8080, "ros_domain_id": 42},
    }
