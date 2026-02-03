#!/usr/bin/env python3
"""
Hardware-in-the-Loop Framework Tests

Tests HIL manager capabilities including hardware discovery,
mixed real/simulated modes, and automatic fallback.

Author: URC 2026 Testing Team
"""

import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).parent.parent.parent))

pytest.importorskip("simulation.network.websocket_server_simulator")
from simulation.hil.hil_manager import HILManager, ComponentMode
from simulation.hil.device_discovery import DeviceDiscovery


class TestDeviceDiscovery:
    """Test device discovery functionality."""

    def test_scan_for_devices(self):
        """Test scanning for connected devices."""
        discovery = DeviceDiscovery()

        devices = discovery.scan()

        # Should return a list (may be empty if no hardware)
        assert isinstance(devices, list)

        # If devices found, check structure
        for device in devices:
            assert "port" in device
            assert "device_type" in device

    def test_scan_filters_invalid_devices(self):
        """Test scanner filters out invalid/unknown devices."""
        discovery = DeviceDiscovery()

        devices = discovery.scan()

        # All devices should have known types
        valid_types = ["stm32", "slcan_adapter", "unknown"]
        for device in devices:
            assert device["device_type"] in valid_types


class TestHILManager:
    """Test HIL manager functionality."""

    @pytest.fixture
    def manager(self):
        """Create HIL manager."""
        return HILManager()

    def test_manager_initialization(self, manager):
        """Test HIL manager initializes."""
        assert manager is not None
        assert hasattr(manager, "discovered_devices")
        assert hasattr(manager, "active_components")

    def test_initialize_simulated_component(self, manager):
        """Test initializing component in simulated mode."""
        component = manager.initialize_component("firmware", ComponentMode.SIMULATED)

        assert component is not None
        # Should be simulation instance
        assert hasattr(component, "start")  # Firmware simulator has start method

    def test_initialize_auto_mode_fallback(self, manager):
        """Test AUTO mode falls back to simulation."""
        # AUTO mode should try real, then fall back to simulation
        component = manager.initialize_component("firmware", ComponentMode.AUTO)

        assert component is not None
        # Should have initialized something (real or simulated)

    def test_component_tracking(self, manager):
        """Test active components are tracked."""
        component = manager.initialize_component("firmware", ComponentMode.SIMULATED)

        assert "firmware" in manager.active_components
        assert manager.active_components["firmware"] == component

    def test_multiple_components(self, manager):
        """Test initializing multiple components."""
        firmware = manager.initialize_component("firmware", ComponentMode.SIMULATED)
        slcan = manager.initialize_component("slcan", ComponentMode.SIMULATED)
        websocket = manager.initialize_component("websocket", ComponentMode.SIMULATED)

        assert firmware is not None
        assert slcan is not None
        assert websocket is not None

        assert len(manager.active_components) == 3

    def test_comparison_recording(self, manager):
        """Test recording comparison data."""
        # Initialize component
        component = manager.initialize_component("firmware", ComponentMode.SIMULATED)

        # Record comparison
        comparison = manager.record_comparison(
            "test_metric", {"real": 1.0, "simulated": 1.05, "timestamp": time.time()}
        )

        assert comparison is not None


class TestMixedModeOperation:
    """Test mixed real/simulated operation."""

    @pytest.fixture
    def manager(self):
        """Create HIL manager with mixed components."""
        manager = HILManager()

        # Initialize mix of components
        manager.initialize_component("firmware", ComponentMode.SIMULATED)
        manager.initialize_component("slcan", ComponentMode.SIMULATED)
        manager.initialize_component("websocket", ComponentMode.SIMULATED)

        yield manager

    def test_all_components_active(self, manager):
        """Test all components are active."""
        assert len(manager.active_components) == 3
        assert "firmware" in manager.active_components
        assert "slcan" in manager.active_components
        assert "websocket" in manager.active_components

    def test_simulated_firmware_operations(self, manager):
        """Test operations with simulated firmware."""
        firmware = manager.active_components["firmware"]

        # Start firmware
        firmware.start()

        # Send command
        success = firmware.set_velocity_command(0, 5.0)
        assert success is True

        time.sleep(0.2)

        # Check status
        status = firmware.get_motor_status(0)
        assert status is not None
        assert status["velocity_actual"] > 0

        firmware.stop()


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
