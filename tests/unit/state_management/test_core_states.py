#!/usr/bin/env python3
"""
Simple test script for core state machine functionality.

Tests state definitions, transitions, and validation logic without ROS2 dependencies.
"""

import os
import sys

import pytest

# Add the state machine to path
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
STATE_MGMT_ROOT = os.path.join(PROJECT_ROOT, "Autonomy", "code", "state_management")
sys.path.insert(0, STATE_MGMT_ROOT)
sys.path.insert(0, os.path.join(STATE_MGMT_ROOT, "autonomy_state_machine"))

try:
    from autonomy_state_machine.states import (
        AutonomousSubstate,
        EquipmentServicingSubstate,
        SystemState,
        get_required_subsystems,
        get_state_metadata,
        is_valid_transition,
    )
except ImportError:
    pytest.skip("autonomy_state_machine package not importable", allow_module_level=True)


@pytest.mark.unit
class TestStateDefinitions:
    """Test state definitions and metadata."""

    def test_system_states_exist(self):
        """Test that all system states are properly defined."""
        # Test SystemState enum
        assert SystemState.BOOT.value == "BOOT"
        assert SystemState.CALIBRATION.value == "CALIBRATION"
        assert SystemState.IDLE.value == "IDLE"
        assert SystemState.TELEOPERATION.value == "TELEOPERATION"
        assert SystemState.AUTONOMOUS.value == "AUTONOMOUS"
        assert SystemState.SAFETY.value == "SAFETY"
        assert SystemState.SHUTDOWN.value == "SHUTDOWN"

    def test_autonomous_substates_exist(self):
        """Test that autonomous substates are defined."""
        # Test AutonomousSubstate enum
        assert AutonomousSubstate.NONE.value == "NONE"
        assert AutonomousSubstate.SCIENCE.value == "SCIENCE"
        assert AutonomousSubstate.DELIVERY.value == "DELIVERY"
        assert AutonomousSubstate.EQUIPMENT_SERVICING.value == "EQUIPMENT_SERVICING"
        assert AutonomousSubstate.AUTONOMOUS_NAVIGATION.value == "AUTONOMOUS_NAVIGATION"
        assert AutonomousSubstate.FOLLOW_ME.value == "FOLLOW_ME"

    def test_equipment_servicing_substates_exist(self):
        """Test that equipment servicing substates are defined."""
        # Test EquipmentServicingSubstate enum
        assert EquipmentServicingSubstate.NONE.value == "NONE"
        assert EquipmentServicingSubstate.TRAVELING.value == "TRAVELING"
        assert EquipmentServicingSubstate.SAMPLE_DELIVERY.value == "SAMPLE_DELIVERY"
        assert EquipmentServicingSubstate.PANEL_OPERATIONS.value == "PANEL_OPERATIONS"
        assert EquipmentServicingSubstate.AUTONOMOUS_TYPING.value == "AUTONOMOUS_TYPING"
        assert EquipmentServicingSubstate.USB_CONNECTION.value == "USB_CONNECTION"
        assert EquipmentServicingSubstate.FUEL_CONNECTION.value == "FUEL_CONNECTION"
        assert EquipmentServicingSubstate.BUTTON_OPERATIONS.value == "BUTTON_OPERATIONS"
        assert EquipmentServicingSubstate.COMPLETE.value == "COMPLETE"


@pytest.mark.unit
class TestStateTransitions:
    """Test state transition validation."""

    def test_valid_transitions(self):
        """Test valid state transitions."""
        # Test valid transitions
        assert is_valid_transition(SystemState.BOOT, SystemState.CALIBRATION)
        assert is_valid_transition(SystemState.CALIBRATION, SystemState.IDLE)
        assert is_valid_transition(SystemState.IDLE, SystemState.TELEOPERATION)
        assert is_valid_transition(SystemState.IDLE, SystemState.AUTONOMOUS)
        assert is_valid_transition(SystemState.TELEOPERATION, SystemState.IDLE)
        assert is_valid_transition(SystemState.AUTONOMOUS, SystemState.IDLE)
        assert is_valid_transition(SystemState.AUTONOMOUS, SystemState.SAFETY)
        assert is_valid_transition(SystemState.TELEOPERATION, SystemState.SAFETY)
        assert is_valid_transition(SystemState.IDLE, SystemState.SAFETY)
        assert is_valid_transition(SystemState.SAFETY, SystemState.IDLE)
        assert is_valid_transition(SystemState.IDLE, SystemState.SHUTDOWN)

    def test_invalid_transitions(self):
        """Test invalid state transitions."""
        # Test invalid transitions (should return False)
        assert not is_valid_transition(
            SystemState.SHUTDOWN, SystemState.IDLE
        )  # Can't go back from shutdown (terminal state)
        assert not is_valid_transition(
            SystemState.SAFETY, SystemState.AUTONOMOUS
        )  # Can't go directly to autonomous from safety
        assert not is_valid_transition(
            SystemState.CALIBRATION, SystemState.TELEOPERATION
        )  # Can't skip IDLE


@pytest.mark.unit
class TestStateMetadata:
    """Test state metadata retrieval."""

    def test_state_metadata_exists(self):
        """Test that metadata exists for all states."""
        # Test metadata retrieval
        boot_meta = get_state_metadata(SystemState.BOOT)
        assert boot_meta is not None
        assert boot_meta.description == "Initial system boot and initialization"
        assert boot_meta.entry_requirements == []  # BOOT has no entry requirements

        idle_meta = get_state_metadata(SystemState.IDLE)
        assert idle_meta is not None
        assert "boot_complete" in idle_meta.entry_requirements

    def test_required_subsystems(self):
        """Test required subsystems for different states."""
        # Test required subsystems
        boot_subsystems = get_required_subsystems(SystemState.BOOT)
        # BOOT state may have minimal subsystem requirements
        assert isinstance(boot_subsystems, list)

        autonomous_subsystems = get_required_subsystems(SystemState.AUTONOMOUS)
        assert "navigation" in autonomous_subsystems
        assert "slam" in autonomous_subsystems
        assert "computer_vision" in autonomous_subsystems


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
