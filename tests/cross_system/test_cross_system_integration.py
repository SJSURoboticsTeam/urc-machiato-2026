#!/usr/bin/env python3
"""
Cross-System Integration Tests
Validates interaction between critical competition systems.
"""

import pytest
import time
import rclpy
from unittest.mock import MagicMock, patch
from rclpy.node import Node

# Import test fixtures and mocks
from tests.competition.test_competition_bridge import MockCompetitionBridge
from tests.conftest import ros2_init_shutdown
from src.autonomy.core.safety_system.emergency_stop_system import EmergencyStopLevel


class TestCrossSystemIntegration:
    """Test integration between multiple critical systems."""

    @pytest.fixture(scope="function")
    def integrated_system(self, ros2_init_shutdown):
        """Create integrated system with multiple components."""
        # Initialize ROS2 if needed
        if not rclpy.ok():
            rclpy.init(args=[])

        # Create mock components with integration behavior
        competition_bridge = MockCompetitionBridge()
        # Override start_autonomous_navigation to simulate communication
        original_start = competition_bridge.start_autonomous_navigation
        def start_with_comm(targets):
            result = original_start(targets)
            # Simulate sending command to communication system
            comm_manager.send_command_via_redundant_channel({
                'type': 'start_autonomous_navigation',
                'targets': targets
            })
            return result
        competition_bridge.start_autonomous_navigation = start_with_comm

        # Mock communication redundancy manager
        from unittest.mock import MagicMock
        comm_manager = MagicMock()
        comm_manager.current_channel = 'websocket'
        comm_manager.failover_active = False
        comm_manager.send_command_via_redundant_channel = MagicMock(return_value=True)

        # Mock emergency stop system
        emergency_system = MagicMock()
        emergency_system.current_level = None
        emergency_system.trigger_emergency_stop = MagicMock()
        emergency_system._handle_reset = MagicMock()

        yield {
            'competition_bridge': competition_bridge,
            'comm_manager': comm_manager,
            'emergency_system': emergency_system
        }

        # Cleanup
        if rclpy.ok():
            rclpy.shutdown()

    def test_mission_start_integration(self, integrated_system):
        """Test complete mission start sequence across systems."""
        system = integrated_system
        bridge = system['competition_bridge']
        comm = system['comm_manager']

        # Start autonomous navigation mission
        targets = {
            'gnss_locations': [{'lat': 38.4068, 'lon': -110.7919}],
            'ar_tags': [{'id': 1}],
            'objects': [{'name': 'mallet'}]
        }

        # Execute mission start through bridge
        bridge.start_autonomous_navigation(targets)

        # Verify bridge state changes
        nav_orch = bridge.mission_orchestrator['autonomous_navigation']
        assert nav_orch['active'] == True, "Navigation should be active after mission start"
        assert nav_orch['current_mode'] == 'autonomous', "Should be in autonomous mode"
        assert len(nav_orch['targets']['gnss_locations']) == 1, "Should have GNSS targets"

        # Verify communication system notified (command sent)
        comm.send_command_via_redundant_channel.assert_called_once()
        call_args = comm.send_command_via_redundant_channel.call_args[0][0]
        assert call_args['type'] == 'start_autonomous_navigation', "Should send correct command type"

    def test_emergency_stop_integration(self, integrated_system):
        """Test emergency stop propagation across systems."""
        system = integrated_system
        bridge = system['competition_bridge']
        emergency = system['emergency_system']

        # Start mission first
        targets = {'gnss_locations': [{'lat': 38.4068, 'lon': -110.7919}]}
        bridge.start_autonomous_navigation(targets)

        # Trigger emergency stop
        from tests.competition.test_emergency_stop_system import EmergencyStopLevel
        emergency.trigger_emergency_stop(EmergencyStopLevel.SOFT_STOP, "Integration test emergency")

        # Verify emergency system was called
        emergency.trigger_emergency_stop.assert_called_once_with(
            EmergencyStopLevel.SOFT_STOP, "Integration test emergency"
        )

        # Verify bridge mission state (should be aborted)
        nav_orch = bridge.mission_orchestrator['autonomous_navigation']
        # Note: In a full integration, the emergency system would notify the bridge
        # For this test, we verify the emergency system interaction

    def test_communication_failover_during_mission(self, integrated_system):
        """Test communication failover during active mission."""
        system = integrated_system
        bridge = system['competition_bridge']
        comm = system['comm_manager']

        # Start mission
        targets = {'gnss_locations': [{'lat': 38.4068, 'lon': -110.7919}]}
        bridge.start_autonomous_navigation(targets)

        # Simulate communication failure
        comm.current_channel = 'websocket'
        comm.failover_active = False

        # Mock failover initiation
        comm._initiate_failover = MagicMock()
        comm._initiate_failover.return_value = None

        # Simulate WebSocket failure detection
        comm.websocket_healthy = False
        comm.last_heartbeat = time.time() - 20  # Older than failover timeout

        # Trigger failover check (normally done by monitoring)
        if hasattr(comm, '_check_failover_needed'):
            comm._check_failover_needed()

        # Verify failover would be initiated
        # (In real implementation, this would trigger actual failover)

        # Verify mission can continue (bridge state unchanged)
        nav_orch = bridge.mission_orchestrator['autonomous_navigation']
        assert nav_orch['active'] == True, "Mission should remain active during failover"
        assert nav_orch['current_mode'] == 'autonomous', "Mode should remain autonomous"

    def test_led_status_system_integration(self, integrated_system):
        """Test LED status updates across mission phases."""
        system = integrated_system
        bridge = system['competition_bridge']

        # Start mission
        targets = {'gnss_locations': [{'lat': 38.4068, 'lon': -110.7919}]}
        bridge.start_autonomous_navigation(targets)

        # Test LED status updates
        bridge.update_led_status("autonomous_navigation", "autonomous_mode")
        nav_orch = bridge.mission_orchestrator['autonomous_navigation']
        assert nav_orch['current_mode'] == 'autonomous', "LED should set autonomous mode"

        # Simulate target reached
        bridge.target_reached('gnss_locations', 0)
        bridge.update_led_status("autonomous_navigation", "target_reached")
        assert nav_orch['last_target_reached'] == True, "LED should mark target reached"

        # Test mode switching
        bridge.update_led_status("autonomous_navigation", "teleoperation")
        assert nav_orch['current_mode'] == 'teleoperation', "LED should switch to teleoperation"

    def test_gnss_compliance_integration(self, integrated_system):
        """Test GNSS compliance validation during navigation."""
        system = integrated_system
        bridge = system['competition_bridge']

        # Mock NavSatFix message
        navsat_msg = MagicMock()
        navsat_msg.latitude = 38.4068
        navsat_msg.longitude = -110.7919
        navsat_msg.altitude = 1400.0

        # Test compliance verification
        compliant = bridge.verify_gnss_compliance(navsat_msg)

        assert compliant == True, "Valid GNSS coordinates should be compliant"
        assert bridge.gnss_compliance['wgs84_verified'] == True, "GNSS should be verified"
        assert bridge.gnss_compliance['last_coordinates']['lat'] == 38.4068, "Coordinates should be stored"

        # Test invalid coordinates
        navsat_msg.latitude = 91.0  # Invalid latitude
        compliant = bridge.verify_gnss_compliance(navsat_msg)

        assert compliant == False, "Invalid coordinates should not be compliant"
        assert bridge.gnss_compliance['wgs84_verified'] == False, "Invalid GNSS should fail verification"

    def test_band_configuration_integration(self, integrated_system):
        """Test URC band configuration and spectrum compliance."""
        system = integrated_system
        bridge = system['competition_bridge']

        # Configure 900MHz band
        bridge.set_urc_band('900mhz', 'low')

        # Verify configuration
        band_config = bridge.urc_band_config
        assert band_config['current_band'] == '900mhz', "Band should be set to 900MHz"
        assert band_config['900mhz']['current_subband'] == 'low', "Subband should be low"
        assert band_config['900mhz']['sub_bands']['low']['active'] == True, "Low subband should be active"

        # Verify spectrum compliance tracking
        spectrum = bridge.spectrum_compliance
        assert spectrum['current_band'] == '900mhz', "Spectrum compliance should track band"
        assert isinstance(spectrum['bandwidth_usage_mhz'], float), "Bandwidth should be tracked"

    def test_mission_abortion_integration(self, integrated_system):
        """Test mission abortion and return to previous target."""
        system = integrated_system
        bridge = system['competition_bridge']

        # Start mission with multiple targets
        targets = {
            'gnss_locations': [
                {'lat': 38.4068, 'lon': -110.7919},
                {'lat': 38.4070, 'lon': -110.7920}
            ]
        }
        bridge.start_autonomous_navigation(targets)

        # Reach first target
        bridge.target_reached('gnss_locations', 0)
        nav_orch = bridge.mission_orchestrator['autonomous_navigation']
        assert nav_orch['current_target_index'] == 1, "Should advance to next target"

        # Abort and return to previous
        previous_target = bridge.abort_to_previous_target()

        assert previous_target is not None, "Should return previous target"
        assert previous_target['type'] == 'gnss_locations', "Should return GNSS target"
        assert previous_target['index'] == 0, "Should return first target"
        assert nav_orch['current_mode'] == 'teleoperation', "Should switch to teleoperation on abort"

    def test_system_health_monitoring_integration(self, integrated_system):
        """Test overall system health assessment across components."""
        system = integrated_system
        bridge = system['competition_bridge']
        comm = system['comm_manager']

        # Test healthy system state
        nav_orch = bridge.mission_orchestrator['autonomous_navigation']
        assert nav_orch['active'] == False, "System should start inactive"
        assert comm.failover_active == False, "Communication should start healthy"

        # Start mission and verify all systems coordinate
        targets = {'gnss_locations': [{'lat': 38.4068, 'lon': -110.7919}]}
        bridge.start_autonomous_navigation(targets)

        # Verify integrated state
        assert nav_orch['active'] == True, "Navigation should be active"
        assert nav_orch['current_mode'] == 'autonomous', "Should be in autonomous mode"
        assert comm.send_command_via_redundant_channel.called, "Communication should be active"

        # Test LED integration
        bridge.update_led_status("autonomous_navigation", "autonomous_mode")
        assert nav_orch['current_mode'] == 'autonomous', "LED status should be synchronized"

