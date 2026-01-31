#!/usr/bin/env python3
"""
Competition Bridge + Advanced Systems Integration Test

Tests the integration between Competition Bridge and all advanced systems:
- State Synchronization
- DDS Domain Redundancy
- Dynamic Configuration
- WebSocket Redundancy
- Recovery Coordination

Author: URC 2026 Autonomy Team
"""

import pytest
try:
    from core import state_synchronization_manager  # noqa: F401
except ImportError:
    pytest.skip("state_synchronization_manager removed (archived)", allow_module_level=True)

import os
import sys
import time
import unittest
from unittest.mock import Mock, patch

# Add src to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))


class CompetitionBridgeAdvancedSystemsIntegrationTest(unittest.TestCase):
    """Test Competition Bridge integration with advanced systems."""

    def setUp(self):
        """Setup test environment."""
        # Mock ROS2 components since full ROS2 environment may not be available
        self.mock_ros2 = Mock()
        self.mock_node = Mock()

        # Mock WebSocket components
        self.mock_websocket = Mock()
        self.mock_websocket_server = Mock()

    def test_competition_bridge_state_sync_integration(self):
        """Test Competition Bridge integration with state synchronization."""
        print("[REFRESH] Testing Competition Bridge + State Sync Integration...")

        # Create state manager
        from core.state_synchronization_manager import Distributedget_state_manager()

        state_mgr = Distributedget_state_manager()."competition_bridge_test")
        state_mgr.start()
        state_mgr.register_node("competition_bridge_test")

        # Simulate competition bridge telemetry updates
        telemetry_data = {
            "battery_voltage": 24.5,
            "system_status": "operational",
            "mission_progress": 75,
            "emergency_stop_active": False,
        }

        # Update state manager with telemetry data
        for key, value in telemetry_data.items():
            state_mgr.update_state(f"telemetry_{key}", str(value))

        # Verify state updates
        for key, expected_value in telemetry_data.items():
            actual_value = state_mgr.get_state(f"telemetry_{key}")
            self.assertEqual(
                actual_value,
                str(expected_value),
                f"State {key} should be updated correctly",
            )

        # Trigger election to establish master
        state_mgr._trigger_election()

        # Test state manager status
        status = state_mgr.get_system_status()
        self.assertEqual(status["role"], "master", "State manager should be master")
        self.assertIsNotNone(status["master_node"], "Master node should be set")

        print("  [PASS] Competition Bridge + State Sync integration works")
        state_mgr.stop()

    def test_competition_bridge_dds_domain_integration(self):
        """Test Competition Bridge integration with DDS domain redundancy."""
        print("[NETWORK] Testing Competition Bridge + DDS Domain Integration...")

        # Create DDS manager
        from core.dds_domain_redundancy_manager import DDSDomainRedundancyManager

        dds_mgr = DDSDomainRedundancyManager(primary_domain=100)
        dds_mgr.start()

        # Register competition-related nodes
        dds_mgr.register_node(
            "competition_bridge", "ros2 run competition_bridge competition_bridge"
        )
        dds_mgr.register_node("state_manager", "ros2 run state_manager state_manager")
        dds_mgr.register_node("telemetry_node", "ros2 run telemetry telemetry_node")

        # Verify domain setup
        status = dds_mgr.get_system_status()
        self.assertEqual(
            status["current_domain"], 100, "Should start on primary domain"
        )
        self.assertEqual(
            len(status["nodes"]), 3, "Should have registered competition nodes"
        )

        # Test domain health monitoring
        health_score = dds_mgr._measure_domain_health(100)
        self.assertIsInstance(health_score, float, "Health score should be calculable")
        self.assertGreaterEqual(health_score, 0.0, "Health score should be valid")

        # Test domain failover capability
        original_measure = dds_mgr._measure_domain_health
        dds_mgr._measure_domain_health = lambda x: 0.0 if x == 100 else 0.9

        success = dds_mgr.trigger_domain_failover(target_domain_id=101)
        self.assertTrue(success, "Domain failover should succeed")

        failover_status = dds_mgr.get_system_status()
        self.assertEqual(
            failover_status["current_domain"], 101, "Should failover to backup domain"
        )

        print("  [PASS] Competition Bridge + DDS Domain integration works")

        # Restore and cleanup
        dds_mgr._measure_domain_health = original_measure
        dds_mgr.stop()

    def test_competition_bridge_config_integration(self):
        """Test Competition Bridge integration with dynamic configuration."""
        print(" Testing Competition Bridge + Dynamic Config Integration...")

        # Create config manager
        from core.dynamic_config_manager import DynamicConfigManager

        config_mgr = DynamicConfigManager()

        # Setup competition bridge configuration
        bridge_config = {
            "websocket_port": 8080,
            "max_clients": 50,
            "telemetry_rate_hz": 10.0,
            "emergency_stop_timeout": 5.0,
            "log_level": "INFO",
        }

        config_mgr.register_node("competition_bridge", bridge_config)

        # Test configuration updates (simulating runtime config changes)
        updates = [
            {
                "node_name": "competition_bridge",
                "parameter_name": "telemetry_rate_hz",
                "new_value": 20.0,
            },
            {
                "node_name": "competition_bridge",
                "parameter_name": "max_clients",
                "new_value": 100,
            },
        ]

        success = config_mgr.update_multiple_configs(updates)
        self.assertTrue(success, "Configuration updates should succeed")

        # Verify updated configuration
        current_config = config_mgr.get_node_config("competition_bridge")
        self.assertEqual(
            current_config["telemetry_rate_hz"],
            20.0,
            "Telemetry rate should be updated",
        )
        self.assertEqual(
            current_config["max_clients"], 100, "Max clients should be updated"
        )

        # Test configuration history
        history = config_mgr.get_config_history()
        self.assertGreater(len(history), 0, "Should have configuration history")

        print("  [PASS] Competition Bridge + Dynamic Config integration works")

    def test_competition_bridge_websocket_integration(self):
        """Test Competition Bridge integration with WebSocket redundancy."""
        print(
            "[NETWORK] Testing Competition Bridge + WebSocket Redundancy Integration..."
        )

        # Create WebSocket manager
        from bridges.websocket_redundancy_manager import (
            EndpointPriority,
            WebSocketEndpoint,
            WebSocketRedundancyManager,
        )

        ws_mgr = WebSocketRedundancyManager()
        ws_mgr.start_redundancy_system()

        # Setup competition bridge endpoints
        primary_endpoint = WebSocketEndpoint(
            "competition_primary", 8080, EndpointPriority.PRIMARY
        )
        secondary_endpoint = WebSocketEndpoint(
            "competition_secondary", 8081, EndpointPriority.SECONDARY
        )

        # Initialize endpoints properly
        for ep in [primary_endpoint, secondary_endpoint]:
            ep.is_running = True
            ep.response_time = 0.05
            ep.last_health_check = time.time()
            ws_mgr.add_endpoint(ep)

        ws_mgr._check_endpoint_health()

        # Verify endpoints are set up
        status = ws_mgr.get_system_status()
        self.assertEqual(len(status["endpoints"]), 2, "Should have two endpoints")

        # Test primary endpoint health
        primary_status = status["endpoints"]["competition_primary"]
        self.assertEqual(
            primary_status["health"], "healthy", "Primary endpoint should be healthy"
        )
        self.assertTrue(
            primary_status["is_healthy"], "Primary endpoint should be marked healthy"
        )

        # Test load balancing
        primary_endpoint.clients = [Mock()] * 48  # 96% load (ensures DEGRADED status)
        ws_mgr._check_endpoint_health()

        updated_status = ws_mgr.get_system_status()
        primary_load_status = updated_status["endpoints"]["competition_primary"]
        self.assertEqual(
            primary_load_status["health"],
            "degraded",
            "Primary should be degraded under high load",
        )

        print("  [PASS] Competition Bridge + WebSocket Redundancy integration works")
        ws_mgr.stop_redundancy_system()

    def test_competition_bridge_recovery_integration(self):
        """Test Competition Bridge integration with recovery coordination."""
        print(
            "[TOOL] Testing Competition Bridge + Recovery Coordination Integration..."
        )

        # Create recovery coordinator
        from bridges.websocket_redundancy_manager import WebSocketRedundancyManager
        from core.dds_domain_redundancy_manager import DDSDomainRedundancyManager
        from core.dynamic_config_manager import DynamicConfigManager
        from core.recovery_coordinator import RecoveryCoordinator
        from core.state_synchronization_manager import Distributedget_state_manager()

        # Create all system managers
        recovery_coord = RecoveryCoordinator()
        state_mgr = Distributedget_state_manager()."recovery_test")
        dds_mgr = DDSDomainRedundancyManager()
        config_mgr = DynamicConfigManager()
        ws_mgr = WebSocketRedundancyManager()

        # Start systems
        state_mgr.start()
        dds_mgr.start()
        ws_mgr.start_redundancy_system()

        # Register nodes for health checks
        state_mgr.register_node("recovery_test")
        state_mgr._trigger_election()

        dds_mgr.register_node("recovery_test", "echo test")

        # Add healthy WebSocket endpoint
        from bridges.websocket_redundancy_manager import (
            EndpointPriority,
            WebSocketEndpoint,
        )

        ws_endpoint = WebSocketEndpoint("recovery_test", 8080, EndpointPriority.PRIMARY)
        ws_endpoint.is_running = True
        ws_endpoint.response_time = 0.05
        ws_endpoint.last_health_check = time.time()
        ws_mgr.add_endpoint(ws_endpoint)
        ws_mgr._check_endpoint_health()

        # Register systems with recovery coordinator
        recovery_coord.register_system_manager("state", state_mgr)
        recovery_coord.register_system_manager("dds", dds_mgr)
        recovery_coord.register_system_manager("config", config_mgr)
        recovery_coord.register_system_manager("websocket", ws_mgr)

        # Test recovery coordination
        success = recovery_coord.initiate_recovery("Competition system recovery test")

        # Wait for completion
        timeout = 10
        start_time = time.time()
        while recovery_coord.recovery_active and (time.time() - start_time) < timeout:
            time.sleep(0.1)

        # Verify recovery completed successfully
        final_status = recovery_coord.get_recovery_status()
        recovery_success = (
            not recovery_coord.recovery_active
            and final_status["current_phase"] == "complete"
        )

        self.assertTrue(recovery_success, "Recovery should complete successfully")
        self.assertEqual(
            final_status["current_phase"],
            "complete",
            "Recovery phase should be complete",
        )

        print("  [PASS] Competition Bridge + Recovery Coordination integration works")

        # Cleanup
        state_mgr.stop()
        dds_mgr.stop()
        ws_mgr.stop_redundancy_system()

    def test_end_to_end_competition_scenario(self):
        """Test end-to-end competition scenario with all systems integrated."""
        print("[FLAG] Testing End-to-End Competition Scenario...")

        # Setup all systems
        systems = {}

        # State synchronization
        systems["state"] = Distributedget_state_manager()."competition_scenario")
        systems["state"].start()
        systems["state"].register_node("competition_scenario")
        systems["state"]._trigger_election()

        # DDS domain redundancy
        systems["dds"] = DDSDomainRedundancyManager()
        systems["dds"].start()
        systems["dds"].register_node(
            "competition_scenario", "ros2 run competition competition_node"
        )

        # Dynamic configuration
        systems["config"] = DynamicConfigManager()
        systems["config"].register_node(
            "competition_scenario",
            {
                "competition_mode": "active",
                "safety_enabled": True,
                "telemetry_enabled": True,
            },
        )

        # WebSocket redundancy
        systems["websocket"] = WebSocketRedundancyManager()
        systems["websocket"].start_redundancy_system()

        from bridges.websocket_redundancy_manager import (
            EndpointPriority,
            WebSocketEndpoint,
        )

        ws_endpoint = WebSocketEndpoint(
            "competition_ws", 8080, EndpointPriority.PRIMARY
        )
        ws_endpoint.is_running = True
        ws_endpoint.response_time = 0.05
        ws_endpoint.last_health_check = time.time()
        systems["websocket"].add_endpoint(ws_endpoint)
        systems["websocket"]._check_endpoint_health()

        # Recovery coordination
        systems["recovery"] = RecoveryCoordinator()
        for name, system in systems.items():
            if name != "recovery":
                systems["recovery"].register_system_manager(name, system)

        # Simulate competition telemetry flow
        print("  [GRAPH] Simulating competition telemetry flow...")

        telemetry_updates = [
            ("battery_level", "95"),
            ("gps_signal", "strong"),
            ("mission_status", "waypoint_navigation"),
            ("system_health", "nominal"),
            ("competition_time_remaining", "3600"),  # 1 hour
        ]

        for key, value in telemetry_updates:
            systems["state"].update_state(f"telemetry_{key}", value)

        # Verify telemetry propagation
        for key, expected_value in telemetry_updates:
            actual_value = systems["state"].get_state(f"telemetry_{key}")
            self.assertEqual(
                actual_value,
                expected_value,
                f"Telemetry {key} should be propagated correctly",
            )

        # Test configuration updates during competition
        config_updates = [
            {
                "node_name": "competition_scenario",
                "parameter_name": "competition_mode",
                "new_value": "emergency",
            },
            {
                "node_name": "competition_scenario",
                "parameter_name": "safety_enabled",
                "new_value": False,
            },
        ]

        success = systems["config"].update_multiple_configs(config_updates)
        self.assertTrue(success, "Competition configuration updates should succeed")

        # Test system health monitoring
        state_status = systems["state"].get_system_status()
        dds_status = systems["dds"].get_system_status()
        ws_status = systems["websocket"].get_system_status()

        self.assertEqual(
            state_status["role"], "master", "State system should be healthy"
        )
        self.assertIsNotNone(
            dds_status["current_domain"], "DDS system should be healthy"
        )
        self.assertEqual(
            len(ws_status["endpoints"]), 1, "WebSocket system should have endpoints"
        )

        # Test recovery coordination
        recovery_success = systems["recovery"].initiate_recovery(
            "End-to-end competition test"
        )

        timeout = 10
        start_time = time.time()
        while (
            systems["recovery"].recovery_active and (time.time() - start_time) < timeout
        ):
            time.sleep(0.1)

        recovery_status = systems["recovery"].get_recovery_status()
        self.assertEqual(
            recovery_status["current_phase"],
            "complete",
            "Recovery should complete for healthy systems",
        )

        print("  [PASS] End-to-end competition scenario integration works")

        # Cleanup all systems
        for name, system in systems.items():
            if name == "state":
                system.stop()
            elif name == "dds":
                system.stop()
            elif name == "websocket":
                system.stop_redundancy_system()


if __name__ == "__main__":
    # Run tests with verbose output
    unittest.main(verbosity=2)
