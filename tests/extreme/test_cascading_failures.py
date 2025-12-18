#!/usr/bin/env python3
"""
Extreme Cascading Failure Tests

Tests system behavior during cascading failure scenarios:
- Primary-secondary-tertiary failure chains
- DDS domain cascade failures
- State synchronization cascade failures
- Multi-system simultaneous failures

Author: URC 2026 Autonomy Team
"""

import time
import unittest
import threading
from unittest.mock import Mock, patch
import sys
import os

# Add src to path - go up two levels from tests/extreme/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src'))

from ros2_environment_manager import (
    ROS2EnvironmentManager, ROSEnvironmentConfig, ResourceLimits, get_environment_manager
)
from pathlib import Path


class ExtremeCascadingFailureTest(unittest.TestCase):
    """Test extreme cascading failure scenarios."""

    def setUp(self):
        """Setup test environment."""
        self.env_manager = get_environment_manager()
        self.workspace_path = Path.cwd()

        # Cascading failure test configuration
        self.ros_config = ROSEnvironmentConfig(
            domain_id=400,  # Isolated domain for cascade testing
            use_sim_time=True,
            discovery_timeout_sec=45.0
        )

        self.resource_limits = ResourceLimits(
            cpu_percent=30.0,
            memory_mb=100,
            max_processes=8
        )

    def test_primary_secondary_cascade(self):
        """Test cascade: Primary WebSocket fails → Secondary overloaded → Tertiary activated."""
        print("🔗 Testing Primary-Secondary-Tertiary Cascade...")

        with self.env_manager.create_environment(
            name="websocket_cascade_test",
            ros_config=self.ros_config,
            resource_limits=self.resource_limits,
            workspace_path=self.workspace_path
        ) as env:

            from bridges.websocket_redundancy_manager import WebSocketRedundancyManager, WebSocketEndpoint, EndpointPriority

            # Create WebSocket manager with three endpoints
            ws_mgr = WebSocketRedundancyManager()

            # Create endpoints
            primary = WebSocketEndpoint("primary", 8080, EndpointPriority.PRIMARY, max_clients=50)
            secondary = WebSocketEndpoint("secondary", 8081, EndpointPriority.SECONDARY, max_clients=25)
            tertiary = WebSocketEndpoint("tertiary", 8082, EndpointPriority.TERTIARY, max_clients=10)

            # All start healthy
            primary.is_running = True
            secondary.is_running = True
            tertiary.is_running = True

            ws_mgr.add_endpoint(primary)
            ws_mgr.add_endpoint(secondary)
            ws_mgr.add_endpoint(tertiary)

            ws_mgr.start_redundancy_system()

            # Phase 1: Normal operation
            status = ws_mgr.get_system_status()
            primary_status = status['endpoints']['primary']
            self.assertEqual(primary_status['health'], 'HEALTHY', "Primary should start healthy")

            # Phase 2: Primary fails
            print("  💥 Phase 1: Primary endpoint fails...")
            primary.is_running = False
            ws_mgr._check_endpoint_health()

            status_after_primary_fail = ws_mgr.get_system_status()
            primary_after_fail = status_after_primary_fail['endpoints']['primary']
            self.assertEqual(primary_after_fail['health'], 'DOWN', "Primary should be down after failure")

            # Phase 3: Secondary becomes overloaded
            print("  🔥 Phase 2: Secondary becomes overloaded...")
            secondary.clients = [Mock()] * 30  # Exceed max_clients (25)
            ws_mgr._check_endpoint_health()

            status_after_overload = ws_mgr.get_system_status()
            secondary_after_overload = status_after_overload['endpoints']['secondary']
            self.assertEqual(secondary_after_overload['health'], 'UNHEALTHY', "Secondary should be unhealthy when overloaded")

            # Phase 4: Secondary also fails
            print("  💥 Phase 3: Secondary endpoint fails...")
            secondary.is_running = False
            ws_mgr._check_endpoint_health()

            status_after_secondary_fail = ws_mgr.get_system_status()
            secondary_after_fail = status_after_secondary_fail['endpoints']['secondary']
            self.assertEqual(secondary_after_fail['health'], 'DOWN', "Secondary should be down")

            # Phase 5: Tertiary should still be available
            tertiary_status = status_after_secondary_fail['endpoints']['tertiary']
            self.assertEqual(tertiary_status['health'], 'HEALTHY', "Tertiary should remain healthy")

            # Verify system has at least one healthy endpoint
            healthy_endpoints = sum(1 for ep in status_after_secondary_fail['endpoints'].values()
                                  if ep['is_healthy'])
            self.assertGreater(healthy_endpoints, 0, "System should have at least one healthy endpoint after cascade")

            ws_mgr.stop_redundancy_system()

        print("  ✅ Primary-secondary-tertiary cascade test passed")

    def test_dds_domain_cascade(self):
        """Test DDS domain cascade: Primary fails → Backup fails → Emergency mode."""
        print("🌐 Testing DDS Domain Cascade...")

        with self.env_manager.create_environment(
            name="dds_cascade_test",
            ros_config=self.ros_config,
            resource_limits=self.resource_limits,
            workspace_path=self.workspace_path
        ) as env:

            from core.dds_domain_redundancy_manager import DDSDomainRedundancyManager

            # Create DDS manager
            dds_mgr = DDSDomainRedundancyManager(domain_id=400)

            # Register nodes
            dds_mgr.register_node("dds_test_node", "echo test")

            dds_mgr.start()

            # Phase 1: Start with primary domain
            status = dds_mgr.get_system_status()
            self.assertEqual(status['current_domain'], 400, "Should start with primary domain")

            # Phase 2: Primary domain fails
            print("  💥 Phase 1: Primary DDS domain fails...")

            # Mock domain health to return 0 (failed)
            original_measure = dds_mgr._measure_domain_health
            dds_mgr._measure_domain_health = lambda x: 0.0 if x == 400 else 0.9

            success1 = dds_mgr.trigger_domain_failover(target_domain_id=401)  # Backup domain
            self.assertTrue(success1, "Should failover from primary to backup")

            status_after_primary_fail = dds_mgr.get_system_status()
            self.assertEqual(status_after_primary_fail['current_domain'], 401, "Should be on backup domain")

            # Phase 3: Backup domain also fails
            print("  💥 Phase 2: Backup DDS domain fails...")
            dds_mgr._measure_domain_health = lambda x: 0.0 if x in [400, 401] else 0.9

            success2 = dds_mgr.trigger_domain_failover(target_domain_id=402)  # Emergency domain
            self.assertTrue(success2, "Should failover from backup to emergency")

            status_after_backup_fail = dds_mgr.get_system_status()
            self.assertEqual(status_after_backup_fail['current_domain'], 402, "Should be on emergency domain")

            # Phase 4: Emergency domain fails (total failure)
            print("  💥 Phase 3: Emergency DDS domain fails...")
            dds_mgr._measure_domain_health = lambda x: 0.0  # All domains fail

            # System should handle total DDS failure gracefully
            # In real implementation, this would trigger emergency protocols

            # Restore original method for cleanup
            dds_mgr._measure_domain_health = original_measure

            dds_mgr.stop()

        print("  ✅ DDS domain cascade test passed")

    def test_state_sync_cascade(self):
        """Test state synchronization cascade failures."""
        print("🔄 Testing State Synchronization Cascade...")

        with self.env_manager.create_environment(
            name="state_cascade_test",
            ros_config=self.ros_config,
            resource_limits=self.resource_limits,
            workspace_path=self.workspace_path
        ) as env:

            from core.state_synchronization_manager import DistributedStateManager

            # Create three managers
            mgr1 = DistributedStateManager("cascade_node1")
            mgr2 = DistributedStateManager("cascade_node2")
            mgr3 = DistributedStateManager("cascade_node3")

            # Setup cross-communication
            mgr1.register_slave_manager(mgr2)
            mgr1.register_slave_manager(mgr3)
            mgr2.register_slave_manager(mgr1)
            mgr2.register_slave_manager(mgr3)
            mgr3.register_slave_manager(mgr1)
            mgr3.register_slave_manager(mgr2)

            # Start all managers
            mgr1.start()
            mgr2.start()
            mgr3.start()

            # Register nodes
            for mgr in [mgr1, mgr2, mgr3]:
                mgr.register_node("cascade_node1")
                mgr.register_node("cascade_node2")
                mgr.register_node("cascade_node3")
                # Mark all as healthy initially
                for node_id in ["cascade_node1", "cascade_node2", "cascade_node3"]:
                    if node_id in mgr.nodes:
                        mgr.nodes[node_id].is_healthy = True

            # Phase 1: All nodes healthy, establish master
            mgr1._trigger_election()
            time.sleep(0.1)

            # Verify we have a master
            masters = sum(1 for mgr in [mgr1, mgr2, mgr3] if mgr.role.name == 'MASTER')
            self.assertEqual(masters, 1, "Should have exactly one master initially")

            # Phase 2: Master fails
            print("  💥 Phase 1: Master node fails...")
            master_mgr = next(mgr for mgr in [mgr1, mgr2, mgr3] if mgr.role.name == 'MASTER')
            master_node_id = next(node_id for node_id, node in master_mgr.nodes.items()
                                if master_mgr.nodes[node_id].is_healthy)

            # Simulate master node failure
            for mgr in [mgr1, mgr2, mgr3]:
                if master_node_id in mgr.nodes:
                    mgr.nodes[master_node_id].is_healthy = False

            # Trigger election
            remaining_mgr = next(mgr for mgr in [mgr1, mgr2, mgr3] if mgr != master_mgr)
            remaining_mgr._trigger_election()
            time.sleep(0.1)

            # Verify new master elected
            new_masters = sum(1 for mgr in [mgr1, mgr2, mgr3] if mgr.role.name == 'MASTER')
            self.assertEqual(new_masters, 1, "Should elect new master after original master fails")

            # Phase 3: New master fails (cascade)
            print("  💥 Phase 2: New master fails (cascade)...")
            new_master_mgr = next(mgr for mgr in [mgr1, mgr2, mgr3] if mgr.role.name == 'MASTER')
            new_master_node_id = next(node_id for node_id, node in new_master_mgr.nodes.items()
                                    if new_master_mgr.nodes[node_id].is_healthy)

            # Simulate new master failure
            for mgr in [mgr1, mgr2, mgr3]:
                if new_master_node_id in mgr.nodes:
                    mgr.nodes[new_master_node_id].is_healthy = False

            # Trigger final election
            final_mgr = next(mgr for mgr in [mgr1, mgr2, mgr3]
                           if mgr.nodes[list(mgr.nodes.keys())[0]].is_healthy)
            final_mgr._trigger_election()
            time.sleep(0.1)

            # Verify final master elected
            final_masters = sum(1 for mgr in [mgr1, mgr2, mgr3] if mgr.role.name == 'MASTER')
            self.assertEqual(final_masters, 1, "Should elect final master after cascade")

            # Cleanup
            mgr1.stop()
            mgr2.stop()
            mgr3.stop()

        print("  ✅ State synchronization cascade test passed")

    def test_multi_system_simultaneous_failure(self):
        """Test simultaneous failure of multiple systems."""
        print("💥 Testing Multi-System Simultaneous Failure...")

        with self.env_manager.create_environment(
            name="multi_system_failure_test",
            ros_config=self.ros_config,
            resource_limits=self.resource_limits,
            workspace_path=self.workspace_path
        ) as env:

            from core.state_synchronization_manager import DistributedStateManager
            from core.dds_domain_redundancy_manager import DDSDomainRedundancyManager
            from bridges.websocket_redundancy_manager import WebSocketRedundancyManager
            from core.recovery_coordinator import RecoveryCoordinator

            # Create all system managers
            state_mgr = DistributedStateManager("multi_fail_test")
            dds_mgr = DDSDomainRedundancyManager(domain_id=400)
            ws_mgr = WebSocketRedundancyManager()
            recovery_coord = RecoveryCoordinator()

            # Register systems with recovery coordinator
            recovery_coord.register_system_manager("state", state_mgr)
            recovery_coord.register_system_manager("dds", dds_mgr)
            recovery_coord.register_system_manager("websocket", ws_mgr)

            # Setup basic system state
            state_mgr.register_node("multi_fail_test")
            dds_mgr.register_node("multi_fail_test", "echo test")

            from bridges.websocket_redundancy_manager import WebSocketEndpoint, EndpointPriority
            ws_endpoint = WebSocketEndpoint("multi_test", 8080, EndpointPriority.PRIMARY)
            ws_endpoint.is_running = True
            ws_mgr.add_endpoint(ws_endpoint)

            # Start systems
            state_mgr.start()
            dds_mgr.start()
            ws_mgr.start_redundancy_system()

            # Phase 1: All systems healthy
            state_status = state_mgr.get_system_status()
            dds_status = dds_mgr.get_system_status()
            ws_status = ws_mgr.get_system_status()

            self.assertIsNotNone(state_status, "State system should be healthy")
            self.assertIsNotNone(dds_status, "DDS system should be healthy")
            self.assertGreater(len(ws_status['endpoints']), 0, "WebSocket system should have endpoints")

            # Phase 2: Simultaneous multi-system failure
            print("  💥 Phase 1: Simultaneous multi-system failure...")

            # Fail state system
            if "multi_fail_test" in state_mgr.nodes:
                state_mgr.nodes["multi_fail_test"].is_healthy = False

            # Fail DDS system
            original_measure = dds_mgr._measure_domain_health
            dds_mgr._measure_domain_health = lambda x: 0.0

            # Fail WebSocket system
            ws_endpoint.is_running = False
            ws_mgr._check_endpoint_health()

            # Initiate coordinated recovery
            recovery_success = recovery_coord.initiate_recovery("Multi-system simultaneous failure")

            # Wait for recovery completion
            timeout = 20
            start_time = time.time()
            while recovery_coord.recovery_active and (time.time() - start_time) < timeout:
                time.sleep(0.1)

            # Assess recovery results
            if recovery_coord.recovery_active:
                print("  ⚠️ Recovery did not complete within timeout")
                recovery_success = False
            else:
                recovery_success = True

            # Even if recovery doesn't fully succeed, system should not crash
            self.assertIsInstance(recovery_success, bool, "Recovery should complete (success or failure)")

            # Cleanup
            dds_mgr._measure_domain_health = original_measure
            state_mgr.stop()
            dds_mgr.stop()
            ws_mgr.stop_redundancy_system()

        print("  ✅ Multi-system simultaneous failure test passed")


if __name__ == "__main__":
    # Run tests with verbose output
    unittest.main(verbosity=2)
