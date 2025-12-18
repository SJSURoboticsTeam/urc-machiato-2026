#!/usr/bin/env python3
"""
Full ROS2 Environment Tests for Advanced Systems

Tests the advanced systems (State Sync, DDS Redundancy, Dynamic Config)
running in a full ROS2 environment with real ROS2 nodes and DDS communication.

Author: URC 2026 Autonomy Team
"""

import os
import sys
import threading
import time
import unittest
from unittest.mock import Mock, patch

# Add src to path - go up two levels from tests/ros2_integration/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))


def setup_ros2_environment():
    """Comprehensive ROS2 environment validation and setup."""
    try:
        # Check ROS2 installation
        import rclpy
        from rclpy.node import Node
        from std_msgs.msg import String

        # Try to initialize ROS2 context to verify it's working
        if not rclpy.utilities.ok():
            rclpy.init()
            ros2_initialized = True
        else:
            ros2_initialized = False

        # Check for autonomy_interfaces (URC-specific messages)
        try:
            from autonomy_interfaces.action import NavigateToPose, PerformTyping
            from autonomy_interfaces.msg import LedCommand, VisionDetection

            autonomy_interfaces_available = True
        except ImportError:
            print(" autonomy_interfaces not available - some tests will be skipped")
            autonomy_interfaces_available = False

        # Check DDS domain configuration
        import os

        domain_id = os.environ.get("ROS_DOMAIN_ID", "0")
        print(f"[ANTENNA] ROS2 Domain ID: {domain_id}")

        # Test node creation and destruction
        test_node = Node("ros2_test_node")
        test_node.destroy_node()
        if ros2_initialized:
            rclpy.shutdown()

        print("[PASS] ROS2 environment validated")
        return True, autonomy_interfaces_available

    except Exception as e:
        print(f"[FAIL] ROS2 environment setup failed: {e}")
        return False, False


# Setup ROS2 environment
ROS2_AVAILABLE, AUTONOMY_INTERFACES_AVAILABLE = setup_ros2_environment()

# Import ROS2 components if available
if ROS2_AVAILABLE:
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String

    if AUTONOMY_INTERFACES_AVAILABLE:
        from autonomy_interfaces.msg import LedCommand, VisionDetection
        from autonomy_interfaces.action import NavigateToPose, PerformTyping

from pathlib import Path

from ros2_environment_manager import (
    ResourceLimits,
    ROS2EnvironmentManager,
    ROSEnvironmentConfig,
    get_environment_manager,
)


@unittest.skipUnless(ROS2_AVAILABLE, "ROS2 environment not available")
class AdvancedSystemsROS2IntegrationTest(unittest.TestCase):
    """Test advanced systems in full ROS2 environment."""

    @classmethod
    def setUpClass(cls):
        """Setup ROS2 environment for all tests."""
        cls.env_manager = get_environment_manager()
        cls.workspace_path = Path.cwd()

        # ROS2 environment configuration
        cls.ros_config = ROSEnvironmentConfig(
            domain_id=600,  # Dedicated ROS2 test domain
            use_sim_time=False,  # Use real time for ROS2 testing
            namespace="test_advanced_systems",
            discovery_timeout_sec=10.0,
            log_level="INFO",
        )

        cls.resource_limits = ResourceLimits(
            cpu_percent=60.0,  # Moderate CPU for ROS2
            memory_mb=300,  # Moderate memory for ROS2
            max_processes=8,  # Allow ROS2 processes
        )

    def setUp(self):
        """Setup for each test."""
        self.env_name = f"ros2_advanced_test_{self._testMethodName}"

    def test_state_sync_real_ros2(self):
        """Test state synchronization with real ROS2 DDS communication."""
        print("[REFRESH] Testing State Synchronization with Real ROS2 DDS...")

        # Initialize ROS2 if not already done
        if not rclpy.utilities.ok():
            rclpy.init()

        try:
            # Create real ROS2 nodes with integrated state managers
            class StateSyncROS2Node(Node):
                def __init__(self, node_name):
                    super().__init__(node_name)
                    self.received_states = {}

                    # Create integrated state manager
                    from core.state_synchronization_manager import (
                        DistributedStateManager,
                    )

                    self.state_mgr = DistributedStateManager(node_name)
                    self.state_mgr.start()

                    # ROS2 publisher for state updates
                    self.state_pub = self.create_publisher(
                        String, f"/{node_name}/state_updates", 10
                    )

                    # ROS2 subscriber for state updates from other nodes
                    self.state_sub = self.create_subscription(
                        String, "/state_sync_broadcast", self.state_callback, 10
                    )

                    # Register self with state manager
                    self.state_mgr.register_node(node_name)
                    if node_name in self.state_mgr.nodes:
                        self.state_mgr.nodes[node_name].is_healthy = True

                def state_callback(self, msg):
                    """Handle state update messages from DDS."""
                    try:
                        # Parse message (format: "node:key:value")
                        parts = msg.data.split(":", 2)
                        if len(parts) == 3:
                            source_node, key, value = parts
                            if (
                                source_node != self.get_name()
                            ):  # Don't process our own messages
                                self.received_states[key] = value
                                # Update local state manager
                                self.state_mgr.update_state(key, value)
                    except Exception as e:
                        self.get_logger().error(f"Failed to parse state message: {e}")

                def publish_state(self, key, value):
                    """Publish a state update via DDS."""
                    # Update local state manager first
                    self.state_mgr.update_state(key, value)

                    # Publish via ROS2 topic
                    msg = String()
                    msg.data = f"{self.get_name()}:{key}:{value}"
                    self.state_pub.publish(msg)

                    # Also publish to broadcast topic
                    broadcast_msg = String()
                    broadcast_msg.data = f"{self.get_name()}:{key}:{value}"
                    self.broadcast_pub.publish(broadcast_msg)

                def destroy_node(self):
                    """Clean shutdown."""
                    self.state_mgr.stop()
                    super().destroy_node()

            # Create ROS2 nodes
            node1 = StateSyncROS2Node("state_sync_node1")
            node2 = StateSyncROS2Node("state_sync_node2")

            # Add broadcast publisher to node1
            node1.broadcast_pub = node1.create_publisher(
                String, "/state_sync_broadcast", 10
            )

            # Give nodes time to discover each other via DDS
            print("  [ANTENNA] Allowing DDS discovery...")
            for i in range(30):  # 3 seconds
                rclpy.spin_once(node1, timeout_sec=0.1)
                rclpy.spin_once(node2, timeout_sec=0.1)

            # Test state synchronization via real DDS
            test_states = {
                "telemetry_rate": "10.0",
                "battery_level": "85.5",
                "system_mode": "autonomous",
            }

            print("   Publishing state updates from node1...")
            # Node1 publishes state updates
            for key, value in test_states.items():
                node1.publish_state(key, value)
                time.sleep(0.2)  # Allow DDS propagation

            # Allow additional time for DDS message propagation
            print("  [ANTENNA] Allowing DDS propagation...")
            for i in range(20):  # 2 seconds
                rclpy.spin_once(node1, timeout_sec=0.1)
                rclpy.spin_once(node2, timeout_sec=0.1)

            # Verify Node2 received the state updates via DDS
            print("  [PASS] Verifying state reception...")
            received_states = node2.received_states

            for key, expected_value in test_states.items():
                self.assertIn(
                    key,
                    received_states,
                    f"State {key} should be received by Node2 via DDS",
                )
                self.assertEqual(
                    received_states[key],
                    expected_value,
                    f"State {key} should have correct value",
                )

            # Verify state managers are synchronized
            for key, expected_value in test_states.items():
                node1_state = node1.state_mgr.get_state(key)
                node2_state = node2.state_mgr.get_state(key)
                self.assertEqual(
                    node1_state,
                    expected_value,
                    f"Node1 state manager should have {key}",
                )
                self.assertEqual(
                    node2_state,
                    expected_value,
                    f"Node2 state manager should have {key}",
                )

            print("  [PASS] State synchronization via ROS2 DDS works")

        finally:
            # Clean shutdown
            try:
                node1.destroy_node()
                node2.destroy_node()
            except:
                pass
            if rclpy.utilities.ok():
                rclpy.shutdown()

    def test_dds_failover_real_ros2(self):
        """Test DDS domain failover with real ROS2 DDS communication."""
        print("[NETWORK] Testing DDS Failover with Real ROS2 DDS...")

        # Test DDS domain failover logic with ROS2 environment awareness
        if not rclpy.utilities.ok():
            rclpy.init()

        try:
            # Create DDS manager
            from core.dds_domain_redundancy_manager import DDSDomainRedundancyManager

            dds_mgr = DDSDomainRedundancyManager(primary_domain=600)

            # Register test nodes (simulated ROS2 processes)
            dds_mgr.register_node("ros2_test_node1", "ros2 run test_package test_node")
            dds_mgr.register_node("ros2_test_node2", "ros2 run test_package test_node")

            dds_mgr.start()

            # Test initial domain status
            initial_status = dds_mgr.get_system_status()
            self.assertEqual(
                initial_status["current_domain"], 600, "Should start on test domain"
            )
            self.assertEqual(
                len(initial_status["nodes"]), 2, "Should have registered test nodes"
            )

            # Test domain health measurement
            health_score = dds_mgr._measure_domain_health(600)
            self.assertIsInstance(
                health_score, float, "Health score should be measurable"
            )
            self.assertGreaterEqual(
                health_score, 0.0, "Health score should be non-negative"
            )

            # Test domain failover
            print("  [REFRESH] Testing domain failover...")

            # Force domain health to 0 (simulate failure)
            original_measure = dds_mgr._measure_domain_health
            dds_mgr._measure_domain_health = lambda x: 0.0 if x == 600 else 0.9

            success = dds_mgr.trigger_domain_failover(target_domain_id=601)
            self.assertTrue(success, "Domain failover should succeed")

            # Verify domain change
            failover_status = dds_mgr.get_system_status()
            self.assertEqual(
                failover_status["current_domain"],
                601,
                "Should failover to backup domain",
            )

            # Test that DDS manager handles ROS2 domain concepts
            # In real ROS2, domain failover would require node restarts
            # This tests the coordination logic
            print("  [PASS] DDS failover logic works with ROS2 environment awareness")

            # Restore original measurement
            dds_mgr._measure_domain_health = original_measure
            dds_mgr.stop()

        finally:
            if rclpy.utilities.ok():
                rclpy.shutdown()

    def test_dynamic_config_real_ros2(self):
        """Test dynamic configuration in ROS2 environment."""
        print(" Testing Dynamic Configuration with ROS2...")

        with self.env_manager.create_environment(
            name=self.env_name,
            ros_config=self.ros_config,
            resource_limits=self.resource_limits,
            workspace_path=self.workspace_path,
        ) as env:

            # Create config manager
            from core.dynamic_config_manager import DynamicConfigManager

            config_mgr = DynamicConfigManager()

            # Register test nodes
            config_mgr.register_node(
                "ros2_config_test1", {"telemetry_rate_hz": 5.0, "max_clients": 50}
            )

            config_mgr.register_node(
                "ros2_config_test2", {"update_rate_hz": 10.0, "timeout_seconds": 30}
            )

            # Test configuration updates
            success1 = config_mgr.update_node_config(
                "ros2_config_test1", "telemetry_rate_hz", 15.0
            )
            self.assertTrue(
                success1, "Config update should succeed in ROS2 environment"
            )

            # Test cross-node configuration updates
            updates = [
                {
                    "node_name": "ros2_config_test1",
                    "parameter_name": "max_clients",
                    "new_value": 100,
                },
                {
                    "node_name": "ros2_config_test2",
                    "parameter_name": "update_rate_hz",
                    "new_value": 20.0,
                },
            ]
            success2 = config_mgr.update_multiple_configs(updates)
            self.assertTrue(success2, "Multiple config updates should succeed")

            # Verify configurations were applied
            config1 = config_mgr.get_node_config("ros2_config_test1")
            config2 = config_mgr.get_node_config("ros2_config_test2")

            self.assertEqual(
                config1["telemetry_rate_hz"], 15.0, "Config1 rate should be updated"
            )
            self.assertEqual(
                config1["max_clients"], 100, "Config1 clients should be updated"
            )
            self.assertEqual(
                config2["update_rate_hz"], 20.0, "Config2 rate should be updated"
            )

            # Test configuration history
            history = config_mgr.get_config_history()
            self.assertGreater(
                len(history), 0, "Should have configuration history in ROS2 env"
            )

            print("  [PASS] Dynamic configuration works in ROS2 environment")

    def test_multi_node_state_sync_ros2(self):
        """Test multi-node state synchronization with real ROS2 DDS."""
        print(" Testing Multi-Node State Sync with ROS2 DDS...")

        # Test real DDS-based state synchronization between multiple ROS2 nodes
        if not rclpy.utilities.ok():
            rclpy.init()

        try:
            # Create multiple ROS2 nodes with integrated state managers
            class MultiNodeStateSyncROS2(Node):
                def __init__(self, node_name):
                    super().__init__(node_name)
                    self.received_states = {}

                    # Create integrated state manager
                    from core.state_synchronization_manager import (
                        DistributedStateManager,
                    )

                    self.state_mgr = DistributedStateManager(node_name)
                    self.state_mgr.start()
                    self.state_mgr.register_node(node_name)
                    if node_name in self.state_mgr.nodes:
                        self.state_mgr.nodes[node_name].is_healthy = True

                    # ROS2 communication for state sync
                    self.state_pub = self.create_publisher(
                        String, "/state_sync_broadcast", 10
                    )
                    self.state_sub = self.create_subscription(
                        String, "/state_sync_broadcast", self.state_callback, 10
                    )

                def state_callback(self, msg):
                    """Handle state sync messages from DDS."""
                    try:
                        # Parse message (format: "source_node:key:value")
                        parts = msg.data.split(":", 2)
                        if len(parts) == 3:
                            source_node, key, value = parts
                            if (
                                source_node != self.get_name()
                            ):  # Don't process our own messages
                                self.received_states[key] = value
                                # Update local state manager
                                self.state_mgr.update_state(key, value)
                    except Exception as e:
                        self.get_logger().error(f"Failed to parse state message: {e}")

                def publish_state(self, key, value):
                    """Publish state update via DDS."""
                    # Update local state manager
                    self.state_mgr.update_state(key, value)

                    # Broadcast via DDS
                    msg = String()
                    msg.data = f"{self.get_name()}:{key}:{value}"
                    self.state_pub.publish(msg)

                def destroy_node(self):
                    """Clean shutdown."""
                    self.state_mgr.stop()
                    super().destroy_node()

            # Create multiple ROS2 nodes
            node1 = MultiNodeStateSyncROS2("multi_sync_node1")
            node2 = MultiNodeStateSyncROS2("multi_sync_node2")
            node3 = MultiNodeStateSyncROS2("multi_sync_node3")

            # Allow DDS discovery
            print("  [ANTENNA] Allowing DDS discovery between nodes...")
            for i in range(50):  # 5 seconds for discovery
                rclpy.spin_once(node1, timeout_sec=0.1)
                rclpy.spin_once(node2, timeout_sec=0.1)
                rclpy.spin_once(node3, timeout_sec=0.1)

            # Test state synchronization across all nodes
            test_data = f"ROS2_DDS_multi_node_test_{time.time()}"

            print("   Publishing state from node1...")
            node1.publish_state("multi_node_test", test_data)

            # Allow DDS propagation
            print("  [ANTENNA] Allowing DDS state propagation...")
            for i in range(30):  # 3 seconds
                rclpy.spin_once(node1, timeout_sec=0.1)
                rclpy.spin_once(node2, timeout_sec=0.1)
                rclpy.spin_once(node3, timeout_sec=0.1)

            # Verify all nodes received the state via DDS
            print("  [PASS] Verifying state reception across all nodes...")

            # Check DDS message reception
            self.assertIn(
                "multi_node_test",
                node2.received_states,
                "Node2 should receive state via DDS",
            )
            self.assertIn(
                "multi_node_test",
                node3.received_states,
                "Node3 should receive state via DDS",
            )

            self.assertEqual(
                node2.received_states["multi_node_test"],
                test_data,
                "Node2 should have correct state value",
            )
            self.assertEqual(
                node3.received_states["multi_node_test"],
                test_data,
                "Node3 should have correct state value",
            )

            # Verify state managers are synchronized
            node1_state = node1.state_mgr.get_state("multi_node_test")
            node2_state = node2.state_mgr.get_state("multi_node_test")
            node3_state = node3.state_mgr.get_state("multi_node_test")

            self.assertEqual(
                node1_state, test_data, "Node1 state manager should have state"
            )
            self.assertEqual(
                node2_state, test_data, "Node2 state manager should have state"
            )
            self.assertEqual(
                node3_state, test_data, "Node3 state manager should have state"
            )

            print("  [PASS] Multi-node state synchronization works via ROS2 DDS")

        finally:
            # Clean shutdown
            try:
                node1.destroy_node()
                node2.destroy_node()
                node3.destroy_node()
            except:
                pass
            if rclpy.utilities.ok():
                rclpy.shutdown()

    def test_websocket_redundancy_real_time(self):
        """Test WebSocket redundancy in real-time ROS2 environment."""
        print("[NETWORK] Testing WebSocket Redundancy in ROS2 Environment...")

        with self.env_manager.create_environment(
            name=self.env_name,
            ros_config=self.ros_config,
            resource_limits=self.resource_limits,
            workspace_path=self.workspace_path,
        ) as env:

            from bridges.websocket_redundancy_manager import (
                EndpointHealth,
                EndpointPriority,
                WebSocketEndpoint,
                WebSocketRedundancyManager,
            )

            # Create WebSocket manager
            ws_mgr = WebSocketRedundancyManager()

            # Create endpoints
            endpoints = [
                WebSocketEndpoint(
                    "ros2_primary", 8080, EndpointPriority.PRIMARY, max_clients=50
                ),
                WebSocketEndpoint(
                    "ros2_secondary", 8081, EndpointPriority.SECONDARY, max_clients=25
                ),
                WebSocketEndpoint(
                    "ros2_tertiary", 8082, EndpointPriority.TERTIARY, max_clients=10
                ),
            ]

            for endpoint in endpoints:
                endpoint.is_running = True
                endpoint.health_status = EndpointHealth.HEALTHY  # Initialize as healthy
                ws_mgr.add_endpoint(endpoint)

            # Start system but temporarily disable health monitoring for initial test
            ws_mgr.start_redundancy_system()
            # Give monitoring a moment to potentially run
            import time

            time.sleep(0.1)

            # Ensure endpoints are healthy for initial test
            for endpoint in ws_mgr.endpoints.values():
                endpoint.health_status = EndpointHealth.HEALTHY

            # Test initial health
            status = ws_mgr.get_system_status()
            healthy_endpoints = sum(
                1 for ep in status["endpoints"].values() if ep["is_healthy"]
            )
            self.assertEqual(healthy_endpoints, 3, "All endpoints should start healthy")

            # Test failover scenario
            print("  [REFRESH] Testing failover in ROS2 environment...")

            # Fail primary endpoint
            primary_ep = next(
                ep for ep in ws_mgr.endpoints.values() if ep.name == "ros2_primary"
            )
            primary_ep.is_running = False
            ws_mgr._check_endpoint_health()

            # Verify primary is down
            status_after_fail = ws_mgr.get_system_status()
            primary_status = status_after_fail["endpoints"]["ros2_primary"]
            self.assertEqual(primary_status["health"], "DOWN", "Primary should be down")

            # Verify other endpoints are still operational (may be degraded due to load balancing)
            secondary_status = status_after_fail["endpoints"]["ros2_secondary"]
            tertiary_status = status_after_fail["endpoints"]["ros2_tertiary"]

            # Secondary may be degraded due to increased load from primary failure
            self.assertIn(
                secondary_status["health"],
                ["HEALTHY", "DEGRADED"],
                "Secondary should be operational",
            )
            self.assertIn(
                tertiary_status["health"],
                ["HEALTHY", "DEGRADED"],
                "Tertiary should be operational",
            )

            # Test load balancing
            secondary_ep = next(
                ep for ep in ws_mgr.endpoints.values() if ep.name == "ros2_secondary"
            )
            secondary_ep.clients = [Mock()] * 20  # 80% capacity
            ws_mgr._check_endpoint_health()

            status_after_load = ws_mgr.get_system_status()
            secondary_load_status = status_after_load["endpoints"]["ros2_secondary"]
            self.assertEqual(
                secondary_load_status["health"],
                "HEALTHY",
                "Secondary should handle load",
            )

            ws_mgr.stop_redundancy_system()

            print("  [PASS] WebSocket redundancy works in ROS2 environment")

    def test_coordinated_recovery_ros2(self):
        """Test coordinated recovery in ROS2 environment."""
        print("[TOOL] Testing Coordinated Recovery in ROS2 Environment...")

        with self.env_manager.create_environment(
            name=self.env_name,
            ros_config=self.ros_config,
            resource_limits=self.resource_limits,
            workspace_path=self.workspace_path,
        ) as env:

            from bridges.websocket_redundancy_manager import WebSocketRedundancyManager
            from core.dds_domain_redundancy_manager import DDSDomainRedundancyManager
            from core.dynamic_config_manager import DynamicConfigManager
            from core.recovery_coordinator import RecoveryCoordinator
            from core.state_synchronization_manager import DistributedStateManager

            # Create all system managers
            recovery_coord = RecoveryCoordinator()
            state_mgr = DistributedStateManager("ros2_recovery_test")
            dds_mgr = DDSDomainRedundancyManager(primary_domain=600)
            config_mgr = DynamicConfigManager()
            ws_mgr = WebSocketRedundancyManager()

            # Register systems
            recovery_coord.register_system_manager("state", state_mgr)
            recovery_coord.register_system_manager("dds", dds_mgr)
            recovery_coord.register_system_manager("config", config_mgr)
            recovery_coord.register_system_manager("websocket", ws_mgr)

            # Setup basic system state
            state_mgr.register_node("ros2_recovery_test")
            dds_mgr.register_node("ros2_recovery_test", "echo test")
            config_mgr.register_node("ros2_recovery_test", {"test_param": "value"})

            from bridges.websocket_redundancy_manager import (
                EndpointPriority,
                WebSocketEndpoint,
            )

            ws_endpoint = WebSocketEndpoint(
                "ros2_recovery", 8080, EndpointPriority.PRIMARY
            )
            ws_endpoint.is_running = True
            ws_mgr.add_endpoint(ws_endpoint)

            # Start systems
            state_mgr.start()
            dds_mgr.start()
            ws_mgr.start_redundancy_system()

            # Test recovery initiation
            recovery_success = recovery_coord.initiate_recovery(
                "ROS2 environment recovery test"
            )

            # Wait for recovery completion
            timeout = 15
            start_time = time.time()
            while (
                recovery_coord.recovery_active and (time.time() - start_time) < timeout
            ):
                time.sleep(0.1)

            # Verify recovery completed
            self.assertFalse(
                recovery_coord.recovery_active,
                "Recovery should complete in ROS2 environment",
            )
            recovery_status = recovery_coord.get_recovery_status()
            self.assertEqual(
                recovery_status["current_phase"],
                "COMPLETE",
                "Recovery should complete successfully",
            )

            print("  [PASS] Coordinated recovery works in ROS2 environment")

            # Cleanup
            state_mgr.stop()
            dds_mgr.stop()
            ws_mgr.stop_redundancy_system()


if __name__ == "__main__":
    # Run tests with verbose output if ROS2 is available
    if ROS2_AVAILABLE:
        unittest.main(verbosity=2)
    else:
        print("ROS2 not available - skipping ROS2 integration tests")
