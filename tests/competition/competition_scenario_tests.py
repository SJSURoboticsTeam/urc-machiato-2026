#!/usr/bin/env python3
"""
Competition Scenario Tests - Competition Ready
Simple, focused tests for common competition scenarios.
"""

import sys
import threading
import time
import unittest

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool, String


class CompetitionScenarioTests(unittest.TestCase):
    """
    Simple competition scenario tests.

    Tests focus on:
    - Terrain traversal (simulated obstacles)
    - Communication dropout handling
    - Mission sequence execution
    - Time pressure scenarios
    """

    @classmethod
    def setUpClass(cls):
        """Set up ROS2 environment for scenario tests."""
        rclpy.init(args=[])
        cls.node = Node("competition_scenario_test")

        # Test configuration
        cls.scenario_timeout = 30.0  # seconds per scenario
        cls.comm_drop_duration = 10.0  # seconds

    @classmethod
    def tearDownClass(cls):
        """Clean up ROS2 environment."""
        cls.node.destroy_node()
        rclpy.shutdown()

    def setUp(self):
        """Reset test state."""
        self.scenario_start_time = time.time()
        self.events_received = []

    def test_terrain_traversal_scenario(self):
        """Test basic terrain traversal with obstacles."""
        self.get_logger().info("Testing terrain traversal scenario...")

        # Start mission to navigate through obstacles
        self._start_mission("terrain_navigation")

        # Wait for mission progress
        mission_completed = self._wait_for_mission_complete(
            timeout=self.scenario_timeout
        )

        # Verify system handled obstacles appropriately
        self.assertTrue(mission_completed, "Terrain traversal mission did not complete")
        self.assertTrue(
            self._check_emergency_not_triggered(),
            "Emergency stop triggered during terrain traversal",
        )

    def test_communication_dropout_scenario(self):
        """Test handling of communication loss during mission."""
        self.get_logger().info("Testing communication dropout scenario...")

        # Start a mission
        self._start_mission("waypoint_navigation")

        # Simulate communication loss
        self._simulate_comm_loss(duration=self.comm_drop_duration)

        # Verify system continues operating
        time.sleep(5)  # Allow system to adapt

        # Check that mission continues or safely pauses
        mission_status = self._get_mission_status()
        self.assertIn(
            mission_status,
            ["active", "paused", "completed"],
            f"Unexpected mission status during comm loss: {mission_status}",
        )

        # Verify no emergency stops triggered
        self.assertTrue(
            self._check_emergency_not_triggered(),
            "Emergency stop triggered during communication loss",
        )

    def test_multi_mission_sequence(self):
        """Test executing multiple missions in sequence."""
        self.get_logger().info("Testing multi-mission sequence...")

        missions = ["waypoint_nav", "object_search", "return_home"]
        total_start_time = time.time()

        for mission in missions:
            self.get_logger().info(f"Starting mission: {mission}")

            # Start mission
            self._start_mission(mission)

            # Wait for completion
            completed = self._wait_for_mission_complete(timeout=self.scenario_timeout)
            self.assertTrue(completed, f"Mission {mission} did not complete")

            # Brief pause between missions
            time.sleep(2)

        total_duration = time.time() - total_start_time
        self.assertLess(
            total_duration,
            self.scenario_timeout * 2,
            f"Multi-mission sequence took too long: {total_duration:.1f}s",
        )

    def test_time_pressure_scenario(self):
        """Test mission execution under time pressure."""
        self.get_logger().info("Testing time pressure scenario...")

        # Start mission with tight timeout
        tight_timeout = 15.0  # 15 seconds
        self._start_mission("time_critical_delivery")

        # Mission should complete within timeout
        completed = self._wait_for_mission_complete(timeout=tight_timeout)
        self.assertTrue(completed, "Time-critical mission did not complete in time")

    def test_emergency_recovery_scenario(self):
        """Test recovery from emergency situations."""
        self.get_logger().info("Testing emergency recovery scenario...")

        # Start normal mission
        self._start_mission("normal_operation")

        # Trigger soft emergency stop
        self._trigger_emergency_stop("SOFT_STOP", "Test emergency")

        # Verify system stops
        time.sleep(2)
        emergency_status = self._get_emergency_status()
        self.assertEqual(emergency_status, "SOFT_STOP", "Emergency stop not activated")

        # Test recovery
        self._reset_emergency_system()

        # Verify system recovers
        time.sleep(3)
        emergency_status = self._get_emergency_status()
        self.assertEqual(emergency_status, "normal", "Emergency system did not recover")

    # Helper methods

    def _start_mission(self, mission_type: str):
        """Start a test mission."""
        mission_data = {
            "command": "start",
            "config": {
                "name": f"Test {mission_type}",
                "type": mission_type,
                "waypoints": [[0, 0], [5, 0], [5, 5]],  # Simple triangle path
            },
        }

        import json

        msg = String()
        msg.data = json.dumps(mission_data)

        publisher = self.node.create_publisher(String, "/mission/commands", 10)
        publisher.publish(msg)
        self.node.destroy_publisher(publisher)

    def _wait_for_mission_complete(self, timeout: float = 30.0) -> bool:
        """Wait for mission completion."""
        mission_completed = False
        start_time = time.time()

        def status_callback(msg):
            nonlocal mission_completed
            import json

            try:
                data = json.loads(msg.data)
                if data.get("active") == False:  # Mission completed
                    mission_completed = True
            except:
                pass

        subscription = self.node.create_subscription(
            String, "/mission/status", status_callback, 10
        )

        while not mission_completed and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.node.destroy_subscription(subscription)
        return mission_completed

    def _simulate_comm_loss(self, duration: float):
        """Simulate communication loss."""
        # Publish communication loss event
        msg = Bool()
        msg.data = True  # True = communication lost

        publisher = self.node.create_publisher(Bool, "/communication/lost", 10)
        publisher.publish(msg)
        self.node.destroy_publisher(publisher)

        # Wait for duration
        time.sleep(duration)

        # Restore communication
        msg.data = False  # False = communication restored
        publisher = self.node.create_publisher(Bool, "/communication/lost", 10)
        publisher.publish(msg)
        self.node.destroy_publisher(publisher)

    def _get_mission_status(self) -> str:
        """Get current mission status."""
        status = "unknown"

        def callback(msg):
            nonlocal status
            import json

            try:
                data = json.loads(msg.data)
                status = "active" if data.get("active", False) else "idle"
            except:
                pass

        subscription = self.node.create_subscription(
            String, "/mission/status", callback, 10
        )
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.destroy_subscription(subscription)

        return status

    def _trigger_emergency_stop(self, level: str, reason: str):
        """Trigger emergency stop."""
        msg = String()
        msg.data = f"{level}:{reason}"

        publisher = self.node.create_publisher(String, "/emergency/trigger", 10)
        publisher.publish(msg)
        self.node.destroy_publisher(publisher)

    def _reset_emergency_system(self):
        """Reset emergency system."""
        try:
            from std_srvs.srv import Trigger

            client = self.node.create_client(Trigger, "/emergency/reset")

            if client.wait_for_service(timeout_sec=2.0):
                request = Trigger.Request()
                future = client.call_async(request)

                start_time = time.time()
                while not future.done() and (time.time() - start_time) < 5.0:
                    rclpy.spin_once(self.node, timeout_sec=0.1)

        except Exception as e:
            self.get_logger().error(f"Emergency reset failed: {e}")

    def _get_emergency_status(self) -> str:
        """Get emergency system status."""
        status = "unknown"

        def callback(msg):
            nonlocal status
            import json

            try:
                data = json.loads(msg.data)
                status = data.get("level", "unknown")
            except:
                pass

        subscription = self.node.create_subscription(
            String, "/emergency/status", callback, 10
        )
        rclpy.spin_once(self.node, timeout_sec=1.0)
        self.node.destroy_subscription(subscription)

        return status

    def _check_emergency_not_triggered(self) -> bool:
        """Check that emergency stops were not triggered."""
        status = self._get_emergency_status()
        return status == "normal"

    def get_logger(self):
        """Get logger for test output."""
        return self.node.get_logger()


def run_competition_scenarios():
    """Run competition scenario test suite."""
    print("🏁 Running Competition Scenario Tests...")
    print("=" * 50)

    # Load test suite
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromTestCase(CompetitionScenarioTests)

    # Run tests with verbose output
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    # Summary
    print("\n" + "=" * 50)
    if result.wasSuccessful():
        print("✅ COMPETITION SCENARIOS PASSED")
        print("System ready for typical competition challenges")
    else:
        print("❌ COMPETITION SCENARIOS FAILED")
        print(f"Failed tests: {len(result.failures)}")
        print(f"Errors: {len(result.errors)}")
        print("Address scenario failures before competition")

    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_competition_scenarios()
    sys.exit(0 if success else 1)
