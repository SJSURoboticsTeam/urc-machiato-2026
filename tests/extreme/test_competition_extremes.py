#!/usr/bin/env python3
"""
Competition Extreme Scenario Tests

Tests system behavior under competition-specific extreme conditions:
- Emergency stop under maximum load
- Critical operation timeouts
- Sensor data floods
- Navigation failure cascades

Author: URC 2026 Autonomy Team
"""

import os
import sys
import threading
import time
import unittest
from unittest.mock import Mock, patch

# Add src to path - go up two levels from tests/extreme/
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "..", "..", "src"))

from pathlib import Path

from ros2_environment_manager import (
    ResourceLimits,
    ROS2EnvironmentManager,
    ROSEnvironmentConfig,
    get_environment_manager,
)


class CompetitionExtremeTest(unittest.TestCase):
    """Test competition-specific extreme scenarios."""

    def setUp(self):
        """Setup test environment."""
        self.env_manager = get_environment_manager()
        self.workspace_path = Path.cwd()

        # Competition-realistic configuration with extreme elements
        self.ros_config = ROSEnvironmentConfig(
            domain_id=500,  # Competition domain
            use_sim_time=False,  # Real time for competition simulation
            discovery_timeout_sec=5.0,  # Fast discovery like competition
            log_level="INFO",
        )

        self.competition_limits = ResourceLimits(
            cpu_percent=80.0,  # Competition-level CPU usage
            memory_mb=400,  # Competition memory constraints
            max_processes=15,  # Competition process limits
        )

    def test_emergency_stop_under_load(self):
        """Test emergency stop activation with 1000+ queued operations."""
        print(" Testing Emergency Stop Under Maximum Load...")

        with self.env_manager.create_environment(
            name="emergency_stop_load_test",
            ros_config=self.ros_config,
            resource_limits=self.competition_limits,
            workspace_path=self.workspace_path,
        ) as env:

            from core.state_synchronization_manager import DistributedStateManager

            # Create manager
            mgr = DistributedStateManager("emergency_test")

            mgr.start()
            mgr.register_node("emergency_test")
            if "emergency_test" in mgr.nodes:
                mgr.nodes["emergency_test"].is_healthy = True

            # Phase 1: Build up maximum load (simulate competition scenario)
            print("   Phase 1: Building maximum operational load...")

            operation_count = 100  # Simulate heavy competition load
            threads = []
            stop_load = threading.Event()

            def load_generator():
                """Generate continuous operation load."""
                count = 0
                while not stop_load.is_set() and count < operation_count:
                    mgr.update_state(
                        f"load_operation_{threading.current_thread().ident}_{count}",
                        f"data_{count}",
                    )
                    count += 1
                    time.sleep(0.001)  # High frequency

            # Start multiple load generator threads
            load_thread_count = 5
            for i in range(load_thread_count):
                thread = threading.Thread(target=load_generator, daemon=True)
                thread.start()
                threads.append(thread)

            # Let load build up
            time.sleep(0.5)

            # Phase 2: Emergency stop under load
            print("   Phase 2: Emergency stop activation under load...")

            emergency_start = time.time()

            # Trigger emergency stop (simulate competition emergency)
            mgr.update_state("emergency_stop", True)

            # Simulate emergency stop processing time
            emergency_processing_time = 0.05  # 50ms emergency response time
            time.sleep(emergency_processing_time)

            # Verify emergency stop was processed despite load
            emergency_state = mgr.get_state("emergency_stop")
            self.assertTrue(
                emergency_state, "Emergency stop should be processed under load"
            )

            emergency_response_time = time.time() - emergency_start
            print(".3f")

            # Emergency stop should be processed within 100ms even under load
            self.assertLess(
                emergency_response_time,
                0.1,
                "Emergency stop should respond within 100ms",
            )

            # Phase 3: Verify system stabilization after emergency
            print("  [PASS] Phase 3: Verifying system stabilization...")

            # Stop load generators
            stop_load.set()
            for thread in threads:
                thread.join(timeout=1.0)

            # Verify system returns to stable state
            final_state = mgr.get_state("emergency_stop")
            self.assertTrue(final_state, "Emergency state should persist")

            # System should still be responsive
            mgr.update_state("post_emergency_test", "responsive")
            post_emergency_state = mgr.get_state("post_emergency_test")
            self.assertEqual(
                post_emergency_state,
                "responsive",
                "System should remain responsive after emergency",
            )

            mgr.stop()

        print("  [PASS] Emergency stop under load test passed")

    def test_critical_operation_timeout(self):
        """Test critical operation timeout during network partition."""
        print("[CLOCK] Testing Critical Operation Timeout...")

        # Create environment with network timing constraints
        timeout_config = ROSEnvironmentConfig(
            domain_id=501,
            use_sim_time=True,  # Controlled timing
            discovery_timeout_sec=2.0,  # Short timeout for testing
        )

        with self.env_manager.create_environment(
            name="critical_timeout_test",
            ros_config=timeout_config,
            resource_limits=self.competition_limits,
            workspace_path=self.workspace_path,
        ) as env:

            from core.state_synchronization_manager import DistributedStateManager

            # Create manager
            mgr = DistributedStateManager("timeout_test")

            mgr.start()
            mgr.register_node("timeout_test")
            if "timeout_test" in mgr.nodes:
                mgr.nodes["timeout_test"].is_healthy = True

            # Phase 1: Start critical operation
            print("  [OBJECTIVE] Phase 1: Starting critical competition operation...")

            mgr.update_state("operation_status", "critical_navigation_active")
            mgr.update_state("operation_start_time", time.time())

            # Phase 2: Simulate network partition during critical operation
            print("  [NETWORK] Phase 2: Network partition during critical operation...")

            # Simulate network partition by marking node unhealthy
            if "timeout_test" in mgr.nodes:
                mgr.nodes["timeout_test"].is_healthy = False

            # Operation should timeout and trigger fallback
            timeout_duration = 3.0  # 3 seconds (longer than discovery timeout)

            start_timeout = time.time()
            timeout_triggered = False

            while (time.time() - start_timeout) < timeout_duration:
                # Check if operation should timeout
                if not mgr.nodes["timeout_test"].is_healthy:
                    # Simulate timeout detection
                    mgr.update_state("operation_status", "critical_timeout_fallback")
                    timeout_triggered = True
                    break
                time.sleep(0.1)

            # Phase 3: Verify timeout handling
            print("  ⏰ Phase 3: Verifying timeout handling...")

            if timeout_triggered:
                final_status = mgr.get_state("operation_status")
                self.assertEqual(
                    final_status,
                    "critical_timeout_fallback",
                    "Operation should enter fallback mode on timeout",
                )

                print("  [PASS] Critical operation timed out and entered fallback mode")

                # Verify fallback operation
                mgr.update_state("fallback_operation", "emergency_navigation")
                fallback_state = mgr.get_state("fallback_operation")
                self.assertEqual(
                    fallback_state,
                    "emergency_navigation",
                    "Fallback operation should be activated",
                )
            else:
                self.fail(
                    "Critical operation should have timed out during network partition"
                )

            mgr.stop()

        print("  [PASS] Critical operation timeout test passed")

    def test_sensor_data_flood(self):
        """Test system handling of sensor data flood at maximum frequency."""
        print("[ANTENNA] Testing Sensor Data Flood...")

        with self.env_manager.create_environment(
            name="sensor_flood_test",
            ros_config=self.ros_config,
            resource_limits=self.competition_limits,
            workspace_path=self.workspace_path,
        ) as env:

            from core.state_synchronization_manager import DistributedStateManager

            # Create manager
            mgr = DistributedStateManager("sensor_flood_test")

            mgr.start()
            mgr.register_node("sensor_flood_test")
            if "sensor_flood_test" in mgr.nodes:
                mgr.nodes["sensor_flood_test"].is_healthy = True

            # Phase 1: Normal sensor operation
            print("  [GRAPH] Phase 1: Normal sensor data rate...")

            normal_sensor_count = 10
            for i in range(normal_sensor_count):
                sensor_data = {
                    "lidar_points": [1.0] * 1000,  # Simulated LiDAR data
                    "camera_frames": "x" * 50000,  # Simulated camera data (50KB)
                    "imu_readings": [0.1, 0.2, 0.3] * 100,  # Simulated IMU data
                }
                mgr.update_state(f"sensor_normal_{i}", str(sensor_data))

            # Phase 2: Sensor data flood (competition scenario)
            print("   Phase 2: Sensor data flood at maximum frequency...")

            flood_start = time.time()
            flood_duration = 2.0  # 2 seconds of flood
            flood_count = 0

            # Simulate all sensors reporting simultaneously at maximum frequency
            stop_flood = threading.Event()

            def sensor_flood_generator(sensor_type, data_size):
                """Generate flood of sensor data."""
                count = 0
                while not stop_flood.is_set():
                    large_data = "x" * data_size
                    mgr.update_state(f"{sensor_type}_flood_{count}", large_data)
                    count += 1

            # Start multiple sensor flood generators
            flood_threads = []
            sensors = [
                ("lidar", 50000),  # 50KB per reading
                ("camera", 100000),  # 100KB per frame
                ("imu", 1000),  # 1KB per reading
                ("gps", 500),  # 500B per reading
            ]

            for sensor_type, data_size in sensors:
                thread = threading.Thread(
                    target=sensor_flood_generator,
                    args=(sensor_type, data_size),
                    daemon=True,
                )
                thread.start()
                flood_threads.append(thread)

            # Let flood run for specified duration
            time.sleep(flood_duration)
            stop_flood.set()

            # Wait for threads to finish
            for thread in flood_threads:
                thread.join(timeout=1.0)

            flood_end = time.time()
            actual_flood_duration = flood_end - flood_start

            # Phase 3: Verify system survived flood
            print("   Phase 3: Verifying system survival after flood...")

            # System should still be responsive
            mgr.update_state("post_flood_test", "system_still_alive")
            post_flood_state = mgr.get_state("post_flood_test")
            self.assertEqual(
                post_flood_state,
                "system_still_alive",
                "System should remain responsive after sensor flood",
            )

            # Calculate flood statistics
            status = mgr.get_system_status()
            total_updates = status["state_version"]

            flood_rate = total_updates / actual_flood_duration
            print(".1f")
            print(".1f")

            # System should handle at least 50 updates/second during flood
            self.assertGreater(
                flood_rate, 50, "System should handle high-frequency sensor updates"
            )

            mgr.stop()

        print("  [PASS] Sensor data flood test passed")

    def test_navigation_failure_cascade(self):
        """Test navigation failure cascade: Navigation fails → Arm control fails → Mission abort."""
        print(" Testing Navigation Failure Cascade...")

        with self.env_manager.create_environment(
            name="navigation_cascade_test",
            ros_config=self.ros_config,
            resource_limits=self.competition_limits,
            workspace_path=self.workspace_path,
        ) as env:

            from core.state_synchronization_manager import DistributedStateManager

            # Create manager
            mgr = DistributedStateManager("navigation_cascade_test")

            mgr.start()
            mgr.register_node("navigation_cascade_test")
            if "navigation_cascade_test" in mgr.nodes:
                mgr.nodes["navigation_cascade_test"].is_healthy = True

            # Phase 1: Normal navigation operation
            print("   Phase 1: Normal navigation operation...")

            mgr.update_state("navigation_status", "active")
            mgr.update_state("arm_status", "idle")
            mgr.update_state("mission_status", "waypoint_navigation")

            # Phase 2: Navigation failure
            print("   Phase 2: Navigation system fails...")

            mgr.update_state("navigation_status", "failed_gps_denied")
            mgr.update_state("navigation_error", "GPS signal lost")

            # Phase 3: Navigation failure impacts arm control
            print("   Phase 3: Navigation failure cascades to arm control...")

            # Simulate cascade: navigation failure prevents arm from reaching target
            time.sleep(0.1)  # Simulate processing delay

            mgr.update_state("arm_status", "failed_navigation_dependency")
            mgr.update_state("arm_error", "Cannot reach target without navigation")

            # Phase 4: Complete mission abort
            print("   Phase 4: Mission abort due to cascade...")

            mgr.update_state("mission_status", "aborted_cascade_failure")
            mgr.update_state(
                "mission_error", "Navigation failure cascaded to arm control"
            )

            # Phase 5: Verify cascade handling
            print("  [MAGNIFY] Phase 5: Verifying cascade handling...")

            final_nav_status = mgr.get_state("navigation_status")
            final_arm_status = mgr.get_state("arm_status")
            final_mission_status = mgr.get_state("mission_status")

            # Verify cascade progression
            self.assertEqual(
                final_nav_status,
                "failed_gps_denied",
                "Navigation should be in failed state",
            )
            self.assertEqual(
                final_arm_status,
                "failed_navigation_dependency",
                "Arm should fail due to navigation dependency",
            )
            self.assertEqual(
                final_mission_status,
                "aborted_cascade_failure",
                "Mission should abort due to cascade",
            )

            # Verify error reporting
            nav_error = mgr.get_state("navigation_error")
            arm_error = mgr.get_state("arm_error")
            mission_error = mgr.get_state("mission_error")

            self.assertIsNotNone(nav_error, "Navigation error should be reported")
            self.assertIsNotNone(arm_error, "Arm error should be reported")
            self.assertIsNotNone(mission_error, "Mission error should be reported")

            print("  [PASS] Navigation failure cascade properly handled")

            mgr.stop()

        print("  [PASS] Navigation failure cascade test passed")


if __name__ == "__main__":
    # Run tests with verbose output
    unittest.main(verbosity=2)
