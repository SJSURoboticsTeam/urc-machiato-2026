#!/usr/bin/env python3
"""
Comprehensive Mission Execution Integration Tests

Tests complete mission execution workflows combining:
- Mission behaviors (waypoint navigation, object detection, follow-me)
- Mission executor (command processing, state management)
- Hardware interfaces (mocked sensors and actuators)
- Safety systems (emergency stops, monitoring)
- Navigation stack integration

Uses realistic simulation and comprehensive mocking.

Author: URC 2026 Test Suite
"""

import pytest
import time
import math
from unittest.mock import Mock, MagicMock, patch, AsyncMock
import sys
import os
from typing import Dict, List, Any, Tuple
import threading

# Add repo root so missions and src are importable (tests/integration/autonomy -> ../../..)
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "../../.."))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(PROJECT_ROOT, "src"))

from missions.mission_behaviors import (
    DeliveryMission,
    FollowMeMission,
    ObjectDetectionMission,
    SampleCollection,
    WaypointNavigation,
)

# Aliases for test compatibility
FollowMe = FollowMeMission
ObjectDetection = ObjectDetectionMission
from missions.mission_executor import MissionExecutor
from missions.waypoint_navigation_mission import WaypointNavigationMission


# Mock hardware interfaces
class MockHardwareInterface:
    """Comprehensive mock hardware interface for integration testing."""

    def __init__(self):
        self.position = [0.0, 0.0, 0.0]
        self.velocity = [0.0, 0.0, 0.0]
        self.gps_position = [40.0, -74.0]  # Default GPS coordinates
        self.imu_data = {"accel": [0.0, 0.0, 9.81], "gyro": [0.0, 0.0, 0.0]}
        self.lidar_points = []
        self.camera_images = []
        self.battery_level = 100.0
        self.motor_speeds = [0.0, 0.0, 0.0, 0.0]  # 4 wheels
        self.servo_positions = [90.0, 90.0]  # Pan/tilt servos
        self.samples_collected = []

    def get_current_position(self) -> List[float]:
        """Get current position from odometry/GPS fusion."""
        return self.position.copy()

    def get_gps_position(self) -> List[float]:
        """Get GPS position."""
        return self.gps_position.copy()

    def get_imu_data(self) -> Dict[str, List[float]]:
        """Get IMU sensor data."""
        return self.imu_data.copy()

    def get_lidar_scan(self) -> List[List[float]]:
        """Get LiDAR point cloud."""
        return self.lidar_points.copy()

    def get_camera_image(self) -> Any:
        """Get camera image."""
        return self.camera_images[-1] if self.camera_images else None

    def get_battery_level(self) -> float:
        """Get battery level percentage."""
        return self.battery_level

    def set_motor_speeds(self, speeds: List[float]) -> bool:
        """Set motor speeds for all wheels."""
        if len(speeds) != 4:
            return False
        self.motor_speeds = speeds.copy()
        return True

    def set_servo_positions(self, positions: List[float]) -> bool:
        """Set servo positions."""
        if len(positions) != 2:
            return False
        self.servo_positions = positions.copy()
        return True

    def collect_sample(self) -> bool:
        """Collect a sample."""
        self.samples_collected.append(
            {
                "timestamp": time.time(),
                "position": self.position.copy(),
                "gps": self.gps_position.copy(),
            }
        )
        return True

    def emergency_stop(self) -> bool:
        """Emergency stop all actuators."""
        self.motor_speeds = [0.0, 0.0, 0.0, 0.0]
        self.servo_positions = [90.0, 90.0]
        return True

    def simulate_movement(self, distance: float, direction: float = 0.0):
        """Simulate rover movement for testing."""
        # Update position based on distance and direction
        self.position[0] += distance * math.cos(math.radians(direction))
        self.position[1] += distance * math.sin(math.radians(direction))

    def simulate_sensor_noise(self, noise_level: float = 0.1):
        """Add realistic sensor noise."""
        import random

        for i in range(3):
            self.position[i] += random.uniform(-noise_level, noise_level)
        for i in range(2):
            self.gps_position[i] += random.uniform(-noise_level, noise_level)


class MissionIntegrationTestHarness:
    """Test harness for comprehensive mission integration testing."""

    def __init__(self):
        self.hardware = MockHardwareInterface()
        self.mission_executor = None
        self.waypoint_navigation = None
        self.object_detection = None
        self.follow_me = None
        self.delivery_mission = None
        self.sample_collection = None

        self.mock_node = self._create_mock_node()
        self._initialize_components()

    def _create_mock_node(self) -> Mock:
        """Create comprehensive mock ROS2 node."""
        node = Mock()
        node.create_subscription = Mock()
        node.create_publisher = Mock()
        node.create_client = Mock()
        node.create_timer = Mock()
        node.get_logger = Mock()
        node.get_logger.return_value.info = Mock()
        node.get_logger.return_value.warn = Mock()
        node.get_logger.return_value.error = Mock()
        node.get_logger.return_value.debug = Mock()
        node.get_clock = Mock()
        node.get_clock.return_value.now = Mock()
        node.get_clock.return_value.now.return_value.nanoseconds = 1000000000
        return node

    def _initialize_components(self):
        """Initialize all mission components with mocks."""
        with patch("mission_executor.Node", return_value=self.mock_node):
            self.mission_executor = MissionExecutor()

        self.waypoint_navigation = WaypointNavigation(self.mock_node)
        self.object_detection = ObjectDetection(self.mock_node)
        self.follow_me = FollowMe(self.mock_node)
        self.delivery_mission = DeliveryMission(self.mock_node)
        self.sample_collection = SampleCollection(self.mock_node)

    def execute_complete_mission(
        self, mission_config: Dict[str, Any]
    ) -> Dict[str, Any]:
        """
        Execute a complete mission from configuration.

        Args:
            mission_config: Mission configuration dictionary

        Returns:
            Mission execution results
        """
        results = {
            "success": False,
            "phases_completed": [],
            "errors": [],
            "metrics": {},
        }

        try:
            # Phase 1: Waypoint Navigation
            if "waypoints" in mission_config:
                waypoint_result = self._execute_waypoint_navigation(
                    mission_config["waypoints"]
                )
                results["phases_completed"].append("waypoint_navigation")
                if not waypoint_result["success"]:
                    results["errors"].append(
                        f"Waypoint navigation failed: {waypoint_result.get('error', 'Unknown error')}"
                    )
                    return results

            # Phase 2: Object Detection/Collection
            if mission_config.get("collect_samples", False):
                sample_result = self._execute_sample_collection()
                results["phases_completed"].append("sample_collection")
                if not sample_result["success"]:
                    results["errors"].append(
                        f"Sample collection failed: {sample_result.get('error', 'Unknown error')}"
                    )

            # Phase 3: Delivery (if applicable)
            if mission_config.get("delivery_required", False):
                delivery_result = self._execute_delivery_mission(
                    mission_config.get("delivery_config", {})
                )
                results["phases_completed"].append("delivery")
                if not delivery_result["success"]:
                    results["errors"].append(
                        f"Delivery failed: {delivery_result.get('error', 'Unknown error')}"
                    )

            # Phase 4: Return to Operator
            if mission_config.get("return_to_operator", False):
                return_result = self._execute_return_to_operator()
                results["phases_completed"].append("return_to_operator")
                if not return_result["success"]:
                    results["errors"].append(
                        f"Return to operator failed: {return_result.get('error', 'Unknown error')}"
                    )

            results["success"] = len(results["errors"]) == 0
            results["metrics"] = self._collect_metrics()

        except Exception as e:
            results["errors"].append(f"Mission execution failed: {str(e)}")

        return results

    def _execute_waypoint_navigation(
        self, waypoints: List[Dict[str, float]]
    ) -> Dict[str, Any]:
        """Execute waypoint navigation phase."""
        return self.waypoint_navigation.execute(waypoints, self.hardware)

    def _execute_sample_collection(self) -> Dict[str, Any]:
        """Execute sample collection phase."""
        # Simulate finding samples at current location
        self.hardware.lidar_points = [
            [1.0, 0.0, 0.0],  # Sample location
            [2.0, 1.0, 0.0],  # Another sample
        ]
        return self.sample_collection.execute(self.hardware)

    def _execute_delivery_mission(
        self, delivery_config: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Execute delivery mission phase."""
        pickup_location = delivery_config.get("pickup", {"x": 1.0, "y": 1.0})
        delivery_location = delivery_config.get("delivery", {"x": 5.0, "y": 5.0})
        return self.delivery_mission.execute(
            pickup_location, delivery_location, self.hardware
        )

    def _execute_return_to_operator(self) -> Dict[str, Any]:
        """Execute return to operator phase."""
        # Simulate operator at known GPS location
        operator_gps = [40.001, -74.001]  # Slightly offset from current position
        return self.waypoint_navigation.execute(
            [{"x": operator_gps[0], "y": operator_gps[1], "heading": 0.0}],
            self.hardware,
        )

    def _collect_metrics(self) -> Dict[str, Any]:
        """Collect mission execution metrics."""
        return {
            "total_samples_collected": len(self.hardware.samples_collected),
            "final_position": self.hardware.get_current_position(),
            "final_gps": self.hardware.get_gps_position(),
            "battery_remaining": self.hardware.get_battery_level(),
            "total_distance_traveled": 0.0,  # Would need to track this
            "execution_time": 0.0,  # Would need to track this
        }


class TestComprehensiveMissionIntegration:
    """Comprehensive integration tests for mission execution."""

    @pytest.fixture
    def test_harness(self):
        """Create mission integration test harness."""
        return MissionIntegrationTestHarness()

    @pytest.mark.integration
    def test_sample_collection_mission(self, test_harness):
        """Test complete sample collection mission."""
        mission_config = {
            "waypoints": [
                {"x": 2.0, "y": 0.0, "heading": 0.0},
                {"x": 4.0, "y": 2.0, "heading": 45.0},
                {"x": 6.0, "y": 4.0, "heading": 90.0},
            ],
            "collect_samples": True,
            "return_to_operator": True,
        }

        result = test_harness.execute_complete_mission(mission_config)

        # Mission should complete successfully
        assert result["success"] is True
        assert "waypoint_navigation" in result["phases_completed"]
        assert "sample_collection" in result["phases_completed"]
        assert "return_to_operator" in result["phases_completed"]
        assert len(result["errors"]) == 0

        # Should have collected samples
        assert result["metrics"]["total_samples_collected"] > 0

    @pytest.mark.integration
    def test_delivery_mission(self, test_harness):
        """Test complete delivery mission."""
        mission_config = {
            "waypoints": [{"x": 1.0, "y": 1.0, "heading": 0.0}],  # Pickup location
            "delivery_required": True,
            "delivery_config": {
                "pickup": {"x": 1.0, "y": 1.0},
                "delivery": {"x": 5.0, "y": 5.0},
            },
        }

        result = test_harness.execute_complete_mission(mission_config)

        assert result["success"] is True
        assert "waypoint_navigation" in result["phases_completed"]
        assert "delivery" in result["phases_completed"]

    @pytest.mark.integration
    def test_mission_failure_recovery(self, test_harness):
        """Test mission failure and recovery scenarios."""
        # Configure mission that will fail
        mission_config = {
            "waypoints": [
                {"x": 100.0, "y": 100.0, "heading": 0.0}  # Very far waypoint
            ],
            "collect_samples": True,
        }

        # Set timeout to very short for test
        test_harness.waypoint_navigation.timeout = 0.1

        result = test_harness.execute_complete_mission(mission_config)

        # Mission should fail due to timeout
        assert result["success"] is False
        assert len(result["errors"]) > 0
        assert "timeout" in str(result["errors"]).lower()

    @pytest.mark.integration
    def test_emergency_stop_integration(self, test_harness):
        """Test emergency stop integration across all systems."""
        # Start a mission
        mission_config = {"waypoints": [{"x": 1.0, "y": 0.0, "heading": 0.0}]}

        # Start mission execution in thread
        mission_thread = threading.Thread(
            target=test_harness.execute_complete_mission, args=(mission_config,)
        )
        mission_thread.start()

        # Wait a bit then trigger emergency stop
        time.sleep(0.1)
        emergency_result = test_harness.hardware.emergency_stop()

        # Wait for mission to complete
        mission_thread.join(timeout=1.0)

        # Emergency stop should succeed
        assert emergency_result is True

        # Motors should be stopped
        assert all(speed == 0.0 for speed in test_harness.hardware.motor_speeds)

    @pytest.mark.integration
    @pytest.mark.slow
    def test_sensor_fusion_integration(self, test_harness):
        """Test sensor fusion across multiple sensor types."""
        # Add realistic sensor noise
        test_harness.hardware.simulate_sensor_noise(0.05)

        # Execute navigation mission
        mission_config = {
            "waypoints": [
                {"x": 1.0, "y": 1.0, "heading": 45.0},
                {"x": 2.0, "y": 2.0, "heading": 45.0},
            ]
        }

        result = test_harness.execute_complete_mission(mission_config)

        # Should still succeed despite sensor noise
        assert result["success"] is True

        # Position should be reasonably close to final waypoint
        final_pos = result["metrics"]["final_position"]
        expected_pos = [2.0, 2.0, 0.0]

        distance_error = math.sqrt(
            (final_pos[0] - expected_pos[0]) ** 2
            + (final_pos[1] - expected_pos[1]) ** 2
        )

        # Allow for some error due to sensor noise and timing
        assert distance_error < 0.5, f"Position error too large: {distance_error}"


class TestSystemIntegrationWithMocks:
    """System-level integration tests using comprehensive mocking."""

    @pytest.fixture
    def full_system_mock(self):
        """Create full system mock for integration testing."""
        system = {
            "hardware": MockHardwareInterface(),
            "navigation": Mock(),
            "vision": Mock(),
            "control": Mock(),
            "safety": Mock(),
            "communication": Mock(),
        }

        # Configure navigation mock
        system["navigation"].navigate_to_waypoint = Mock(return_value=True)
        system["navigation"].get_navigation_status = Mock(
            return_value={
                "is_navigating": True,
                "current_position": [1.0, 1.0, 0.0],
                "distance_to_target": 0.5,
            }
        )

        # Configure vision mock
        system["vision"].detect_objects = Mock(
            return_value=[
                {"type": "sample", "position": [1.1, 1.1, 0.0], "confidence": 0.9}
            ]
        )
        system["vision"].detect_aruco_markers = Mock(return_value=[])

        # Configure control mock
        system["control"].set_velocity = Mock(return_value=True)
        system["control"].emergency_stop = Mock(return_value=True)

        # Configure safety mock
        system["safety"].get_safety_status = Mock(
            return_value={
                "system_safe": True,
                "active_triggers": [],
                "highest_severity": None,
            }
        )

        return system

    @pytest.mark.integration
    def test_cross_system_data_flow(self, full_system_mock):
        """Test data flow between all major systems."""
        system = full_system_mock

        # Simulate mission execution data flow
        # 1. Navigation requests position from hardware
        current_pos = system["hardware"].get_current_position()

        # 2. Vision processes sensor data
        objects = system["vision"].detect_objects()

        # 3. Navigation uses vision data for planning
        system["navigation"].navigate_to_waypoint.assert_not_called()  # Not yet called

        # 4. Control executes navigation commands
        system["control"].set_velocity.assert_not_called()  # Not yet called

        # 5. Safety monitors all systems
        safety_status = system["safety"].get_safety_status()
        assert safety_status["system_safe"] is True

        # Verify data consistency
        assert len(current_pos) == 3  # 3D position
        assert len(objects) > 0
        assert objects[0]["type"] == "sample"

    @pytest.mark.integration
    def test_error_propagation(self, full_system_mock):
        """Test error propagation through the system."""
        system = full_system_mock

        # Simulate hardware failure
        system["hardware"].get_current_position = Mock(
            side_effect=Exception("GPS failure")
        )

        # Navigation should handle the error
        try:
            pos = system["hardware"].get_current_position()
            assert False, "Should have raised exception"
        except Exception as e:
            assert "GPS failure" in str(e)

        # Safety system should detect the issue
        system["safety"].get_safety_status = Mock(
            return_value={
                "system_safe": False,
                "active_triggers": ["sensor_failure"],
                "highest_severity": "high",
            }
        )

        safety_status = system["safety"].get_safety_status()
        assert safety_status["system_safe"] is False
        assert "sensor_failure" in safety_status["active_triggers"]

    @pytest.mark.integration
    def test_performance_under_load(self, full_system_mock, performance_monitor):
        """Test system performance under simulated load."""
        system = full_system_mock
        performance_monitor.start()

        # Simulate high-frequency updates (typical mission load)
        iterations = 100

        for i in range(iterations):
            # Hardware position updates
            system["hardware"].simulate_movement(0.1, i * 3.6)  # Move in circle
            pos = system["hardware"].get_current_position()

            # Vision processing
            objects = system["vision"].detect_objects()

            # Navigation updates
            nav_status = system["navigation"].get_navigation_status()

            # Safety checks
            safety_status = system["safety"].get_safety_status()

        elapsed = performance_monitor.get_elapsed_time()

        # Should handle 100 iterations quickly (< 1 second)
        assert (
            elapsed < 1.0
        ), f"Performance too slow: {elapsed} seconds for {iterations} iterations"

        # All systems should remain functional
        assert len(pos) == 3
        assert len(objects) > 0
        assert nav_status["is_navigating"] is True
        assert safety_status["system_safe"] is True


if __name__ == "__main__":
    pytest.main([__file__, "-v", "-s"])
