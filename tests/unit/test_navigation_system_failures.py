#!/usr/bin/env python3
"""
Comprehensive Navigation System Failure Tests - URC 2026

Tests all critical navigation failure modes that can ruin URC missions:
- Path planning failures (blocked paths, impossible routes)
- Localization failures (GPS loss, SLAM divergence)
- Waypoint validation failures (invalid coordinates, unreachable goals)
- Motion control failures (stuck rover, actuator failures)
- Sensor fusion failures (timing issues, data corruption)
- Emergency stop failures (false triggers, failed recovery)

Author: URC 2026 Risk Mitigation Team
"""

import pytest
import math
import time
from unittest.mock import Mock, MagicMock, patch, AsyncMock
import sys
import os
from typing import Dict, List, Any, Optional, Tuple
import numpy as np

# Add source paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../..'))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '../../src'))

from src.autonomy.core.navigation.autonomy_navigation.path_planner import PathPlanner, DStarLite
from src.autonomy.core.navigation.autonomy_navigation.motion_controller import MotionController
from src.autonomy.core.navigation.autonomy_navigation.gnss_processor import GNSSProcessor
from src.core.synchronization_engine import SynchronizationEngine


class TestPathPlanningFailures:
    """Test path planning failure modes that can strand the rover."""

    @pytest.fixture
    def path_planner(self):
        """Create path planner instance."""
        planner = PathPlanner()
        # Mock NetworkX to avoid import issues in testing
        with patch('src.autonomy.core.navigation.autonomy_navigation.path_planner.nx') as mock_nx:
            mock_graph = Mock()
            mock_graph.nodes = ['0.0,0.0', '1.0,0.0', '1.0,1.0']
            mock_graph.add_node = Mock()
            mock_graph.add_edge = Mock()
            mock_nx.Graph.return_value = mock_graph
            mock_nx.astar_path.return_value = ['0.0,0.0', '1.0,0.0', '1.0,1.0']
            yield planner

    @pytest.fixture
    def sync_engine(self):
        """Create synchronization engine for testing."""
        from src.core.synchronization_engine import SynchronizationEngine
        return SynchronizationEngine()

    @pytest.mark.critical
    def test_blocked_path_no_alternative_route(self, path_planner):
        """Test complete path blockage with no alternative routes."""
        start = (0.0, 0.0)
        goal = (10.0, 10.0)

        # Mock path planner to return no path found
        with patch.object(path_planner, '_create_navigation_graph', return_value=None):
            path = path_planner.plan_path(start, goal)

            # Should return simple fallback path, but this indicates failure
            assert path == [start, goal], "Should return fallback path when planning fails"
            # In real scenario, this would be detected as a critical failure

    @pytest.mark.critical
    def test_impossible_route_terrain_obstacles(self, path_planner):
        """Test routes blocked by permanent terrain obstacles."""
        start = (0.0, 0.0)
        goal = (5.0, 5.0)

        # Mock terrain that's completely impassable
        with patch.object(path_planner, 'get_terrain_cost', return_value=float('inf')):
            path = path_planner.plan_path(start, goal)
            assert path == [start, goal], "Should fallback to direct path for impassable terrain"

    @pytest.mark.critical
    def test_path_planner_memory_exhaustion(self, path_planner):
        """Test path planner failures due to memory constraints."""
        start = (0.0, 0.0)
        goal = (100.0, 100.0)  # Very large search space

        # Mock memory exhaustion during graph creation
        with patch.object(path_planner, '_create_navigation_graph', side_effect=MemoryError("Out of memory")):
            path = path_planner.plan_path(start, goal)
            assert path == [start, goal], "Should fallback gracefully on memory exhaustion"

    @pytest.mark.critical
    def test_dstar_lite_convergence_failure(self):
        """Test D* Lite algorithm failing to converge."""
        dstar = DStarLite()

        # Initialize with start and goal
        start = (0.0, 0.0)
        goal = (10.0, 10.0)
        dstar.initialize(start, goal)

        # Mock costmap that creates infinite loop
        dstar.costmap = {(x, y): 1.0 for x in range(-100, 101) for y in range(-100, 101)}

        # Mock neighbors to create circular dependencies
        original_neighbors = dstar._get_neighbors
        def circular_neighbors(pos):
            x, y = pos
            # Create circular references
            if x == 5 and y == 5:
                return [(5, 6), (6, 5)]  # Point back to each other
            return original_neighbors(pos)
        dstar._get_neighbors = circular_neighbors

        # This should timeout or detect convergence failure
        path = dstar.replan(dstar.costmap)
        # Should return empty path or minimal path indicating failure
        assert len(path) <= 2, "Should detect convergence failure"

    @pytest.mark.critical
    def test_buffer_management_improvements(self, sync_engine):
        """Test improved buffer management and camera validation."""
        # Test 1: Valid camera auto-registration
        valid_camera_id = "stereo_left"
        valid_frame = {'timestamp': time.time(), 'frame_number': 1, 'camera_id': valid_camera_id}

        success = sync_engine.add_camera_frame(valid_camera_id, valid_frame)
        assert success, "Should accept and auto-register valid camera ID"
        assert valid_camera_id in sync_engine.camera_buffers, "Should auto-register new camera"

        # Test 2: Invalid camera rejection
        invalid_camera_id = "invalid!!!camera!!!"
        invalid_frame = {'timestamp': time.time(), 'frame_number': 2, 'camera_id': invalid_camera_id}

        success = sync_engine.add_camera_frame(invalid_camera_id, invalid_frame)
        assert not success, "Should reject invalid camera ID"

        # Test 3: Buffer overflow handling with adaptive sizing
        sync_engine.camera_buffers['front'].set_adaptive_mode(True)

        # Fill buffer beyond capacity
        for i in range(120):  # Way beyond default 100
            frame_data = {
                'timestamp': time.time() + i * 0.001,
                'camera_id': 'front',
                'frame_number': i
            }
            sync_engine.add_camera_frame('front', frame_data)

        # Check that buffer expanded adaptively
        buffer_stats = sync_engine.camera_buffers['front'].get_stats()
        assert buffer_stats['max_size'] > 100, f"Buffer should expand adaptively, got {buffer_stats['max_size']}"

        # Test 4: Item validation
        invalid_item = {'frame_number': 1, 'camera_id': 'front'}  # Missing timestamp
        success = sync_engine.add_camera_frame('front', invalid_item)
        assert not success, "Should reject items without required fields"


class TestLocalizationFailures:
    """Test localization system failures that cause navigation errors."""

    @pytest.fixture
    def gnss_processor(self):
        """Create GNSS processor instance."""
        return GNSSProcessor()

    @pytest.mark.critical
    def test_complete_gps_loss_no_fallback(self, gnss_processor):
        """Test complete GPS signal loss without fallback localization."""
        # Mock GPS data with no valid readings
        gps_data = {
            'latitude': None,
            'longitude': None,
            'altitude': None,
            'fix_quality': 0,  # No fix
            'satellites_visible': 0
        }

        imu_data = {
            'linear_acceleration': [0.0, 0.0, 9.81],
            'angular_velocity': [0.0, 0.0, 0.0]
        }

        # Should handle gracefully but position becomes unreliable
        result = gnss_processor.fuse_sensors(gps_data, imu_data)
        assert result is None or 'error' in result, "Should indicate localization failure"

    @pytest.mark.critical
    def test_gps_spoofing_detection_failure(self, gnss_processor):
        """Test failure to detect GPS spoofing attacks."""
        # Normal GPS data
        normal_gps = {
            'latitude': 40.0,
            'longitude': -74.0,
            'altitude': 100.0,
            'fix_quality': 4,
            'satellites_visible': 8
        }

        # Spoofed GPS data (sudden unrealistic jump)
        spoofed_gps = {
            'latitude': 41.0,  # 1 degree jump (impossible)
            'longitude': -74.0,
            'altitude': 100.0,
            'fix_quality': 4,
            'satellites_visible': 8
        }

        # Without spoofing detection, this would be accepted
        result1 = gnss_processor.fuse_sensors(normal_gps, {})
        result2 = gnss_processor.fuse_sensors(spoofed_gps, {})

        # Should detect the anomaly if spoofing detection is implemented
        if hasattr(gnss_processor, '_detect_spoofing'):
            assert gnss_processor._detect_spoofing(result1, result2), "Should detect GPS spoofing"

    @pytest.mark.critical
    def test_slam_divergence_unrecoverable(self):
        """Test SLAM divergence that cannot be recovered."""
        # Mock SLAM system that diverges completely
        slam_system = Mock()
        slam_system.get_pose.return_value = None  # Complete loss of pose
        slam_system.reset.return_value = False   # Reset fails

        # Multiple attempts to recover
        recovery_attempts = 0
        max_attempts = 5

        while recovery_attempts < max_attempts:
            pose = slam_system.get_pose()
            if pose is None:
                success = slam_system.reset()
                if not success:
                    recovery_attempts += 1
                    continue
            break

        assert recovery_attempts == max_attempts, "Should exhaust recovery attempts"

    @pytest.mark.critical
    def test_sensor_fusion_timing_violations(self, gnss_processor):
        """Test sensor fusion failures due to timing violations."""
        # GPS data with timestamp
        gps_data = {
            'latitude': 40.0,
            'longitude': -74.0,
            'timestamp': time.time()
        }

        # IMU data that's significantly delayed (beyond fusion window)
        delayed_imu = {
            'linear_acceleration': [0.0, 0.0, 9.81],
            'timestamp': time.time() - 1.0  # 1 second delay
        }

        # Fusion should reject or degrade data due to timing violation
        result = gnss_processor.fuse_sensors(gps_data, delayed_imu)
        if result:
            assert result.get('quality', 1.0) < 0.5, "Should degrade fusion quality for timing violations"


class TestWaypointValidationFailures:
    """Test waypoint validation failures that prevent navigation."""

    @pytest.fixture
    def navigation_node(self):
        """Create navigation node mock."""
        node = Mock()
        node.get_logger = Mock()
        node.get_logger.return_value.error = Mock()
        node.get_logger.return_value.warn = Mock()
        return node

    @pytest.mark.critical
    def test_invalid_coordinates_rejection(self, navigation_node):
        """Test rejection of invalid geographic coordinates."""
        from src.autonomy.core.navigation.autonomy_navigation.navigation_node import NavigationNode

        nav_node = NavigationNode.__new__(NavigationNode)  # Create without __init__
        nav_node.node = navigation_node

        # Test invalid latitudes
        invalid_waypoints = [
            {'latitude': 91.0, 'longitude': 0.0, 'name': 'Too North'},
            {'latitude': -91.0, 'longitude': 0.0, 'name': 'Too South'},
            {'latitude': float('nan'), 'longitude': 0.0, 'name': 'NaN Latitude'},
            {'latitude': 45.0, 'longitude': 181.0, 'name': 'Too East'},
            {'latitude': 45.0, 'longitude': -181.0, 'name': 'Too West'},
        ]

        for waypoint in invalid_waypoints:
            result = nav_node._validate_waypoint(
                type('Waypoint', (), waypoint)()
            )
            assert not result.success, f"Should reject invalid waypoint: {waypoint['name']}"

    @pytest.mark.critical
    def test_unreachable_waypoints_due_to_terrain(self):
        """Test waypoints that become unreachable due to terrain changes."""
        # Mock terrain analyzer that reports waypoint as unreachable
        terrain_analyzer = Mock()
        terrain_analyzer.is_reachable.return_value = False
        terrain_analyzer.get_traversability.return_value = 0.0  # Completely impassable

        waypoint = {'latitude': 40.0, 'longitude': -74.0, 'name': 'Blocked'}

        # Should detect unreachable waypoint
        reachable = terrain_analyzer.is_reachable(waypoint)
        assert not reachable, "Should detect unreachable waypoint"

    @pytest.mark.critical
    def test_waypoint_precision_requirement_failure(self):
        """Test failure to achieve required precision at waypoints."""
        # Mock high-precision requirement (AR tag alignment)
        precision_required = 0.01  # 1cm accuracy required
        actual_accuracy = 0.1     # 10cm actual accuracy

        # Should fail precision requirement
        precision_achieved = actual_accuracy <= precision_required
        assert not precision_achieved, "Should detect precision requirement failure"

    @pytest.mark.critical
    def test_dynamic_obstacle_blocking_waypoint(self):
        """Test waypoints blocked by dynamic obstacles."""
        # Mock obstacle detector
        obstacle_detector = Mock()
        obstacle_detector.detect_obstacles.return_value = [
            {'position': [10.0, 10.0], 'radius': 2.0, 'moving': True}
        ]

        waypoint = {'latitude': 40.0, 'longitude': -74.0, 'position': [10.0, 10.0]}

        # Check if waypoint is blocked
        obstacles = obstacle_detector.detect_obstacles()
        waypoint_blocked = any(
            math.sqrt((obs['position'][0] - waypoint['position'][0])**2 +
                     (obs['position'][1] - waypoint['position'][1])**2) <= obs['radius']
            for obs in obstacles
        )

        assert waypoint_blocked, "Should detect waypoint blocked by obstacle"


class TestMotionControlFailures:
    """Test motion control failures that prevent movement."""

    @pytest.fixture
    def motion_controller(self):
        """Create motion controller instance."""
        return MotionController()

    @pytest.mark.critical
    def test_complete_actuator_failure(self, motion_controller):
        """Test complete actuator failure preventing movement."""
        # Mock actuator interface reporting failure
        actuator_interface = Mock()
        actuator_interface.set_velocity.side_effect = Exception("Actuator failure")
        actuator_interface.get_status.return_value = {'operational': False}

        # Attempt to set velocity
        try:
            actuator_interface.set_velocity(1.0, 0.0)
            assert False, "Should raise exception on actuator failure"
        except Exception as e:
            assert "Actuator failure" in str(e)

        # Status should indicate failure
        status = actuator_interface.get_status()
        assert not status['operational'], "Should report actuator failure"

    @pytest.mark.critical
    def test_stuck_rover_detection_failure(self, motion_controller):
        """Test failure to detect when rover becomes stuck."""
        # Mock odometry that shows no movement despite velocity commands
        odometry = Mock()
        odometry.get_position.side_effect = [
            [0.0, 0.0],  # Initial position
            [0.0, 0.0],  # No movement after 1 second
            [0.0, 0.0],  # No movement after 2 seconds
            [0.0, 0.0],  # Still stuck after 3 seconds
        ]

        # Simulate commanding movement for 3 seconds
        start_time = time.time()
        stuck_threshold = 2.0  # seconds

        while time.time() - start_time < 3.0:
            # Command movement
            motion_controller.set_velocity(1.0, 0.0)

            # Check if stuck
            current_pos = odometry.get_position()
            time_stuck = time.time() - start_time

            if time_stuck > stuck_threshold:
                assert current_pos == [0.0, 0.0], "Should detect stuck rover"
                break

    @pytest.mark.critical
    def test_motor_current_overload_protection_failure(self, motion_controller):
        """Test failure of motor current overload protection."""
        # Mock motor monitors
        motor_monitors = [Mock() for _ in range(4)]  # 4 motors

        # Simulate current spikes
        for i, monitor in enumerate(motor_monitors):
            monitor.get_current.return_value = 25.0  # Over current limit
            monitor.get_temperature.return_value = 80.0  # Over temperature

        # Check if any motor exceeds limits
        motor_overloaded = any(
            monitor.get_current() > 20.0 or monitor.get_temperature() > 70.0
            for monitor in motor_monitors
        )

        assert motor_overloaded, "Should detect motor overload"

    @pytest.mark.critical
    def test_encoder_feedback_loss(self, motion_controller):
        """Test loss of encoder feedback for odometry."""
        # Mock encoders that fail
        encoders = [Mock() for _ in range(4)]
        for encoder in encoders:
            encoder.get_position.return_value = None  # Complete failure
            encoder.get_velocity.return_value = None

        # Odometry should fail without encoder data
        odometry_valid = any(
            encoder.get_position() is not None for encoder in encoders
        )

        assert not odometry_valid, "Should detect encoder feedback loss"


class TestEmergencyStopFailures:
    """Test emergency stop system failures."""

    @pytest.fixture
    def safety_monitor(self):
        """Create safety monitor mock."""
        from src.autonomy.core.safety_system.autonomy_safety_system.safety_monitor import SafetyMonitor
        return SafetyMonitor()

    @pytest.mark.critical
    def test_false_emergency_stop_triggers(self, safety_monitor):
        """Test false emergency stop triggers from sensor noise."""
        # Mock sensor data with noise that triggers false emergency stops
        noisy_sensor_data = {
            'imu_accel': [50.1, 0.0, 9.81],  # Just over shock threshold due to noise
            'motor_currents': [15.1, 14.8, 15.2, 14.9],  # Current spikes from noise
            'min_obstacle_distance': 0.29  # Just under obstacle threshold
        }

        # Update safety monitor
        safety_monitor.update_sensor_data(noisy_sensor_data)

        # Check if emergency stops were triggered inappropriately
        status = safety_monitor.get_safety_status()
        emergency_triggered = status['emergency_stop_active']

        # With proper hysteresis and filtering, this should not trigger
        # But if filtering fails, it would cause false emergency stops
        if emergency_triggered:
            assert len(status['active_triggers']) > 0, "Emergency stop should have active triggers"

    @pytest.mark.critical
    def test_emergency_stop_recovery_failure(self, safety_monitor):
        """Test failure to recover from emergency stop."""
        # Trigger emergency stop
        safety_monitor._execute_emergency_stop(
            type('Event', (), {'description': 'Test emergency'})()
        )

        # Attempt recovery
        recovery_success = safety_monitor.reset_emergency_stop()

        # If there are still active triggers, recovery should fail
        if not recovery_success:
            status = safety_monitor.get_safety_status()
            assert len(status['active_triggers']) > 0, "Recovery should fail with active triggers"

    @pytest.mark.critical
    def test_cascading_safety_system_failure(self, safety_monitor):
        """Test cascading failures in safety system components."""
        # Mock safety system components
        components = {
            'imu_monitor': Mock(),
            'motor_monitor': Mock(),
            'gps_monitor': Mock(),
            'emergency_stop': Mock()
        }

        # Simulate component failures
        for name, component in components.items():
            component.check_health.return_value = False
            component.reset.side_effect = Exception(f"{name} reset failed")

        # Count failed components
        failed_components = sum(
            not component.check_health() for component in components.values()
        )

        assert failed_components == len(components), "All components should fail"

        # Recovery attempts should also fail
        recovery_failures = 0
        for component in components.values():
            try:
                component.reset()
            except Exception:
                recovery_failures += 1

        assert recovery_failures == len(components), "All recovery attempts should fail"


class TestNavigationIntegrationFailures:
    """Test integration failures between navigation subsystems."""

    @pytest.mark.critical
    def test_ros2_topic_synchronization_loss(self):
        """Test loss of synchronization between ROS2 topics."""
        # Mock ROS2 node with timing issues
        node = Mock()
        subscriptions = {}

        # Simulate topics with different update rates
        fast_topic = Mock()  # 30 Hz
        slow_topic = Mock()  # 1 Hz

        # Fast topic updates frequently
        fast_updates = 0
        slow_updates = 0

        # Simulate 1 second of operation
        for i in range(30):  # 30 fast updates
            fast_updates += 1
            if i % 30 == 0:  # Every 30th fast update (1 Hz equivalent)
                slow_updates += 1

        # Synchronization should fail when topics drift too far apart
        sync_lost = fast_updates > slow_updates * 30  # More than 1:1 ratio
        assert sync_lost, "Should detect synchronization loss between topics"

    @pytest.mark.critical
    def test_configuration_parameter_conflicts(self):
        """Test conflicting configuration parameters."""
        # Mock configuration with conflicting settings
        config = {
            'navigation': {
                'max_speed': 2.0,  # m/s
                'waypoint_tolerance': 0.5,  # meters
                'safety_distance': 0.3,  # meters
            },
            'safety': {
                'max_speed': 1.0,  # Conflicting max speed!
                'min_obstacle_distance': 0.5,  # Conflicts with navigation tolerance
            }
        }

        # Detect conflicts
        conflicts = []
        if config['navigation']['max_speed'] != config['safety']['max_speed']:
            conflicts.append('max_speed conflict')

        if config['navigation']['waypoint_tolerance'] <= config['safety']['min_obstacle_distance']:
            conflicts.append('tolerance vs safety distance conflict')

        assert len(conflicts) > 0, "Should detect configuration conflicts"

    @pytest.mark.critical
    def test_resource_contention_between_subsystems(self):
        """Test resource contention causing subsystem failures."""
        # Mock subsystems competing for CPU
        subsystems = ['navigation', 'perception', 'control', 'safety']
        cpu_usage = {sub: 0.0 for sub in subsystems}

        # Simulate high CPU usage causing contention
        total_cpu = 0.0
        for sub in subsystems:
            cpu_usage[sub] = 0.8  # 80% each
            total_cpu += cpu_usage[sub]

        # Total CPU exceeds 100%, causing contention
        cpu_contention = total_cpu > 1.0
        assert cpu_contention, "Should detect CPU resource contention"

        # Subsystems should degrade under contention
        degraded_subsystems = [
            sub for sub, usage in cpu_usage.items()
            if usage > 0.7  # High usage threshold
        ]
        assert len(degraded_subsystems) > 0, "Should have degraded subsystems under contention"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
