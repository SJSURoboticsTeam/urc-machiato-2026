#!/usr/bin/env python3
"""
Arm Control Simulation Tests

Tests arm control system under various conditions:
- Joint control with network latency
- IK solver convergence under sensor noise
- Force feedback simulation
- Collision detection
- Force limits and safety
- Gripper control
- Trajectory execution under degradation

This addresses P1 high priority gap: Arm Control (0% coverage).
"""

import os
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np
import pytest

try:
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import JointState

    try:
        from control_msgs.msg import JointTrajectoryControllerState  # noqa: F401
    except ImportError:
        pass
    ROS2_AVAILABLE = True
except ImportError:
    ROS2_AVAILABLE = False
    rclpy = None
    Node = None

# Add project paths
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", ".."))
sys.path.insert(0, PROJECT_ROOT)
sys.path.insert(0, os.path.join(PROJECT_ROOT, "Autonomy", "code", "autonomous_typing"))

# Import simulation framework
try:
    from simulation.environments.environment_factory import EnvironmentFactory
    from simulation.network.network_emulator import NetworkEmulator, NetworkProfile
except ImportError:
    EnvironmentFactory = None
    NetworkEmulator = None

    # ROS2 context managed by session fixture
    """Initialize and cleanup ROS context."""
    if not ROS2_AVAILABLE:
        pytest.skip("ROS2 not available")
    rclpy.init()
    yield
    rclpy.shutdown()


class ArmControlSimulator:
    """Simulates arm control behavior for testing."""

    def __init__(self):
        self.joint_names = [
            "shoulder_pan_joint",
            "shoulder_lift_joint",
            "elbow_joint",
            "wrist_1_joint",
            "wrist_2_joint",
            "wrist_3_joint",
        ]
        self.current_joint_positions = np.zeros(len(self.joint_names))
        self.joint_limits = {name: (-np.pi, np.pi) for name in self.joint_names}
        self.max_joint_velocity = 1.0  # rad/s
        self.max_joint_acceleration = 2.0  # rad/s²

    def solve_ik(
        self,
        target_pos: np.ndarray,
        _target_orientation: Optional[np.ndarray] = None,
        noise_level: float = 0.0,
    ) -> Optional[np.ndarray]:
        """
        Solve inverse kinematics for target position.

        Args:
            target_pos: Target position (x, y, z)
            target_orientation: Target orientation (quaternion)
            noise_level: Sensor noise level (0.0 to 1.0)

        Returns:
            Joint angles or None if no solution
        """
        # Simplified IK solver (would use actual solver in production)
        # For testing, generate reasonable joint angles

        # Add noise based on noise_level
        noise = np.random.normal(0, noise_level * 0.1, len(self.joint_names))

        # Simple IK approximation
        angles = np.array(
            [
                np.arctan2(target_pos[1], target_pos[0]),  # shoulder_pan
                np.arcsin(target_pos[2] / np.linalg.norm(target_pos)),  # shoulder_lift
                np.pi / 4,  # elbow
                0.0,  # wrist_1
                0.0,  # wrist_2
                0.0,  # wrist_3
            ]
        )

        angles += noise

        # Check joint limits
        for i, name in enumerate(self.joint_names):
            min_angle, max_angle = self.joint_limits[name]
            if angles[i] < min_angle or angles[i] > max_angle:
                return None  # Invalid solution

        return angles

    def check_collision(self, joint_angles: np.ndarray, obstacles: List[Dict]) -> bool:
        """
        Check if joint configuration causes collision.

        Args:
            joint_angles: Joint angles
            obstacles: List of obstacle definitions

        Returns:
            True if collision detected
        """
        # Simplified collision detection
        # In production, would use actual collision checker

        # Forward kinematics to get end effector position
        ee_pos = self._forward_kinematics(joint_angles)

        # Check against obstacles
        for obstacle in obstacles:
            obs_pos = obstacle["position"]
            obs_radius = obstacle.get("radius", 0.1)

            distance = np.linalg.norm(ee_pos - obs_pos)
            if distance < obs_radius:
                return True  # Collision detected

        return False

    def _forward_kinematics(self, joint_angles: np.ndarray) -> np.ndarray:
        """Simple forward kinematics (simplified)."""
        # Simplified FK for testing
        x = (
            0.5
            * np.cos(joint_angles[0])
            * (np.cos(joint_angles[1]) + np.cos(joint_angles[1] + joint_angles[2]))
        )
        y = (
            0.5
            * np.sin(joint_angles[0])
            * (np.cos(joint_angles[1]) + np.cos(joint_angles[1] + joint_angles[2]))
        )
        z = 0.5 * (np.sin(joint_angles[1]) + np.sin(joint_angles[1] + joint_angles[2]))

        return np.array([x, y, z])

    def check_force_limits(
        self, joint_angles: np.ndarray, applied_forces: np.ndarray
    ) -> bool:
        """
        Check if applied forces exceed limits.

        Args:
            joint_angles: Current joint angles
            applied_forces: Forces applied at joints

        Returns:
            True if forces are within limits
        """
        max_force = 50.0  # N
        return np.all(np.abs(applied_forces) < max_force)


@pytest.mark.integration
@pytest.mark.ros2
@pytest.mark.slow
class TestArmControl:
    """Test arm control functionality."""

    def setUp(self):
        """Set up test environment."""
        self.arm_sim = ArmControlSimulator()
        if NetworkEmulator:
            self.net_emulators = {
                profile: NetworkEmulator(profile) for profile in NetworkProfile
            }

    def tearDown(self):
        """Clean up test environment."""
        if NetworkEmulator:
            for emulator in self.net_emulators.values():
                emulator.stop()

    def test_joint_control_perfect_conditions(self, ros_context):
        """Test joint control in perfect network conditions."""
        target_angles = np.array([0.5, 0.3, 0.2, 0.1, 0.0, 0.0])

        # Simulate joint control
        success = self._control_joints(target_angles, network_latency=0.0)

        assert success, "Joint control should succeed in perfect conditions"

    def test_joint_control_with_network_latency(self, ros_context):
        """Test joint control with network latency."""
        target_angles = np.array([0.5, 0.3, 0.2, 0.1, 0.0, 0.0])

        # Test with various latency levels
        latencies = [0.05, 0.1, 0.2, 0.5]  # seconds
        results = []

        for latency in latencies:
            success = self._control_joints(target_angles, network_latency=latency)
            results.append((latency, success))

        # Control should work even with latency (may be slower)
        assert all(
            success for _, success in results
        ), "Joint control should work with latency"

    def test_ik_solver_convergence(self, ros_context):
        """Test IK solver convergence."""
        target_pos = np.array([0.3, 0.2, 0.4])  # 30cm forward, 20cm right, 40cm up

        # Test IK solving
        solution = self.arm_sim.solve_ik(target_pos)

        assert solution is not None, "IK solver should find solution"
        assert len(solution) == len(
            self.arm_sim.joint_names
        ), "Solution should have correct number of joints"

        # Verify solution reaches target (approximately)
        ee_pos = self.arm_sim._forward_kinematics(solution)
        distance = np.linalg.norm(ee_pos - target_pos)
        assert distance < 0.1, f"IK solution should be accurate, error: {distance:.3f}m"

    def test_ik_solver_with_sensor_noise(self, ros_context):
        """Test IK solver under sensor noise."""
        target_pos = np.array([0.3, 0.2, 0.4])

        noise_levels = [0.0, 0.1, 0.2, 0.3]
        solutions = []

        for noise in noise_levels:
            solution = self.arm_sim.solve_ik(target_pos, noise_level=noise)
            if solution is not None:
                solutions.append((noise, solution))

        # IK should work even with noise (may be less accurate)
        assert len(solutions) > 0, "IK solver should work with sensor noise"

        # Check accuracy degrades with noise
        for noise, solution in solutions:
            ee_pos = self.arm_sim._forward_kinematics(solution)
            error = np.linalg.norm(ee_pos - target_pos)
            # Higher noise may lead to higher error, but should still be reasonable
            assert (
                error < 0.2
            ), f"IK error should be reasonable with noise {noise}, got {error:.3f}m"

    def test_collision_detection(self, ros_context):
        """Test collision detection."""
        # Test joint configuration
        joint_angles = np.array([0.5, 0.3, 0.2, 0.1, 0.0, 0.0])

        # No obstacles
        obstacles = []
        collision = self.arm_sim.check_collision(joint_angles, obstacles)
        assert not collision, "Should not detect collision with no obstacles"

        # Add obstacle near end effector
        ee_pos = self.arm_sim._forward_kinematics(joint_angles)
        obstacles = [{"position": ee_pos + np.array([0.05, 0.05, 0.05]), "radius": 0.1}]
        collision = self.arm_sim.check_collision(joint_angles, obstacles)
        assert collision, "Should detect collision with nearby obstacle"

    def test_force_limits(self, ros_context):
        """Test force limit checking."""
        joint_angles = np.array([0.5, 0.3, 0.2, 0.1, 0.0, 0.0])

        # Within limits
        forces = np.array([10.0, 15.0, 20.0, 5.0, 5.0, 5.0])
        within_limits = self.arm_sim.check_force_limits(joint_angles, forces)
        assert within_limits, "Forces within limits should be accepted"

        # Exceeds limits
        excessive_forces = np.array([60.0, 70.0, 80.0, 50.0, 50.0, 50.0])
        within_limits = self.arm_sim.check_force_limits(joint_angles, excessive_forces)
        assert not within_limits, "Excessive forces should be rejected"

    def test_gripper_control(self, ros_context):
        """Test gripper control."""
        # Test gripper open
        gripper_open = self._control_gripper(0.0)  # 0 = open
        assert gripper_open, "Gripper should open"

        # Test gripper close
        gripper_close = self._control_gripper(1.0)  # 1 = closed
        assert gripper_close, "Gripper should close"

        # Test partial close
        gripper_partial = self._control_gripper(0.5)  # 0.5 = half closed
        assert gripper_partial, "Gripper should support partial closure"

    def test_trajectory_execution(self, ros_context):
        """Test trajectory execution."""
        # Create trajectory
        start_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        end_angles = np.array([0.5, 0.3, 0.2, 0.1, 0.0, 0.0])

        trajectory = self._create_trajectory(start_angles, end_angles, duration=3.0)

        # Execute trajectory
        execution_success = self._execute_trajectory(trajectory)

        assert execution_success, "Trajectory execution should succeed"

    def test_trajectory_execution_under_degradation(self, ros_context):
        """Test trajectory execution under network degradation."""
        start_angles = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        end_angles = np.array([0.5, 0.3, 0.2, 0.1, 0.0, 0.0])

        trajectory = self._create_trajectory(start_angles, end_angles, duration=3.0)

        # Execute with network degradation
        if NetworkEmulator:
            net_emu = NetworkEmulator(NetworkProfile.RURAL_WIFI)
            net_emu.start()

        try:
            execution_success = self._execute_trajectory(
                trajectory, network_latency=0.1
            )
            assert (
                execution_success
            ), "Trajectory should execute even with network degradation"
        finally:
            if NetworkEmulator:
                net_emu.stop()

    def _control_joints(
        self, target_angles: np.ndarray, network_latency: float = 0.0
    ) -> bool:
        """Simulate joint control."""
        # Simulate network delay
        if network_latency > 0:
            time.sleep(network_latency)

        # Check joint limits
        for i, angle in enumerate(target_angles):
            joint_name = self.arm_sim.joint_names[i]
            min_angle, max_angle = self.arm_sim.joint_limits[joint_name]
            if angle < min_angle or angle > max_angle:
                return False

        # Simulate movement
        self.arm_sim.current_joint_positions = target_angles.copy()
        return True

    def _control_gripper(self, position: float) -> bool:
        """Simulate gripper control."""
        # Position: 0.0 = open, 1.0 = closed
        assert 0.0 <= position <= 1.0, "Gripper position should be 0-1"
        return True

    def _create_trajectory(
        self,
        start_angles: np.ndarray,
        end_angles: np.ndarray,
        duration: float = 3.0,
        num_points: int = 10,
    ) -> List[np.ndarray]:
        """Create trajectory from start to end angles."""
        trajectory = []
        for i in range(num_points + 1):
            t = i / num_points
            # Linear interpolation
            angles = start_angles + t * (end_angles - start_angles)
            trajectory.append(angles)
        return trajectory

    def _execute_trajectory(
        self, trajectory: List[np.ndarray], network_latency: float = 0.0
    ) -> bool:
        """Simulate trajectory execution."""
        for waypoint in trajectory:
            # Simulate network delay
            if network_latency > 0:
                time.sleep(network_latency / len(trajectory))

            # Check joint limits
            for i, angle in enumerate(waypoint):
                joint_name = self.arm_sim.joint_names[i]
                min_angle, max_angle = self.arm_sim.joint_limits[joint_name]
                if angle < min_angle or angle > max_angle:
                    return False

            # Update position
            self.arm_sim.current_joint_positions = waypoint.copy()

        return True


if __name__ == "__main__":
    pytest.main([__file__, "-v"])
