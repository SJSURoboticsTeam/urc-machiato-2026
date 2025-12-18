#!/usr/bin/env python3
"""
Robotic Arm Simulator - URC Machiato 2026

MOCK IMPLEMENTATION - Simulates robotic arm kinematics and control systems.
Provides forward kinematics, inverse kinematics, motion planning, and control.

This is a software simulation of the physical robotic arm hardware and should be
replaced with actual arm control hardware when available.

Author: URC 2026 Autonomy Team
"""

import logging
import time
from dataclasses import dataclass
from enum import Enum
from typing import Dict, List, Optional, Tuple

import numpy as np

logger = logging.getLogger(__name__)


class JointType(Enum):
    """Types of robotic joints."""

    REVOLUTE = "revolute"  # Rotary joint
    PRISMATIC = "prismatic"  # Linear joint


@dataclass
class JointConfig:
    """Configuration for a robotic joint."""

    name: str
    joint_type: JointType
    position_limits: Tuple[float, float]  # min, max in radians or meters
    velocity_limit: float  # rad/s or m/s
    acceleration_limit: float  # rad/s² or m/s²
    gear_ratio: float = 1.0
    offset: float = 0.0  # Zero position offset


@dataclass
class LinkConfig:
    """Configuration for a robotic link."""

    length: float  # Link length in meters
    mass: float  # Link mass in kg
    com_offset: float  # Center of mass offset from joint


class RoboticArmSimulator:
    """
    MOCK IMPLEMENTATION - Complete robotic arm simulation.

    Simulates a 6-DOF robotic arm with:
    - Forward kinematics (position calculation)
    - Inverse kinematics (joint angle calculation)
    - Motion planning and trajectory generation
    - Joint control with PID controllers
    - Collision detection and safety limits
    - Force/torque sensing simulation
    - End-effector control (gripper)

    This replaces physical robotic arm hardware for software development.
    """

    def __init__(self, config: Optional[Dict] = None):
        self.logger = logging.getLogger(__name__)

        # Default arm configuration (6-DOF industrial robot style)
        if config is None:
            config = self._get_default_config()

        self._load_configuration(config)
        self._initialize_state()
        self._initialize_controllers()

        self.logger.info("🤖 MOCK Robotic Arm Simulator initialized")

    def _get_default_config(self) -> Dict:
        """Get default arm configuration."""
        return {
            "name": "URC_Arm_Simulator",
            "dof": 6,
            "joints": [
                # Joint 1: Base rotation
                {
                    "name": "base_rotation",
                    "joint_type": "revolute",
                    "position_limits": (-np.pi, np.pi),
                    "velocity_limit": 1.5,
                    "acceleration_limit": 3.0,
                    "gear_ratio": 100.0,
                },
                # Joint 2: Shoulder
                {
                    "name": "shoulder",
                    "joint_type": "revolute",
                    "position_limits": (-np.pi / 2, np.pi / 2),
                    "velocity_limit": 1.2,
                    "acceleration_limit": 2.5,
                    "gear_ratio": 80.0,
                },
                # Joint 3: Elbow
                {
                    "name": "elbow",
                    "joint_type": "revolute",
                    "position_limits": (-np.pi, 0),
                    "velocity_limit": 1.0,
                    "acceleration_limit": 2.0,
                    "gear_ratio": 60.0,
                },
                # Joint 4: Wrist pitch
                {
                    "name": "wrist_pitch",
                    "joint_type": "revolute",
                    "position_limits": (-np.pi / 2, np.pi / 2),
                    "velocity_limit": 2.0,
                    "acceleration_limit": 4.0,
                    "gear_ratio": 40.0,
                },
                # Joint 5: Wrist roll
                {
                    "name": "wrist_roll",
                    "joint_type": "revolute",
                    "position_limits": (-np.pi, np.pi),
                    "velocity_limit": 2.5,
                    "acceleration_limit": 5.0,
                    "gear_ratio": 30.0,
                },
                # Joint 6: Gripper rotation (placeholder for linear actuator)
                {
                    "name": "gripper_rotation",
                    "joint_type": "revolute",
                    "position_limits": (-np.pi / 4, np.pi / 4),
                    "velocity_limit": 1.0,
                    "acceleration_limit": 2.0,
                    "gear_ratio": 20.0,
                },
            ],
            "links": [
                {"length": 0.2, "mass": 2.0, "com_offset": 0.1},  # Base to shoulder
                {"length": 0.3, "mass": 1.5, "com_offset": 0.15},  # Shoulder to elbow
                {"length": 0.25, "mass": 1.0, "com_offset": 0.125},  # Elbow to wrist
                {"length": 0.1, "mass": 0.5, "com_offset": 0.05},  # Wrist pitch to roll
                {
                    "length": 0.08,
                    "mass": 0.3,
                    "com_offset": 0.04,
                },  # Wrist roll to gripper
                {"length": 0.05, "mass": 0.2, "com_offset": 0.025},  # Gripper
            ],
            "end_effector": {
                "gripper_open_width": 0.1,  # meters
                "gripper_close_width": 0.0,  # meters
                "max_grip_force": 50.0,  # Newtons
                "grip_sensor_resolution": 0.001,  # meters
            },
        }

    def _load_configuration(self, config: Dict):
        """Load arm configuration."""
        self.name = config["name"]
        self.dof = config["dof"]

        # Load joint configurations
        self.joints = []
        for joint_config in config["joints"]:
            joint = JointConfig(**joint_config)
            self.joints.append(joint)

        # Load link configurations
        self.links = []
        for link_config in config["links"]:
            link = LinkConfig(**link_config)
            self.links.append(link)

        # End effector configuration
        self.end_effector_config = config["end_effector"]

    def _initialize_state(self):
        """Initialize arm state."""
        self.joint_positions = np.zeros(self.dof)  # Current joint positions
        self.joint_velocities = np.zeros(self.dof)  # Current joint velocities
        self.joint_torques = np.zeros(self.dof)  # Current joint torques

        self.joint_position_targets = np.zeros(self.dof)  # Target positions
        self.joint_velocity_targets = np.zeros(self.dof)  # Target velocities

        # End effector state
        self.gripper_width = self.end_effector_config["gripper_open_width"]
        self.gripper_force = 0.0
        self.object_gripped = False

        # Simulation state
        self.last_update_time = time.time()
        self.enabled = True
        self.emergency_stop = False

        # Safety limits
        self.collision_detected = False
        self.joint_limit_violation = False

    def _initialize_controllers(self):
        """Initialize joint controllers."""
        # PID controller gains (tuned for simulation)
        self.kp = np.array([100.0, 80.0, 60.0, 40.0, 30.0, 20.0])
        self.ki = np.array([10.0, 8.0, 6.0, 4.0, 3.0, 2.0])
        self.kd = np.array([20.0, 16.0, 12.0, 8.0, 6.0, 4.0])

        # Controller state
        self.position_errors_integral = np.zeros(self.dof)
        self.previous_position_errors = np.zeros(self.dof)

    def forward_kinematics(self, joint_angles: np.ndarray) -> np.ndarray:
        """
        Calculate forward kinematics.

        Args:
            joint_angles: Joint angles in radians

        Returns:
            4x4 transformation matrix for end effector pose
        """
        # Simplified DH parameters for 6-DOF arm
        # This is a mock implementation - real FK would use proper DH parameters
        T = np.eye(4)

        # Base rotation
        T = T @ self._rotation_z(joint_angles[0])

        # Shoulder
        T = T @ self._translation_z(self.links[0].length)
        T = T @ self._rotation_y(joint_angles[1])

        # Elbow
        T = T @ self._translation_z(self.links[1].length)
        T = T @ self._rotation_y(joint_angles[2])

        # Wrist
        T = T @ self._translation_z(self.links[2].length)
        T = T @ self._rotation_x(joint_angles[3])
        T = T @ self._rotation_z(joint_angles[4])

        # End effector
        T = T @ self._translation_z(self.links[3].length)
        T = T @ self._rotation_z(joint_angles[5])

        return T

    def inverse_kinematics(
        self, target_pose: np.ndarray, current_joints: Optional[np.ndarray] = None
    ) -> Optional[np.ndarray]:
        """
        Calculate inverse kinematics.

        Args:
            target_pose: 4x4 target transformation matrix
            current_joints: Current joint angles for solution selection

        Returns:
            Joint angles that achieve target pose, or None if unreachable
        """
        if current_joints is None:
            current_joints = self.joint_positions.copy()

        # Mock IK implementation using numerical optimization
        # In reality, this would use analytical IK or optimization

        def pose_error(joints):
            """Calculate pose error for optimization."""
            actual_pose = self.forward_kinematics(joints)
            position_error = np.linalg.norm(target_pose[:3, 3] - actual_pose[:3, 3])
            orientation_error = np.linalg.norm(
                target_pose[:3, :3] @ actual_pose[:3, :3].T - np.eye(3)
            )
            return position_error + orientation_error

        # Simple gradient descent (mock implementation)
        joints = current_joints.copy()
        learning_rate = 0.01
        max_iterations = 100

        for _ in range(max_iterations):
            error = pose_error(joints)
            if error < 0.001:  # Convergence threshold
                break

            # Mock gradient calculation (would be computed properly)
            gradient = np.random.normal(0, 0.1, self.dof)
            joints -= learning_rate * gradient

            # Apply joint limits
            joints = np.clip(
                joints,
                [j.position_limits[0] for j in self.joints],
                [j.position_limits[1] for j in self.joints],
            )

        # Check if solution is valid
        final_error = pose_error(joints)
        if final_error < 0.01:  # Acceptable error threshold
            return joints
        else:
            return None

    def move_to_pose(self, target_pose: np.ndarray, duration: float = 2.0) -> bool:
        """
        Move arm to target pose.

        Args:
            target_pose: 4x4 target transformation matrix
            duration: Movement duration in seconds

        Returns:
            bool: True if movement planned successfully
        """
        if self.emergency_stop or not self.enabled:
            return False

        # Calculate inverse kinematics
        target_joints = self.inverse_kinematics(target_pose, self.joint_positions)

        if target_joints is None:
            self.logger.error("Target pose unreachable")
            return False

        # Plan trajectory (simple linear interpolation for simulation)
        trajectory = self._plan_trajectory(
            self.joint_positions, target_joints, duration
        )

        # Execute movement
        self._execute_trajectory(trajectory)

        return True

    def move_joints(self, target_joints: np.ndarray, duration: float = 2.0) -> bool:
        """
        Move joints to target positions.

        Args:
            target_joints: Target joint angles
            duration: Movement duration in seconds

        Returns:
            bool: True if movement executed successfully
        """
        if self.emergency_stop or not self.enabled:
            return False

        # Validate joint limits
        for i, (target, joint) in enumerate(zip(target_joints, self.joints)):
            if not (joint.position_limits[0] <= target <= joint.position_limits[1]):
                self.logger.error(
                    f"Joint {i} target {target} outside limits {joint.position_limits}"
                )
                return False

        # Plan and execute trajectory
        trajectory = self._plan_trajectory(
            self.joint_positions, target_joints, duration
        )
        self._execute_trajectory(trajectory)

        return True

    def set_gripper(self, width: float, force: Optional[float] = None) -> bool:
        """
        Control gripper.

        Args:
            width: Gripper opening width in meters
            force: Grip force in Newtons (optional)

        Returns:
            bool: True if gripper command executed
        """
        # Validate gripper limits
        min_width = self.end_effector_config["gripper_close_width"]
        max_width = self.end_effector_config["gripper_open_width"]

        if not (min_width <= width <= max_width):
            self.logger.error(
                f"Gripper width {width} outside limits [{min_width}, {max_width}]"
            )
            return False

        self.gripper_width = width

        if force is not None:
            max_force = self.end_effector_config["max_grip_force"]
            self.gripper_force = min(force, max_force)
        else:
            self.gripper_force = 0.0

        # Simulate gripping logic
        if width < 0.01:  # Closed position
            self.object_gripped = self.gripper_force > 10.0  # Mock gripping detection
        else:
            self.object_gripped = False

        return True

    def get_arm_state(self) -> Dict:
        """Get complete arm state."""
        end_effector_pose = self.forward_kinematics(self.joint_positions)

        return {
            "joint_positions": self.joint_positions.tolist(),
            "joint_velocities": self.joint_velocities.tolist(),
            "joint_torques": self.joint_torques.tolist(),
            "end_effector_pose": {
                "position": end_effector_pose[:3, 3].tolist(),
                "orientation": self._rotation_matrix_to_quaternion(
                    end_effector_pose[:3, :3]
                ).tolist(),
            },
            "gripper_width": self.gripper_width,
            "gripper_force": self.gripper_force,
            "object_gripped": self.object_gripped,
            "enabled": self.enabled,
            "emergency_stop": self.emergency_stop,
            "collision_detected": self.collision_detected,
            "joint_limit_violation": self.joint_limit_violation,
            "mock": True,
            "simulated": True,
        }

    def emergency_stop_arm(self) -> bool:
        """Emergency stop the arm."""
        self.logger.warning("🚨 ARM EMERGENCY STOP")
        self.emergency_stop = True
        self.joint_velocity_targets = np.zeros(self.dof)
        return True

    def enable_arm(self) -> bool:
        """Enable arm operation."""
        if not self.emergency_stop:
            self.enabled = True
            self.logger.info("Arm enabled")
            return True
        else:
            self.logger.warning("Cannot enable arm while in emergency stop")
            return False

    def disable_arm(self) -> bool:
        """Disable arm operation."""
        self.enabled = False
        self.joint_velocity_targets = np.zeros(self.dof)
        self.logger.info("Arm disabled")
        return True

    def clear_emergency_stop(self) -> bool:
        """Clear emergency stop condition."""
        if self.emergency_stop:
            self.emergency_stop = False
            self.logger.info("Emergency stop cleared")
        return True

    def update_simulation(self, dt: float):
        """
        Update arm simulation.

        Args:
            dt: Time step in seconds
        """
        if not self.enabled or self.emergency_stop:
            return

        # Update joint controllers
        for i in range(self.dof):
            error = self.joint_position_targets[i] - self.joint_positions[i]

            # PID control
            self.position_errors_integral[i] += error * dt
            derivative = (error - self.previous_position_errors[i]) / dt

            torque = (
                self.kp[i] * error
                + self.ki[i] * self.position_errors_integral[i]
                + self.kd[i] * derivative
            )

            # Apply velocity limits
            velocity = np.clip(
                torque * 0.1,
                -self.joints[i].velocity_limit,
                self.joints[i].velocity_limit,
            )
            self.joint_velocities[i] = velocity

            # Integrate position
            self.joint_positions[i] += velocity * dt

            # Apply position limits
            self.joint_positions[i] = np.clip(
                self.joint_positions[i],
                self.joints[i].position_limits[0],
                self.joints[i].position_limits[1],
            )

            self.previous_position_errors[i] = error

        # Check for joint limit violations
        self.joint_limit_violation = any(
            pos <= joint.position_limits[0] or pos >= joint.position_limits[1]
            for pos, joint in zip(self.joint_positions, self.joints)
        )

        # Mock collision detection
        self.collision_detected = self._check_collisions()

        self.last_update_time = time.time()

    def _plan_trajectory(
        self, start_joints: np.ndarray, end_joints: np.ndarray, duration: float
    ) -> List[np.ndarray]:
        """Plan joint trajectory (simple linear interpolation)."""
        num_points = max(10, int(duration * 50))  # 50 Hz trajectory
        trajectory = []

        for i in range(num_points + 1):
            t = i / num_points
            # Linear interpolation with smooth acceleration/deceleration
            s = 3 * t**2 - 2 * t**3  # Smooth step function
            joints = start_joints + s * (end_joints - start_joints)
            trajectory.append(joints.copy())

        return trajectory

    def _execute_trajectory(self, trajectory: List[np.ndarray]):
        """Execute joint trajectory."""
        for target_joints in trajectory:
            self.joint_position_targets = target_joints
            # In simulation, we can "instantly" move to target
            # In reality, this would be handled by the control loop
            self.joint_positions = target_joints.copy()
            time.sleep(0.02)  # Simulate control loop timing

    def _check_collisions(self) -> bool:
        """Mock collision detection."""
        # Simple mock: check if arm is in self-collision configuration
        # Real implementation would use collision checking algorithms
        shoulder_angle = abs(self.joint_positions[1])
        elbow_angle = abs(self.joint_positions[2])

        # Mock collision condition
        return shoulder_angle > np.pi / 3 and elbow_angle > np.pi / 4

    def _rotation_z(self, angle: float) -> np.ndarray:
        """Rotation matrix around Z axis."""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])

    def _rotation_y(self, angle: float) -> np.ndarray:
        """Rotation matrix around Y axis."""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[c, 0, s, 0], [0, 1, 0, 0], [-s, 0, c, 0], [0, 0, 0, 1]])

    def _rotation_x(self, angle: float) -> np.ndarray:
        """Rotation matrix around X axis."""
        c, s = np.cos(angle), np.sin(angle)
        return np.array([[1, 0, 0, 0], [0, c, -s, 0], [0, s, c, 0], [0, 0, 0, 1]])

    def _translation_z(self, distance: float) -> np.ndarray:
        """Translation matrix along Z axis."""
        return np.array([[1, 0, 0, 0], [0, 1, 0, 0], [0, 0, 1, distance], [0, 0, 0, 1]])

    def _rotation_matrix_to_quaternion(self, R: np.ndarray) -> np.ndarray:
        """Convert rotation matrix to quaternion."""
        # Simplified implementation
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                w = (R[2, 1] - R[1, 2]) / s
                x = 0.25 * s
                y = (R[0, 1] + R[1, 0]) / s
                z = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                w = (R[0, 2] - R[2, 0]) / s
                x = (R[0, 1] + R[1, 0]) / s
                y = 0.25 * s
                z = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                w = (R[1, 0] - R[0, 1]) / s
                x = (R[0, 2] + R[2, 0]) / s
                y = (R[1, 2] + R[2, 1]) / s
                z = 0.25 * s

        return np.array([w, x, y, z])
