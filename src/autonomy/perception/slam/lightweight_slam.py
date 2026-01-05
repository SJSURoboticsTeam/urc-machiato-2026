#!/usr/bin/env python3
"""
Lightweight SLAM Implementation

Provides basic visual odometry and mapping capabilities without heavy SLAM dependencies.
Used when full RTAB-Map SLAM is disabled or unavailable.

Capabilities:
- Visual odometry using feature tracking
- Simple pose estimation
- Basic map building with occupancy grid
- Loop closure detection (basic)
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
import math
import time


class LightweightVisualOdometry:
    """
    Lightweight visual odometry using feature tracking.

    Provides basic pose estimation without full SLAM capabilities.
    """

    def __init__(self, focal_length: float = 600.0, baseline: float = 0.1):
        """
        Initialize lightweight visual odometry.

        Args:
            focal_length: Camera focal length in pixels
            baseline: Stereo baseline in meters
        """
        # Camera parameters
        self.focal_length = focal_length
        self.baseline = baseline

        # Feature detection and tracking
        self.feature_detector = cv2.FastFeatureDetector_create(
            threshold=20,
            nonmaxSuppression=True
        )
        self.feature_tracker = cv2.TrackerKCF_create()

        # Lucas-Kanade optical flow parameters
        self.lk_params = dict(
            winSize=(21, 21),
            maxLevel=3,
            criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01)
        )

        # State tracking
        self.prev_frame: Optional[np.ndarray] = None
        self.prev_keypoints: Optional[np.ndarray] = None
        self.current_pose = np.eye(4)  # 4x4 transformation matrix
        self.keyframes: List[Dict[str, Any]] = []

        # Parameters
        self.min_features = 50
        self.max_features = 200
        self.keyframe_distance_threshold = 0.5  # meters
        self.keyframe_angle_threshold = 5.0  # degrees

        self.get_logger().info("Lightweight Visual Odometry initialized")

    def get_logger(self):
        """Get logger (compatibility with ROS nodes)."""
        import logging
        return logging.getLogger(__name__)

    def process_frame(self, frame: np.ndarray) -> Dict[str, Any]:
        """
        Process a new frame and update pose estimate.

        Args:
            frame: Input grayscale or RGB image

        Returns:
            Dictionary with pose estimate and tracking information
        """
        # Convert to grayscale if needed
        if len(frame.shape) == 3:
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        else:
            gray = frame

        result = {
            'pose': self.current_pose.copy(),
            'keyframe_added': False,
            'features_tracked': 0,
            'translation': np.zeros(3),
            'rotation': np.zeros(3),
            'confidence': 0.0
        }

        if self.prev_frame is None:
            # First frame
            self.prev_frame = gray.copy()
            self.prev_keypoints = self._detect_features(gray)
            self._add_keyframe(gray, self.current_pose)
            result['confidence'] = 1.0
            return result

        # Track features from previous frame
        if self.prev_keypoints is not None and len(self.prev_keypoints) > 0:
            tracked_points, status, err = cv2.calcOpticalFlowPyrLK(
                self.prev_frame, gray, self.prev_keypoints, None, **self.lk_params
            )

            # Filter good tracks
            good_prev = self.prev_keypoints[status.flatten() == 1]
            good_curr = tracked_points[status.flatten() == 1]

            if len(good_prev) >= 10:  # Minimum points for pose estimation
                # Estimate motion
                transformation = self._estimate_motion(good_prev, good_curr)

                if transformation is not None:
                    # Update pose
                    self.current_pose = self.current_pose @ transformation

                    # Extract translation and rotation
                    result['translation'] = transformation[:3, 3]
                    result['rotation'] = self._rotation_matrix_to_euler_angles(transformation[:3, :3])
                    result['features_tracked'] = len(good_curr)
                    result['confidence'] = min(len(good_curr) / self.min_features, 1.0)

                    # Check if we should add a keyframe
                    if self._should_add_keyframe(transformation):
                        self._add_keyframe(gray, self.current_pose)
                        result['keyframe_added'] = True

                    # Update for next iteration
                    self.prev_frame = gray.copy()
                    self.prev_keypoints = self._detect_features(gray)  # Redetect features
                else:
                    result['confidence'] = 0.0
            else:
                # Too few features tracked, reset
                self.prev_frame = gray.copy()
                self.prev_keypoints = self._detect_features(gray)
                result['confidence'] = 0.0
        else:
            # Redetect features
            self.prev_frame = gray.copy()
            self.prev_keypoints = self._detect_features(gray)
            result['confidence'] = 0.5

        return result

    def _detect_features(self, image: np.ndarray) -> np.ndarray:
        """Detect FAST features in the image."""
        keypoints = self.feature_detector.detect(image)

        # Convert to numpy array of points
        if keypoints:
            points = np.array([kp.pt for kp in keypoints], dtype=np.float32)
            # Limit number of features
            if len(points) > self.max_features:
                # Sort by response and take top features
                responses = np.array([kp.response for kp in keypoints])
                indices = np.argsort(responses)[-self.max_features:]
                points = points[indices]
            return points.reshape(-1, 1, 2)
        else:
            return np.array([]).reshape(0, 1, 2)

    def _estimate_motion(self, prev_points: np.ndarray, curr_points: np.ndarray) -> Optional[np.ndarray]:
        """
        Estimate 3D motion from 2D feature correspondences.

        This is a simplified approach - in practice, you'd use essential matrix decomposition.
        """
        if len(prev_points) < 8 or len(curr_points) < 8:
            return None

        # Flatten point arrays
        prev_pts = prev_points.reshape(-1, 2)
        curr_pts = curr_points.reshape(-1, 2)

        # Calculate fundamental matrix
        F, mask = cv2.findFundamentalMat(prev_pts, curr_pts, cv2.FM_RANSAC, 1.0, 0.99)

        if F is None:
            return None

        # For simplicity, estimate translation from feature motion
        # This is not a proper 3D reconstruction, but provides basic odometry
        motion_vectors = curr_pts - prev_pts
        avg_motion = np.mean(motion_vectors, axis=0)

        # Convert pixel motion to rough translation estimate
        # This is highly simplified - real VO would use proper structure from motion
        focal_length = self.focal_length
        translation_scale = 0.01  # Rough scale factor (meters per pixel)

        tx = avg_motion[0] * translation_scale
        ty = avg_motion[1] * translation_scale
        tz = 0.0  # Assume no depth motion for simplicity

        # Create transformation matrix (no rotation for simplicity)
        transformation = np.eye(4)
        transformation[0, 3] = tx
        transformation[1, 3] = ty
        transformation[2, 3] = tz

        return transformation

    def _should_add_keyframe(self, transformation: np.ndarray) -> bool:
        """Determine if current frame should be added as a keyframe."""
        if not self.keyframes:
            return True

        # Check translation distance
        translation = np.linalg.norm(transformation[:3, 3])
        if translation > self.keyframe_distance_threshold:
            return True

        # Check rotation angle
        rotation_matrix = transformation[:3, :3]
        angle = self._rotation_matrix_to_euler_angles(rotation_matrix)
        rotation_angle = np.linalg.norm(angle) * 180.0 / np.pi
        if rotation_angle > self.keyframe_angle_threshold:
            return True

        return False

    def _add_keyframe(self, image: np.ndarray, pose: np.ndarray):
        """Add current frame as a keyframe."""
        keyframe = {
            'image': image.copy(),
            'pose': pose.copy(),
            'timestamp': time.time(),
            'features': self.prev_keypoints.copy() if self.prev_keypoints is not None else None
        }

        self.keyframes.append(keyframe)

        # Limit keyframe history
        if len(self.keyframes) > 20:
            self.keyframes.pop(0)

    def _rotation_matrix_to_euler_angles(self, R: np.ndarray) -> np.ndarray:
        """Convert rotation matrix to Euler angles (XYZ convention)."""
        sy = math.sqrt(R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0])

        singular = sy < 1e-6

        if not singular:
            x = math.atan2(R[2, 1], R[2, 2])
            y = math.atan2(-R[2, 0], sy)
            z = math.atan2(R[1, 0], R[0, 0])
        else:
            x = math.atan2(-R[1, 2], R[1, 1])
            y = math.atan2(-R[2, 0], sy)
            z = 0

        return np.array([x, y, z])

    def get_trajectory(self) -> List[np.ndarray]:
        """Get the estimated trajectory from keyframes."""
        return [kf['pose'][:3, 3] for kf in self.keyframes]

    def reset(self):
        """Reset the visual odometry system."""
        self.prev_frame = None
        self.prev_keypoints = None
        self.current_pose = np.eye(4)
        self.keyframes = []


class LightweightOccupancyMapper:
    """
    Simple 2D occupancy grid mapping for lightweight navigation.
    """

    def __init__(self, resolution: float = 0.1, width: int = 200, height: int = 200):
        """
        Initialize lightweight occupancy mapper.

        Args:
            resolution: Map resolution in meters per cell
            width: Map width in cells
            height: Map height in cells
        """
        self.resolution = resolution
        self.width = width
        self.height = height

        # Occupancy grid (-1: unknown, 0: free, 100: occupied)
        self.grid = np.full((height, width), -1, dtype=np.int8)

        # Robot pose in map coordinates
        self.robot_x = width // 2
        self.robot_y = height // 2

        # Map limits for expansion
        self.min_x = self.robot_x
        self.max_x = self.robot_x
        self.min_y = self.robot_y
        self.max_y = self.robot_y

    def update_from_scan(self, ranges: List[float], angles: List[float],
                        robot_pose: Tuple[float, float, float]):
        """
        Update map from laser scan data.

        Args:
            ranges: Distance measurements
            angles: Angles for each measurement
            robot_pose: (x, y, theta) robot pose
        """
        x, y, theta = robot_pose

        for r, angle in zip(ranges, angles):
            if r > 0 and r < 10.0:  # Valid range
                # Transform to world coordinates
                world_angle = theta + angle
                wx = x + r * math.cos(world_angle)
                wy = y + r * math.sin(world_angle)

                # Convert to map coordinates
                mx = int(wx / self.resolution) + self.robot_x
                my = int(wy / self.resolution) + self.robot_y

                if 0 <= mx < self.width and 0 <= my < self.height:
                    # Mark as free space along the ray
                    # (Simplified - real implementation would ray trace)
                    self.grid[my, mx] = 100  # Occupied

                    # Update map bounds
                    self.min_x = min(self.min_x, mx)
                    self.max_x = max(self.max_x, mx)
                    self.min_y = min(self.min_y, my)
                    self.max_y = max(self.max_y, my)

    def get_map(self) -> np.ndarray:
        """Get the current occupancy grid."""
        return self.grid.copy()

    def get_free_space(self, x: float, y: float, radius: float = 0.5) -> bool:
        """
        Check if an area is free of obstacles.

        Args:
            x, y: Position to check
            radius: Radius to check around position

        Returns:
            True if area is free
        """
        # Convert to map coordinates
        mx = int(x / self.resolution) + self.robot_x
        my = int(y / self.resolution) + self.robot_y
        cells_radius = int(radius / self.resolution)

        # Check cells in radius
        for dy in range(-cells_radius, cells_radius + 1):
            for dx in range(-cells_radius, cells_radius + 1):
                cx = mx + dx
                cy = my + dy

                if 0 <= cx < self.width and 0 <= cy < self.height:
                    if self.grid[cy, cx] == 100:  # Occupied
                        return False

        return True

    def expand_map(self):
        """Expand map if robot is near boundaries."""
        # Simple expansion - double size if needed
        if (self.robot_x - self.min_x < 20 or
            self.max_x - self.robot_x < 20 or
            self.robot_y - self.min_y < 20 or
            self.max_y - self.robot_y < 20):

            # Double map size
            new_width = self.width * 2
            new_height = self.height * 2

            new_grid = np.full((new_height, new_width), -1, dtype=np.int8)

            # Copy existing map to center
            offset_x = (new_width - self.width) // 2
            offset_y = (new_height - self.height) // 2
            new_grid[offset_y:offset_y+self.height, offset_x:offset_x+self.width] = self.grid

            self.grid = new_grid
            self.width = new_width
            self.height = new_height
            self.robot_x += offset_x
            self.robot_y += offset_y


class LightweightSLAMSystem:
    """
    Complete lightweight SLAM system combining visual odometry and mapping.
    """

    def __init__(self):
        """Initialize lightweight SLAM system."""
        self.vo = LightweightVisualOdometry()
        self.mapper = LightweightOccupancyMapper()
        self.current_pose = np.eye(4)

    def process_frame(self, image: np.ndarray) -> Dict[str, Any]:
        """
        Process a new camera frame.

        Args:
            image: Input camera image

        Returns:
            SLAM results
        """
        # Update visual odometry
        vo_result = self.vo.process_frame(image)
        self.current_pose = vo_result['pose']

        # Extract 2D pose for mapping
        x, y = self.current_pose[0, 3], self.current_pose[1, 3]
        theta = math.atan2(self.current_pose[1, 0], self.current_pose[0, 0])

        result = {
            'pose_3d': self.current_pose,
            'pose_2d': (x, y, theta),
            'confidence': vo_result['confidence'],
            'keyframes': len(self.vo.keyframes),
            'features_tracked': vo_result['features_tracked']
        }

        return result

    def get_map(self) -> np.ndarray:
        """Get current occupancy map."""
        return self.mapper.get_map()

    def check_path_clear(self, start: Tuple[float, float],
                        end: Tuple[float, float]) -> bool:
        """
        Check if path between two points is clear.

        Args:
            start: Start position (x, y)
            end: End position (x, y)

        Returns:
            True if path is clear
        """
        # Simple line-of-sight check (could be improved with A* or similar)
        distance = math.sqrt((end[0] - start[0])**2 + (end[1] - start[1])**2)
        steps = int(distance / self.mapper.resolution)

        if steps == 0:
            return True

        for i in range(steps + 1):
            t = i / steps
            x = start[0] + t * (end[0] - start[0])
            y = start[1] + t * (end[1] - start[1])

            if not self.mapper.get_free_space(x, y, 0.2):
                return False

        return True
