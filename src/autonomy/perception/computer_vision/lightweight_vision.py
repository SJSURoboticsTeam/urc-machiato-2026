#!/usr/bin/env python3
"""
Lightweight Computer Vision Fallback Implementation

Provides basic computer vision capabilities without heavy ML dependencies.
Used when full computer vision is disabled or unavailable.

Capabilities:
- Basic color segmentation
- Simple shape detection
- Edge detection for obstacle avoidance
- Minimal ArUco marker detection
"""

import cv2
import numpy as np
from typing import List, Tuple, Optional, Dict, Any
import math


class LightweightVisionProcessor:
    """
    Lightweight computer vision processor with minimal dependencies.

    Provides basic vision capabilities for navigation and obstacle avoidance
    without requiring PyTorch, TensorFlow, or Detectron2.
    """

    def __init__(self):
        """Initialize lightweight vision processor."""
        # Color ranges for basic object detection (HSV)
        self.color_ranges = {
            'orange': {
                'lower': np.array([5, 50, 50]),
                'upper': np.array([15, 255, 255])
            },
            'blue': {
                'lower': np.array([90, 50, 50]),
                'upper': np.array([130, 255, 255])
            },
            'red': {
                'lower': np.array([0, 50, 50]),
                'upper': np.array([10, 255, 255])
            },
            'green': {
                'lower': np.array([40, 50, 50]),
                'upper': np.array([80, 255, 255])
            }
        }

        # Basic ArUco setup (minimal memory footprint)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        # Reduce complexity for lightweight mode
        self.aruco_params.adaptiveThreshWinSizeMin = 3
        self.aruco_params.adaptiveThreshWinSizeMax = 23
        self.aruco_params.adaptiveThreshWinSizeStep = 10
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Edge detection parameters
        self.canny_low_threshold = 50
        self.canny_high_threshold = 150

        # Contour detection parameters
        self.min_contour_area = 100
        self.max_contour_area = 50000

    def detect_objects_basic(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """
        Basic object detection using color segmentation and contours.

        Args:
            image: Input RGB image

        Returns:
            List of detected objects with basic properties
        """
        detections = []
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        for color_name, color_range in self.color_ranges.items():
            # Create mask for color
            mask = cv2.inRange(hsv_image, color_range['lower'], color_range['upper'])

            # Apply morphological operations to reduce noise
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Find contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            for contour in contours:
                area = cv2.contourArea(contour)
                if self.min_contour_area < area < self.max_contour_area:
                    # Get bounding rectangle
                    x, y, w, h = cv2.boundingRect(contour)

                    # Calculate centroid
                    M = cv2.moments(contour)
                    if M["m00"] != 0:
                        cx = int(M["m10"] / M["m00"])
                        cy = int(M["m01"] / M["m00"])
                    else:
                        cx, cy = x + w//2, y + h//2

                    detections.append({
                        'type': 'color_object',
                        'color': color_name,
                        'bbox': (x, y, w, h),
                        'centroid': (cx, cy),
                        'area': area,
                        'confidence': 0.7  # Fixed confidence for basic detection
                    })

        return detections

    def detect_aruco_markers_basic(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """
        Basic ArUco marker detection with minimal processing.

        Args:
            image: Input grayscale or RGB image

        Returns:
            List of detected ArUco markers
        """
        markers = []

        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None:
            for i, marker_id in enumerate(ids.flatten()):
                corner = corners[i][0]

                # Calculate centroid
                cx = int(np.mean(corner[:, 0]))
                cy = int(np.mean(corner[:, 1]))

                # Calculate bounding box
                x = int(np.min(corner[:, 0]))
                y = int(np.min(corner[:, 1]))
                w = int(np.max(corner[:, 0]) - x)
                h = int(np.max(corner[:, 1]) - y)

                markers.append({
                    'type': 'aruco_marker',
                    'id': int(marker_id),
                    'centroid': (cx, cy),
                    'bbox': (x, y, w, h),
                    'corners': corner.tolist(),
                    'confidence': 0.9  # ArUco detection is generally reliable
                })

        return markers

    def detect_edges_basic(self, image: np.ndarray) -> np.ndarray:
        """
        Basic edge detection for obstacle avoidance.

        Args:
            image: Input image

        Returns:
            Binary edge image
        """
        # Convert to grayscale if needed
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        # Apply Gaussian blur to reduce noise
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)

        # Apply Canny edge detection
        edges = cv2.Canny(blurred, self.canny_low_threshold, self.canny_high_threshold)

        return edges

    def detect_shapes_basic(self, image: np.ndarray) -> List[Dict[str, Any]]:
        """
        Basic shape detection using contour approximation.

        Args:
            image: Input image

        Returns:
            List of detected shapes
        """
        shapes = []

        # Convert to grayscale and apply threshold
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Apply Gaussian blur and threshold
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        _, thresh = cv2.threshold(blurred, 60, 255, cv2.THRESH_BINARY)

        # Find contours
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for contour in contours:
            area = cv2.contourArea(contour)
            if self.min_contour_area < area < self.max_contour_area:
                # Approximate contour
                perimeter = cv2.arcLength(contour, True)
                approx = cv2.approxPolyDP(contour, 0.04 * perimeter, True)

                # Classify shape based on number of vertices
                shape_name = "unknown"
                if len(approx) == 3:
                    shape_name = "triangle"
                elif len(approx) == 4:
                    # Check if it's a square or rectangle
                    x, y, w, h = cv2.boundingRect(approx)
                    aspect_ratio = float(w) / h
                    if 0.95 <= aspect_ratio <= 1.05:
                        shape_name = "square"
                    else:
                        shape_name = "rectangle"
                elif len(approx) > 8:
                    shape_name = "circle"

                # Get bounding box
                x, y, w, h = cv2.boundingRect(contour)

                shapes.append({
                    'type': 'shape',
                    'shape': shape_name,
                    'bbox': (x, y, w, h),
                    'area': area,
                    'vertices': len(approx),
                    'confidence': 0.6  # Lower confidence for basic shape detection
                })

        return shapes

    def process_frame_lightweight(self, image: np.ndarray) -> Dict[str, Any]:
        """
        Complete lightweight vision processing pipeline.

        Args:
            image: Input RGB image

        Returns:
            Dictionary containing all detection results
        """
        results = {
            'objects': [],
            'markers': [],
            'edges': None,
            'shapes': [],
            'timestamp': None,
            'processing_time_ms': 0
        }

        import time
        start_time = time.time()

        try:
            # Perform detections
            results['objects'] = self.detect_objects_basic(image)
            results['markers'] = self.detect_aruco_markers_basic(image)
            results['edges'] = self.detect_edges_basic(image)
            results['shapes'] = self.detect_shapes_basic(image)

            results['processing_time_ms'] = (time.time() - start_time) * 1000
            results['timestamp'] = time.time()

        except Exception as e:
            results['error'] = str(e)
            results['processing_time_ms'] = (time.time() - start_time) * 1000

        return results

    def get_obstacle_mask(self, image: np.ndarray, distance_threshold: int = 100) -> np.ndarray:
        """
        Generate obstacle mask for navigation using edge detection.

        Args:
            image: Input image
            distance_threshold: Distance threshold for obstacle detection

        Returns:
            Binary mask where obstacles are marked
        """
        edges = self.detect_edges_basic(image)

        # Dilate edges to create obstacle zones
        kernel = np.ones((7, 7), np.uint8)
        obstacle_mask = cv2.dilate(edges, kernel, iterations=2)

        return obstacle_mask

    def estimate_motion_basic(self, prev_image: np.ndarray, curr_image: np.ndarray) -> Tuple[float, float]:
        """
        Basic motion estimation using optical flow.

        Args:
            prev_image: Previous frame
            curr_image: Current frame

        Returns:
            Tuple of (dx, dy) motion estimates
        """
        # Convert to grayscale
        if len(prev_image.shape) == 3:
            prev_gray = cv2.cvtColor(prev_image, cv2.COLOR_BGR2GRAY)
        else:
            prev_gray = prev_image

        if len(curr_image.shape) == 3:
            curr_gray = cv2.cvtColor(curr_image, cv2.COLOR_BGR2GRAY)
        else:
            curr_gray = curr_image

        # Calculate optical flow using Farneback method (lighter than Lucas-Kanade)
        flow = cv2.calcOpticalFlowFarneback(prev_gray, curr_gray, None, 0.5, 3, 15, 3, 5, 1.2, 0)

        # Calculate average motion
        dx = np.mean(flow[:, :, 0])
        dy = np.mean(flow[:, :, 1])

        return dx, dy


class LightweightTerrainAnalyzer:
    """
    Lightweight terrain analysis without ML dependencies.

    Provides basic terrain classification using image processing techniques.
    """

    def __init__(self):
        """Initialize lightweight terrain analyzer."""
        # Basic terrain color ranges (rough approximations)
        self.terrain_types = {
            'sand': {
                'hue_range': (20, 40),  # Yellow-orange hues
                'sat_range': (50, 200),
                'val_range': (150, 255)
            },
            'rock': {
                'hue_range': (0, 180),  # Wide range for rocks
                'sat_range': (10, 100),  # Low saturation
                'val_range': (50, 150)   # Medium brightness
            },
            'soil': {
                'hue_range': (10, 30),  # Brown hues
                'sat_range': (30, 150),
                'val_range': (50, 120)
            }
        }

    def classify_terrain_basic(self, image: np.ndarray) -> Dict[str, float]:
        """
        Basic terrain classification using color histogram analysis.

        Args:
            image: Input RGB image

        Returns:
            Dictionary with terrain type probabilities
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Calculate histogram
        hist = cv2.calcHist([hsv], [0, 1, 2], None, [18, 5, 5], [0, 180, 0, 256, 0, 256])
        hist = cv2.normalize(hist, hist).flatten()

        # Simple classification based on dominant colors
        terrain_scores = {}

        for terrain_type, params in self.terrain_types.items():
            score = 0

            # Check if dominant hues fall within terrain range
            hue_bins = np.arange(18)
            dominant_hues = hue_bins[hist[:18] > np.max(hist[:18]) * 0.3]

            for hue in dominant_hues:
                hue_deg = (hue / 18) * 180
                if params['hue_range'][0] <= hue_deg <= params['hue_range'][1]:
                    score += 0.5

            terrain_scores[terrain_type] = min(score, 1.0)

        return terrain_scores

    def detect_obstacles_basic(self, image: np.ndarray) -> List[Tuple[int, int, int, int]]:
        """
        Basic obstacle detection using edge density.

        Args:
            image: Input image

        Returns:
            List of obstacle bounding boxes
        """
        # Convert to grayscale and detect edges
        if len(image.shape) == 3:
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        else:
            gray = image

        edges = cv2.Canny(gray, 50, 150)

        # Divide image into grid and check edge density
        height, width = edges.shape
        grid_size = 32
        obstacles = []

        for y in range(0, height, grid_size):
            for x in range(0, width, grid_size):
                cell = edges[y:y+grid_size, x:x+grid_size]
                edge_density = np.sum(cell > 0) / (grid_size * grid_size)

                if edge_density > 0.1:  # Threshold for obstacle detection
                    obstacles.append((x, y, grid_size, grid_size))

        return obstacles
