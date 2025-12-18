#!/usr/bin/env python3
"""
Terrain Intelligence - URC Terrain Analysis and Traversability Mapping

Provides real-time terrain analysis for Mars-like environments using:
- Computer vision for terrain classification
- Point cloud processing for slope analysis
- Traversability cost mapping for path planning
- Hazard detection and avoidance

Critical for URC competition terrain navigation.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, PointCloud2, LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header
import cv2
from cv_bridge import CvBridge
import numpy as np
import math
from typing import Optional, Tuple, List, Dict
import time


class TerrainType:
    """Terrain classification types for URC."""
    SAND = "sand"
    ROCK = "rock"
    SLOPE = "slope"
    HAZARD = "hazard"
    UNKNOWN = "unknown"


class TerrainAnalyzer(Node):
    """
    Terrain Intelligence System for URC Competition

    Analyzes terrain using multiple sensor inputs to provide:
    - Terrain classification (sand, rock, slope, hazard)
    - Traversability cost mapping
    - Slope analysis and hazard detection
    - Path planning cost information
    """

    def __init__(self):
        super().__init__('terrain_analyzer')

        # Parameters
        self.declare_parameter('map_resolution', 0.1)  # meters per cell
        self.declare_parameter('map_width', 20.0)      # meters
        self.declare_parameter('map_height', 20.0)     # meters
        self.declare_parameter('max_slope_angle', 30.0)  # degrees
        self.declare_parameter('hazard_height_threshold', 0.3)  # meters

        self.map_resolution = self.get_parameter('map_resolution').value
        self.map_width = self.get_parameter('map_width').value
        self.map_height = self.get_parameter('map_height').value
        self.max_slope = math.radians(self.get_parameter('max_slope_angle').value)
        self.hazard_threshold = self.get_parameter('hazard_height_threshold').value

        # CV bridge for image processing
        self.bridge = CvBridge()

        # Terrain map data
        self.terrain_map = None
        self.robot_pose = None
        self.map_initialized = False

        # Subscribers (optimized - use centralized vision processing)
        self.terrain_map_sub = self.create_subscription(
            OccupancyGrid, '/vision/terrain_map', self.terrain_map_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=3))

        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/camera/depth/points', self.pointcloud_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=3))

        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self.odom_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=3))

        # Publishers
        self.traversability_pub = self.create_publisher(
            OccupancyGrid, '/terrain/traversability', 10)

        self.terrain_class_pub = self.create_publisher(
            Image, '/terrain/classification', 10)

        # Processing rate control
        self.last_processing_time = 0
        self.processing_interval = 0.5  # 2Hz processing

        # Initialize terrain map
        self.initialize_terrain_map()

        self.get_logger().info("Terrain Analyzer initialized")
        self.get_logger().info(f"Map size: {self.map_width}x{self.map_height}m @ {self.map_resolution}m/cell")

    def initialize_terrain_map(self):
        """Initialize the traversability map."""
        width_cells = int(self.map_width / self.map_resolution)
        height_cells = int(self.map_height / self.map_resolution)

        self.terrain_map = OccupancyGrid()
        self.terrain_map.header.frame_id = 'map'
        self.terrain_map.info.resolution = self.map_resolution
        self.terrain_map.info.width = width_cells
        self.terrain_map.info.height = height_cells

        # Initialize map origin (centered on robot start position)
        self.terrain_map.info.origin.position.x = -self.map_width / 2.0
        self.terrain_map.info.origin.position.y = -self.map_height / 2.0
        self.terrain_map.info.origin.position.z = 0.0
        self.terrain_map.info.origin.orientation.w = 1.0

        # Initialize all cells as unknown (-1)
        self.terrain_map.data = [-1] * (width_cells * height_cells)

        self.map_initialized = True

    def odom_callback(self, msg: Odometry):
        """Update robot pose for map coordinate transformations."""
        self.robot_pose = msg.pose.pose

    def terrain_map_callback(self, msg: OccupancyGrid):
        """Receive processed terrain map from centralized vision processing."""
        current_time = time.time()
        if current_time - self.last_processing_time < self.processing_interval:
            return

        try:
            # Use the pre-processed terrain map from vision processor
            # This eliminates duplicate image processing
            self.terrain_map.header = msg.header
            self.terrain_map.info = msg.info
            self.terrain_map.data = msg.data

            # Publish traversability map (already processed)
            self.terrain_map.header.stamp = self.get_clock().now().to_msg()
            self.traversability_pub.publish(self.terrain_map)

            self.last_processing_time = current_time

        except Exception as e:
            self.get_logger().error(f"Terrain map processing error: {e}")

    def pointcloud_callback(self, msg: PointCloud2):
        """Process point cloud data for slope and hazard analysis."""
        try:
            # Extract height map from point cloud
            height_map = self.extract_height_map(msg)

            # Analyze slopes and hazards
            slope_map = self.calculate_slopes(height_map)
            hazard_map = self.detect_hazards(height_map)

            # Update traversability map with 3D data
            self.update_traversability_from_pointcloud(slope_map, hazard_map, msg.header)

        except Exception as e:
            self.get_logger().error(f"Point cloud processing error: {e}")

    def classify_terrain(self, image: np.ndarray) -> np.ndarray:
        """
        Classify terrain types using computer vision.

        Returns a classification map where:
        0 = sand (traversable)
        1 = rock (difficult)
        2 = slope (challenging)
        3 = hazard (avoid)
        """
        # Convert to HSV for better color analysis
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Create classification mask
        height, width = image.shape[:2]
        classification = np.zeros((height, width), dtype=np.uint8)

        # Simple terrain classification based on color and texture
        # This is a simplified version - real implementation would use ML

        # Detect sand (bright, uniform colors)
        sand_mask = cv2.inRange(hsv, (0, 30, 150), (50, 150, 255))
        classification[sand_mask > 0] = 0  # Traversable

        # Detect rocks (dark, textured areas)
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        texture = cv2.Laplacian(gray, cv2.CV_64F).var()
        if texture > 100:  # High texture = rocks
            rock_mask = cv2.inRange(hsv, (0, 50, 30), (180, 255, 120))
            classification[rock_mask > 0] = 1  # Difficult

        # Detect slopes (by edge detection and geometry)
        edges = cv2.Canny(gray, 50, 150)
        if np.mean(edges) > 10:  # High edge content = slope
            classification[edges > 0] = 2  # Challenging

        # Detect hazards (very dark or unusual colors)
        hazard_mask = cv2.inRange(hsv, (0, 0, 0), (180, 255, 30))
        classification[hazard_mask > 0] = 3  # Avoid

        return classification

    def extract_height_map(self, pointcloud: PointCloud2) -> np.ndarray:
        """Extract height map from point cloud data."""
        # This is a simplified implementation
        # Real implementation would properly parse PointCloud2 format

        # Create a simple height map (placeholder)
        height_map = np.zeros((100, 100))  # 10m x 10m at 0.1m resolution

        # In a real implementation, you would:
        # 1. Parse PointCloud2 data points
        # 2. Project onto ground plane
        # 3. Create height grid map
        # 4. Filter noise and outliers

        return height_map

    def calculate_slopes(self, height_map: np.ndarray) -> np.ndarray:
        """Calculate slope angles from height map."""
        # Calculate gradients
        dx = np.gradient(height_map, axis=1)
        dy = np.gradient(height_map, axis=0)

        # Calculate slope magnitude
        slope_magnitude = np.sqrt(dx**2 + dy**2)

        # Convert to angles (simplified)
        slope_angles = np.arctan(slope_magnitude)

        return slope_angles

    def detect_hazards(self, height_map: np.ndarray) -> np.ndarray:
        """Detect terrain hazards based on height variations."""
        # Detect steep drops or large obstacles
        hazard_map = np.zeros_like(height_map)

        # Find areas with large height differences (potential hazards)
        kernel = np.ones((3, 3)) / 9.0
        smoothed = cv2.filter2D(height_map, -1, kernel)
        height_diff = np.abs(height_map - smoothed)

        hazard_map[height_diff > self.hazard_threshold] = 1

        return hazard_map

    def update_traversability_from_image(self, classification: np.ndarray, header: Header):
        """Update traversability map with image-based classification."""
        if not self.terrain_map or not self.robot_pose:
            return

        # Project image classification onto traversability map
        # This is a simplified projection - real implementation would use camera intrinsics/extrinsics

        height, width = classification.shape

        # Map image pixels to world coordinates (simplified)
        for i in range(0, height, 10):  # Sample every 10th pixel
            for j in range(0, width, 10):
                terrain_type = classification[i, j]

                # Convert pixel coordinates to world coordinates
                world_x = self.robot_pose.position.x + (j - width/2) * 0.01  # 1cm per pixel
                world_y = self.robot_pose.position.y + (i - height/2) * 0.01

                # Update map cell
                cost = self.terrain_type_to_cost(terrain_type)
                self.update_map_cell(world_x, world_y, cost)

    def update_traversability_from_pointcloud(self, slope_map: np.ndarray, hazard_map: np.ndarray, header: Header):
        """Update traversability map with 3D terrain data."""
        if not self.terrain_map:
            return

        height, width = slope_map.shape

        # Update map based on slope analysis
        for i in range(height):
            for j in range(width):
                slope = slope_map[i, j]
                hazard = hazard_map[i, j]

                # Convert grid coordinates to world coordinates
                world_x = self.robot_pose.position.x + (j - width/2) * 0.1  # 10cm per cell
                world_y = self.robot_pose.position.y + (i - height/2) * 0.1

                # Calculate cost based on slope and hazards
                cost = 0
                if slope > self.max_slope:
                    cost = 100  # Impassable slope
                elif hazard > 0:
                    cost = 100  # Hazard
                else:
                    cost = min(100, int(slope * 180.0 / math.pi))  # Cost based on slope angle

                self.update_map_cell(world_x, world_y, cost)

        # Publish updated map
        self.terrain_map.header.stamp = self.get_clock().now().to_msg()
        self.traversability_pub.publish(self.terrain_map)

    def terrain_type_to_cost(self, terrain_type: int) -> int:
        """Convert terrain classification to traversability cost."""
        cost_map = {
            0: 10,   # Sand - easy traversal
            1: 50,   # Rock - moderate difficulty
            2: 80,   # Slope - challenging
            3: 100   # Hazard - avoid
        }
        return cost_map.get(terrain_type, 50)  # Default moderate cost

    def update_map_cell(self, world_x: float, world_y: float, cost: int):
        """Update a cell in the traversability map."""
        # Convert world coordinates to map coordinates
        map_x = world_x - self.terrain_map.info.origin.position.x
        map_y = world_y - self.terrain_map.info.origin.position.y

        # Convert to cell coordinates
        cell_x = int(map_x / self.map_resolution)
        cell_y = int(map_y / self.map_resolution)

        # Check bounds
        if (0 <= cell_x < self.terrain_map.info.width and
            0 <= cell_y < self.terrain_map.info.height):

            # Update cell value (use maximum cost seen)
            cell_index = cell_y * self.terrain_map.info.width + cell_x
            current_cost = self.terrain_map.data[cell_index]

            if current_cost == -1 or cost > current_cost:
                self.terrain_map.data[cell_index] = cost

    def get_traversability_cost(self, x: float, y: float) -> int:
        """Get traversability cost at a specific location."""
        if not self.terrain_map:
            return 50  # Default moderate cost

        # Convert world coordinates to cell coordinates
        map_x = x - self.terrain_map.info.origin.position.x
        map_y = y - self.terrain_map.info.origin.position.y

        cell_x = int(map_x / self.map_resolution)
        cell_y = int(map_y / self.map_resolution)

        # Check bounds
        if (0 <= cell_x < self.terrain_map.info.width and
            0 <= cell_y < self.terrain_map.info.height):

            cell_index = cell_y * self.terrain_map.info.width + cell_x
            cost = self.terrain_map.data[cell_index]

            return cost if cost != -1 else 50  # Default cost for unknown areas

        return 100  # Out of bounds = impassable


def main(args=None):
    rclpy.init(args=args)

    analyzer = TerrainAnalyzer()

    try:
        rclpy.spin(analyzer)
    except KeyboardInterrupt:
        pass
    finally:
        analyzer.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
