#!/usr/bin/env python3
"""
Centralized Vision Processing Node

Eliminates image duplication by processing camera data once and distributing
results to multiple consumers (terrain analysis, keyboard detection, obstacle avoidance).

Optimizations:
- Single image decode operation
- Shared memory for large data structures
- Parallel processing branches
- Reduced memory allocations
- Zero-copy message passing where possible
"""

import math
import multiprocessing as mp
import time
from concurrent.futures import ThreadPoolExecutor
from threading import Lock
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_msgs.msg import Bool, Float32MultiArray, Header


class VisionProcessingNode(Node):
    """
    Centralized Vision Processing Node

    Processes camera images once and provides:
    - Keyboard detection results
    - Terrain classification maps
    - Obstacle detection data
    - Feature extraction for SLAM

    Optimized for minimal memory usage and maximum throughput.
    """

    def __init__(self):
        super().__init__("vision_processor")

        # Declare parameters
        self.declare_parameter("processing_rate_hz", 15.0)
        self.declare_parameter("enable_shared_memory", True)
        self.declare_parameter("use_zero_copy", True)
        self.declare_parameter("image_width", 640)  # Reduced resolution for performance
        self.declare_parameter("image_height", 480)
        self.declare_parameter("max_workers", 4)  # Parallel processing threads

        # Get parameters
        self.processing_rate = self.get_parameter("processing_rate_hz").value
        self.enable_shared_memory = self.get_parameter("enable_shared_memory").value
        self.use_zero_copy = self.get_parameter("use_zero_copy").value
        self.image_width = self.get_parameter("image_width").value
        self.image_height = self.get_parameter("image_height").value
        self.max_workers = self.get_parameter("max_workers").value

        # Optimized QoS for real-time vision processing
        vision_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,  # Allow drops for speed
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=3,  # Minimal buffer
            deadline=rclpy.duration.Duration(
                milliseconds=67
            ),  # 15Hz processing deadline
            lifespan=rclpy.duration.Duration(milliseconds=100),
        )

        # CV bridge for optimized image handling
        self.bridge = CvBridge()

        # Thread pool for parallel processing
        self.executor = ThreadPoolExecutor(max_workers=self.max_workers)

        # Shared memory for large data structures (if enabled)
        self.shared_memory_enabled = self.enable_shared_memory
        self.image_buffer = None
        self.result_cache = {}

        # Processing state
        self.last_processing_time = 0.0
        self.frame_count = 0
        self.processing_lock = Lock()

        # Subscribers (single image input)
        self.image_sub = self.create_subscription(
            Image, "/camera/image_raw", self.image_callback, vision_qos
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo, "/camera/camera_info", self.camera_info_callback, vision_qos
        )

        self.pointcloud_sub = self.create_subscription(
            PointCloud2, "/camera/depth/points", self.pointcloud_callback, vision_qos
        )

        # Publishers for processed results (smaller, optimized messages)
        self.keyboard_detection_pub = self.create_publisher(
            PoseStamped, "/vision/keyboard_pose", vision_qos
        )

        self.terrain_map_pub = self.create_publisher(
            OccupancyGrid, "/vision/terrain_map", vision_qos
        )

        self.obstacle_detection_pub = self.create_publisher(
            Float32MultiArray, "/vision/obstacles", vision_qos
        )

        self.feature_detection_pub = self.create_publisher(
            Float32MultiArray, "/vision/features", vision_qos
        )

        self.processing_status_pub = self.create_publisher(
            Bool, "/vision/processing_status", vision_qos
        )

        # Camera intrinsics
        self.camera_matrix = None
        self.dist_coeffs = None

        # Processing timer (controlled rate)
        self.processing_timer = self.create_timer(
            1.0 / self.processing_rate, self.process_pending_data
        )

        # Performance monitoring
        self.performance_stats = {
            "frames_processed": 0,
            "avg_processing_time": 0.0,
            "memory_usage": 0,
            "cpu_usage": 0.0,
        }

        self.get_logger().info("Centralized Vision Processing Node initialized")
        self.get_logger().info(
            f"Processing rate: {self.processing_rate}Hz, Workers: {self.max_workers}"
        )

    def camera_info_callback(self, msg: CameraInfo):
        """Update camera intrinsics for optimized processing."""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def image_callback(self, msg: Image):
        """Receive camera images for processing."""
        if self.shared_memory_enabled:
            # Store in shared buffer to avoid copying
            if self.image_buffer is None:
                # Pre-allocate buffer for performance
                self.image_buffer = np.zeros(
                    (self.image_height, self.image_width, 3), dtype=np.uint8
                )

            # Decode once and store
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Resize for performance if needed
            if cv_image.shape[:2] != (self.image_height, self.image_width):
                cv_image = cv2.resize(cv_image, (self.image_width, self.image_height))
            self.image_buffer[:] = cv_image
        else:
            # Traditional approach (less efficient)
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        self.frame_count += 1

    def pointcloud_callback(self, msg: PointCloud2):
        """Receive point cloud data for 3D processing."""
        # Store for combined vision processing
        self.latest_pointcloud = msg

    def process_pending_data(self):
        """Process accumulated vision data at controlled rate."""
        current_time = time.time()

        # Rate limiting
        if current_time - self.last_processing_time < (1.0 / self.processing_rate):
            return

        with self.processing_lock:
            try:
                start_time = time.time()

                # Get image data (zero-copy if possible)
                if self.shared_memory_enabled and self.image_buffer is not None:
                    image = self.image_buffer  # Direct reference, no copy
                elif hasattr(self, "latest_image"):
                    image = self.latest_image
                else:
                    return  # No image data available

                # Parallel processing of different vision tasks
                futures = []

                # Keyboard detection (high priority for autonomous typing)
                futures.append(
                    self.executor.submit(
                        self.detect_keyboard,
                        image.copy() if not self.shared_memory_enabled else image,
                    )
                )

                # Terrain analysis (medium priority)
                futures.append(
                    self.executor.submit(
                        self.analyze_terrain,
                        image.copy() if not self.shared_memory_enabled else image,
                    )
                )

                # Obstacle detection (high priority for safety)
                futures.append(
                    self.executor.submit(
                        self.detect_obstacles,
                        image.copy() if not self.shared_memory_enabled else image,
                    )
                )

                # Feature extraction for SLAM (background task)
                futures.append(
                    self.executor.submit(
                        self.extract_features,
                        image.copy() if not self.shared_memory_enabled else image,
                    )
                )

                # Wait for results and publish
                results = [future.result() for future in futures]

                # Publish results
                self.publish_keyboard_detection(results[0])
                self.publish_terrain_analysis(results[1])
                self.publish_obstacle_detection(results[2])
                self.publish_feature_extraction(results[3])

                # Update performance stats
                processing_time = time.time() - start_time
                self.update_performance_stats(processing_time)

                # Publish processing status
                status_msg = Bool()
                status_msg.data = True
                self.processing_status_pub.publish(status_msg)

                self.last_processing_time = current_time

            except Exception as e:
                self.get_logger().error(f"Vision processing error: {e}")
                # Publish failure status
                status_msg = Bool()
                status_msg.data = False
                self.processing_status_pub.publish(status_msg)

    def detect_keyboard(self, image: np.ndarray) -> Optional[Dict[str, Any]]:
        """Detect keyboard in image with optimized processing."""
        try:
            # Convert to HSV for color-based detection
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Keyboard detection logic (optimized)
            # Look for characteristic keyboard patterns
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            edges = cv2.Canny(gray, 50, 150)

            # Find rectangular regions (keyboard keys)
            contours, _ = cv2.findContours(
                edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            keyboard_contours = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if 100 < area < 5000:  # Keyboard key size range
                    perimeter = cv2.arcLength(contour, True)
                    approx = cv2.approxPolyDP(contour, 0.02 * perimeter, True)
                    if len(approx) == 4:  # Rectangular shape
                        keyboard_contours.append(contour)

            if len(keyboard_contours) > 10:  # Threshold for keyboard detection
                # Calculate keyboard center
                all_points = np.vstack(keyboard_contours)
                center_x = np.mean(all_points[:, :, 0])
                center_y = np.mean(all_points[:, :, 1])

                return {
                    "detected": True,
                    "center_x": float(center_x),
                    "center_y": float(center_y),
                    "confidence": min(1.0, len(keyboard_contours) / 50.0),
                }

            return {"detected": False}

        except Exception as e:
            self.get_logger().error(f"Keyboard detection error: {e}")
            return {"detected": False}

    def analyze_terrain(self, image: np.ndarray) -> Dict[str, Any]:
        """Analyze terrain with optimized processing."""
        try:
            # Convert to HSV for terrain classification
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # Simplified terrain classification (can be expanded)
            # Sand: bright, uniform colors
            sand_mask = cv2.inRange(hsv, (10, 30, 150), (40, 150, 255))

            # Rocks: darker, textured areas
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            texture = cv2.Laplacian(gray, cv2.CV_64F).var()

            # Create terrain map (simplified occupancy grid)
            height, width = image.shape[:2]
            terrain_map = np.zeros(
                (height // 10, width // 10), dtype=np.int8
            )  # Downsampled

            # Classify regions
            for i in range(0, height, 10):
                for j in range(0, width, 10):
                    if sand_mask[i, j] > 0:
                        terrain_map[i // 10, j // 10] = 10  # Traversable sand
                    elif texture > 100:
                        terrain_map[i // 10, j // 10] = 50  # Difficult rocks
                    else:
                        terrain_map[i // 10, j // 10] = 80  # Challenging terrain

            return {
                "terrain_map": terrain_map.flatten().tolist(),
                "width": terrain_map.shape[1],
                "height": terrain_map.shape[0],
                "resolution": 0.1,  # 10cm per cell
            }

        except Exception as e:
            self.get_logger().error(f"Terrain analysis error: {e}")
            return {"terrain_map": [], "width": 0, "height": 0, "resolution": 0.1}

    def detect_obstacles(self, image: np.ndarray) -> List[float]:
        """Detect obstacles with optimized processing."""
        try:
            # Convert to grayscale
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

            # Apply Gaussian blur to reduce noise
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)

            # Edge detection
            edges = cv2.Canny(blurred, 50, 150)

            # Find contours (potential obstacles)
            contours, _ = cv2.findContours(
                edges, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
            )

            obstacles = []
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > 500:  # Minimum obstacle size
                    # Get bounding box
                    x, y, w, h = cv2.boundingRect(contour)
                    # Convert to normalized coordinates (0-1)
                    obstacles.extend(
                        [
                            x / image.shape[1],  # center_x
                            y / image.shape[0],  # center_y
                            w / image.shape[1],  # width
                            h / image.shape[0],  # height
                            area / 10000.0,  # normalized area
                        ]
                    )

            return obstacles[:50]  # Limit to top 50 obstacles

        except Exception as e:
            self.get_logger().error(f"Obstacle detection error: {e}")
            return []

    def extract_features(self, image: np.ndarray) -> List[float]:
        """Extract features for SLAM with optimized processing."""
        try:
            # Use ORB for fast feature extraction
            orb = cv2.ORB_create(
                nfeatures=100, fastThreshold=20
            )  # Optimized parameters

            # Detect keypoints and descriptors
            keypoints, descriptors = orb.detectAndCompute(image, None)

            if descriptors is not None:
                # Convert descriptors to list for publishing
                # Take only first 32 features to reduce message size
                features = descriptors[:32].flatten().tolist()
                return features
            else:
                return []

        except Exception as e:
            self.get_logger().error(f"Feature extraction error: {e}")
            return []

    def publish_keyboard_detection(self, result: Dict[str, Any]):
        """Publish keyboard detection results."""
        if result.get("detected", False):
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "camera_link"

            # Convert image coordinates to camera frame (simplified)
            pose_msg.pose.position.x = (
                result["center_x"] - self.image_width / 2
            ) * 0.001  # 1mm per pixel approx
            pose_msg.pose.position.y = (
                result["center_y"] - self.image_height / 2
            ) * 0.001
            pose_msg.pose.position.z = 0.5  # Assume 50cm in front

            self.keyboard_detection_pub.publish(pose_msg)

    def publish_terrain_analysis(self, result: Dict[str, Any]):
        """Publish terrain analysis results."""
        if result.get("terrain_map"):
            map_msg = OccupancyGrid()
            map_msg.header.stamp = self.get_clock().now().to_msg()
            map_msg.header.frame_id = "camera_link"

            map_msg.info.width = result["width"]
            map_msg.info.height = result["height"]
            map_msg.info.resolution = result["resolution"]

            # Convert to int8 for occupancy grid (-1 = unknown, 0-100 = occupied)
            map_msg.data = [
                int(val) if val >= 0 else -1 for val in result["terrain_map"]
            ]

            self.terrain_map_pub.publish(map_msg)

    def publish_obstacle_detection(self, obstacles: List[float]):
        """Publish obstacle detection results."""
        if obstacles:
            array_msg = Float32MultiArray()
            array_msg.data = obstacles
            self.obstacle_detection_pub.publish(array_msg)

    def publish_feature_extraction(self, features: List[float]):
        """Publish feature extraction results."""
        if features:
            array_msg = Float32MultiArray()
            array_msg.data = features
            self.feature_detection_pub.publish(array_msg)

    def update_performance_stats(self, processing_time: float):
        """Update performance monitoring statistics."""
        self.performance_stats["frames_processed"] += 1
        self.performance_stats["avg_processing_time"] = (
            (
                self.performance_stats["avg_processing_time"]
                * (self.performance_stats["frames_processed"] - 1)
            )
            + processing_time
        ) / self.performance_stats["frames_processed"]

        # Log performance every 100 frames
        if self.performance_stats["frames_processed"] % 100 == 0:
            self.get_logger().info(
                f"Vision Performance: {self.performance_stats['avg_processing_time']:.3f}s avg, "
                f"{1.0/self.performance_stats['avg_processing_time']:.1f} FPS"
            )

    def destroy_node(self):
        """Clean shutdown with resource cleanup."""
        self.executor.shutdown(wait=True)
        if self.shared_memory_enabled and self.image_buffer is not None:
            # Clean up shared memory
            del self.image_buffer
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = VisionProcessingNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
