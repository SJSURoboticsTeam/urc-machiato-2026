"""
Computer Vision Subsystem Node with Conditional Dependencies.

URC 2026 Computer Vision implementation providing ArUco marker detection,
competition object recognition, and real-time vision processing.

Supports mission-based conditional loading of heavy ML dependencies.
"""

import math
from typing import List, Optional, Tuple, Dict, Any

import cv2
import numpy as np
import rclpy
from autonomy_interfaces.msg import VisionDetection
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Header, String

# Import feature flag system
try:
    from src.core.feature_flags import get_feature_flag_manager, is_feature_enabled, require_feature, with_feature
    FEATURE_FLAGS_AVAILABLE = True
except ImportError:
    FEATURE_FLAGS_AVAILABLE = False

# Conditional imports for heavy dependencies
try:
    from src.core.mission_resource_manager import get_mission_resource_manager
    RESOURCE_MANAGER_AVAILABLE = True
except ImportError:
    RESOURCE_MANAGER_AVAILABLE = False

# Conditional ML library imports - loaded only when needed
TORCH_AVAILABLE = False
DETECTRON_AVAILABLE = False
TENSORFLOW_AVAILABLE = False

def _load_ml_dependencies():
    """Conditionally load heavy ML dependencies based on mission requirements."""
    global TORCH_AVAILABLE, DETECTRON_AVAILABLE, TENSORFLOW_AVAILABLE

    if not RESOURCE_MANAGER_AVAILABLE:
        # Load all dependencies if no resource manager (backward compatibility)
        try:
            import torch
            TORCH_AVAILABLE = True
        except ImportError:
            pass

        try:
            import detectron2
            DETECTRON_AVAILABLE = True
        except ImportError:
            pass

        try:
            import tensorflow as tf
            TENSORFLOW_AVAILABLE = True
        except ImportError:
            pass

        return

    # Check mission profile for vision requirements
    resource_manager = get_mission_resource_manager()
    mission_profile = resource_manager.current_mission_profile

    if not mission_profile:
        # No active mission, load minimal dependencies
        return

    # Load dependencies based on mission requirements
    config = resource_manager.config
    mission_config = config.get('mission_profiles', {}).get(mission_profile, {})

    if mission_config.get('computer_vision_enabled', False):
        # Load PyTorch/TorchVision for ML-based vision
        try:
            import torch
            import torchvision
            TORCH_AVAILABLE = True
        except ImportError:
            pass

        # Load Detectron2 for advanced object detection
        try:
            import detectron2
            DETECTRON_AVAILABLE = True
        except ImportError:
            pass

        # Load TensorFlow for terrain analysis
        try:
            import tensorflow as tf
            TENSORFLOW_AVAILABLE = True
        except ImportError:
            pass

# Load dependencies on module import
_load_ml_dependencies()


class ComputerVisionNode(Node):
    """
    Computer vision processing for URC 2026 competition.

    Provides ArUco marker detection, competition object recognition,
    and real-time vision processing for navigation assistance.
    """

    def __init__(self) -> None:
        """Initialize computer vision node with camera interfaces."""
        super().__init__("computer_vision_node")

        # Publishers
        self.detection_publisher = self.create_publisher(
            VisionDetection, "vision/detections", 10
        )
        self.status_publisher = self.create_publisher(
            String, "computer_vision/status", 10
        )
        self.debug_image_publisher = self.create_publisher(
            Image, "vision/debug_image", 10
        )

        # Camera data republishers (for data flow and monitoring)
        self.camera_image_republisher = self.create_publisher(
            Image, "camera/image_raw", 10
        )
        self.camera_depth_republisher = self.create_publisher(
            Image, "camera/depth/image_raw", 10
        )

        # Initialize vision capabilities based on available dependencies
        self._initialize_vision_capabilities()
        self.camera_info_republisher = self.create_publisher(
            CameraInfo, "camera/camera_info", 10
        )

        # Subscribers
        self.image_subscription = self.create_subscription(
            Image, "camera/image_raw", self.image_callback, 10
        )
        self.depth_subscription = self.create_subscription(
            Image, "camera/depth/image_raw", self.depth_callback, 10
        )
        self.camera_info_subscription = self.create_subscription(
            CameraInfo, "camera/camera_info", self.camera_info_callback, 10
        )

        # CV Bridge for ROS/OpenCV conversion
        self.bridge = CvBridge()

        # ArUco marker detection setup
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Camera parameters (placeholder - would be calibrated)
        self.camera_matrix = np.array(
            [[600, 0, 320], [0, 600, 240], [0, 0, 1]], dtype=np.float32
        )
        self.dist_coeffs = np.zeros((5, 1), dtype=np.float32)

        # Competition targets
        self.target_objects = {
            "mallet": {"color": "orange", "shape": "cylindrical"},
            "water_bottle": {"color": "various", "shape": "cylindrical"},
        }

        # Processing state
        self.last_image: Optional[np.ndarray] = None
        self.current_detections: List[VisionDetection] = []

        # Control timer
        self.processing_timer = self.create_timer(0.1, self.process_image)  # 10 Hz
        self.status_timer = self.create_timer(1.0, self.status_callback)

        self.get_logger().info("Computer Vision node initialized")

    def _initialize_vision_capabilities(self):
        """Initialize vision capabilities based on available dependencies and mission profile."""

        # Track available capabilities
        self.capabilities = {
            'basic_cv': True,  # OpenCV always available
            'aruco_detection': True,  # Basic ArUco detection
            'ml_vision': TORCH_AVAILABLE,
            'object_detection': DETECTRON_AVAILABLE,
            'terrain_analysis': TENSORFLOW_AVAILABLE,
        }

        # Initialize ML models if available
        self.ml_models = {}
        if TORCH_AVAILABLE:
            try:
                self._initialize_torch_models()
                self.get_logger().info("PyTorch models initialized")
            except Exception as e:
                self.get_logger().warning(f"Failed to initialize PyTorch models: {e}")

        if DETECTRON_AVAILABLE:
            try:
                self._initialize_detectron_models()
                self.get_logger().info("Detectron2 models initialized")
            except Exception as e:
                self.get_logger().warning(f"Failed to initialize Detectron2 models: {e}")

        if TENSORFLOW_AVAILABLE:
            try:
                self._initialize_tensorflow_models()
                self.get_logger().info("TensorFlow models initialized")
            except Exception as e:
                self.get_logger().warning(f"Failed to initialize TensorFlow models: {e}")

        # Set processing rate based on capabilities and mission profile
        self._configure_processing_rate()

        # Log available capabilities
        available_caps = [cap for cap, available in self.capabilities.items() if available]
        self.get_logger().info(f"Vision capabilities initialized: {', '.join(available_caps)}")

    def _initialize_torch_models(self):
        """Initialize PyTorch models for computer vision."""
        if not TORCH_AVAILABLE:
            return

        import torch
        import torchvision

        # Placeholder for model initialization
        # In real implementation, load pre-trained models for object detection
        self.ml_models['torch_object_detector'] = None  # Placeholder

    def _initialize_detectron_models(self):
        """Initialize Detectron2 models."""
        if not DETECTRON_AVAILABLE:
            return

        # Placeholder for Detectron2 model initialization
        self.ml_models['detectron_detector'] = None  # Placeholder

    def _initialize_tensorflow_models(self):
        """Initialize TensorFlow models for terrain analysis."""
        if not TENSORFLOW_AVAILABLE:
            return

        import tensorflow as tf

        # Placeholder for TensorFlow model initialization
        self.ml_models['terrain_analyzer'] = None  # Placeholder

    def _configure_processing_rate(self):
        """Configure processing rate based on mission requirements."""
        if RESOURCE_MANAGER_AVAILABLE:
            resource_manager = get_mission_resource_manager()
            mission_profile = resource_manager.current_mission_profile

            if mission_profile:
                config = resource_manager.config
                mission_config = config.get('mission_profiles', {}).get(mission_profile, {})
                fps = mission_config.get('vision_fps', 10)

                # Update timer rate
                if hasattr(self, 'processing_timer'):
                    # Cancel existing timer and create new one with updated rate
                    self.processing_timer.cancel()
                    interval = 1.0 / fps
                    self.processing_timer = self.create_timer(interval, self.process_image)
                    self.get_logger().info(f"Vision processing rate set to {fps} FPS for mission {mission_profile}")

    def image_callback(self, msg: Image):
        """Handle incoming camera images."""
        try:
            self.last_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")

            # Republish camera image for other nodes
            self.camera_image_republisher.publish(msg)

            # Create and publish camera info (placeholder)
            camera_info = CameraInfo()
            camera_info.header = msg.header
            camera_info.width = msg.width
            camera_info.height = msg.height
            camera_info.distortion_model = "plumb_bob"
            camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]  # No distortion
            camera_info.k = [
                600.0,
                0.0,
                320.0,
                0.0,
                600.0,
                240.0,
                0.0,
                0.0,
                1.0,
            ]  # Camera matrix
            camera_info.r = [
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
                0.0,
                0.0,
                1.0,
            ]  # Rectification matrix
            camera_info.p = [
                600.0,
                0.0,
                320.0,
                0.0,
                0.0,
                600.0,
                240.0,
                0.0,
                0.0,
                0.0,
                1.0,
                0.0,
            ]  # Projection matrix

            self.camera_info_republisher.publish(camera_info)

        except Exception as e:
            self.get_logger().error(f"Failed to convert image: {e}")

    def depth_callback(self, msg: Image):
        """Handle incoming depth images."""
        # Republish depth image for other nodes
        self.camera_depth_republisher.publish(msg)

    def camera_info_callback(self, msg: CameraInfo):
        """Handle incoming camera info."""
        # Republish camera info for other nodes
        self.camera_info_republisher.publish(msg)

    @with_feature('computer_vision')
    def process_image(self):
        """Main vision processing loop."""
        if self.last_image is None:
            return

        # Check if computer vision is enabled for current mission
        if FEATURE_FLAGS_AVAILABLE and not is_feature_enabled('computer_vision'):
            self.get_logger().debug("Computer vision disabled for current mission")
            return

        # Clear previous detections
        self.current_detections.clear()

        # Process ArUco markers (always available as basic feature)
        self.detect_aruco_markers()

        # Process competition objects (only if ML vision is enabled)
        if not FEATURE_FLAGS_AVAILABLE or is_feature_enabled('ml_vision'):
            self.detect_competition_objects()
        else:
            # Use lightweight object detection
            self.detect_competition_objects_lightweight()

        # Publish detections
        for detection in self.current_detections:
            self.detection_publisher.publish(detection)

        # Publish debug image (optional)
        if self.debug_image_publisher.get_subscription_count() > 0:
            self.publish_debug_image()

    def detect_aruco_markers(self):
        """Detect ArUco markers in the image."""
        if self.last_image is None:
            return

        # Convert to grayscale
        gray = cv2.cvtColor(self.last_image, cv2.COLOR_BGR2GRAY)

        # Detect markers
        corners, ids, rejected = self.detector.detectMarkers(gray)

        if ids is not None:
            # Estimate pose for each marker
            for i, marker_id in enumerate(ids.flatten()):
                # Pose estimation
                rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                    corners[i], 0.05, self.camera_matrix, self.dist_coeffs
                )

                if rvecs is not None and tvecs is not None:
                    # Convert rotation vector to quaternion
                    rotation_matrix, _ = cv2.Rodrigues(rvecs[0])
                    quaternion = self.rotation_matrix_to_quaternion(rotation_matrix)

                    # Create detection message
                    detection = VisionDetection()
                    detection.header = self.create_header()
                    detection.object_type = f"aruco_{marker_id}"
                    detection.confidence = 0.95
                    detection.position.x = tvecs[0][0][0]
                    detection.position.y = tvecs[0][0][1]
                    detection.position.z = tvecs[0][0][2]
                    detection.orientation.x = quaternion[0]
                    detection.orientation.y = quaternion[1]
                    detection.orientation.z = quaternion[2]
                    detection.orientation.w = quaternion[3]

                    self.current_detections.append(detection)

                    # Draw marker on debug image
                    cv2.aruco.drawDetectedMarkers(
                        self.last_image, [corners[i]], ids[i : i + 1]
                    )
                    cv2.aruco.drawAxis(
                        self.last_image,
                        self.camera_matrix,
                        self.dist_coeffs,
                        rvecs[0],
                        tvecs[0],
                        0.1,
                    )

    def detect_competition_objects(self):
        """Detect competition objects (mallet, water bottle)."""
        if self.last_image is None:
            return

        # Convert to HSV for color-based detection
        hsv = cv2.cvtColor(self.last_image, cv2.COLOR_BGR2HSV)

        # Detect orange mallet (placeholder implementation)
        # In practice, this would use trained ML models or sophisticated CV algorithms
        orange_lower = np.array([5, 50, 50])
        orange_upper = np.array([15, 255, 255])
        orange_mask = cv2.inRange(hsv, orange_lower, orange_upper)

        # Find contours in mask
        contours, _ = cv2.findContours(
            orange_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 500:  # Minimum area threshold
                # Get bounding rectangle
                x, y, w, h = cv2.boundingRect(contour)

                # Estimate position (simplified - would use depth sensing in practice)
                distance = 2.0  # Placeholder distance estimate
                angle = (
                    (x + w / 2) - self.last_image.shape[1] / 2
                ) * 0.002  # Rough angle estimate

                # Create detection message
                detection = VisionDetection()
                detection.header = self.create_header()
                detection.object_type = "mallet"
                detection.confidence = 0.7  # Lower confidence for color-based detection
                detection.position.x = distance * math.sin(angle)
                detection.position.y = 0.0
                detection.position.z = distance * math.cos(angle)
                detection.orientation.w = 1.0  # No rotation info

                self.current_detections.append(detection)

                # Draw bounding box on debug image
                cv2.rectangle(self.last_image, (x, y), (x + w, y + h), (0, 255, 255), 2)
                cv2.putText(
                    self.last_image,
                    "Mallet",
                    (x, y - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 255),
                    2,
                )

    def detect_competition_objects_lightweight(self):
        """Lightweight object detection using basic computer vision without ML."""
        if self.last_image is None:
            return

        # Import lightweight vision processor
        try:
            from src.autonomy.perception.computer_vision.lightweight_vision import LightweightVisionProcessor
            processor = LightweightVisionProcessor()
        except ImportError:
            self.get_logger().warning("Lightweight vision processor not available")
            return

        # Process frame with lightweight algorithms
        results = processor.process_frame_lightweight(self.last_image)

        # Convert results to ROS messages
        for obj in results.get('objects', []):
            detection = VisionDetection()
            detection.header = self.create_header()
            detection.object_type = obj.get('color', 'unknown') + "_object"
            detection.confidence = obj.get('confidence', 0.5)

            # Convert centroid to rough position estimate
            cx, cy = obj.get('centroid', (320, 240))
            distance = 2.0  # Placeholder distance
            angle = (cx - self.last_image.shape[1] / 2) * 0.002

            detection.position.x = distance * math.sin(angle)
            detection.position.y = 0.0
            detection.position.z = distance * math.cos(angle)
            detection.orientation.w = 1.0

            self.current_detections.append(detection)

            # Draw bounding box on debug image
            bbox = obj.get('bbox', (0, 0, 10, 10))
            cv2.rectangle(self.last_image, (bbox[0], bbox[1]),
                         (bbox[0] + bbox[2], bbox[1] + bbox[3]), (255, 0, 0), 2)
            cv2.putText(
                self.last_image,
                f"{detection.object_type}",
                (bbox[0], bbox[1] - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (255, 0, 0),
                2,
            )

    def rotation_matrix_to_quaternion(
        self, R: np.ndarray
    ) -> Tuple[float, float, float, float]:
        """Convert rotation matrix to quaternion."""
        q = np.zeros(4)
        trace = np.trace(R)
        if trace > 0:
            s = 0.5 / math.sqrt(trace + 1.0)
            q[3] = 0.25 / s
            q[0] = (R[2, 1] - R[1, 2]) * s
            q[1] = (R[0, 2] - R[2, 0]) * s
            q[2] = (R[1, 0] - R[0, 1]) * s
        else:
            if R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
                s = 2.0 * math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
                q[3] = (R[2, 1] - R[1, 2]) / s
                q[0] = 0.25 * s
                q[1] = (R[0, 1] + R[1, 0]) / s
                q[2] = (R[0, 2] + R[2, 0]) / s
            elif R[1, 1] > R[2, 2]:
                s = 2.0 * math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
                q[3] = (R[0, 2] - R[2, 0]) / s
                q[0] = (R[0, 1] + R[1, 0]) / s
                q[1] = 0.25 * s
                q[2] = (R[1, 2] + R[2, 1]) / s
            else:
                s = 2.0 * math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
                q[3] = (R[1, 0] - R[0, 1]) / s
                q[0] = (R[0, 2] + R[2, 0]) / s
                q[1] = (R[1, 2] + R[2, 1]) / s
                q[2] = 0.25 * s
        return tuple(q)

    def publish_debug_image(self):
        """Publish debug image with detections overlaid."""
        if self.last_image is not None:
            debug_msg = self.bridge.cv2_to_imgmsg(self.last_image, encoding="bgr8")
            debug_msg.header = self.create_header()
            self.debug_image_publisher.publish(debug_msg)

    def create_header(self) -> Header:
        """Create a standard ROS2 header."""
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = "camera"
        return header

    def status_callback(self):
        """Publish vision system status."""
        status_msg = String()
        num_detections = len(self.current_detections)
        status_msg.data = f"Vision operational - {num_detections} detections"
        self.status_publisher.publish(status_msg)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    node = ComputerVisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
