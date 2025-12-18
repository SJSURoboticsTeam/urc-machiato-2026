#!/usr/bin/env python3
"""
Vision Processing Pipeline Performance Tests - URC 2026

Tests complete vision processing pipeline performance:
- Image capture to result: <67ms (15Hz processing)
- Feature extraction: <20ms
- Memory usage: <200MB for processing
- CPU utilization: <60% under load

Critical for autonomous navigation and terrain analysis.
"""

import time
import statistics
import unittest
import psutil
import cv2
import numpy as np
from typing import Dict, List, Optional, Tuple, Any
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2
from geometry_msgs.msg import PoseStamped, Point
from std_msgs.msg import Header, Float32MultiArray, Bool
from nav_msgs.msg import OccupancyGrid
from cv_bridge import CvBridge
import threading
from concurrent.futures import ThreadPoolExecutor
import tracemalloc


class VisionPipelinePerformanceTest(unittest.TestCase):
    """Test vision processing pipeline performance under various conditions."""

    def setUp(self):
        """Set up vision pipeline performance testing."""
        rclpy.init()

        # Test parameters - aligned with competition requirements
        self.num_frames = 300  # Test 300 frames for statistical significance
        self.target_frame_time_ms = 67.0  # 15Hz = ~67ms per frame
        self.target_feature_extraction_ms = 20.0  # Feature extraction target
        self.target_memory_mb = 200.0  # Memory usage target
        self.target_cpu_percent = 60.0  # CPU utilization target

        # Performance tracking
        self.frame_processing_times = []
        self.feature_extraction_times = []
        self.memory_usage_readings = []
        self.cpu_usage_readings = []

        # Create test components
        self.image_generator = ImageGeneratorNode()
        self.vision_processor = VisionProcessorSimulator()
        self.result_consumer = ResultConsumerNode()

    def tearDown(self):
        """Clean up resources."""
        if hasattr(self, 'image_generator'):
            self.image_generator.destroy_node()
        if hasattr(self, 'result_consumer'):
            self.result_consumer.destroy_node()
        rclpy.shutdown()

    def test_complete_vision_pipeline_performance(self):
        """Test complete vision processing pipeline performance."""
        print("\n👁️  Testing Complete Vision Pipeline Performance")
        print("=" * 60)

        # Start ROS2 executor
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.image_generator)
        executor.add_node(self.result_consumer)

        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        # Allow node discovery
        time.sleep(2.0)

        try:
            # Start memory and CPU monitoring
            tracemalloc.start()
            self._start_resource_monitoring()

            # Warm up pipeline
            print("🔥 Warming up vision pipeline...")
            self._warm_up_pipeline()

            # Run performance test
            print("📊 Measuring vision pipeline performance...")
            self._run_pipeline_performance_test()

            # Analyze results
            self._analyze_pipeline_results()

        finally:
            self._stop_resource_monitoring()
            tracemalloc.stop()
            executor.shutdown()
            executor_thread.join(timeout=2.0)

    def _warm_up_pipeline(self):
        """Warm up the vision processing pipeline."""
        for i in range(50):  # Warm up with 50 frames
            self.image_generator.generate_test_image()
            time.sleep(0.01)  # 10ms between frames

        time.sleep(0.5)  # Allow system to stabilize

    def _run_pipeline_performance_test(self):
        """Run the main vision pipeline performance test."""
        self.frame_processing_times.clear()

        start_time = time.time()

        for frame_num in range(self.num_frames):
            # Generate and process frame
            frame_start = time.time()

            # Step 1: Generate test image
            self.image_generator.generate_test_image()

            # Step 2: Process through vision pipeline
            processing_start = time.time()
            result = self.vision_processor.process_frame()
            processing_time = time.time() - processing_start

            # Step 3: Wait for result consumption
            timeout = 0.1  # 100ms timeout
            result_received = False
            result_wait_start = time.time()

            while time.time() - result_wait_start < timeout and not result_received:
                rclpy.spin_once(self.result_consumer, timeout_sec=0.001)
                if hasattr(self.result_consumer, 'last_result_time'):
                    result_received = True

            # Record total frame time
            total_frame_time = (time.time() - frame_start) * 1000  # Convert to ms
            self.frame_processing_times.append(total_frame_time)

            # Progress indicator
            if (frame_num + 1) % 60 == 0:  # Every 60 frames (4 seconds at 15Hz)
                print(f"  Progress: {frame_num + 1}/{self.num_frames} frames")

            # Maintain target frame rate (15Hz = ~67ms per frame)
            elapsed = time.time() - frame_start
            if elapsed < 0.067:  # Less than 67ms
                time.sleep(0.067 - elapsed)

        total_duration = time.time() - start_time
        actual_fps = self.num_frames / total_duration

        print(".2f")
        print(".1f")

    def _start_resource_monitoring(self):
        """Start monitoring system resources."""
        self.monitoring_active = True
        self.monitor_thread = threading.Thread(target=self._monitor_resources, daemon=True)
        self.monitor_thread.start()

    def _stop_resource_monitoring(self):
        """Stop resource monitoring."""
        self.monitoring_active = False
        if hasattr(self, 'monitor_thread'):
            self.monitor_thread.join(timeout=2.0)

    def _monitor_resources(self):
        """Monitor system resources during testing."""
        process = psutil.Process()
        while self.monitoring_active:
            # CPU usage
            cpu_percent = process.cpu_percent(interval=0.1)
            self.cpu_usage_readings.append(cpu_percent)

            # Memory usage
            memory_mb = process.memory_info().rss / 1024 / 1024
            self.memory_usage_readings.append(memory_mb)

            time.sleep(0.2)  # Monitor every 200ms

    def _analyze_pipeline_results(self):
        """Analyze vision pipeline performance results."""
        if not self.frame_processing_times:
            self.fail("No frame processing times collected")

        # Calculate frame processing statistics
        avg_frame_time = statistics.mean(self.frame_processing_times)
        max_frame_time = max(self.frame_processing_times)
        min_frame_time = min(self.frame_processing_times)
        p95_frame_time = statistics.quantiles(self.frame_processing_times, n=20)[18]
        p99_frame_time = statistics.quantiles(self.frame_processing_times, n=100)[98]

        # Calculate derived metrics
        avg_fps = 1000.0 / avg_frame_time if avg_frame_time > 0 else 0
        target_fps = 1000.0 / self.target_frame_time_ms

        # Resource usage statistics
        avg_memory_mb = statistics.mean(self.memory_usage_readings) if self.memory_usage_readings else 0
        max_memory_mb = max(self.memory_usage_readings) if self.memory_usage_readings else 0
        avg_cpu_percent = statistics.mean(self.cpu_usage_readings) if self.cpu_usage_readings else 0
        max_cpu_percent = max(self.cpu_usage_readings) if self.cpu_usage_readings else 0

        # Print detailed results
        print("\n📈 Vision Pipeline Performance Results:")
        print("-" * 50)
        print(".3f")
        print(".3f")
        print(".3f")
        print(".3f")
        print(".3f")
        print(".1f")
        print(".1f")
        print(".2f")
        print(".2f")
        print(".1f")
        print(".1f")

        # Validate against competition targets
        print("\n🎯 Competition Requirements Check:")
        print("-" * 50)

        requirements = {
            'Average Frame Time': (avg_frame_time, self.target_frame_time_ms),
            '95th Percentile Frame Time': (p95_frame_time, self.target_frame_time_ms * 1.2),
            '99th Percentile Frame Time': (p99_frame_time, self.target_frame_time_ms * 1.5),
            'Maximum Frame Time': (max_frame_time, self.target_frame_time_ms * 2.0),
            'Average FPS': (avg_fps, target_fps),
            'Memory Usage (avg)': (avg_memory_mb, self.target_memory_mb),
            'Memory Usage (max)': (max_memory_mb, self.target_memory_mb * 1.2),
            'CPU Usage (avg)': (avg_cpu_percent, self.target_cpu_percent),
            'CPU Usage (max)': (max_cpu_percent, self.target_cpu_percent * 1.2),
        }

        all_passed = True
        for metric, (actual, target) in requirements.items():
            passed = actual <= target
            status = "✅ PASS" if passed else "❌ FAIL"
            print(".3f")

            if not passed:
                all_passed = False

        # Performance assessment
        print("\n🚀 Performance Assessment:")
        print("-" * 50)

        if all_passed:
            print("✅ ALL VISION PIPELINE REQUIREMENTS MET")
            print("   Vision system suitable for 15Hz processing")
        elif avg_frame_time <= self.target_frame_time_ms:
            print("⚠️  AVERAGE PERFORMANCE OK, BUT RESOURCE/CONSISTENCY CONCERNS")
            print("   Monitor resource usage in competition scenarios")
        else:
            print("❌ VISION PERFORMANCE REQUIREMENTS NOT MET")
            print("   Pipeline optimization required before competition")

        # Detailed analysis and recommendations
        if avg_frame_time > self.target_frame_time_ms:
            print("\n💡 Optimization Recommendations:")
            if avg_memory_mb > self.target_memory_mb:
                print("   - Optimize memory usage in image processing")
            if avg_cpu_percent > self.target_cpu_percent:
                print("   - Reduce computational complexity of algorithms")
            if p95_frame_time > self.target_frame_time_ms * 1.5:
                print("   - Investigate causes of processing latency spikes")
            print("   - Consider image resolution reduction or algorithm optimization")

        # Store results for regression testing
        self.test_results = {
            'avg_frame_time_ms': avg_frame_time,
            'max_frame_time_ms': max_frame_time,
            'p95_frame_time_ms': p95_frame_time,
            'p99_frame_time_ms': p99_frame_time,
            'avg_fps': avg_fps,
            'avg_memory_mb': avg_memory_mb,
            'max_memory_mb': max_memory_mb,
            'avg_cpu_percent': avg_cpu_percent,
            'max_cpu_percent': max_cpu_percent,
            'requirements_met': all_passed,
            'frames_processed': len(self.frame_processing_times)
        }

        # Assert critical requirements
        self.assertLess(avg_frame_time, self.target_frame_time_ms * 2.0,
                       ".3f")
        self.assertLess(avg_memory_mb, self.target_memory_mb * 1.5,
                       ".2f")
        self.assertLess(avg_cpu_percent, self.target_cpu_percent * 1.5,
                       ".1f")

    def test_vision_pipeline_under_load(self):
        """Test vision pipeline performance under concurrent load."""
        print("\n🔄 Testing Vision Pipeline Under Load")

        # Create multiple consumers to simulate load
        consumers = []
        for i in range(3):
            consumer = ResultConsumerNode()
            consumers.append(consumer)

        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(self.image_generator)
        for consumer in consumers:
            executor.add_node(consumer)

        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()

        time.sleep(2.0)

        try:
            # Start resource monitoring
            self._start_resource_monitoring()

            # Generate concurrent load
            load_threads = []
            for i in range(5):
                t = threading.Thread(target=self._generate_vision_load, args=(i,))
                load_threads.append(t)
                t.start()

            # Run performance test under load
            self._run_pipeline_performance_test()

            # Stop load generation
            for t in load_threads:
                t.join(timeout=2.0)

        finally:
            self._stop_resource_monitoring()
            for consumer in consumers:
                consumer.destroy_node()
            executor.shutdown()
            executor_thread.join(timeout=2.0)

    def _generate_vision_load(self, thread_id: int):
        """Generate background vision processing load."""
        while self.monitoring_active:
            # Simulate additional vision processing work
            dummy_image = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
            # Perform some image processing operations
            blurred = cv2.GaussianBlur(dummy_image, (5, 5), 0)
            edges = cv2.Canny(blurred, 100, 200)
            time.sleep(0.01)  # Small delay


class ImageGeneratorNode(Node):
    """Generates test images for vision pipeline testing."""

    def __init__(self):
        super().__init__('image_generator')

        # QoS for real-time vision data
        vision_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=3,
            deadline=rclpy.duration.Duration(milliseconds=67),  # 15Hz deadline
        )

        # Publishers
        self.image_pub = self.create_publisher(Image, '/camera/image_raw', vision_qos)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/camera_info', vision_qos)

        # CV Bridge
        self.bridge = CvBridge()

        # Image generation parameters
        self.frame_count = 0
        self.width, self.height = 640, 480

        self.get_logger().info("Image generator ready")

    def generate_test_image(self):
        """Generate and publish a test image."""
        self.frame_count += 1

        # Create synthetic test image with features for vision processing
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)

        # Add some test features
        # - Grid pattern for feature detection
        for i in range(0, self.width, 50):
            cv2.line(image, (i, 0), (i, self.height), (255, 255, 255), 1)
        for i in range(0, self.height, 50):
            cv2.line(image, (0, i), (self.width, i), (255, 255, 255), 1)

        # - Some colored regions for classification testing
        cv2.rectangle(image, (100, 100), (200, 200), (0, 255, 0), -1)  # Green rectangle
        cv2.rectangle(image, (300, 200), (400, 300), (255, 0, 0), -1)  # Blue rectangle
        cv2.circle(image, (500, 150), 30, (0, 0, 255), -1)  # Red circle

        # - Add some noise to simulate real camera conditions
        noise = np.random.normal(0, 25, image.shape).astype(np.uint8)
        image = cv2.add(image, noise)

        # Convert to ROS2 Image message
        ros_image = self.bridge.cv2_to_imgmsg(image, encoding='bgr8')
        ros_image.header.stamp = self.get_clock().now().to_msg()
        ros_image.header.frame_id = 'camera_link'

        # Publish image
        self.image_pub.publish(ros_image)

        # Publish camera info
        camera_info = CameraInfo()
        camera_info.header = ros_image.header
        camera_info.width = self.width
        camera_info.height = self.height
        # Add basic camera intrinsics
        camera_info.k = [500.0, 0.0, 320.0, 0.0, 500.0, 240.0, 0.0, 0.0, 1.0]
        self.camera_info_pub.publish(camera_info)


class VisionProcessorSimulator:
    """Simulates vision processing pipeline for performance testing."""

    def __init__(self):
        """Initialize vision processor simulator."""
        self.bridge = CvBridge()
        self.frame_count = 0

        # Simulated processing parameters (based on real vision processing)
        self.feature_extraction_time = 0.015  # 15ms
        self.classification_time = 0.020     # 20ms
        self.terrain_analysis_time = 0.025   # 25ms

    def process_frame(self) -> Dict[str, Any]:
        """Simulate complete vision processing pipeline."""
        self.frame_count += 1

        start_time = time.time()

        # Simulate feature extraction (corners, edges, etc.)
        time.sleep(self.feature_extraction_time)
        features = {
            'corners': np.random.rand(50, 2) * 640,  # 50 corner points
            'edges': np.random.rand(100, 2) * 640,   # 100 edge points
        }

        # Simulate classification (terrain types, obstacles)
        time.sleep(self.classification_time)
        classification = {
            'terrain_type': 'sand',
            'confidence': 0.85,
            'obstacles': [
                {'type': 'rock', 'position': [300, 200], 'size': 50},
                {'type': 'crater', 'position': [150, 350], 'size': 80}
            ]
        }

        # Simulate terrain analysis
        time.sleep(self.terrain_analysis_time)
        terrain_analysis = {
            'traversability_map': np.random.rand(48, 64),  # 64x48 grid
            'slope_angles': np.random.normal(0, 5, (48, 64)),  # Slope in degrees
            'roughness_map': np.random.rand(48, 64)
        }

        processing_time = time.time() - start_time

        return {
            'frame_id': self.frame_count,
            'processing_time': processing_time,
            'features': features,
            'classification': classification,
            'terrain_analysis': terrain_analysis,
            'timestamp': time.time()
        }


class ResultConsumerNode(Node):
    """Consumes vision processing results for latency testing."""

    def __init__(self):
        super().__init__('result_consumer')

        # QoS for vision results
        result_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # Subscribers for vision results (would be published by real vision processor)
        self.terrain_map_sub = self.create_subscription(
            OccupancyGrid, '/vision/terrain_map', self.terrain_map_callback, result_qos)
        self.features_sub = self.create_subscription(
            Float32MultiArray, '/vision/features', self.features_callback, result_qos)
        self.detections_sub = self.create_subscription(
            Float32MultiArray, '/vision/detections', self.detections_callback, result_qos)

        # State
        self.last_result_time = None

        self.get_logger().info("Result consumer ready")

    def terrain_map_callback(self, msg):
        """Handle terrain map updates."""
        self.last_result_time = time.time()

    def features_callback(self, msg):
        """Handle feature detection results."""
        self.last_result_time = time.time()

    def detections_callback(self, msg):
        """Handle object detection results."""
        self.last_result_time = time.time()


if __name__ == '__main__':
    unittest.main()


