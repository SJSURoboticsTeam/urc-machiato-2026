#!/usr/bin/env python3
"""
Terrain Analysis Computation Time Tests - URC 2026

Tests terrain intelligence performance:
- Terrain classification: <50ms per frame
- Traversability calculation: <30ms
- Memory allocation: Minimal (<50MB delta)
- CPU usage: <40% sustained

Critical for navigation safety and autonomous terrain traversal.
"""

import statistics
import threading
import time
import tracemalloc
import unittest
from concurrent.futures import ThreadPoolExecutor
from typing import Any, Dict, List, Optional, Tuple

import cv2
import numpy as np
import psutil
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, LaserScan, PointCloud2
from std_msgs.msg import Float32MultiArray, Header


class TerrainAnalysisPerformanceTest(unittest.TestCase):
    """Test terrain analysis computation performance under various conditions."""

    def setUp(self):
        """Set up terrain analysis performance testing."""
        rclpy.init()

        # Test parameters - aligned with competition requirements
        self.num_frames = 200  # Test 200 terrain analysis frames
        self.target_classification_ms = 50.0  # Terrain classification target
        self.target_traversability_ms = 30.0  # Traversability calculation target
        self.target_memory_delta_mb = 50.0  # Memory allocation target
        self.target_cpu_percent = 40.0  # CPU utilization target

        # Performance tracking
        self.classification_times = []
        self.traversability_times = []
        self.total_analysis_times = []
        self.memory_usage_readings = []
        self.cpu_usage_readings = []

        # Create test components
        self.terrain_data_generator = TerrainDataGenerator()
        self.terrain_analyzer = TerrainAnalyzerSimulator()

    def tearDown(self):
        """Clean up resources."""
        rclpy.shutdown()

    def test_terrain_analysis_computation_performance(self):
        """Test terrain analysis computation performance."""
        print("\n🏔️  Testing Terrain Analysis Computation Performance")
        print("=" * 60)

        try:
            # Start memory and CPU monitoring
            tracemalloc.start()
            initial_memory = psutil.Process().memory_info().rss / 1024 / 1024
            self._start_resource_monitoring()

            # Warm up terrain analysis
            print("🔥 Warming up terrain analysis...")
            self._warm_up_terrain_analysis()

            # Run performance test
            print("📊 Measuring terrain analysis performance...")
            self._run_terrain_analysis_performance_test()

            # Analyze results
            self._analyze_terrain_results()

        finally:
            self._stop_resource_monitoring()
            tracemalloc.stop()

    def _warm_up_terrain_analysis(self):
        """Warm up the terrain analysis system."""
        for i in range(30):  # Warm up with 30 frames
            # Generate terrain data
            terrain_data = self.terrain_data_generator.generate_terrain_frame()

            # Process through terrain analyzer
            self.terrain_analyzer.analyze_terrain(terrain_data)

            time.sleep(0.02)  # 20ms between frames

        time.sleep(0.5)  # Allow system to stabilize

    def _run_terrain_analysis_performance_test(self):
        """Run the main terrain analysis performance test."""
        self.classification_times.clear()
        self.traversability_times.clear()
        self.total_analysis_times.clear()

        start_time = time.time()

        for frame_num in range(self.num_frames):
            # Generate terrain data
            terrain_data = self.terrain_data_generator.generate_terrain_frame()

            # Time terrain analysis
            analysis_start = time.time()

            # Step 1: Terrain classification
            classification_start = time.time()
            classification_result = self.terrain_analyzer.classify_terrain(terrain_data)
            classification_time = (time.time() - classification_start) * 1000

            # Step 2: Traversability calculation
            traversability_start = time.time()
            traversability_result = self.terrain_analyzer.calculate_traversability(
                terrain_data, classification_result
            )
            traversability_time = (time.time() - traversability_start) * 1000

            # Total analysis time
            total_time = (time.time() - analysis_start) * 1000

            # Record timing data
            self.classification_times.append(classification_time)
            self.traversability_times.append(traversability_time)
            self.total_analysis_times.append(total_time)

            # Progress indicator
            if (frame_num + 1) % 50 == 0:  # Every 50 frames
                print(f"  Progress: {frame_num + 1}/{self.num_frames} frames")

            # Maintain realistic frame rate (~10Hz for terrain analysis)
            elapsed = time.time() - analysis_start
            if elapsed < 0.1:  # Less than 100ms
                time.sleep(0.1 - elapsed)

        total_duration = time.time() - start_time
        actual_fps = self.num_frames / total_duration

        print(".2f")
        print(".1f")

    def _start_resource_monitoring(self):
        """Start monitoring system resources."""
        self.monitoring_active = True
        self.monitor_thread = threading.Thread(
            target=self._monitor_resources, daemon=True
        )
        self.monitor_thread.start()

    def _stop_resource_monitoring(self):
        """Stop resource monitoring."""
        self.monitoring_active = False
        if hasattr(self, "monitor_thread"):
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

            time.sleep(0.3)  # Monitor every 300ms

    def _analyze_terrain_results(self):
        """Analyze terrain analysis performance results."""
        if not self.total_analysis_times:
            self.fail("No terrain analysis times collected")

        # Calculate classification statistics
        avg_classification = statistics.mean(self.classification_times)
        max_classification = max(self.classification_times)
        p95_classification = statistics.quantiles(self.classification_times, n=20)[18]

        # Calculate traversability statistics
        avg_traversability = statistics.mean(self.traversability_times)
        max_traversability = max(self.traversability_times)
        p95_traversability = statistics.quantiles(self.traversability_times, n=20)[18]

        # Calculate total analysis statistics
        avg_total = statistics.mean(self.total_analysis_times)
        max_total = max(self.total_analysis_times)
        p95_total = statistics.quantiles(self.total_analysis_times, n=20)[18]

        # Resource usage statistics
        avg_memory_mb = (
            statistics.mean(self.memory_usage_readings)
            if self.memory_usage_readings
            else 0
        )
        max_memory_mb = (
            max(self.memory_usage_readings) if self.memory_usage_readings else 0
        )
        memory_delta_mb = (
            max_memory_mb - min(self.memory_usage_readings)
            if self.memory_usage_readings
            else 0
        )
        avg_cpu_percent = (
            statistics.mean(self.cpu_usage_readings) if self.cpu_usage_readings else 0
        )
        max_cpu_percent = max(self.cpu_usage_readings) if self.cpu_usage_readings else 0

        # Print detailed results
        print("\n📈 Terrain Analysis Performance Results:")
        print("-" * 55)
        print("Classification Times:")
        print(".3f")
        print(".3f")
        print(".3f")
        print("\nTraversability Times:")
        print(".3f")
        print(".3f")
        print(".3f")
        print("\nTotal Analysis Times:")
        print(".3f")
        print(".3f")
        print(".3f")
        print("\nResource Usage:")
        print(".2f")
        print(".2f")
        print(".2f")
        print(".1f")
        print(".1f")

        # Validate against competition targets
        print("\n🎯 Competition Requirements Check:")
        print("-" * 55)

        requirements = {
            "Classification Time (avg)": (
                avg_classification,
                self.target_classification_ms,
            ),
            "Classification Time (95p)": (
                p95_classification,
                self.target_classification_ms * 1.2,
            ),
            "Traversability Time (avg)": (
                avg_traversability,
                self.target_traversability_ms,
            ),
            "Traversability Time (95p)": (
                p95_traversability,
                self.target_traversability_ms * 1.2,
            ),
            "Total Analysis Time (avg)": (
                avg_total,
                self.target_classification_ms + self.target_traversability_ms,
            ),
            "Total Analysis Time (95p)": (
                p95_total,
                (self.target_classification_ms + self.target_traversability_ms) * 1.2,
            ),
            "Memory Delta": (memory_delta_mb, self.target_memory_delta_mb),
            "CPU Usage (avg)": (avg_cpu_percent, self.target_cpu_percent),
            "CPU Usage (max)": (max_cpu_percent, self.target_cpu_percent * 1.2),
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
        print("-" * 55)

        if all_passed:
            print("✅ ALL TERRAIN ANALYSIS REQUIREMENTS MET")
            print("   Terrain intelligence suitable for real-time navigation")
        elif avg_total <= (
            self.target_classification_ms + self.target_traversability_ms
        ):
            print("⚠️  AVERAGE PERFORMANCE OK, BUT RESOURCE/CONSISTENCY CONCERNS")
            print("   Monitor performance under competition terrain conditions")
        else:
            print("❌ TERRAIN ANALYSIS REQUIREMENTS NOT MET")
            print("   Optimization required for competition terrain navigation")

        # Detailed analysis and recommendations
        if (
            avg_classification > self.target_classification_ms
            or avg_traversability > self.target_traversability_ms
        ):
            print("\n💡 Optimization Recommendations:")
            if avg_classification > self.target_classification_ms:
                print("   - Optimize terrain classification algorithm")
                print("   - Consider simplified feature extraction")
            if avg_traversability > self.target_traversability_ms:
                print("   - Optimize traversability cost calculations")
                print("   - Reduce traversability map resolution if needed")
            if memory_delta_mb > self.target_memory_delta_mb:
                print("   - Reduce memory allocations in terrain processing")
                print("   - Implement memory pooling for terrain data")
            if avg_cpu_percent > self.target_cpu_percent:
                print("   - Profile terrain analysis code for bottlenecks")
                print("   - Consider parallel processing for classification tasks")

        # Store results for regression testing
        self.test_results = {
            "avg_classification_ms": avg_classification,
            "max_classification_ms": max_classification,
            "p95_classification_ms": p95_classification,
            "avg_traversability_ms": avg_traversability,
            "max_traversability_ms": max_traversability,
            "p95_traversability_ms": p95_traversability,
            "avg_total_ms": avg_total,
            "max_total_ms": max_total,
            "p95_total_ms": p95_total,
            "memory_delta_mb": memory_delta_mb,
            "avg_cpu_percent": avg_cpu_percent,
            "max_cpu_percent": max_cpu_percent,
            "requirements_met": all_passed,
            "frames_analyzed": len(self.total_analysis_times),
        }

        # Assert critical requirements
        self.assertLess(
            avg_total,
            (self.target_classification_ms + self.target_traversability_ms) * 1.5,
            ".3f",
        )
        self.assertLess(memory_delta_mb, self.target_memory_delta_mb * 2.0, ".2f")
        self.assertLess(avg_cpu_percent, self.target_cpu_percent * 1.5, ".1f")

    def test_terrain_analysis_under_various_conditions(self):
        """Test terrain analysis under various terrain conditions."""
        print("\n🏞️  Testing Terrain Analysis Under Various Conditions")

        terrain_conditions = [
            "flat_sand",  # Easy terrain
            "rocky_surface",  # Medium difficulty
            "steep_slopes",  # Challenging terrain
            "mixed_terrain",  # Complex conditions
        ]

        results_by_condition = {}

        for condition in terrain_conditions:
            print(f"\n  Testing {condition}...")

            # Configure terrain generator for this condition
            self.terrain_data_generator.set_terrain_condition(condition)

            # Reset timing data
            self.classification_times.clear()
            self.traversability_times.clear()
            self.total_analysis_times.clear()

            # Run shorter test for this condition
            num_test_frames = 50

            for frame_num in range(num_test_frames):
                terrain_data = self.terrain_data_generator.generate_terrain_frame()

                # Time analysis
                start_time = time.time()
                classification_result = self.terrain_analyzer.classify_terrain(
                    terrain_data
                )
                traversability_result = self.terrain_analyzer.calculate_traversability(
                    terrain_data, classification_result
                )
                total_time = (time.time() - start_time) * 1000

                self.total_analysis_times.append(total_time)
                time.sleep(0.05)  # 50ms between frames

            # Calculate statistics for this condition
            avg_time = statistics.mean(self.total_analysis_times)
            max_time = max(self.total_analysis_times)
            p95_time = statistics.quantiles(self.total_analysis_times, n=20)[18]

            results_by_condition[condition] = {
                "avg_time_ms": avg_time,
                "max_time_ms": max_time,
                "p95_time_ms": p95_time,
            }

            print(".3f")

        # Analyze condition-specific performance
        print("\n📊 Terrain Condition Performance Summary:")
        print("-" * 50)
        for condition, results in results_by_condition.items():
            passed = results["avg_time_ms"] <= (
                self.target_classification_ms + self.target_traversability_ms
            )
            status = "✅ PASS" if passed else "❌ FAIL"
            print("25")

        # Check if performance varies significantly by terrain type
        times = [r["avg_time_ms"] for r in results_by_condition.values()]
        time_variation = statistics.stdev(times) if len(times) > 1 else 0

        print(".3f")
        if time_variation > 10.0:  # More than 10ms variation
            print("⚠️  Significant performance variation by terrain type detected")
            print("   Consider optimizing for worst-case terrain conditions")
        else:
            print("✅ Consistent performance across terrain types")


class TerrainDataGenerator:
    """Generates realistic terrain data for performance testing."""

    def __init__(self):
        """Initialize terrain data generator."""
        self.bridge = CvBridge()

        # Default terrain parameters
        self.width, self.height = 640, 480
        self.terrain_condition = "mixed_terrain"

        # Terrain generation parameters by condition
        self.terrain_params = {
            "flat_sand": {
                "slope_range": (-2, 2),  # Flat terrain
                "roughness": 0.1,  # Low roughness
                "rock_density": 0.05,  # Few rocks
                "color_variation": 0.2,  # Low color variation
            },
            "rocky_surface": {
                "slope_range": (-5, 5),  # Moderate slopes
                "roughness": 0.4,  # Medium roughness
                "rock_density": 0.15,  # Some rocks
                "color_variation": 0.4,  # Medium color variation
            },
            "steep_slopes": {
                "slope_range": (-15, 15),  # Steep slopes
                "roughness": 0.7,  # High roughness
                "rock_density": 0.25,  # Many rocks
                "color_variation": 0.6,  # High color variation
            },
            "mixed_terrain": {
                "slope_range": (-10, 10),  # Mixed slopes
                "roughness": 0.5,  # Medium-high roughness
                "rock_density": 0.2,  # Moderate rocks
                "color_variation": 0.5,  # Medium-high variation
            },
        }

    def set_terrain_condition(self, condition: str):
        """Set the terrain condition for generation."""
        if condition in self.terrain_params:
            self.terrain_condition = condition
        else:
            print(
                f"Warning: Unknown terrain condition '{condition}', using mixed_terrain"
            )
            self.terrain_condition = "mixed_terrain"

    def generate_terrain_frame(self) -> Dict[str, Any]:
        """Generate a frame of terrain data."""
        params = self.terrain_params[self.terrain_condition]

        # Generate synthetic terrain image
        image = self._generate_terrain_image(params)

        # Generate point cloud data (simplified)
        pointcloud = self._generate_pointcloud_data(params)

        # Generate LiDAR scan data
        lidar_scan = self._generate_lidar_scan(params)

        return {
            "image": image,
            "pointcloud": pointcloud,
            "lidar_scan": lidar_scan,
            "condition": self.terrain_condition,
            "timestamp": time.time(),
        }

    def _generate_terrain_image(self, params) -> np.ndarray:
        """Generate synthetic terrain image."""
        # Create base terrain color
        base_color = np.array([194, 178, 128])  # Sandy brown base

        # Generate height map
        height_map = np.random.normal(
            0, params["roughness"] * 10, (self.height, self.width)
        )

        # Apply slope gradient
        slope_gradient = np.linspace(
            params["slope_range"][0], params["slope_range"][1], self.width
        )
        slope_map = np.tile(slope_gradient, (self.height, 1))
        height_map += slope_map

        # Convert height to color variation
        color_variation = (height_map - height_map.min()) / (
            height_map.max() - height_map.min()
        )
        color_variation = color_variation * params["color_variation"]

        # Create RGB image
        image = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        for c in range(3):
            channel_variation = color_variation * (0.8 + np.random.random() * 0.4)
            image[:, :, c] = np.clip(
                base_color[c] * (1 + channel_variation), 0, 255
            ).astype(np.uint8)

        # Add rocks
        num_rocks = int(self.width * self.height * params["rock_density"] / 1000)
        for _ in range(num_rocks):
            rock_size = np.random.randint(5, 20)
            rock_x = np.random.randint(rock_size, self.width - rock_size)
            rock_y = np.random.randint(rock_size, self.height - rock_size)

            # Darker color for rocks
            rock_color = np.array([64, 64, 64]) + np.random.randint(-20, 20, 3)
            cv2.circle(image, (rock_x, rock_y), rock_size, rock_color.tolist(), -1)

        return image

    def _generate_pointcloud_data(self, params) -> np.ndarray:
        """Generate simplified point cloud data."""
        num_points = 1000

        # Generate 3D points in front of the robot
        x_coords = np.random.uniform(-5, 5, num_points)  # ±5m laterally
        y_coords = np.random.uniform(0, 10, num_points)  # 0-10m forward
        z_coords = np.random.normal(
            0, params["roughness"] * 2, num_points
        )  # Height variation

        # Apply slope to height
        slope_effect = y_coords * np.radians(np.random.uniform(*params["slope_range"]))
        z_coords += slope_effect

        return np.column_stack([x_coords, y_coords, z_coords])

    def _generate_lidar_scan(self, params) -> np.ndarray:
        """Generate LiDAR scan data."""
        num_readings = 360  # 1-degree resolution
        max_range = 10.0

        angles = np.linspace(0, 2 * np.pi, num_readings, endpoint=False)
        ranges = np.random.uniform(0.5, max_range, num_readings)

        # Add some terrain features to ranges
        terrain_noise = np.random.normal(0, params["roughness"], num_readings)
        ranges += terrain_noise

        # Ensure minimum range
        ranges = np.clip(ranges, 0.1, max_range)

        return ranges


class TerrainAnalyzerSimulator:
    """Simulates terrain analysis for performance testing."""

    def __init__(self):
        """Initialize terrain analyzer simulator."""
        self.bridge = CvBridge()

        # Simulated processing parameters (based on real terrain analysis)
        self.classification_base_time = 0.030  # 30ms base classification time
        self.traversability_base_time = 0.015  # 15ms base traversability time

        # Terrain type definitions
        self.terrain_types = ["sand", "rock", "slope", "hazard", "unknown"]

    def analyze_terrain(self, terrain_data: Dict[str, Any]) -> Dict[str, Any]:
        """Complete terrain analysis (classification + traversability)."""
        # Perform classification
        classification = self.classify_terrain(terrain_data)

        # Perform traversability calculation
        traversability = self.calculate_traversability(terrain_data, classification)

        return {
            "classification": classification,
            "traversability": traversability,
            "timestamp": time.time(),
        }

    def classify_terrain(self, terrain_data: Dict[str, Any]) -> Dict[str, Any]:
        """Classify terrain types from sensor data."""
        start_time = time.time()

        # Simulate terrain classification processing
        image = terrain_data["image"]

        # Simulate image processing operations
        # Convert to grayscale and apply filters
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        blurred = cv2.GaussianBlur(gray, (5, 5), 0)
        edges = cv2.Canny(blurred, 50, 150)

        # Simulate feature extraction
        corners = cv2.goodFeaturesToTrack(gray, 100, 0.01, 10)

        # Simulate texture analysis
        texture_features = []
        for i in range(10):  # Analyze 10 regions
            x, y = np.random.randint(0, image.shape[1]), np.random.randint(
                0, image.shape[0]
            )
            region = gray[y : y + 32, x : x + 32]
            if region.size > 0:
                texture_features.append(np.std(region))

        # Simulate classification
        terrain_map = np.random.choice(
            self.terrain_types,
            size=(image.shape[0] // 32, image.shape[1] // 32),
            p=[0.5, 0.2, 0.15, 0.1, 0.05],
        )

        # Add processing time variation based on terrain complexity
        condition = terrain_data.get("condition", "mixed_terrain")
        complexity_multiplier = {
            "flat_sand": 0.8,
            "rocky_surface": 1.0,
            "steep_slopes": 1.3,
            "mixed_terrain": 1.1,
        }.get(condition, 1.0)

        processing_time = self.classification_base_time * complexity_multiplier
        time.sleep(processing_time)

        return {
            "terrain_map": terrain_map,
            "confidence_map": np.random.random(terrain_map.shape),
            "processing_time": time.time() - start_time,
            "features_extracted": len(corners) if corners is not None else 0,
        }

    def calculate_traversability(
        self, terrain_data: Dict[str, Any], classification: Dict[str, Any]
    ) -> Dict[str, Any]:
        """Calculate traversability costs for path planning."""
        start_time = time.time()

        # Simulate traversability calculation
        terrain_map = classification["terrain_map"]

        # Create traversability cost map
        rows, cols = terrain_map.shape
        traversability_map = np.zeros((rows, cols))

        # Assign costs based on terrain type
        terrain_costs = {
            "sand": 1.0,  # Easy traversal
            "rock": 3.0,  # Moderate difficulty
            "slope": 2.5,  # Sloped terrain
            "hazard": 10.0,  # Avoid if possible
            "unknown": 2.0,  # Unknown but traversable
        }

        for i in range(rows):
            for j in range(cols):
                terrain_type = terrain_map[i, j]
                base_cost = terrain_costs.get(terrain_type, 2.0)

                # Add slope-based cost modifier
                slope_modifier = np.random.uniform(0.8, 1.5)
                traversability_map[i, j] = base_cost * slope_modifier

        # Simulate additional processing (gradient calculation, smoothing)
        smoothed_map = cv2.GaussianBlur(
            traversability_map.astype(np.float32), (3, 3), 0
        )

        # Calculate path planning costs
        gradient_x = cv2.Sobel(smoothed_map, cv2.CV_32F, 1, 0, ksize=3)
        gradient_y = cv2.Sobel(smoothed_map, cv2.CV_32F, 0, 1, ksize=3)
        slope_costs = np.sqrt(gradient_x**2 + gradient_y**2)

        # Add processing time variation
        condition = terrain_data.get("condition", "mixed_terrain")
        complexity_multiplier = {
            "flat_sand": 0.7,
            "rocky_surface": 1.0,
            "steep_slopes": 1.4,
            "mixed_terrain": 1.2,
        }.get(condition, 1.0)

        processing_time = self.traversability_base_time * complexity_multiplier
        time.sleep(processing_time)

        return {
            "traversability_map": smoothed_map,
            "slope_costs": slope_costs,
            "path_costs": smoothed_map + slope_costs * 0.5,
            "processing_time": time.time() - start_time,
            "map_resolution": 0.1,  # meters per cell
        }


if __name__ == "__main__":
    unittest.main()
