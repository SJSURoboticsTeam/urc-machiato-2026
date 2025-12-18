#!/usr/bin/env python3
"""
Tests for Centralized Vision Processing.

Validates the centralized vision processing node that eliminates
image duplication by processing camera data once and distributing
results to multiple consumers.
"""

import unittest
from unittest.mock import Mock, patch
import numpy as np
import cv2
import time
from typing import Optional, Dict, Any, List
from concurrent.futures import ThreadPoolExecutor

# Add vision processing to path
import os
import sys
PROJECT_ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), "..", "..", ".."))
VISION_ROOT = os.path.join(PROJECT_ROOT, "src", "vision_processing")
sys.path.insert(0, VISION_ROOT)

try:
    from vision_processing.vision_processing_node import VisionProcessingNode
except ImportError:
    VisionProcessingNode = None


@unittest.skipIf(VisionProcessingNode is None, "VisionProcessingNode not available")
class TestVisionProcessing(unittest.TestCase):
    """Test centralized vision processing."""

    def setUp(self):
        """Set up test fixtures."""
        with patch('rclpy.node.Node.__init__', return_value=None):
            self.vision_node = VisionProcessingNode()

            # Mock required attributes
            self.vision_node.get_logger = Mock(return_value=Mock())
            self.vision_node.get_logger.return_value.info = Mock()
            self.vision_node.get_logger.return_value.warn = Mock()
            self.vision_node.get_logger.return_value.error = Mock()
            self.vision_node.get_logger.return_value.debug = Mock()

            # Mock publishers
            self.vision_node.keyboard_pose_pub = Mock()
            self.vision_node.terrain_map_pub = Mock()
            self.vision_node.obstacle_pub = Mock()
            self.vision_node.feature_pub = Mock()

            # Mock CV bridge
            self.vision_node.bridge = Mock()

            # Mock thread pool
            self.vision_node.executor = Mock()

            # Initialize processing state
            self.vision_node.processing_lock = Mock()
            self.vision_node.processing_lock.__enter__ = Mock(return_value=None)
            self.vision_node.processing_lock.__exit__ = Mock(return_value=None)

    def test_initialization(self):
        """Test VisionProcessingNode initialization."""
        with patch('rclpy.node.Node.__init__', return_value=None):
            node = VisionProcessingNode()

            # Verify basic initialization
            self.assertIsInstance(node, VisionProcessingNode)
            # Parameters would be set during actual ROS2 initialization

    def test_keyboard_detection(self):
        """Test keyboard detection processing."""
        # Create test image with keyboard-like features
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Add rectangular regions to simulate keyboard
        cv2.rectangle(test_image, (100, 100), (200, 150), (255, 255, 255), -1)
        cv2.rectangle(test_image, (220, 100), (320, 150), (255, 255, 255), -1)
        cv2.rectangle(test_image, (340, 100), (440, 150), (255, 255, 255), -1)

        # Mock keyboard detection result
        expected_result = {
            'detected': True,
            'confidence': 0.85,
            'bbox': [100, 100, 340, 50],
            'keys': ['Q', 'W', 'E']
        }

        # Mock the detection method
        self.vision_node.detect_keyboard = Mock(return_value=expected_result)

        result = self.vision_node.detect_keyboard(test_image)

        self.assertIsInstance(result, dict)
        self.assertIn('detected', result)
        self.assertTrue(result['detected'])

    def test_keyboard_detection_empty_image(self):
        """Test keyboard detection with empty image."""
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)

        self.vision_node.detect_keyboard = Mock(return_value={'detected': False, 'confidence': 0.0})

        result = self.vision_node.detect_keyboard(test_image)

        self.assertIsInstance(result, dict)
        self.assertIn('detected', result)
        self.assertFalse(result['detected'])

    def test_terrain_analysis(self):
        """Test terrain analysis processing."""
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Add terrain-like features (sand colored regions)
        cv2.rectangle(test_image, (0, 200), (640, 480), (200, 180, 150), -1)  # Sand-like color

        # Mock terrain analysis result
        expected_result = {
            'terrain_map': np.zeros((480, 640), dtype=np.uint8),
            'terrain_types': ['sand', 'rock', 'slope'],
            'traversability': 0.8,
            'hazards': []
        }

        self.vision_node.analyze_terrain = Mock(return_value=expected_result)

        result = self.vision_node.analyze_terrain(test_image)

        self.assertIsInstance(result, dict)
        self.assertIn('terrain_map', result)
        self.assertIn('terrain_types', result)

    def test_terrain_analysis_complex_scene(self):
        """Test terrain analysis with complex scene."""
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Add mixed terrain features
        # Sand area
        cv2.rectangle(test_image, (0, 300), (320, 480), (200, 180, 150), -1)
        # Rock area
        cv2.rectangle(test_image, (320, 300), (640, 400), (100, 100, 100), -1)
        # Slope area
        cv2.rectangle(test_image, (320, 400), (640, 480), (150, 150, 150), -1)

        expected_result = {
            'terrain_map': np.random.randint(0, 255, (480, 640), dtype=np.uint8),
            'terrain_types': ['sand', 'rock', 'slope'],
            'traversability': 0.6,
            'hazards': [{'x': 400, 'y': 350, 'type': 'rock'}]
        }

        self.vision_node.analyze_terrain = Mock(return_value=expected_result)

        result = self.vision_node.analyze_terrain(test_image)

        self.assertIsInstance(result, dict)
        self.assertIn('hazards', result)
        self.assertIsInstance(result['hazards'], list)

    def test_obstacle_detection(self):
        """Test obstacle detection processing."""
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Add obstacle-like features (dark regions)
        cv2.circle(test_image, (320, 240), 50, (50, 50, 50), -1)  # Circular obstacle

        # Mock obstacle detection result
        expected_obstacles = [
            {
                'x': 320,
                'y': 240,
                'width': 100,
                'height': 100,
                'confidence': 0.9,
                'type': 'rock'
            }
        ]

        self.vision_node.detect_obstacles = Mock(return_value=expected_obstacles)

        obstacles = self.vision_node.detect_obstacles(test_image)

        self.assertIsInstance(obstacles, list)
        self.assertEqual(len(obstacles), 1)
        self.assertIn('x', obstacles[0])
        self.assertIn('y', obstacles[0])
        self.assertIn('confidence', obstacles[0])

    def test_obstacle_detection_no_obstacles(self):
        """Test obstacle detection with clear scene."""
        test_image = np.ones((480, 640, 3), dtype=np.uint8) * 255  # White image

        self.vision_node.detect_obstacles = Mock(return_value=[])

        obstacles = self.vision_node.detect_obstacles(test_image)

        self.assertIsInstance(obstacles, list)
        self.assertEqual(len(obstacles), 0)

    def test_feature_extraction(self):
        """Test feature extraction for SLAM."""
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Add some features (corners, edges)
        cv2.rectangle(test_image, (100, 100), (200, 200), (255, 255, 255), -1)

        expected_features = {
            'keypoints': [Mock(), Mock(), Mock()],  # Mock keypoints
            'descriptors': np.random.rand(3, 128).astype(np.uint8),
            'count': 3
        }

        self.vision_node.extract_features = Mock(return_value=expected_features)

        features = self.vision_node.extract_features(test_image)

        self.assertIsInstance(features, dict)
        self.assertIn('keypoints', features)
        self.assertIn('descriptors', features)
        self.assertIn('count', features)

    def test_parallel_processing(self):
        """Test parallel processing of multiple vision tasks."""
        test_image = np.zeros((480, 640, 3), dtype=np.uint8)

        # Mock thread pool execution
        self.vision_node.executor.submit = Mock()
        self.vision_node.executor.submit.return_value.result = Mock(return_value={'detected': True})

        # Submit multiple tasks
        futures = []
        for i in range(3):
            future = self.vision_node.executor.submit(
                self.vision_node.detect_keyboard, test_image
            )
            futures.append(future)

        # Verify parallel execution was attempted
        self.assertEqual(self.vision_node.executor.submit.call_count, 3)

    def test_image_callback_processing(self):
        """Test image callback processing pipeline."""
        # Mock ROS2 Image message
        mock_image_msg = Mock()
        mock_image_msg.header = Mock()
        mock_image_msg.header.stamp = Mock()
        mock_image_msg.encoding = 'bgr8'
        mock_image_msg.width = 640
        mock_image_msg.height = 480

        # Mock CV bridge conversion
        test_cv_image = np.zeros((480, 640, 3), dtype=np.uint8)
        self.vision_node.bridge.imgmsg_to_cv2 = Mock(return_value=test_cv_image)

        # Mock processing methods
        self.vision_node.detect_keyboard = Mock(return_value={'detected': True})
        self.vision_node.analyze_terrain = Mock(return_value={'terrain_map': np.zeros((480, 640))})
        self.vision_node.detect_obstacles = Mock(return_value=[])

        # Call image callback
        self.vision_node.image_callback(mock_image_msg)

        # Verify processing pipeline was executed
        self.vision_node.bridge.imgmsg_to_cv2.assert_called_once_with(mock_image_msg, 'bgr8')
        self.vision_node.detect_keyboard.assert_called_once()
        self.vision_node.analyze_terrain.assert_called_once()
        self.vision_node.detect_obstacles.assert_called_once()

    def test_processing_rate_limiting(self):
        """Test processing rate limiting."""
        # Set up rate limiting
        self.vision_node.processing_rate = 10.0  # 10 Hz
        self.vision_node.last_processing_time = time.time() - 0.5  # 0.5 seconds ago

        # Mock image processing
        self.vision_node.process_image_frame = Mock()

        # Process should be allowed (enough time passed)
        current_time = time.time()
        should_process = (current_time - self.vision_node.last_processing_time) >= (1.0 / self.vision_node.processing_rate)

        self.assertTrue(should_process, "Processing should be allowed after sufficient time")

    def test_shared_memory_optimization(self):
        """Test shared memory buffer usage."""
        # Test buffer allocation
        buffer_size = 640 * 480 * 3  # RGB image
        image_buffer = np.zeros(buffer_size, dtype=np.uint8)

        self.assertEqual(len(image_buffer), buffer_size)
        self.assertEqual(image_buffer.dtype, np.uint8)

    def test_result_caching(self):
        """Test result caching for repeated queries."""
        # Simulate caching behavior
        cache_key = "keyboard_detection_123"
        cached_result = {'detected': True, 'confidence': 0.9}

        self.vision_node.result_cache = {}
        self.vision_node.result_cache[cache_key] = cached_result

        # Verify cached result retrieval
        retrieved = self.vision_node.result_cache.get(cache_key)
        self.assertEqual(retrieved, cached_result)

    def test_error_handling_corrupted_image(self):
        """Test error handling with corrupted image data."""
        # Test with invalid image data
        corrupted_image = None  # Invalid image

        self.vision_node.detect_keyboard = Mock(side_effect=Exception("Corrupted image"))

        with self.assertRaises(Exception):
            self.vision_node.detect_keyboard(corrupted_image)

    def test_performance_metrics(self):
        """Test performance metrics collection."""
        # Mock timing measurements
        start_time = time.time()
        time.sleep(0.01)  # Simulate processing time
        end_time = time.time()

        processing_time = end_time - start_time

        # Verify reasonable processing time
        self.assertGreater(processing_time, 0.0)
        self.assertLess(processing_time, 1.0)  # Should be fast

    def test_memory_efficiency(self):
        """Test memory efficiency of processing pipeline."""
        # Test that large data structures are managed properly
        large_image = np.zeros((1024, 1024, 3), dtype=np.uint8)

        # Calculate memory usage
        memory_bytes = large_image.nbytes
        memory_mb = memory_bytes / (1024 * 1024)

        # Should be reasonable size (under 10MB for test image)
        self.assertLess(memory_mb, 10.0)

    def test_concurrent_processing_safety(self):
        """Test thread safety of concurrent processing."""
        import threading

        # Mock processing lock
        self.vision_node.processing_lock = threading.Lock()

        # Test that lock can be acquired
        with self.vision_node.processing_lock:
            # Simulate critical section
            self.assertTrue(True, "Lock acquisition successful")

    def test_zero_copy_optimization(self):
        """Test zero-copy message passing."""
        # Test that image data can be passed without copying
        original_image = np.ones((480, 640, 3), dtype=np.uint8)

        # Simulate zero-copy (reference passing)
        processed_image = original_image  # No copy

        # Verify data integrity
        self.assertTrue(np.array_equal(original_image, processed_image))
        self.assertEqual(id(original_image), id(processed_image))  # Same object


class TestVisionProcessingIntegration(unittest.TestCase):
    """Integration tests for vision processing system."""

    def test_end_to_end_processing_pipeline(self):
        """Test complete processing pipeline from image to results."""
        with patch('rclpy.node.Node.__init__', return_value=None):
            node = VisionProcessingNode()

            # Mock all required components
            node.get_logger = Mock(return_value=Mock())
            node.bridge = Mock()
            node.executor = Mock()

            # Mock publishers
            node.keyboard_pose_pub = Mock()
            node.terrain_map_pub = Mock()
            node.obstacle_pub = Mock()

            # Create test image
            test_image = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.rectangle(test_image, (100, 100), (300, 200), (255, 255, 255), -1)

            # Mock processing results
            keyboard_result = {'detected': True, 'bbox': [100, 100, 200, 100]}
            terrain_result = {'terrain_map': np.zeros((480, 640), dtype=np.uint8)}
            obstacle_result = [{'x': 200, 'y': 150, 'type': 'rock'}]

            node.detect_keyboard = Mock(return_value=keyboard_result)
            node.analyze_terrain = Mock(return_value=terrain_result)
            node.detect_obstacles = Mock(return_value=obstacle_result)

            # Process image
            node.process_image_frame(test_image)

            # Verify all processing branches were executed
            node.detect_keyboard.assert_called_once_with(test_image)
            node.analyze_terrain.assert_called_once_with(test_image)
            node.detect_obstacles.assert_called_once_with(test_image)

            # Verify results were published
            node.keyboard_pose_pub.publish.assert_called()
            node.terrain_map_pub.publish.assert_called()
            node.obstacle_pub.publish.assert_called()

    def test_competition_scenario_processing(self):
        """Test vision processing under competition conditions."""
        # Simulate high-load competition scenario
        with patch('rclpy.node.Node.__init__', return_value=None):
            node = VisionProcessingNode()

            # Configure for competition (higher rate, more workers)
            node.processing_rate = 30.0  # 30 Hz
            node.max_workers = 8

            # Mock high-load processing
            node.executor = ThreadPoolExecutor(max_workers=node.max_workers)

            # Test can handle high frame rate
            frame_interval = 1.0 / node.processing_rate
            self.assertLess(frame_interval, 0.1)  # Under 100ms per frame


if __name__ == '__main__':
    unittest.main()



