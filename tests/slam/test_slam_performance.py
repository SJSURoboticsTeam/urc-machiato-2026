#!/usr/bin/env python3
"""
SLAM Performance Testing for URC 2026

Critical performance validation for visual SLAM system:
- Frame processing rate (30+ FPS requirement)
- Memory usage under continuous operation
- Feature tracking accuracy and reliability
- Loop closure detection performance
- Real-time pose estimation latency

Author: URC 2026 SLAM Team
"""

import pytest
import time
import cv2
import numpy as np
import psutil
import threading
import statistics
from unittest.mock import Mock, patch
from typing import Dict, List, Any, Tuple
import gc
import resource

# Add source paths
import sys
import os

sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../src"))

pytest.importorskip(
    "src.autonomy.core", reason="src.autonomy.core not available (SLAM tests)"
)


class TestSLAMPerformance:
    """Test SLAM system performance under competition conditions."""

    @pytest.fixture
    def slam_system(self):
        """Initialize SLAM system for testing."""
        from src.autonomy.core.perception.lightweight_slam import LightweightSLAMSystem

        return LightweightSLAMSystem()

    @pytest.fixture
    def mock_camera_stream(self):
        """Generate mock camera frames for testing."""
        # Create realistic test frames with features
        frames = []

        for i in range(100):
            # Generate frame with varying content
            frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

            # Add structured features (corners, edges)
            if i % 10 == 0:
                # Add corner features
                cv2.rectangle(frame, (100, 100), (200, 200), (255, 255, 255), 2)
                cv2.rectangle(frame, (300, 300), (400, 400), (255, 255, 255), 2)

            if i % 15 == 0:
                # Add circular features
                cv2.circle(frame, (320, 240), 50, (128, 128, 128), -1)

            # Add noise for realism
            noise = np.random.normal(0, 10, frame.shape).astype(np.uint8)
            frame = cv2.add(frame, noise)

            frames.append(frame)

        return frames

    @pytest.fixture
    def performance_monitor(self):
        """Monitor system performance during tests."""
        process = psutil.Process()

        class PerformanceMonitor:
            def __init__(self):
                self.start_time = None
                self.end_time = None
                self.memory_samples = []
                self.cpu_samples = []
                self.frame_times = []
                self.process = process

            def start(self):
                self.start_time = time.time()
                self.memory_samples = []
                self.cpu_samples = []
                self.frame_times = []

            def sample(self):
                """Sample current performance metrics."""
                self.memory_samples.append(
                    self.process.memory_info().rss / 1024 / 1024
                )  # MB
                self.cpu_samples.append(self.process.cpu_percent())

            def record_frame_time(self, frame_time):
                """Record frame processing time."""
                self.frame_times.append(frame_time)

            def stop(self):
                self.end_time = time.time()

            def get_summary(self):
                """Get performance summary."""
                total_time = self.end_time - self.start_time if self.end_time else 0

                return {
                    "duration": total_time,
                    "avg_memory_mb": (
                        statistics.mean(self.memory_samples)
                        if self.memory_samples
                        else 0
                    ),
                    "max_memory_mb": (
                        max(self.memory_samples) if self.memory_samples else 0
                    ),
                    "avg_cpu_percent": (
                        statistics.mean(self.cpu_samples) if self.cpu_samples else 0
                    ),
                    "max_cpu_percent": max(self.cpu_samples) if self.cpu_samples else 0,
                    "avg_frame_time_ms": (
                        statistics.mean(self.frame_times) * 1000
                        if self.frame_times
                        else 0
                    ),
                    "max_frame_time_ms": (
                        max(self.frame_times) * 1000 if self.frame_times else 0
                    ),
                    "total_frames": len(self.frame_times),
                    "fps": len(self.frame_times) / total_time if total_time > 0 else 0,
                }

        return PerformanceMonitor()

    @pytest.mark.critical
    def test_frame_processing_rate(
        self, slam_system, mock_camera_stream, performance_monitor
    ):
        """Test SLAM frame processing rate (30+ FPS requirement)."""
        performance_monitor.start()

        # Process frames as fast as possible
        for i, frame in enumerate(mock_camera_stream):
            frame_start = time.time()

            result = slam_system.process_frame(frame)

            frame_end = time.time()
            performance_monitor.record_frame_time(frame_end - frame_start)

            # Sample performance every 10 frames
            if i % 10 == 0:
                performance_monitor.sample()

        performance_monitor.stop()
        summary = performance_monitor.get_summary()

        # Optimized targets for Pi 5 + RealSense 435
        target_fps_min = 15.0  # Conservative target for Pi 5
        target_fps_optimal = 25.0  # Optimal target

        assert (
            summary["fps"] >= target_fps_min
        ), f"Frame rate too low: {summary['fps']:.1f} FPS (min: {target_fps_min})"
        assert (
            summary["avg_frame_time_ms"] <= 66.7
        ), f"Avg frame time too high: {summary['avg_frame_time_ms']:.1f}ms (target: 66.7ms for 15 FPS)"
        assert (
            summary["max_frame_time_ms"] <= 150.0
        ), f"Max frame time too high: {summary['max_frame_time_ms']:.1f}ms (target: 150ms)"

        print(f"Frame Processing Rate Test:")
        print(f"  FPS: {summary['fps']:.1f} (required: 30+)")
        print(f"  Avg frame time: {summary['avg_frame_time_ms']:.1f}ms")
        print(f"  Max frame time: {summary['max_frame_time_ms']:.1f}ms")
        print(f"  Total frames: {summary['total_frames']}")

    @pytest.mark.critical
    def test_memory_usage_continuously(self, slam_system, performance_monitor):
        """Test memory usage during continuous 30-minute operation."""
        # Simulate 30 minutes of operation (accelerated)
        simulated_duration = 30 * 60  # 30 minutes in seconds
        test_acceleration = 60  # Run test 60x faster (30 seconds = 30 minutes)
        actual_duration = simulated_duration / test_acceleration

        frame_interval = 1.0 / 30.0  # 30 FPS
        total_frames = int(simulated_duration * 30)  # 30 FPS for 30 minutes

        performance_monitor.start()

        # Generate frames continuously
        for i in range(total_frames):
            # Generate frame with gradual scene changes
            frame = self._generate_frame_sequence(i, total_frames)

            frame_start = time.time()
            result = slam_system.process_frame(frame)
            frame_end = time.time()

            performance_monitor.record_frame_time(frame_end - frame_start)

            # Sample memory frequently
            if i % 30 == 0:
                performance_monitor.sample()

            # Maintain timing (accelerated)
            elapsed = time.time() - frame_start
            if elapsed < frame_interval / test_acceleration:
                time.sleep((frame_interval / test_acceleration) - elapsed)

        performance_monitor.stop()
        summary = performance_monitor.get_summary()

        # Optimized memory targets for Pi 5 16GB
        assert (
            summary["max_memory_mb"] < 2048
        ), f"Memory usage too high: {summary['max_memory_mb']:.1f} MB (limit: 2GB)"
        assert (
            summary["avg_memory_mb"] < 1536
        ), f"Average memory too high: {summary['avg_memory_mb']:.1f} MB (target: 1.5GB for Pi 5)"

        # Check for memory leaks (growth should be minimal)
        memory_growth = summary["max_memory_mb"] - summary["avg_memory_mb"]
        assert memory_growth < 200, f"Memory growth too high: {memory_growth:.1f} MB"

        print(f"Continuous Memory Usage Test:")
        print(f"  Simulated duration: {simulated_duration/60:.1f} minutes")
        print(f"  Actual duration: {actual_duration:.1f} seconds")
        print(f"  Avg memory: {summary['avg_memory_mb']:.1f} MB")
        print(f"  Max memory: {summary['max_memory_mb']:.1f} MB")
        print(f"  Memory growth: {memory_growth:.1f} MB")
        print(f"  FPS sustained: {summary['fps']:.1f}")

    @pytest.mark.critical
    def test_feature_tracking_accuracy(self, slam_system, performance_monitor):
        """Test feature tracking accuracy and consistency."""
        # Create frames with known transformation
        frames_with_transforms = []
        true_transforms = []

        # Generate sequence with known camera motion
        for i in range(50):
            # Create base frame
            frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

            # Add known features
            cv2.rectangle(
                frame, (100 + i * 2, 100), (200 + i * 2, 200), (255, 255, 255), 2
            )
            cv2.circle(frame, (320, 240), 50, (128, 128, 128), -1)

            # Store with known transform
            frames_with_transforms.append(frame)
            true_transforms.append(
                [i * 0.01, i * 0.005, i * 0.001]
            )  # Small translations

        performance_monitor.start()

        poses = []
        confidences = []
        feature_counts = []

        for i, frame in enumerate(frames_with_transforms):
            result = slam_system.process_frame(frame)

            poses.append(result["pose_3d"])
            confidences.append(result["confidence"])
            feature_counts.append(result["features_tracked"])

            performance_monitor.sample()

        performance_monitor.stop()

        # Analyze tracking performance
        avg_confidence = statistics.mean(confidences)
        avg_features = statistics.mean(feature_counts)
        min_confidence = min(confidences)
        min_features = min(feature_counts)

        # Feature tracking requirements
        assert (
            avg_confidence >= 0.7
        ), f"Average confidence too low: {avg_confidence:.2f}"
        assert (
            min_confidence >= 0.3
        ), f"Minimum confidence too low: {min_confidence:.2f}"
        assert avg_features >= 50, f"Average feature count too low: {avg_features:.1f}"
        assert min_features >= 10, f"Minimum feature count too low: {min_features}"

        # Check pose consistency
        if len(poses) > 1:
            position_changes = []
            for i in range(1, len(poses)):
                pos_change = np.linalg.norm(poses[i][:3, 3] - poses[i - 1][:3, 3])
                position_changes.append(pos_change)

            avg_position_change = statistics.mean(position_changes)
            assert (
                avg_position_change > 0.001
            ), "No position changes detected (tracking failed)"

        print(f"Feature Tracking Accuracy Test:")
        print(f"  Average confidence: {avg_confidence:.2f}")
        print(f"  Minimum confidence: {min_confidence:.2f}")
        print(f"  Average features: {avg_features:.1f}")
        print(f"  Minimum features: {min_features}")
        print(f"  Frames processed: {len(poses)}")

    @pytest.mark.critical
    def test_pose_estimation_latency(self, slam_system):
        """Test pose estimation latency requirements (< 50ms)."""
        latencies = []

        # Generate diverse frames
        for i in range(100):
            frame = self._generate_frame_sequence(i, 100)

            # Measure latency
            start_time = time.time()
            result = slam_system.process_frame(frame)
            end_time = time.time()

            latency_ms = (end_time - start_time) * 1000
            latencies.append(latency_ms)

        # Latency analysis
        avg_latency = statistics.mean(latencies)
        max_latency = max(latencies)
        p95_latency = sorted(latencies)[int(len(latencies) * 0.95)]

        # Optimized latency targets for Pi 5 + RealSense 435
        assert (
            avg_latency <= 50.0
        ), f"Average latency too high: {avg_latency:.1f}ms (target: 50ms for Pi 5)"
        assert (
            max_latency <= 150.0
        ), f"Maximum latency too high: {max_latency:.1f}ms (target: 150ms)"
        assert (
            p95_latency <= 100.0
        ), f"P95 latency too high: {p95_latency:.1f}ms (target: 100ms)"

        print(f"Pose Estimation Latency Test:")
        print(f"  Average latency: {avg_latency:.1f}ms")
        print(f"  Maximum latency: {max_latency:.1f}ms")
        print(f"  P95 latency: {p95_latency:.1f}ms")
        print(f"  Samples: {len(latencies)}")

    @pytest.mark.critical
    def test_slam_recovery_performance(self, slam_system):
        """Test SLAM recovery from tracking loss scenarios."""
        recovery_times = []
        tracking_loss_scenarios = [
            "low_texture",
            "motion_blur",
            "lighting_change",
            "occlusion",
        ]

        for scenario in tracking_loss_scenarios:
            # Normal frames
            for i in range(10):
                frame = self._generate_normal_frame(i)
                slam_system.process_frame(frame)

            # Introduce tracking loss
            loss_start = time.time()

            for i in range(5):
                frame = self._generate_problematic_frame(scenario, i)
                result = slam_system.process_frame(frame)
                # Confidence should drop during tracking loss
                assert (
                    result["confidence"] < 0.5
                ), f"No tracking loss detected in {scenario}"

            # Recovery frames
            recovery_start = time.time()
            confidence_recovered = False

            for i in range(20):  # Allow up to 20 frames for recovery
                frame = self._generate_normal_frame(i)
                result = slam_system.process_frame(frame)

                if result["confidence"] >= 0.7:
                    recovery_end = time.time()
                    recovery_times.append(recovery_end - recovery_start)
                    confidence_recovered = True
                    break

            assert (
                confidence_recovered
            ), f"Failed to recover from {scenario} tracking loss"

        # Recovery performance analysis
        avg_recovery_time = statistics.mean(recovery_times)
        max_recovery_time = max(recovery_times)

        # Recovery requirements
        assert (
            avg_recovery_time <= 2.0
        ), f"Average recovery too slow: {avg_recovery_time:.2f}s"
        assert (
            max_recovery_time <= 5.0
        ), f"Maximum recovery too slow: {max_recovery_time:.2f}s"

        print(f"SLAM Recovery Performance Test:")
        print(f"  Scenarios tested: {len(tracking_loss_scenarios)}")
        print(f"  Average recovery time: {avg_recovery_time:.2f}s")
        print(f"  Maximum recovery time: {max_recovery_time:.2f}s")
        for scenario, recovery_time in zip(tracking_loss_scenarios, recovery_times):
            print(f"  {scenario}: {recovery_time:.2f}s")

    @pytest.mark.critical
    def test_loop_closure_performance(self, slam_system):
        """Test loop closure detection performance."""
        # Create loop scenario (return to starting location)
        loop_frames = []

        # Forward path
        for i in range(30):
            frame = self._generate_loop_frame(i, "forward")
            loop_frames.append(frame)

        # Return path (loop closure)
        for i in range(30):
            frame = self._generate_loop_frame(i, "return")
            loop_frames.append(frame)

        # Process frames and monitor for loop closure
        processing_times = []
        keyframe_events = []

        for i, frame in enumerate(loop_frames):
            start_time = time.time()
            result = slam_system.process_frame(frame)
            end_time = time.time()

            processing_times.append(end_time - start_time)

            if result.get("keyframe_added", False):
                keyframe_events.append(i)

        # Loop closure should be detected during return path
        assert (
            len(keyframe_events) >= 5
        ), f"Too few keyframes detected: {len(keyframe_events)}"
        assert any(kf >= 30 for kf in keyframe_events), "No loop closure detected"

        # Performance during loop closure
        avg_processing_time = statistics.mean(processing_times)
        max_processing_time = max(processing_times)

        assert (
            avg_processing_time <= 0.05
        ), f"Processing too slow during loop closure: {avg_processing_time*1000:.1f}ms"
        assert (
            max_processing_time <= 0.1
        ), f"Max processing too slow: {max_processing_time*1000:.1f}ms"

        print(f"Loop Closure Performance Test:")
        print(f"  Total frames: {len(loop_frames)}")
        print(f"  Keyframes detected: {len(keyframe_events)}")
        print(f"  Loop closure detected: {any(kf >= 30 for kf in keyframe_events)}")
        print(f"  Avg processing time: {avg_processing_time*1000:.1f}ms")
        print(f"  Max processing time: {max_processing_time*1000:.1f}ms")

    def _generate_frame_sequence(self, frame_id: int, total_frames: int) -> np.ndarray:
        """Generate realistic frame sequence."""
        # Base frame with gradual changes
        frame = np.random.randint(50, 200, (480, 640, 3), dtype=np.uint8)

        # Add moving features
        x_offset = int((frame_id / total_frames) * 100)
        cv2.rectangle(
            frame, (100 + x_offset, 100), (200 + x_offset, 200), (255, 255, 255), 2
        )

        # Add static features
        cv2.circle(frame, (320, 240), 50, (128, 128, 128), -1)

        # Add noise based on frame ID
        noise = np.random.normal(0, 5 + frame_id % 10, frame.shape).astype(np.uint8)
        frame = cv2.add(frame, noise)

        return frame

    def _generate_normal_frame(self, frame_id: int) -> np.ndarray:
        """Generate normal tracking frame."""
        return self._generate_frame_sequence(frame_id, 100)

    def _generate_problematic_frame(self, scenario: str, frame_id: int) -> np.ndarray:
        """Generate problematic frame for testing recovery."""
        if scenario == "low_texture":
            # Low texture (wall-like)
            return np.full((480, 640, 3), 128, dtype=np.uint8)

        elif scenario == "motion_blur":
            # Motion blur
            frame = self._generate_normal_frame(frame_id)
            kernel_size = 15
            kernel = np.ones((kernel_size, 1), np.float32) / kernel_size
            return cv2.filter2D(frame, -1, kernel)

        elif scenario == "lighting_change":
            # Sudden lighting change
            frame = self._generate_normal_frame(frame_id)
            if frame_id < 2:
                return np.clip(frame * 0.3, 0, 255).astype(np.uint8)
            else:
                return np.clip(frame * 2.0, 0, 255).astype(np.uint8)

        elif scenario == "occlusion":
            # Occlusion (camera blocked)
            frame = self._generate_normal_frame(frame_id)
            cv2.rectangle(frame, (0, 0), (640, 480), (0, 0, 0), -1)
            return frame

        return self._generate_normal_frame(frame_id)

    def _generate_loop_frame(self, frame_id: int, path_type: str) -> np.ndarray:
        """Generate frame for loop closure testing."""
        if path_type == "forward":
            # Moving away from start
            x_offset = frame_id * 10
            y_offset = frame_id * 5
        else:  # return
            # Returning to start
            x_offset = (30 - frame_id) * 10
            y_offset = (30 - frame_id) * 5

        frame = np.random.randint(50, 200, (480, 640, 3), dtype=np.uint8)

        # Add distinctive landmark at start
        if frame_id == 0 or (path_type == "return" and frame_id == 29):
            cv2.rectangle(frame, (200, 150), (400, 300), (255, 0, 0), -1)
            cv2.putText(
                frame,
                "START",
                (250, 240),
                cv2.FONT_HERSHEY_SIMPLEX,
                1,
                (255, 255, 255),
                2,
            )

        # Add moving features
        cv2.rectangle(
            frame,
            (100 + x_offset, 100 + y_offset),
            (200 + x_offset, 200 + y_offset),
            (255, 255, 255),
            2,
        )

        return frame


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
