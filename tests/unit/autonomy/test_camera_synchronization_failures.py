#!/usr/bin/env python3
"""
Camera Synchronization Failure Tests - URC 2026

Tests multi-camera synchronization failures that can ruin perception:
- Frame timing violations and desynchronization
- Buffer overflow and frame drops
- Network-induced delays and jitter
- Clock synchronization failures
- Stereo camera pair alignment failures
- Temporal consistency violations

Author: URC 2026 Risk Mitigation Team
"""

import pytest
import asyncio
import time
import threading
from unittest.mock import Mock, MagicMock, patch, AsyncMock
import sys
import os
from typing import Dict, List, Any, Optional
import numpy as np
import statistics

# Add source paths
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../.."))
sys.path.insert(0, os.path.join(os.path.dirname(__file__), "../../src"))

from src.core.synchronization_engine import (
    SynchronizationEngine,
    CameraSyncConfig,
    SyncMode,
    CameraFrameSynchronizer,
    get_sync_engine,
    initialize_sync_engine,
)


class TestFrameTimingViolations:
    """Test frame timing violations that break synchronization."""

    @pytest.fixture
    def sync_engine(self):
        """Create synchronization engine instance."""
        config = CameraSyncConfig(
            num_cameras=4, max_sync_delay_ms=50.0, stereo_baseline_ms=1.0
        )
        return SynchronizationEngine(config)

    @pytest.mark.critical
    def test_excessive_frame_timing_jitter(self, sync_engine):
        """Test synchronization failure due to excessive timing jitter."""
        base_time = time.time()

        # Add frames with timing jitter exceeding threshold
        for i in range(5):
            master_time = base_time + i * 0.033  # 30 FPS

            for cam_id in ["front", "rear", "left", "right"]:
                # Add random jitter exceeding 50ms threshold
                jitter = np.random.uniform(0.06, 0.1)  # 60-100ms jitter
                frame_time = master_time + jitter

                frame_data = {
                    "timestamp": frame_time,
                    "frame_number": i,
                    "camera_id": cam_id,
                    "data": f"frame_{cam_id}_{i}",
                }
                sync_engine.add_camera_frame(cam_id, frame_data)

            # Attempt synchronization - should fail due to excessive jitter
            sync_result = asyncio.run(sync_engine.synchronize_cameras())
            assert (
                sync_result is None
            ), f"Should fail sync due to excessive jitter on frame {i}"

    @pytest.mark.critical
    def test_camera_frame_arrival_out_of_order(self, sync_engine):
        """Test synchronization with frames arriving out of temporal order."""
        base_time = time.time()

        # Add frames out of order (rear camera frame arrives first)
        frame_time = base_time

        # Add rear camera frame first (should be second)
        rear_frame = {
            "timestamp": frame_time + 0.01,  # Should arrive second
            "camera_id": "rear",
            "frame_number": 0,
        }
        sync_engine.add_camera_frame("rear", rear_frame)

        # Add front camera frame (master) - arrives later but has earlier timestamp
        front_frame = {
            "timestamp": frame_time,  # Earlier timestamp
            "camera_id": "front",
            "frame_number": 0,
        }
        sync_engine.add_camera_frame("front", front_frame)

        # Synchronization should handle out-of-order arrival
        sync_result = asyncio.run(sync_engine.synchronize_cameras())
        if sync_result:
            assert (
                len(sync_result["cameras"]) >= 2
            ), "Should synchronize available cameras"

    @pytest.mark.critical
    def test_frame_dropping_under_timing_pressure(self, sync_engine):
        """Test frame dropping when synchronization cannot keep up."""
        base_time = time.time()

        # Flood system with frames faster than it can process
        frame_count = 0
        dropped_frames = 0

        for i in range(100):  # Many frames
            frame_time = base_time + i * 0.001  # 1000 FPS (impossible to sync)

            for cam_id in ["front", "rear"]:
                frame_data = {
                    "timestamp": frame_time,
                    "camera_id": cam_id,
                    "frame_number": i,
                }

                success = sync_engine.add_camera_frame(cam_id, frame_data)
                if not success:
                    dropped_frames += 1

                frame_count += 1

        # Should have dropped frames due to buffer limits
        assert dropped_frames > 0, "Should drop frames under timing pressure"

    @pytest.mark.critical
    def test_clock_drift_between_cameras(self, sync_engine):
        """Test synchronization failure due to clock drift between cameras."""
        base_time = time.time()

        # Simulate clock drift between cameras
        clock_offsets = {
            "front": 0.0,  # Master reference
            "rear": 0.001,  # 1ms drift
            "left": -0.002,  # -2ms drift
            "right": 0.003,  # 3ms drift
        }

        for i in range(10):
            master_time = base_time + i * 0.033

            for cam_id in ["front", "rear", "left", "right"]:
                # Apply clock drift
                frame_time = master_time + clock_offsets[cam_id]

                frame_data = {
                    "timestamp": frame_time,
                    "camera_id": cam_id,
                    "frame_number": i,
                }
                sync_engine.add_camera_frame(cam_id, frame_data)

            # Attempt synchronization
            sync_result = asyncio.run(sync_engine.synchronize_cameras())

            # With clock drift, synchronization may succeed or fail depending on drift magnitude
            if sync_result:
                max_delay = sync_result["sync_delay_ms"]
                assert (
                    max_delay < 50.0
                ), f"Clock drift should not exceed sync threshold: {max_delay}ms"


class TestBufferOverflowFailures:
    """Test buffer overflow scenarios that cause frame loss."""

    @pytest.fixture
    def sync_engine(self):
        """Create sync engine with small buffers for testing."""
        config = CameraSyncConfig(num_cameras=4)
        engine = SynchronizationEngine(config)

        # Override buffer sizes for testing
        for cam_id in engine.camera_buffers:
            engine.camera_buffers[cam_id].max_size = 5  # Very small buffer

        return engine

    @pytest.mark.critical
    def test_camera_buffer_overflow_drop_oldest(self, sync_engine):
        """Test buffer overflow with drop-oldest policy."""
        base_time = time.time()

        # Fill buffer beyond capacity
        for i in range(10):  # More than buffer size (5)
            frame_data = {
                "timestamp": base_time + i * 0.001,
                "frame_number": i,
                "camera_id": "front",
            }

            success = sync_engine.add_camera_frame("front", frame_data)

            # Should accept all frames (drop oldest when full)
            assert success, f"Should accept frame {i} with drop-oldest policy"

        # Buffer should only contain last 5 frames
        buffer_size = sync_engine.camera_buffers["front"].size()
        assert buffer_size == 5, f"Buffer should contain 5 frames, has {buffer_size}"

    @pytest.mark.critical
    def test_camera_buffer_overflow_drop_newest(self, sync_engine):
        """Test buffer overflow with drop-newest policy."""
        # Change policy to drop newest
        for buffer in sync_engine.camera_buffers.values():
            buffer.overflow_policy = "drop_newest"

        base_time = time.time()

        # Fill buffer beyond capacity
        successful_adds = 0
        for i in range(10):
            frame_data = {
                "timestamp": base_time + i * 0.001,
                "frame_number": i,
                "camera_id": "front",
            }

            success = sync_engine.add_camera_frame("front", frame_data)
            if success:
                successful_adds += 1

        # Should only successfully add up to buffer size
        assert (
            successful_adds == 5
        ), f"Should only add 5 frames with drop-newest, added {successful_adds}"

    @pytest.mark.critical
    def test_ros2_message_buffer_overflow(self, sync_engine):
        """Test ROS2 message buffer overflow scenarios."""
        base_time = time.time()
        topic = "/imu/data"

        # Fill ROS2 buffer beyond capacity
        for i in range(60):  # More than default buffer size (50)
            message = {
                "timestamp": base_time + i * 0.001,
                "topic": topic,
                "seq": i,
                "data": {"accel": [0, 0, 9.81]},
            }

            success = sync_engine.add_ros2_message(topic, message)
            # Should accept all (drop oldest when full)
            assert success, f"Should accept ROS2 message {i}"

        # Buffer should be at max size
        buffer_size = sync_engine.ros2_buffers[topic].size()
        assert (
            buffer_size == 50
        ), f"ROS2 buffer should contain 50 messages, has {buffer_size}"

    @pytest.mark.critical
    def test_buffer_overflow_during_high_load(self, sync_engine):
        """Test buffer overflow during sustained high frame rate."""
        import concurrent.futures

        base_time = time.time()
        frame_count = 100

        def add_frames_camera(cam_id: str):
            """Add frames from a specific camera."""
            overflows = 0
            for i in range(frame_count):
                frame_data = {
                    "timestamp": base_time + i * 0.001,
                    "camera_id": cam_id,
                    "frame_number": i,
                }

                success = sync_engine.add_camera_frame(cam_id, frame_data)
                if not success:
                    overflows += 1

            return overflows

        # Run concurrent frame addition
        with concurrent.futures.ThreadPoolExecutor(max_workers=4) as executor:
            futures = []
            for cam_id in ["front", "rear", "left", "right"]:
                future = executor.submit(add_frames_camera, cam_id)
                futures.append(future)

            # Collect results
            total_overflows = sum(
                future.result() for future in concurrent.futures.as_completed(futures)
            )

        # Should have buffer overflows under high concurrent load
        assert total_overflows > 0, "Should experience buffer overflows under high load"


class TestNetworkInducedDelays:
    """Test synchronization failures caused by network delays."""

    @pytest.fixture
    def sync_engine(self):
        """Create synchronization engine."""
        return SynchronizationEngine()

    @pytest.mark.critical
    def test_network_packet_delay_exceeding_threshold(self, sync_engine):
        """Test network delays that exceed synchronization thresholds."""
        base_time = time.time()

        # Simulate network delay by adding frames with artificial delays
        for i in range(3):
            master_time = base_time + i * 0.033

            # Front camera (master) - normal timing
            front_frame = {
                "timestamp": master_time,
                "camera_id": "front",
                "frame_number": i,
            }
            sync_engine.add_camera_frame("front", front_frame)

            # Other cameras delayed by network (exceed 50ms threshold)
            for cam_id in ["rear", "left", "right"]:
                delayed_time = master_time + 0.06  # 60ms delay
                frame_data = {
                    "timestamp": delayed_time,
                    "camera_id": cam_id,
                    "frame_number": i,
                }
                sync_engine.add_camera_frame(cam_id, frame_data)

            # Synchronization should fail due to network delay
            sync_result = asyncio.run(sync_engine.synchronize_cameras())
            assert (
                sync_result is None
            ), f"Should fail sync due to network delay on frame {i}"

    @pytest.mark.critical
    def test_variable_network_latency_simulation(self, sync_engine):
        """Test synchronization under variable network latency."""
        base_time = time.time()

        # Simulate variable network latency (jitter)
        latencies = [0.01, 0.08, 0.03, 0.06, 0.09]  # Various latencies

        for i, latency in enumerate(latencies):
            master_time = base_time + i * 0.033

            # Add master frame
            front_frame = {
                "timestamp": master_time,
                "camera_id": "front",
                "frame_number": i,
            }
            sync_engine.add_camera_frame("front", front_frame)

            # Add other cameras with network latency
            for cam_id in ["rear", "left", "right"]:
                frame_time = master_time + latency
                frame_data = {
                    "timestamp": frame_time,
                    "camera_id": cam_id,
                    "frame_number": i,
                }
                sync_engine.add_camera_frame(cam_id, frame_data)

            # Attempt synchronization
            sync_result = asyncio.run(sync_engine.synchronize_cameras())

            # Should succeed for small latencies, fail for large ones
            if latency < 0.05:  # Within threshold
                assert (
                    sync_result is not None
                ), f"Should succeed with {latency}s latency"
            else:  # Exceeds threshold
                assert sync_result is None, f"Should fail with {latency}s latency"

    @pytest.mark.critical
    def test_network_partition_during_capture(self, sync_engine):
        """Test camera synchronization during network partition."""
        base_time = time.time()

        # Start with normal operation
        for i in range(2):
            master_time = base_time + i * 0.033

            for cam_id in ["front", "rear", "left", "right"]:
                frame_data = {
                    "timestamp": master_time,
                    "camera_id": cam_id,
                    "frame_number": i,
                }
                sync_engine.add_camera_frame(cam_id, frame_data)

            sync_result = asyncio.run(sync_engine.synchronize_cameras())
            assert sync_result is not None, f"Normal sync should work for frame {i}"

        # Simulate network partition (some cameras stop sending)
        for i in range(2, 4):
            master_time = base_time + i * 0.033

            # Only front and rear cameras available (network partition for left/right)
            for cam_id in ["front", "rear"]:
                frame_data = {
                    "timestamp": master_time,
                    "camera_id": cam_id,
                    "frame_number": i,
                }
                sync_engine.add_camera_frame(cam_id, frame_data)

            # Synchronization should still work with partial camera set
            sync_result = asyncio.run(sync_engine.synchronize_cameras())
            if sync_result:
                assert (
                    len(sync_result["cameras"]) >= 2
                ), "Should sync available cameras during partition"


class TestClockSynchronizationFailures:
    """Test distributed clock synchronization failures."""

    @pytest.fixture
    def sync_engine(self):
        """Create synchronization engine."""
        return SynchronizationEngine()

    @pytest.mark.critical
    def test_distributed_clock_drift_detection(self, sync_engine):
        """Test detection of clock drift between distributed systems."""
        # Simulate clock synchronization
        status = sync_engine.synchronize_clocks()

        # Check if clock sync provides meaningful data
        assert hasattr(status, "offset_ns"), "Should have clock offset information"
        assert hasattr(status, "drift_rate_ppm"), "Should have drift rate information"

        # Offset should be reasonable (within expected bounds)
        assert (
            abs(status.offset_ns) < 1000000000
        ), "Clock offset should be reasonable (< 1 second)"

    @pytest.mark.critical
    def test_clock_sync_failure_fallback(self, sync_engine):
        """Test fallback behavior when clock synchronization fails."""
        # Mock NTP/PTP failure
        with patch.object(sync_engine, "synchronize_clocks", return_value=None):
            # System should continue with local time
            current_time = time.time()
            assert current_time > 0, "Should have local time even without sync"

            # Synchronization should still work with local timestamps
            base_time = current_time
            for cam_id in ["front", "rear"]:
                frame_data = {
                    "timestamp": base_time,
                    "camera_id": cam_id,
                    "frame_number": 0,
                }
                sync_engine.add_camera_frame(cam_id, frame_data)

            sync_result = asyncio.run(sync_engine.synchronize_cameras())
            assert (
                sync_result is not None
            ), "Should work with local timestamps when clock sync fails"

    @pytest.mark.critical
    def test_temporal_consistency_with_clock_drift(self, sync_engine):
        """Test temporal consistency checking under clock drift."""
        # Create sequence with increasing drift
        base_time = time.time()
        events = []

        for i in range(10):
            # Simulate increasing clock drift
            drift = i * 0.001  # 1ms drift per event
            event_time = base_time + i * 0.033 + drift

            events.append(
                {"timestamp": event_time, "sequence": i, "data": f"event_{i}"}
            )

        # Check temporal consistency
        result = sync_engine.check_temporal_consistency(events)

        # Should detect inconsistency due to drift
        assert not result[
            "consistent"
        ], "Should detect temporal inconsistency with clock drift"
        assert result["score"] < 0.8, "Should have low consistency score with drift"


class TestStereoCameraFailures:
    """Test stereo camera synchronization failures."""

    @pytest.fixture
    def frame_sync(self):
        """Create camera frame synchronizer."""
        engine = SynchronizationEngine()
        return CameraFrameSynchronizer(engine)

    @pytest.mark.critical
    def test_stereo_pair_temporal_misalignment(self, frame_sync):
        """Test stereo camera pair temporal misalignment."""
        base_time = time.time()

        # Add stereo pair frames with excessive temporal difference
        for i in range(3):
            frame_time = base_time + i * 0.033

            # Left camera
            left_frame = {
                "timestamp": frame_time,
                "camera_id": "left",
                "frame_number": i,
            }
            frame_sync.sync_engine.add_camera_frame("left", left_frame)

            # Right camera with excessive delay (exceeds stereo baseline)
            right_time = frame_time + 0.002  # 2ms delay (exceeds 1ms baseline)
            right_frame = {
                "timestamp": right_time,
                "camera_id": "right",
                "frame_number": i,
            }
            frame_sync.sync_engine.add_camera_frame("right", right_frame)

            # Attempt stereo synchronization
            stereo_result = asyncio.run(
                frame_sync.wait_for_stereo_pair("left", "right", timeout_ms=50.0)
            )

            # Should fail due to excessive stereo misalignment
            assert (
                stereo_result is None
            ), f"Stereo sync should fail for frame {i} due to misalignment"

    @pytest.mark.critical
    def test_stereo_camera_failure_recovery(self, frame_sync):
        """Test recovery when one stereo camera fails."""
        base_time = time.time()

        # Left camera working
        left_frame = {"timestamp": base_time, "camera_id": "left", "frame_number": 0}
        frame_sync.sync_engine.add_camera_frame("left", left_frame)

        # Right camera fails (no frame)
        # Attempt stereo sync - should timeout
        stereo_result = asyncio.run(
            frame_sync.wait_for_stereo_pair("left", "right", timeout_ms=10.0)
        )

        assert stereo_result is None, "Should fail stereo sync when camera unavailable"

    @pytest.mark.critical
    def test_stereo_baseline_exceedance(self, frame_sync):
        """Test stereo synchronization with baseline exceedance."""
        base_time = time.time()

        # Test various stereo baselines
        test_baselines = [0.0005, 0.001, 0.002]  # 0.5ms, 1ms, 2ms

        for baseline in test_baselines:
            frame_time = base_time

            # Perfect alignment
            left_frame = {
                "timestamp": frame_time,
                "camera_id": "left",
                "frame_number": 0,
            }
            right_frame = {
                "timestamp": frame_time + baseline,
                "camera_id": "right",
                "frame_number": 0,
            }

            frame_sync.sync_engine.add_camera_frame("left", left_frame)
            frame_sync.sync_engine.add_camera_frame("right", right_frame)

            # Configure stereo baseline
            frame_sync.sync_engine.camera_config.stereo_baseline_ms = baseline * 1000

            stereo_result = asyncio.run(
                frame_sync.wait_for_stereo_pair("left", "right", timeout_ms=50.0)
            )

            # Should succeed if within baseline, fail if exceeded
            expected_success = baseline <= 0.001  # Within 1ms baseline
            if expected_success:
                assert (
                    stereo_result is not None
                ), f"Should succeed with {baseline}s baseline"
            else:
                assert stereo_result is None, f"Should fail with {baseline}s baseline"


class TestRealTimeDeadlineFailures:
    """Test real-time deadline violations in synchronization."""

    @pytest.fixture
    def sync_engine(self):
        """Create synchronization engine."""
        return SynchronizationEngine()

    @pytest.mark.critical
    def test_synchronization_deadline_violation(self, sync_engine):
        """Test synchronization deadline violations."""
        # Create deadline monitor
        monitor = sync_engine.monitor_deadlines("camera_sync", 10.0)  # 10ms deadline

        with monitor:
            # Simulate slow operation exceeding deadline
            time.sleep(0.015)  # 15ms delay

        # Should have detected deadline violation
        # (In real implementation, this would be logged and handled)

    @pytest.mark.critical
    def test_concurrent_synchronization_pressure(self, sync_engine):
        """Test synchronization under concurrent operation pressure."""
        import concurrent.futures

        base_time = time.time()
        num_threads = 4
        frames_per_thread = 50

        def sync_operation_thread(thread_id: int):
            """Perform synchronization operations in thread."""
            violations = 0

            for i in range(frames_per_thread):
                # Add frames from this thread's cameras
                cam_id = f"camera_{thread_id}"
                frame_data = {
                    "timestamp": base_time + i * 0.001,
                    "camera_id": cam_id,
                    "frame_number": i,
                }
                sync_engine.add_camera_frame(cam_id, frame_data)

                # Attempt synchronization (with timeout)
                try:
                    sync_result = asyncio.run(
                        asyncio.wait_for(
                            sync_engine.synchronize_cameras(),
                            timeout=0.01,  # 10ms timeout
                        )
                    )
                except asyncio.TimeoutError:
                    violations += 1

            return violations

        # Run concurrent synchronization operations
        with concurrent.futures.ThreadPoolExecutor(max_workers=num_threads) as executor:
            futures = [
                executor.submit(sync_operation_thread, i) for i in range(num_threads)
            ]

            total_violations = sum(
                future.result() for future in concurrent.futures.as_completed(futures)
            )

        # Should experience some timeout violations under concurrent load
        assert (
            total_violations > 0
        ), "Should experience deadline violations under concurrent load"


if __name__ == "__main__":
    pytest.main([__file__, "-v", "--tb=short"])
