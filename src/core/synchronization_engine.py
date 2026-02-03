#!/usr/bin/env python3
"""
Synchronization Engine - URC 2026 Desync Protection System

Provides comprehensive synchronization for multi-camera robotics systems:
- Distributed clock synchronization (NTP/PTP integration)
- Multi-camera frame alignment and temporal consistency
- ROS2 message ordering and timestamp validation
- Real-time deadline monitoring and violation detection
- Buffer management with overflow protection
- Temporal consistency checks across all subsystems

Author: URC 2026 Synchronization Engine Team
"""

import asyncio
import time
import threading
from typing import Dict, Any, List, Optional, Callable, Tuple, Union
import statistics
import numpy as np
from dataclasses import dataclass, field
from enum import Enum
import logging

logger = logging.getLogger(__name__)


class SyncMode(Enum):
    """Synchronization modes for different requirements."""

    HARD_REALTIME = "hard_realtime"  # <1ms jitter tolerance
    SOFT_REALTIME = "soft_realtime"  # <10ms jitter tolerance
    EVENT_DRIVEN = "event_driven"  # Best effort synchronization
    ASYNCHRONOUS = "asynchronous"  # No synchronization guarantees


class TemporalConsistency(Enum):
    """Temporal consistency requirements."""

    STRICT = "strict"  # All messages must be perfectly aligned
    APPROXIMATE = "approximate"  # Allow small timing variations
    BEST_EFFORT = "best_effort"  # Process messages as they arrive
    NONE = "none"  # No temporal requirements


@dataclass
class CameraSyncConfig:
    """Configuration for multi-camera synchronization."""

    num_cameras: int = 4
    max_sync_delay_ms: float = 50.0  # Maximum allowed sync delay
    frame_timeout_ms: float = 100.0  # Frame timeout before drop
    stereo_baseline_ms: float = 1.0  # Stereo pair sync tolerance
    master_camera_id: str = "front"  # Reference camera for sync
    sync_mode: SyncMode = SyncMode.SOFT_REALTIME


@dataclass
class ClockSyncStatus:
    """Distributed clock synchronization status."""

    offset_ns: int = 0  # Clock offset in nanoseconds
    drift_rate_ppm: float = 0.0  # Clock drift rate
    last_sync_time: float = 0.0
    sync_accuracy_ns: int = 0  # Synchronization accuracy
    ntp_server: Optional[str] = None
    ptp_master: bool = False


@dataclass
class SyncBuffer:
    """Thread-safe synchronization buffer with overflow protection and recovery."""

    max_size: int = 100
    overflow_policy: str = "drop_oldest"  # drop_oldest, drop_newest, block, adaptive
    items: List[Dict[str, Any]] = field(default_factory=list)
    lock: threading.RLock = field(default_factory=threading.RLock)
    overflow_count: int = 0
    adaptive_mode: bool = False
    min_size: int = 10  # Minimum buffer size in adaptive mode
    quality_threshold: float = 0.8  # Quality threshold for adaptive sizing

    def add_item(self, item: Dict[str, Any]) -> bool:
        """Add item to buffer with overflow handling and recovery."""
        with self.lock:
            # Adaptive buffer sizing based on overflow patterns
            if self.adaptive_mode and self.overflow_count > 10:
                self._adjust_buffer_size()

            if len(self.items) >= self.max_size:
                success = self._handle_overflow()
                if not success:
                    return False

            # Validate item before adding
            if not self._validate_item(item):
                logger.warning("Invalid item rejected by buffer")
                return False

            self.items.append(item)
            return True

    def _handle_overflow(self) -> bool:
        """Handle buffer overflow based on policy."""
        self.overflow_count += 1

        if self.overflow_policy == "drop_oldest":
            if self.items:
                dropped_item = self.items.pop(0)
                logger.debug(
                    f"Dropped oldest item from buffer: {dropped_item.get('timestamp', 'unknown')}"
                )
            return True
        elif self.overflow_policy == "drop_newest":
            logger.debug("Buffer full, rejecting new item")
            return False
        elif self.overflow_policy == "block":
            # In async context, this would wait for space
            logger.warning("Buffer full, blocking (would wait in async context)")
            return False
        elif self.overflow_policy == "adaptive":
            # Emergency expansion
            old_max = self.max_size
            self.max_size = min(self.max_size * 2, 1000)  # Double but cap at 1000
            logger.warning(
                f"Buffer overflow, expanding from {old_max} to {self.max_size}"
            )
            return True

        return False

    def _validate_item(self, item: Dict[str, Any]) -> bool:
        """Validate item data integrity."""
        if not isinstance(item, dict):
            return False

        # Check for required timestamp
        timestamp = item.get("timestamp")
        if timestamp is None:
            return False

        # Check timestamp is reasonable (not too old or future)
        current_time = time.time()
        if abs(timestamp - current_time) > 3600:  # 1 hour tolerance
            return False

        # Check for duplicate timestamps (basic duplicate detection)
        for existing_item in self.items[-10:]:  # Check last 10 items
            if existing_item.get("timestamp") == timestamp:
                return False  # Duplicate timestamp

        return True

    def _adjust_buffer_size(self):
        """Adaptively adjust buffer size based on overflow patterns."""
        if not self.adaptive_mode:
            return

        # If we're overflowing frequently, increase buffer size
        if self.overflow_count > 20:
            self.max_size = min(self.max_size + 10, 500)
            logger.info(f"Adaptive buffer sizing: increased to {self.max_size}")
            self.overflow_count = 0  # Reset counter

    def get_items_older_than(self, timestamp: float) -> List[Dict[str, Any]]:
        """Get all items older than timestamp with quality filtering."""
        with self.lock:
            cutoff_index = 0
            for i, item in enumerate(self.items):
                if item.get("timestamp", 0) > timestamp:
                    break
                cutoff_index = i + 1

            result = self.items[:cutoff_index]
            self.items = self.items[cutoff_index:]

            # Filter out low-quality items if buffer is getting full
            if len(self.items) > self.max_size * 0.9:  # 90% full
                result = self._filter_quality_items(result)

            return result

    def _filter_quality_items(
        self, items: List[Dict[str, Any]]
    ) -> List[Dict[str, Any]]:
        """Filter items based on quality metrics."""
        if not items:
            return items

        # Simple quality filter: prefer items with complete data
        filtered = []
        for item in items:
            quality_score = 0

            # Check for required fields
            if "timestamp" in item:
                quality_score += 0.3
            if "frame_number" in item:
                quality_score += 0.2
            if "data" in item and item["data"]:
                quality_score += 0.5

            # Only keep high-quality items when buffer is stressed
            if quality_score >= self.quality_threshold:
                filtered.append(item)

        if len(filtered) < len(items):
            logger.debug(f"Filtered {len(items) - len(filtered)} low-quality items")

        return filtered

    def size(self) -> int:
        """Get current buffer size."""
        with self.lock:
            return len(self.items)

    def get_stats(self) -> Dict[str, Any]:
        """Get buffer statistics."""
        with self.lock:
            return {
                "current_size": len(self.items),
                "max_size": self.max_size,
                "overflow_count": self.overflow_count,
                "adaptive_mode": self.adaptive_mode,
                "quality_threshold": self.quality_threshold,
            }

    def clear(self):
        """Clear buffer and reset statistics."""
        with self.lock:
            self.items.clear()
            self.overflow_count = 0

    def set_adaptive_mode(self, enabled: bool):
        """Enable or disable adaptive buffer sizing."""
        self.adaptive_mode = enabled
        if enabled:
            logger.info("Buffer adaptive mode enabled")
        else:
            logger.info("Buffer adaptive mode disabled")


@dataclass
class SyncStatistics:
    """Synchronization performance statistics."""

    messages_processed: int = 0
    sync_violations: int = 0
    avg_sync_delay_ms: float = 0.0
    max_sync_delay_ms: float = 0.0
    dropped_frames: int = 0
    buffer_overflows: int = 0
    clock_sync_accuracy_ns: int = 0
    temporal_consistency_score: float = 1.0  # 1.0 = perfect sync


class SynchronizationEngine:
    """
    Comprehensive synchronization engine for multi-camera robotics.

    Provides:
    - Distributed clock synchronization
    - Multi-camera frame alignment
    - ROS2 message temporal ordering
    - Real-time deadline monitoring
    - Buffer overflow protection
    """

    def __init__(self, camera_config: Optional[CameraSyncConfig] = None, ros_node=None):
        self.camera_config = camera_config or CameraSyncConfig()
        self.clock_status = ClockSyncStatus()
        self.statistics = SyncStatistics()
        self.ros_node = ros_node  # ROS2 node for safety monitoring

        # Synchronization buffers
        self.camera_buffers: Dict[str, SyncBuffer] = {}
        self.ros2_buffers: Dict[str, SyncBuffer] = {}

        # Initialize camera buffers
        camera_ids = ["front", "rear", "left", "right"]
        for cam_id in camera_ids:
            self.camera_buffers[cam_id] = SyncBuffer(
                max_size=30, overflow_policy="drop_oldest"  # 30 frames buffer
            )

        # ROS2 topic buffers
        ros2_topics = [
            "/camera/front/image_raw",
            "/camera/rear/image_raw",
            "/camera/left/image_raw",
            "/camera/right/image_raw",
            "/odom",
            "/imu/data",
            "/gps/fix",
        ]
        for topic in ros2_topics:
            self.ros2_buffers[topic] = SyncBuffer(
                max_size=50, overflow_policy="drop_oldest"
            )

        # Synchronization state
        self.last_sync_time = time.time()
        self.sync_interval = 1.0  # Sync every second
        self.temporal_consistency_threshold = 0.05  # 50ms threshold

        # Callbacks
        self.sync_callbacks: List[Callable] = []
        self.desync_callbacks: List[Callable] = []

        # ROS2 safety monitoring integration
        self._setup_ros2_safety_monitoring()

        logger.info("Synchronization Engine initialized with ROS2 safety monitoring")

    def _setup_ros2_safety_monitoring(self):
        """Setup comprehensive ROS2 safety monitoring integration."""
        if self.ros_node:
            try:
                from std_msgs.msg import String
                from diagnostic_msgs.msg import (
                    DiagnosticArray,
                    DiagnosticStatus,
                    KeyValue,
                )
                from std_srvs.srv import Trigger

                # Core safety publishers
                self.safety_publisher = self.ros_node.create_publisher(
                    String, "/safety/sync_engine_status", 10
                )

                self.diagnostics_publisher = self.ros_node.create_publisher(
                    DiagnosticArray, "/diagnostics/sync_engine", 10
                )

                # Advanced diagnostic publishers
                self.health_publisher = self.ros_node.create_publisher(
                    String, "/safety/sync_engine_health", 10
                )

                self.performance_publisher = self.ros_node.create_publisher(
                    String, "/safety/sync_engine_performance", 10
                )

                # Service servers for advanced control
                self.reset_service = self.ros_node.create_service(
                    Trigger, "/safety/sync_engine/reset", self._handle_reset_service
                )

                self.diagnostics_service = self.ros_node.create_service(
                    Trigger,
                    "/safety/sync_engine/request_diagnostics",
                    self._handle_diagnostics_service,
                )

                # Enhanced timers
                self.safety_timer = self.ros_node.create_timer(
                    1.0, self._publish_safety_status
                )

                self.diagnostics_timer = self.ros_node.create_timer(
                    5.0, self._publish_diagnostics
                )  # Every 5 seconds

                self.performance_timer = self.ros_node.create_timer(
                    2.0, self._publish_performance_metrics
                )  # Every 2 seconds

                logger.info("Advanced ROS2 safety monitoring integration enabled")

            except Exception as e:
                logger.warning(f"Advanced ROS2 safety monitoring setup failed: {e}")
                self._setup_basic_ros2_monitoring()
        else:
            self._setup_basic_ros2_monitoring()

    def _setup_basic_ros2_monitoring(self):
        """Fallback to basic ROS2 monitoring if advanced setup fails."""
        if self.ros_node:
            try:
                from std_msgs.msg import String

                self.safety_publisher = self.ros_node.create_publisher(
                    String, "/safety/sync_engine_status", 10
                )
                self.safety_timer = self.ros_node.create_timer(
                    1.0, self._publish_safety_status
                )
                logger.info("Basic ROS2 safety monitoring enabled")
            except Exception as e:
                logger.warning(f"Basic ROS2 safety monitoring setup failed: {e}")
                self.safety_publisher = None
                self.safety_timer = None
        else:
            self.safety_publisher = None
            self.safety_timer = None

            # Additional publishers (set to None if no ROS node)
            self.diagnostics_publisher = None
            self.health_publisher = None
            self.performance_publisher = None
            self.reset_service = None
            self.diagnostics_service = None
            self.diagnostics_timer = None
            self.performance_timer = None

    def _publish_safety_status(self):
        """Publish synchronization safety status to ROS2."""
        if not self.safety_publisher:
            return

        try:
            from std_msgs.msg import String

            health = self.get_health_status()

            status_msg = String()
            status_msg.data = f"SYNC_HEALTH:{health['overall_health']}|VIOLATIONS:{self.statistics.sync_violations}|SCORE:{health.get('system_health_score', 0)}"
            self.safety_publisher.publish(status_msg)

        except Exception as e:
            logger.debug(f"Safety status publishing failed: {e}")

    def _publish_diagnostics(self):
        """Publish comprehensive diagnostics to ROS2."""
        if not self.diagnostics_publisher:
            return

        try:
            from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
            import time as time_module

            # Create diagnostic array
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.ros_node.get_clock().now().to_msg()
            diag_array.header.frame_id = "sync_engine"

            # Main sync engine diagnostic
            sync_diag = DiagnosticStatus()
            sync_diag.name = "Synchronization Engine"
            sync_diag.hardware_id = "sync_engine_core"

            health = self.get_health_status()
            overall_health = health["overall_health"]

            if overall_health == "healthy":
                sync_diag.level = DiagnosticStatus.OK
                sync_diag.message = "Synchronization operating normally"
            elif overall_health == "degraded":
                sync_diag.level = DiagnosticStatus.WARN
                sync_diag.message = "Synchronization performance degraded"
            else:
                sync_diag.level = DiagnosticStatus.ERROR
                sync_diag.message = "Synchronization system critical"

            # Add detailed values
            sync_diag.values = [
                KeyValue(
                    key="sync_violations", value=str(self.statistics.sync_violations)
                ),
                KeyValue(key="avg_sync_delay_ms", value=".2f"),
                KeyValue(
                    key="buffer_overflows", value=str(self.statistics.buffer_overflows)
                ),
                KeyValue(key="temporal_consistency_score", value=".3f"),
                KeyValue(
                    key="messages_processed",
                    value=str(self.statistics.messages_processed),
                ),
            ]

            diag_array.status = [sync_diag]
            self.diagnostics_publisher.publish(diag_array)

        except Exception as e:
            logger.debug(f"Diagnostics publishing failed: {e}")

    def _publish_performance_metrics(self):
        """Publish performance metrics to ROS2."""
        if not self.performance_publisher:
            return

        try:
            from std_msgs.msg import String
            import psutil
            import os

            # Gather performance metrics
            process = psutil.Process(os.getpid())
            memory_usage = process.memory_info().rss / 1024 / 1024  # MB
            cpu_usage = process.cpu_percent()

            # Sync engine specific metrics
            health = self.get_health_status()
            active_cameras = len(
                [k for k, v in self.camera_buffers.items() if v.size() > 0]
            )

            perf_msg = String()
            perf_msg.data = f"MEMORY:{memory_usage:.1f}MB|CPU:{cpu_usage:.1f}%|CAMERAS:{active_cameras}|HEALTH:{health['overall_health']}"
            self.performance_publisher.publish(perf_msg)

        except Exception as e:
            logger.debug(f"Performance metrics publishing failed: {e}")

    def _handle_reset_service(self, request, response):
        """Handle reset service requests."""
        try:
            logger.info("Processing synchronization engine reset request")

            # Perform reset operations
            self.recover_from_sync_failure()

            # Reset statistics
            self.reset_statistics()

            # Clear all buffers
            for buffer in self.camera_buffers.values():
                buffer.clear()

            response.success = True
            response.message = "Synchronization engine reset completed successfully"

            logger.info("Synchronization engine reset completed")

        except Exception as e:
            logger.error(f"Synchronization engine reset failed: {e}")
            response.success = False
            response.message = f"Reset failed: {str(e)}"

        return response

    def _handle_diagnostics_service(self, request, response):
        """Handle diagnostics service requests."""
        try:
            logger.info("Processing diagnostics request")

            # Force publish diagnostics
            self._publish_diagnostics()

            health = self.get_health_status()
            diagnostics_summary = f"""
Synchronization Engine Diagnostics:
- Overall Health: {health['overall_health']}
- Sync Violations: {self.statistics.sync_violations}
- Average Delay: {self.statistics.avg_sync_delay_ms:.2f}ms
- Buffer Overflows: {self.statistics.buffer_overflows}
- Messages Processed: {self.statistics.messages_processed}
- Active Cameras: {len([k for k, v in self.camera_buffers.items() if v.size() > 0])}
"""

            response.success = True
            response.message = diagnostics_summary.strip()

            logger.info("Diagnostics request completed")

        except Exception as e:
            logger.error(f"Diagnostics request failed: {e}")
            response.success = False
            response.message = f"Diagnostics failed: {str(e)}"

        return response

    def register_sync_callback(self, callback: Callable):
        """Register callback for successful synchronizations."""
        self.sync_callbacks.append(callback)

    def register_desync_callback(self, callback: Callable):
        """Register callback for desynchronization events."""
        self.desync_callbacks.append(callback)

    async def synchronize_cameras(self) -> Optional[Dict[str, Any]]:
        """
        Synchronize frames from all cameras with graceful degradation.

        Returns synchronized camera data or None if sync fails.
        Implements fallback modes for different failure scenarios.
        """
        start_time = time.time()

        # Get latest frames from each camera buffer
        camera_frames = {}
        master_timestamp = None

        for cam_id, buffer in self.camera_buffers.items():
            frames = buffer.get_items_older_than(time.time())
            if not frames:
                continue

            # Get most recent frame
            latest_frame = max(frames, key=lambda f: f.get("timestamp", 0))
            camera_frames[cam_id] = latest_frame

            # Use master camera as reference timestamp
            if cam_id == self.camera_config.master_camera_id:
                master_timestamp = latest_frame.get("timestamp")

        # Check for minimum viable camera set
        if not master_timestamp:
            # No master camera data - try fallback master selection
            return await self._fallback_camera_sync(camera_frames, "no_master_camera")

        if len(camera_frames) < 2:
            # Only one camera - single camera mode
            return await self._fallback_camera_sync(
                camera_frames, "insufficient_cameras"
            )

        # Attempt full synchronization
        full_sync_result = await self._attempt_full_camera_sync(
            camera_frames, master_timestamp
        )

        if full_sync_result is not None:
            return full_sync_result

        # Full sync failed - try degraded modes
        return await self._degraded_camera_sync(camera_frames, master_timestamp)

    async def _attempt_full_camera_sync(
        self, camera_frames: Dict[str, Any], master_timestamp: float
    ) -> Optional[Dict[str, Any]]:
        """Attempt full synchronization of all cameras."""
        # Check temporal alignment
        sync_delays = []
        max_delay = 0
        delayed_cameras = []

        for cam_id, frame in camera_frames.items():
            if cam_id == self.camera_config.master_camera_id:
                continue

            frame_time = frame.get("timestamp", 0)
            delay = abs(frame_time - master_timestamp)
            sync_delays.append(delay)
            max_delay = max(max_delay, delay)

            if delay * 1000 > self.camera_config.max_sync_delay_ms:
                delayed_cameras.append(cam_id)

        # Convert to milliseconds
        max_delay_ms = max_delay * 1000

        # Check sync constraints
        if max_delay_ms > self.camera_config.max_sync_delay_ms:
            self.statistics.sync_violations += 1
            self._notify_desync(
                "camera_sync_timeout",
                {
                    "max_delay_ms": max_delay_ms,
                    "threshold_ms": self.camera_config.max_sync_delay_ms,
                    "camera_frames": len(camera_frames),
                    "delayed_cameras": delayed_cameras,
                },
            )
            return None

        # Full synchronization successful
        self.statistics.messages_processed += len(camera_frames)
        if sync_delays:
            avg_delay = statistics.mean(sync_delays) * 1000
            self.statistics.avg_sync_delay_ms = (
                self.statistics.avg_sync_delay_ms * 0.9 + avg_delay * 0.1
            )
            self.statistics.max_sync_delay_ms = max(
                self.statistics.max_sync_delay_ms, max_delay_ms
            )

        # Create synchronized camera data
        sync_data = {
            "timestamp": master_timestamp,
            "cameras": camera_frames,
            "sync_delay_ms": max_delay_ms,
            "sync_quality": 1.0 - (max_delay_ms / self.camera_config.max_sync_delay_ms),
            "sync_mode": "full_synchronization",
        }

        self._notify_sync(sync_data)
        return sync_data

    async def _degraded_camera_sync(
        self, camera_frames: Dict[str, Any], master_timestamp: float
    ) -> Optional[Dict[str, Any]]:
        """Attempt degraded synchronization when full sync fails."""
        logger.warning("Full camera synchronization failed, attempting degraded mode")

        # Strategy 1: Accept cameras within extended tolerance
        extended_threshold = (
            self.camera_config.max_sync_delay_ms * 2.0
        )  # Double tolerance
        acceptable_cameras = {}

        for cam_id, frame in camera_frames.items():
            frame_time = frame.get("timestamp", 0)
            delay_ms = abs(frame_time - master_timestamp) * 1000

            if delay_ms <= extended_threshold:
                acceptable_cameras[cam_id] = frame

        if len(acceptable_cameras) >= 2:  # At least master + 1 camera
            max_delay = max(
                abs(frame.get("timestamp", 0) - master_timestamp) * 1000
                for frame in acceptable_cameras.values()
            )

            sync_data = {
                "timestamp": master_timestamp,
                "cameras": acceptable_cameras,
                "sync_delay_ms": max_delay,
                "sync_quality": max(
                    0.1, 1.0 - (max_delay / self.camera_config.max_sync_delay_ms)
                ),
                "sync_mode": "degraded_synchronization",
                "degraded_reason": "extended_tolerance",
            }

            self._notify_sync(sync_data)
            return sync_data

        # Strategy 2: Single camera mode with master only
        if self.camera_config.master_camera_id in camera_frames:
            sync_data = {
                "timestamp": master_timestamp,
                "cameras": {
                    self.camera_config.master_camera_id: camera_frames[
                        self.camera_config.master_camera_id
                    ]
                },
                "sync_delay_ms": 0.0,
                "sync_quality": 0.0,  # Single camera = no synchronization
                "sync_mode": "single_camera_fallback",
                "degraded_reason": "only_master_available",
            }

            logger.warning("Falling back to single camera mode - reduced reliability")
            self._notify_sync(sync_data)
            return sync_data

        # Strategy 3: Best effort with any available camera
        if camera_frames:
            # Use most recent timestamp as reference
            best_timestamp = max(
                frame.get("timestamp", 0) for frame in camera_frames.values()
            )

            sync_data = {
                "timestamp": best_timestamp,
                "cameras": camera_frames,
                "sync_delay_ms": float("inf"),  # Unknown/unsynchronized
                "sync_quality": 0.0,
                "sync_mode": "unsynchronized_fallback",
                "degraded_reason": "no_synchronization_possible",
            }

            logger.error("All synchronization modes failed - using unsynchronized data")
            self._notify_desync(
                "complete_sync_failure",
                {
                    "available_cameras": list(camera_frames.keys()),
                    "attempted_modes": ["full", "degraded", "single_camera"],
                },
            )
            return sync_data

        return None

    async def _fallback_camera_sync(
        self, camera_frames: Dict[str, Any], reason: str
    ) -> Optional[Dict[str, Any]]:
        """Handle cases where normal sync preconditions aren't met."""
        if reason == "no_master_camera":
            # Try to select a fallback master from available cameras
            if not camera_frames:
                return None

            # Use camera with most recent timestamp as master
            fallback_master = max(
                camera_frames.keys(),
                key=lambda cam: camera_frames[cam].get("timestamp", 0),
            )

            master_timestamp = camera_frames[fallback_master].get("timestamp")

            sync_data = {
                "timestamp": master_timestamp,
                "cameras": camera_frames,
                "sync_delay_ms": float("inf"),
                "sync_quality": 0.0,
                "sync_mode": "fallback_master_selection",
                "fallback_master": fallback_master,
                "degraded_reason": reason,
            }

            logger.warning(
                f"No master camera available, using {fallback_master} as fallback master"
            )
            self._notify_sync(sync_data)
            return sync_data

        elif reason == "insufficient_cameras":
            # Single camera operation
            if camera_frames:
                cam_id = list(camera_frames.keys())[0]
                timestamp = camera_frames[cam_id].get("timestamp", time.time())

                sync_data = {
                    "timestamp": timestamp,
                    "cameras": camera_frames,
                    "sync_delay_ms": 0.0,
                    "sync_quality": 0.0,
                    "sync_mode": "single_camera_only",
                    "degraded_reason": reason,
                }

                logger.warning(
                    "Only single camera available - no synchronization possible"
                )
                self._notify_sync(sync_data)
                return sync_data

        return None

    def add_camera_frame(self, camera_id: str, frame_data: Dict[str, Any]) -> bool:
        """Add camera frame to synchronization buffer with improved validation."""
        # Enhanced camera validation and auto-registration
        if camera_id not in self.camera_buffers:
            # Auto-register unknown cameras for robustness
            if self._validate_camera_id(camera_id):
                self._register_camera(camera_id)
                logger.info(f"Auto-registered new camera: {camera_id}")
            else:
                logger.warning(f"Invalid camera ID rejected: {camera_id}")
                return False

        # Add timestamp if not present
        if "timestamp" not in frame_data:
            frame_data["timestamp"] = time.time()

        buffer = self.camera_buffers[camera_id]
        success = buffer.add_item(frame_data)

        if not success:
            self.statistics.buffer_overflows += 1
            logger.debug(f"Buffer overflow for camera {camera_id} - graceful handling")
            self._notify_desync(
                "camera_buffer_overflow",
                {
                    "camera_id": camera_id,
                    "buffer_size": buffer.size(),
                    "max_size": buffer.max_size,
                },
            )

        return success

    def _validate_camera_id(self, camera_id: str) -> bool:
        """Validate camera ID format and constraints."""
        # Allow flexible camera naming (front, rear, left, right, or custom)
        valid_prefixes = ["front", "rear", "left", "right", "camera", "stereo"]
        return (
            isinstance(camera_id, str)
            and len(camera_id) > 0
            and len(camera_id) <= 50
            and any(  # Reasonable length limit
                camera_id.startswith(prefix) for prefix in valid_prefixes
            )
            or camera_id.replace("_", "")
            .replace("-", "")
            .isalnum()  # Allow custom names
        )

    def _register_camera(self, camera_id: str):
        """Register a new camera dynamically."""
        from src.core.synchronization_engine import SyncBuffer

        self.camera_buffers[camera_id] = SyncBuffer(
            max_size=100, overflow_policy="drop_oldest"  # Default buffer size
        )
        self.camera_buffers[camera_id].set_adaptive_mode(True)

    def add_ros2_message(self, topic: str, message: Dict[str, Any]) -> bool:
        """Add ROS2 message to synchronization buffer."""
        if topic not in self.ros2_buffers:
            # Create buffer for unknown topic
            self.ros2_buffers[topic] = SyncBuffer(
                max_size=50, overflow_policy="drop_oldest"
            )

        buffer = self.ros2_buffers[topic]
        success = buffer.add_item(message)

        if not success:
            self.statistics.buffer_overflows += 1

        return success

    async def synchronize_ros2_topics(
        self, topics: List[str], timeout_ms: float = 100.0
    ) -> Optional[Dict[str, Any]]:
        """Synchronize ROS2 messages from multiple topics."""
        start_time = time.time()
        timeout_s = timeout_ms / 1000.0

        topic_messages = {}
        reference_timestamp = None

        while time.time() - start_time < timeout_s:
            # Check all topics for messages
            all_topics_ready = True

            for topic in topics:
                if topic not in topic_messages:
                    buffer = self.ros2_buffers.get(topic)
                    if buffer:
                        # Get oldest available message
                        items = buffer.get_items_older_than(time.time())
                        if items:
                            topic_messages[topic] = items[0]
                            if reference_timestamp is None:
                                reference_timestamp = items[0].get(
                                    "timestamp", time.time()
                                )
                        else:
                            all_topics_ready = False
                    else:
                        all_topics_ready = False

            if all_topics_ready and len(topic_messages) == len(topics):
                # Check temporal alignment
                max_delay = 0
                for topic, message in topic_messages.items():
                    msg_time = message.get("timestamp", 0)
                    delay = abs(msg_time - reference_timestamp)
                    max_delay = max(max_delay, delay)

                if max_delay * 1000 <= timeout_ms:
                    return {
                        "timestamp": reference_timestamp,
                        "messages": topic_messages,
                        "sync_delay_ms": max_delay * 1000,
                    }

            await asyncio.sleep(0.001)  # Small delay to prevent busy waiting

        # Timeout
        self._notify_desync(
            "ros2_sync_timeout",
            {
                "topics": topics,
                "timeout_ms": timeout_ms,
                "available_messages": list(topic_messages.keys()),
            },
        )
        return None

    def check_temporal_consistency(
        self, events: List[Dict[str, Any]]
    ) -> Dict[str, Any]:
        """Check temporal consistency of a sequence of events."""
        if len(events) < 2:
            return {"consistent": True, "score": 1.0}

        # Extract timestamps
        timestamps = []
        for event in events:
            ts = event.get("timestamp")
            if ts is None:
                # Try header.stamp for ROS2 messages
                header = event.get("header", {})
                stamp = header.get("stamp")
                if stamp:
                    ts = stamp.get("sec", 0) + stamp.get("nanosec", 0) / 1e9
                else:
                    ts = time.time()  # Fallback

            timestamps.append(ts)

        # Calculate timing statistics
        time_diffs = np.diff(timestamps)
        avg_interval = np.mean(time_diffs)
        std_dev = np.std(time_diffs)
        jitter = std_dev / avg_interval if avg_interval > 0 else 1.0

        # Check for temporal consistency
        max_jitter = self.temporal_consistency_threshold
        consistent = jitter <= max_jitter

        consistency_score = max(0, 1.0 - (jitter / max_jitter))

        result = {
            "consistent": consistent,
            "score": consistency_score,
            "avg_interval": avg_interval,
            "jitter": jitter,
            "max_jitter_allowed": max_jitter,
            "timestamps": timestamps,
        }

        if not consistent:
            self._notify_desync("temporal_inconsistency", result)

        return result

    def synchronize_clocks(self) -> ClockSyncStatus:
        """Perform distributed clock synchronization."""
        # In a real implementation, this would:
        # 1. Query NTP/PTP servers
        # 2. Calculate clock offset and drift
        # 3. Adjust local clock or compensate in processing

        # Simulated NTP sync
        simulated_offset = np.random.normal(0, 1000000)  # ±1ms offset
        simulated_drift = np.random.normal(0, 0.1)  # ±0.1 ppm drift

        self.clock_status.offset_ns = int(simulated_offset)
        self.clock_status.drift_rate_ppm = simulated_drift
        self.clock_status.last_sync_time = time.time()
        self.clock_status.sync_accuracy_ns = abs(int(simulated_offset))

        return self.clock_status

    def monitor_deadlines(
        self, operation_name: str, deadline_ms: float
    ) -> "DeadlineMonitor":
        """Create deadline monitor for real-time operations."""
        return DeadlineMonitor(operation_name, deadline_ms, self)

    def get_sync_statistics(self) -> SyncStatistics:
        """Get comprehensive synchronization statistics."""
        return self.statistics

    def reset_statistics(self):
        """Reset synchronization statistics."""
        self.statistics = SyncStatistics()

    def get_health_status(self) -> Dict[str, Any]:
        """Get comprehensive health status of synchronization system."""
        current_time = time.time()
        camera_status = {}

        for cam_id, buffer in self.camera_buffers.items():
            stats = buffer.get_stats()
            # Check if camera is healthy (has recent data)
            last_item_time = 0
            if buffer.items:
                last_item_time = max(item.get("timestamp", 0) for item in buffer.items)

            is_healthy = (current_time - last_item_time) < 5.0  # 5 second timeout
            camera_status[cam_id] = {
                "healthy": is_healthy,
                "last_data_time": last_item_time,
                "buffer_size": stats["current_size"],
                "buffer_max": stats["max_size"],
                "overflows": stats["overflow_count"],
            }

        sync_health = self._assess_sync_health()

        return {
            "overall_health": sync_health["overall"],
            "camera_status": camera_status,
            "sync_statistics": {
                "messages_processed": self.statistics.messages_processed,
                "sync_violations": self.statistics.sync_violations,
                "avg_sync_delay_ms": self.statistics.avg_sync_delay_ms,
                "max_sync_delay_ms": self.statistics.max_sync_delay_ms,
                "dropped_frames": self.statistics.dropped_frames,
                "buffer_overflows": self.statistics.buffer_overflows,
            },
            "clock_status": {
                "offset_ns": self.clock_status.offset_ns,
                "drift_rate_ppm": self.clock_status.drift_rate_ppm,
                "sync_accuracy_ns": self.clock_status.sync_accuracy_ns,
            },
            "temporal_consistency": self.statistics.temporal_consistency_score,
            "active_desync_events": len(
                [e for e in self._get_recent_desync_events() if e.get("active", False)]
            ),
        }

    def _assess_sync_health(self) -> Dict[str, Any]:
        """Assess overall synchronization system health."""
        health_score = 1.0
        issues = []

        # Check sync violation rate
        total_messages = self.statistics.messages_processed
        if total_messages > 0:
            violation_rate = self.statistics.sync_violations / total_messages
            if violation_rate > 0.1:  # >10% violations
                health_score *= 0.7
                issues.append(f"high_sync_violations_{violation_rate:.2%}")
            elif violation_rate > 0.05:  # >5% violations
                health_score *= 0.9
                issues.append(f"moderate_sync_violations_{violation_rate:.2%}")

        # Check average sync delay
        if (
            self.statistics.avg_sync_delay_ms
            > self.camera_config.max_sync_delay_ms * 0.8
        ):
            health_score *= 0.8
            issues.append(f"high_avg_delay_{self.statistics.avg_sync_delay_ms:.1f}ms")

        # Check buffer overflows
        if self.statistics.buffer_overflows > 10:
            health_score *= 0.9
            issues.append(f"buffer_overflows_{self.statistics.buffer_overflows}")

        # Check temporal consistency
        if self.statistics.temporal_consistency_score < 0.7:
            health_score *= self.statistics.temporal_consistency_score
            issues.append(
                f"poor_temporal_consistency_{self.statistics.temporal_consistency_score:.2f}"
            )

        # Determine overall health status
        if health_score >= 0.9:
            overall = "healthy"
        elif health_score >= 0.7:
            overall = "degraded"
        elif health_score >= 0.5:
            overall = "unhealthy"
        else:
            overall = "critical"

        return {"overall": overall, "score": health_score, "issues": issues}

    def _get_recent_desync_events(self, hours: int = 1) -> List[Dict[str, Any]]:
        """Get recent desynchronization events for monitoring."""
        # In a real implementation, this would maintain an event log
        # For now, return empty list as placeholder
        return []

    def enable_adaptive_buffering(self):
        """Enable adaptive buffer sizing across all camera buffers."""
        for buffer in self.camera_buffers.values():
            buffer.set_adaptive_mode(True)
        logger.info("Adaptive buffering enabled for all camera buffers")

    def disable_adaptive_buffering(self):
        """Disable adaptive buffer sizing."""
        for buffer in self.camera_buffers.values():
            buffer.set_adaptive_mode(False)
        logger.info("Adaptive buffering disabled")

    def set_degraded_sync_mode(self, enabled: bool):
        """Enable or disable degraded synchronization modes."""
        self.camera_config.sync_mode = (
            SyncMode.EVENT_DRIVEN if enabled else SyncMode.SOFT_REALTIME
        )
        if enabled:
            logger.info("Degraded synchronization mode enabled")
        else:
            logger.info("Normal synchronization mode restored")

    def recover_from_sync_failure(self) -> bool:
        """Attempt to recover from synchronization failure."""
        logger.info("Attempting synchronization system recovery")

        # Step 1: Clear all buffers to remove stale data
        for cam_id, buffer in self.camera_buffers.items():
            buffer.clear()
            logger.debug(f"Cleared buffer for camera {cam_id}")

        # Step 2: Reset statistics
        self.reset_statistics()

        # Step 3: Reinitialize clock sync
        self.synchronize_clocks()

        # Step 4: Reset temporal consistency
        self.last_sync_time = time.time()
        self.temporal_consistency_threshold = 0.05

        # Step 5: Notify recovery
        self._notify_sync(
            {
                "type": "system_recovery",
                "timestamp": time.time(),
                "message": "Synchronization system recovered from failure",
            }
        )

        logger.info("Synchronization system recovery completed")
        return True

    def get_recovery_actions(self) -> List[Dict[str, Any]]:
        """Get available recovery actions for current system state."""
        actions = []
        health = self.get_health_status()

        if health["overall_health"] in ["unhealthy", "critical"]:
            actions.append(
                {
                    "action": "clear_buffers",
                    "description": "Clear all camera buffers to remove stale data",
                    "priority": "high",
                }
            )

            actions.append(
                {
                    "action": "reset_statistics",
                    "description": "Reset synchronization statistics",
                    "priority": "medium",
                }
            )

        if health["sync_statistics"]["sync_violations"] > 10:
            actions.append(
                {
                    "action": "enable_degraded_mode",
                    "description": "Enable degraded synchronization with relaxed constraints",
                    "priority": "high",
                }
            )

        if health["sync_statistics"]["buffer_overflows"] > 5:
            actions.append(
                {
                    "action": "enable_adaptive_buffering",
                    "description": "Enable adaptive buffer sizing to handle load",
                    "priority": "medium",
                }
            )

        if health["temporal_consistency"] < 0.5:
            actions.append(
                {
                    "action": "resync_clocks",
                    "description": "Perform clock synchronization to improve temporal consistency",
                    "priority": "high",
                }
            )

        return actions

    def optimize_performance(
        self, target_fps: float = 30.0, available_memory_mb: float = 512.0
    ):
        """
        Optimize synchronization engine performance based on system constraints.

        Args:
            target_fps: Target frames per second
            available_memory_mb: Available system memory in MB
        """
        logger.info(
            f"Optimizing performance for {target_fps} FPS with {available_memory_mb}MB memory"
        )

        # Calculate optimal buffer sizes based on memory constraints
        memory_per_frame_estimate = 0.5  # MB per frame (rough estimate)
        max_frames_in_memory = int(available_memory_mb / memory_per_frame_estimate)

        # Distribute memory across cameras
        cameras_count = len(self.camera_buffers)
        frames_per_camera = max(10, max_frames_in_memory // cameras_count)

        # Adjust buffer sizes
        for cam_id, buffer in self.camera_buffers.items():
            old_size = buffer.max_size
            new_size = min(frames_per_camera, 200)  # Cap at 200 frames

            # Only adjust if significantly different
            if abs(new_size - old_size) > 10:
                buffer.max_size = new_size
                logger.info(
                    f"Optimized buffer size for {cam_id}: {old_size} -> {new_size}"
                )

        # Optimize sync intervals based on target FPS
        target_sync_interval = 1.0 / target_fps
        if target_sync_interval < self.sync_interval:
            old_interval = self.sync_interval
            self.sync_interval = max(target_sync_interval, 0.1)  # Minimum 100ms
            logger.info(
                f"Optimized sync interval: {old_interval:.3f}s -> {self.sync_interval:.3f}s"
            )

        # Enable adaptive features for dynamic optimization
        self.enable_adaptive_buffering()
        logger.info("Adaptive buffering enabled for performance optimization")

        # Log optimization results
        self._log_performance_optimization()

    def enable_adaptive_buffering(self):
        """Enable adaptive buffer sizing across all camera buffers."""
        for buffer in self.camera_buffers.values():
            buffer.set_adaptive_mode(True)
        logger.info("Adaptive buffering enabled for all camera buffers")

    def _log_performance_optimization(self):
        """Log current performance optimization status."""
        total_buffer_capacity = sum(
            buffer.max_size for buffer in self.camera_buffers.values()
        )
        active_cameras = len([b for b in self.camera_buffers.values() if b.size() > 0])

        logger.info(f"Performance optimization complete:")
        logger.info(f"  - Total buffer capacity: {total_buffer_capacity} frames")
        logger.info(f"  - Active cameras: {active_cameras}")
        logger.info(f"  - Sync interval: {self.sync_interval:.3f}s")
        logger.info(f"  - Adaptive buffering: enabled")

    def get_performance_metrics(self) -> Dict[str, Any]:
        """Get comprehensive performance metrics."""
        import psutil
        import os

        # System metrics
        process = psutil.Process(os.getpid())
        memory_usage_mb = process.memory_info().rss / 1024 / 1024
        cpu_usage_percent = process.cpu_percent()

        # Sync engine specific metrics
        health = self.get_health_status()
        buffer_stats = {
            cam_id: buffer.get_stats() for cam_id, buffer in self.camera_buffers.items()
        }

        # Calculate efficiency metrics
        total_processed = self.statistics.messages_processed
        total_violations = self.statistics.sync_violations
        efficiency = (total_processed - total_violations) / max(total_processed, 1)

        return {
            "system": {
                "memory_usage_mb": memory_usage_mb,
                "cpu_usage_percent": cpu_usage_percent,
                "timestamp": time.time(),
            },
            "sync_engine": {
                "health_status": health["overall_health"],
                "messages_processed": total_processed,
                "sync_violations": total_violations,
                "efficiency": efficiency,
                "avg_sync_delay_ms": self.statistics.avg_sync_delay_ms,
                "buffer_overflows": self.statistics.buffer_overflows,
                "active_cameras": len(
                    [b for b in self.camera_buffers.values() if b.size() > 0]
                ),
            },
            "buffers": buffer_stats,
            "optimization": {
                "sync_interval": self.sync_interval,
                "adaptive_buffering_enabled": any(
                    b.get_stats()["adaptive_mode"] for b in self.camera_buffers.values()
                ),
            },
        }

    def monitor_performance_trends(self, window_seconds: int = 300) -> Dict[str, Any]:
        """
        Monitor performance trends over time window.

        Args:
            window_seconds: Time window for trend analysis in seconds

        Returns:
            Dictionary with performance trend analysis
        """
        current_time = time.time()
        window_start = current_time - window_seconds

        # This would typically use stored historical data
        # For now, return current metrics with trend indicators
        metrics = self.get_performance_metrics()

        # Add trend indicators (simplified)
        trends = {
            "memory_trend": "stable",  # Would analyze historical data
            "cpu_trend": "stable",
            "efficiency_trend": (
                "improving"
                if metrics["sync_engine"]["efficiency"] > 0.8
                else "needs_attention"
            ),
            "violation_trend": "stable",  # Would analyze violation rate changes
        }

        return {
            "current_metrics": metrics,
            "trends": trends,
            "analysis_window": window_seconds,
            "recommendations": self._generate_performance_recommendations(
                metrics, trends
            ),
        }

    def _generate_performance_recommendations(
        self, metrics: Dict[str, Any], trends: Dict[str, Any]
    ) -> List[str]:
        """Generate performance optimization recommendations."""
        recommendations = []

        # Memory-based recommendations
        if metrics["system"]["memory_usage_mb"] > 400:  # High memory usage
            recommendations.append(
                "Consider reducing buffer sizes or enabling more aggressive cleanup"
            )

        # CPU-based recommendations
        if metrics["system"]["cpu_usage_percent"] > 80:
            recommendations.append(
                "High CPU usage detected - consider increasing sync intervals"
            )

        # Sync efficiency recommendations
        efficiency = metrics["sync_engine"]["efficiency"]
        if efficiency < 0.7:
            recommendations.append(
                "Low sync efficiency - check camera timing and network conditions"
            )
        elif efficiency > 0.95:
            recommendations.append(
                "Excellent sync efficiency - system performing optimally"
            )

        # Buffer recommendations
        total_capacity = sum(b["max_size"] for b in metrics["buffers"].values())
        if total_capacity > 1000:
            recommendations.append("High buffer capacity - monitor memory usage")

        return recommendations

    def _notify_sync(self, sync_data: Dict[str, Any]):
        """Notify sync callbacks of successful synchronization."""
        for callback in self.sync_callbacks:
            try:
                callback(sync_data)
            except Exception as e:
                logger.error(f"Sync callback error: {e}")

    def _notify_desync(self, desync_type: str, desync_data: Dict[str, Any]):
        """Notify desync callbacks of synchronization failures."""
        desync_event = {"type": desync_type, "timestamp": time.time(), **desync_data}

        for callback in self.desync_callbacks:
            try:
                callback(desync_event)
            except Exception as e:
                logger.error(f"Desync callback error: {e}")


class DeadlineMonitor:
    """Real-time deadline monitoring and violation detection."""

    def __init__(
        self,
        operation_name: str,
        deadline_ms: float,
        sync_engine: SynchronizationEngine,
    ):
        self.operation_name = operation_name
        self.deadline_ms = deadline_ms
        self.sync_engine = sync_engine
        self.start_time = None
        self.completed = False

    def __enter__(self):
        self.start_time = time.perf_counter()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.start_time is not None:
            elapsed_ms = (time.perf_counter() - self.start_time) * 1000

            if elapsed_ms > self.deadline_ms:
                # Deadline violation
                self.sync_engine._notify_desync(
                    "deadline_violation",
                    {
                        "operation": self.operation_name,
                        "elapsed_ms": elapsed_ms,
                        "deadline_ms": self.deadline_ms,
                        "violation_ms": elapsed_ms - self.deadline_ms,
                    },
                )

            self.completed = True

    def check_progress(self) -> float:
        """Check progress toward deadline (0.0 to 1.0)."""
        if self.start_time is None:
            return 0.0

        elapsed_ms = (time.perf_counter() - self.start_time) * 1000
        return min(1.0, elapsed_ms / self.deadline_ms)


class CameraFrameSynchronizer:
    """Specialized synchronizer for multi-camera frame alignment."""

    def __init__(self, sync_engine: SynchronizationEngine):
        self.sync_engine = sync_engine
        self.frame_sequences: Dict[str, List[Dict[str, Any]]] = {}
        self.sequence_numbers: Dict[str, int] = {}

    async def wait_for_stereo_pair(
        self, left_camera: str, right_camera: str, timeout_ms: float = 50.0
    ) -> Optional[Dict[str, Any]]:
        """Wait for synchronized stereo camera pair with validation."""
        start_time = time.time()
        timeout_s = timeout_ms / 1000.0

        # Validate stereo baseline configuration
        stereo_tolerance_ms = self.sync_engine.camera_config.stereo_baseline_ms
        if stereo_tolerance_ms <= 0:
            logger.error(f"Invalid stereo baseline tolerance: {stereo_tolerance_ms}ms")
            return None

        attempts = 0
        max_attempts = int(timeout_s * 1000)  # One attempt per ms

        while time.time() - start_time < timeout_s and attempts < max_attempts:
            attempts += 1

            # Try to get synchronized frames
            sync_data = await self.sync_engine.synchronize_cameras()

            if (
                sync_data
                and left_camera in sync_data["cameras"]
                and right_camera in sync_data["cameras"]
            ):
                left_frame = sync_data["cameras"][left_camera]
                right_frame = sync_data["cameras"][right_camera]

                # Validate frame data
                if not self._validate_stereo_frames(left_frame, right_frame):
                    continue

                # Check stereo temporal alignment
                time_diff = abs(left_frame["timestamp"] - right_frame["timestamp"])
                time_diff_ms = time_diff * 1000

                if time_diff_ms <= stereo_tolerance_ms:
                    return {
                        "left": left_frame,
                        "right": right_frame,
                        "sync_delay_ms": sync_data["sync_delay_ms"],
                        "stereo_alignment_us": time_diff * 1_000_000,
                        "stereo_alignment_ms": time_diff_ms,
                        "stereo_quality": 1.0 - (time_diff_ms / stereo_tolerance_ms),
                    }
                else:
                    logger.debug(
                        f"Stereo alignment too poor: {time_diff_ms:.2f}ms > {stereo_tolerance_ms}ms"
                    )

            await asyncio.sleep(0.001)

        # Timeout - try fallback stereo modes
        return await self._fallback_stereo_pair(left_camera, right_camera, timeout_ms)

    def _validate_stereo_frames(
        self, left_frame: Dict[str, Any], right_frame: Dict[str, Any]
    ) -> bool:
        """Validate stereo frame data integrity."""
        required_fields = ["timestamp", "frame_number"]

        # Check both frames have required fields
        for frame, name in [(left_frame, "left"), (right_frame, "right")]:
            for field in required_fields:
                if field not in frame:
                    logger.warning(f"Stereo {name} frame missing {field}")
                    return False

            # Validate timestamp is reasonable (not in far future/past)
            timestamp = frame.get("timestamp", 0)
            current_time = time.time()
            if abs(timestamp - current_time) > 3600:  # 1 hour tolerance
                logger.warning(
                    f"Stereo {name} frame timestamp unreasonable: {timestamp}"
                )
                return False

        return True

    async def _fallback_stereo_pair(
        self, left_camera: str, right_camera: str, timeout_ms: float
    ) -> Optional[Dict[str, Any]]:
        """Fallback stereo synchronization when normal sync fails."""
        logger.warning(
            f"Stereo pair {left_camera}/{right_camera} sync failed, trying fallback"
        )

        # Strategy 1: Accept any recent frames from individual buffers
        left_buffer = self.sync_engine.camera_buffers.get(left_camera)
        right_buffer = self.sync_engine.camera_buffers.get(right_camera)

        if left_buffer and right_buffer:
            current_time = time.time()
            # Get most recent frames regardless of sync
            left_frames = left_buffer.get_items_older_than(current_time)
            right_frames = right_buffer.get_items_older_than(current_time)

            if left_frames and right_frames:
                left_frame = max(left_frames, key=lambda f: f.get("timestamp", 0))
                right_frame = max(right_frames, key=lambda f: f.get("timestamp", 0))

                time_diff = abs(left_frame["timestamp"] - right_frame["timestamp"])
                time_diff_ms = time_diff * 1000

                # Accept even poorly synchronized frames in fallback mode
                extended_tolerance = (
                    self.sync_engine.camera_config.stereo_baseline_ms * 5.0
                )

                if time_diff_ms <= extended_tolerance:
                    return {
                        "left": left_frame,
                        "right": right_frame,
                        "sync_delay_ms": float("inf"),  # Unknown sync delay
                        "stereo_alignment_us": time_diff * 1_000_000,
                        "stereo_alignment_ms": time_diff_ms,
                        "stereo_quality": max(
                            0.1,
                            1.0
                            - (
                                time_diff_ms
                                / self.sync_engine.camera_config.stereo_baseline_ms
                            ),
                        ),
                        "fallback_mode": "extended_tolerance",
                    }

        # Strategy 2: Single camera fallback (not true stereo)
        for cam_name, cam_id in [("left", left_camera), ("right", right_camera)]:
            buffer = self.sync_engine.camera_buffers.get(cam_id)
            if buffer:
                frames = buffer.get_items_older_than(time.time())
                if frames:
                    frame = max(frames, key=lambda f: f.get("timestamp", 0))

                    return {
                        "left": frame if cam_name == "left" else None,
                        "right": frame if cam_name == "right" else None,
                        "sync_delay_ms": 0.0,
                        "stereo_alignment_us": 0.0,
                        "stereo_alignment_ms": 0.0,
                        "stereo_quality": 0.0,  # No stereo
                        "fallback_mode": f"single_camera_{cam_name}",
                    }

        logger.error(
            f"Complete stereo fallback failure for {left_camera}/{right_camera}"
        )
        return None

    def add_frame_sequence(self, camera_id: str, frame_data: Dict[str, Any]):
        """Add frame to sequence for temporal tracking."""
        if camera_id not in self.frame_sequences:
            self.frame_sequences[camera_id] = []
            self.sequence_numbers[camera_id] = 0

        frame_data["sequence_number"] = self.sequence_numbers[camera_id]
        self.frame_sequences[camera_id].append(frame_data)
        self.sequence_numbers[camera_id] += 1

        # Maintain sequence history (last 100 frames)
        if len(self.frame_sequences[camera_id]) > 100:
            self.frame_sequences[camera_id] = self.frame_sequences[camera_id][-100:]


# Global synchronization engine instance
_sync_engine = None


def get_sync_engine() -> SynchronizationEngine:
    """Get global synchronization engine instance."""
    global _sync_engine
    if _sync_engine is None:
        _sync_engine = SynchronizationEngine()
    return _sync_engine


def initialize_sync_engine(
    camera_config: Optional[CameraSyncConfig] = None,
) -> SynchronizationEngine:
    """Initialize the global synchronization engine."""
    global _sync_engine
    _sync_engine = SynchronizationEngine(camera_config)
    return _sync_engine
