#!/usr/bin/env python3
"""
Sample Collection Mission - Complete URC Sample Workflow

Autonomous sample excavation, collection, and caching system integrated with
science payload STM32 controllers. Provides complete end-to-end sample handling.

URC Requirements:
- Autonomous sample detection and approach
- Excavation and collection from subsurface
- Sample caching and preservation
- Science payload integration
"""

import json
import math
import time
from enum import Enum
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseStamped, Quaternion
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import Bool, Float32, Float32MultiArray, String
from tf2_ros import Buffer, TransformListener


class SampleCollectionState(Enum):
    """States for sample collection mission."""

    IDLE = "idle"
    SEARCHING = "searching"
    APPROACHING = "approaching"
    ANALYZING = "analyzing"
    EXCAVATING = "excavating"
    COLLECTING = "collecting"
    CACHING = "caching"
    COMPLETED = "completed"
    FAILED = "failed"


class SampleCollectionMission(Node):
    """
    Sample Collection Mission

    Complete autonomous sample collection system that integrates:
    - Computer vision sample detection
    - Autonomous navigation to sample sites
    - Excavation control via STM32 science payload
    - Sample caching and preservation
    """

    def __init__(self):
        super().__init__("sample_collection_mission")

        # Mission parameters
        self.declare_parameter("max_samples", 5)  # Maximum samples to collect
        self.declare_parameter("sample_search_timeout", 300.0)  # 5 minutes search time
        self.declare_parameter("approach_timeout", 60.0)  # 1 minute approach time
        self.declare_parameter("excavation_timeout", 120.0)  # 2 minutes excavation time
        self.declare_parameter("sample_approach_distance", 0.2)  # 20cm from sample
        self.declare_parameter("excavation_depth", 0.15)  # 15cm excavation depth
        self.declare_parameter("sample_cache_slots", 6)  # Available cache slots

        # Get parameters
        self.max_samples = self.get_parameter("max_samples").value
        self.search_timeout = self.get_parameter("sample_search_timeout").value
        self.approach_timeout = self.get_parameter("approach_timeout").value
        self.excavation_timeout = self.get_parameter("excavation_timeout").value
        self.approach_distance = self.get_parameter("sample_approach_distance").value
        self.excavation_depth = self.get_parameter("excavation_depth").value
        self.cache_slots = self.get_parameter("sample_cache_slots").value

        # Mission state
        self.state = SampleCollectionState.IDLE
        self.start_time = None
        self.samples_collected = 0
        self.cache_used = 0
        self.current_sample_location = None
        self.potential_samples = []  # List of detected sample locations

        # TF2 for coordinate transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # CV bridge for image processing
        self.bridge = CvBridge()

        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, "/goal_pose", 10)

        self.excavate_cmd_pub = self.create_publisher(
            String, "/hardware/excavate_command", 10
        )

        self.mission_status_pub = self.create_publisher(
            String, "/mission/sample_collection_status", 10
        )

        self.sample_data_pub = self.create_publisher(String, "/science/sample_data", 10)

        # Subscribers (optimized - use centralized vision processing)
        self.obstacle_sub = self.create_subscription(
            Float32MultiArray,
            "/vision/obstacles",
            self.obstacle_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=3),
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=3),
        )

        self.excavate_status_sub = self.create_subscription(
            String, "/hardware/excavate_status", self.excavate_status_callback, 10
        )

        self.mission_cmd_sub = self.create_subscription(
            String,
            "/mission/sample_collection_command",
            self.mission_command_callback,
            10,
        )

        # Mission control integration
        self.control_sub = self.create_subscription(
            String, "/mission/commands", self.control_callback, 10
        )

        # State tracking
        self.current_pose = None
        self.image_count = 0
        self.excavation_active = False
        self.sample_in_gripper = False

        # Timers for different phases
        self.search_timer = None
        self.approach_timer = None
        self.excavation_timer = None

        self.get_logger().info("Sample Collection Mission initialized")
        self.get_logger().info(
            f"Target: {self.max_samples} samples, {self.cache_slots} cache slots available"
        )

    def mission_command_callback(self, msg: String):
        """Handle sample collection mission specific commands."""
        try:
            command = msg.data.lower()
            if command == "start":
                self.start_mission()
            elif command == "stop":
                self.stop_mission()
            elif command == "reset":
                self.reset_mission()
            elif command.startswith("collect_sample_at"):
                # Parse coordinates: "collect_sample_at:lat,lng"
                coords = command.split(":")[1].split(",")
                lat, lng = float(coords[0]), float(coords[1])
                self.collect_sample_at_location(lat, lng)
        except Exception as e:
            self.get_logger().error(f"Mission command error: {e}")

    def control_callback(self, msg: String):
        """Handle general mission control commands."""
        try:
            command = msg.data.lower()
            if command == "start_sample_collection":
                self.start_mission()
            elif command == "stop_sample_collection":
                self.stop_mission()
        except Exception as e:
            self.get_logger().error(f"Control command error: {e}")

    def start_mission(self):
        """Start the sample collection mission."""
        if self.state != SampleCollectionState.IDLE:
            self.get_logger().warn(
                f"Cannot start mission from state {self.state.value}"
            )
            return

        self.get_logger().info("🚀 Starting Sample Collection Mission")
        self.state = SampleCollectionState.SEARCHING
        self.start_time = time.time()
        self.samples_collected = 0
        self.cache_used = 0
        self.potential_samples = []

        # Start search timeout
        self.search_timer = self.create_timer(
            self.search_timeout, self.search_timeout_callback
        )

        self.publish_status()

    def stop_mission(self):
        """Stop the current mission."""
        self.get_logger().info("🛑 Stopping Sample Collection Mission")
        self.state = SampleCollectionState.IDLE
        self.cleanup_timers()

        # Stop any active excavation
        if self.excavation_active:
            self.stop_excavation()

        self.publish_status()

    def reset_mission(self):
        """Reset mission to initial state."""
        self.get_logger().info("🔄 Resetting Sample Collection Mission")
        self.stop_mission()
        self.start_mission()

    def collect_sample_at_location(self, latitude: float, longitude: float):
        """Manually specify a sample collection location."""
        self.get_logger().info(f"Manual sample collection at {latitude}, {longitude}")

        # Convert lat/lng to map coordinates (simplified)
        sample_pose = PoseStamped()
        sample_pose.header.frame_id = "map"
        sample_pose.header.stamp = self.get_clock().now().to_msg()
        sample_pose.pose.position.x = longitude * 1000  # Rough conversion
        sample_pose.pose.position.y = latitude * 1000
        sample_pose.pose.position.z = 0.0
        sample_pose.pose.orientation.w = 1.0

        self.potential_samples.append(sample_pose)
        self.current_sample_location = sample_pose

        if self.state == SampleCollectionState.SEARCHING:
            self.state = SampleCollectionState.APPROACHING
            self.approach_sample()

    def search_timeout_callback(self):
        """Handle sample search timeout."""
        if self.state == SampleCollectionState.SEARCHING:
            if len(self.potential_samples) > 0:
                self.get_logger().info(
                    f"Search timeout - proceeding with {len(self.potential_samples)} detected samples"
                )
                self.state = SampleCollectionState.APPROACHING
                self.approach_sample()
            else:
                self.get_logger().error("⌛ Sample search timeout - no samples detected")
                self.state = SampleCollectionState.FAILED
                self.publish_status()

    def obstacle_callback(self, msg: Float32MultiArray):
        """Receive obstacle detection from centralized vision processing."""
        if self.state != SampleCollectionState.SEARCHING:
            return

        try:
            # Use obstacle detections as potential sample locations
            # Format: [center_x, center_y, width, height, area, ...]
            obstacles = msg.data

            if len(obstacles) >= 5:  # At least one obstacle detected
                num_obstacles = len(obstacles) // 5
                self.get_logger().info(
                    f"🔍 Detected {num_obstacles} potential sample locations via vision processor"
                )

                # Convert obstacle coordinates to world poses
                for i in range(0, len(obstacles), 5):
                    if i + 4 < len(obstacles):
                        center_x = obstacles[i]
                        center_y = obstacles[i + 1]
                        area = obstacles[i + 4]

                        # Only consider large obstacles as potential samples
                        if area > 0.1:  # Threshold for sample size
                            # Create sample pose (simplified - would need proper transform)
                            sample_pose = PoseStamped()
                            sample_pose.header.frame_id = "camera_link"
                            sample_pose.header.stamp = self.get_clock().now().to_msg()
                            sample_pose.pose.position.x = (
                                center_x - 0.5
                            ) * 2.0  # Convert normalized to meters
                            sample_pose.pose.position.y = (center_y - 0.5) * 2.0
                            sample_pose.pose.position.z = 0.0
                            sample_pose.pose.orientation.w = 1.0

                            # Transform to map frame
                            try:
                                world_pose = self.tf_buffer.transform(
                                    sample_pose,
                                    "map",
                                    timeout=rclpy.duration.Duration(seconds=1.0),
                                )
                                self.potential_samples.append(world_pose)
                            except:
                                pass

                # If we have enough samples, start collection
                if len(self.potential_samples) >= 2 or self.should_start_collection():
                    self.state = SampleCollectionState.APPROACHING
                    self.cleanup_timers()
                    self.approach_sample()

        except Exception as e:
            self.get_logger().error(f"Obstacle processing error: {e}")

    def detect_samples(self, image: np.ndarray) -> List[Tuple[int, int]]:
        """Detect potential sample locations using computer vision."""
        # Convert to HSV for color analysis
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Look for soil/rock colors that might indicate sample sites
        # URC samples are typically reddish/brownish rocks or soil
        sample_mask = cv2.inRange(hsv, (5, 50, 50), (15, 255, 200))  # Reddish colors

        # Find contours of potential sample areas
        contours, _ = cv2.findContours(
            sample_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )

        sample_locations = []
        for contour in contours:
            area = cv2.contourArea(contour)
            if area > 1000:  # Minimum area threshold
                # Get center of contour
                M = cv2.moments(contour)
                if M["m00"] != 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    sample_locations.append((cx, cy))

        return sample_locations

    def image_to_world_coordinates(
        self, image_coords: Tuple[int, int], header
    ) -> Optional[PoseStamped]:
        """Convert image pixel coordinates to world coordinates."""
        try:
            # This is a simplified conversion
            # Real implementation would use camera intrinsics and extrinsics

            pixel_x, pixel_y = image_coords

            # Get camera intrinsics (would come from CameraInfo topic)
            # For now, use reasonable defaults
            focal_length = 500  # pixels
            image_center_x = 320
            image_center_y = 240

            # Convert to camera coordinates (simplified pinhole model)
            camera_x = (pixel_x - image_center_x) / focal_length
            camera_y = (pixel_y - image_center_y) / focal_length
            camera_z = 1.0  # Assume 1 meter in front

            # Create pose stamped in camera frame
            sample_pose = PoseStamped()
            sample_pose.header = header
            sample_pose.pose.position.x = camera_x
            sample_pose.pose.position.y = camera_y
            sample_pose.pose.position.z = camera_z
            sample_pose.pose.orientation.w = 1.0

            # Transform to map frame
            try:
                transformed_pose = self.tf_buffer.transform(
                    sample_pose, "map", timeout=rclpy.duration.Duration(seconds=1.0)
                )
                return transformed_pose
            except Exception as tf_error:
                self.get_logger().warn(f"TF sample transform failed: {tf_error}")
                return None

        except Exception as e:
            self.get_logger().error(f"Coordinate conversion error: {e}")
            return None

    def should_start_collection(self) -> bool:
        """Determine if we should start sample collection."""
        # Start if we have samples and search time is getting long
        if self.start_time:
            elapsed = time.time() - self.start_time
            return len(self.potential_samples) > 0 and elapsed > 60.0  # 1 minute
        return False

    def approach_sample(self):
        """Navigate to the next sample location."""
        if not self.potential_samples:
            self.get_logger().error("No samples available to approach")
            self.state = SampleCollectionState.FAILED
            return

        # Get next sample (for now, just take the first one)
        self.current_sample_location = self.potential_samples.pop(0)

        self.get_logger().info("🧭 Approaching sample location...")

        # Create approach pose (close but not too close for excavation)
        approach_pose = PoseStamped()
        approach_pose.header = self.current_sample_location.header
        approach_pose.header.frame_id = "map"

        # Calculate approach position
        dx = self.current_sample_location.pose.position.x - (
            self.current_pose.pose.pose.position.x if self.current_pose else 0
        )
        dy = self.current_sample_location.pose.position.y - (
            self.current_pose.pose.pose.position.y if self.current_pose else 0
        )
        distance = math.sqrt(dx * dx + dy * dy)

        if distance > 0:
            # Position approach_distance meters from sample
            scale = (distance - self.approach_distance) / distance
            approach_pose.pose.position.x = (
                self.current_pose.pose.pose.position.x + dx * scale
            )
            approach_pose.pose.position.y = (
                self.current_pose.pose.pose.position.y + dy * scale
            )
            approach_pose.pose.position.z = 0.0

            # Face the sample
            approach_pose.pose.orientation = self.calculate_facing_orientation(
                approach_pose.pose.position, self.current_sample_location.pose.position
            )
        else:
            # Already close
            approach_pose.pose = self.current_sample_location.pose

        # Publish navigation goal
        self.goal_pub.publish(approach_pose)

        # Start approach timeout
        self.approach_timer = self.create_timer(
            self.approach_timeout, self.approach_timeout_callback
        )

    def calculate_facing_orientation(
        self, from_pos: Point, to_pos: Point
    ) -> Quaternion:
        """Calculate quaternion to face from one position to another."""
        dx = to_pos.x - from_pos.x
        dy = to_pos.y - from_pos.y
        yaw = math.atan2(dy, dx)

        quaternion = Quaternion()
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)

        return quaternion

    def approach_timeout_callback(self):
        """Handle approach timeout."""
        if self.state == SampleCollectionState.APPROACHING:
            self.get_logger().error("⌛ Sample approach timeout")
            self.state = SampleCollectionState.FAILED
            self.publish_status()

    def odom_callback(self, msg: Odometry):
        """Track current robot position."""
        self.current_pose = msg

        # Check if we've reached the sample
        if (
            self.state == SampleCollectionState.APPROACHING
            and self.current_sample_location
            and self.is_at_sample_location()
        ):

            self.get_logger().info("📍 Reached sample location")
            self.state = SampleCollectionState.ANALYZING
            self.cleanup_timers()

            # Start sample analysis
            self.analyze_sample()

    def is_at_sample_location(self) -> bool:
        """Check if robot is close enough to sample for excavation."""
        if not self.current_sample_location or not self.current_pose:
            return False

        dx = (
            self.current_sample_location.pose.position.x
            - self.current_pose.pose.pose.position.x
        )
        dy = (
            self.current_sample_location.pose.position.y
            - self.current_pose.pose.pose.position.y
        )
        distance = math.sqrt(dx * dx + dy * dy)

        return distance < self.approach_distance

    def analyze_sample(self):
        """Analyze sample before excavation."""
        self.get_logger().info("🔬 Analyzing sample...")

        # In a real implementation, this would:
        # - Take close-up images
        # - Use spectroscopy if available
        # - Determine sample type and excavation approach

        # For now, proceed directly to excavation
        self.create_timer(2.0, self.start_excavation)

    def start_excavation(self):
        """Start the excavation process."""
        self.get_logger().info("⛏️ Starting sample excavation...")

        # Send excavation command to science payload STM32
        excavate_cmd = f"EXCAVATE:START:{self.excavation_depth:.3f}"
        self.excavate_cmd_pub.publish(String(data=excavate_cmd))

        self.excavation_active = True
        self.state = SampleCollectionState.EXCAVATING

        # Start excavation timeout
        self.excavation_timer = self.create_timer(
            self.excavation_timeout, self.excavation_timeout_callback
        )

    def excavate_status_callback(self, msg: String):
        """Handle excavation status updates from hardware."""
        try:
            status = json.loads(msg.data)

            if status.get("complete", False):
                self.get_logger().info("✅ Excavation completed")
                self.excavation_active = False
                self.sample_in_gripper = True
                self.state = SampleCollectionState.COLLECTING
                self.cleanup_timers()

                # Proceed to caching
                self.cache_sample()

            elif status.get("error"):
                self.get_logger().error(f"Excavation error: {status['error']}")
                self.excavation_active = False
                self.state = SampleCollectionState.FAILED

        except Exception as e:
            self.get_logger().error(f"Excavation status parsing error: {e}")

    def excavation_timeout_callback(self):
        """Handle excavation timeout."""
        if self.state == SampleCollectionState.EXCAVATING:
            self.get_logger().error("⌛ Excavation timeout")
            self.stop_excavation()
            self.state = SampleCollectionState.FAILED
            self.publish_status()

    def stop_excavation(self):
        """Stop active excavation."""
        excavate_cmd = "EXCAVATE:STOP"
        self.excavate_cmd_pub.publish(String(data=excavate_cmd))
        self.excavation_active = False

    def cache_sample(self):
        """Cache the collected sample."""
        self.get_logger().info("📦 Caching sample...")

        # Check if we have cache space
        if self.cache_used >= self.cache_slots:
            self.get_logger().error("No cache slots available")
            self.state = SampleCollectionState.FAILED
            return

        # Send cache command
        cache_cmd = f"CACHE:STORE:{self.cache_used}"
        self.excavate_cmd_pub.publish(String(data=cache_cmd))

        # Wait for caching completion (simplified)
        self.create_timer(3.0, self.caching_complete)

    def caching_complete(self):
        """Handle sample caching completion."""
        self.get_logger().info("✅ Sample cached successfully")
        self.samples_collected += 1
        self.cache_used += 1
        self.sample_in_gripper = False

        # Publish sample data for science analysis
        sample_data = {
            "sample_id": self.samples_collected,
            "collection_time": time.time(),
            "location": {
                "x": self.current_sample_location.pose.position.x,
                "y": self.current_sample_location.pose.position.y,
                "z": self.current_sample_location.pose.position.z,
            },
            "type": "unknown",  # Would be determined by analysis
            "cache_slot": self.cache_used - 1,
        }

        sample_msg = String()
        sample_msg.data = json.dumps(sample_data)
        self.sample_data_pub.publish(sample_msg)

        # Check if mission complete
        if self.samples_collected >= self.max_samples:
            self.get_logger().info(
                f"🎉 Mission complete! Collected {self.samples_collected} samples"
            )
            self.state = SampleCollectionState.COMPLETED

            # Publish mission completion
            completion_msg = String()
            completion_msg.data = json.dumps(
                {
                    "mission": "sample_collection",
                    "status": "completed",
                    "samples_collected": self.samples_collected,
                    "cache_used": self.cache_used,
                    "timestamp": time.time(),
                }
            )
            self.mission_status_pub.publish(completion_msg)
        else:
            # Continue with next sample
            self.get_logger().info(
                f"Continuing mission - {self.samples_collected}/{self.max_samples} samples collected"
            )
            if self.potential_samples:
                self.state = SampleCollectionState.APPROACHING
                self.approach_sample()
            else:
                self.state = SampleCollectionState.SEARCHING
                self.search_timer = self.create_timer(
                    self.search_timeout, self.search_timeout_callback
                )

        self.publish_status()

    def cleanup_timers(self):
        """Clean up all active timers."""
        timers = [self.search_timer, self.approach_timer, self.excavation_timer]

        for timer in timers:
            if timer:
                timer.cancel()

        self.search_timer = None
        self.approach_timer = None
        self.excavation_timer = None

    def publish_status(self):
        """Publish current mission status."""
        status_data = {
            "mission": "sample_collection",
            "state": self.state.value,
            "samples_collected": self.samples_collected,
            "cache_used": self.cache_used,
            "cache_slots": self.cache_slots,
            "potential_samples": len(self.potential_samples),
            "excavation_active": self.excavation_active,
            "sample_in_gripper": self.sample_in_gripper,
            "timestamp": time.time(),
        }

        if self.start_time:
            status_data["elapsed_time"] = time.time() - self.start_time

        status_msg = String()
        status_msg.data = json.dumps(status_data)
        self.mission_status_pub.publish(status_msg)

        self.get_logger().info(
            f"Mission status: {self.state.value} ({self.samples_collected}/{self.max_samples} samples)"
        )


def main(args=None):
    rclpy.init(args=args)

    mission = SampleCollectionMission()

    try:
        rclpy.spin(mission)
    except KeyboardInterrupt:
        mission.get_logger().info("Keyboard interrupt received")
    finally:
        mission.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
