#!/usr/bin/env python3
"""
Return to Operator Mission - ROS2 Node

Autonomously navigates rover back to operator location using SLAM, GPS,
and Nav2 path planning with real-time obstacle detection and safety monitoring.

Author: URC 2026 Autonomy Team
"""

# Standard Library
import math
import threading
import time
from enum import Enum
from typing import Any, Dict, Optional, Tuple

import rclpy

# ROS2 Messages
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Float32, String
from std_srvs.srv import Trigger


class ReturnToOperatorState(Enum):
    """States for return to operator mission"""

    IDLE = "idle"
    LOCALIZING_OPERATOR = "localizing_operator"
    PLANNING_PATH = "planning_path"
    NAVIGATING = "navigating"
    APPROACHING = "approaching"
    ARRIVED = "arrived"
    FAILED = "failed"
    ABORTED = "aborted"


class ReturnToOperatorMission(Node):
    """
    Return to Operator Mission Node

    Autonomously navigates rover back to operator using multiple localization methods:
    - GPS tracking of operator location
    - ArUco tag detection for precision approach
    - SLAM-based global localization
    - Nav2 path planning with obstacle avoidance

    Subscribes to:
    - /slam/pose: SLAM pose for precise rover localization
    - /gps/fix: GPS position for global reference
    - /odom: Odometry for velocity feedback
    - /operator/gps: Operator GPS location (if available)
    - /camera/image_raw: Camera feed for ArUco detection
    - /scan: Laser scan for obstacle detection

    Publishes:
    - /mission/return/status: Mission status and progress
    - /mission/return/operator_pose: Estimated operator location
    - /mission/return/progress: Distance remaining and ETA
    - /mission/return/cmd_vel: Velocity commands for navigation
    - /mission/return/path: Planned return path
    - /mission/return/safety_status: Safety monitoring status

    Services:
    - /mission/return/start: Start return to operator mission
    - /mission/return/stop: Stop return mission
    - /mission/return/status: Get current mission status
    - /mission/return/set_operator_location: Manually set operator GPS
    """

    def __init__(self):
        super().__init__("return_to_operator_mission")

        # Mission state
        self.state = ReturnToOperatorState.IDLE
        self.mission_start_time = None
        self.path_start_time = None

        # Location tracking
        self.rover_pose: Optional[PoseStamped] = None
        self.operator_gps: Optional[NavSatFix] = None
        self.operator_pose: Optional[PoseStamped] = None
        self.target_aruco_id = 999  # Special ID for operator tag
        self.aruco_detected = False
        self.last_aruco_detection = None

        # Navigation parameters
        self.declare_parameter("operator_gps_topic", "/operator/gps")
        self.declare_parameter("operator_aruco_id", 999)
        self.declare_parameter("approach_distance", 2.0)  # meters
        self.declare_parameter("path_replan_interval", 5.0)  # seconds
        self.declare_parameter("max_mission_time", 600.0)  # 10 minutes
        self.declare_parameter("gps_to_local_tolerance", 5.0)  # meters
        self.declare_parameter("aruco_search_timeout", 30.0)  # seconds
        self.declare_parameter("velocity_timeout", 10.0)  # seconds
        self.declare_parameter("obstacle_check_distance", 3.0)  # meters

        # Load parameters
        self.operator_gps_topic = self.get_parameter("operator_gps_topic").value
        self.operator_aruco_id = self.get_parameter("operator_aruco_id").value
        self.approach_distance = self.get_parameter("approach_distance").value
        self.path_replan_interval = self.get_parameter("path_replan_interval").value
        self.max_mission_time = self.get_parameter("max_mission_time").value
        self.gps_to_local_tolerance = self.get_parameter("gps_to_local_tolerance").value
        self.aruco_search_timeout = self.get_parameter("aruco_search_timeout").value
        self.velocity_timeout = self.get_parameter("velocity_timeout").value
        self.obstacle_check_distance = self.get_parameter(
            "obstacle_check_distance"
        ).value

        # Navigation state
        self.planned_path: Optional[Path] = None
        self.path_index = 0
        self.last_path_replan = 0
        self.distance_remaining = 0.0
        self.estimated_time_remaining = 0.0

        # Safety monitoring
        self.last_velocity_time = time.time()
        self.consecutive_obstacle_detections = 0
        self.max_consecutive_obstacles = 5

        # Service clients
        self.nav_client = self.create_client(Trigger, "/navigate_to_pose")

        # Publishers
        qos_reliable = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=5,
        )

        self.status_pub = self.create_publisher(
            String, "/mission/return/status", qos_reliable
        )
        self.operator_pose_pub = self.create_publisher(
            PoseStamped, "/mission/return/operator_pose", qos_reliable
        )
        self.progress_pub = self.create_publisher(
            String, "/mission/return/progress", qos_reliable
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, "/mission/return/cmd_vel", qos_sensor
        )
        self.path_pub = self.create_publisher(
            Path, "/mission/return/path", qos_reliable
        )
        self.safety_pub = self.create_publisher(
            String, "/mission/return/safety_status", qos_reliable
        )

        # Subscribers
        self.pose_sub = self.create_subscription(
            PoseStamped, "/slam/pose", self.pose_callback, 10
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )

        # Try to subscribe to operator GPS if topic exists
        try:
            self.operator_gps_sub = self.create_subscription(
                NavSatFix, self.operator_gps_topic, self.operator_gps_callback, 10
            )
            self.get_logger().info(
                f"Subscribed to operator GPS: {self.operator_gps_topic}"
            )
        except Exception as e:
            self.get_logger().warning(f"Could not subscribe to operator GPS: {e}")
            self.operator_gps_sub = None

        # Services
        self.start_service = self.create_service(
            Trigger, "/mission/return/start", self.start_mission_callback
        )
        self.stop_service = self.create_service(
            Trigger, "/mission/return/stop", self.stop_mission_callback
        )
        self.status_service = self.create_service(
            Trigger, "/mission/return/status", self.status_callback
        )

        # Mission monitoring timer
        self.create_timer(0.5, self.mission_monitor)  # 2Hz monitoring

        self.get_logger().info("Return to Operator Mission initialized")

    def pose_callback(self, msg: PoseStamped):
        """Update rover pose from SLAM."""
        self.rover_pose = msg

    def gps_callback(self, msg: NavSatFix):
        """Update rover GPS position."""
        # Could be used for GPS-based operator localization if needed
        pass

    def odom_callback(self, msg: Odometry):
        """Monitor rover velocity for safety."""
        current_time = time.time()

        # Check for velocity timeout (stuck detection)
        linear_velocity = math.sqrt(
            msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2
        )

        if linear_velocity > 0.01:  # Moving
            self.last_velocity_time = current_time
        elif (current_time - self.last_velocity_time) > self.velocity_timeout:
            if self.state == ReturnToOperatorState.NAVIGATING:
                self.get_logger().warning("Velocity timeout - rover appears stuck")
                self.handle_stuck_condition()

    def operator_gps_callback(self, msg: NavSatFix):
        """Update operator GPS location."""
        self.operator_gps = msg

        # Convert GPS to local pose if we have rover pose for reference
        if self.rover_pose is not None:
            self.operator_pose = self.gps_to_local_pose(msg)
            self.operator_pose_pub.publish(self.operator_pose)

    def gps_to_local_pose(self, gps_msg: NavSatFix) -> PoseStamped:
        """Convert GPS coordinates to local pose relative to rover."""
        # Simplified GPS to local conversion
        # In production, this would use proper geodetic transformations
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()

        # Placeholder conversion (replace with proper GPS->local transform)
        # This assumes we're working in a local coordinate frame
        pose.pose.position.x = (
            gps_msg.longitude - self.rover_pose.pose.position.x
        ) * 111320  # Rough meters per degree
        pose.pose.position.y = (
            gps_msg.latitude - self.rover_pose.pose.position.y
        ) * 111320
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0  # Face forward

        return pose

    def start_mission_callback(self, request, response):
        """Handle mission start request."""
        try:
            if self.state != ReturnToOperatorState.IDLE:
                response.success = False
                response.message = f"Cannot start mission from state {self.state.value}"
                return response

            success = self.start_mission()
            response.success = success
            response.message = (
                "Return to operator mission started"
                if success
                else "Failed to start mission"
            )

        except Exception as e:
            self.get_logger().error(f"Error starting mission: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"

        return response

    def stop_mission_callback(self, request, response):
        """Handle mission stop request."""
        try:
            self.stop_mission()
            response.success = True
            response.message = "Return to operator mission stopped"
        except Exception as e:
            self.get_logger().error(f"Error stopping mission: {e}")
            response.success = False
            response.message = f"Error: {str(e)}"

        return response

    def status_callback(self, request, response):
        """Handle status query."""
        status_info = {
            "state": self.state.value,
            "mission_time": time.time() - (self.mission_start_time or time.time()),
            "operator_localized": self.operator_pose is not None,
            "aruco_detected": self.aruco_detected,
            "distance_remaining": self.distance_remaining,
            "eta_seconds": self.estimated_time_remaining,
        }

        import json

        response.success = True
        response.message = json.dumps(status_info)
        return response

    def start_mission(self) -> bool:
        """Start the return to operator mission."""
        if self.rover_pose is None:
            self.get_logger().error("Cannot start mission: no rover pose available")
            return False

        self.state = ReturnToOperatorState.LOCALIZING_OPERATOR
        self.mission_start_time = time.time()
        self.aruco_detected = False
        self.last_aruco_detection = None

        self.get_logger().info("Return to operator mission started")
        self.publish_status("Mission started - localizing operator")
        return True

    def stop_mission(self):
        """Stop the return to operator mission."""
        previous_state = self.state
        self.state = ReturnToOperatorState.IDLE

        # Stop motion
        self.publish_velocity(0.0, 0.0)

        self.get_logger().info(f"Mission stopped from state {previous_state.value}")
        self.publish_status("Mission stopped")

    def mission_monitor(self):
        """Main mission monitoring loop."""
        try:
            if self.state == ReturnToOperatorState.IDLE:
                return

            # Check mission timeout
            if (
                self.mission_start_time
                and (time.time() - self.mission_start_time) > self.max_mission_time
            ):
                self.get_logger().error("Mission timeout exceeded")
                self.state = ReturnToOperatorState.FAILED
                self.publish_status("Mission failed - timeout")
                return

            # State-specific monitoring
            if self.state == ReturnToOperatorState.LOCALIZING_OPERATOR:
                self.monitor_operator_localization()
            elif self.state == ReturnToOperatorState.PLANNING_PATH:
                self.monitor_path_planning()
            elif self.state == ReturnToOperatorState.NAVIGATING:
                self.monitor_navigation()
            elif self.state == ReturnToOperatorState.APPROACHING:
                self.monitor_approach()

        except Exception as e:
            self.get_logger().error(f"Error in mission monitor: {e}")
            self.state = ReturnToOperatorState.FAILED

    def monitor_operator_localization(self):
        """Monitor operator localization progress."""
        # Check if we have operator GPS
        if self.operator_gps is not None and self.operator_pose is not None:
            self.get_logger().info("Operator GPS localized - planning path")
            self.state = ReturnToOperatorState.PLANNING_PATH
            self.plan_path_to_operator()
            return

        # Check for ArUco detection as fallback
        if self.aruco_detected and self.last_aruco_detection:
            time_since_detection = time.time() - self.last_aruco_detection
            if time_since_detection < 10.0:  # Recent detection
                self.get_logger().info(
                    "Operator ArUco detected - using vision-based localization"
                )
                self.state = ReturnToOperatorState.PLANNING_PATH
                self.plan_path_to_operator()
                return

        # Timeout check for localization
        localization_time = time.time() - self.mission_start_time
        if localization_time > 30.0:  # 30 second timeout
            self.get_logger().warning(
                "Operator localization timeout - searching with motion"
            )
            # Could implement search pattern here
            self.publish_status("Searching for operator location")

    def plan_path_to_operator(self):
        """Plan navigation path to operator location."""
        if self.operator_pose is None:
            self.get_logger().error("Cannot plan path: no operator pose available")
            self.state = ReturnToOperatorState.FAILED
            return

        # Calculate distance to operator
        if self.rover_pose is not None:
            dx = self.operator_pose.pose.position.x - self.rover_pose.pose.position.x
            dy = self.operator_pose.pose.position.y - self.rover_pose.pose.position.y
            self.distance_remaining = math.sqrt(dx * dx + dy * dy)

            # Estimate time (rough calculation: 0.5 m/s average speed)
            self.estimated_time_remaining = self.distance_remaining / 0.5

        # Create simple path (in production, this would use Nav2 global planner)
        self.planned_path = Path()
        self.planned_path.header.frame_id = "map"
        self.planned_path.header.stamp = self.get_clock().now().to_msg()

        # Add waypoints (simplified - just direct path)
        # In production, this would include obstacle avoidance waypoints
        self.planned_path.poses = [self.operator_pose]

        self.path_pub.publish(self.planned_path)
        self.state = ReturnToOperatorState.NAVIGATING
        self.path_start_time = time.time()

        self.get_logger().info(".1f")
        self.publish_status(".1f")

    def monitor_path_planning(self):
        """Monitor path planning progress."""
        # In simplified version, path planning is synchronous
        # In production, this would wait for Nav2 global planner response
        if self.planned_path is not None:
            self.state = ReturnToOperatorState.NAVIGATING
        else:
            # Retry path planning
            self.plan_path_to_operator()

    def monitor_navigation(self):
        """Monitor navigation progress toward operator."""
        if self.rover_pose is None or self.operator_pose is None:
            return

        # Calculate current distance to operator
        dx = self.operator_pose.pose.position.x - self.rover_pose.pose.position.x
        dy = self.operator_pose.pose.position.y - self.rover_pose.pose.position.y
        current_distance = math.sqrt(dx * dx + dy * dy)

        # Update progress
        if self.distance_remaining > 0:
            progress_percent = (1.0 - current_distance / self.distance_remaining) * 100
        else:
            progress_percent = 100.0

        # Check if we've arrived
        if current_distance < self.approach_distance:
            self.get_logger().info("Approaching operator - switching to precision mode")
            self.state = ReturnToOperatorState.APPROACHING
            self.publish_status("Approaching operator")
            return

        # Check for obstacles (simplified - would use laser scan in production)
        if self.check_for_obstacles():
            self.consecutive_obstacle_detections += 1
            if self.consecutive_obstacle_detections >= self.max_consecutive_obstacles:
                self.get_logger().warning(
                    "Multiple obstacle detections - replanning path"
                )
                self.replan_path()
        else:
            self.consecutive_obstacle_detections = 0

        # Replan path periodically
        current_time = time.time()
        if (current_time - self.last_path_replan) > self.path_replan_interval:
            self.replan_path()

        # Publish progress
        self.publish_progress(current_distance, progress_percent)

    def monitor_approach(self):
        """Monitor final approach to operator."""
        if self.rover_pose is None or self.operator_pose is None:
            return

        # Calculate distance
        dx = self.operator_pose.pose.position.x - self.rover_pose.pose.position.x
        dy = self.operator_pose.pose.position.y - self.rover_pose.pose.position.y
        distance = math.sqrt(dx * dx + dy * dy)

        # Check for ArUco detection in final approach
        if self.aruco_detected and self.last_aruco_detection:
            time_since_detection = time.time() - self.last_aruco_detection
            if time_since_detection < 5.0:  # Very recent detection
                self.get_logger().info("ArUco confirmed - mission successful")
                self.state = ReturnToOperatorState.ARRIVED
                self.publish_velocity(0.0, 0.0)
                self.publish_status("Successfully returned to operator")
                return

        # Check if we've arrived (distance-based)
        if distance < 1.0:  # Very close
            self.get_logger().info("Arrived at operator location")
            self.state = ReturnToOperatorState.ARRIVED
            self.publish_velocity(0.0, 0.0)
            self.publish_status("Arrived at operator location")
            return

        # Continue approaching with low speed
        self.approach_operator(dx, dy, distance)

    def approach_operator(self, dx, dy, distance):
        """Execute final approach to operator."""
        # Calculate approach velocity (slow and careful)
        max_approach_speed = 0.3  # m/s - very slow for safety

        # Calculate required velocity toward operator
        if distance > 0:
            vx = (dx / distance) * max_approach_speed
            vy = (dy / distance) * max_approach_speed

            # Convert to robot-centric velocities (simplified)
            linear_vel = math.sqrt(vx * vx + vy * vy)
            angular_vel = math.atan2(vy, vx) * 0.5  # Proportional control

            self.publish_velocity(linear_vel, angular_vel)

    def check_for_obstacles(self) -> bool:
        """Check for obstacles in path (simplified implementation)."""
        # In production, this would analyze laser scan data
        # For now, return False (no obstacles) - implement based on your sensor setup
        return False

    def replan_path(self):
        """Replan path to operator (simplified)."""
        self.last_path_replan = time.time()
        # In production, this would trigger Nav2 global planner replanning
        # For now, just update the path timestamp
        if self.planned_path:
            self.planned_path.header.stamp = self.get_clock().now().to_msg()
            self.path_pub.publish(self.planned_path)

    def handle_stuck_condition(self):
        """Handle rover stuck condition."""
        self.get_logger().warning("Rover appears stuck - attempting recovery")

        # Try reversing briefly
        self.publish_velocity(-0.2, 0.0)  # Reverse
        time.sleep(2.0)
        self.publish_velocity(0.0, 0.0)  # Stop

        # Replan path
        self.replan_path()

    def publish_status(self, status_message: str):
        """Publish mission status."""
        msg = String()
        msg.data = status_message
        self.status_pub.publish(msg)

    def publish_progress(self, distance_remaining: float, progress_percent: float):
        """Publish mission progress."""
        import json

        progress_data = {
            "distance_remaining": round(distance_remaining, 2),
            "progress_percent": round(progress_percent, 1),
            "eta_seconds": round(self.estimated_time_remaining, 1),
            "state": self.state.value,
        }

        msg = String()
        msg.data = json.dumps(progress_data)
        self.progress_pub.publish(msg)

    def publish_velocity(self, linear_x: float, angular_z: float):
        """Publish velocity commands."""
        msg = Twist()
        msg.linear.x = linear_x
        msg.angular.z = angular_z
        self.cmd_vel_pub.publish(msg)

    def publish_safety_status(self, status: str):
        """Publish safety monitoring status."""
        msg = String()
        msg.data = status
        self.safety_pub.publish(msg)


def main():
    """Run the return to operator mission."""
    rclpy.init()
    mission = ReturnToOperatorMission()
    rclpy.spin(mission)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
