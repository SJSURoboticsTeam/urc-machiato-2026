#!/usr/bin/env python3
"""
Competition Geofencing - GPS-based Boundary Enforcement

Critical URC safety system that ensures rover stays within competition boundaries
and enforces geofenced areas to prevent damage to equipment or environment.

Features:
- GPS-based boundary checking with configurable polygons
- Real-time boundary violation detection and alerts
- Emergency stop activation for boundary violations
- Competition area definitions and restrictions
"""

import json
import math
import os
import sys
from typing import List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Point, Point32, PolygonStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker

# Import direct CAN safety
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
try:
    from direct_can_safety import DirectCANSafety

    DIRECT_CAN_AVAILABLE = True
except ImportError:
    DIRECT_CAN_AVAILABLE = False
    DirectCANSafety = None


class CompetitionGeofencing(Node):
    """
    Competition Geofencing System

    Monitors GPS position against competition boundaries and geofenced areas.
    Automatically triggers safety responses for boundary violations.
    """

    def __init__(self):
        super().__init__("competition_geofencing")

        # Declare parameters
        self.declare_parameter(
            "competition_boundary_file", "config/competition_boundary.json"
        )
        self.declare_parameter("geofence_check_rate", 1.0)  # Hz
        self.declare_parameter("boundary_violation_timeout", 5.0)  # seconds
        self.declare_parameter("emergency_stop_on_violation", True)
        self.declare_parameter("can_port", "/dev/ttyACM0")

        # Get parameters
        self.boundary_file = self.get_parameter("competition_boundary_file").value
        self.check_rate = self.get_parameter("geofence_check_rate").value
        self.violation_timeout = self.get_parameter("boundary_violation_timeout").value
        self.emergency_stop_enabled = self.get_parameter(
            "emergency_stop_on_violation"
        ).value
        can_port = self.get_parameter("can_port").value

        # Initialize direct CAN safety for immediate hardware response
        try:
            self.direct_can_safety = DirectCANSafety(can_port=can_port)
            if self.direct_can_safety.is_connected():
                self.get_logger().info(
                    "Direct CAN safety connected for emergency stops"
                )
            else:
                self.get_logger().warn(
                    "Direct CAN safety not connected - using ROS2 only"
                )
                self.direct_can_safety = None
        except Exception as e:
            self.get_logger().warn(f"Failed to initialize direct CAN safety: {e}")
            self.direct_can_safety = None

        # GPS subscriber
        self.gps_sub = self.create_subscription(
            NavSatFix, "/gps/fix", self.gps_callback, 10
        )

        # Publishers
        self.violation_pub = self.create_publisher(
            Bool, "/safety/boundary_violation", 10
        )

        self.alert_pub = self.create_publisher(String, "/safety/alert", 10)

        self.boundary_viz_pub = self.create_publisher(
            Marker, "/safety/boundary_visualization", 10
        )

        # Emergency stop publisher
        self.emergency_stop_pub = self.create_publisher(Bool, "/emergency_stop", 10)

        # Boundary data
        self.competition_boundary = []  # List of (lat, lon) tuples
        self.geofenced_zones = []  # List of restricted areas
        self.current_position = None
        self.boundary_violation = False
        self.violation_start_time = None

        # Load competition boundaries
        self.load_competition_boundaries()

        # Periodic boundary checking
        self.create_timer(1.0 / self.check_rate, self.check_boundaries)

        # Visualization timer
        self.create_timer(5.0, self.publish_boundary_visualization)

        self.get_logger().info("Competition Geofencing initialized")
        self.get_logger().info(
            f"Competition boundary loaded with {len(self.competition_boundary)} points"
        )

    def load_competition_boundaries(self):
        """Load competition boundary and geofenced zones from configuration."""
        try:
            # Try to load from parameter file
            with open(self.boundary_file, "r") as f:
                boundary_data = json.load(f)

            # Load competition boundary
            if "competition_boundary" in boundary_data:
                self.competition_boundary = boundary_data["competition_boundary"]

            # Load geofenced zones
            if "geofenced_zones" in boundary_data:
                self.geofenced_zones = boundary_data["geofenced_zones"]

        except FileNotFoundError:
            # Use default URC-style boundaries if file not found
            self.get_logger().warn(
                f"Boundary file {self.boundary_file} not found, using defaults"
            )
            self.set_default_boundaries()

        except Exception as e:
            self.get_logger().error(f"Error loading boundaries: {e}")
            self.set_default_boundaries()

    def set_default_boundaries(self):
        """Set default competition boundaries for testing."""
        # Default rectangular boundary (can be customized for actual URC site)
        self.competition_boundary = [
            [33.0, -117.1],  # Southwest corner
            [33.0, -117.0],  # Southeast corner
            [33.1, -117.0],  # Northeast corner
            [33.1, -117.1],  # Northwest corner
        ]

        # Default geofenced zones (lander area, spectator areas, etc.)
        self.geofenced_zones = [
            {
                "name": "lander_area",
                "boundary": [
                    [33.05, -117.05],
                    [33.05, -117.04],
                    [33.06, -117.04],
                    [33.06, -117.05],
                ],
                "severity": "critical",
            }
        ]

    def gps_callback(self, msg: NavSatFix):
        """Process GPS position updates."""
        self.current_position = (msg.latitude, msg.longitude)

        # Check boundaries immediately on position update
        self.check_boundaries()

    def check_boundaries(self):
        """Check if current position violates any boundaries."""
        if not self.current_position:
            return

        lat, lon = self.current_position

        # Check competition boundary
        in_competition_area = self.point_in_polygon(lat, lon, self.competition_boundary)

        # Check geofenced zones
        in_restricted_zone = False
        restricted_zone_name = None

        for zone in self.geofenced_zones:
            if self.point_in_polygon(lat, lon, zone["boundary"]):
                in_restricted_zone = True
                restricted_zone_name = zone["name"]
                break

        # Determine violation status
        boundary_violation = not in_competition_area or in_restricted_zone

        # Handle boundary violation state changes
        if boundary_violation and not self.boundary_violation:
            # Violation started
            self.boundary_violation = True
            self.violation_start_time = self.get_clock().now().nanoseconds / 1e9

            self.get_logger().error("[ALERT] BOUNDARY VIOLATION DETECTED!")
            if not in_competition_area:
                self.get_logger().error("Outside competition boundary")
            if in_restricted_zone:
                self.get_logger().error(
                    f"Entered restricted zone: {restricted_zone_name}"
                )

            # Publish violation alert
            self.violation_pub.publish(Bool(data=True))

            # Send emergency stop if enabled
            if self.emergency_stop_enabled:
                # Direct CAN bypass for immediate hardware response (<1ms latency)
                if self.direct_can_safety:
                    self.direct_can_safety.boundary_violation_stop()

                # Also publish ROS2 message (for logging/monitoring)
                self.emergency_stop_pub.publish(Bool(data=True))
                self.send_alert(
                    "BOUNDARY_VIOLATION",
                    f"Outside safe zone. Emergency stop activated.",
                )

        elif not boundary_violation and self.boundary_violation:
            # Violation ended
            self.boundary_violation = False
            self.violation_start_time = None

            self.get_logger().info("[SUCCESS] Returned to safe zone")
            self.violation_pub.publish(Bool(data=False))

        # Check for prolonged violations
        if self.boundary_violation and self.violation_start_time:
            violation_duration = (
                self.get_clock().now().nanoseconds / 1e9 - self.violation_start_time
            )
            if violation_duration > self.violation_timeout:
                self.send_alert(
                    "PROLONGED_VIOLATION",
                    f"Boundary violation for {violation_duration:.1f} seconds",
                )

    def point_in_polygon(
        self, lat: float, lon: float, polygon: List[List[float]]
    ) -> bool:
        """
        Check if a point is inside a polygon using ray casting algorithm.

        Args:
            lat, lon: Point coordinates
            polygon: List of [lat, lon] coordinate pairs

        Returns:
            True if point is inside polygon
        """
        if len(polygon) < 3:
            return False

        # Convert polygon to list of points
        points = [(p[0], p[1]) for p in polygon]
        point = (lat, lon)

        # Ray casting algorithm
        inside = False
        j = len(points) - 1

        for i in range(len(points)):
            if (points[i][1] > point[1]) != (points[j][1] > point[1]) and (
                point[0]
                < (points[j][0] - points[i][0])
                * (point[1] - points[i][1])
                / (points[j][1] - points[i][1])
                + points[i][0]
            ):
                inside = not inside
            j = i

        return inside

    def send_alert(self, alert_type: str, message: str):
        """Send safety alert."""
        alert_data = {
            "type": alert_type,
            "message": message,
            "timestamp": self.get_clock().now().nanoseconds / 1e9,
            "position": self.current_position,
        }

        alert_msg = String()
        alert_msg.data = json.dumps(alert_data)
        self.alert_pub.publish(alert_msg)

    def publish_boundary_visualization(self):
        """Publish boundary visualization for RViz."""
        if not self.competition_boundary:
            return

        # Create marker for competition boundary
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "competition_boundary"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD

        # Set visual properties
        marker.scale.x = 0.1  # Line width
        marker.color.r = 1.0  # Red color
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0

        # Add boundary points (convert lat/lon to map coordinates if needed)
        # For simplicity, assume boundary is already in map coordinates
        for lat, lon in self.competition_boundary:
            point = Point()
            point.x = lon * 1000  # Rough conversion (should use proper transform)
            point.y = lat * 1000
            point.z = 0.0
            marker.points.append(point)

        # Close the loop
        if self.competition_boundary:
            first_point = Point()
            first_lat, first_lon = self.competition_boundary[0]
            first_point.x = first_lon * 1000
            first_point.y = first_lat * 1000
            first_point.z = 0.0
            marker.points.append(first_point)

        self.boundary_viz_pub.publish(marker)

    def get_boundary_status(self) -> dict:
        """Get current boundary status for monitoring."""
        return {
            "boundary_violation": self.boundary_violation,
            "current_position": self.current_position,
            "competition_boundary_points": len(self.competition_boundary),
            "geofenced_zones": len(self.geofenced_zones),
            "emergency_stop_enabled": self.emergency_stop_enabled,
        }


def main(args=None):
    rclpy.init(args=args)

    geofencing = CompetitionGeofencing()

    try:
        rclpy.spin(geofencing)
    except KeyboardInterrupt:
        pass
    finally:
        geofencing.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
