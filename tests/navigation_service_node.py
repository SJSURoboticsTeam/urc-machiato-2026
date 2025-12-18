#!/usr/bin/env python3
"""
Navigation Service ROS2 Node for Integration Testing

Provides path planning and navigation services needed for navigation integration tests.
This is a simplified mock implementation for testing purposes.
"""

import math
from typing import List

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from nav_msgs.srv import GetPlan
from rclpy.node import Node

# Mock service if not available
try:
    from nav2_msgs.srv import ComputePathToPose
except ImportError:
    from std_srvs.srv import Empty as ComputePathToPose


class NavigationServiceNode(Node):
    """ROS2 node that provides navigation services for integration testing."""

    def __init__(self):
        super().__init__("navigation_service_node")

        # Services
        self.path_planning_service = self.create_service(
            GetPlan, "/plan", self.plan_path_callback
        )

        self.compute_path_service = self.create_service(
            ComputePathToPose, "/compute_path_to_pose", self.compute_path_callback
        )

        # Publishers
        self.path_pub = self.create_publisher(Path, "/planned_path", 10)

        self.get_logger().info("Navigation Service Node initialized")

    def plan_path_callback(self, request, response):
        """Handle path planning requests."""
        try:
            # Create a simple path from start to goal
            path = Path()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = "map"

            # Generate waypoints (simple straight line for testing)
            start = request.start.pose.position
            goal = request.goal.pose.position

            num_waypoints = 10
            for i in range(num_waypoints + 1):
                t = i / num_waypoints

                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = start.x + t * (goal.x - start.x)
                pose.pose.position.y = start.y + t * (goal.y - start.y)
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0

                path.poses.append(pose)

            response.plan = path
            self.path_pub.publish(path)

            self.get_logger().info(f"Planned path with {len(path.poses)} waypoints")

        except Exception as e:
            self.get_logger().error(f"Path planning failed: {e}")
            response.plan = Path()  # Return empty path

        return response

    def compute_path_callback(self, request, response):
        """Handle compute path to pose requests."""
        try:
            # Mock path computation
            path = Path()
            path.header.stamp = self.get_clock().now().to_msg()
            path.header.frame_id = "map"

            # Simple path to goal
            start = Point(x=0.0, y=0.0, z=0.0)
            goal = request.goal.pose.position

            # Generate simple path
            num_waypoints = 5
            for i in range(num_waypoints + 1):
                t = i / num_waypoints

                pose = PoseStamped()
                pose.header = path.header
                pose.pose.position.x = start.x + t * (goal.x - start.x)
                pose.pose.position.y = start.y + t * (goal.y - start.y)
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0

                path.poses.append(pose)

            # Mock response
            if hasattr(response, "path"):
                response.path = path
            if hasattr(response, "planning_time"):
                response.planning_time = 0.1

            self.path_pub.publish(path)
            self.get_logger().info("Computed path to pose")

        except Exception as e:
            self.get_logger().error(f"Path computation failed: {e}")

        return response


def main():
    """Main entry point."""
    import signal
    import sys

    def signal_handler(signum, frame):
        """Handle shutdown signals."""
        print("\n Shutting down navigation service...")
        sys.exit(0)

    # Set up signal handlers
    signal.signal(signal.SIGINT, signal_handler)
    signal.signal(signal.SIGTERM, signal_handler)

    rclpy.init()
    node = NavigationServiceNode()

    try:
        print("[IGNITE] Navigation Service running...")
        print("[ANTENNA] Providing path planning services")
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        print("[PASS] Navigation Service stopping...")
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
