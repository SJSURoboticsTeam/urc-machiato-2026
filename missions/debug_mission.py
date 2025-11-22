#!/usr/bin/env python3
"""
Debug Mission Execution - Detailed monitoring of mission progress
"""

import json
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String


class MissionDebugger(Node):
    """Debug mission execution in detail"""

    def __init__(self):
        super().__init__("mission_debugger")

        # Subscribers for monitoring
        self.odom_sub = self.create_subscription(
            Odometry, "/odom", self.odom_callback, 10
        )
        self.mission_sub = self.create_subscription(
            String, "/mission/status", self.mission_callback, 10
        )
        self.progress_sub = self.create_subscription(
            Float32, "/mission/progress", self.progress_callback, 10
        )
        self.cmd_vel_sub = self.create_subscription(
            Twist, "/cmd_vel", self.cmd_vel_callback, 10
        )

        # Publisher for test commands
        self.cmd_pub = self.create_publisher(String, "/mission/commands", 10)

        # Data tracking
        self.current_pos = None
        self.mission_status = None
        self.progress = 0.0
        self.cmd_vel = None
        self.start_time = time.time()

        self.get_logger().info("🕵️ Mission Debugger started")

    def odom_callback(self, msg):
        self.current_pos = msg.pose.pose.position
        timestamp = time.time() - self.start_time
        self.get_logger().info(".2f" ".2f")

    def mission_callback(self, msg):
        try:
            self.mission_status = json.loads(msg.data)
            self.get_logger().info(f"📋 Mission status: {self.mission_status}")
        except json.JSONDecodeError:
            pass

    def progress_callback(self, msg):
        self.progress = msg.data
        self.get_logger().info(".1f")

    def cmd_vel_callback(self, msg):
        self.cmd_vel = msg
        self.get_logger().info(".2f" ".2f")

    def send_mission_command(self, command, params=None):
        """Send a mission command"""
        cmd_data = {"command": command}
        if params:
            cmd_data.update(params)

        msg = String()
        msg.data = json.dumps(cmd_data)
        self.cmd_pub.publish(msg)
        self.get_logger().info(f"📤 Sent command: {cmd_data}")

    def run_debug_test(self):
        """Run the debug test"""
        self.get_logger().info("🧪 Starting mission debug test...")

        # Wait for system to be ready
        time.sleep(2)

        # Send mission command
        waypoints = [{"x": 2.0, "y": 0.0}]
        self.send_mission_command("start_waypoint_mission", {"waypoints": waypoints})

        # Monitor for 10 seconds
        end_time = time.time() + 10
        while time.time() < end_time and rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

            # Check if mission completed
            if self.mission_status and self.mission_status.get("status") == "completed":
                self.get_logger().info("✅ Mission completed successfully!")
                break

        # Send stop command
        self.send_mission_command("stop_mission")

        self.get_logger().info("🧪 Debug test complete")
        self.print_summary()

    def print_summary(self):
        """Print test summary"""
        self.get_logger().info("=" * 50)
        self.get_logger().info("MISSION DEBUG SUMMARY")
        self.get_logger().info("=" * 50)

        if self.current_pos:
            self.get_logger().info(".2f" ".2f")
        else:
            self.get_logger().info("❌ No position data received")

        if self.mission_status:
            self.get_logger().info(f"📋 Final mission status: {self.mission_status}")
        else:
            self.get_logger().info("❌ No mission status received")

        self.get_logger().info(".1f")
        self.get_logger().info("=" * 50)


def main():
    rclpy.init()

    try:
        debugger = MissionDebugger()
        debugger.run_debug_test()
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == "__main__":
    main()
