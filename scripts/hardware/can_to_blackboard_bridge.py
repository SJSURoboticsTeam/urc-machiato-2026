#!/usr/bin/env python3
"""
CAN-to-Blackboard Bridge: republish /hardware/* to topics that blackboard writers expect.

Safety watchdog subscribes to /battery/status; SLAM node subscribes to /odom.
Hardware interface publishes /hardware/battery_state and /hardware/chassis_velocity.
This node republishes so that CAN (or simulated CAN) data flows into the blackboard.

In sim-only mode (use_sim_fallback=true), publishes mock /battery/status and /odom
when hardware topics are not available, so you can test blackboard updates without hardware.

Usage:
  python3 scripts/hardware/can_to_blackboard_bridge.py
  ros2 run <pkg> can_to_blackboard_bridge  # if installed as a node
"""

import math
import sys
from pathlib import Path

# Add workspace src for ROS2 and interfaces
WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "src"))

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header


class CANToBlackboardBridge(Node):
    """Republish /hardware/battery_state -> /battery/status, /hardware/chassis_velocity -> /odom."""

    def __init__(self):
        super().__init__("can_to_blackboard_bridge")

        self.declare_parameter("use_sim_fallback", True)
        self.declare_parameter("sim_fallback_timeout_sec", 2.0)
        self.declare_parameter("odom_frame_id", "odom")
        self.declare_parameter("child_frame_id", "base_link")

        self.use_sim_fallback = self.get_parameter("use_sim_fallback").value
        self.sim_fallback_timeout = self.get_parameter("sim_fallback_timeout_sec").value
        self.odom_frame_id = self.get_parameter("odom_frame_id").value
        self.child_frame_id = self.get_parameter("child_frame_id").value

        # Odometry integration state (for Twist -> Odometry)
        self.odom_x = 0.0
        self.odom_y = 0.0
        self.odom_theta = 0.0
        self.last_twist_time = None

        # Publishers (topics that blackboard writers subscribe to)
        self.battery_pub = self.create_publisher(BatteryState, "/battery/status", 10)
        self.odom_pub = self.create_publisher(Odometry, "/odom", 10)

        # Subscribers (hardware interface topics)
        self.battery_sub = self.create_subscription(
            BatteryState,
            "/hardware/battery_state",
            self._on_battery,
            10,
        )
        self.chassis_sub = self.create_subscription(
            TwistStamped,
            "/hardware/chassis_velocity",
            self._on_chassis_velocity,
            10,
        )

        # Sim fallback: publish mock data when no hardware data received
        if self.use_sim_fallback:
            self._last_battery_time = None
            self._last_chassis_time = None
            self.sim_timer = self.create_timer(0.5, self._sim_fallback_publish)
        else:
            self.sim_timer = None

        self.get_logger().info(
            "can_to_blackboard_bridge started: /hardware/battery_state -> /battery/status, "
            "/hardware/chassis_velocity -> /odom; use_sim_fallback=%s"
            % self.use_sim_fallback
        )

    def _on_battery(self, msg: BatteryState) -> None:
        """Republish hardware battery to /battery/status for safety_watchdog -> blackboard."""
        msg.header.stamp = self.get_clock().now().to_msg()
        self.battery_pub.publish(msg)
        if self.use_sim_fallback:
            self._last_battery_time = self.get_clock().now().nanoseconds / 1e9

    def _on_chassis_velocity(self, msg: TwistStamped) -> None:
        """Convert chassis velocity to odometry and publish /odom for SLAM node -> blackboard."""
        now = self.get_clock().now()
        t = now.nanoseconds / 1e9
        if self.last_twist_time is not None:
            dt = t - self.last_twist_time
            vx = msg.twist.linear.x
            vy = msg.twist.linear.y
            omega = msg.twist.angular.z
            self.odom_x += (vx * math.cos(self.odom_theta) - vy * math.sin(self.odom_theta)) * dt
            self.odom_y += (vx * math.sin(self.odom_theta) + vy * math.cos(self.odom_theta)) * dt
            self.odom_theta += omega * dt
        self.last_twist_time = t

        odom = Odometry()
        odom.header = Header(stamp=now.to_msg(), frame_id=self.odom_frame_id)
        odom.child_frame_id = self.child_frame_id
        odom.pose.pose.position.x = self.odom_x
        odom.pose.pose.position.y = self.odom_y
        odom.pose.pose.position.z = 0.0
        q = self._yaw_to_quaternion(self.odom_theta)
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        odom.twist.twist = msg.twist
        self.odom_pub.publish(odom)

        if self.use_sim_fallback:
            self._last_chassis_time = t

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> tuple:
        """Convert yaw (radians) to quaternion (x, y, z, w)."""
        return (0.0, 0.0, math.sin(yaw / 2.0), math.cos(yaw / 2.0))

    def _sim_fallback_publish(self) -> None:
        """If no hardware messages received within timeout, publish mock /battery/status and /odom."""
        now_ns = self.get_clock().now().nanoseconds
        now_s = now_ns / 1e9
        battery_stale = (
            self._last_battery_time is None
            or (now_s - self._last_battery_time) > self.sim_fallback_timeout
        )
        chassis_stale = (
            self._last_chassis_time is None
            or (now_s - self._last_chassis_time) > self.sim_fallback_timeout
        )
        if not battery_stale and not chassis_stale:
            return

        if battery_stale:
            bat = BatteryState()
            bat.header = Header(stamp=self.get_clock().now().to_msg(), frame_id="battery")
            bat.voltage = 24.0
            bat.current = -5.0
            bat.percentage = 0.85
            self.battery_pub.publish(bat)

        if chassis_stale:
            if self.last_twist_time is None:
                self.last_twist_time = now_s
            odom = Odometry()
            odom.header = Header(
                stamp=self.get_clock().now().to_msg(),
                frame_id=self.odom_frame_id,
            )
            odom.child_frame_id = self.child_frame_id
            odom.pose.pose.position.x = self.odom_x
            odom.pose.pose.position.y = self.odom_y
            odom.pose.pose.position.z = 0.0
            q = self._yaw_to_quaternion(self.odom_theta)
            odom.pose.pose.orientation.x = q[0]
            odom.pose.pose.orientation.y = q[1]
            odom.pose.pose.orientation.z = q[2]
            odom.pose.pose.orientation.w = q[3]
            self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = CANToBlackboardBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
