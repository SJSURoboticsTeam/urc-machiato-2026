#!/usr/bin/env python3
"""
Blackboard Visualizer: print unified blackboard keys for debugging CAN/sim -> blackboard flow.

Polls /blackboard/get_value for key keys and prints them periodically.
Use with: bridge (can_to_blackboard_bridge) + safety_watchdog + slam_node (or sim)
to verify that CAN or simulated data is reaching the blackboard.

Usage:
  python3 scripts/hardware/blackboard_visualizer.py [--rate 2.0]
  (Requires ROS2 workspace sourced and blackboard service running, e.g. from BT.CPP.)
"""

import argparse
import sys
import time
from pathlib import Path

WORKSPACE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(WORKSPACE / "src"))

import rclpy
from rclpy.node import Node

# Keys to display (subset of BlackboardKeys that CAN/hardware/sim can affect)
DISPLAY_KEYS = [
    ("robot_x", "double"),
    ("robot_y", "double"),
    ("robot_yaw", "double"),
    ("robot_velocity_x", "double"),
    ("robot_velocity_y", "double"),
    ("slam_x", "double"),
    ("slam_y", "double"),
    ("slam_confidence", "double"),
    ("perception_confidence", "double"),
    ("map_quality", "double"),
    ("feature_count", "int"),
    ("battery_level", "double"),
    ("safety_stop_active", "bool"),
    ("emergency_stop_active", "bool"),
    ("navigation_status", "string"),
    ("path_clear", "bool"),
    ("distance_to_target", "double"),
    ("obstacle_detected", "bool"),
    ("closest_obstacle_distance", "double"),
    ("mission_active", "bool"),
    ("waypoints_completed", "int"),
    ("current_mission_phase", "string"),
    ("sensors_ok", "bool"),
    ("navigation_ok", "bool"),
    ("last_error", "string"),
]


def main():
    parser = argparse.ArgumentParser(
        description="Print blackboard keys for CAN/sim testing"
    )
    parser.add_argument(
        "--rate", type=float, default=2.0, help="Print interval in seconds"
    )
    parser.add_argument("--once", action="store_true", help="Print once and exit")
    args = parser.parse_args()

    rclpy.init()
    node = Node("blackboard_visualizer")

    try:
        from autonomy_interfaces.srv import GetBlackboardValue
    except ImportError:
        node.get_logger().error(
            "autonomy_interfaces not found. Source workspace and build: "
            "colcon build --packages-select autonomy_interfaces"
        )
        rclpy.shutdown()
        return 1

    client = node.create_client(GetBlackboardValue, "/blackboard/get_value")
    if not client.wait_for_service(timeout_sec=5.0):
        node.get_logger().warn(
            "Blackboard service /blackboard/get_value not available. "
            "Start the BT.CPP node or blackboard server, then run this again."
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    def get_value(key: str, value_type: str) -> str:
        req = GetBlackboardValue.Request()
        req.key = key
        req.value_type = value_type
        future = client.call_async(req)
        rclpy.spin_until_future_complete(node, future, timeout_sec=0.5)
        if future.done():
            resp = future.result()
            if resp and resp.success:
                return resp.value
        return "<n/a>"

    try:
        while True:
            lines = ["Blackboard (keys affected by CAN/sim):"]
            for key, vtype in DISPLAY_KEYS:
                val = get_value(key, vtype)
                lines.append("  %s: %s" % (key, val))
            print("\n".join(lines))
            print()

            if args.once:
                break
            time.sleep(args.rate)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
