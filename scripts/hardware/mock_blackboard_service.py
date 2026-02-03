#!/usr/bin/env python3
"""
Mock Blackboard Service - ROS2 node for testing CAN to blackboard flow.

Provides /blackboard/get_value and /blackboard/set_value services
matching bt_orchestrator interface. Stores values in memory for verification.

Usage (after sourcing install/setup.bash):
  python3 scripts/hardware/mock_blackboard_service.py
"""

import rclpy
from rclpy.node import Node
from autonomy_interfaces.srv import GetBlackboardValue, SetBlackboardValue
import time


class MockBlackboardService(Node):
    """Mock blackboard service for testing CAN to blackboard without BT.CPP."""

    def __init__(self):
        super().__init__("mock_blackboard_service")
        self.storage = {}  # key -> (value, value_type)

        self.get_srv = self.create_service(
            GetBlackboardValue,
            "/blackboard/get_value",
            self.handle_get,
        )
        self.set_srv = self.create_service(
            SetBlackboardValue,
            "/blackboard/set_value",
            self.handle_set,
        )

        self.set_count = 0
        self.status_timer = self.create_timer(5.0, self.print_status)

        self.get_logger().info("Mock blackboard service started")

    def handle_get(self, request, response):
        """Handle GetBlackboardValue requests."""
        key = request.key
        if key in self.storage:
            value, value_type = self.storage[key]
            response.success = True
            response.value = str(value)
            response.value_type = value_type
            response.error_message = ""
        else:
            response.success = False
            response.value = ""
            response.value_type = ""
            response.error_message = f"Key '{key}' not found"
        return response

    def handle_set(self, request, response):
        """Handle SetBlackboardValue requests."""
        key = request.key
        value_str = request.value
        value_type = request.value_type

        try:
            if value_type == "bool":
                value = value_str.lower() in ("true", "1")
            elif value_type == "int":
                value = int(float(value_str))
            elif value_type == "double":
                value = float(value_str)
            else:
                value = value_str

            self.storage[key] = (value, value_type)
            self.set_count += 1
            response.success = True
            response.error_message = ""

            self.get_logger().info("SET %s = %s (%s)" % (key, value, value_type))
        except Exception as e:
            response.success = False
            response.error_message = str(e)
            self.get_logger().error("SET %s failed: %s" % (key, e))

        return response

    def print_status(self):
        """Print stored values periodically."""
        if not self.storage:
            return
        self.get_logger().info(
            "Blackboard state (%d keys, %d total writes): %s"
            % (len(self.storage), self.set_count, list(self.storage.keys()))
        )
        for key, (val, _) in sorted(self.storage.items()):
            self.get_logger().info("  %s: %s" % (key, val))


def main(args=None):
    rclpy.init(args=args)
    node = MockBlackboardService()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, Exception):
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == "__main__":
    main()
