#!/usr/bin/env python3
"""Smoke test: read LiDAR-related blackboard keys via GetBlackboardValue.

Requires a sourced ROS 2 workspace with ``autonomy_interfaces``, and
``bt_orchestrator`` running with lifecycle configured and activated so
``/blackboard/get_value`` exists.

Example::

    source install/setup.bash
    python3 scripts/hardware/lidar_blackboard_smoke_test.py

See docs/hardware/l2_lidar_blackboard_e2e.rst for full stack order.
"""

from __future__ import annotations

import argparse
import sys
from typing import Sequence

import rclpy
from rclpy.node import Node

from autonomy_interfaces.srv import GetBlackboardValue

DEFAULT_KEYS: tuple[str, ...] = (
    "closest_obstacle_distance",
    "obstacle_detected",
    "proximity_violation_distance",
)


def main(argv: Sequence[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--service",
        default="/blackboard/get_value",
        help="GetBlackboardValue service name",
    )
    parser.add_argument(
        "--timeout",
        type=float,
        default=15.0,
        help="Seconds to wait for the service",
    )
    parser.add_argument(
        "keys",
        nargs="*",
        default=list(DEFAULT_KEYS),
        help="Blackboard keys to read (default: obstacle-related set)",
    )
    args = parser.parse_args(list(argv) if argv is not None else None)

    rclpy.init()
    node = Node("lidar_blackboard_smoke_test")
    client = node.create_client(GetBlackboardValue, args.service)
    if not client.wait_for_service(timeout_sec=args.timeout):
        node.get_logger().error(
            "Service %s not available. Start bt_orchestrator and run "
            "lifecycle configure then activate. "
            "See docs/hardware/l2_lidar_blackboard_e2e.rst.",
            args.service,
        )
        node.destroy_node()
        rclpy.shutdown()
        return 1

    exit_code = 0
    for key in args.keys:
        request = GetBlackboardValue.Request()
        request.key = key
        request.value_type = ""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if not future.done():
            node.get_logger().error("Timeout calling get_value for %s", key)
            exit_code = 1
            continue
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001
            node.get_logger().error("get_value failed for %s: %s", key, exc)
            exit_code = 1
            continue
        node.get_logger().info(
            "key=%s success=%s type=%s value=%s err=%s",
            key,
            response.success,
            response.value_type,
            response.value,
            response.error_message or "",
        )
        if not response.success:
            exit_code = 1

    node.destroy_node()
    rclpy.shutdown()
    return exit_code


if __name__ == "__main__":
    sys.exit(main())
