#!/usr/bin/env python3
"""
Safety adapter stub between Isaac Sim and autonomy safety topics.

Subscribes to a minimal topic set when rclpy is available; no-op otherwise.
"""

from __future__ import annotations

import logging
import sys
from typing import Any, Optional

logger = logging.getLogger(__name__)


def main(args: Optional[Any] = None) -> None:
    logging.basicConfig(level=logging.INFO)
    try:
        import rclpy
        from rclpy.node import Node
    except ImportError:
        logger.warning("rclpy not available; sim_safety_adapter exiting")
        sys.exit(0)

    rclpy.init(args=args)

    class _Adapter(Node):
        def __init__(self) -> None:
            super().__init__("isaac_sim_safety_adapter_stub")
            self.get_logger().info("sim_safety_adapter stub running")

    node = _Adapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
