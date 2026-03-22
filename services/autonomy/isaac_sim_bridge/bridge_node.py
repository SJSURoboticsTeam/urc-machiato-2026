#!/usr/bin/env python3
"""
Isaac Sim bridge node stub.

Publishes a heartbeat and logs when rclpy is available. Replace with full
Isaac<->ROS2 integration when `omni.isaac` is present in the runtime image.
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
        from std_msgs.msg import String
    except ImportError:
        logger.warning("rclpy not available; isaac_sim_bridge.bridge_node exiting")
        sys.exit(0)

    rclpy.init(args=args)

    class _BridgeStub(Node):
        def __init__(self) -> None:
            super().__init__("isaac_sim_bridge_stub")
            self._pub = self.create_publisher(String, "/isaac_sim/bridge_status", 10)
            self._timer = self.create_timer(5.0, self._tick)
            self._n = 0

        def _tick(self) -> None:
            self._n += 1
            m = String()
            m.data = f"stub_ok:{self._n}"
            self._pub.publish(m)

    node = _BridgeStub()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
