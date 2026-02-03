"""
Base node utilities for autonomy nodes - URC 2026.

Provides BaseURCNode for navigation and other autonomy nodes when running
inside ROS2 or in tests. Uses rclpy.node.Node when available.
"""

try:
    import rclpy
    from rclpy.node import Node

    BaseURCNode = Node
except ImportError:
    # Stub when rclpy not available (e.g. unit tests without ROS2)
    class BaseURCNode:
        """Minimal stub for Node when rclpy is not available."""

        def __init__(self, node_name: str):
            self._node_name = node_name

        def get_logger(self):
            import logging

            return logging.getLogger(self._node_name)
