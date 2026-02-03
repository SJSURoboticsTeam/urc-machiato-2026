"""
Navigation Subsystem - Autonomy Core

Handles path planning, motion control, and GNSS processing.
NavigationNode is not imported here to avoid pulling in ROS2/node_utils when
only path_planner, motion_controller, or gnss_processor are needed.
Import via: from autonomy_core.navigation.navigation_node import NavigationNode
"""

from .path_planner import PathPlanner
from .motion_controller import MotionController
from .gnss_processor import GNSSProcessor

__all__ = [
    "PathPlanner",
    "MotionController",
    "GNSSProcessor",
]
