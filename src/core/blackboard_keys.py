"""
Blackboard Key Constants (Python)

Centralized definitions for all blackboard keys used in the system.
This prevents typos, enables IDE autocomplete, and documents ownership.

Author: URC 2026 Autonomy Team
"""


class BlackboardKeys:
    """Blackboard key constants for unified access."""

    # ===== ROBOT STATE (Updated by odometry callback) =====
    ROBOT_X = "robot_x"
    ROBOT_Y = "robot_y"
    ROBOT_YAW = "robot_yaw"
    ROBOT_VELOCITY_X = "robot_velocity_x"
    ROBOT_VELOCITY_Y = "robot_velocity_y"

    # ===== SLAM STATE (Updated by SLAM pose callback) =====
    SLAM_X = "slam_x"
    SLAM_Y = "slam_y"
    SLAM_CONFIDENCE = "slam_confidence"

    # ===== NAVIGATION STATE (Updated by navigation system) =====
    NAVIGATION_STATUS = "navigation_status"  # "idle", "navigating", "arrived", "failed"
    PATH_CLEAR = "path_clear"  # bool
    CURRENT_WAYPOINT_INDEX = "current_waypoint_index"  # int
    NAV_TARGET_X = "nav_target_x"  # double
    NAV_TARGET_Y = "nav_target_y"  # double
    DISTANCE_TO_TARGET = "distance_to_target"  # double

    # ===== PERCEPTION STATE (Updated by perception/vision system) =====
    PERCEPTION_CONFIDENCE = "perception_confidence"  # 0.0-1.0
    MAP_QUALITY = "map_quality"  # 0.0-1.0
    FEATURE_COUNT = "feature_count"  # int
    OBSTACLE_DETECTION_CONFIDENCE = "obstacle_detection_confidence"  # 0.0-1.0

    # ===== OBSTACLE DETECTION (Updated by proximity monitor) =====
    CLOSEST_OBSTACLE_DISTANCE = "closest_obstacle_distance"  # double
    OBSTACLE_DETECTED = "obstacle_detected"  # bool
    PROXIMITY_VIOLATION_DISTANCE = "proximity_violation_distance"  # double

    # ===== MISSION STATE (Updated by BT nodes and missions) =====
    MISSION_ACTIVE = "mission_active"  # bool
    SAMPLES_COLLECTED = "samples_collected"  # int
    WAYPOINTS_COMPLETED = "waypoints_completed"  # int
    CURRENT_MISSION_PHASE = "current_mission_phase"  # string

    # ===== SAFETY STATE (Updated by safety system) =====
    SAFETY_STOP_ACTIVE = "safety_stop_active"  # bool
    EMERGENCY_STOP_ACTIVE = "emergency_stop_active"  # bool
    BATTERY_LEVEL = "battery_level"  # double (0.0-100.0)

    # ===== SYSTEM HEALTH (Updated by sensor checks and watchdogs) =====
    SENSORS_OK = "sensors_ok"  # bool
    NAVIGATION_OK = "navigation_ok"  # bool
    LAST_ERROR = "last_error"  # string
