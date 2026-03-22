"""
Blackboard Key Constants (Python)

Centralized definitions for all blackboard keys used in the system.
This prevents typos, enables IDE autocomplete, and documents ownership.
Mirrors C++ blackboard_keys.hpp (same key strings).

Author: URC 2026 Autonomy Team
"""


class Auto:
    """Hybrid / autonomous mode keys (dot notation)."""

    MODE = "auto.mode"  # "teleop" | "autonomous" | "emergency"
    MODE_REQUEST = "auto.mode_request"
    HUMAN_OVERRIDE_ACTIVE = "auto.human_override_active"  # bool
    ASSIST_ENABLED = "auto.assist_enabled"  # bool


class System:
    """System health / safety keys (dot notation)."""

    EMERGENCY_STOP = "system.emergency_stop"  # bool
    BATTERY_PERCENT = "system.battery_percent"  # float 0-100
    LAST_ERROR = "system.last_error"  # string


class BlackboardKeys:
    """Blackboard key constants for unified access. Flat keys for compatibility."""

    # Hierarchical (dot) namespaces
    Auto = Auto
    System = System

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
    LAST_ERROR = "last_error"  # string (flat key, compatibility)

    # ===== HARDWARE SENSORS (Updated by hardware_interface_node) =====
    # IMU (double, m/s^2 or rad/s)
    IMU_LINEAR_ACCEL_X = "imu_linear_accel_x"
    IMU_LINEAR_ACCEL_Y = "imu_linear_accel_y"
    IMU_LINEAR_ACCEL_Z = "imu_linear_accel_z"
    IMU_ANGULAR_VEL_X = "imu_angular_vel_x"
    IMU_ANGULAR_VEL_Y = "imu_angular_vel_y"
    IMU_ANGULAR_VEL_Z = "imu_angular_vel_z"
    # Motor temperatures (double, Celsius)
    MOTOR_TEMP_MAX = "motor_temp_max"
    # GPS (double)
    GPS_LATITUDE = "gps_latitude"
    GPS_LONGITUDE = "gps_longitude"
    GPS_ALTITUDE = "gps_altitude"
