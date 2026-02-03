/**
 * Blackboard Key Constants
 * 
 * Centralized definitions for all blackboard keys used in the system.
 * This prevents typos, enables IDE autocomplete, and documents ownership.
 * 
 * Author: URC 2026 Autonomy Team
 */

#ifndef AUTONOMY_BT_BLACKBOARD_KEYS_HPP
#define AUTONOMY_BT_BLACKBOARD_KEYS_HPP

#include <string>

namespace BlackboardKeys {

    // ===== HIERARCHICAL (DOT) KEYS - URC / Hybrid =====
    namespace Auto {
        constexpr const char* MODE = "auto.mode";  // "teleop" | "autonomous" | "emergency"
        constexpr const char* MODE_REQUEST = "auto.mode_request";  // pending mode change
        constexpr const char* HUMAN_OVERRIDE_ACTIVE = "auto.human_override_active";  // bool
        constexpr const char* ASSIST_ENABLED = "auto.assist_enabled";  // bool
    }
    namespace System {
        constexpr const char* EMERGENCY_STOP = "system.emergency_stop";  // bool
        constexpr const char* BATTERY_PERCENT = "system.battery_percent";  // float 0-100
        constexpr const char* LAST_ERROR = "system.last_error";  // string
    }

    // ===== ROBOT STATE (Updated by odometry callback) - flat keys kept for compatibility =====
    constexpr const char* ROBOT_X = "robot_x";
    constexpr const char* ROBOT_Y = "robot_y";
    constexpr const char* ROBOT_YAW = "robot_yaw";
    constexpr const char* ROBOT_VELOCITY_X = "robot_velocity_x";
    constexpr const char* ROBOT_VELOCITY_Y = "robot_velocity_y";

    // ===== SLAM STATE (Updated by SLAM pose callback) =====
    constexpr const char* SLAM_X = "slam_x";
    constexpr const char* SLAM_Y = "slam_y";
    constexpr const char* SLAM_CONFIDENCE = "slam_confidence";

    // ===== NAVIGATION STATE (Updated by navigation system) =====
    constexpr const char* NAVIGATION_STATUS = "navigation_status";  // "idle", "navigating", "arrived", "failed"
    constexpr const char* PATH_CLEAR = "path_clear";  // bool
    constexpr const char* CURRENT_WAYPOINT_INDEX = "current_waypoint_index";  // int
    constexpr const char* NAV_TARGET_X = "nav_target_x";  // double
    constexpr const char* NAV_TARGET_Y = "nav_target_y";  // double
    constexpr const char* DISTANCE_TO_TARGET = "distance_to_target";  // double

    // ===== PERCEPTION STATE (Updated by perception/vision system) =====
    constexpr const char* PERCEPTION_CONFIDENCE = "perception_confidence";  // 0.0-1.0
    constexpr const char* MAP_QUALITY = "map_quality";  // 0.0-1.0
    constexpr const char* FEATURE_COUNT = "feature_count";  // int
    constexpr const char* OBSTACLE_DETECTION_CONFIDENCE = "obstacle_detection_confidence";  // 0.0-1.0

    // ===== OBSTACLE DETECTION (Updated by proximity monitor) =====
    constexpr const char* CLOSEST_OBSTACLE_DISTANCE = "closest_obstacle_distance";  // double
    constexpr const char* OBSTACLE_DETECTED = "obstacle_detected";  // bool

    // ===== MISSION STATE (Updated by BT nodes and missions) =====
    constexpr const char* MISSION_ACTIVE = "mission_active";  // bool
    constexpr const char* SAMPLES_COLLECTED = "samples_collected";  // int
    constexpr const char* WAYPOINTS_COMPLETED = "waypoints_completed";  // int
    constexpr const char* CURRENT_MISSION_PHASE = "current_mission_phase";  // string

    // ===== SAFETY STATE (Updated by safety system) =====
    constexpr const char* SAFETY_STOP_ACTIVE = "safety_stop_active";  // bool
    constexpr const char* PROXIMITY_VIOLATION_DISTANCE = "proximity_violation_distance";  // double
    constexpr const char* BATTERY_LEVEL = "battery_level";  // double (0.0-100.0)
    constexpr const char* EMERGENCY_STOP_ACTIVE = "emergency_stop_active";  // bool

    // ===== SYSTEM HEALTH (Updated by sensor checks and watchdogs) =====
    constexpr const char* SENSORS_OK = "sensors_ok";  // bool
    constexpr const char* NAVIGATION_OK = "navigation_ok";  // bool
    constexpr const char* LAST_ERROR = "last_error";  // string (flat key, alias for compatibility)

    // ===== HARDWARE SENSORS (Updated by hardware_interface_node) =====
    // IMU (double, m/s^2 or rad/s)
    constexpr const char* IMU_LINEAR_ACCEL_X = "imu_linear_accel_x";
    constexpr const char* IMU_LINEAR_ACCEL_Y = "imu_linear_accel_y";
    constexpr const char* IMU_LINEAR_ACCEL_Z = "imu_linear_accel_z";
    constexpr const char* IMU_ANGULAR_VEL_X = "imu_angular_vel_x";
    constexpr const char* IMU_ANGULAR_VEL_Y = "imu_angular_vel_y";
    constexpr const char* IMU_ANGULAR_VEL_Z = "imu_angular_vel_z";
    // Motor temperatures (double, Celsius)
    constexpr const char* MOTOR_TEMP_MAX = "motor_temp_max";
    // GPS (double)
    constexpr const char* GPS_LATITUDE = "gps_latitude";
    constexpr const char* GPS_LONGITUDE = "gps_longitude";
    constexpr const char* GPS_ALTITUDE = "gps_altitude";

}  // namespace BlackboardKeys

#endif  // AUTONOMY_BT_BLACKBOARD_KEYS_HPP
