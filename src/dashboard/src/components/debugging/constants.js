/**
 * Debugging interface constants: sensor IDs, blackboard key categories, thresholds.
 * Aligns with backend SensorId enum and BlackboardKeys.
 */

export const SENSOR_IDS = [
  'imu',
  'gps',
  'camera',
  'lidar',
  'slam_pose',
  'odom',
  'battery',
  'proximity',
  'vision'
];

export const SENSOR_LABELS = {
  imu: 'IMU',
  gps: 'GPS',
  camera: 'Camera',
  lidar: 'Lidar',
  slam_pose: 'SLAM Pose',
  odom: 'Odometry',
  battery: 'Battery',
  proximity: 'Proximity',
  vision: 'Vision'
};

/** Confidence bands: green >= HIGH, yellow in [LOW, HIGH), red < LOW */
export const CONFIDENCE_THRESHOLDS = {
  HIGH: 0.8,
  LOW: 0.5
};

/** Staleness: seconds after which sensor is considered stale */
export const STALENESS_SEC = 2.0;

/** Time-series window options (seconds) */
export const TIME_WINDOWS = [30, 60, 300];

/** Blackboard key categories for grouped display (matches BlackboardKeys) */
export const BLACKBOARD_DOMAINS = {
  sensors: {
    label: 'Sensors',
    keys: [
      'imu_linear_accel_x', 'imu_linear_accel_y', 'imu_linear_accel_z',
      'imu_angular_vel_x', 'imu_angular_vel_y', 'imu_angular_vel_z',
      'gps_latitude', 'gps_longitude', 'gps_altitude',
      'motor_temp_max'
    ]
  },
  navigation: {
    label: 'Navigation',
    keys: [
      'robot_x', 'robot_y', 'robot_yaw', 'robot_velocity_x', 'robot_velocity_y',
      'slam_x', 'slam_y', 'slam_confidence',
      'navigation_status', 'path_clear', 'current_waypoint_index',
      'nav_target_x', 'nav_target_y', 'distance_to_target'
    ]
  },
  safety: {
    label: 'Safety',
    keys: [
      'safety_stop_active', 'emergency_stop_active', 'battery_level',
      'closest_obstacle_distance', 'obstacle_detected', 'proximity_violation_distance',
      'system.emergency_stop', 'system.battery_percent', 'system.last_error'
    ]
  },
  system: {
    label: 'System',
    keys: [
      'sensors_ok', 'navigation_ok', 'last_error',
      'auto.mode', 'auto.mode_request', 'auto.human_override_active', 'auto.assist_enabled'
    ]
  },
  perception: {
    label: 'Perception',
    keys: [
      'perception_confidence', 'map_quality', 'feature_count',
      'obstacle_detection_confidence'
    ]
  },
  mission: {
    label: 'Mission',
    keys: [
      'mission_active', 'samples_collected', 'waypoints_completed',
      'current_mission_phase'
    ]
  }
};

/** Key type hints for type-aware widgets (gps, quaternion, bool, int, double, string) */
export const BLACKBOARD_KEY_TYPES = {
  gps_latitude: 'double',
  gps_longitude: 'double',
  gps_altitude: 'double',
  robot_x: 'double',
  robot_y: 'double',
  robot_yaw: 'double',
  slam_x: 'double',
  slam_y: 'double',
  nav_target_x: 'double',
  nav_target_y: 'double',
  distance_to_target: 'double',
  battery_level: 'double',
  slam_confidence: 'double',
  perception_confidence: 'double',
  map_quality: 'double',
  obstacle_detection_confidence: 'double',
  closest_obstacle_distance: 'double',
  proximity_violation_distance: 'double',
  motor_temp_max: 'double',
  'system.battery_percent': 'double',
  path_clear: 'bool',
  obstacle_detected: 'bool',
  safety_stop_active: 'bool',
  emergency_stop_active: 'bool',
  mission_active: 'bool',
  sensors_ok: 'bool',
  navigation_ok: 'bool',
  'auto.human_override_active': 'bool',
  'auto.assist_enabled': 'bool',
  'system.emergency_stop': 'bool',
  current_waypoint_index: 'int',
  samples_collected: 'int',
  waypoints_completed: 'int',
  feature_count: 'int',
  navigation_status: 'string',
  last_error: 'string',
  current_mission_phase: 'string',
  'auto.mode': 'string',
  'auto.mode_request': 'string',
  'system.last_error': 'string'
};
