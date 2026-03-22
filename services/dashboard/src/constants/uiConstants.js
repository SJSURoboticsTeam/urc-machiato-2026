import { MessageSquare, GitBranch, Database, Activity, Columns3 } from 'lucide-react';

/**
 * UI Constants - Extracted magic numbers and configuration values
 * Centralizes all hardcoded values for easier maintenance
 */

export const UI_CONSTANTS = {
  // Timing
  FLASH_DURATION: 500,                    // Visual flash duration (ms)
  MESSAGE_DELAY_MIN: 500,                 // Minimum message delay (ms)
  MESSAGE_DELAY_MAX: 1000,                // Maximum message delay (ms)
  TEST_EXECUTION_DELAY: 2000,             // Base test execution time (ms)
  TEST_EXECUTION_VARIANCE: 3000,          // Test execution variance (ms)
  TEST_PHASE_DELAY: 500,                  // Delay between test phases (ms)

  // Probabilities & Success Rates
  TEST_SUCCESS_RATE: 0.8,                 // 80% success rate for mock tests

  // Mock Data Variance
  GPS_LATITUDE_BASE: 38.4,
  GPS_LONGITUDE_BASE: -110.8,
  GPS_ALTITUDE_BASE: 1500,
  GPS_POSITION_VARIANCE: 0.01,            // ±0.01 degrees
  GPS_ALTITUDE_VARIANCE: 10,              // ±10 meters

  IMU_ACCEL_VARIANCE: 2.0,                // ±2.0 m/s²
  IMU_GYRO_VARIANCE: 1.0,                 // ±1.0 rad/s
  IMU_BASE_ACCEL_Z: 9.81,                 // Gravity
  IMU_TEMPERATURE_BASE: 25.0,

  MOTOR_VELOCITY_VARIANCE: 2.0,           // ±2.0 rad/s

  CMD_VEL_LINEAR_VARIANCE: 2.0,           // ±2.0 m/s
  CMD_VEL_ANGULAR_VARIANCE: 1.0,          // ±1.0 rad/s

  BATTERY_VOLTAGE_BASE: 24.0,
  BATTERY_CURRENT_BASE: 5.0,
  BATTERY_VOLTAGE_VARIANCE: 2.0,          // ±2.0V
  BATTERY_CURRENT_VARIANCE: 2.0,          // ±2.0A
  BATTERY_PERCENTAGE_BASE: 75,
  BATTERY_PERCENTAGE_VARIANCE: 10,        // ±10%

  // Limits & History
  MESSAGE_HISTORY_LIMIT: 19,              // Keep last 20 messages
  RECENT_TESTS_LIMIT: 5,                  // Show last 5 tests
  TOPIC_UPDATE_INTERVAL: 1000,            // Update topics every second

  // Component Config
  AVAILABLE_STATES: [
    'BOOT', 'READY', 'TELEOPERATION', 'AUTONOMOUS',
    'NAVIGATE', 'SEARCH', 'APPROACH', 'MANIPULATE',
    'SCIENCE', 'RETURN', 'ERROR', 'EMERGENCY_STOP'
  ],

  MESSAGE_TARGETS: ['websocket', 'ros2', 'can'],

  CAN_MESSAGE_TYPES: [
    'can_sensor_request',
    'can_motor_command',
    'can_diagnostic_query',
    'can_bus_reset',
    'can_status_check'
  ],

  COMPONENT_TABS: [
    { id: 'overview', name: 'Overview', icon: '📊' },
    { id: 'safety', name: 'Safety System', icon: '🛡️' },
    { id: 'sensors', name: 'Sensors & CAN', icon: '🔌' },
    { id: 'vision', name: 'Computer Vision', icon: '👁️' },
    { id: 'navigation', name: 'Navigation', icon: '🧭' },
    { id: 'history', name: 'Test History', icon: '📋' }
  ],

  TESTING_TABS: [
    { id: 'three-column', label: 'Three Column', icon: Columns3 },
    { id: 'messages', label: 'Messages', icon: MessageSquare },
    { id: 'states', label: 'States', icon: GitBranch },
    { id: 'topics', label: 'Topics', icon: Database },
    { id: 'metrics', label: 'Metrics', icon: Activity }
  ]
};

// Status mappings for consistent styling
export const STATUS_STYLES = {
  operational: 'text-green-600 bg-green-100',
  connected: 'text-green-600 bg-green-100',
  demo: 'text-yellow-600 bg-yellow-100',
  mock: 'text-orange-600 bg-orange-100',
  disconnected: 'text-red-600 bg-red-100',
  unknown: 'text-gray-600 bg-gray-100',
  passed: 'text-green-600',
  failed: 'text-red-600',
  running: 'text-blue-600',
  testing: 'text-blue-400'
};

// Priority badge styles
export const PRIORITY_STYLES = {
  1: 'bg-red-100 text-red-800',
  2: 'bg-blue-100 text-blue-800',
  3: 'bg-green-100 text-green-800',
  4: 'bg-orange-100 text-orange-800',
  5: 'bg-purple-100 text-purple-800'
};
