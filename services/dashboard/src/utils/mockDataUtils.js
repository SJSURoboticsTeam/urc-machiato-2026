/**
 * Mock Data Utilities - Centralized mock data generation
 * Eliminates code duplication and provides consistent test data
 */

import { UI_CONSTANTS } from '../constants/uiConstants';

/**
 * Generate a random value around a base value with specified variance
 * @param {number} base - Base value
 * @param {number} variance - Maximum deviation from base
 * @returns {number} Random value within variance range
 */
export const generateMockSensorData = (base, variance) =>
  base + (Math.random() - 0.5) * variance * 2;

/**
 * Generate mock GPS data
 * @returns {Object} GPS position data
 */
export const generateMockGPS = () => ({
  latitude: generateMockSensorData(UI_CONSTANTS.GPS_LATITUDE_BASE, UI_CONSTANTS.GPS_POSITION_VARIANCE),
  longitude: generateMockSensorData(UI_CONSTANTS.GPS_LONGITUDE_BASE, UI_CONSTANTS.GPS_POSITION_VARIANCE),
  altitude: generateMockSensorData(UI_CONSTANTS.GPS_ALTITUDE_BASE, UI_CONSTANTS.GPS_ALTITUDE_VARIANCE)
});

/**
 * Generate mock IMU data
 * @returns {Object} IMU sensor data
 */
export const generateMockIMU = () => ({
  accel_x: generateMockSensorData(0, UI_CONSTANTS.IMU_ACCEL_VARIANCE),
  accel_y: generateMockSensorData(0, UI_CONSTANTS.IMU_ACCEL_VARIANCE),
  accel_z: generateMockSensorData(UI_CONSTANTS.IMU_BASE_ACCEL_Z, UI_CONSTANTS.IMU_ACCEL_VARIANCE * 0.1),
  gyro_x: generateMockSensorData(0, UI_CONSTANTS.IMU_GYRO_VARIANCE),
  gyro_y: generateMockSensorData(0, UI_CONSTANTS.IMU_GYRO_VARIANCE),
  gyro_z: generateMockSensorData(0, UI_CONSTANTS.IMU_GYRO_VARIANCE),
  temperature: UI_CONSTANTS.IMU_TEMPERATURE_BASE + (Math.random() - 0.5) * 5
});

/**
 * Generate mock command velocity data
 * @returns {Object} Velocity command data
 */
export const generateMockCmdVel = () => ({
  linear: {
    x: generateMockSensorData(0, UI_CONSTANTS.CMD_VEL_LINEAR_VARIANCE),
    y: 0,
    z: 0
  },
  angular: {
    x: 0,
    y: 0,
    z: generateMockSensorData(0, UI_CONSTANTS.CMD_VEL_ANGULAR_VARIANCE)
  }
});

/**
 * Generate mock battery data
 * @returns {Object} Battery status data
 */
export const generateMockBattery = () => ({
  voltage: generateMockSensorData(UI_CONSTANTS.BATTERY_VOLTAGE_BASE, UI_CONSTANTS.BATTERY_VOLTAGE_VARIANCE),
  current: generateMockSensorData(UI_CONSTANTS.BATTERY_CURRENT_BASE, UI_CONSTANTS.BATTERY_CURRENT_VARIANCE),
  percentage: Math.max(0, Math.min(100,
    generateMockSensorData(UI_CONSTANTS.BATTERY_PERCENTAGE_BASE, UI_CONSTANTS.BATTERY_PERCENTAGE_VARIANCE)
  ))
});

/**
 * Generate mock motor data for multiple motors
 * @param {number} motorCount - Number of motors to generate data for
 * @returns {Array} Array of motor data objects
 */
export const generateMockMotors = (motorCount = 4) =>
  Array.from({ length: motorCount }, (_, index) => ({
    id: index,
    position: generateMockSensorData(0, Math.PI), // ±π radians
    velocity: generateMockSensorData(0, UI_CONSTANTS.MOTOR_VELOCITY_VARIANCE)
  }));

/**
 * Generate random message delay within configured range
 * @returns {number} Delay in milliseconds
 */
export const generateMessageDelay = () =>
  UI_CONSTANTS.MESSAGE_DELAY_MIN +
  Math.random() * (UI_CONSTANTS.MESSAGE_DELAY_MAX - UI_CONSTANTS.MESSAGE_DELAY_MIN);

/**
 * Generate random test execution time
 * @returns {number} Execution time in milliseconds
 */
export const generateTestExecutionTime = () =>
  UI_CONSTANTS.TEST_EXECUTION_DELAY +
  Math.random() * UI_CONSTANTS.TEST_EXECUTION_VARIANCE;

/**
 * Determine if a mock test should succeed based on success rate
 * @returns {boolean} True if test should succeed
 */
export const shouldTestSucceed = () =>
  Math.random() < UI_CONSTANTS.TEST_SUCCESS_RATE;
