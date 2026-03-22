/**
 * Validation utilities for ROS and simulation messages.
 * Uses Zod schemas; returns { valid, sanitized }. Used before updating telemetry/state.
 */
import { telemetryPayloadSchema, simulationMessageSchema } from './validationSchemas';

const DEFAULT_TELEMETRY = {
  battery: 85,
  gps: { satellites: 12, hdop: 1.2, position: { lat: 38.406, lon: -110.792 } },
  speed: 0.0,
  temperature: 28,
  timestamp: Date.now(),
  imu: { accel_x: 0, accel_y: 0, accel_z: 9.81, gyro_x: 0, gyro_y: 0, gyro_z: 0 },
  environment: { temperature: 25, humidity: 30, visibility: 1.0, dust_density: 0.0 }
};

/**
 * Validate a telemetry payload (full or partial). Returns { valid: true, sanitized } or { valid: false, sanitized }.
 * sanitized merges with defaults so consumers always get a full shape.
 */
export function validateTelemetryPayload(obj) {
  if (obj == null || typeof obj !== 'object' || Array.isArray(obj)) {
    return { valid: false, sanitized: { ...DEFAULT_TELEMETRY, timestamp: Date.now() } };
  }
  const result = telemetryPayloadSchema.safeParse(obj);
  if (result.success) {
    return { valid: true, sanitized: result.data };
  }
  return { valid: false, sanitized: { ...DEFAULT_TELEMETRY, timestamp: Date.now() } };
}

/**
 * Validate simulation WebSocket message. Expects { type: 'simulation_update', simulation_data: object }.
 * Returns { valid: true, sanitized } for simulation_data, or { valid: false, sanitized: null }.
 */
export function validateSimulationMessage(obj) {
  if (obj == null || typeof obj !== 'object' || Array.isArray(obj)) {
    return { valid: false, sanitized: null };
  }
  const result = simulationMessageSchema.safeParse(obj);
  if (result.success && result.data.simulation_data != null) {
    return { valid: true, sanitized: result.data.simulation_data };
  }
  return { valid: false, sanitized: null };
}
