/**
 * Zod schemas for ROS and simulation messages.
 * Used by validation.js to validate and sanitize incoming data.
 */
import { z } from 'zod';

const gpsPositionSchema = z.object({
  lat: z.number(),
  lon: z.number()
}).default({ lat: 38.406, lon: -110.792 });

const gpsSchema = z.object({
  satellites: z.number().default(12),
  hdop: z.number().default(1.2),
  position: gpsPositionSchema
}).default({
  satellites: 12,
  hdop: 1.2,
  position: { lat: 38.406, lon: -110.792 }
});

const imuSchema = z.object({
  accel_x: z.number().default(0),
  accel_y: z.number().default(0),
  accel_z: z.number().default(9.81),
  gyro_x: z.number().default(0),
  gyro_y: z.number().default(0),
  gyro_z: z.number().default(0)
}).default({ accel_x: 0, accel_y: 0, accel_z: 9.81, gyro_x: 0, gyro_y: 0, gyro_z: 0 });

const environmentSchema = z.object({
  temperature: z.number().default(25),
  humidity: z.number().default(30),
  visibility: z.number().default(1.0),
  dust_density: z.number().default(0.0)
}).default({ temperature: 25, humidity: 30, visibility: 1.0, dust_density: 0.0 });

/** Full or partial telemetry payload schema; merges with defaults */
export const telemetryPayloadSchema = z.object({
  battery: z.number().min(0).max(100).optional(),
  gps: gpsSchema.optional(),
  speed: z.number().min(0).optional(),
  temperature: z.number().optional(),
  timestamp: z.number().optional(),
  imu: imuSchema.optional(),
  environment: environmentSchema.optional()
}).transform((data) => {
  const defaults = {
    battery: 85,
    gps: { satellites: 12, hdop: 1.2, position: { lat: 38.406, lon: -110.792 } },
    speed: 0.0,
    temperature: 28,
    timestamp: Date.now(),
    imu: { accel_x: 0, accel_y: 0, accel_z: 9.81, gyro_x: 0, gyro_y: 0, gyro_z: 0 },
    environment: { temperature: 25, humidity: 30, visibility: 1.0, dust_density: 0.0 }
  };
  return {
    ...defaults,
    ...data,
    timestamp: data.timestamp ?? Date.now(),
    gps: data.gps ? { ...defaults.gps, ...data.gps, position: { ...defaults.gps.position, ...(data.gps.position || {}) } } : defaults.gps,
    imu: data.imu ? { ...defaults.imu, ...data.imu } : defaults.imu,
    environment: data.environment ? { ...defaults.environment, ...data.environment } : defaults.environment
  };
});

/** Simulation WebSocket message: { type: 'simulation_update', simulation_data: object } */
export const simulationMessageSchema = z.object({
  type: z.literal('simulation_update'),
  simulation_data: z.record(z.string(), z.unknown())
});

/** WebSocket message: { type: 'command_update', command_data: object } */
export const commandUpdateMessageSchema = z.object({
  type: z.literal('command_update'),
  command_data: z.record(z.string(), z.unknown())
});

/** Mission progress from /mission/progress (std_msgs/String JSON) */
export const missionProgressSchema = z.object({
  mission_id: z.string().optional(),
  name: z.string().optional(),
  progress: z.number().min(0).max(100).optional(),
  current_task: z.string().optional(),
  next_task: z.string().optional(),
  eta: z.string().optional(),
  waypoints: z.union([z.number(), z.string()]).optional(),
  samples: z.union([z.number(), z.string()]).optional(),
  analysis: z.string().optional()
});

/** Mission status from /mission/status */
export const missionStatusSchema = z.object({
  state: z.string().optional(),
  mission_id: z.string().optional(),
  error: z.string().optional()
}).passthrough();

/** CAN sensor data from /can/sensor_data */
export const canSensorDataSchema = z.object({
  sensor_type: z.string().optional(),
  value: z.number().optional(),
  lat: z.number().optional(),
  lon: z.number().optional(),
  temp: z.number().optional(),
  status: z.string().optional()
}).passthrough();

/** State machine current state from /state_machine/current_state */
export const stateMachineCurrentStateSchema = z.object({
  state: z.string().optional(),
  substate: z.string().optional(),
  metadata: z.record(z.unknown()).optional()
}).passthrough();

/** State machine transition from /state_machine/state_transition */
export const stateMachineTransitionSchema = z.object({
  from_state: z.string().optional(),
  to_state: z.string().optional(),
  reason: z.string().optional(),
  timestamp: z.number().optional(),
  success: z.boolean().optional()
}).passthrough();

/** Generic safety/health JSON from ROS (dashboard, alerts, system_health, etc.) */
export const safetyPayloadSchema = z.record(z.string(), z.unknown());

/** Sensor analytics from debugging topic: { sensors: Record<string, { confidence?, health_score?, last_update_ns? }> } */
export const sensorAnalyticsSchema = z.object({
  sensors: z.record(z.string(), z.object({
    confidence: z.number().optional(),
    health_score: z.number().optional(),
    last_update_ns: z.number().optional()
  }).passthrough()).optional()
}).passthrough();

/** Communication health from /system/communication_health */
export const communicationHealthSchema = z.object({
  failover_active: z.boolean().optional(),
  current_channel: z.string().optional()
}).passthrough();

/** Map/path data from /frontend/map */
export const mapPathSchema = z.object({
  path: z.array(z.unknown()).optional()
}).passthrough();

/** Waypoints from /frontend/waypoints */
export const waypointsSchema = z.object({
  waypoints: z.array(z.unknown()).optional()
}).passthrough();

/** AOI status per sensor from /system/aoi_status */
export const aoiStatusSchema = z.object({
  sensor_name: z.string().optional(),
  current_aoi: z.number().optional(),
  freshness_status: z.string().optional(),
  quality_score: z.number().optional(),
  transport_type: z.string().optional(),
  network_latency: z.number().optional(),
  transport_latency: z.number().optional(),
  congestion_detected: z.boolean().optional(),
  congestion_factor: z.number().optional(),
  predicted_aoi: z.number().optional(),
  aoi_trend: z.number().optional()
}).passthrough();

/** AOI metrics from /system/aoi_metrics */
export const aoiMetricsSchema = z.object({
  system_average_aoi: z.number().optional(),
  fresh_sensors: z.number().optional(),
  total_sensors: z.number().optional(),
  health_status: z.string().optional(),
  serial_sensors: z.number().optional(),
  can_sensors: z.number().optional(),
  ethernet_sensors: z.number().optional(),
  local_sensors: z.number().optional(),
  avg_network_latency: z.number().optional(),
  max_network_latency: z.number().optional(),
  congested_links: z.number().optional(),
  network_health_score: z.number().optional(),
  network_recommendations: z.array(z.unknown()).optional()
}).passthrough();

/**
 * Parse JSON string and validate with Zod schema. Returns parsed data or null on failure.
 * @param {string} raw - JSON string (e.g. message.data)
 * @param {import('zod').ZodSchema} schema
 * @returns {unknown|null}
 */
export function parseAndValidate(raw, schema) {
  if (typeof raw !== 'string') return null;
  try {
    const parsed = JSON.parse(raw);
    const result = schema.safeParse(parsed);
    return result.success ? result.data : null;
  } catch {
    return null;
  }
}
