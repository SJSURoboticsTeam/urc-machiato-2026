/**
 * Shared types for ROS messages and dashboard ROS-related payloads.
 * Used for TypeScript migration and JSDoc. Validation via Zod in validationSchemas.
 */

export interface RosStringMessage {
  data: string;
}

/** Mission progress from /mission/progress */
export interface MissionProgressPayload {
  mission_id?: string;
  name?: string;
  progress?: number;
  current_task?: string;
  next_task?: string;
  eta?: string;
  waypoints?: number | string;
  samples?: number | string;
  analysis?: string;
}

/** Mission status from /mission/status */
export interface MissionStatusPayload {
  state?: string;
  mission_id?: string;
  error?: string;
}

/** CAN sensor message from /can/sensor_data */
export interface CanSensorPayload {
  sensor_type?: string;
  value?: number;
  lat?: number;
  lon?: number;
  temp?: number;
  status?: string;
  [key: string]: unknown;
}

/** WebSocket command update to network */
export interface CommandUpdatePayload {
  navigation?: unknown;
  motion?: unknown;
  left_motor?: unknown;
  right_motor?: unknown;
}
