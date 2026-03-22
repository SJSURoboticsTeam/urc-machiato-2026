/**
 * Shared type definitions for dashboard telemetry and ROS-related payloads.
 * Used for JSDoc and gradual TypeScript migration.
 */

export interface GpsPosition {
  lat: number;
  lon: number;
}

export interface GpsData {
  satellites: number;
  hdop: number;
  position: GpsPosition;
}

export interface ImuData {
  accel_x: number;
  accel_y: number;
  accel_z: number;
  gyro_x: number;
  gyro_y: number;
  gyro_z: number;
}

export interface EnvironmentData {
  temperature: number;
  humidity: number;
  visibility: number;
  dust_density: number;
}

export interface Telemetry {
  battery: number;
  gps: GpsData;
  speed: number;
  temperature: number;
  timestamp: number;
  imu: ImuData;
  environment: EnvironmentData;
}

export interface SystemStatus {
  safety?: string;
  navigation?: string;
  vision?: string;
  can?: string;
  websocket?: string;
}

export interface SimulationMessage {
  type: 'simulation_update';
  simulation_data: Record<string, unknown>;
}
