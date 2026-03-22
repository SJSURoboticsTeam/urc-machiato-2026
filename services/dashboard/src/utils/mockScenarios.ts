/**
 * Deterministic mock scenarios for dashboard testing.
 * Same elapsedSeconds produces same output (no random values).
 */

import type { Telemetry } from '../types/telemetry';

export interface SimulationData {
  /** Telemetry shape consumed by TelemetryContext */
  battery: number;
  gps: Telemetry['gps'];
  speed: number;
  temperature: number;
  timestamp: number;
  imu: Telemetry['imu'];
  environment: Telemetry['environment'];
  /** State machine state */
  current_state: string;
  /** Network/CAN status for useNetworkData */
  network_status: { ros: string; can: string };
  /** CAN sensor payload (e.g. for /can/sensor_data) */
  can_data: Record<string, unknown>;
  /** Mission data when scenario has active/paused mission */
  mission_data?: {
    id: string;
    status: string;
    progress: number;
    waypoints_completed?: number;
    error?: string;
  };
}

const BASE_GPS = { lat: 37.7749, lon: -122.4194 };
const BASE_ALT = 10;

function makeGps(lat: number, lon: number, satellites = 12, hdop = 1.2): Telemetry['gps'] {
  return {
    satellites,
    hdop,
    position: { lat, lon }
  };
}

function makeImu(accelZ = 9.81): Telemetry['imu'] {
  return {
    accel_x: 0,
    accel_y: 0,
    accel_z: accelZ,
    gyro_x: 0,
    gyro_y: 0,
    gyro_z: 0
  };
}

function makeEnvironment(temp = 25, humidity = 30): Telemetry['environment'] {
  return {
    temperature: temp,
    humidity,
    visibility: 1.0,
    dust_density: 0.0
  };
}

/** Idle: robot sitting, all systems nominal. Deterministic. */
function idleData(elapsedSeconds: number): SimulationData {
  return {
    current_state: 'IDLE',
    battery: 85,
    gps: makeGps(BASE_GPS.lat, BASE_GPS.lon),
    speed: 0.0,
    temperature: 28,
    timestamp: Date.now(),
    imu: makeImu(),
    environment: makeEnvironment(),
    network_status: { ros: 'connected', can: 'connected' },
    can_data: { status: 'normal', sensors: 'all_ok' }
  };
}

/** Mission active: battery drains, GPS moves along path. Deterministic. */
function missionActiveData(elapsedSeconds: number): SimulationData {
  const battery = Math.max(65, 70 - elapsedSeconds * 0.1);
  const latOffset = Math.sin(elapsedSeconds * 0.1) * 0.0001;
  const progress = Math.min(100, elapsedSeconds * 2);
  const waypointsCompleted = Math.floor(elapsedSeconds / 10);

  return {
    current_state: 'EXECUTING',
    battery,
    gps: makeGps(BASE_GPS.lat + latOffset, BASE_GPS.lon),
    speed: 0.5,
    temperature: 28,
    timestamp: Date.now(),
    imu: makeImu(),
    environment: makeEnvironment(),
    network_status: { ros: 'connected', can: 'connected' },
    can_data: { status: 'normal', sensors: 'all_ok' },
    mission_data: {
      id: 'mission_001',
      status: 'active',
      progress,
      waypoints_completed: waypointsCompleted
    }
  };
}

/** Sensor degraded: some sensors failing, mission paused. Deterministic (use sin for drift). */
function sensorDegradedData(elapsedSeconds: number): SimulationData {
  const driftLat = Math.sin(elapsedSeconds) * 0.0001;

  return {
    current_state: 'PAUSED',
    battery: 60,
    gps: makeGps(BASE_GPS.lat + driftLat, BASE_GPS.lon, 8, 2.5),
    speed: 0.0,
    temperature: 32,
    timestamp: Date.now(),
    imu: makeImu(9.81),
    environment: makeEnvironment(32, 35),
    network_status: { ros: 'connected', can: 'degraded' },
    can_data: { status: 'degraded', imu: null, compass: null },
    mission_data: {
      id: 'mission_002',
      status: 'paused',
      progress: 45,
      error: 'Sensor timeout: IMU, Compass'
    }
  };
}

const SCENARIOS: Record<string, (t: number) => SimulationData> = {
  idle: idleData,
  mission_active: missionActiveData,
  sensor_degraded: sensorDegradedData
};

/**
 * Get deterministic simulation data for a scenario at a given elapsed time.
 * @param scenario - Scenario name: 'idle' | 'mission_active' | 'sensor_degraded'
 * @param elapsedSeconds - Seconds since scenario start (deterministic input)
 */
export function getSimulationDataForScenario(
  scenario: string,
  elapsedSeconds: number
): SimulationData {
  const fn = SCENARIOS[scenario];
  if (!fn) {
    throw new Error(`Unknown scenario: ${scenario}`);
  }
  return fn(elapsedSeconds);
}

export function getScenarioNames(): string[] {
  return Object.keys(SCENARIOS);
}
