import { describe, it, expect } from 'vitest';
import {
  getSimulationDataForScenario,
  getScenarioNames,
  type SimulationData
} from '../mockScenarios';

describe('mockScenarios', () => {
  describe('getScenarioNames', () => {
    it('returns all scenario names', () => {
      const names = getScenarioNames();
      expect(names).toContain('idle');
      expect(names).toContain('mission_active');
      expect(names).toContain('sensor_degraded');
      expect(names.length).toBeGreaterThanOrEqual(3);
    });
  });

  describe('getSimulationDataForScenario', () => {
    it('throws for unknown scenario', () => {
      expect(() => getSimulationDataForScenario('unknown', 0)).toThrow('Unknown scenario');
    });

    it('generates deterministic data for same time (idle)', () => {
      const a = getSimulationDataForScenario('idle', 10);
      const b = getSimulationDataForScenario('idle', 10);
      expect(a.current_state).toBe(b.current_state);
      expect(a.battery).toBe(b.battery);
      expect(a.gps.position.lat).toBe(b.gps.position.lat);
      expect(a.gps.position.lon).toBe(b.gps.position.lon);
    });

    it('generates deterministic data for same time (mission_active)', () => {
      const a = getSimulationDataForScenario('mission_active', 5);
      const b = getSimulationDataForScenario('mission_active', 5);
      expect(a.battery).toBe(b.battery);
      expect(a.current_state).toBe(b.current_state);
      expect(a.mission_data?.progress).toBe(b.mission_data?.progress);
    });

    it('generates deterministic data for same time (sensor_degraded)', () => {
      const a = getSimulationDataForScenario('sensor_degraded', 3);
      const b = getSimulationDataForScenario('sensor_degraded', 3);
      expect(a.current_state).toBe(b.current_state);
      expect(a.battery).toBe(b.battery);
      expect(a.network_status.can).toBe(b.network_status.can);
    });

    it('idle scenario has expected shape and values', () => {
      const data = getSimulationDataForScenario('idle', 0) as SimulationData;
      expect(data.current_state).toBe('IDLE');
      expect(data.battery).toBe(85);
      expect(data.gps.position).toEqual({ lat: 37.7749, lon: -122.4194 });
      expect(data.speed).toBe(0);
      expect(data.network_status.ros).toBe('connected');
      expect(data.network_status.can).toBe('connected');
      expect(data.mission_data).toBeUndefined();
    });

    it('mission_active scenario has mission data and draining battery', () => {
      const data0 = getSimulationDataForScenario('mission_active', 0);
      const data50 = getSimulationDataForScenario('mission_active', 50);
      expect(data0.current_state).toBe('EXECUTING');
      expect(data0.mission_data?.id).toBe('mission_001');
      expect(data0.mission_data?.status).toBe('active');
      expect(data50.battery).toBeLessThanOrEqual(data0.battery);
      expect(data50.mission_data?.progress).toBeGreaterThanOrEqual(data0.mission_data?.progress ?? 0);
    });

    it('sensor_degraded scenario has paused state and degraded CAN', () => {
      const data = getSimulationDataForScenario('sensor_degraded', 0);
      expect(data.current_state).toBe('PAUSED');
      expect(data.battery).toBe(60);
      expect(data.network_status.can).toBe('degraded');
      expect(data.mission_data?.status).toBe('paused');
      expect(data.mission_data?.error).toContain('Sensor');
    });

    it('returns valid telemetry-shaped fields', () => {
      const data = getSimulationDataForScenario('idle', 0);
      expect(typeof data.timestamp).toBe('number');
      expect(data.imu).toHaveProperty('accel_x');
      expect(data.imu).toHaveProperty('accel_z', 9.81);
      expect(data.environment).toHaveProperty('temperature');
      expect(data.gps).toHaveProperty('satellites');
      expect(data.gps).toHaveProperty('hdop');
    });
  });
});
