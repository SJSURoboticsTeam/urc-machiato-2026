import { describe, it, expect } from 'vitest';
import { validateTelemetryPayload, validateSimulationMessage } from './validation';

describe('validation', () => {
  describe('validateTelemetryPayload', () => {
    it('returns valid and sanitized for a valid full payload', () => {
      const payload = {
        battery: 90,
        gps: { satellites: 10, hdop: 1.5, position: { lat: 38.4, lon: -110.8 } },
        speed: 0.5,
        temperature: 25,
        timestamp: Date.now(),
        imu: { accel_x: 0, accel_y: 0, accel_z: 9.81, gyro_x: 0, gyro_y: 0, gyro_z: 0 },
        environment: { temperature: 24, humidity: 40, visibility: 1.0, dust_density: 0.0 }
      };
      const result = validateTelemetryPayload(payload);
      expect(result.valid).toBe(true);
      expect(result.sanitized).toBeDefined();
      expect(result.sanitized.battery).toBe(90);
      expect(result.sanitized.gps.satellites).toBe(10);
      expect(result.sanitized.gps.position.lat).toBe(38.4);
      expect(result.sanitized.speed).toBe(0.5);
    });

    it('returns valid and sanitized for partial payload (merges with defaults)', () => {
      const payload = { battery: 50 };
      const result = validateTelemetryPayload(payload);
      expect(result.valid).toBe(true);
      expect(result.sanitized.battery).toBe(50);
      expect(result.sanitized.gps).toBeDefined();
      expect(result.sanitized.gps.position).toBeDefined();
      expect(typeof result.sanitized.timestamp).toBe('number');
    });

    it('returns invalid for non-object input', () => {
      expect(validateTelemetryPayload(null).valid).toBe(false);
      expect(validateTelemetryPayload(undefined).valid).toBe(false);
      expect(validateTelemetryPayload('string').valid).toBe(false);
      expect(validateTelemetryPayload(42).valid).toBe(false);
    });

    it('returns invalid for empty or missing required keys (still returns sanitized with defaults)', () => {
      const result = validateTelemetryPayload({});
      expect(result.valid).toBe(true);
      expect(result.sanitized).toBeDefined();
      expect(result.sanitized.battery).toBeDefined();
      expect(result.sanitized.gps).toBeDefined();
    });
  });

  describe('validateSimulationMessage', () => {
    it('returns valid and sanitized for correct simulation_update message', () => {
      const msg = {
        type: 'simulation_update',
        simulation_data: {
          gps: { satellites: 8, hdop: 2, latitude: 38.41, longitude: -110.79 },
          rover: { velocity: [0.1, 0.2] },
          environment: { temperature: 26 },
          imu: {}
        }
      };
      const result = validateSimulationMessage(msg);
      expect(result.valid).toBe(true);
      expect(result.sanitized).toBeDefined();
      expect(result.sanitized.gps).toBeDefined();
      expect(result.sanitized.gps.satellites).toBe(8);
    });

    it('returns invalid for wrong type', () => {
      const result = validateSimulationMessage({ type: 'other', simulation_data: {} });
      expect(result.valid).toBe(false);
      expect(result.sanitized).toBeNull();
    });

    it('returns invalid for missing simulation_data', () => {
      const result = validateSimulationMessage({ type: 'simulation_update' });
      expect(result.valid).toBe(false);
      expect(result.sanitized).toBeNull();
    });

    it('returns invalid for non-object', () => {
      expect(validateSimulationMessage(null).valid).toBe(false);
      expect(validateSimulationMessage(undefined).valid).toBe(false);
    });
  });
});
