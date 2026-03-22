/**
 * Error scenarios: invalid messages are rejected by validation and do not crash.
 */
import { describe, it, expect } from 'vitest';
import { validateTelemetryPayload, validateSimulationMessage } from '../validation';

describe('Validation error scenarios', () => {
  it('rejects null telemetry and returns safe default sanitized', () => {
    const result = validateTelemetryPayload(null);
    expect(result.valid).toBe(false);
    expect(result.sanitized).toBeDefined();
    expect(result.sanitized.battery).toBeDefined();
    expect(result.sanitized.gps).toBeDefined();
  });

  it('rejects invalid simulation message (wrong type) and returns null sanitized', () => {
    const result = validateSimulationMessage({ type: 'other', data: {} });
    expect(result.valid).toBe(false);
    expect(result.sanitized).toBeNull();
  });

  it('rejects malformed simulation message (missing simulation_data)', () => {
    const result = validateSimulationMessage({ type: 'simulation_update' });
    expect(result.valid).toBe(false);
    expect(result.sanitized).toBeNull();
  });

  it('rejects telemetry with out-of-range battery and returns default sanitized', () => {
    const result = validateTelemetryPayload({ battery: 150 });
    expect(result.valid).toBe(false);
    expect(result.sanitized.battery).toBeDefined();
  });
});
