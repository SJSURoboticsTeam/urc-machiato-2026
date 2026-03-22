/**
 * validationSchemas: parseAndValidate and schema behavior.
 */
import { describe, it, expect } from 'vitest';
import { parseAndValidate, missionStatusSchema } from '../validationSchemas';

describe('parseAndValidate', () => {
  it('returns null when raw is not a string', () => {
    expect(parseAndValidate(null, missionStatusSchema)).toBe(null);
    expect(parseAndValidate(undefined, missionStatusSchema)).toBe(null);
    expect(parseAndValidate(123, missionStatusSchema)).toBe(null);
    expect(parseAndValidate({}, missionStatusSchema)).toBe(null);
  });

  it('returns null when raw is invalid JSON', () => {
    expect(parseAndValidate('not json', missionStatusSchema)).toBe(null);
    expect(parseAndValidate('{', missionStatusSchema)).toBe(null);
  });

  it('returns null when parsed object fails schema', () => {
    expect(parseAndValidate('{"state": 123}', missionStatusSchema)).toBe(null);
  });

  it('returns parsed data when valid JSON passes schema', () => {
    const out = parseAndValidate('{"state": "IDLE"}', missionStatusSchema);
    expect(out).not.toBe(null);
    expect(out).toEqual({ state: 'IDLE' });
  });

  it('returns null when JSON.parse throws (invalid JSON)', () => {
    expect(parseAndValidate('undefined', missionStatusSchema)).toBe(null);
  });
});
