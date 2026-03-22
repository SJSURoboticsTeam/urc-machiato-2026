import { describe, it, expect, beforeEach, afterEach } from 'vitest';
import { MockDataSource } from '../MockDataSource';

describe('MockDataSource', () => {
  let source: MockDataSource;

  beforeEach(() => {
    source = new MockDataSource('idle', 100);
  });

  afterEach(async () => {
    await source.disconnect();
  });

  it('starts disconnected', () => {
    expect(source.getConnectionState().status).toBe('disconnected');
  });

  it('getSourceType returns mock', () => {
    expect(source.getSourceType()).toBe('mock');
  });

  it('connects without errors', async () => {
    await expect(source.connect()).resolves.not.toThrow();
    expect(source.getConnectionState().status).toBe('connected');
  });

  it('publishes data to telemetry subscribers', async () => {
    await source.connect();
    const received: unknown[] = [];
    source.subscribe('telemetry', (data) => received.push(data));

    await new Promise((r) => setTimeout(r, 250));
    expect(received.length).toBeGreaterThanOrEqual(2);
    expect((received[0] as Record<string, unknown>).battery).toBe(85);
    expect((received[0] as Record<string, unknown>).current_state).toBeUndefined();
    expect((received[0] as Record<string, unknown>).gps).toBeDefined();
  });

  it('publishes system/state to subscribers', async () => {
    await source.connect();
    const states: string[] = [];
    source.subscribe('system/state', (data) => states.push(data as string));

    await new Promise((r) => setTimeout(r, 150));
    expect(states.length).toBeGreaterThanOrEqual(1);
    expect(states).toContain('IDLE');
  });

  it('unsubscribe stops receiving updates', async () => {
    await source.connect();
    const received: unknown[] = [];
    const unsub = source.subscribe('telemetry', (data) => received.push(data));
    await new Promise((r) => setTimeout(r, 80));
    const countBefore = received.length;
    unsub();
    await new Promise((r) => setTimeout(r, 150));
    expect(received.length).toBe(countBefore);
  });

  it('disconnect clears state and subscriptions', async () => {
    await source.connect();
    source.subscribe('telemetry', () => {});
    await source.disconnect();
    expect(source.getConnectionState().status).toBe('disconnected');
  });

  it('setScenario switches scenario and resets time', async () => {
    await source.connect();
    const states: string[] = [];
    source.subscribe('system/state', (data) => states.push(data as string));
    await new Promise((r) => setTimeout(r, 120));
    expect(states).toContain('IDLE');
    source.setScenario('mission_active');
    await new Promise((r) => setTimeout(r, 120));
    expect(states).toContain('EXECUTING');
  });

  it('mission_active scenario publishes mission_data', async () => {
    const missionSource = new MockDataSource('mission_active', 100);
    await missionSource.connect();
    const missionUpdates: unknown[] = [];
    missionSource.subscribe('mission/status', (data) => missionUpdates.push(data));
    await new Promise((r) => setTimeout(r, 150));
    await missionSource.disconnect();
    expect(missionUpdates.length).toBeGreaterThanOrEqual(1);
    expect((missionUpdates[0] as Record<string, unknown>)?.id).toBe('mission_001');
  });
});
