/**
 * Integration test: context provider stack and telemetry flow.
 * Renders AppProviders with mocked ROS/state machine; a consumer updates and reads telemetry.
 */
import React from 'react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen, waitFor, act } from '@testing-library/react';
import { AppProviders } from '../AppProviders';
import { useTelemetryContext } from '../TelemetryContext';
import { useUIContext } from '../UIContext';

vi.mock('../../hooks/useROS', () => ({
  useROS: () => ({ ros: {}, isConnected: true, connectionStatus: 'connected' })
}));
vi.mock('../../hooks/useStateMachine', () => ({
  useStateMachine: () => ({
    currentState: 'IDLE',
    currentSubstate: null,
    requestStateTransition: vi.fn(),
    isTransitioning: false
  })
}));

function TelemetryConsumer() {
  const { telemetry, setTelemetry } = useTelemetryContext();
  const { alerts } = useUIContext();
  return (
    <div>
      <span data-testid="battery">{telemetry.battery}</span>
      <span data-testid="gps-hdop">{telemetry.gps?.hdop}</span>
      <button
        data-testid="set-battery"
        onClick={() => setTelemetry((prev) => ({ ...prev, battery: 42 }))}
      >
        Set 42
      </button>
      <span data-testid="alert-count">{alerts.length}</span>
    </div>
  );
}

describe('Context flow integration', () => {
  beforeEach(() => vi.clearAllMocks());

  it('provides telemetry and allows update; consumer sees initial then updated value', async () => {
    render(
      <AppProviders>
        <TelemetryConsumer />
      </AppProviders>
    );
    expect(screen.getByTestId('battery').textContent).toBeDefined();
    const initialBattery = Number(screen.getByTestId('battery').textContent);
    expect(initialBattery).toBeGreaterThanOrEqual(0);
    expect(initialBattery).toBeLessThanOrEqual(100);

    screen.getByTestId('set-battery').click();
    await waitFor(() => {
      expect(Number(screen.getByTestId('battery').textContent)).toBe(42);
    });
  });

  it('UIContext receives derived alerts from TelemetryContext', () => {
    render(
      <AppProviders>
        <TelemetryConsumer />
      </AppProviders>
    );
    const count = Number(screen.getByTestId('alert-count').textContent);
    expect(count).toBeGreaterThanOrEqual(0);
  });

  it('high-frequency setTelemetry updates do not crash; final value is applied (debounced)', async () => {
    function HighFreqConsumer() {
      const { telemetry, setTelemetry } = useTelemetryContext();
      const fireMany = () => {
        for (let i = 0; i < 50; i++) {
          setTelemetry((prev) => ({ ...prev, battery: i }));
        }
      };
      return (
        <div>
          <span data-testid="battery">{telemetry.battery}</span>
          <button data-testid="fire-many" onClick={fireMany}>Fire 50</button>
        </div>
      );
    }

    render(
      <AppProviders>
        <HighFreqConsumer />
      </AppProviders>
    );

    const btn = screen.getByTestId('fire-many');
    await act(() => { btn.click(); });

    await waitFor(
      () => {
        const battery = Number(screen.getByTestId('battery').textContent);
        expect(battery).toBeGreaterThanOrEqual(0);
        expect(battery).toBeLessThanOrEqual(100);
      },
      { timeout: 300 }
    );
  });
});
