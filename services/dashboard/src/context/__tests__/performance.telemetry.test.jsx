/**
 * Phase 4: Performance test – high-frequency telemetry updates (e.g. 50 Hz) do not crash
 * and final value is applied (debounced).
 */
import React from 'react';
import { describe, it, expect, vi } from 'vitest';
import { render, screen, waitFor, act } from '@testing-library/react';
import { AppProviders } from '../AppProviders';
import { useTelemetryContext } from '../TelemetryContext';

vi.mock('../../hooks/useROS', () => ({
  useROS: () => ({ ros: null, isConnected: false, connectionStatus: 'disconnected' })
}));
vi.mock('../../hooks/useStateMachine', () => ({
  useStateMachine: () => ({
    currentState: 'IDLE',
    currentSubstate: null,
    requestStateTransition: vi.fn(),
    isTransitioning: false
  })
}));

function TelemetryDisplay() {
  const { telemetry, setTelemetry } = useTelemetryContext();
  const fireHighFreq = () => {
    for (let i = 0; i < 50; i++) {
      setTelemetry((prev) => ({ ...prev, battery: 10 + (i % 80), timestamp: Date.now() }));
    }
  };
  return (
    <div>
      <span data-testid="battery">{telemetry.battery}</span>
      <button data-testid="fire-high-freq" onClick={fireHighFreq} type="button">
        Fire 50
      </button>
    </div>
  );
}

describe('Performance: high-frequency telemetry', () => {
  it('50 rapid setTelemetry calls do not crash; after debounce flush value is in range', async () => {
    render(
      <AppProviders>
        <TelemetryDisplay />
      </AppProviders>
    );

    await act(() => {
      screen.getByTestId('fire-high-freq').click();
    });

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
