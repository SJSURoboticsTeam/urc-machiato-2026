/**
 * Error scenario tests: ROS disconnect, invalid messages, error boundary fallback.
 * Part A Phase 4 - assert error boundaries and validation prevent crashes.
 */
import React, { useState } from 'react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen, waitFor, act } from '@testing-library/react';
import { AppProviders } from '../AppProviders';
import { useTelemetryContext } from '../TelemetryContext';
import { SectionErrorBoundary } from '../../components/debugging/UnifiedDebugging/SectionErrorBoundary';

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

function TelemetryEditor() {
  const { telemetry, setTelemetry } = useTelemetryContext();
  return (
    <div>
      <span data-testid="battery">{telemetry.battery}</span>
      <button
        data-testid="set-invalid"
        onClick={() => setTelemetry({ invalid: 'payload', battery: 'not a number' })}
      >
        Set invalid
      </button>
      <button
        data-testid="set-valid"
        onClick={() => setTelemetry((prev) => ({ ...prev, battery: 50 }))}
      >
        Set valid
      </button>
    </div>
  );
}

describe('ROS error scenarios', () => {
  beforeEach(() => vi.clearAllMocks());

  it('invalid telemetry payload is rejected by validation; UI keeps previous value', async () => {
    render(
      <AppProviders>
        <TelemetryEditor />
      </AppProviders>
    );

    const batteryBefore = Number(screen.getByTestId('battery').textContent);
    expect(batteryBefore).toBeGreaterThanOrEqual(0);

    await act(() => {
      screen.getByTestId('set-invalid').click();
    });

    await waitFor(() => {}, { timeout: 200 });
    const batteryAfter = Number(screen.getByTestId('battery').textContent);
    expect(batteryAfter).toBe(batteryBefore);
  });

  it('valid partial update is applied', async () => {
    render(
      <AppProviders>
        <TelemetryEditor />
      </AppProviders>
    );

    await act(() => {
      screen.getByTestId('set-valid').click();
    });

    await waitFor(() => {
      expect(Number(screen.getByTestId('battery').textContent)).toBe(50);
    });
  });

  it('SectionErrorBoundary shows fallback when child throws', async () => {
    function Thrower() {
      throw new Error('Simulated crash');
    }

    render(
      <SectionErrorBoundary>
        <Thrower />
      </SectionErrorBoundary>
    );

    await waitFor(() => {
      expect(screen.getByText('Section unavailable')).toBeInTheDocument();
    });
    expect(screen.getByText(/This part of the dashboard failed to load/)).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /Retry/i })).toBeInTheDocument();
  });
});
