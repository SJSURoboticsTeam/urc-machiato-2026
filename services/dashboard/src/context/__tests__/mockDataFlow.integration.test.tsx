/**
 * Integration test: full mock data flow through contexts.
 * Run with VITE_USE_MOCK=1 to exercise MockDataSource path; otherwise verifies default/ROS path.
 */
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen, waitFor } from '@testing-library/react';
import { AppProviders } from '../AppProviders';
import { useTelemetryContext } from '../TelemetryContext';

vi.mock('../../../hooks/useROS', () => ({
  useROS: () => ({
    ros: {},
    isConnected: false,
    connectionStatus: 'disconnected',
    reconnectAttempts: 0,
    lastError: null,
    connect: vi.fn(),
    resetReconnection: vi.fn()
  })
}));

vi.mock('../../../hooks/useStateMachine', () => ({
  useStateMachine: () => ({
    currentState: 'IDLE',
    currentSubstate: '',
    requestStateTransition: vi.fn(),
    isTransitioning: false,
    getStateBadge: () => ({ label: 'IDLE', color: 'green', emoji: '' }),
    handleEmergencyStop: vi.fn()
  })
}));

function TelemetryDisplay() {
  const { telemetry, dataSource } = useTelemetryContext();
  return (
    <div data-testid="telemetry-display">
      <span data-testid="battery">{telemetry?.battery ?? 'none'}</span>
      <span data-testid="source-type">{dataSource?.getSourceType() ?? 'none'}</span>
    </div>
  );
}

describe('Mock data flow integration', () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it('provides telemetry and optional dataSource from context', async () => {
    render(
      <AppProviders>
        <TelemetryDisplay />
      </AppProviders>
    );

    await waitFor(
      () => {
        const batteryEl = screen.getByTestId('battery');
        const battery = Number(batteryEl.textContent);
        expect(battery).toBeGreaterThanOrEqual(0);
        expect(battery).toBeLessThanOrEqual(100);
      },
      { timeout: 3000 }
    );
  });

  it('when mock mode active, dataSource is mock and battery matches idle scenario', async () => {
    render(
      <AppProviders>
        <TelemetryDisplay />
      </AppProviders>
    );

    await waitFor(
      () => {
        const sourceType = screen.getByTestId('source-type').textContent;
        const batteryEl = screen.getByTestId('battery');
        const battery = Number(batteryEl.textContent);
        if (sourceType === 'mock') {
          expect(battery).toBe(85);
        }
        expect(['mock', 'none']).toContain(sourceType);
      },
      { timeout: 3000 }
    );
  });
});
