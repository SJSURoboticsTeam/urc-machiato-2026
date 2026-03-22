import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen, waitFor, act } from '@testing-library/react';
import { AppProviders } from '../AppProviders';
import { useROSContext } from '../ROSContext';
import { useStateMachineContext } from '../StateMachineContext';
import { useTelemetryContext } from '../TelemetryContext';
import { useUIContext } from '../UIContext';
import { useStateMachine } from '../../hooks/useStateMachine';

vi.mock('../../hooks/useROS', () => ({
  useROS: () => ({
    ros: { isConnected: true },
    isConnected: true,
    connectionStatus: 'connected'
  })
}));

const mockRequestStateTransition = vi.fn();
vi.mock('../../hooks/useStateMachine', () => ({
  useStateMachine: vi.fn(() => ({
    currentState: 'IDLE',
    currentSubstate: null,
    requestStateTransition: mockRequestStateTransition,
    isTransitioning: false
  }))
}));

function ROSConsumer() {
  const ctx = useROSContext();
  return (
    <div data-testid="ros-consumer">
      <span data-ros-connected={ctx.isConnected} />
      <span data-connection-status={ctx.connectionStatus} />
      {ctx.ros != null ? 'has-ros' : 'no-ros'}
    </div>
  );
}

function StateMachineConsumer() {
  const ctx = useStateMachineContext();
  return (
    <div data-testid="sm-consumer">
      <span data-state={ctx.currentState} />
      <span data-transitioning={ctx.isTransitioning} />
      {typeof ctx.requestStateTransition === 'function' ? 'has-request' : 'no-request'}
      {typeof ctx.getStateBadge === 'function' ? 'has-badge' : 'no-badge'}
      {typeof ctx.handleEmergencyStop === 'function' ? 'has-estop' : 'no-estop'}
    </div>
  );
}

function TelemetryConsumer() {
  const ctx = useTelemetryContext();
  return (
    <div data-testid="tel-consumer">
      <span data-has-telemetry={ctx.telemetry != null} />
      <span data-has-system-status={ctx.systemStatus != null} />
      {typeof ctx.setTelemetry === 'function' ? 'has-setTelemetry' : 'no-setTelemetry'}
    </div>
  );
}

function UIConsumer() {
  const ctx = useUIContext();
  return (
    <div data-testid="ui-consumer">
      <span data-online={ctx.isOnline} />
      {Array.isArray(ctx.alerts) ? 'has-alerts' : 'no-alerts'}
      {typeof ctx.setActiveMission === 'function' ? 'has-setMission' : 'no-setMission'}
      {typeof ctx.setRunningTests === 'function' ? 'has-setRunningTests' : 'no-setRunningTests'}
      {typeof ctx.cacheDataForOffline === 'function' ? 'has-cache' : 'no-cache'}
    </div>
  );
}

describe('Context providers', () => {
  beforeEach(() => {
    vi.clearAllMocks();
  });

  it('ROSContextProvider supplies ros, isConnected, connectionStatus', () => {
    render(
      <AppProviders>
        <ROSConsumer />
      </AppProviders>
    );
    const el = screen.getByTestId('ros-consumer');
    expect(el).toBeInTheDocument();
    expect(el.querySelector('[data-ros-connected="true"]')).toBeInTheDocument();
    expect(el.querySelector('[data-connection-status="connected"]')).toBeInTheDocument();
    expect(el).toHaveTextContent('has-ros');
  });

  it('StateMachineContextProvider supplies state machine API', () => {
    render(
      <AppProviders>
        <StateMachineConsumer />
      </AppProviders>
    );
    const el = screen.getByTestId('sm-consumer');
    expect(el).toBeInTheDocument();
    expect(el.querySelector('[data-state="IDLE"]')).toBeInTheDocument();
    expect(el).toHaveTextContent('has-request');
    expect(el).toHaveTextContent('has-badge');
    expect(el).toHaveTextContent('has-estop');
  });

  it('TelemetryContextProvider supplies telemetry, setTelemetry, systemStatus', () => {
    render(
      <AppProviders>
        <TelemetryConsumer />
      </AppProviders>
    );
    const el = screen.getByTestId('tel-consumer');
    expect(el).toBeInTheDocument();
    expect(el.querySelector('[data-has-telemetry="true"]')).toBeInTheDocument();
    expect(el.querySelector('[data-has-system-status="true"]')).toBeInTheDocument();
    expect(el).toHaveTextContent('has-setTelemetry');
  });

  it('UIContextProvider supplies alerts, activeMission, runningTests, isOnline, offline helpers', () => {
    render(
      <AppProviders>
        <UIConsumer />
      </AppProviders>
    );
    const el = screen.getByTestId('ui-consumer');
    expect(el).toBeInTheDocument();
    expect(el).toHaveTextContent('has-alerts');
    expect(el).toHaveTextContent('has-setMission');
    expect(el).toHaveTextContent('has-setRunningTests');
    expect(el).toHaveTextContent('has-cache');
  });

  it('getStateBadge returns fallback for unknown state', () => {
    vi.mocked(useStateMachine).mockReturnValueOnce({
      currentState: 'UNKNOWN_STATE',
      currentSubstate: null,
      requestStateTransition: vi.fn(),
      isTransitioning: false
    });
    function BadgeConsumer() {
      const { getStateBadge } = useStateMachineContext();
      const badge = getStateBadge();
      return <span data-testid="badge-label">{badge.label}</span>;
    }
    render(
      <AppProviders>
        <BadgeConsumer />
      </AppProviders>
    );
    expect(screen.getByTestId('badge-label')).toHaveTextContent('UNKNOWN_STATE');
  });

  it('handleEmergencyStop calls requestStateTransition and catches errors', async () => {
    mockRequestStateTransition.mockRejectedValueOnce(new Error('Service unavailable'));
    function EstopConsumer() {
      const { handleEmergencyStop } = useStateMachineContext();
      return (
        <button type="button" onClick={() => handleEmergencyStop()}>
          E-Stop
        </button>
      );
    }
    const consoleSpy = vi.spyOn(console, 'error').mockImplementation(() => {});
    render(
      <AppProviders>
        <EstopConsumer />
      </AppProviders>
    );
    screen.getByRole('button', { name: /E-Stop/i }).click();
    await waitFor(() => {
      expect(mockRequestStateTransition).toHaveBeenCalledWith('SAFETY', 'Emergency stop activated from UI');
    });
    consoleSpy.mockRestore();
  });

  it('UIContext derives alerts from low battery and high GPS hdop', async () => {
    function AlertsConsumer() {
      const { setTelemetry, setSystemStatus } = useTelemetryContext();
      const { alerts } = useUIContext();
      return (
        <div>
          <span data-testid="alert-count">{alerts.length}</span>
          <span data-testid="alert-ids">{alerts.map((a) => a.id).join(',')}</span>
          <button
            type="button"
            onClick={() => {
              setTelemetry((p) => ({ ...p, battery: 10, gps: { ...(p?.gps || {}), hdop: 3 } }));
              setSystemStatus((s) => ({ ...s, navigation: 'degraded' }));
            }}
          >
            Trigger alerts
          </button>
        </div>
      );
    }
    render(
      <AppProviders>
        <AlertsConsumer />
      </AppProviders>
    );
    await act(async () => { screen.getByRole('button', { name: /Trigger alerts/i }).click(); });
    await waitFor(() => {
      const count = Number(screen.getByTestId('alert-count').textContent);
      expect(count).toBeGreaterThanOrEqual(0);
    }, { timeout: 500 });
    await waitFor(() => {
      const ids = screen.getByTestId('alert-ids').textContent;
      expect(ids).toMatch(/low_battery|nav_degraded|gps_drift/);
    }, { timeout: 300 });
  });

  it('UIContext cacheDataForOffline sets offlineData when offline', async () => {
    const origOnLine = Object.getOwnPropertyDescriptor(navigator, 'onLine');
    Object.defineProperty(navigator, 'onLine', { value: false, configurable: true });
    function OfflineConsumer() {
      const { offlineData, cacheDataForOffline } = useUIContext();
      return (
        <div>
          <span data-testid="offline-data">{offlineData ? JSON.stringify(offlineData) : 'null'}</span>
          <button type="button" onClick={() => cacheDataForOffline({ key: 'cached' })}>Cache</button>
        </div>
      );
    }
    render(
      <AppProviders>
        <OfflineConsumer />
      </AppProviders>
    );
    expect(screen.getByTestId('offline-data')).toHaveTextContent('null');
    await act(async () => { screen.getByRole('button', { name: /Cache/i }).click(); });
    await waitFor(() => {
      const data = screen.getByTestId('offline-data').textContent;
      expect(data).not.toBe('null');
      expect(data).toContain('cached');
    });
    if (origOnLine) Object.defineProperty(navigator, 'onLine', origOnLine);
    else delete navigator.onLine;
  });

  it('UIContext setActiveMission and setRunningTests update state', async () => {
    function MissionConsumer() {
      const { activeMission, setActiveMission, runningTests, setRunningTests } = useUIContext();
      return (
        <div>
          <span data-testid="mission">{activeMission ?? 'none'}</span>
          <span data-testid="tests">{String(runningTests)}</span>
          <button type="button" onClick={() => setActiveMission('sample_return')}>Set mission</button>
          <button type="button" onClick={() => setRunningTests(2)}>Set tests</button>
        </div>
      );
    }
    render(
      <AppProviders>
        <MissionConsumer />
      </AppProviders>
    );
    expect(screen.getByTestId('mission')).toHaveTextContent('none');
    await act(async () => { screen.getByRole('button', { name: /Set mission/i }).click(); });
    await waitFor(() => expect(screen.getByTestId('mission')).toHaveTextContent('sample_return'));
    await act(async () => { screen.getByRole('button', { name: /Set tests/i }).click(); });
    await waitFor(() => expect(screen.getByTestId('tests')).toHaveTextContent('2'));
  });

  describe('hooks throw when used outside provider', () => {
    it('useROSContext throws outside ROSContextProvider', () => {
      expect(() => render(<ROSConsumer />)).toThrow(
        'useROSContext must be used within ROSContextProvider'
      );
    });
    it('useStateMachineContext throws outside StateMachineContextProvider', () => {
      expect(() => render(<StateMachineConsumer />)).toThrow(
        'useStateMachineContext must be used within StateMachineContextProvider'
      );
    });
    it('useTelemetryContext throws outside TelemetryContextProvider', () => {
      expect(() => render(<TelemetryConsumer />)).toThrow(
        'useTelemetryContext must be used within TelemetryContextProvider'
      );
    });
    it('useUIContext throws outside UIContextProvider', () => {
      expect(() => render(<UIConsumer />)).toThrow(
        'useUIContext must be used within UIContextProvider'
      );
    });
  });
});
