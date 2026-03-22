import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { render, screen } from '@testing-library/react';
import { SensorAlerts } from '../SensorAlerts';

describe('SensorAlerts', () => {
  beforeEach(() => {
    vi.useFakeTimers();
    vi.setSystemTime(new Date('2025-02-07T12:00:00.000Z'));
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  it('shows no alerts message when sensorData is empty and no alerts', () => {
    render(<SensorAlerts sensorData={{}} alerts={[]} />);
    expect(screen.getByText(/no sensor alerts/i)).toBeInTheDocument();
  });

  it('derives stale alert when last update exceeds threshold', () => {
    const sensorData = {
      imu: { confidence: 0.9, lastUpdateMs: Date.now() - 4000 }
    };
    render(<SensorAlerts sensorData={sensorData} />);
    expect(screen.getByText(/IMU has not updated in \d+s/)).toBeInTheDocument();
  });

  it('derives low confidence alert when confidence below 0.5', () => {
    const sensorData = {
      gps: { confidence: 0.3, lastUpdateMs: Date.now() }
    };
    render(<SensorAlerts sensorData={sensorData} />);
    expect(screen.getByText(/GPS confidence low \(\d+%\)/)).toBeInTheDocument();
  });

  it('renders external alerts with correct severity styling', () => {
    render(
      <SensorAlerts
        sensorData={{}}
        alerts={[
          { id: 'ext-1', severity: 'error', message: 'Battery critical' },
          { id: 'ext-2', severity: 'warning', message: 'High temperature' }
        ]}
      />
    );
    expect(screen.getByText('Battery critical')).toBeInTheDocument();
    expect(screen.getByText('High temperature')).toBeInTheDocument();
  });

  it('handles empty sensorData and empty alerts', () => {
    render(<SensorAlerts />);
    expect(screen.getByText(/no sensor alerts/i)).toBeInTheDocument();
  });
});
