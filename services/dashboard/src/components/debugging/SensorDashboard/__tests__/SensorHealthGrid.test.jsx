import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { render, screen } from '@testing-library/react';
import { SensorHealthGrid } from '../SensorHealthGrid';
import { SENSOR_IDS, SENSOR_LABELS } from '../../constants';

describe('SensorHealthGrid', () => {
  beforeEach(() => {
    vi.useFakeTimers();
    vi.setSystemTime(new Date('2025-02-07T12:00:00.000Z'));
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  it('renders all 9 sensor cells', () => {
    render(<SensorHealthGrid sensorData={{}} stalenessSec={2} />);
    SENSOR_IDS.forEach((id) => {
      expect(screen.getByText(SENSOR_LABELS[id] || id)).toBeInTheDocument();
    });
  });

  it('has region label for accessibility', () => {
    render(<SensorHealthGrid sensorData={{}} />);
    const region = screen.getByRole('region', { name: /sensor health status grid/i });
    expect(region).toBeInTheDocument();
  });

  it('shows OK status and label when confidence is high', () => {
    const data = { imu: { confidence: 0.9, lastUpdateMs: Date.now() - 500, healthScore: 0.9 } };
    render(<SensorHealthGrid sensorData={data} stalenessSec={2} />);
    expect(screen.getByLabelText(/IMU: OK,/i)).toBeInTheDocument();
  });

  it('shows Stale status when last update exceeds staleness threshold', () => {
    const data = { imu: { confidence: 0.9, lastUpdateMs: Date.now() - 5000, healthScore: 0.9 } };
    render(<SensorHealthGrid sensorData={data} stalenessSec={2} />);
    expect(screen.getByLabelText(/IMU: Stale,/i)).toBeInTheDocument();
  });

  it('shows Degraded when confidence is between 0.5 and 0.8', () => {
    const data = { gps: { confidence: 0.6, lastUpdateMs: Date.now(), healthScore: 0.6 } };
    render(<SensorHealthGrid sensorData={data} stalenessSec={2} />);
    expect(screen.getByLabelText(/GPS: Degraded,/i)).toBeInTheDocument();
  });

  it('shows Low when confidence is below 0.5', () => {
    const data = { camera: { confidence: 0.3, lastUpdateMs: Date.now(), healthScore: 0.3 } };
    render(<SensorHealthGrid sensorData={data} stalenessSec={2} />);
    expect(screen.getByLabelText(/Camera: Low,/i)).toBeInTheDocument();
  });

  it('uses progressbar role and aria-valuenow for confidence', () => {
    const data = { imu: { confidence: 0.75, lastUpdateMs: Date.now(), healthScore: 0.75 } };
    render(<SensorHealthGrid sensorData={data} />);
    const bars = screen.getAllByRole('progressbar');
    expect(bars.length).toBeGreaterThanOrEqual(1);
    expect(bars.some((el) => el.getAttribute('aria-valuenow') === '75')).toBe(true);
  });

  it('handles empty sensorData without throwing', () => {
    expect(() => render(<SensorHealthGrid sensorData={{}} />)).not.toThrow();
  });
});
