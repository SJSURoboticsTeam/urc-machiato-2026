import { describe, it, expect } from 'vitest';
import { render, screen } from '@testing-library/react';
import { CriticalStateStrip } from '../CriticalStateStrip';

describe('CriticalStateStrip', () => {
  it('renders connection status with icon and text', () => {
    render(<CriticalStateStrip isConnected={true} sensorData={{}} />);
    expect(screen.getByText('Connected')).toBeInTheDocument();
  });

  it('shows Disconnected when not connected', () => {
    render(<CriticalStateStrip isConnected={false} sensorData={{}} />);
    expect(screen.getByText('Disconnected')).toBeInTheDocument();
  });

  it('shows "Sensors (no data)" when sensorData is empty', () => {
    render(<CriticalStateStrip isConnected={false} sensorData={{}} />);
    expect(screen.getByText('Sensors (no data)')).toBeInTheDocument();
  });

  it('shows sensor summary when data present and all OK', () => {
    const now = Date.now();
    const data = {
      imu: { confidence: 0.9, lastUpdateMs: now - 500 },
      gps: { confidence: 0.85, lastUpdateMs: now - 500 }
    };
    render(<CriticalStateStrip isConnected={true} sensorData={data} />);
    expect(screen.getByText(/Sensors 2\/9 OK/)).toBeInTheDocument();
  });

  it('shows alert count when non-zero', () => {
    render(
      <CriticalStateStrip isConnected={true} sensorData={{}} alertCount={3} />
    );
    expect(screen.getByText('3 alerts')).toBeInTheDocument();
  });

  it('has role="status" and aria-live for live region', () => {
    render(<CriticalStateStrip isConnected={true} sensorData={{}} />);
    const status = screen.getByRole('status');
    expect(status).toHaveAttribute('aria-live', 'polite');
  });

  it('includes aria-label describing connection and sensors', () => {
    render(<CriticalStateStrip isConnected={true} sensorData={{}} />);
    const status = screen.getByRole('status');
    expect(status.getAttribute('aria-label')).toMatch(/Connection connected/i);
  });
});
