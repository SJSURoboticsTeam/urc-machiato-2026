/**
 * SensorDashboardContainer: heading, Live/Mock label, onSensorDataChange callback.
 */
import React from 'react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen, waitFor } from '@testing-library/react';
import { SensorDashboardContainer } from '../SensorDashboardContainer';

const mockUseROS = vi.fn(() => ({ isConnected: false, ros: null }));
vi.mock('../../../../hooks/useROS', () => ({ useROS: (...args) => mockUseROS(...args) }));

// Avoid ROSLIB.Topic in useEffect when connected
vi.mock('../../../../utils/rosbridge', () => ({
  default: {
    Topic: vi.fn().mockImplementation(() => ({
      subscribe: vi.fn(),
      unsubscribe: vi.fn()
    }))
  }
}));

describe('SensorDashboardContainer', () => {
  beforeEach(() => {
    vi.clearAllMocks();
    mockUseROS.mockReturnValue({ isConnected: false, ros: null });
  });

  it('renders heading and Mock data label when disconnected', () => {
    render(<SensorDashboardContainer />);
    expect(screen.getByText('Sensor Health Dashboard')).toBeInTheDocument();
    expect(screen.getByText('Mock data')).toBeInTheDocument();
  });

  it('renders Status grid and Time series sections', () => {
    render(<SensorDashboardContainer />);
    expect(screen.getByText('Status grid')).toBeInTheDocument();
    expect(screen.getByText('Time series')).toBeInTheDocument();
    expect(screen.getByText('Analytics')).toBeInTheDocument();
  });

  it('calls onSensorDataChange when sensor data is available', async () => {
    const onSensorDataChange = vi.fn();
    render(<SensorDashboardContainer onSensorDataChange={onSensorDataChange} />);
    await waitFor(() => {
      expect(onSensorDataChange).toHaveBeenCalled();
    }, { timeout: 5000 });
    expect(typeof onSensorDataChange.mock.calls[0][0]).toBe('object');
  });
});
