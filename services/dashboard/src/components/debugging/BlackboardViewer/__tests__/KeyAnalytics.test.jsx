/**
 * KeyAnalytics: anomalies list, key statistics table, empty state.
 */
import React from 'react';
import { describe, it, expect } from 'vitest';
import { render, screen } from '@testing-library/react';
import { KeyAnalytics } from '../KeyAnalytics';

describe('KeyAnalytics', () => {
  it('renders key statistics section', () => {
    render(<KeyAnalytics />);
    expect(screen.getByText('Key statistics')).toBeInTheDocument();
    expect(screen.getByText(/No key statistics yet/)).toBeInTheDocument();
  });

  it('renders anomalies when provided', () => {
    render(
      <KeyAnalytics
        anomalies={[
          { key: 'battery_level', message: 'Low battery' },
          { key: 'robot_x', message: 'Out of range' }
        ]}
      />
    );
    expect(screen.getByText('Anomalies')).toBeInTheDocument();
    expect(screen.getByText(/battery_level: Low battery/)).toBeInTheDocument();
    expect(screen.getByText(/robot_x: Out of range/)).toBeInTheDocument();
  });

  it('does not render anomalies section when empty', () => {
    render(<KeyAnalytics anomalies={[]} />);
    expect(screen.queryByText('Anomalies')).not.toBeInTheDocument();
  });

  it('renders key stats table when keyStats provided', () => {
    render(
      <KeyAnalytics
        keyStats={{
          robot_x: { changeCount: 10, min: 0, max: 5.5 },
          battery_level: { changeCount: 2, min: 0.2, max: 1 }
        }}
      />
    );
    expect(screen.getByText('Key statistics')).toBeInTheDocument();
    expect(screen.queryByText(/No key statistics yet/)).not.toBeInTheDocument();
    expect(screen.getByText('robot_x')).toBeInTheDocument();
    expect(screen.getByText('10')).toBeInTheDocument();
    expect(screen.getByText('0.2000')).toBeInTheDocument();
    expect(screen.getByText('5.5000')).toBeInTheDocument();
  });

  it('shows – for missing min/max and changeCount', () => {
    render(
      <KeyAnalytics
        keyStats={{
          mission_active: {}
        }}
      />
    );
    const cells = screen.getAllByText('–');
    expect(cells.length).toBeGreaterThanOrEqual(1);
  });
});
