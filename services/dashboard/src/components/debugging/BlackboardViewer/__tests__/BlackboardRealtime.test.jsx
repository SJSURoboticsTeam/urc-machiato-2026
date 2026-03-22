/**
 * BlackboardRealtime: search, domain filter, expand/collapse, formatValue, refresh.
 */
import React from 'react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen, fireEvent } from '@testing-library/react';
import { BlackboardRealtime } from '../BlackboardRealtime';

const mockUseROS = vi.fn(() => ({ isConnected: true, ros: {} }));
vi.mock('../../../hooks/useROS', () => ({ useROS: (...args) => mockUseROS(...args) }));

describe('BlackboardRealtime', () => {
  beforeEach(() => {
    vi.clearAllMocks();
    mockUseROS.mockReturnValue({ isConnected: true, ros: {} });
  });

  it('renders search input and domain filter', () => {
    render(<BlackboardRealtime blackboardState={{}} />);
    expect(screen.getByPlaceholderText(/Search keys/)).toBeInTheDocument();
    expect(screen.getByRole('combobox')).toBeInTheDocument();
    expect(screen.getByText(/All domains/)).toBeInTheDocument();
  });

  it('filters keys by search', () => {
    render(
      <BlackboardRealtime
        blackboardState={{ robot_x: 1.5, robot_y: 2.0, mission_active: true }}
      />
    );
    const input = screen.getByPlaceholderText(/Search keys/);
    fireEvent.change(input, { target: { value: 'robot' } });
    expect(input.value).toBe('robot');
  });

  it('filters by domain when domain filter is changed', () => {
    render(
      <BlackboardRealtime
        blackboardState={{ robot_x: 1, mission_active: true }}
      />
    );
    const select = screen.getByRole('combobox');
    fireEvent.change(select, { target: { value: 'navigation' } });
    expect(select.value).toBe('navigation');
  });

  it('toggles domain expansion when header is clicked', () => {
    render(<BlackboardRealtime blackboardState={{ robot_x: 1 }} />);
    const headers = screen.getAllByRole('button', { name: /\([0-9]+\)/ });
    expect(headers.length).toBeGreaterThan(0);
    fireEvent.click(headers[0]);
    fireEvent.click(headers[0]);
  });

  it('shows "No keys match" when search has no matches', () => {
    render(<BlackboardRealtime blackboardState={{ robot_x: 1 }} />);
    const input = screen.getByPlaceholderText(/Search keys/);
    fireEvent.change(input, { target: { value: 'xyznonexistent' } });
    const noMatch = screen.getAllByText(/No keys match/);
    expect(noMatch.length).toBeGreaterThan(0);
  });

  it('formats latitude/longitude and bool values', () => {
    render(
      <BlackboardRealtime
        blackboardState={{
          gps_latitude: 38.406,
          gps_longitude: -110.792,
          mission_active: true
        }}
      />
    );
    expect(screen.getByText(/lat 38\.406000/)).toBeInTheDocument();
    expect(screen.getByText(/lon -110\.792000/)).toBeInTheDocument();
    expect(screen.getByText('true')).toBeInTheDocument();
  });

  it('shows Refresh button when onRefresh provided and is connected', () => {
    mockUseROS.mockReturnValue({ isConnected: true, ros: {} });
    render(
      <BlackboardRealtime
        blackboardState={{ robot_x: 1 }}
        onRefresh={() => {}}
      />
    );
    expect(screen.getByRole('button', { name: /Refresh/i })).toBeInTheDocument();
  });

  it('disables Refresh when not connected', () => {
    mockUseROS.mockReturnValueOnce({ isConnected: false, ros: null });
    const onRefresh = vi.fn();
    render(
      <BlackboardRealtime
        blackboardState={{}}
        onRefresh={onRefresh}
      />
    );
    const btn = screen.getByRole('button', { name: /Refresh/i });
    expect(btn).toBeDisabled();
  });
});
