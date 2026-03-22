import React from 'react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen, fireEvent } from '@testing-library/react';
import { BlackboardHistory } from '../BlackboardHistory';

describe('BlackboardHistory', () => {
  const defaultProps = {
    historySnapshots: [],
    selectedTimeIndex: 0,
    onSelectTime: undefined,
    changePatterns: {},
    onExport: undefined
  };

  beforeEach(() => {
    vi.spyOn(console, 'error').mockImplementation(() => {});
  });

  it('renders timeline and export controls', () => {
    render(<BlackboardHistory {...defaultProps} />);
    expect(screen.getByText('Timeline')).toBeInTheDocument();
    expect(screen.getByRole('combobox')).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /Export/i })).toBeInTheDocument();
  });

  it('shows "No history snapshots" when empty', () => {
    render(<BlackboardHistory {...defaultProps} />);
    expect(screen.getByText(/No history snapshots/)).toBeInTheDocument();
  });

  it('renders snapshot timeline and state when history provided', () => {
    const snapshots = [
      { timestamp: Date.now() - 2000, state: { key1: 'a', key2: 1 } },
      { timestamp: Date.now() - 1000, state: { key1: 'b', key2: 2 } }
    ];
    render(
      <BlackboardHistory
        {...defaultProps}
        historySnapshots={snapshots}
        selectedTimeIndex={0}
      />
    );
    expect(screen.getByRole('slider')).toBeInTheDocument();
    expect(screen.getByText(/Time:/)).toBeInTheDocument();
    expect(screen.getByText(/"key1": "a"/)).toBeInTheDocument();
    expect(screen.getByText(/"key2": 1/)).toBeInTheDocument();
  });

  it('calls onSelectTime when slider changes', () => {
    const onSelectTime = vi.fn();
    const snapshots = [
      { timestamp: Date.now(), state: {} },
      { timestamp: Date.now(), state: {} }
    ];
    render(
      <BlackboardHistory
        {...defaultProps}
        historySnapshots={snapshots}
        selectedTimeIndex={0}
        onSelectTime={onSelectTime}
      />
    );
    fireEvent.change(screen.getByRole('slider'), { target: { value: '1' } });
    expect(onSelectTime).toHaveBeenCalledWith(1);
  });

  it('export format select toggles json/csv', () => {
    const snapshots = [{ timestamp: Date.now(), state: { x: 1 } }];
    render(
      <BlackboardHistory
        {...defaultProps}
        historySnapshots={snapshots}
      />
    );
    const select = screen.getByRole('combobox');
    expect(select).toHaveValue('json');
    fireEvent.change(select, { target: { value: 'csv' } });
    expect(select).toHaveValue('csv');
  });

  it('Export button with data and onExport calls onExport', () => {
    const onExport = vi.fn();
    const snapshots = [{ timestamp: Date.now(), state: { a: 1 } }];
    render(
      <BlackboardHistory
        {...defaultProps}
        historySnapshots={snapshots}
        onExport={onExport}
      />
    );
    fireEvent.click(screen.getByRole('button', { name: /Export/i }));
    expect(onExport).toHaveBeenCalled();
    const [data, format] = onExport.mock.calls[0];
    expect(format).toBe('json');
    expect(JSON.parse(data)).toEqual(snapshots);
  });

  it('Export with csv format returns csv string to onExport', () => {
    const onExport = vi.fn();
    const snapshots = [
      { timestamp: Date.now(), state: { id: 'x', value: 10 } }
    ];
    render(
      <BlackboardHistory
        {...defaultProps}
        historySnapshots={snapshots}
        onExport={onExport}
      />
    );
    fireEvent.change(screen.getByRole('combobox'), { target: { value: 'csv' } });
    fireEvent.click(screen.getByRole('button', { name: /Export/i }));
    expect(onExport).toHaveBeenCalled();
    const [data, format] = onExport.mock.calls[0];
    expect(format).toBe('csv');
    expect(data).toContain('timestamp');
    expect(data).toContain('id');
    expect(data).toContain('value');
    expect(data).toContain('x');
    expect(data).toContain('10');
  });

  it('renders change patterns when provided', () => {
    render(
      <BlackboardHistory
        {...defaultProps}
        changePatterns={{ key_a: 5, key_b: 3 }}
      />
    );
    expect(screen.getByText('Change frequency (key)')).toBeInTheDocument();
    expect(screen.getByText('key_a')).toBeInTheDocument();
    expect(screen.getByText('5 changes')).toBeInTheDocument();
    expect(screen.getByText('key_b')).toBeInTheDocument();
    expect(screen.getByText('3 changes')).toBeInTheDocument();
  });

  it('snapshot without timestamp shows – for time', () => {
    const snapshots = [{ state: { x: 1 } }];
    render(
      <BlackboardHistory
        {...defaultProps}
        historySnapshots={snapshots}
        selectedTimeIndex={0}
      />
    );
    expect(screen.getByText(/Time: –/)).toBeInTheDocument();
  });
});
