/**
 * IncidentReplayer: time range slider, step back/forward, play/pause, stateAtTime.
 */
import React from 'react';
import { describe, it, expect, vi } from 'vitest';
import { render, screen, fireEvent } from '@testing-library/react';
import { IncidentReplayer } from '../IncidentReplayer';

describe('IncidentReplayer', () => {
  it('renders heading and controls', () => {
    render(
      <IncidentReplayer
        timeRange={{ start: 0, end: 1000 }}
        currentTime={500}
      />
    );
    expect(screen.getByText('Incident replay')).toBeInTheDocument();
    expect(screen.getByRole('slider')).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /Step back/i })).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /Step fwd/i })).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /Play/i })).toBeInTheDocument();
  });

  it('shows progress percentage', () => {
    render(
      <IncidentReplayer
        timeRange={{ start: 0, end: 1000 }}
        currentTime={500}
      />
    );
    expect(screen.getByText('50%')).toBeInTheDocument();
  });

  it('calls onSeek when slider changes', () => {
    const onSeek = vi.fn();
    render(
      <IncidentReplayer
        timeRange={{ start: 0, end: 1000 }}
        currentTime={0}
        onSeek={onSeek}
      />
    );
    fireEvent.change(screen.getByRole('slider'), { target: { value: '300' } });
    expect(onSeek).toHaveBeenCalledWith(300);
  });

  it('calls onStep(-1) when Step back clicked', () => {
    const onStep = vi.fn();
    render(
      <IncidentReplayer
        timeRange={{ start: 0, end: 1000 }}
        currentTime={500}
        onStep={onStep}
      />
    );
    fireEvent.click(screen.getByRole('button', { name: /Step back/i }));
    expect(onStep).toHaveBeenCalledWith(-1);
  });

  it('calls onStep(1) when Step fwd clicked', () => {
    const onStep = vi.fn();
    render(
      <IncidentReplayer
        timeRange={{ start: 0, end: 1000 }}
        currentTime={500}
        onStep={onStep}
      />
    );
    fireEvent.click(screen.getByRole('button', { name: /Step fwd/i }));
    expect(onStep).toHaveBeenCalledWith(1);
  });

  it('toggles Play/Pause when play button clicked', () => {
    render(
      <IncidentReplayer
        timeRange={{ start: 0, end: 1000 }}
        currentTime={500}
      />
    );
    const playBtn = screen.getByRole('button', { name: /Play/i });
    fireEvent.click(playBtn);
    expect(screen.getByRole('button', { name: /Pause/i })).toBeInTheDocument();
    fireEvent.click(screen.getByRole('button', { name: /Pause/i }));
    expect(screen.getByRole('button', { name: /Play/i })).toBeInTheDocument();
  });

  it('shows stateAtTime when provided', () => {
    const t = 1700000000000;
    render(
      <IncidentReplayer
        timeRange={{ start: t - 1000, end: t + 1000 }}
        currentTime={t}
        stateAtTime={{ robot_x: 1 }}
      />
    );
    expect(screen.getByText(/State at/)).toBeInTheDocument();
  });

  it('handles zero duration', () => {
    render(
      <IncidentReplayer
        timeRange={{ start: 100, end: 100 }}
        currentTime={100}
      />
    );
    expect(screen.getByText('0%')).toBeInTheDocument();
  });
});
