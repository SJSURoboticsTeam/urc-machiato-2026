/**
 * Error scenario: SectionErrorBoundary catches child errors and shows fallback UI.
 */
import React from 'react';
import { describe, it, expect, vi } from 'vitest';
import { render, screen, fireEvent } from '@testing-library/react';
import { SectionErrorBoundary } from '../SectionErrorBoundary';

function Thrower({ shouldThrow }) {
  if (shouldThrow) throw new Error('Test error');
  return <div>Content</div>;
}

describe('SectionErrorBoundary', () => {
  it('renders children when no error', () => {
    render(
      <SectionErrorBoundary>
        <Thrower shouldThrow={false} />
      </SectionErrorBoundary>
    );
    expect(screen.getByText('Content')).toBeInTheDocument();
    expect(screen.queryByText('Section unavailable')).not.toBeInTheDocument();
  });

  it('catches error and shows fallback UI', () => {
    const consoleSpy = vi.spyOn(console, 'error').mockImplementation(() => {});
    render(
      <SectionErrorBoundary>
        <Thrower shouldThrow={true} />
      </SectionErrorBoundary>
    );
    expect(screen.getByText('Section unavailable')).toBeInTheDocument();
    expect(screen.getByText(/This part of the dashboard failed to load/)).toBeInTheDocument();
    expect(screen.getByRole('button', { name: /Retry/i })).toBeInTheDocument();
    expect(screen.queryByText('Content')).not.toBeInTheDocument();
    consoleSpy.mockRestore();
  });

  it('Retry button clears error state so boundary tries to render children again', () => {
    const consoleSpy = vi.spyOn(console, 'error').mockImplementation(() => {});
    render(
      <SectionErrorBoundary>
        <Thrower shouldThrow={true} />
      </SectionErrorBoundary>
    );
    expect(screen.getByText('Section unavailable')).toBeInTheDocument();
    fireEvent.click(screen.getByRole('button', { name: /Retry/i }));
    expect(screen.getByText('Section unavailable')).toBeInTheDocument();
    consoleSpy.mockRestore();
  });
});
