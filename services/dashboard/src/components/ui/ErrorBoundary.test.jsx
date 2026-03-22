/**
 * DashboardErrorBoundary: catches child errors and shows fallback UI.
 * withErrorBoundary: HOC that wraps a component in the boundary.
 */
import React from 'react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen, fireEvent } from '@testing-library/react';
import { DashboardErrorBoundary, withErrorBoundary } from './ErrorBoundary';

function Thrower({ shouldThrow }) {
  if (shouldThrow) throw new Error('Test error');
  return <div>Content</div>;
}

describe('DashboardErrorBoundary', () => {
  beforeEach(() => {
    vi.spyOn(console, 'error').mockImplementation(() => {});
  });

  it('renders children when no error', () => {
    render(
      <DashboardErrorBoundary>
        <Thrower shouldThrow={false} />
      </DashboardErrorBoundary>
    );
    expect(screen.getByText('Content')).toBeInTheDocument();
    expect(screen.queryByText('Dashboard Error')).not.toBeInTheDocument();
  });

  it('getDerivedStateFromError and componentDidCatch: shows fallback UI when child throws', () => {
    render(
      <DashboardErrorBoundary>
        <Thrower shouldThrow={true} />
      </DashboardErrorBoundary>
    );
    expect(screen.getByText('Dashboard Error')).toBeInTheDocument();
    expect(screen.getByText(/Something went wrong with the URC rover dashboard/)).toBeInTheDocument();
    expect(screen.getByText('Error Details:')).toBeInTheDocument();
    expect(screen.getByText(/Test error/)).toBeInTheDocument();
    expect(screen.queryByText('Content')).not.toBeInTheDocument();
  });

  it('Try Again button resets error state and re-renders children', () => {
    const { rerender } = render(
      <DashboardErrorBoundary key="a">
        <Thrower shouldThrow={true} />
      </DashboardErrorBoundary>
    );
    expect(screen.getByText('Dashboard Error')).toBeInTheDocument();
    fireEvent.click(screen.getByRole('button', { name: /Try Again/i }));
    rerender(
      <DashboardErrorBoundary key="b">
        <Thrower shouldThrow={false} />
      </DashboardErrorBoundary>
    );
    expect(screen.getByText('Content')).toBeInTheDocument();
    expect(screen.queryByText('Dashboard Error')).not.toBeInTheDocument();
  });

  it('Reload Page button calls window.location.reload', () => {
    const reloadMock = vi.fn();
    Object.defineProperty(window, 'location', { value: { reload: reloadMock }, writable: true });
    render(
      <DashboardErrorBoundary>
        <Thrower shouldThrow={true} />
      </DashboardErrorBoundary>
    );
    fireEvent.click(screen.getByRole('button', { name: /Reload Page/i }));
    expect(reloadMock).toHaveBeenCalled();
  });

  it('fallback UI shows "What to do next" list', () => {
    render(
      <DashboardErrorBoundary>
        <Thrower shouldThrow={true} />
      </DashboardErrorBoundary>
    );
    expect(screen.getByText('What to do next:')).toBeInTheDocument();
    expect(screen.getByText(/Check your internet connection/)).toBeInTheDocument();
    expect(screen.getByText(/Verify the rover system is running/)).toBeInTheDocument();
  });
});

describe('withErrorBoundary', () => {
  beforeEach(() => {
    vi.spyOn(console, 'error').mockImplementation(() => {});
  });

  it('wraps component and renders it when no error', () => {
    const Wrapped = withErrorBoundary(() => <span>Wrapped content</span>);
    render(<Wrapped />);
    expect(screen.getByText('Wrapped content')).toBeInTheDocument();
  });

  it('shows boundary fallback when wrapped component throws', () => {
    const Failing = () => {
      throw new Error('Wrapped failed');
    };
    const Wrapped = withErrorBoundary(Failing);
    render(<Wrapped />);
    expect(screen.getByText('Dashboard Error')).toBeInTheDocument();
  });
});
