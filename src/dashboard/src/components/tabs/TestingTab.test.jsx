import React from 'react';
import { describe, it, expect, vi } from 'vitest';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { TestingTab } from './TestingTab';
import { AppProviders } from '../../context/AppProviders';

vi.mock('../../hooks/useROS', () => ({
  useROS: () => ({ ros: {}, isConnected: true, connectionStatus: 'connected' })
}));
vi.mock('../../hooks/useStateMachine', () => ({
  useStateMachine: () => ({
    currentState: 'BOOT',
    currentSubstate: null,
    requestStateTransition: vi.fn(),
    isTransitioning: false
  })
}));

vi.mock('../testing-dashboard/IntegratedTestingDashboard', () => ({
  IntegratedTestingDashboard: () => (
    <div data-testid="integrated-dashboard">
      <h1>Integrated Testing Dashboard</h1>
      <p>Real-time communication flow & state visualization</p>
    </div>
  )
}));

describe('TestingTab', () => {
  it('renders with view toggle buttons', () => {
    render(
      <AppProviders>
        <TestingTab />
      </AppProviders>
    );

    expect(screen.getByText('Integrated Dashboard')).toBeInTheDocument();
    expect(screen.getByText('Component View')).toBeInTheDocument();
    expect(screen.getAllByText(/Real-time communication flow/).length).toBeGreaterThanOrEqual(1);
    expect(screen.getByTestId('integrated-dashboard')).toBeInTheDocument();
  });

  it('switches to Component View when button is clicked', async () => {
    render(
      <AppProviders>
        <TestingTab />
      </AppProviders>
    );

    fireEvent.click(screen.getByText('Component View'));

    await waitFor(() => {
      expect(screen.getByText('Traditional component-based testing')).toBeInTheDocument();
    });

    expect(screen.getByText('SAFETY')).toBeInTheDocument();
    expect(screen.getByText('NAVIGATION')).toBeInTheDocument();
    expect(screen.getByText('VISION')).toBeInTheDocument();
    expect(screen.getAllByText(/CAN BUS/i).length).toBeGreaterThanOrEqual(1);
    expect(screen.getByText('WEBSOCKET')).toBeInTheDocument();
  });

  it('shows component filter tabs in Component View', async () => {
    render(
      <AppProviders>
        <TestingTab />
      </AppProviders>
    );

    fireEvent.click(screen.getByText('Component View'));
    await waitFor(() => {
      expect(screen.getByText('Traditional component-based testing')).toBeInTheDocument();
    });

    // Check filter tabs
    expect(screen.getByText('All')).toBeInTheDocument();
    expect(screen.getByText('Safety')).toBeInTheDocument();
    expect(screen.getByText('Navigation')).toBeInTheDocument();
    expect(screen.getByText('Vision')).toBeInTheDocument();
    expect(screen.getByText('CAN Bus')).toBeInTheDocument();
    expect(screen.getByText('WebSocket')).toBeInTheDocument();
  });
});
