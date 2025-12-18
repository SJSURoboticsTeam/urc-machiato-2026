import { describe, it, expect, vi } from 'vitest';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { TestingTab } from './TestingTab';

// Mock all the context and child components
vi.mock('../../context/SystemContext', () => ({
  useSystemContext: () => ({
    currentState: 'BOOT',
    systemStatus: {
      safety: 'ready',
      navigation: 'ok',
      vision: 'ready',
      can: 'mock',
      websocket: 'connected'
    },
    setRunningTests: vi.fn()
  }),
  SystemContextProvider: ({ children }) => <div>{children}</div>
}));

// Mock the IntegratedTestingDashboard
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
    render(<TestingTab />);

    expect(screen.getByText('Integrated Dashboard')).toBeInTheDocument();
    expect(screen.getByText('Component View')).toBeInTheDocument();
    expect(screen.getByText('Real-time communication flow & state visualization')).toBeInTheDocument();
    expect(screen.getByTestId('integrated-dashboard')).toBeInTheDocument();
  });

  it('switches to Component View when button is clicked', async () => {
    render(<TestingTab />);

    // Click Component View button
    fireEvent.click(screen.getByText('Component View'));

    // Should show Component View description
    await waitFor(() => {
      expect(screen.getByText('Traditional component-based testing')).toBeInTheDocument();
    });

    // Should show component cards
    expect(screen.getByText('SAFETY')).toBeInTheDocument();
    expect(screen.getByText('NAVIGATION')).toBeInTheDocument();
    expect(screen.getByText('VISION')).toBeInTheDocument();
    expect(screen.getByText('CAN BUS (MOCK)')).toBeInTheDocument();
    expect(screen.getByText('WEBSOCKET')).toBeInTheDocument();
  });

  it('shows component filter tabs in Component View', async () => {
    render(<TestingTab />);

    // Switch to Component View
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
