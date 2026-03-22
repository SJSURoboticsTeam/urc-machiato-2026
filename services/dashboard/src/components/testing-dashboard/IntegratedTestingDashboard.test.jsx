import React from 'react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen, fireEvent, waitFor } from '@testing-library/react';
import { IntegratedTestingDashboard } from './IntegratedTestingDashboard';
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

// Mock the child components
vi.mock('./MessageTester', () => ({
  MessageTester: ({ messageHistory, setMessageHistory, setFlashingChannel, setLastMessageFlow, commMetrics, setCommMetrics }) => (
    <div data-testid="message-tester">
      <button onClick={() => {
        setMessageHistory([{ id: '1', content: 'test', target: 'websocket', timestamp: Date.now(), status: 'sent' }]);
        setFlashingChannel('websocket');
        setLastMessageFlow({ from: 'user', to: 'websocket', timestamp: Date.now() });
        setCommMetrics({ websocket: { sent: 1, received: 0, lastMsg: 'test', active: true } });
      }}>
        Send Test Message
      </button>
      <div>Messages: {messageHistory.length}</div>
    </div>
  )
}));

vi.mock('./StateTransitionTester', () => ({
  StateTransitionTester: ({ currentState, requestStateTransition, stateTransitionResult, setStateTransitionResult }) => (
    <div data-testid="state-transition-tester">
      <div>Current State: {currentState}</div>
      <button onClick={() => {
        requestStateTransition('READY', 'test transition');
        setStateTransitionResult({ success: true, message: 'Transition successful' });
      }}>
        Transition to READY
      </button>
      {stateTransitionResult && <div>Result: {stateTransitionResult.message}</div>}
    </div>
  )
}));

vi.mock('./TopicDataViewer', () => ({
  TopicDataViewer: () => (
    <div data-testid="topic-data-viewer">
      <div>Available Topics: 6</div>
    </div>
  )
}));

vi.mock('./CommunicationMetrics', () => ({
  CommunicationMetrics: ({ commMetrics }) => (
    <div data-testid="communication-metrics">
      <div>WebSocket: {commMetrics.websocket.sent} sent</div>
    </div>
  )
}));

vi.mock('./ThreeColumnTestingDashboard', () => ({
  ThreeColumnTestingDashboard: () => <div data-testid="three-column-mock">Three Column View</div>
}));

describe('IntegratedTestingDashboard', () => {
  beforeEach(() => {
    // Reset all mocks
    vi.clearAllMocks();
  });

  it('renders all testing tabs correctly', () => {
    render(
      <AppProviders>
        <IntegratedTestingDashboard />
      </AppProviders>
    );

    // Check that the main header is rendered
    expect(screen.getByText('Integrated Testing Dashboard')).toBeInTheDocument();
    expect(screen.getByText('Human verification interface for system integration testing')).toBeInTheDocument();

    // Check that all tab buttons are rendered
    expect(screen.getByText('Messages')).toBeInTheDocument();
    expect(screen.getByText('States')).toBeInTheDocument();
    expect(screen.getByText('Topics')).toBeInTheDocument();
    expect(screen.getByText('Metrics')).toBeInTheDocument();
  });

  it('shows first tab (Three Column) by default', () => {
    render(
      <AppProviders>
        <IntegratedTestingDashboard />
      </AppProviders>
    );

    expect(screen.getByTestId('three-column-mock')).toBeInTheDocument();
  });

  it('switches between tabs correctly', async () => {
    render(
      <AppProviders>
        <IntegratedTestingDashboard />
      </AppProviders>
    );

    // Click on States tab
    fireEvent.click(screen.getByText('States'));
    await waitFor(() => {
      expect(screen.getByTestId('state-transition-tester')).toBeInTheDocument();
    });

    // Click on Topics tab
    fireEvent.click(screen.getByText('Topics'));
    await waitFor(() => {
      expect(screen.getByTestId('topic-data-viewer')).toBeInTheDocument();
    });

    // Click on Metrics tab
    fireEvent.click(screen.getByText('Metrics'));
    await waitFor(() => {
      expect(screen.getByTestId('communication-metrics')).toBeInTheDocument();
    });
  });

  it('displays connection status correctly', () => {
    render(
      <AppProviders>
        <IntegratedTestingDashboard />
      </AppProviders>
    );

    expect(screen.getByText(/Connected/)).toBeInTheDocument();
    expect(screen.getByText(/State:/)).toBeInTheDocument();
    expect(screen.getByText('BOOT')).toBeInTheDocument();
  });

  it('handles message sending interaction', async () => {
    render(
      <AppProviders>
        <IntegratedTestingDashboard />
      </AppProviders>
    );

    fireEvent.click(screen.getByText('Messages'));
    await waitFor(() => {
      expect(screen.getByTestId('message-tester')).toBeInTheDocument();
    });
    const sendButton = screen.getByText('Send Test Message');
    fireEvent.click(sendButton);

    await waitFor(() => {
      expect(screen.getByText('Messages: 1')).toBeInTheDocument();
    });
  });

  it('handles state transition interaction', async () => {
    render(
      <AppProviders>
        <IntegratedTestingDashboard />
      </AppProviders>
    );

    // Switch to States tab
    fireEvent.click(screen.getByText('States'));
    await waitFor(() => {
      expect(screen.getByTestId('state-transition-tester')).toBeInTheDocument();
    });

    // Click the transition button
    const transitionButton = screen.getByText('Transition to READY');
    fireEvent.click(transitionButton);

    // Check that the result is displayed
    await waitFor(() => {
      expect(screen.getByText('Result: Transition successful')).toBeInTheDocument();
    });
  });
});
