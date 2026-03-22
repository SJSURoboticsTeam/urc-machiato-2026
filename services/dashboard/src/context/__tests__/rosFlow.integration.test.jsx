/**
 * Integration tests: ROS flow (connect -> subscribe -> receive message -> update UI).
 * Uses mocked rosbridge so we can trigger connection and topic messages without a real ROS bridge.
 */
import React, { useEffect, useState } from 'react';
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { render, screen, waitFor, act } from '@testing-library/react';
import { AppProviders } from '../AppProviders';
import { useROSContext } from '../ROSContext';
import ROSLIB from '../../utils/rosbridge';

const connectionHandlers = {};
let topicSubscribeCallback = null;

vi.mock('../../utils/rosbridge', () => {
  const TopicCtor = function Topic() {
    return {
      subscribe: vi.fn((cb) => {
        topicSubscribeCallback = cb;
      }),
      unsubscribe: vi.fn()
    };
  };
  return {
    default: {
      Ros: function Ros() {
        return {
          on: vi.fn((ev, cb) => {
            connectionHandlers[ev] = cb;
          }),
          close: vi.fn()
        };
      },
      Topic: TopicCtor
    }
  };
});

vi.mock('../../hooks/useStateMachine', () => ({
  useStateMachine: () => ({
    currentState: 'IDLE',
    currentSubstate: null,
    requestStateTransition: vi.fn(),
    isTransitioning: false
  })
}));

function ROSFlowConsumer() {
  const { ros, isConnected, connectionStatus } = useROSContext();
  const [received, setReceived] = useState('');

  useEffect(() => {
    if (!ros || !isConnected) return;
    const Topic = ROSLIB.Topic;
    const topic = new Topic({ ros, name: '/chatter', messageType: 'std_msgs/String' });
    topic.subscribe((msg) => setReceived(msg?.data ?? ''));
    return () => topic.unsubscribe?.();
  }, [ros, isConnected]);

  return (
    <div>
      <span data-testid="connection-status">{connectionStatus}</span>
      <span data-testid="received">{received || 'none'}</span>
    </div>
  );
}

describe('ROS flow integration', () => {
  beforeEach(() => {
    vi.clearAllMocks();
    Object.keys(connectionHandlers).forEach((k) => delete connectionHandlers[k]);
    topicSubscribeCallback = null;
  });

  it('connect -> UI shows connected after connection event', async () => {
    render(
      <AppProviders>
        <ROSFlowConsumer />
      </AppProviders>
    );

    expect(screen.getByTestId('connection-status').textContent).toBe('connecting');

    const onConnection = connectionHandlers.connection;
    expect(onConnection).toBeDefined();
    await act(() => { onConnection(); });

    await waitFor(() => {
      expect(screen.getByTestId('connection-status').textContent).toBe('connected');
    });
  });

  it('subscribe -> receive message -> UI updates', async () => {
    render(
      <AppProviders>
        <ROSFlowConsumer />
      </AppProviders>
    );

    await act(() => { connectionHandlers.connection?.(); });

    await waitFor(() => {
      expect(screen.getByTestId('connection-status').textContent).toBe('connected');
    });

    await act(() => {
      if (topicSubscribeCallback) topicSubscribeCallback({ data: 'hello from ROS' });
    });

    await waitFor(() => {
      expect(screen.getByTestId('received').textContent).toBe('hello from ROS');
    });
  });

  it('close -> UI shows disconnected', async () => {
    render(
      <AppProviders>
        <ROSFlowConsumer />
      </AppProviders>
    );

    await act(() => { connectionHandlers.connection?.(); });
    await waitFor(() => {
      expect(screen.getByTestId('connection-status').textContent).toBe('connected');
    });

    await act(() => { connectionHandlers.close?.(); });
    await waitFor(() => {
      expect(screen.getByTestId('connection-status').textContent).toBe('disconnected');
    });
  });
});
