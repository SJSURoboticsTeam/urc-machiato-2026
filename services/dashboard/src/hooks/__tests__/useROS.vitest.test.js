/**
 * useROS hook unit tests (Vitest). Mocks rosbridge to test connection lifecycle.
 */
import { describe, it, expect, vi, beforeEach, afterEach } from 'vitest';
import { renderHook, act, waitFor } from '@testing-library/react';
import { useROS } from '../useROS';

const handlers = {};
function createRos() {
  return {
    on: vi.fn((ev, cb) => {
      handlers[ev] = cb;
    }),
    close: vi.fn()
  };
}

vi.mock('../../utils/rosbridge', () => ({
  default: {
    Ros: function MockRos() {
      return createRos();
    }
  }
}));

describe('useROS', () => {
  beforeEach(() => {
    vi.clearAllMocks();
    Object.keys(handlers).forEach((k) => delete handlers[k]);
  });

  afterEach(() => {
    vi.useRealTimers();
  });

  it('returns initial state and connect/disconnect/resetReconnection', () => {
    const { result } = renderHook(() => useROS());

    expect(result.current.isConnected).toBe(false);
    expect(result.current.connectionStatus).toBe('connecting');
    expect(result.current.reconnectAttempts).toBe(0);
    expect(result.current.lastError).toBe(null);
    expect(result.current.maxReconnectAttempts).toBe(10);
    expect(typeof result.current.connect).toBe('function');
    expect(typeof result.current.disconnect).toBe('function');
    expect(typeof result.current.resetReconnection).toBe('function');
  });

  it('connection event sets isConnected and connectionStatus', async () => {
    const { result } = renderHook(() => useROS());

    await act(() => {
      if (handlers.connection) handlers.connection();
    });

    await waitFor(() => {
      expect(result.current.isConnected).toBe(true);
      expect(result.current.connectionStatus).toBe('connected');
      expect(result.current.lastError).toBe(null);
    });
  });

  it('error event sets lastError and connectionStatus', async () => {
    const { result } = renderHook(() => useROS());
    const err = new Error('Connection refused');

    await act(() => {
      if (handlers.error) handlers.error(err);
    });

    await waitFor(() => {
      expect(result.current.lastError).toBe(err);
      expect(result.current.connectionStatus).toBe('error');
      expect(result.current.isConnected).toBe(false);
    });
  });

  it('close event sets disconnected', async () => {
    const { result } = renderHook(() => useROS());

    await act(() => { handlers.connection?.(); });
    await waitFor(() => expect(result.current.isConnected).toBe(true));

    await act(() => { handlers.close?.(); });

    await waitFor(() => {
      expect(result.current.isConnected).toBe(false);
      expect(result.current.connectionStatus).toBe('disconnected');
    });
  });

  it('disconnect clears state and stops reconnection', async () => {
    const { result } = renderHook(() => useROS());

    await act(() => { handlers.connection?.(); });
    await waitFor(() => expect(result.current.isConnected).toBe(true));

    act(() => { result.current.disconnect(); });

    expect(result.current.isConnected).toBe(false);
    expect(result.current.connectionStatus).toBe('disconnected');
    expect(result.current.reconnectAttempts).toBe(0);
    expect(result.current.lastError).toBe(null);
  });

  it('accepts custom url and maxReconnectAttempts', () => {
    const { result } = renderHook(() =>
      useROS({ url: 'ws://192.168.1.1:9090', maxReconnectAttempts: 5 })
    );

    expect(result.current.maxReconnectAttempts).toBe(5);
  });

  it('resetReconnection clears attempt count', async () => {
    const { result } = renderHook(() => useROS({ maxReconnectAttempts: 3 }));

    await act(() => { handlers.error?.(new Error('err')); });
    await waitFor(() => expect(result.current.reconnectAttempts).toBe(1));

    act(() => { result.current.resetReconnection(); });
    expect(result.current.reconnectAttempts).toBe(0);
  });
});
