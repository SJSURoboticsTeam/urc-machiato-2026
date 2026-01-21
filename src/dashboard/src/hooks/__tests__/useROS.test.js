import { renderHook, act, waitFor } from '@testing-library/react';
import { useROS } from '../useROS';

// Mock ROSLIB
jest.mock('roslib', () => ({
  Ros: jest.fn().mockImplementation(() => ({
    connect: jest.fn(),
    close: jest.fn(),
    on: jest.fn(),
    callOnConnection: jest.fn(),
    isConnected: true,
  })),
  Topic: jest.fn().mockImplementation(() => ({
    subscribe: jest.fn(),
    unsubscribe: jest.fn(),
    publish: jest.fn(),
    advertise: jest.fn(),
    unadvertise: jest.fn(),
  })),
  Service: jest.fn().mockImplementation(() => ({
    callService: jest.fn(),
  })),
  Message: jest.fn(),
}));

describe('useROS Hook', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  test('initializes ROS connection', () => {
    const { result } = renderHook(() => useROS());

    expect(result.current.ros).toBeDefined();
    expect(result.current.isConnected).toBe(false);
    expect(result.current.error).toBeNull();
    expect(result.current.topics).toEqual({});
    expect(result.current.services).toEqual({});
  });

  test('connects to ROS on mount', async () => {
    const mockRos = {
      connect: jest.fn(),
      on: jest.fn((event, callback) => {
        if (event === 'connection') {
          callback();
        }
      }),
      isConnected: true,
    };

    // Mock the Ros constructor
    const RosMock = jest.fn(() => mockRos);
    require('roslib').Ros = RosMock;

    const { result } = renderHook(() => useROS());

    await waitFor(() => {
      expect(result.current.isConnected).toBe(true);
    });

    expect(mockRos.connect).toHaveBeenCalledWith('ws://localhost:9090');
  });

  test('handles connection errors', async () => {
    const mockRos = {
      connect: jest.fn(),
      on: jest.fn((event, callback) => {
        if (event === 'error') {
          callback({ message: 'Connection failed' });
        }
      }),
      isConnected: false,
    };

    const RosMock = jest.fn(() => mockRos);
    require('roslib').Ros = RosMock;

    const { result } = renderHook(() => useROS());

    await waitFor(() => {
      expect(result.current.error).toBe('Connection failed');
      expect(result.current.isConnected).toBe(false);
    });
  });

  test('handles disconnection', async () => {
    const mockRos = {
      connect: jest.fn(),
      on: jest.fn(),
      isConnected: true,
    };

    const RosMock = jest.fn(() => mockRos);
    require('roslib').Ros = RosMock;

    const { result } = renderHook(() => useROS());

    // Simulate disconnection
    act(() => {
      mockRos.on.mock.calls.find(call => call[0] === 'close')[1]();
    });

    await waitFor(() => {
      expect(result.current.isConnected).toBe(false);
    });
  });

  test('creates topic subscription', () => {
    const { result } = renderHook(() => useROS());

    const mockTopic = { subscribe: jest.fn() };
    const TopicMock = jest.fn(() => mockTopic);
    require('roslib').Topic = TopicMock;

    act(() => {
      result.current.subscribe('/test/topic', 'std_msgs/String', jest.fn());
    });

    expect(TopicMock).toHaveBeenCalledWith({
      ros: result.current.ros,
      name: '/test/topic',
      messageType: 'std_msgs/String',
    });

    expect(mockTopic.subscribe).toHaveBeenCalled();
    expect(result.current.topics).toHaveProperty('/test/topic');
  });

  test('publishes to topic', () => {
    const { result } = renderHook(() => useROS());

    const mockTopic = {
      advertise: jest.fn(),
      publish: jest.fn(),
      unadvertise: jest.fn(),
    };
    const TopicMock = jest.fn(() => mockTopic);
    require('roslib').Topic = TopicMock;

    act(() => {
      result.current.publish('/test/topic', 'std_msgs/String', { data: 'test' });
    });

    expect(TopicMock).toHaveBeenCalledWith({
      ros: result.current.ros,
      name: '/test/topic',
      messageType: 'std_msgs/String',
    });

    expect(mockTopic.advertise).toHaveBeenCalled();
    expect(mockTopic.publish).toHaveBeenCalledWith({ data: 'test' });
    expect(mockTopic.unadvertise).toHaveBeenCalled();
  });

  test('calls service', async () => {
    const { result } = renderHook(() => useROS());

    const mockService = {
      callService: jest.fn((request, callback) => {
        callback({ success: true });
      }),
    };
    const ServiceMock = jest.fn(() => mockService);
    require('roslib').Service = ServiceMock;

    let serviceResponse = null;
    await act(async () => {
      serviceResponse = await result.current.callService(
        '/test/service',
        'std_srvs/Trigger',
        {}
      );
    });

    expect(ServiceMock).toHaveBeenCalledWith({
      ros: result.current.ros,
      name: '/test/service',
      serviceType: 'std_srvs/Trigger',
    });

    expect(mockService.callService).toHaveBeenCalledWith({}, expect.any(Function));
    expect(serviceResponse).toEqual({ success: true });
  });

  test('unsubscribes from topic', () => {
    const { result } = renderHook(() => useROS());

    const mockTopic = {
      subscribe: jest.fn(),
      unsubscribe: jest.fn(),
    };
    const TopicMock = jest.fn(() => mockTopic);
    require('roslib').Topic = TopicMock;

    act(() => {
      result.current.subscribe('/test/topic', 'std_msgs/String', jest.fn());
      result.current.unsubscribe('/test/topic');
    });

    expect(mockTopic.unsubscribe).toHaveBeenCalled();
    expect(result.current.topics).not.toHaveProperty('/test/topic');
  });

  test('handles multiple topics and services', () => {
    const { result } = renderHook(() => useROS());

    act(() => {
      result.current.subscribe('/topic1', 'std_msgs/String', jest.fn());
      result.current.subscribe('/topic2', 'std_msgs/Int32', jest.fn());
    });

    expect(Object.keys(result.current.topics)).toHaveLength(2);
    expect(result.current.topics).toHaveProperty('/topic1');
    expect(result.current.topics).toHaveProperty('/topic2');
  });

  test('cleans up on unmount', () => {
    const { result, unmount } = renderHook(() => useROS());

    const mockRos = { close: jest.fn() };
    result.current.ros = mockRos;

    unmount();

    expect(mockRos.close).toHaveBeenCalled();
  });

  test('reconnects on error', async () => {
    const mockRos = {
      connect: jest.fn(),
      close: jest.fn(),
      on: jest.fn(),
      isConnected: false,
    };

    const RosMock = jest.fn(() => mockRos);
    require('roslib').Ros = RosMock;

    const { result } = renderHook(() => useROS());

    // Simulate connection error
    act(() => {
      mockRos.on.mock.calls.find(call => call[0] === 'error')[1]({
        message: 'Connection lost'
      });
    });

    await waitFor(() => {
      expect(result.current.error).toBe('Connection lost');
    });

    // Test reconnection
    act(() => {
      result.current.reconnect();
    });

    expect(mockRos.close).toHaveBeenCalled();
    expect(RosMock).toHaveBeenCalledTimes(2); // Original + reconnect
  });

  test('provides connection status callbacks', () => {
    const onConnect = jest.fn();
    const onDisconnect = jest.fn();
    const onError = jest.fn();

    const { result } = renderHook(() =>
      useROS({ onConnect, onDisconnect, onError })
    );

    expect(result.current).toHaveProperty('reconnect');
    expect(result.current).toHaveProperty('publish');
    expect(result.current).toHaveProperty('subscribe');
    expect(result.current).toHaveProperty('callService');
  });
});
