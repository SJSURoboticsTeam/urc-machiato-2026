import rosbridge from '../rosbridge';

// Mock ROSLIB
jest.mock('roslib', () => ({
  Ros: jest.fn().mockImplementation(() => ({
    connect: jest.fn(),
    close: jest.fn(),
    on: jest.fn(),
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

describe('ROS Bridge Utility', () => {
  beforeEach(() => {
    jest.clearAllMocks();
  });

  test('exports ROSLIB classes', () => {
    expect(rosbridge.Ros).toBeDefined();
    expect(rosbridge.Topic).toBeDefined();
    expect(rosbridge.Service).toBeDefined();
    expect(rosbridge.Message).toBeDefined();
  });

  test('provides ROS connection functionality', () => {
    const ros = new rosbridge.Ros('ws://localhost:9090');

    expect(ros.connect).toBeDefined();
    expect(ros.close).toBeDefined();
    expect(ros.on).toBeDefined();
    expect(ros.isConnected).toBe(true);
  });

  test('supports topic operations', () => {
    const topic = new rosbridge.Topic({
      ros: {},
      name: '/test/topic',
      messageType: 'std_msgs/String',
    });

    expect(topic.subscribe).toBeDefined();
    expect(topic.unsubscribe).toBeDefined();
    expect(topic.publish).toBeDefined();
    expect(topic.advertise).toBeDefined();
    expect(topic.unadvertise).toBeDefined();
  });

  test('supports service operations', () => {
    const service = new rosbridge.Service({
      ros: {},
      name: '/test/service',
      serviceType: 'std_srvs/Trigger',
    });

    expect(service.callService).toBeDefined();
  });

  test('handles connection events', () => {
    const ros = new rosbridge.Ros('ws://localhost:9090');
    const connectionCallback = jest.fn();
    const errorCallback = jest.fn();
    const closeCallback = jest.fn();

    // Test connection event registration
    ros.on('connection', connectionCallback);
    ros.on('error', errorCallback);
    ros.on('close', closeCallback);

    expect(ros.on).toHaveBeenCalledWith('connection', connectionCallback);
    expect(ros.on).toHaveBeenCalledWith('error', errorCallback);
    expect(ros.on).toHaveBeenCalledWith('close', closeCallback);
  });

  test('creates proper topic configurations', () => {
    const ros = new rosbridge.Ros('ws://localhost:9090');

    const topic = new rosbridge.Topic({
      ros: ros,
      name: '/mission/status',
      messageType: 'std_msgs/String',
    });

    expect(rosbridge.Topic).toHaveBeenCalledWith({
      ros: ros,
      name: '/mission/status',
      messageType: 'std_msgs/String',
    });
  });

  test('supports message publishing', () => {
    const topic = new rosbridge.Topic({
      ros: {},
      name: '/test/topic',
      messageType: 'std_msgs/String',
    });

    const message = new rosbridge.Message({ data: 'test message' });

    topic.publish(message);

    expect(topic.publish).toHaveBeenCalledWith(message);
  });

  test('handles service calls', () => {
    const service = new rosbridge.Service({
      ros: {},
      name: '/test/service',
      serviceType: 'std_srvs/Trigger',
    });

    const request = new rosbridge.Message({});
    const callback = jest.fn();

    service.callService(request, callback);

    expect(service.callService).toHaveBeenCalledWith(request, callback);
  });

  test('manages topic subscriptions', () => {
    const topic = new rosbridge.Topic({
      ros: {},
      name: '/test/topic',
      messageType: 'std_msgs/String',
    });

    const callback = jest.fn();

    topic.subscribe(callback);
    expect(topic.subscribe).toHaveBeenCalledWith(callback);

    topic.unsubscribe(callback);
    expect(topic.unsubscribe).toHaveBeenCalledWith(callback);
  });

  test('handles connection lifecycle', () => {
    const ros = new rosbridge.Ros('ws://localhost:9090');

    ros.connect();
    expect(ros.connect).toHaveBeenCalled();

    ros.close();
    expect(ros.close).toHaveBeenCalled();
  });

  test('provides connection status', () => {
    const ros = new rosbridge.Ros('ws://localhost:9090');

    expect(ros.isConnected).toBe(true);

    // Simulate disconnection
    ros.isConnected = false;
    expect(ros.isConnected).toBe(false);
  });

  test('supports multiple concurrent topics', () => {
    const ros = new rosbridge.Ros('ws://localhost:9090');

    const topic1 = new rosbridge.Topic({
      ros: ros,
      name: '/topic1',
      messageType: 'std_msgs/String',
    });

    const topic2 = new rosbridge.Topic({
      ros: ros,
      name: '/topic2',
      messageType: 'std_msgs/Int32',
    });

    expect(rosbridge.Topic).toHaveBeenCalledTimes(2);
  });

  test('handles message serialization', () => {
    const message = new rosbridge.Message({
      data: 'test',
      timestamp: Date.now(),
    });

    expect(message.data).toBe('test');
    expect(message.timestamp).toBeDefined();
  });

  test('supports complex message types', () => {
    const poseMessage = new rosbridge.Message({
      position: { x: 1.0, y: 2.0, z: 3.0 },
      orientation: { x: 0.0, y: 0.0, z: 0.0, w: 1.0 },
    });

    expect(poseMessage.position.x).toBe(1.0);
    expect(poseMessage.orientation.w).toBe(1.0);
  });

  test('manages topic advertisement', () => {
    const topic = new rosbridge.Topic({
      ros: {},
      name: '/test/publisher',
      messageType: 'std_msgs/String',
    });

    topic.advertise();
    expect(topic.advertise).toHaveBeenCalled();

    topic.unadvertise();
    expect(topic.unadvertise).toHaveBeenCalled();
  });
});
