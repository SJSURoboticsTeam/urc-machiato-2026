import type { IDataSource, ConnectionState } from '../interfaces/IDataSource';
import ROSLIB from '../utils/rosbridge';
import { createSubscriber } from '../utils/rosbridge';

const DEFAULT_ROS_URL = 'ws://localhost:9090';
const CONNECTION_TIMEOUT_MS = 5000;
const DEFAULT_RECONNECT_INTERVAL = 3000;
const DEFAULT_MAX_RECONNECT_ATTEMPTS = 10;

/** Map topic names (logical or path) to ROS message types. */
const TOPIC_MESSAGE_TYPES: Record<string, string> = {
  '/state_machine/current_state': 'std_msgs/String',
  '/state_machine/state_transition': 'std_msgs/String',
  '/state_machine/substate': 'std_msgs/String',
  '/mission/status': 'std_msgs/String',
  '/mission/progress': 'std_msgs/String',
  '/mission/telemetry': 'std_msgs/String',
  '/can/sensor_data': 'std_msgs/String',
  '/telemetry': 'std_msgs/String',
  telemetry: 'std_msgs/String',
  'system/state': 'std_msgs/String',
  'network/status': 'std_msgs/String',
  'mission/status': 'std_msgs/String'
};

export interface ROSDataSourceConfig {
  url?: string;
  reconnectInterval?: number;
  maxReconnectAttempts?: number;
}

interface ROSSubscription {
  topic: ROSLIB.Topic;
  unsubscribe: () => void;
}

export class ROSDataSource implements IDataSource {
  private config: Required<ROSDataSourceConfig>;
  private ros: ROSLIB.Ros | null = null;
  private connectionState: ConnectionState = { status: 'disconnected' };
  private subscriptions = new Map<string, ROSSubscription>();
  private reconnectTimeoutId: ReturnType<typeof setTimeout> | null = null;
  private reconnectAttempts = 0;

  constructor(config: ROSDataSourceConfig = {}) {
    this.config = {
      url: config.url ?? DEFAULT_ROS_URL,
      reconnectInterval: config.reconnectInterval ?? DEFAULT_RECONNECT_INTERVAL,
      maxReconnectAttempts:
        config.maxReconnectAttempts ?? DEFAULT_MAX_RECONNECT_ATTEMPTS
    };
  }

  async connect(): Promise<void> {
    if (this.ros) {
      this.ros.close();
      this.ros = null;
    }

    this.connectionState = { status: 'connecting', attempt: this.reconnectAttempts + 1 };

    return new Promise((resolve, reject) => {
      const timeoutId = setTimeout(() => {
        this.connectionState = {
          status: 'error',
          error: new Error('ROS connection timeout'),
          retryable: true
        };
        reject(new Error('ROS connection timeout'));
      }, CONNECTION_TIMEOUT_MS);

      this.ros = new ROSLIB.Ros({ url: this.config.url });

      this.ros.on('connection', () => {
        clearTimeout(timeoutId);
        this.connectionState = { status: 'connected', since: new Date() };
        this.reconnectAttempts = 0;
        resolve();
      });

      this.ros.on('error', (error: Error) => {
        clearTimeout(timeoutId);
        this.connectionState = {
          status: 'error',
          error: error instanceof Error ? error : new Error(String(error)),
          retryable: true
        };
        this._scheduleReconnect();
        reject(error);
      });

      this.ros.on('close', () => {
        this.connectionState = { status: 'disconnected' };
        if (
          this.ros &&
          this.reconnectAttempts < this.config.maxReconnectAttempts
        ) {
          this._scheduleReconnect();
        }
      });
    });
  }

  private _scheduleReconnect(): void {
    if (this.reconnectTimeoutId) return;
    this.reconnectAttempts += 1;
    const delay = Math.min(
      this.config.reconnectInterval *
        Math.pow(2, this.reconnectAttempts - 1),
      30000
    );
    this.reconnectTimeoutId = setTimeout(() => {
      this.reconnectTimeoutId = null;
      this.connect().catch(() => {
        // Reconnection failed; state already updated
      });
    }, delay);
  }

  async disconnect(): Promise<void> {
    if (this.reconnectTimeoutId) {
      clearTimeout(this.reconnectTimeoutId);
      this.reconnectTimeoutId = null;
    }
    for (const sub of this.subscriptions.values()) {
      sub.unsubscribe();
    }
    this.subscriptions.clear();
    if (this.ros) {
      this.ros.close();
      this.ros = null;
    }
    this.connectionState = { status: 'disconnected' };
    this.reconnectAttempts = 0;
  }

  subscribe<T>(topic: string, callback: (data: T) => void): () => void {
    if (!this.ros) {
      throw new Error('Cannot subscribe: not connected to ROS');
    }

    const topicPath = topic.startsWith('/') ? topic : this._logicalToPath(topic);
    const messageType = this._getMessageType(topicPath);

    const rosTopic = createSubscriber(
      this.ros,
      topicPath,
      messageType,
      (message: T) => callback(message)
    );

    const unsubscribe = () => {
      rosTopic.unsubscribe();
      this.subscriptions.delete(topic);
    };

    this.subscriptions.set(topic, { topic: rosTopic, unsubscribe });

    return unsubscribe;
  }

  private _logicalToPath(topic: string): string {
    const map: Record<string, string> = {
      telemetry: '/telemetry',
      'system/state': '/state_machine/current_state',
      'network/status': '/can/sensor_data',
      'can/sensor_data': '/can/sensor_data',
      'mission/status': '/mission/status'
    };
    return map[topic] ?? topic;
  }

  private _getMessageType(topic: string): string {
    const path = topic.startsWith('/') ? topic : this._logicalToPath(topic);
    return TOPIC_MESSAGE_TYPES[path] ?? TOPIC_MESSAGE_TYPES[topic] ?? 'std_msgs/String';
  }

  getConnectionState(): ConnectionState {
    return this.connectionState;
  }

  getSourceType(): 'ros' | 'mock' | 'replay' {
    return 'ros';
  }
}
