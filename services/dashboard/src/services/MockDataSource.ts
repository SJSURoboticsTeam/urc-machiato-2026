import type { IDataSource, ConnectionState } from '../interfaces/IDataSource';
import { getSimulationDataForScenario } from '../utils/mockScenarios';

const DEFAULT_UPDATE_INTERVAL_MS = 500;

export class MockDataSource implements IDataSource {
  private scenario: string;
  private updateInterval: number;
  private intervalId: ReturnType<typeof setInterval> | null = null;
  private subscriptions = new Map<string, Set<(data: unknown) => void>>();
  private connectionState: ConnectionState = { status: 'disconnected' };
  private startTime = 0;

  constructor(scenario = 'idle', updateInterval = DEFAULT_UPDATE_INTERVAL_MS) {
    this.scenario = scenario;
    this.updateInterval = updateInterval;
  }

  async connect(): Promise<void> {
    if (this.intervalId) return;
    console.log(`[MockDataSource] Connecting to scenario: ${this.scenario}`);
    this.connectionState = { status: 'connected', since: new Date() };
    this.startTime = Date.now();

    this.intervalId = setInterval(() => {
      this.generateAndPublish();
    }, this.updateInterval);

    this.generateAndPublish();
  }

  async disconnect(): Promise<void> {
    if (this.intervalId) {
      clearInterval(this.intervalId);
      this.intervalId = null;
    }
    this.subscriptions.clear();
    this.connectionState = { status: 'disconnected' };
  }

  subscribe<T>(topic: string, callback: (data: T) => void): () => void {
    if (!this.subscriptions.has(topic)) {
      this.subscriptions.set(topic, new Set());
    }
    const cb = callback as (data: unknown) => void;
    this.subscriptions.get(topic)!.add(cb);

    return () => {
      const callbacks = this.subscriptions.get(topic);
      if (callbacks) {
        callbacks.delete(cb);
        if (callbacks.size === 0) {
          this.subscriptions.delete(topic);
        }
      }
    };
  }

  getConnectionState(): ConnectionState {
    return this.connectionState;
  }

  getSourceType(): 'ros' | 'mock' | 'replay' {
    return 'mock';
  }

  /**
   * Switch scenario at runtime. Resets elapsed time for the new scenario.
   */
  setScenario(scenario: string): void {
    console.log(`[MockDataSource] Switching to scenario: ${scenario}`);
    this.scenario = scenario;
    this.startTime = Date.now();
  }

  private generateAndPublish(): void {
    const elapsedSeconds = (Date.now() - this.startTime) / 1000;
    let data: ReturnType<typeof getSimulationDataForScenario>;
    try {
      data = getSimulationDataForScenario(this.scenario, elapsedSeconds);
    } catch (e) {
      console.warn('[MockDataSource] Scenario error:', e);
      return;
    }

    this.publish('telemetry', this.telemetryFromSimulation(data));
    this.publish('system/state', data.current_state);
    this.publish('network/status', data.network_status);
    this.publish('can/sensor_data', data.can_data);
    this.publish('/can/sensor_data', data.can_data);
    if (data.mission_data) {
      this.publish('mission/status', data.mission_data);
      this.publish('/mission/status', data.mission_data);
    }
  }

  private telemetryFromSimulation(
    data: ReturnType<typeof getSimulationDataForScenario>
  ): Record<string, unknown> {
    return {
      battery: data.battery,
      gps: data.gps,
      speed: data.speed,
      temperature: data.temperature,
      timestamp: data.timestamp,
      imu: data.imu,
      environment: data.environment
    };
  }

  private publish<T>(topic: string, data: T): void {
    const callbacks = this.subscriptions.get(topic);
    if (callbacks) {
      callbacks.forEach((cb) => cb(data));
    }
  }
}
