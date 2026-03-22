import type { IDataSource } from '../interfaces/IDataSource';
import { MockDataSource } from './MockDataSource';
import { ROSDataSource } from './ROSDataSource';

export interface DataSourceConfig {
  mode?: 'auto' | 'ros' | 'mock';
  rosUrl?: string;
  mockScenario?: string;
  mockUpdateInterval?: number;
}

const DEFAULT_ROS_URL = 'ws://localhost:9090';
const DEFAULT_MOCK_SCENARIO = 'idle';
const DEFAULT_MOCK_UPDATE_INTERVAL = 500;

export class DataSourceFactory {
  static create(config: DataSourceConfig = {}): IDataSource {
    const {
      mode = 'auto',
      rosUrl = DEFAULT_ROS_URL,
      mockScenario = DEFAULT_MOCK_SCENARIO,
      mockUpdateInterval = DEFAULT_MOCK_UPDATE_INTERVAL
    } = config;

    const useMock =
      typeof import.meta !== 'undefined' &&
      import.meta.env &&
      String(import.meta.env.VITE_USE_MOCK) === '1';

    let effectiveMode = mode;
    if (effectiveMode === 'auto') {
      effectiveMode = useMock ? 'mock' : 'ros';
    }

    console.log(`[DataSourceFactory] Creating ${effectiveMode} data source`);

    if (effectiveMode === 'mock') {
      const urlParams =
        typeof window !== 'undefined'
          ? new URLSearchParams(window.location.search)
          : new URLSearchParams();
      const scenarioParam = urlParams.get('mock') ?? mockScenario;
      return new MockDataSource(scenarioParam, mockUpdateInterval);
    }

    if (effectiveMode === 'ros') {
      return new ROSDataSource({ url: rosUrl });
    }

    throw new Error(`Unknown data source mode: ${effectiveMode}`);
  }
}
