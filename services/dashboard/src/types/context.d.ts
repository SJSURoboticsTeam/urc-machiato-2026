/**
 * Context value types for dashboard providers.
 */

import type { IDataSource } from '../interfaces/IDataSource';
import type { Telemetry, SystemStatus } from './telemetry';
import type { ActiveMission } from './stateMachine';

export interface ROSContextValue {
  ros: unknown;
  isConnected: boolean;
  connectionStatus: string;
  lastError: Error | null;
  reconnectAttempts: number;
  resetReconnection: () => void;
  connect: () => void;
}

export interface StateBadgeConfig {
  label: string;
  color: string;
  emoji: string;
}

export interface StateMachineContextValue {
  currentState: string;
  currentSubstate: string;
  requestStateTransition: (state: string, reason: string) => Promise<unknown>;
  isTransitioning: boolean;
  getStateBadge: () => StateBadgeConfig;
  handleEmergencyStop: () => Promise<void>;
}

export interface TelemetryContextValue {
  telemetry: Telemetry;
  setTelemetry: (next: Telemetry | ((prev: Telemetry) => Telemetry)) => void;
  systemStatus: SystemStatus;
  setSystemStatus: (value: SystemStatus | ((prev: SystemStatus) => SystemStatus)) => void;
  /** Current data source when using DataSourceFactory (mock mode). */
  dataSource?: IDataSource | null;
}

export interface UIContextAlert {
  id: string;
  type: string;
  message: string;
  component?: string;
  timestamp?: number;
}

export interface UIContextValue {
  alerts: UIContextAlert[];
  setAlerts: (value: UIContextAlert[] | ((prev: UIContextAlert[]) => UIContextAlert[])) => void;
  errorCount: number;
  activeMission: ActiveMission | null;
  setActiveMission: (mission: ActiveMission | null) => void;
  runningTests: number;
  setRunningTests: (n: number | ((prev: number) => number)) => void;
  isOnline: boolean;
  offlineData: Record<string, unknown> | null;
  cacheDataForOffline: (data: Record<string, unknown>) => void;
}
