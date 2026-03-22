/**
 * Types for network topology tab (nodes, connections, CAN data).
 */

export interface NetworkNodePosition {
  x: number;
  y: number;
}

export interface NetworkNode {
  id: string;
  type: string;
  label: string;
  position: NetworkNodePosition;
  data: unknown;
  connections: string[];
  lastUpdate?: number;
}

export type NetworkNodesMap = Record<string, NetworkNode>;

export interface NetworkConnectionsState {
  websocket: string;
  ros2: string;
  can: string;
}

export interface CanBusStatus {
  messages_per_sec: number;
  error_count: number;
}

export interface CanDataState {
  imu: unknown;
  gps: unknown;
  motor_left: unknown;
  motor_right: unknown;
  bus_status: CanBusStatus;
}
