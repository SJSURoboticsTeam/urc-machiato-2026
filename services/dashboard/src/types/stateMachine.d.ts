/**
 * Shared type definitions for state machine and mission UI.
 */

export type ConnectionStatus = 'connected' | 'connecting' | 'disconnected' | 'error' | 'failed';

export interface StateBadge {
  label: string;
  color: string;
  emoji: string;
}

export interface ActiveMission {
  id?: string;
  name?: string;
  progress?: number;
  eta?: string;
  nextTask?: string;
  currentTask?: string;
  waypoints?: number | string;
  samples?: number | string;
  analysis?: string;
}

export interface MissionTemplate {
  id: string;
  name: string;
  icon?: string;
  description?: string;
}
