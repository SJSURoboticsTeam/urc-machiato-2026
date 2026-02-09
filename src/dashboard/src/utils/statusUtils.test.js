import { describe, it, expect } from 'vitest';
import {
  getStatusTextColor,
  getConnectionStateLabel,
  getLogLevelColor,
  getAlertTypeColor
} from './statusUtils';

describe('statusUtils', () => {
  describe('getStatusTextColor', () => {
    it('returns green for ok, ready, operational, connected, passed', () => {
      expect(getStatusTextColor('ok')).toBe('text-green-400');
      expect(getStatusTextColor('ready')).toBe('text-green-400');
      expect(getStatusTextColor('operational')).toBe('text-green-400');
      expect(getStatusTextColor('connected')).toBe('text-green-400');
      expect(getStatusTextColor('passed')).toBe('text-green-400');
    });
    it('returns yellow for degraded, warning, mock', () => {
      expect(getStatusTextColor('degraded')).toBe('text-yellow-400');
      expect(getStatusTextColor('warning')).toBe('text-yellow-400');
      expect(getStatusTextColor('mock')).toBe('text-yellow-400');
    });
    it('returns red for error, failed, disconnected', () => {
      expect(getStatusTextColor('error')).toBe('text-red-400');
      expect(getStatusTextColor('failed')).toBe('text-red-400');
      expect(getStatusTextColor('disconnected')).toBe('text-red-400');
    });
    it('returns blue for busy, active, running', () => {
      expect(getStatusTextColor('busy')).toBe('text-blue-400');
      expect(getStatusTextColor('active')).toBe('text-blue-400');
      expect(getStatusTextColor('running')).toBe('text-blue-400');
    });
    it('returns zinc for unknown or null', () => {
      expect(getStatusTextColor('unknown')).toBe('text-zinc-400');
      expect(getStatusTextColor(null)).toBe('text-zinc-400');
      expect(getStatusTextColor(undefined)).toBe('text-zinc-400');
    });
  });

  describe('getConnectionStateLabel', () => {
    it('returns known labels for connection status', () => {
      expect(getConnectionStateLabel('connected')).toBe('Connected');
      expect(getConnectionStateLabel('connecting')).toBe('Connecting...');
      expect(getConnectionStateLabel('disconnected')).toBe('Disconnected');
      expect(getConnectionStateLabel('error')).toBe('Error');
      expect(getConnectionStateLabel('failed')).toBe('Connection failed');
    });
    it('returns status or Unknown for unknown values', () => {
      expect(getConnectionStateLabel('other')).toBe('other');
      expect(getConnectionStateLabel(null)).toBe('Unknown');
    });
  });

  describe('getLogLevelColor', () => {
    it('returns correct colors for log levels', () => {
      expect(getLogLevelColor('ERROR')).toBe('text-red-400');
      expect(getLogLevelColor('WARN')).toBe('text-yellow-400');
      expect(getLogLevelColor('INFO')).toBe('text-blue-400');
      expect(getLogLevelColor('DEBUG')).toBe('text-zinc-400');
    });
    it('returns zinc for unknown or null', () => {
      expect(getLogLevelColor('TRACE')).toBe('text-zinc-300');
      expect(getLogLevelColor(null)).toBe('text-zinc-300');
    });
  });

  describe('getAlertTypeColor', () => {
    it('returns red for error, yellow for warning', () => {
      expect(getAlertTypeColor('error')).toBe('text-red-400');
      expect(getAlertTypeColor('warning')).toBe('text-yellow-400');
      expect(getAlertTypeColor('warn')).toBe('text-yellow-400');
    });
    it('returns zinc for unknown or null', () => {
      expect(getAlertTypeColor('info')).toBe('text-zinc-400');
      expect(getAlertTypeColor(null)).toBe('text-zinc-400');
    });
  });
});
