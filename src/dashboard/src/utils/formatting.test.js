import { describe, it, expect, vi, afterEach } from 'vitest';
import { formatTimestamp, formatPercentage, formatRelativeTime } from './formatting';

describe('formatting', () => {
  describe('formatTimestamp', () => {
    it('returns string for valid timestamp', () => {
      const result = formatTimestamp(Date.now());
      expect(typeof result).toBe('string');
      expect(result.length).toBeGreaterThan(0);
    });
    it('returns -- for null or NaN', () => {
      expect(formatTimestamp(null)).toBe('--');
      expect(formatTimestamp(undefined)).toBe('--');
      expect(formatTimestamp(NaN)).toBe('--');
    });
  });

  describe('formatPercentage', () => {
    it('returns percentage string for value and total', () => {
      expect(formatPercentage(50, 100)).toBe('50%');
      expect(formatPercentage(0, 100)).toBe('0%');
      expect(formatPercentage(100, 100)).toBe('100%');
    });
    it('returns 0% when total is 0 or null', () => {
      expect(formatPercentage(10, 0)).toBe('0%');
      expect(formatPercentage(10, null)).toBe('0%');
    });
  });

  describe('formatRelativeTime', () => {
    afterEach(() => vi.useRealTimers());

    it('returns Never for null or undefined', () => {
      expect(formatRelativeTime(null)).toBe('Never');
      expect(formatRelativeTime(undefined)).toBe('Never');
    });
    it('returns Just now for within last second', () => {
      vi.useFakeTimers();
      const now = 1000000000000;
      vi.setSystemTime(now);
      expect(formatRelativeTime(now - 500)).toBe('Just now');
    });
    it('returns Xs ago for within last minute', () => {
      vi.useFakeTimers();
      const now = 1000000000000;
      vi.setSystemTime(now);
      expect(formatRelativeTime(now - 5000)).toBe('5s ago');
    });
    it('returns Xm ago for within last hour', () => {
      vi.useFakeTimers();
      const now = 1000000000000;
      vi.setSystemTime(now);
      expect(formatRelativeTime(now - 120000)).toBe('2m ago');
    });
    it('returns Xh ago for older than one hour', () => {
      vi.useFakeTimers();
      const now = 1000000000000;
      vi.setSystemTime(now);
      expect(formatRelativeTime(now - 7200000)).toBe('2h ago');
    });
  });
});
