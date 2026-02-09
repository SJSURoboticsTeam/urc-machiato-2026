/**
 * Centralized number/date formatting for the dashboard.
 * Re-exports and supplements uiUtils where appropriate.
 */

/**
 * Format a Unix timestamp for display (time only).
 * @param {number} timestamp - Unix ms
 * @returns {string}
 */
export function formatTimestamp(timestamp) {
  if (timestamp == null || Number.isNaN(Number(timestamp))) return '--';
  return new Date(Number(timestamp)).toLocaleTimeString();
}

/**
 * Format value/total as percentage string.
 * @param {number} value
 * @param {number} total
 * @returns {string}
 */
export function formatPercentage(value, total) {
  if (total == null || total <= 0) return '0%';
  return `${Math.round((Number(value) / total) * 100)}%`;
}

/**
 * Format a timestamp as relative time (e.g. "Just now", "5s ago", "2m ago").
 * @param {number|null|undefined} timestamp - Unix ms
 * @returns {string}
 */
export function formatRelativeTime(timestamp) {
  if (timestamp == null) return 'Never';
  const age = Date.now() - Number(timestamp);
  if (age < 1000) return 'Just now';
  if (age < 60000) return `${Math.floor(age / 1000)}s ago`;
  if (age < 3600000) return `${Math.floor(age / 60000)}m ago`;
  return `${Math.floor(age / 3600000)}h ago`;
}
