/**
 * Centralized status colors and connection labels for the dashboard.
 * Use these instead of duplicating switch/object logic across components.
 */

/** Tailwind text color class for a given status string */
export function getStatusTextColor(status) {
  if (!status) return 'text-zinc-400';
  const s = String(status).toLowerCase();
  if (['ok', 'ready', 'operational', 'connected', 'passed', 'healthy'].includes(s)) return 'text-green-400';
  if (['degraded', 'warning', 'mock'].includes(s)) return 'text-yellow-400';
  if (['error', 'failed', 'disconnected', 'unhealthy'].includes(s)) return 'text-red-400';
  if (['busy', 'active', 'running'].includes(s)) return 'text-blue-400';
  return 'text-zinc-400';
}

/** Human-readable label for ROS connection status */
export function getConnectionStateLabel(connectionStatus) {
  const labels = {
    connected: 'Connected',
    connecting: 'Connecting...',
    disconnected: 'Disconnected',
    error: 'Error',
    failed: 'Connection failed'
  };
  return labels[connectionStatus] || connectionStatus || 'Unknown';
}

/**
 * Tailwind text color class for log level (DEBUG, INFO, WARN, ERROR).
 * Use for log viewers and debug panels.
 * @param {string} level - Log level
 * @returns {string}
 */
export function getLogLevelColor(level) {
  if (!level) return 'text-zinc-300';
  const l = String(level).toUpperCase();
  if (l === 'ERROR') return 'text-red-400';
  if (l === 'WARN') return 'text-yellow-400';
  if (l === 'INFO') return 'text-blue-400';
  if (l === 'DEBUG') return 'text-zinc-400';
  return 'text-zinc-300';
}

/**
 * Tailwind text color class for alert type (error, warning, etc.).
 * @param {string} type - Alert type
 * @returns {string}
 */
export function getAlertTypeColor(type) {
  if (!type) return 'text-zinc-400';
  const t = String(type).toLowerCase();
  if (t === 'error') return 'text-red-400';
  if (t === 'warning' || t === 'warn') return 'text-yellow-400';
  return 'text-zinc-400';
}
