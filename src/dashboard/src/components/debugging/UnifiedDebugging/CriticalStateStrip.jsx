import React from 'react';
import { Wifi, WifiOff, CheckCircle, AlertTriangle, XCircle } from 'lucide-react';
import { SENSOR_IDS, CONFIDENCE_THRESHOLDS, STALENESS_SEC } from '../constants';

/**
 * Glanceable strip: connection, sensor summary (X/9 OK), alert count.
 * Designed for 2-3 second comprehension. Color + icon + text (accessibility).
 */
export function CriticalStateStrip({
  isConnected = false,
  sensorData = {},
  alertCount = 0,
  stalenessSec = STALENESS_SEC,
  className = ''
}) {
  const nowMs = Date.now();
  let okCount = 0;
  let staleCount = 0;
  let lowConfidenceCount = 0;

  const hasAnyData = Object.keys(sensorData).length > 0;
  SENSOR_IDS.forEach((id) => {
    const data = sensorData[id] || {};
    const confidence = data.confidence ?? 0;
    const lastUpdate = data.lastUpdateMs ?? null;
    const noData = lastUpdate == null && confidence === 0;
    if (noData && !hasAnyData) return;
    const isStale = lastUpdate != null && (nowMs - lastUpdate) / 1000 > stalenessSec;
    if (isStale) staleCount += 1;
    else if (!noData && confidence < CONFIDENCE_THRESHOLDS.LOW) lowConfidenceCount += 1;
    else if (noData) return;
    else okCount += 1;
  });

  const totalSensors = SENSOR_IDS.length;
  const sensorsOk = hasAnyData && staleCount === 0 && lowConfidenceCount === 0;
  const sensorsDegraded = hasAnyData && (staleCount > 0 || lowConfidenceCount > 0);
  const statusLabel = !hasAnyData
    ? 'Sensors (no data)'
    : sensorsOk
      ? `Sensors ${okCount}/${totalSensors} OK`
      : `Sensors: ${okCount} OK, ${staleCount} stale, ${lowConfidenceCount} low`;

  return (
    <div
      role="status"
      aria-live="polite"
      aria-label={`Connection ${isConnected ? 'connected' : 'disconnected'}. ${statusLabel}. ${alertCount} alerts.`}
      className={`flex flex-wrap items-center gap-4 border-b border-zinc-800 bg-zinc-900/80 px-4 py-2 text-sm ${className}`}
    >
      {/* Connection: icon + text, not color-only */}
      <div className="flex items-center gap-2">
        {isConnected ? (
          <Wifi className="h-4 w-4 text-emerald-500" aria-hidden />
        ) : (
          <WifiOff className="h-4 w-4 text-zinc-500" aria-hidden />
        )}
        <span className={isConnected ? 'text-emerald-400' : 'text-zinc-500'}>
          {isConnected ? 'Connected' : 'Disconnected'}
        </span>
      </div>

      {/* Sensor summary: icon + count + text (not color-only) */}
      <div className="flex items-center gap-2">
        {sensorsOk ? (
          <CheckCircle className="h-4 w-4 text-emerald-500" aria-hidden />
        ) : sensorsDegraded ? (
          <AlertTriangle className="h-4 w-4 text-amber-500" aria-hidden />
        ) : (
          <span className="inline-block h-4 w-4 text-center text-zinc-500" aria-hidden>-</span>
        )}
        <span className={sensorsOk ? 'text-zinc-300' : sensorsDegraded ? 'text-amber-400' : 'text-zinc-500'}>
          {statusLabel}
        </span>
      </div>

      {/* Alerts: only show when non-zero */}
      {alertCount > 0 && (
        <div className="flex items-center gap-2">
          <XCircle className="h-4 w-4 text-red-500" aria-hidden />
          <span className="text-red-400" aria-label={`${alertCount} alerts`}>
            {alertCount} alert{alertCount !== 1 ? 's' : ''}
          </span>
        </div>
      )}

      <div className="ml-auto text-xs text-zinc-500 tabular-nums">
        {new Date().toISOString().slice(11, 19)}
      </div>
    </div>
  );
}
