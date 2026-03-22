import React from 'react';
import { AlertCircle } from 'lucide-react';
import { SENSOR_LABELS, CONFIDENCE_THRESHOLDS, STALENESS_SEC } from '../constants';

/**
 * Alert list: staleness, low confidence, failure root cause.
 */
export function SensorAlerts({ sensorData = {}, alerts = [] }) {
  const nowMs = Date.now();
  const derivedAlerts = [];

  Object.entries(sensorData).forEach(([id, data]) => {
    const lastUpdate = data?.lastUpdateMs;
    const confidence = data?.confidence ?? 0;
    if (lastUpdate != null && (nowMs - lastUpdate) / 1000 > STALENESS_SEC) {
      derivedAlerts.push({
        id: `stale-${id}`,
        type: 'stale',
        severity: 'warning',
        message: `${SENSOR_LABELS[id] || id} has not updated in ${((nowMs - lastUpdate) / 1000).toFixed(0)}s`
      });
    }
    if (confidence < CONFIDENCE_THRESHOLDS.LOW && confidence > 0) {
      derivedAlerts.push({
        id: `confidence-${id}`,
        type: 'confidence',
        severity: 'error',
        message: `${SENSOR_LABELS[id] || id} confidence low (${(confidence * 100).toFixed(0)}%)`
      });
    }
  });

  const allAlerts = [...derivedAlerts, ...alerts];

  if (allAlerts.length === 0) {
    return (
      <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-3 text-xs text-zinc-500">
        No sensor alerts.
      </div>
    );
  }

  return (
    <div className="space-y-2">
      <div className="flex items-center gap-2 text-sm font-medium text-zinc-300">
        <AlertCircle className="h-4 w-4" />
        Alerts
      </div>
      <ul className="space-y-1">
        {allAlerts.map((a) => (
          <li
            key={a.id}
            className={`flex items-center gap-2 rounded px-2 py-1 text-xs ${
              a.severity === 'error' ? 'bg-red-500/10 text-red-400' : 'bg-amber-500/10 text-amber-400'
            }`}
          >
            <AlertCircle className="h-3 w-3 shrink-0" />
            {a.message}
          </li>
        ))}
      </ul>
    </div>
  );
}
