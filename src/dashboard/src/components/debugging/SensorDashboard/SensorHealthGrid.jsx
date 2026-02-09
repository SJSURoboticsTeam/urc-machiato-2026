import React from 'react';
import { CheckCircle, AlertTriangle, XCircle, Clock } from 'lucide-react';
import { SENSOR_IDS, SENSOR_LABELS, CONFIDENCE_THRESHOLDS } from '../constants';

/**
 * Grid of 9 sensors: health score, last update, confidence.
 * Color + icon + text for accessibility (not color-only). Staleness clearly indicated.
 */
export function SensorHealthGrid({ sensorData = {}, stalenessSec = 2 }) {
  const nowMs = Date.now();

  const getStatus = (confidence, isStale) => {
    if (isStale) return { icon: XCircle, label: 'Stale', colorClass: 'red' };
    if (confidence >= CONFIDENCE_THRESHOLDS.HIGH) return { icon: CheckCircle, label: 'OK', colorClass: 'green' };
    if (confidence >= CONFIDENCE_THRESHOLDS.LOW) return { icon: AlertTriangle, label: 'Degraded', colorClass: 'yellow' };
    return { icon: XCircle, label: 'Low', colorClass: 'red' };
  };

  const colorClasses = {
    green: 'border-emerald-500/50 bg-emerald-500/10 text-emerald-400',
    yellow: 'border-amber-500/50 bg-amber-500/10 text-amber-400',
    red: 'border-red-500/50 bg-red-500/10 text-red-400'
  };

  const barColors = {
    green: 'bg-emerald-500',
    yellow: 'bg-amber-500',
    red: 'bg-red-500'
  };

  const formatLastUpdate = (ts) => {
    if (ts == null) return 'Never';
    const ageSec = (nowMs - ts) / 1000;
    if (ageSec < 60) return `${ageSec.toFixed(1)}s ago`;
    const ageMin = ageSec / 60;
    return `${ageMin.toFixed(1)}m ago`;
  };

  return (
    <div
      className="grid grid-cols-3 gap-3 min-w-[280px] max-w-4xl"
      role="region"
      aria-label="Sensor health status grid"
    >
      {SENSOR_IDS.map((id) => {
        const data = sensorData[id] || {};
        const confidence = data.confidence ?? 0;
        const lastUpdate = data.lastUpdateMs ?? null;
        const isStale = lastUpdate != null && (nowMs - lastUpdate) / 1000 > stalenessSec;
        const healthScore = data.healthScore ?? confidence;
        const status = getStatus(confidence, isStale);
        const Icon = status.icon;

        return (
          <div
            key={id}
            className={`rounded-lg border p-3 ${colorClasses[status.colorClass]} ${isStale ? 'ring-2 ring-red-400/50' : ''}`}
            role="article"
            aria-label={`${SENSOR_LABELS[id] || id}: ${status.label}, health ${(healthScore * 100).toFixed(0)} percent, last update ${formatLastUpdate(lastUpdate)}`}
          >
            <div className="flex items-center justify-between gap-2">
              <span className="text-sm font-medium truncate">{SENSOR_LABELS[id] || id}</span>
              <span className="flex items-center gap-1 shrink-0" aria-hidden>
                <Icon className="h-4 w-4" />
                <span className="text-xs font-medium">{status.label}</span>
              </span>
            </div>
            <div className="mt-1 flex items-center gap-1 text-xs opacity-90">
              <Clock className="h-3 w-3 shrink-0" aria-hidden />
              {formatLastUpdate(lastUpdate)}
            </div>
            <div className="mt-1 flex items-center gap-1">
              <div
                className="h-1.5 flex-1 rounded-full bg-zinc-700 min-w-0"
                role="progressbar"
                aria-valuenow={Math.round(confidence * 100)}
                aria-valuemin={0}
                aria-valuemax={100}
                aria-label={`Confidence ${(confidence * 100).toFixed(0)} percent`}
              >
                <div
                  className={`h-full rounded-full ${barColors[status.colorClass]}`}
                  style={{ width: `${Math.min(100, confidence * 100)}%` }}
                />
              </div>
              <span className="text-xs tabular-nums w-8 shrink-0">{(confidence * 100).toFixed(0)}%</span>
            </div>
          </div>
        );
      })}
    </div>
  );
}
