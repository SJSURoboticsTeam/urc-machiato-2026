import React, { memo } from 'react';
import { AlertCircle } from 'lucide-react';
import { getAlertTypeColor } from '../../../utils/statusUtils';

const METRICS = [
  { key: 'CPU', value: '45%' },
  { key: 'Memory', value: '60%' },
  { key: 'Network', value: '100%' },
  { key: 'Latency', value: '12ms' },
  { key: 'Queue', value: '15/1000' },
  { key: 'Messages/s', value: '248' }
];

/**
 * Performance metrics and alerts panel. Memoized for stable props.
 * @param {Object} props
 * @param {Array<{ id: string, type: string, message: string }>} props.alerts
 */
export const DebugPerformancePanel = memo(function DebugPerformancePanel({ alerts = [] }) {
  return (
    <div className="grid grid-cols-2 gap-4">
      <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
        <h3 className="text-sm font-semibold text-zinc-200 mb-4">Performance</h3>
        <div className="space-y-3 text-sm">
          {METRICS.map(({ key, value }) => (
            <div key={key} className="flex items-center justify-between">
              <span className="text-zinc-400">{key}</span>
              <span className="text-zinc-200 font-medium">{value}</span>
            </div>
          ))}
        </div>
      </div>

      <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
        <h3 className="text-sm font-semibold text-zinc-200 mb-4">Alerts</h3>
        <div className="space-y-2">
          {alerts.length > 0 ? (
            alerts.map((alert) => (
              <div key={alert.id} className="flex items-center gap-2 text-xs">
                <AlertCircle className={`w-4 h-4 ${getAlertTypeColor(alert.type)}`} />
                <span className="text-zinc-300">{alert.message}</span>
              </div>
            ))
          ) : (
            <div className="text-xs text-zinc-500">No alerts</div>
          )}
        </div>
      </div>
    </div>
  );
});
