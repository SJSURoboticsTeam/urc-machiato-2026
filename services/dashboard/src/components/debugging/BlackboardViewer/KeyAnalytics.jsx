import React from 'react';
import { BLACKBOARD_DOMAINS } from '../constants';

/**
 * Per-key analytics: anomaly flags, value range, change count.
 */
export function KeyAnalytics({ keyStats = {}, anomalies = [] }) {
  const allKeys = Object.values(BLACKBOARD_DOMAINS).flatMap((d) => d.keys);
  const keysWithStats = allKeys.filter((k) => keyStats[k] != null);

  return (
    <div className="space-y-4">
      {anomalies.length > 0 && (
        <div className="rounded-lg border border-amber-500/30 bg-amber-500/10 p-3">
          <h4 className="text-sm font-medium text-amber-400 mb-2">Anomalies</h4>
          <ul className="space-y-1 text-xs text-amber-200/90">
            {anomalies.map((a, i) => (
              <li key={i}>{a.key}: {a.message}</li>
            ))}
          </ul>
        </div>
      )}
      <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 overflow-hidden">
        <h4 className="px-3 py-2 text-sm font-medium text-zinc-300 border-b border-zinc-800">Key statistics</h4>
        <div className="max-h-64 overflow-y-auto">
          <table className="w-full text-xs">
            <thead>
              <tr className="border-b border-zinc-800 text-zinc-500">
                <th className="px-3 py-2 text-left">Key</th>
                <th className="px-3 py-2 text-right">Changes</th>
                <th className="px-3 py-2 text-right">Min</th>
                <th className="px-3 py-2 text-right">Max</th>
              </tr>
            </thead>
            <tbody>
              {keysWithStats.slice(0, 30).map((key) => {
                const s = keyStats[key] || {};
                return (
                  <tr key={key} className="border-b border-zinc-800/50">
                    <td className="px-3 py-1.5 font-mono text-zinc-400 truncate max-w-[200px]">{key}</td>
                    <td className="px-3 py-1.5 text-right text-zinc-300">{s.changeCount ?? '–'}</td>
                    <td className="px-3 py-1.5 text-right text-zinc-300">{s.min != null ? Number(s.min).toFixed(4) : '–'}</td>
                    <td className="px-3 py-1.5 text-right text-zinc-300">{s.max != null ? Number(s.max).toFixed(4) : '–'}</td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
        {keysWithStats.length === 0 && (
          <p className="px-3 py-4 text-xs text-zinc-500">No key statistics yet.</p>
        )}
      </div>
    </div>
  );
}
