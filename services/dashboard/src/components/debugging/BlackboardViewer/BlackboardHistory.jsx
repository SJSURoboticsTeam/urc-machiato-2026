import React, { useState } from 'react';
import { BLACKBOARD_DOMAINS } from '../constants';

/**
 * Historical blackboard: timeline selector, state at time, change patterns, export.
 */
export function BlackboardHistory({
  historySnapshots = [],
  selectedTimeIndex = 0,
  onSelectTime,
  changePatterns = {},
  onExport
}) {
  const [exportFormat, setExportFormat] = useState('json');
  const snapshot = historySnapshots[selectedTimeIndex] ?? {};
  const snapshotTime = snapshot.timestamp != null ? new Date(snapshot.timestamp).toISOString() : '–';

  const handleExport = () => {
    const data = historySnapshots.length > 0
      ? (exportFormat === 'json'
          ? JSON.stringify(historySnapshots, null, 2)
          : exportCsv(historySnapshots))
      : '';
    if (data && onExport) onExport(data, exportFormat);
    else if (typeof navigator !== 'undefined' && navigator.clipboard) {
      navigator.clipboard.writeText(data || 'No data');
    }
  };

  const exportCsv = (snapshots) => {
    const keys = new Set();
    snapshots.forEach((s) => Object.keys(s.state || {}).forEach((k) => keys.add(k)));
    const header = ['timestamp', ...Array.from(keys)];
    const rows = snapshots.map((s) => [
      new Date(s.timestamp).toISOString(),
      ...Array.from(keys).map((k) => (s.state && s.state[k] != null ? String(s.state[k]) : ''))
    ]);
    return [header.join(','), ...rows.map((r) => r.join(','))].join('\n');
  };

  return (
    <div className="space-y-4">
      <div className="flex flex-wrap items-center justify-between gap-2">
        <span className="text-sm text-zinc-400">Timeline</span>
        <div className="flex items-center gap-2">
          <select
            value={exportFormat}
            onChange={(e) => setExportFormat(e.target.value)}
            className="rounded border border-zinc-600 bg-zinc-800 px-2 py-1 text-xs text-zinc-200"
          >
            <option value="json">JSON</option>
            <option value="csv">CSV</option>
          </select>
          <button
            type="button"
            onClick={handleExport}
            className="rounded border border-zinc-600 bg-zinc-800 px-2 py-1 text-xs text-zinc-200 hover:bg-zinc-700"
          >
            Export
          </button>
        </div>
      </div>

      {historySnapshots.length > 0 ? (
        <>
          <input
            type="range"
            min={0}
            max={Math.max(0, historySnapshots.length - 1)}
            value={selectedTimeIndex}
            onChange={(e) => onSelectTime?.(Number(e.target.value))}
            className="w-full accent-zinc-500"
          />
          <p className="text-xs text-zinc-500">Time: {snapshotTime}</p>
          <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-3 max-h-64 overflow-y-auto">
            <pre className="text-xs text-zinc-300 whitespace-pre-wrap">
              {JSON.stringify(snapshot.state ?? {}, null, 2)}
            </pre>
          </div>
        </>
      ) : (
        <p className="text-sm text-zinc-500">No history snapshots. Start recording or load a session.</p>
      )}

      {Object.keys(changePatterns).length > 0 && (
        <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-3">
          <h4 className="text-sm font-medium text-zinc-300 mb-2">Change frequency (key)</h4>
          <ul className="space-y-1 text-xs">
            {Object.entries(changePatterns)
              .sort((a, b) => (b[1] ?? 0) - (a[1] ?? 0))
              .slice(0, 15)
              .map(([key, count]) => (
                <li key={key} className="flex justify-between text-zinc-400">
                  <span className="font-mono">{key}</span>
                  <span>{count} changes</span>
                </li>
              ))}
          </ul>
        </div>
      )}
    </div>
  );
}
