import React from 'react';

/**
 * Bottleneck summary, resource usage, optimization suggestions.
 */
export function PerformanceAnalyzer({
  bottlenecks = [],
  resourceUsage = { cpu: 0, memory: 0, network: 0 },
  recommendations = []
}) {
  return (
    <div className="space-y-4">
      <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4">
        <h4 className="text-sm font-medium text-zinc-300 mb-2">Resource usage</h4>
        <div className="grid grid-cols-3 gap-2 text-sm">
          <div className="rounded bg-zinc-800/50 p-2 text-center">
            <div className="text-zinc-500">CPU</div>
            <div className="text-zinc-200 font-medium">{(resourceUsage.cpu * 100).toFixed(0)}%</div>
          </div>
          <div className="rounded bg-zinc-800/50 p-2 text-center">
            <div className="text-zinc-500">Memory</div>
            <div className="text-zinc-200 font-medium">{(resourceUsage.memory * 100).toFixed(0)}%</div>
          </div>
          <div className="rounded bg-zinc-800/50 p-2 text-center">
            <div className="text-zinc-500">Network</div>
            <div className="text-zinc-200 font-medium">{(resourceUsage.network * 100).toFixed(0)}%</div>
          </div>
        </div>
      </div>
      {bottlenecks.length > 0 && (
        <div className="rounded-lg border border-amber-500/30 bg-amber-500/10 p-4">
          <h4 className="text-sm font-medium text-amber-400 mb-2">Bottlenecks</h4>
          <ul className="space-y-1 text-xs text-zinc-300">
            {bottlenecks.map((b, i) => (
              <li key={i}>{b}</li>
            ))}
          </ul>
        </div>
      )}
      {recommendations.length > 0 && (
        <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4">
          <h4 className="text-sm font-medium text-zinc-300 mb-2">Recommendations</h4>
          <ul className="space-y-1 text-xs text-zinc-400">
            {recommendations.map((r, i) => (
              <li key={i}>{r}</li>
            ))}
          </ul>
        </div>
      )}
    </div>
  );
}
