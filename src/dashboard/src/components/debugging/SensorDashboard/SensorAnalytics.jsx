import React from 'react';
import { SENSOR_IDS, SENSOR_LABELS } from '../constants';

/**
 * Advanced sensor analytics: noise (std dev, outliers), fusion contribution, correlation matrix, performance (Hz, latency, success rate).
 */
export function SensorAnalytics({
  noiseMetrics = {},
  fusionContribution = {},
  correlationMatrix = {},
  performanceMetrics = {}
}) {
  const sensors = SENSOR_IDS;

  const getNoise = (id) => noiseMetrics[id] ?? { stdDev: 0, outlierCount: 0, signalQuality: 1 };
  const getFusion = (id) => fusionContribution[id] ?? 0;
  const getPerf = (id) => performanceMetrics[id] ?? { hz: 0, latencyMs: 0, successRate: 1 };

  return (
    <div className="grid gap-4 md:grid-cols-2">
      <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4">
        <h4 className="mb-3 text-sm font-semibold text-zinc-200">Noise & signal quality</h4>
        <div className="space-y-2">
          {sensors.map((id) => {
            const n = getNoise(id);
            return (
              <div key={id} className="flex items-center justify-between text-xs">
                <span className="text-zinc-400">{SENSOR_LABELS[id] || id}</span>
                <span className="text-zinc-300">
                  std={n.stdDev.toFixed(3)} | outliers={n.outlierCount} | quality={(n.signalQuality * 100).toFixed(0)}%
                </span>
              </div>
            );
          })}
        </div>
      </div>

      <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4">
        <h4 className="mb-3 text-sm font-semibold text-zinc-200">Fusion contribution</h4>
        <div className="space-y-2">
          {sensors.map((id) => {
            const c = getFusion(id);
            return (
              <div key={id} className="flex items-center justify-between text-xs">
                <span className="text-zinc-400">{SENSOR_LABELS[id] || id}</span>
                <div className="flex items-center gap-2">
                  <div className="h-2 w-24 overflow-hidden rounded bg-zinc-700">
                    <div
                      className="h-full bg-blue-500"
                      style={{ width: `${c * 100}%` }}
                    />
                  </div>
                  <span className="text-zinc-300">{(c * 100).toFixed(0)}%</span>
                </div>
              </div>
            );
          })}
        </div>
      </div>

      <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4 md:col-span-2">
        <h4 className="mb-3 text-sm font-semibold text-zinc-200">Performance (update rate, latency, success)</h4>
        <div className="overflow-x-auto">
          <table className="w-full text-xs">
            <thead>
              <tr className="border-b border-zinc-700 text-zinc-400">
                <th className="py-2 text-left">Sensor</th>
                <th className="py-2 text-right">Hz</th>
                <th className="py-2 text-right">Latency (ms)</th>
                <th className="py-2 text-right">Success %</th>
              </tr>
            </thead>
            <tbody>
              {sensors.map((id) => {
                const p = getPerf(id);
                return (
                  <tr key={id} className="border-b border-zinc-800">
                    <td className="py-1.5 text-zinc-300">{SENSOR_LABELS[id] || id}</td>
                    <td className="py-1.5 text-right text-zinc-300">{p.hz.toFixed(1)}</td>
                    <td className="py-1.5 text-right text-zinc-300">{p.latencyMs.toFixed(0)}</td>
                    <td className="py-1.5 text-right text-zinc-300">{(p.successRate * 100).toFixed(1)}</td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
      </div>

      <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4 md:col-span-2">
        <h4 className="mb-3 text-sm font-semibold text-zinc-200">Correlation matrix (cross-sensor)</h4>
        <div className="overflow-x-auto">
          <table className="w-full text-xs">
            <thead>
              <tr className="border-b border-zinc-700 text-zinc-400">
                <th className="p-1 text-left"></th>
                {sensors.slice(0, 6).map((id) => (
                  <th key={id} className="p-1 text-center">
                    {(SENSOR_LABELS[id] || id).slice(0, 6)}
                  </th>
                ))}
              </tr>
            </thead>
            <tbody>
              {sensors.slice(0, 6).map((id1) => (
                <tr key={id1} className="border-b border-zinc-800">
                  <td className="p-1 text-zinc-400">{(SENSOR_LABELS[id1] || id1).slice(0, 8)}</td>
                  {sensors.slice(0, 6).map((id2) => {
                    const r = id1 === id2 ? 1 : (correlationMatrix[id1]?.[id2] ?? 0);
                    const intensity = Math.abs(r);
                    const bg = intensity > 0.7 ? 'bg-blue-500/30' : intensity > 0.3 ? 'bg-zinc-500/20' : '';
                    return (
                      <td key={id2} className={`p-1 text-center text-zinc-300 ${bg}`}>
                        {r.toFixed(2)}
                      </td>
                    );
                  })}
                </tr>
              ))}
            </tbody>
          </table>
        </div>
      </div>
    </div>
  );
}
