import React, { memo } from 'react';
import { SystemStatusCard } from '../../cards/SystemStatusCard';
import { TelemetryCard } from '../../cards/TelemetryCard';
import { AlertTriangle } from 'lucide-react';
import { getContextualSystems } from './getContextualSystems';

export const OverviewAutonomousView = memo(function OverviewAutonomousView({
  currentState,
  systemStatus,
  telemetry,
  activeMission,
  alerts
}) {
  const systems = getContextualSystems(currentState, systemStatus);

  return (
    <div className="p-4 space-y-4">
      {activeMission && (
        <div className="bg-cyan-900/20 border border-cyan-800 rounded p-4">
          <div className="flex items-center justify-between mb-2">
            <h2 className="text-base font-semibold text-cyan-400">
              Active Mission: {activeMission.name} ({Math.round(activeMission.progress)}% complete)
            </h2>
            <div className="flex items-center gap-2">
              <span className="text-xs text-zinc-400">ETA: {activeMission.eta}</span>
              <span className="text-xs text-zinc-400">Next: {activeMission.nextTask}</span>
              <button className="px-3 py-1 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded">
                Pause
              </button>
              <button className="px-3 py-1 bg-red-900/30 hover:bg-red-900/50 text-red-400 text-xs rounded">
                Abort
              </button>
            </div>
          </div>
          <div className="text-sm text-zinc-300">Current Task: {activeMission.currentTask}</div>
        </div>
      )}

      <div className="grid grid-cols-3 gap-4">
        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">Mission Context</h3>
          <div className="space-y-2 text-sm">
            <div className="flex items-center justify-between">
              <span className="text-zinc-400">Waypoints</span>
              <span className="text-zinc-200 font-medium">{activeMission?.waypoints ?? '0/0'}</span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-zinc-400">Samples</span>
              <span className="text-zinc-200 font-medium">{activeMission?.samples ?? '0/0'}</span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-zinc-400">Analysis</span>
              <span className="text-zinc-200 font-medium">{activeMission?.analysis ?? '0/0'}</span>
            </div>
          </div>
        </div>

        <SystemStatusCard systems={systems} title="System Status" />
        <TelemetryCard telemetry={telemetry} />
      </div>

      <div className="grid grid-cols-3 gap-4">
        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">Camera Feed</h3>
          <div className="bg-zinc-950 rounded aspect-video flex items-center justify-center">
            <div className="text-xs text-yellow-400">MOCK</div>
          </div>
        </div>

        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">Map View</h3>
          <div className="bg-zinc-950 rounded aspect-video flex items-center justify-center">
            <div className="text-xs text-zinc-400">Rover Position & Path</div>
          </div>
        </div>

        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">Alerts</h3>
          <div className="space-y-2">
            {alerts.length > 0 ? (
              alerts.map((alert) => (
                <div key={alert.id} className="flex items-center gap-2 text-xs">
                  <AlertTriangle
                    className={`w-4 h-4 ${alert.type === 'error' ? 'text-red-400' : 'text-yellow-400'}`}
                  />
                  <span className="text-zinc-300">{alert.message}</span>
                </div>
              ))
            ) : (
              <div className="text-xs text-zinc-500">No alerts</div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
});
