import React, { memo } from 'react';
import { Target } from 'lucide-react';
import { TelemetryCard } from '../../cards/TelemetryCard';

/**
 * Mission execution view when state is AUTONOMOUS and activeMission is set.
 * Memoized for stable props.
 */
export const MissionExecutionView = memo(function MissionExecutionView({
  activeMission,
  telemetry
}) {
  return (
    <div className="p-4 space-y-4">
      <div className="bg-cyan-900/20 border border-cyan-800 rounded p-4">
        <div className="flex items-center justify-between">
          <div>
            <h2 className="text-lg font-semibold text-cyan-400 flex items-center gap-2">
              <Target className="w-5 h-5" />
              {activeMission.name.toUpperCase()} MISSION -{' '}
              {Math.round(activeMission.progress)}% Complete
            </h2>
            <div className="flex items-center gap-4 mt-2 text-sm text-zinc-400">
              <span>Phase: Sample Collection</span>
              <span>ETA: {activeMission.eta}</span>
              <span>Next: {activeMission.nextTask}</span>
            </div>
          </div>
          <div className="flex items-center gap-2">
            <button
              type="button"
              className="px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded"
            >
              Pause
            </button>
            <button
              type="button"
              className="px-3 py-1.5 bg-red-900/30 hover:bg-red-900/50 text-red-400 text-sm rounded"
            >
              Abort
            </button>
          </div>
        </div>
      </div>

      <div className="grid grid-cols-3 gap-4">
        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">Progress</h3>
          <div className="space-y-3">
            <div>
              <div className="flex items-center justify-between text-xs mb-1">
                <span className="text-zinc-400">Waypoints</span>
                <span className="text-zinc-200">{activeMission.waypoints}</span>
              </div>
              <div className="w-full bg-zinc-800 rounded-full h-1.5">
                <div
                  className="bg-cyan-500 h-1.5 rounded-full"
                  style={{ width: '67%' }}
                />
              </div>
            </div>
            <div>
              <div className="flex items-center justify-between text-xs mb-1">
                <span className="text-zinc-400">Samples</span>
                <span className="text-zinc-200">{activeMission.samples}</span>
              </div>
              <div className="w-full bg-zinc-800 rounded-full h-1.5">
                <div
                  className="bg-cyan-500 h-1.5 rounded-full"
                  style={{ width: '33%' }}
                />
              </div>
            </div>
            <div>
              <div className="flex items-center justify-between text-xs mb-1">
                <span className="text-zinc-400">Analysis</span>
                <span className="text-zinc-200">{activeMission.analysis}</span>
              </div>
              <div className="w-full bg-zinc-800 rounded-full h-1.5">
                <div
                  className="bg-zinc-700 h-1.5 rounded-full"
                  style={{ width: '0%' }}
                />
              </div>
            </div>
          </div>
        </div>

        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">Details</h3>
          <div className="space-y-2 text-sm">
            <div>
              <div className="text-zinc-400">Current</div>
              <div className="text-zinc-200 font-medium">WP #2</div>
            </div>
            <div>
              <div className="text-zinc-400">Next</div>
              <div className="text-zinc-200 font-medium">{activeMission.nextTask}</div>
            </div>
            <div>
              <div className="text-zinc-400">Status</div>
              <div className="text-green-400 font-medium">Active</div>
            </div>
          </div>
        </div>

        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">Controls</h3>
          <div className="space-y-2">
            <button
              type="button"
              className="w-full px-3 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded"
            >
              Pause
            </button>
            <button
              type="button"
              className="w-full px-3 py-2 bg-red-900/30 hover:bg-red-900/50 text-red-400 text-sm rounded"
            >
              Abort
            </button>
            <button
              type="button"
              className="w-full px-3 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded"
            >
              Details →
            </button>
          </div>
        </div>
      </div>

      <div className="grid grid-cols-3 gap-4">
        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">Map View</h3>
          <div className="bg-zinc-950 rounded aspect-video flex items-center justify-center">
            <div className="text-xs text-zinc-400">Interactive Map</div>
          </div>
          <div className="mt-2 text-xs text-zinc-400">
            • Current Position<br />
            • Waypoints<br />• Path
          </div>
        </div>

        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">Camera</h3>
          <div className="bg-zinc-950 rounded aspect-video flex items-center justify-center">
            <div className="text-xs text-yellow-400">MOCK</div>
          </div>
        </div>

        <TelemetryCard telemetry={telemetry} />
      </div>
    </div>
  );
});
