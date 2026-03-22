import React, { memo } from 'react';
import { SystemStatusCard } from '../../cards/SystemStatusCard';
import { TelemetryCard } from '../../cards/TelemetryCard';
import { Play, TestTube } from 'lucide-react';
import { SystemState } from '../../../config/stateDefinitions';
import { getContextualSystems } from './getContextualSystems';

export const OverviewIdleView = memo(function OverviewIdleView({
  currentState,
  systemStatus,
  telemetry,
  requestStateTransition
}) {
  const systems = getContextualSystems(currentState, systemStatus);

  return (
    <div className="p-4 space-y-4">
      <div className="grid grid-cols-3 gap-4">
        <SystemStatusCard systems={systems} title="System Status" />

        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">Quick Actions</h3>
          <div className="space-y-2">
            <button
              onClick={() => requestStateTransition(SystemState.AUTONOMOUS, 'Start mission from UI')}
              className="w-full px-3 py-2 bg-cyan-600 hover:bg-cyan-700 text-white text-sm rounded transition-colors flex items-center justify-center gap-2"
            >
              <Play className="w-4 h-4" />
              Start Mission
            </button>
            <button
              onClick={() => {}}
              className="w-full px-3 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded transition-colors flex items-center justify-center gap-2"
            >
              <TestTube className="w-4 h-4" />
              Run Tests
            </button>
            <button
              onClick={() => requestStateTransition(SystemState.CALIBRATION, 'Calibration from UI')}
              className="w-full px-3 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded transition-colors"
            >
              Calibrate
            </button>
          </div>
        </div>

        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">Recent Activity</h3>
          <div className="text-sm text-zinc-400">
            <div className="mb-2">Last: Science Mission</div>
            <div className="text-green-400">Completed</div>
            <div className="text-xs text-zinc-500 mt-1">2h ago</div>
          </div>
        </div>
      </div>

      <div className="grid grid-cols-3 gap-4">
        <TelemetryCard telemetry={telemetry} />

        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">State Machine</h3>
          <div className="text-center py-4">
            <div className="text-2xl font-bold text-green-400 mb-2">{currentState}</div>
            <div className="text-xs text-zinc-400">Ready for:</div>
            <div className="text-xs text-zinc-300 mt-1">Mission</div>
            <div className="text-xs text-zinc-300">Teleop</div>
          </div>
        </div>

        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">Mission Queue</h3>
          <div className="text-sm text-zinc-400 mb-3">Queue: Empty</div>
          <button className="w-full px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded transition-colors">
            + New
          </button>
        </div>
      </div>
    </div>
  );
});
