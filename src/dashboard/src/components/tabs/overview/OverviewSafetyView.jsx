import React, { memo } from 'react';
import { SystemStatusCard } from '../../cards/SystemStatusCard';
import { AlertTriangle } from 'lucide-react';
import { SystemState } from '../../../config/stateDefinitions';
import { getContextualSystems } from './getContextualSystems';

export const OverviewSafetyView = memo(function OverviewSafetyView({
  currentState,
  systemStatus,
  requestStateTransition
}) {
  const systems = getContextualSystems(currentState, systemStatus);

  return (
    <div className="p-4 space-y-4">
      <div className="bg-yellow-900/20 border border-yellow-800 rounded p-4">
        <div className="flex items-center gap-2 mb-2">
          <AlertTriangle className="w-5 h-5 text-yellow-400" />
          <h2 className="text-base font-semibold text-yellow-400">
            Safety Alert:{' '}
            {systemStatus?.navigation === 'degraded' ? 'Navigation system degraded' : 'System in safety mode'}
          </h2>
        </div>
        <div className="text-sm text-zinc-300 mb-3">
          {systemStatus?.navigation === 'degraded'
            ? 'GPS accuracy below threshold (HDOP: 2.5). Autonomous navigation limited.'
            : 'System has entered safety mode. Review status before continuing.'}
        </div>
        <div className="flex items-center gap-2">
          <button
            onClick={() => requestStateTransition(SystemState.TELEOPERATION, 'Switch to teleop from safety')}
            className="px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded"
          >
            Switch to Teleop
          </button>
          <button
            onClick={() => requestStateTransition(SystemState.IDLE, 'Continue with caution')}
            className="px-3 py-1.5 bg-yellow-900/30 hover:bg-yellow-900/50 text-yellow-400 text-sm rounded"
          >
            Continue with Caution
          </button>
        </div>
      </div>

      <div className="grid grid-cols-3 gap-4">
        <SystemStatusCard systems={systems} title="Affected Systems" />

        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">Recommended Actions</h3>
          <div className="space-y-2">
            <button className="w-full px-3 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded text-left">
              Switch Mode
            </button>
            <button className="w-full px-3 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded text-left">
              Calibrate GPS
            </button>
            <button className="w-full px-3 py-2 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-sm rounded text-left">
              View Details
            </button>
          </div>
        </div>

        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">System Status</h3>
          <div className="space-y-2 text-sm">
            <div className="flex items-center justify-between">
              <span className="text-zinc-400">Overall</span>
              <span className="text-yellow-400 font-medium">DEGRADED</span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-zinc-400">Safety</span>
              <span className="text-green-400 font-medium">SAFE</span>
            </div>
            <div className="flex items-center justify-between">
              <span className="text-zinc-400">Capability</span>
              <span className="text-yellow-400 font-medium">LIMITED</span>
            </div>
          </div>
        </div>
      </div>
    </div>
  );
});
