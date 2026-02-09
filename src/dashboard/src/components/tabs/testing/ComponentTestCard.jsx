import React, { memo } from 'react';
import { CheckCircle2, XCircle, Loader2 } from 'lucide-react';
import { getStatusTextColor } from '../../../utils/statusUtils';

function getStatusIcon(status) {
  switch (status) {
    case 'passed':
      return <CheckCircle2 className="w-4 h-4 text-green-400" />;
    case 'failed':
      return <XCircle className="w-4 h-4 text-red-400" />;
    case 'running':
      return <Loader2 className="w-4 h-4 text-blue-400 animate-spin" />;
    default:
      return <div className="w-4 h-4 rounded-full border-2 border-zinc-500" />;
  }
}

export const ComponentTestCard = memo(function ComponentTestCard({ component }) {
  const Icon = component.icon;
  const statusColor = getStatusTextColor(component.status);

  return (
    <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
      <div className="flex items-center justify-between mb-4">
        <div className="flex items-center gap-3">
          <Icon className="w-5 h-5 text-zinc-400" />
          <div>
            <h3 className="text-sm font-semibold text-zinc-200 flex items-center gap-2">
              {component.name.toUpperCase()}
              {component.isMock && (
                <span className="text-xs text-yellow-400">(MOCK)</span>
              )}
            </h3>
            <div className={`text-xs font-medium ${statusColor}`}>
              {component.status === 'mock' ? 'Mock Data' : component.status.toUpperCase()}
            </div>
          </div>
        </div>
        <div className="text-xs text-zinc-400">
          {component.status === 'operational' || component.status === 'ready'
            ? 'Operational'
            : component.status}
        </div>
      </div>

      {component.isMock && (
        <div className="mb-4 p-3 bg-yellow-900/10 border border-yellow-800/50 rounded">
          <div className="text-xs text-yellow-400 mb-2">
            Using simulated data - NOT REAL HARDWARE
          </div>
          <div className="grid grid-cols-2 gap-2 text-xs">
            <div>
              <div className="text-zinc-400">IMU</div>
              <div className="text-zinc-200">[0.1, 0.2, 9.8] m/s</div>
            </div>
            <div>
              <div className="text-zinc-400">GPS</div>
              <div className="text-zinc-200">38.406, -110.792</div>
            </div>
            <div>
              <div className="text-zinc-400">Battery</div>
              <div className="text-zinc-200">85% (25.2V)</div>
            </div>
            <div>
              <div className="text-zinc-400">Motors</div>
              <div className="text-zinc-200">0.0, 0.0 rad/s</div>
            </div>
          </div>
          <div className="flex items-center gap-2 mt-3">
            <button className="px-2 py-1 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded">
              Simulate Failure
            </button>
            <button className="px-2 py-1 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded">
              Adjust Params
            </button>
            <button className="px-2 py-1 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded">
              Details
            </button>
          </div>
        </div>
      )}

      {!component.isMock && (
        <>
          <div className="grid grid-cols-4 gap-4 mb-4 text-xs">
            <div>
              <div className="text-zinc-400">Tests</div>
              <div className="text-zinc-200 font-medium">{component.total} total</div>
            </div>
            <div>
              <div className="text-zinc-400">Passed</div>
              <div className="text-green-400 font-medium">{component.passed}</div>
            </div>
            <div>
              <div className="text-zinc-400">Failed</div>
              <div className="text-red-400 font-medium">{component.failed}</div>
            </div>
            <div>
              <div className="text-zinc-400">Running</div>
              <div className="text-blue-400 font-medium">{component.running}</div>
            </div>
          </div>
          <div className="flex flex-wrap gap-2 mb-4">
            {component.tests.map((test) => (
              <div
                key={test.id}
                className="flex items-center gap-1.5 px-2 py-1 bg-zinc-800 rounded text-xs"
              >
                {getStatusIcon(test.status)}
                <span className="text-zinc-300">{test.name}</span>
              </div>
            ))}
          </div>
          <div className="flex items-center gap-2">
            <button className="px-3 py-1.5 bg-cyan-600 hover:bg-cyan-700 text-white text-xs rounded">
              Run All
            </button>
            <button className="px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded">
              Export
            </button>
            <button className="px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded">
              Details
            </button>
          </div>
        </>
      )}
    </div>
  );
});
