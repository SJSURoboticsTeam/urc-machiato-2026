import React, { memo } from 'react';
import { DataDisplay } from './DataDisplay';

const TYPE_COLORS = {
  generator: 'border-green-500/20 bg-green-500/5',
  processor: 'border-blue-500/20 bg-blue-500/5',
  controller: 'border-purple-500/20 bg-purple-500/5',
  actuator: 'border-orange-500/20 bg-orange-500/5',
  bus: 'border-cyan-500/20 bg-cyan-500/5'
};

const TYPE_ICONS = {
  generator: '📡',
  processor: '⚙️',
  controller: '🎛️',
  actuator: '🔧',
  bus: '🔌'
};

/**
 * Side panel showing selected node details and live data. Memoized.
 */
export const NodeInspector = memo(function NodeInspector({ node }) {
  const typeColor = TYPE_COLORS[node.type] || 'border-gray-500/20 bg-gray-500/5';
  const typeIcon = TYPE_ICONS[node.type] || '🔧';

  return (
    <div className={`rounded-lg border p-4 ${typeColor}`}>
      <div className="flex items-center justify-between mb-3">
        <div className="flex items-center gap-2">
          <span className="text-lg">{typeIcon}</span>
          <h4 className="text-sm font-semibold text-zinc-200">{node.label}</h4>
        </div>
        <div
          className={`w-3 h-3 rounded-full ${node.data ? 'bg-green-400' : 'bg-zinc-500'}`}
        />
      </div>

      <div className="space-y-3">
        <div className="text-xs text-zinc-400">
          <div>
            Type: <span className="text-zinc-300 capitalize">{node.type}</span>
          </div>
          <div>
            Connections: <span className="text-zinc-300">{node.connections.length}</span>
          </div>
          {node.lastUpdate && (
            <div>
              Last Update:{' '}
              <span className="text-zinc-300">
                {new Date(node.lastUpdate).toLocaleTimeString()}
              </span>
            </div>
          )}
        </div>

        {node.data ? (
          <div className="bg-zinc-800/50 rounded p-3">
            <h5 className="text-xs font-medium text-zinc-300 mb-2">Live Data</h5>
            <DataDisplay data={node.data} compact />
          </div>
        ) : (
          <div className="bg-zinc-800/50 rounded p-3 text-center">
            <div className="text-zinc-500 text-sm">No data available</div>
          </div>
        )}

        <div className="bg-zinc-800/50 rounded p-3">
          <h5 className="text-xs font-medium text-zinc-300 mb-2">Connections</h5>
          <div className="space-y-1">
            {node.connections.map((connId) => (
              <div
                key={connId}
                className="text-xs text-zinc-400 flex items-center gap-2"
              >
                <div className="w-1.5 h-1.5 bg-zinc-500 rounded-full" />
                {connId.replace(/_/g, ' ').replace(/\b\w/g, (l) => l.toUpperCase())}
              </div>
            ))}
            {node.connections.length === 0 && (
              <div className="text-xs text-zinc-500 italic">No outgoing connections</div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
});
