import React, { memo } from 'react';

/**
 * Network status summary panel. Memoized for stable props.
 * @param {Object} props
 * @param {number} props.errorCount
 */
export const DebugNetworkPanel = memo(function DebugNetworkPanel({ errorCount = 0 }) {
  return (
    <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
      <h3 className="text-sm font-semibold text-zinc-200 mb-4">Network Status</h3>
      <div className="space-y-3 text-sm">
        <div className="flex items-center justify-between">
          <span className="text-zinc-400">WebSocket</span>
          <span className="text-green-400 font-medium">Connected</span>
        </div>
        <div className="flex items-center justify-between">
          <span className="text-zinc-400">Latency</span>
          <span className="text-zinc-200 font-medium">12ms</span>
        </div>
        <div className="flex items-center justify-between">
          <span className="text-zinc-400">Messages/s</span>
          <span className="text-zinc-200 font-medium">248</span>
        </div>
        <div className="flex items-center justify-between">
          <span className="text-zinc-400">Errors</span>
          <span className="text-red-400 font-medium">{errorCount}</span>
        </div>
      </div>
    </div>
  );
});
