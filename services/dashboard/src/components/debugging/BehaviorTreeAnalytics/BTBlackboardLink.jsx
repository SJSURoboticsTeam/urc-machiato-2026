import React from 'react';

/**
 * Data dependency: which BT nodes read/write which blackboard keys; simple causality.
 */
export function BTBlackboardLink({ nodeKeys = [] }) {
  return (
    <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-3">
      <h4 className="text-sm font-medium text-zinc-300 mb-2">Node – Blackboard links</h4>
      {nodeKeys.length === 0 ? (
        <p className="text-xs text-zinc-500">No mapping data. Requires BT metadata or /debugging/bt_performance.</p>
      ) : (
        <ul className="space-y-2 text-xs">
          {nodeKeys.map(({ nodeId, reads = [], writes = [] }) => (
            <li key={nodeId} className="border-b border-zinc-800 pb-2 last:border-b-0">
              <span className="font-mono text-zinc-200">{nodeId}</span>
              <div className="mt-1 pl-2 text-zinc-500">
                {reads.length > 0 && <span>Reads: {reads.join(', ')}</span>}
                {reads.length > 0 && writes.length > 0 && ' | '}
                {writes.length > 0 && <span>Writes: {writes.join(', ')}</span>}
              </div>
            </li>
          ))}
        </ul>
      )}
    </div>
  );
}
