import React from 'react';

/**
 * BT execution view: node tree with timing, success/failure status.
 */
export function BTExecutionView({ nodes = [], selectedNodeId, onSelectNode }) {
  const renderNode = (node, depth = 0) => {
    const isSelected = node.id === selectedNodeId;
    const statusColor = node.status === 'success' ? 'text-emerald-400' : node.status === 'failure' ? 'text-red-400' : 'text-zinc-400';
    return (
      <div key={node.id} className="pl-4" style={{ marginLeft: depth * 12 }}>
        <button
          type="button"
          onClick={() => onSelectNode?.(node.id)}
          className={`text-left w-full rounded px-2 py-1 text-xs hover:bg-zinc-800/50 ${isSelected ? 'bg-zinc-700' : ''}`}
        >
          <span className="font-mono text-zinc-300">{node.name ?? node.id}</span>
          <span className={`ml-2 ${statusColor}`}>{node.status ?? '–'}</span>
          {node.durationMs != null && (
            <span className="ml-2 text-zinc-500">{node.durationMs.toFixed(0)}ms</span>
          )}
        </button>
        {(node.children || []).map((c) => renderNode(c, depth + 1))}
      </div>
    );
  };

  return (
    <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-3 max-h-96 overflow-y-auto">
      <h4 className="text-sm font-medium text-zinc-300 mb-2">Execution tree</h4>
      {nodes.length === 0 ? (
        <p className="text-xs text-zinc-500">No node data. Subscribe to /debugging/bt_performance.</p>
      ) : (
        nodes.map((n) => renderNode(n))
      )}
    </div>
  );
}
