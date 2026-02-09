import React, { useRef } from 'react';
import { Network, Settings } from 'lucide-react';
import {
  useNetworkData,
  NetworkNode,
  NetworkConnection,
  NodeInspector,
  NetworkHeader,
  NetworkLegend
} from './network';

/**
 * Network tab: topology graph and node inspector.
 * State and subscriptions live in useNetworkData; this component stays thin.
 */
export const NetworkTab = () => {
  const canvasRef = useRef(null);
  const {
    networkNodes,
    connections,
    selectedNode,
    setSelectedNode
  } = useNetworkData();

  const nodeList = Object.values(networkNodes);

  return (
    <div className="h-full flex flex-col bg-zinc-950">
      <NetworkHeader connections={connections} />

      <div className="flex-1 flex">
        <div className="flex-1 relative">
          <div className="absolute inset-0 bg-gradient-to-br from-zinc-900 to-zinc-950">
            <svg
              ref={canvasRef}
              className="w-full h-full"
              style={{ minHeight: '500px' }}
            >
              {nodeList.map((node) =>
                node.connections.map((targetId) => {
                  const targetNode = networkNodes[targetId];
                  if (!targetNode) return null;
                  return (
                    <NetworkConnection
                      key={`${node.id}-${targetId}`}
                      from={node.position}
                      to={targetNode.position}
                      active={!!(node.data && targetNode.data)}
                    />
                  );
                })
              )}

              {nodeList.map((node) => (
                <NetworkNode
                  key={node.id}
                  node={node}
                  isSelected={selectedNode === node.id}
                  onClick={() =>
                    setSelectedNode(selectedNode === node.id ? null : node.id)
                  }
                />
              ))}
            </svg>

            <NetworkLegend />
          </div>
        </div>

        <div className="w-80 bg-zinc-900 border-l border-zinc-800 flex flex-col">
          <div className="p-4 border-b border-zinc-800">
            <h3 className="text-sm font-semibold text-zinc-200 flex items-center gap-2">
              <Settings className="w-4 h-4" />
              Node Inspector
            </h3>
            <p className="text-xs text-zinc-400 mt-1">
              {selectedNode
                ? 'Click nodes to inspect data'
                : 'Select a node to view details'}
            </p>
          </div>

          <div className="flex-1 overflow-y-auto p-4">
            {selectedNode ? (
              <NodeInspector node={networkNodes[selectedNode]} />
            ) : (
              <div className="text-center py-8 text-zinc-500">
                <Network className="w-8 h-8 mx-auto mb-3 opacity-50" />
                <p className="text-sm">Select a node to inspect</p>
              </div>
            )}
          </div>
        </div>
      </div>
    </div>
  );
};
