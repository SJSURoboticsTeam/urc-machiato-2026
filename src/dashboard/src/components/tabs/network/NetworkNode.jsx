import React, { memo } from 'react';

const NODE_COLORS = {
  generator: 'fill-green-500 stroke-green-400',
  processor: 'fill-blue-500 stroke-blue-400',
  controller: 'fill-purple-500 stroke-purple-400',
  actuator: 'fill-orange-500 stroke-orange-400',
  bus: 'fill-cyan-500 stroke-cyan-400'
};

/**
 * Single node in the network graph. Memoized for 60fps with many nodes.
 */
export const NetworkNode = memo(function NetworkNode({ node, isSelected, onClick }) {
  const colorClass = NODE_COLORS[node.type] || 'fill-gray-500 stroke-gray-400';
  const isActive =
    node.data && node.lastUpdate && Date.now() - node.lastUpdate < 2000;

  return (
    <g
      className="cursor-pointer transition-transform hover:scale-110"
      onClick={onClick}
    >
      <circle
        cx={node.position.x}
        cy={node.position.y}
        r="28"
        className={`${colorClass} transition-all duration-300 ${
          isSelected ? 'stroke-2 stroke-cyan-400' : 'stroke-1'
        } ${isActive ? 'animate-pulse' : ''} filter drop-shadow-lg`}
      />

      {isSelected && (
        <circle
          cx={node.position.x}
          cy={node.position.y}
          r="35"
          className="fill-none stroke-cyan-400 stroke-2 animate-ping"
        />
      )}

      <text
        x={node.position.x}
        y={node.position.y + 42}
        textAnchor="middle"
        className="text-xs fill-zinc-300 font-medium pointer-events-none"
      >
        {node.label}
      </text>

      {node.data && (
        <circle
          cx={node.position.x + 24}
          cy={node.position.y - 24}
          r="5"
          className="fill-green-400 animate-ping opacity-75"
        />
      )}

      {node.connections.length > 0 && (
        <text
          x={node.position.x + 20}
          y={node.position.y - 15}
          textAnchor="middle"
          className="text-xs fill-zinc-400 font-bold"
        >
          {node.connections.length}
        </text>
      )}
    </g>
  );
});
