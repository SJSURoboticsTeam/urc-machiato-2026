import React, { memo } from 'react';

/**
 * Connection line between two nodes. Memoized for stable from/to/active.
 */
export const NetworkConnection = memo(function NetworkConnection({ from, to, active }) {
  const midX = (from.x + to.x) / 2;
  const midY = (from.y + to.y) / 2;

  return (
    <g>
      <line
        x1={from.x}
        y1={from.y}
        x2={to.x}
        y2={to.y}
        className={`stroke-2 transition-colors duration-300 ${
          active ? 'stroke-green-400' : 'stroke-zinc-600'
        }`}
        strokeWidth="2"
        markerEnd="url(#arrowhead)"
      />

      {active && (
        <circle
          cx={midX}
          cy={midY}
          r="4"
          className="fill-blue-400 animate-bounce"
          style={{ animationDuration: '1.5s', animationDirection: 'alternate' }}
        />
      )}

      <defs>
        <marker
          id="arrowhead"
          markerWidth="12"
          markerHeight="8"
          refX="11"
          refY="4"
          orient="auto"
        >
          <polygon points="0 0, 12 4, 0 8" className="fill-zinc-400" />
        </marker>
      </defs>
    </g>
  );
});
