import React, { useMemo } from 'react';
import { SystemState, STATE_METADATA } from '../../../config/stateDefinitions';

const STATE_ORDER = [
  SystemState.BOOT,
  SystemState.CALIBRATION,
  SystemState.IDLE,
  SystemState.TELEOPERATION,
  SystemState.AUTONOMOUS,
  SystemState.SAFETY,
  SystemState.SHUTDOWN
];

const NODE_STROKE = {
  blue: '#60a5fa',
  yellow: '#fbbf24',
  green: '#34d399',
  cyan: '#22d3ee',
  red: '#f87171',
  orange: '#fb923c',
  gray: '#a1a1aa'
};

const NODE_WIDTH = 100;
const NODE_HEIGHT = 36;
const HORIZONTAL_GAP = 24;
const VERTICAL_GAP = 16;

/**
 * Compute positions for state nodes in a simple left-to-right flow.
 * Returns { nodePositions: Map<state, {x,y}>, edges: [{ from, to }], width, height }.
 */
function computeLayout() {
  const nodePositions = new Map();
  const rowCount = 3;
  let x = 0;
  let y = 0;
  let maxX = 0;
  STATE_ORDER.forEach((state, i) => {
    const row = i % rowCount;
    const col = Math.floor(i / rowCount);
    const xPos = col * (NODE_WIDTH + HORIZONTAL_GAP);
    const yPos = row * (NODE_HEIGHT + VERTICAL_GAP);
    nodePositions.set(state, { x: xPos, y: yPos });
    maxX = Math.max(maxX, xPos + NODE_WIDTH);
  });
  const width = maxX + HORIZONTAL_GAP;
  const height = rowCount * (NODE_HEIGHT + VERTICAL_GAP) - VERTICAL_GAP + 24;

  const edges = [];
  STATE_ORDER.forEach((fromState) => {
    const meta = STATE_METADATA[fromState];
    if (!meta || !meta.allowedTransitions) return;
    meta.allowedTransitions.forEach((toState) => {
      edges.push({ from: fromState, to: toState });
    });
  });

  return { nodePositions, edges, width, height };
}

/**
 * State machine graph: states as nodes, allowedTransitions as edges, current state highlighted.
 */
export function StateMachineGraph({ currentState }) {
  const { nodePositions, edges, width, height } = useMemo(computeLayout, []);

  return (
    <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-3 overflow-auto">
      <h4 className="text-sm font-medium text-zinc-300 mb-3">State machine</h4>
      <svg
        width={width}
        height={height}
        className="min-w-full"
        viewBox={`0 0 ${width} ${height}`}
      >
        <defs>
          <marker
            id="arrowhead"
            markerWidth="8"
            markerHeight="6"
            refX="7"
            refY="3"
            orient="auto"
          >
            <polygon points="0 0, 8 3, 0 6" fill="currentColor" className="text-zinc-500" />
          </marker>
        </defs>
        {/* Edges */}
        {edges.map(({ from: fromState, to: toState }, i) => {
          const fromPos = nodePositions.get(fromState);
          const toPos = nodePositions.get(toState);
          if (!fromPos || !toPos) return null;
          const x1 = fromPos.x + NODE_WIDTH;
          const y1 = fromPos.y + NODE_HEIGHT / 2;
          const x2 = toPos.x;
          const y2 = toPos.y + NODE_HEIGHT / 2;
          const midX = (x1 + x2) / 2;
          const isFromCurrent = fromState === currentState;
          return (
            <path
              key={`${fromState}-${toState}-${i}`}
              d={`M ${x1} ${y1} C ${midX} ${y1}, ${midX} ${y2}, ${x2} ${y2}`}
              fill="none"
              stroke="currentColor"
              strokeWidth="1.5"
              className={isFromCurrent ? 'text-cyan-500' : 'text-zinc-600'}
              markerEnd="url(#arrowhead)"
            />
          );
        })}
        {/* Nodes */}
        {STATE_ORDER.map((state) => {
          const pos = nodePositions.get(state);
          if (!pos) return null;
          const meta = STATE_METADATA[state];
          const displayName = meta?.displayName ?? state;
          const colorKey = (meta?.color ?? 'gray').toLowerCase();
          const strokeColor = NODE_STROKE[colorKey] ?? NODE_STROKE.gray;
          const isCurrent = state === currentState;
          return (
            <g key={state}>
              <rect
                x={pos.x}
                y={pos.y}
                width={NODE_WIDTH}
                height={NODE_HEIGHT}
                rx="6"
                fill="#27272a"
                stroke={isCurrent ? '#22d3ee' : strokeColor}
                strokeWidth={isCurrent ? 2.5 : 1}
              />
              <text
                x={pos.x + NODE_WIDTH / 2}
                y={pos.y + NODE_HEIGHT / 2 + 4}
                textAnchor="middle"
                className="text-xs font-medium fill-zinc-200"
                style={isCurrent ? { fill: '#a5f3fc' } : {}}
              >
                {displayName}
              </text>
            </g>
          );
        })}
      </svg>
      {currentState && (
        <p className="text-xs text-zinc-500 mt-2">
          Current: <span className="text-zinc-300 font-medium">{STATE_METADATA[currentState]?.displayName ?? currentState}</span>
        </p>
      )}
    </div>
  );
}
