import React, { useState } from 'react';
import { useStateMachineContext } from '../../context/StateMachineContext';
import { BTExecutionView, BTPerformanceChart } from '../debugging/BehaviorTreeAnalytics';
import { StateMachineGraph } from './cognition';
import { STATE_METADATA } from '../../config/stateDefinitions';

/** Mock BT tree for when /bt/telemetry or /debugging/bt_performance has no per-node data. */
const MOCK_BT_NODES = [
  {
    id: 'root',
    name: 'Root',
    status: 'running',
    durationMs: 120,
    children: [
      { id: 'nav', name: 'Navigate', status: 'running', durationMs: 85, children: [] },
      { id: 'sense', name: 'Sense', status: 'success', durationMs: 25, children: [] }
    ]
  }
];

const MOCK_NODE_TIMINGS = [
  { id: 'Navigate', name: 'Navigate', durationMs: 85, count: 10 },
  { id: 'Sense', name: 'Sense', durationMs: 25, count: 10 },
  { id: 'CheckBattery', name: 'CheckBattery', durationMs: 2, count: 10 }
];

/**
 * Cognition tab: Groot-like behavior tree view and state machine graph.
 * Replaces the former Network tab content.
 */
export function CognitionTab() {
  const [selectedNodeId, setSelectedNodeId] = useState(null);
  const { currentState, requestStateTransition, isTransitioning } = useStateMachineContext();
  const meta = currentState ? STATE_METADATA[currentState] : null;
  const validTransitions = meta?.allowedTransitions ?? [];

  return (
    <div className="p-4 space-y-6">
      <header>
        <h2 className="text-lg font-semibold text-zinc-100">Cognition</h2>
        <p className="text-sm text-zinc-400">
          Behavior tree execution and system state machine. Connect ROS for live data.
        </p>
      </header>

      <div className="grid grid-cols-1 lg:grid-cols-2 gap-6">
        {/* Behavior tree (Groot-style) */}
        <section className="space-y-3">
          <h3 className="text-sm font-medium text-zinc-300">Behavior tree</h3>
          <BTExecutionView
            nodes={MOCK_BT_NODES}
            selectedNodeId={selectedNodeId}
            onSelectNode={setSelectedNodeId}
          />
          <BTPerformanceChart nodeTimings={MOCK_NODE_TIMINGS} />
        </section>

        {/* State machine graph */}
        <section className="space-y-3">
          <h3 className="text-sm font-medium text-zinc-300">State machine</h3>
          <StateMachineGraph currentState={currentState ?? undefined} />
          {validTransitions.length > 0 && (
            <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-3">
              <h4 className="text-xs font-medium text-zinc-400 mb-2">Valid transitions</h4>
              <div className="flex flex-wrap gap-2">
                {validTransitions.map((targetState) => {
                  const targetMeta = STATE_METADATA[targetState];
                  const label = targetMeta?.displayName ?? targetState;
                  return (
                    <button
                      key={targetState}
                      type="button"
                      onClick={() => requestStateTransition(targetState, 'Cognition tab transition')}
                      disabled={isTransitioning}
                      className="px-3 py-1.5 text-xs rounded bg-zinc-700 hover:bg-zinc-600 text-zinc-200 disabled:opacity-50 transition-colors"
                    >
                      → {label}
                    </button>
                  );
                })}
              </div>
            </div>
          )}
        </section>
      </div>
    </div>
  );
}
