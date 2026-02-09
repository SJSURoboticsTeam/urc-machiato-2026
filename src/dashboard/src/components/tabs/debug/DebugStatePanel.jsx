import React from 'react';
import { SystemState } from '../../../config/stateDefinitions';

/**
 * State machine panel: current state, time in state, transitions, preconditions.
 * @param {Object} props
 * @param {string} props.currentState
 * @param {function(string, string): void} props.requestStateTransition
 * @param {boolean} [props.isTransitioning]
 * @param {Array<{ name: string, rate: number, size: string }>} [props.topics]
 */
export function DebugStatePanel({
  currentState,
  requestStateTransition,
  isTransitioning = false,
  topics = []
}) {
  const getValidTransitions = () => {
    const transitions = {
      [SystemState.IDLE]: [SystemState.AUTONOMOUS, SystemState.TELEOPERATION, SystemState.SAFETY],
      [SystemState.AUTONOMOUS]: [SystemState.IDLE, SystemState.TELEOPERATION, SystemState.SAFETY],
      [SystemState.TELEOPERATION]: [SystemState.IDLE, SystemState.AUTONOMOUS, SystemState.SAFETY],
      [SystemState.SAFETY]: [SystemState.IDLE, SystemState.SHUTDOWN]
    };
    return transitions[currentState] || [];
  };

  return (
    <div className="grid grid-cols-2 gap-4">
      <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
        <h3 className="text-sm font-semibold text-zinc-200 mb-4">State Machine</h3>
        <div className="space-y-3">
          <div>
            <div className="text-xs text-zinc-400">Current</div>
            <div className="text-lg font-bold text-green-400">{currentState}</div>
          </div>
          <div>
            <div className="text-xs text-zinc-400">Time in State</div>
            <div className="text-sm text-zinc-200">45.2s</div>
          </div>
          <div className="pt-3 border-t border-zinc-800">
            <div className="text-xs text-zinc-400 mb-2">Transitions:</div>
            <div className="space-y-1">
              {getValidTransitions().map((state) => (
                <button
                  key={state}
                  type="button"
                  onClick={() => requestStateTransition(state, 'Debug tab transition')}
                  disabled={isTransitioning}
                  className="block w-full text-left px-2 py-1 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded transition-colors disabled:opacity-50"
                >
                  → {state}
                </button>
              ))}
            </div>
          </div>
          <div className="pt-3 border-t border-zinc-800">
            <div className="text-xs text-zinc-400 mb-2">Preconditions:</div>
            <div className="space-y-1 text-xs">
              <div className="flex items-center gap-2">
                <span className="text-green-400">✓</span>
                <span className="text-zinc-300">Boot Complete</span>
              </div>
              <div className="flex items-center gap-2">
                <span className="text-green-400">✓</span>
                <span className="text-zinc-300">Communication OK</span>
              </div>
              <div className="flex items-center gap-2">
                <span className="text-yellow-400">⚠</span>
                <span className="text-zinc-300">Calibration Pending</span>
              </div>
            </div>
          </div>
        </div>
      </div>

      <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
        <h3 className="text-sm font-semibold text-zinc-200 mb-4">ROS2 Topics</h3>
        <div className="space-y-2">
          <div className="grid grid-cols-3 gap-2 text-xs text-zinc-400 border-b border-zinc-800 pb-2">
            <div>Topic</div>
            <div>Rate</div>
            <div>Size</div>
          </div>
          {topics.length > 0
            ? topics.map((topic, idx) => (
                <div key={topic.name || idx} className="grid grid-cols-3 gap-2 text-xs">
                  <div className="text-zinc-200 font-mono">{topic.name}</div>
                  <div className="text-zinc-400">{topic.rate} Hz</div>
                  <div className="text-zinc-400">{topic.size}</div>
                </div>
              ))
            : (
                <div className="text-xs text-zinc-500">(Topics in Topics panel)</div>
              )}
        </div>
      </div>
    </div>
  );
}
