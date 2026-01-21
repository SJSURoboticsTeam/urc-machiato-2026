import React, { useState, useCallback } from 'react';
import { GitBranch, ArrowRight, CheckCircle2, XCircle } from 'lucide-react';
import { UI_CONSTANTS } from '../../constants/uiConstants';

/**
 * State Transition Tester Component
 * Handles testing manual state transitions
 */
export const StateTransitionTester = ({
  currentState,
  requestStateTransition,
  stateTransitionResult,
  setStateTransitionResult
}) => {
  const availableStates = UI_CONSTANTS.AVAILABLE_STATES;

  const testStateTransition = useCallback(async (newState) => {
    setStateTransitionResult({ status: 'testing', from: currentState, to: newState });

    try {
      const success = await requestStateTransition(newState);
      setStateTransitionResult({
        status: success ? 'success' : 'failed',
        from: currentState,
        to: newState,
        timestamp: Date.now()
      });
    } catch (error) {
      setStateTransitionResult({
        status: 'error',
        from: currentState,
        to: newState,
        error: error.message,
        timestamp: Date.now()
      });
    }
  }, [currentState, requestStateTransition, setStateTransitionResult]);

  const getStatusIcon = (status) => {
    switch (status) {
      case 'success': return <CheckCircle2 className="w-4 h-4 text-green-400" />;
      case 'failed': return <XCircle className="w-4 h-4 text-red-400" />;
      case 'error': return <XCircle className="w-4 h-4 text-red-400" />;
      case 'testing': return <div className="w-4 h-4 border-2 border-blue-400 border-t-transparent rounded-full animate-spin" />;
      default: return null;
    }
  };

  const getStatusColor = (status) => {
    switch (status) {
      case 'success': return 'text-green-400';
      case 'failed': return 'text-red-400';
      case 'error': return 'text-red-400';
      case 'testing': return 'text-blue-400';
      default: return 'text-zinc-400';
    }
  };

  return (
    <div className="space-y-4">
      <h3 className="text-lg font-semibold text-zinc-200 flex items-center gap-2">
        <GitBranch className="w-5 h-5" />
        State Transition Tester
      </h3>

      {/* Current state display */}
      <div className="bg-zinc-800 p-3 rounded">
        <div className="text-sm text-zinc-400 mb-1">Current State</div>
        <div className="text-lg font-mono text-blue-400">{currentState}</div>
      </div>

      {/* Available states grid */}
      <div className="space-y-2">
        <div className="text-sm text-zinc-400">Available States</div>
        <div className="grid grid-cols-2 gap-2">
          {availableStates.map(state => (
            <button
              key={state}
              onClick={() => testStateTransition(state)}
              disabled={stateTransitionResult?.status === 'testing'}
              className={`p-2 text-left rounded transition-colors ${
                state === currentState
                  ? 'bg-blue-900 border-2 border-blue-500 text-blue-200'
                  : 'bg-zinc-800 hover:bg-zinc-700 text-zinc-300 hover:text-zinc-200'
              } ${stateTransitionResult?.status === 'testing' ? 'opacity-50 cursor-not-allowed' : ''}`}
            >
              <div className="font-mono text-sm">{state}</div>
            </button>
          ))}
        </div>
      </div>

      {/* Transition result */}
      {stateTransitionResult && (
        <div className="bg-zinc-800 p-3 rounded">
          <div className="flex items-center gap-2 mb-2">
            {getStatusIcon(stateTransitionResult.status)}
            <span className={`text-sm font-medium ${getStatusColor(stateTransitionResult.status)}`}>
              {stateTransitionResult.status === 'testing' ? 'Testing...' :
               stateTransitionResult.status === 'success' ? 'Transition Successful' :
               stateTransitionResult.status === 'failed' ? 'Transition Failed' :
               'Transition Error'}
            </span>
          </div>

          {stateTransitionResult.status !== 'testing' && (
            <div className="space-y-1 text-xs text-zinc-400">
              <div className="flex items-center gap-2">
                <span>{stateTransitionResult.from}</span>
                <ArrowRight className="w-3 h-3" />
                <span>{stateTransitionResult.to}</span>
              </div>
              {stateTransitionResult.error && (
                <div className="text-red-400 mt-2 p-2 bg-red-900/20 rounded">
                  {stateTransitionResult.error}
                </div>
              )}
            </div>
          )}
        </div>
      )}
    </div>
  );
};
