import React from 'react';
import { useROS } from '../../hooks/useROS';
import { useBlackboardState } from '../../hooks/useBlackboardState';
import { BLACKBOARD_DOMAINS, BLACKBOARD_KEY_TYPES } from '../debugging/constants';
import { Eye } from 'lucide-react';

/** Format blackboard value for display */
function formatValue(key, value) {
  if (value === undefined || value === null) return '–';
  const type = BLACKBOARD_KEY_TYPES[key];
  if (type === 'bool') return value ? 'true' : 'false';
  if (type === 'double') return Number(value).toFixed(4);
  return String(value);
}

/**
 * Perception tab: perception-related metrics (confidence, map quality, features, obstacle detection).
 * Replaces the former Analytics tab.
 */
export function PerceptionTab() {
  const { ros, isConnected } = useROS();
  const [blackboardState, loading, refresh] = useBlackboardState(ros, true);
  const perceptionKeys = BLACKBOARD_DOMAINS.perception?.keys ?? [];

  return (
    <div className="p-4 space-y-6">
      <header className="flex items-center justify-between">
        <div>
          <h2 className="text-lg font-semibold text-zinc-100 flex items-center gap-2">
            <Eye className="w-5 h-5 text-cyan-400" />
            Perception
          </h2>
          <p className="text-sm text-zinc-400 mt-0.5">
            Confidence, map quality, features, and obstacle detection. Data from blackboard when ROS is connected.
          </p>
        </div>
        {refresh && (
          <button
            type="button"
            onClick={() => refresh()}
            disabled={loading || !isConnected}
            className="rounded border border-zinc-600 bg-zinc-800 px-3 py-1.5 text-sm text-zinc-200 hover:bg-zinc-700 disabled:opacity-50"
          >
            {loading ? '...' : 'Refresh'}
          </button>
        )}
      </header>

      <div className="grid grid-cols-1 sm:grid-cols-2 lg:grid-cols-4 gap-4">
        {perceptionKeys.map((key) => {
          const value = blackboardState[key];
          const label = key.replace(/_/g, ' ').replace(/\b\w/g, (c) => c.toUpperCase());
          const type = BLACKBOARD_KEY_TYPES[key];
          const numVal = type === 'double' ? Number(value) : null;
          const isConfidence = key.includes('confidence');
          const status = isConfidence && numVal != null
            ? (numVal >= 0.8 ? 'good' : numVal >= 0.5 ? 'warn' : 'low')
            : null;

          return (
            <div
              key={key}
              className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4"
            >
              <div className="text-xs font-medium text-zinc-400 uppercase tracking-wide mb-1">
                {label}
              </div>
              <div className={`text-lg font-mono ${
                status === 'good' ? 'text-emerald-400' :
                status === 'warn' ? 'text-amber-400' :
                status === 'low' ? 'text-red-400' :
                'text-zinc-200'
              }`}>
                {formatValue(key, value)}
              </div>
              {!isConnected && (
                <div className="text-xs text-zinc-500 mt-1">Connect ROS for live data</div>
              )}
            </div>
          );
        })}
      </div>

      {perceptionKeys.length === 0 && (
        <p className="text-sm text-zinc-500">No perception keys defined in BLACKBOARD_DOMAINS.</p>
      )}
    </div>
  );
}
