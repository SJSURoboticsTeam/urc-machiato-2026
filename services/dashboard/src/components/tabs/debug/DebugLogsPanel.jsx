import React from 'react';
import { Search } from 'lucide-react';
import { VirtualizedLogList } from '../../ui/VirtualizedLogList';

/**
 * Logs panel with level/component filters and virtualized list.
 * @param {Object} props
 * @param {string} props.currentState - Current state machine state (for context label)
 * @param {Array<{ time: string, level: string, component: string, message: string }>} props.logs
 * @param {Array} props.filteredLogs
 * @param {string} props.logLevel
 * @param {function(string): void} props.setLogLevel
 * @param {string} props.logComponent
 * @param {function(string): void} props.setLogComponent
 * @param {function(): void} [props.onClear]
 */
export function DebugLogsPanel({
  currentState,
  logs,
  filteredLogs,
  logLevel,
  setLogLevel,
  logComponent,
  setLogComponent,
  onClear
}) {
  return (
    <div className="space-y-4">
      <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
        <div className="flex items-center justify-between mb-4">
          <h3 className="text-sm font-semibold text-zinc-200">
            Logs (Context: Current State = {currentState})
          </h3>
          <div className="flex items-center gap-2">
            <div className="flex items-center gap-1 px-2 py-1 bg-zinc-800 rounded">
              <Search className="w-3 h-3 text-zinc-400" />
              <input
                type="text"
                placeholder="Search..."
                className="bg-transparent border-0 text-xs text-zinc-200 placeholder-zinc-500 focus:outline-none w-24"
              />
            </div>
            <select
              value={logLevel}
              onChange={(e) => setLogLevel(e.target.value)}
              className="px-2 py-1 bg-zinc-800 border border-zinc-700 rounded text-xs text-zinc-200"
            >
              <option value="all">Level: All</option>
              <option value="ERROR">ERROR</option>
              <option value="WARN">WARN</option>
              <option value="INFO">INFO</option>
              <option value="DEBUG">DEBUG</option>
            </select>
            <select
              value={logComponent}
              onChange={(e) => setLogComponent(e.target.value)}
              className="px-2 py-1 bg-zinc-800 border border-zinc-700 rounded text-xs text-zinc-200"
            >
              <option value="all">Component: All</option>
              <option value="StateMachine">StateMachine</option>
              <option value="Navigation">Navigation</option>
              <option value="Vision">Vision</option>
              <option value="CAN">CAN</option>
              <option value="Safety">Safety</option>
            </select>
            <span className="text-xs text-zinc-400">Last 100</span>
            {onClear && (
              <button
                type="button"
                onClick={onClear}
                className="px-2 py-1 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded"
              >
                Clear
              </button>
            )}
          </div>
        </div>
        <div className="max-h-96 overflow-hidden">
          <VirtualizedLogList logs={filteredLogs} height={384} itemHeight={28} />
        </div>
      </div>
    </div>
  );
}
