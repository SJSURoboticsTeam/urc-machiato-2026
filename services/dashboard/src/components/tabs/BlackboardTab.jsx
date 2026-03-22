import React, { useState } from 'react';
import { useROS } from '../../hooks/useROS';
import { useBlackboardState } from '../../hooks/useBlackboardState';
import { BLACKBOARD_DOMAINS, BLACKBOARD_KEY_TYPES } from '../debugging/constants';
import { BookOpen, Search, ChevronDown, ChevronRight } from 'lucide-react';

function formatValue(key, value) {
  if (value === undefined || value === null) return '–';
  const type = BLACKBOARD_KEY_TYPES[key];
  if (type === 'bool') return value ? 'true' : 'false';
  if (type === 'double' && key.includes('latitude')) return `lat ${Number(value).toFixed(6)}`;
  if (type === 'double' && key.includes('longitude')) return `lon ${Number(value).toFixed(6)}`;
  if (type === 'double') return Number(value).toFixed(4);
  return String(value);
}

/**
 * Blackboard tab: all keys grouped by category in a ledger-style layout.
 * Each category is a "ledger" section with rows: Key | Value | Type.
 */
export function BlackboardTab() {
  const { ros, isConnected } = useROS();
  const [blackboardState, loading, refresh] = useBlackboardState(ros, true);
  const [search, setSearch] = useState('');
  const [expandedCategories, setExpandedCategories] = useState(() =>
    Object.keys(BLACKBOARD_DOMAINS).reduce((acc, k) => ({ ...acc, [k]: true }), {})
  );

  const toggleCategory = (key) => {
    setExpandedCategories((prev) => ({ ...prev, [key]: !prev[key] }));
  };

  const matchesSearch = (keyName, value) => {
    if (!search.trim()) return true;
    const s = search.toLowerCase();
    return keyName.toLowerCase().includes(s) || String(value).toLowerCase().includes(s);
  };

  const categories = Object.entries(BLACKBOARD_DOMAINS);

  return (
    <div className="p-4 space-y-4">
      <header className="flex flex-wrap items-center justify-between gap-3">
        <div>
          <h2 className="text-lg font-semibold text-zinc-100 flex items-center gap-2">
            <BookOpen className="w-5 h-5 text-amber-500" />
            Blackboard
          </h2>
          <p className="text-sm text-zinc-400 mt-0.5">
            All blackboard values by category. Ledger view.
          </p>
        </div>
        <div className="flex items-center gap-2">
          <div className="flex items-center gap-2 rounded border border-zinc-600 bg-zinc-800 px-2 py-1.5 min-w-[200px]">
            <Search className="h-4 w-4 text-zinc-500 shrink-0" />
            <input
              type="text"
              placeholder="Search keys or values..."
              value={search}
              onChange={(e) => setSearch(e.target.value)}
              className="flex-1 bg-transparent text-sm text-zinc-200 placeholder-zinc-500 focus:outline-none min-w-0"
            />
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
        </div>
      </header>

      <div className="space-y-4">
        {categories.map(([domainKey, { label, keys }]) => {
          const expanded = expandedCategories[domainKey];
          const visibleKeys = keys.filter((k) => matchesSearch(k, blackboardState[k]));

          return (
            <div
              key={domainKey}
              className="rounded-lg border border-zinc-700 bg-zinc-900/50 overflow-hidden"
            >
              <button
                type="button"
                onClick={() => toggleCategory(domainKey)}
                className="flex w-full items-center gap-2 px-4 py-3 text-left font-medium text-zinc-200 hover:bg-zinc-800/50 border-b border-zinc-700"
              >
                {expanded ? <ChevronDown className="h-4 w-4" /> : <ChevronRight className="h-4 w-4" />}
                {label}
                <span className="text-zinc-500 font-normal text-sm">({visibleKeys.length} keys)</span>
              </button>

              {expanded && (
                <div className="overflow-x-auto">
                  <table className="w-full text-sm">
                    <thead>
                      <tr className="border-b border-zinc-700 text-zinc-400 text-left">
                        <th className="px-4 py-2 font-medium w-1/3">Key</th>
                        <th className="px-4 py-2 font-medium w-1/3">Value</th>
                        <th className="px-4 py-2 font-medium w-1/4">Type</th>
                      </tr>
                    </thead>
                    <tbody>
                      {visibleKeys.length === 0 ? (
                        <tr>
                          <td colSpan={3} className="px-4 py-3 text-zinc-500">
                            No keys match.
                          </td>
                        </tr>
                      ) : (
                        visibleKeys.map((key, i) => (
                          <tr
                            key={key}
                            className={`border-b border-zinc-800 last:border-b-0 ${
                              i % 2 === 0 ? 'bg-zinc-900/30' : 'bg-zinc-900/10'
                            }`}
                          >
                            <td className="px-4 py-2 font-mono text-zinc-300">{key}</td>
                            <td className="px-4 py-2 font-mono text-zinc-200">
                              {formatValue(key, blackboardState[key])}
                            </td>
                            <td className="px-4 py-2 text-zinc-500">
                              {BLACKBOARD_KEY_TYPES[key] ?? '–'}
                            </td>
                          </tr>
                        ))
                      )}
                    </tbody>
                  </table>
                </div>
              )}
            </div>
          );
        })}
      </div>

      {!isConnected && (
        <p className="text-sm text-zinc-500">Connect ROS to load values from blackboard service.</p>
      )}
    </div>
  );
}
