import React, { useState, useEffect, useCallback } from 'react';
import { Search, ChevronDown, ChevronRight } from 'lucide-react';
import { useROS } from '../../../hooks/useROS';
import { BLACKBOARD_SERVICES, SERVICE_TYPES } from '../../../config/rosTopics';
import { BLACKBOARD_DOMAINS, BLACKBOARD_KEY_TYPES } from '../constants';

/**
 * Real-time blackboard: categorized keys, type-aware display, search/filter, change highlighting.
 */
export function BlackboardRealtime({ blackboardState = {}, onRefresh, pollIntervalMs = 500 }) {
  const [search, setSearch] = useState('');
  const [domainFilter, setDomainFilter] = useState('all');
  const [expandedDomains, setExpandedDomains] = useState(() => Object.keys(BLACKBOARD_DOMAINS).reduce((a, k) => ({ ...a, [k]: true }), {}));
  const [loading, setLoading] = useState(false);
  const { isConnected, ros } = useROS();

  const toggleDomain = (key) => {
    setExpandedDomains((prev) => ({ ...prev, [key]: !prev[key] }));
  };

  const domains = Object.entries(BLACKBOARD_DOMAINS);
  const filteredDomains = domainFilter === 'all'
    ? domains
    : domains.filter(([k]) => k === domainFilter);

  const matchesSearch = (keyName, value) => {
    if (!search.trim()) return true;
    const s = search.toLowerCase();
    return keyName.toLowerCase().includes(s) || String(value).toLowerCase().includes(s);
  };

  const formatValue = (key, value) => {
    if (value === undefined || value === null) return '–';
    const type = BLACKBOARD_KEY_TYPES[key];
    if (type === 'double' && key.includes('latitude')) return `lat ${Number(value).toFixed(6)}`;
    if (type === 'double' && key.includes('longitude')) return `lon ${Number(value).toFixed(6)}`;
    if (type === 'bool') return value ? 'true' : 'false';
    if (type === 'double') return Number(value).toFixed(4);
    return String(value);
  };

  return (
    <div className="space-y-4">
      <div className="flex flex-wrap items-center gap-2">
        <div className="flex flex-1 items-center gap-2 rounded border border-zinc-600 bg-zinc-800 px-2 py-1 min-w-[200px]">
          <Search className="h-4 w-4 text-zinc-500" />
          <input
            type="text"
            placeholder="Search keys..."
            value={search}
            onChange={(e) => setSearch(e.target.value)}
            className="flex-1 bg-transparent text-sm text-zinc-200 placeholder-zinc-500 focus:outline-none"
          />
        </div>
        <select
          value={domainFilter}
          onChange={(e) => setDomainFilter(e.target.value)}
          className="rounded border border-zinc-600 bg-zinc-800 px-2 py-1 text-sm text-zinc-200"
        >
          <option value="all">All domains</option>
          {domains.map(([key, { label }]) => (
            <option key={key} value={key}>{label}</option>
          ))}
        </select>
        {onRefresh && (
          <button
            type="button"
            onClick={() => onRefresh()}
            disabled={loading || !isConnected}
            className="rounded border border-zinc-600 bg-zinc-800 px-3 py-1 text-sm text-zinc-200 hover:bg-zinc-700 disabled:opacity-50"
          >
            {loading ? '...' : 'Refresh'}
          </button>
        )}
      </div>

      <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 overflow-hidden">
        {filteredDomains.map(([domainKey, { label, keys }]) => {
          const expanded = expandedDomains[domainKey];
          const visibleKeys = keys.filter((k) => matchesSearch(k, blackboardState[k]));

          return (
            <div key={domainKey} className="border-b border-zinc-800 last:border-b-0">
              <button
                type="button"
                onClick={() => toggleDomain(domainKey)}
                className="flex w-full items-center gap-2 px-3 py-2 text-left text-sm font-medium text-zinc-300 hover:bg-zinc-800/50"
              >
                {expanded ? <ChevronDown className="h-4 w-4" /> : <ChevronRight className="h-4 w-4" />}
                {label} ({visibleKeys.length})
              </button>
              {expanded && (
                <div className="bg-zinc-900/30 px-3 pb-2">
                  {visibleKeys.length === 0 ? (
                    <p className="py-2 text-xs text-zinc-500">No keys match.</p>
                  ) : (
                    <ul className="space-y-1">
                      {visibleKeys.map((key) => (
                        <li key={key} className="flex items-center justify-between gap-4 rounded py-1.5 px-2 font-mono text-xs hover:bg-zinc-800/50">
                          <span className="text-zinc-400 truncate">{key}</span>
                          <span className="text-zinc-200 shrink-0">{formatValue(key, blackboardState[key])}</span>
                        </li>
                      ))}
                    </ul>
                  )}
                </div>
              )}
            </div>
          );
        })}
      </div>
    </div>
  );
}
