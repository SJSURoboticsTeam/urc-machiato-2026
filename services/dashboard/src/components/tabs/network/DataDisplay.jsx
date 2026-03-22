import React, { memo } from 'react';

function formatValue(value, compact) {
  if (typeof value === 'number') {
    return compact ? value.toFixed(2) : value.toFixed(4);
  }
  return String(value);
}

function RenderValue({ fieldKey, value, depth, compact }) {
  if (typeof value === 'object' && value !== null && depth < (compact ? 2 : 3)) {
    return (
      <div className={depth > 0 ? 'ml-3' : ''}>
        {fieldKey && (
          <div className="text-xs text-zinc-400 font-medium mb-1">{fieldKey}:</div>
        )}
        <div className="space-y-0.5">
          {Object.entries(value)
            .slice(0, compact ? 4 : undefined)
            .map(([subK, v]) => (
              <RenderValue
                key={subK}
                fieldKey={subK}
                value={v}
                depth={depth + 1}
                compact={compact}
              />
            ))}
        </div>
      </div>
    );
  }

  return (
    <div className="flex justify-between text-xs py-0.5 px-2 bg-zinc-800/30 rounded">
      <span className="text-zinc-400">{fieldKey}:</span>
      <span className="text-green-400 font-mono">{formatValue(value, compact)}</span>
    </div>
  );
}

/**
 * Recursive key-value display for node data. Memoized.
 */
export const DataDisplay = memo(function DataDisplay({ data, compact = false }) {
  if (!data || typeof data !== 'object') {
    return (
      <span className="text-green-400 font-mono text-sm">{String(data)}</span>
    );
  }

  return (
    <div className="space-y-1 max-h-40 overflow-y-auto">
      {Object.entries(data)
        .slice(0, compact ? 6 : undefined)
        .map(([key, value]) => (
          <RenderValue
            key={key}
            fieldKey={key}
            value={value}
            depth={0}
            compact={compact}
          />
        ))}
    </div>
  );
});
