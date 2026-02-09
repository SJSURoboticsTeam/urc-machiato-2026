import React, { memo } from 'react';

const LEGEND_ITEMS = [
  { color: 'bg-green-500', label: 'Generator (Sensors)' },
  { color: 'bg-blue-500', label: 'Processor' },
  { color: 'bg-purple-500', label: 'Controller (Commands)' },
  { color: 'bg-orange-500', label: 'Actuator' },
  { color: 'bg-cyan-500', label: 'CAN Bus' }
];

/**
 * Legend for node types. Memoized.
 */
export const NetworkLegend = memo(function NetworkLegend() {
  return (
    <div className="absolute top-4 left-4 bg-zinc-800/90 rounded-lg p-3 border border-zinc-600 backdrop-blur-sm">
      <h4 className="text-sm font-medium text-zinc-200 mb-2">Node Types</h4>
      <div className="space-y-1 text-xs">
        {LEGEND_ITEMS.map(({ color, label }) => (
          <div key={label} className="flex items-center gap-2">
            <div className={`w-3 h-3 ${color} rounded-full`} />
            <span className="text-zinc-300">{label}</span>
          </div>
        ))}
      </div>
    </div>
  );
});
