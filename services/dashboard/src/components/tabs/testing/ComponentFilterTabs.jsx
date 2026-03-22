import React, { memo } from 'react';

export const ComponentFilterTabs = memo(function ComponentFilterTabs({
  components,
  activeFilter,
  setActiveFilter
}) {
  return (
    <div className="flex items-center gap-2 border-b border-zinc-800 pb-2">
      <button
        onClick={() => setActiveFilter('all')}
        className={`px-3 py-1 text-xs rounded transition-colors ${
          activeFilter === 'all' ? 'bg-zinc-800 text-zinc-200' : 'text-zinc-400 hover:text-zinc-200'
        }`}
      >
        All
      </button>
      {components.map((comp) => (
        <button
          key={comp.id}
          onClick={() => setActiveFilter(comp.id)}
          className={`px-3 py-1 text-xs rounded transition-colors ${
            activeFilter === comp.id ? 'bg-zinc-800 text-zinc-200' : 'text-zinc-400 hover:text-zinc-200'
          }`}
        >
          {comp.name}
        </button>
      ))}
    </div>
  );
});
