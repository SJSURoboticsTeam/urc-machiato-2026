import React, { memo } from 'react';

/**
 * Grid of mission template cards. Memoized for stable templates/onSelect.
 */
export const MissionTemplateGrid = memo(function MissionTemplateGrid({
  templates,
  selectedId,
  onSelect
}) {
  return (
    <div>
      <h3 className="text-sm font-semibold text-zinc-200 mb-3">Available Missions</h3>
      <div className="grid grid-cols-4 gap-4">
        {templates.map((template) => (
          <div
            key={template.id}
            role="button"
            tabIndex={0}
            className="bg-zinc-900 border border-zinc-800 rounded p-4 cursor-pointer hover:border-cyan-700 transition-colors"
            onClick={() => onSelect(template.id)}
            onKeyDown={(e) => {
              if (e.key === 'Enter' || e.key === ' ') onSelect(template.id);
            }}
          >
            <div className="text-2xl mb-2">{template.icon}</div>
            <div className="text-sm font-semibold text-zinc-200 mb-1">{template.name}</div>
            <div className="text-xs text-zinc-400 mb-3">{template.description}</div>
            <button
              type="button"
              className="w-full px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded"
              onClick={(e) => {
                e.stopPropagation();
                onSelect(template.id);
              }}
            >
              Select
            </button>
          </div>
        ))}
      </div>
    </div>
  );
});
