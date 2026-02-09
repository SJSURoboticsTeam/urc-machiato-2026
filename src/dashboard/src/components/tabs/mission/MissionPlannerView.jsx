import React, { memo } from 'react';
import { Plus, FileText, Play } from 'lucide-react';
import { MissionTemplateGrid } from './MissionTemplateGrid';

/**
 * Mission planner form and template grid (IDLE state). Memoized.
 */
export const MissionPlannerView = memo(function MissionPlannerView({
  missionType,
  setMissionType,
  priority,
  setPriority,
  missionTemplates,
  onStartMission
}) {
  return (
    <div className="p-4 space-y-4">
      <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
        <h2 className="text-base font-semibold text-zinc-200 mb-4">Mission Planner</h2>

        <div className="grid grid-cols-3 gap-4 mb-4">
          <div>
            <label className="block text-xs text-zinc-400 mb-1" htmlFor="mission-type">
              Type
            </label>
            <select
              id="mission-type"
              value={missionType}
              onChange={(e) => setMissionType(e.target.value)}
              className="w-full px-3 py-2 bg-zinc-800 border border-zinc-700 rounded text-sm text-zinc-200"
            >
              {missionTemplates.map((t) => (
                <option key={t.id} value={t.id}>
                  {t.name}
                </option>
              ))}
            </select>
          </div>

          <div>
            <label className="block text-xs text-zinc-400 mb-1" htmlFor="mission-priority">
              Priority
            </label>
            <select
              id="mission-priority"
              value={priority}
              onChange={(e) => setPriority(e.target.value)}
              className="w-full px-3 py-2 bg-zinc-800 border border-zinc-700 rounded text-sm text-zinc-200"
            >
              <option value="normal">Normal</option>
              <option value="high">High</option>
              <option value="urgent">Urgent</option>
            </select>
          </div>

          <div className="flex items-end">
            <button
              type="button"
              onClick={() => onStartMission(missionType, priority, missionTemplates)}
              className="w-full px-4 py-2 bg-cyan-600 hover:bg-cyan-700 text-white text-sm rounded flex items-center justify-center gap-2"
            >
              <Play className="w-4 h-4" />
              Create
            </button>
          </div>
        </div>

        <div className="grid grid-cols-3 gap-4 text-xs text-zinc-400 mb-4">
          <div>Waypoints: (0)</div>
          <div>Parameters: (default)</div>
          <div className="text-green-400">Validation: ✓</div>
        </div>

        <div className="flex items-center gap-2">
          <button
            type="button"
            className="px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded flex items-center gap-2"
          >
            <Plus className="w-3 h-3" />
            Add Waypoint
          </button>
          <button
            type="button"
            className="px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded flex items-center gap-2"
          >
            <FileText className="w-3 h-3" />
            Load Template
          </button>
          <button
            type="button"
            className="px-3 py-1.5 bg-zinc-800 hover:bg-zinc-700 text-zinc-200 text-xs rounded"
          >
            Clear
          </button>
        </div>
      </div>

      <MissionTemplateGrid
        templates={missionTemplates}
        selectedId={missionType}
        onSelect={setMissionType}
      />
    </div>
  );
});
