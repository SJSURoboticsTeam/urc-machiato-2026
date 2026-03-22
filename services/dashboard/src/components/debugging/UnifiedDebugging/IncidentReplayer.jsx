import React, { useState } from 'react';

/**
 * Time-scrubber for incident replay: select time range, step through, show reconstructed state.
 */
export function IncidentReplayer({
  timeRange = { start: 0, end: 0 },
  currentTime = 0,
  onSeek,
  onStep,
  stateAtTime = null
}) {
  const [playing, setPlaying] = useState(false);
  const duration = Math.max(0, timeRange.end - timeRange.start);
  const progress = duration > 0 ? (currentTime - timeRange.start) / duration : 0;

  return (
    <div className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4 space-y-3">
      <h4 className="text-sm font-medium text-zinc-300">Incident replay</h4>
      <div className="flex items-center gap-2">
        <input
          type="range"
          min={timeRange.start}
          max={timeRange.end}
          value={currentTime}
          onChange={(e) => onSeek?.(Number(e.target.value))}
          className="flex-1 accent-zinc-500"
        />
        <span className="text-xs text-zinc-500 w-24">
          {(progress * 100).toFixed(0)}%
        </span>
      </div>
      <div className="flex gap-2">
        <button
          type="button"
          onClick={() => onStep?.(-1)}
          className="rounded border border-zinc-600 bg-zinc-800 px-2 py-1 text-xs text-zinc-200 hover:bg-zinc-700"
        >
          Step back
        </button>
        <button
          type="button"
          onClick={() => setPlaying((p) => !p)}
          className="rounded border border-zinc-600 bg-zinc-800 px-2 py-1 text-xs text-zinc-200 hover:bg-zinc-700"
        >
          {playing ? 'Pause' : 'Play'}
        </button>
        <button
          type="button"
          onClick={() => onStep?.(1)}
          className="rounded border border-zinc-600 bg-zinc-800 px-2 py-1 text-xs text-zinc-200 hover:bg-zinc-700"
        >
          Step fwd
        </button>
      </div>
      {stateAtTime && (
        <div className="text-xs text-zinc-500">
          State at {new Date(currentTime).toISOString()}
        </div>
      )}
    </div>
  );
}
