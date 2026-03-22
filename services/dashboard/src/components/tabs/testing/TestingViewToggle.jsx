import React, { memo } from 'react';
import { BarChart3, Shield } from 'lucide-react';

export const TestingViewToggle = memo(function TestingViewToggle({ viewMode, setViewMode }) {
  return (
    <div className="bg-zinc-900 border-b border-zinc-800 p-3">
      <div className="flex items-center justify-between">
        <div className="flex items-center gap-2">
          <button
            onClick={() => setViewMode('integrated')}
            className={`px-4 py-2 rounded-md text-sm font-medium transition-colors ${
              viewMode === 'integrated'
                ? 'bg-cyan-600 text-white'
                : 'bg-zinc-800 text-zinc-400 hover:bg-zinc-700'
            }`}
          >
            <BarChart3 className="w-4 h-4 inline mr-2" />
            Integrated Dashboard
          </button>
          <button
            onClick={() => setViewMode('legacy')}
            className={`px-4 py-2 rounded-md text-sm font-medium transition-colors ${
              viewMode === 'legacy'
                ? 'bg-cyan-600 text-white'
                : 'bg-zinc-800 text-zinc-400 hover:bg-zinc-700'
            }`}
          >
            <Shield className="w-4 h-4 inline mr-2" />
            Component View
          </button>
        </div>
        <div className="text-xs text-zinc-400">
          {viewMode === 'integrated'
            ? 'Real-time communication flow & state visualization'
            : 'Traditional component-based testing'}
        </div>
      </div>
    </div>
  );
});
