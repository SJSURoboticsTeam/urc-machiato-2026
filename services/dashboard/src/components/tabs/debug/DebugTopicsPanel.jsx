import React, { memo } from 'react';
import { getStatusTextColor } from '../../../utils/statusUtils';

/**
 * Topics table panel. Memoized for stable props.
 * @param {Object} props
 * @param {Array<{ name: string, rate: number, size: string }>} props.topics
 */
export const DebugTopicsPanel = memo(function DebugTopicsPanel({ topics }) {
  return (
    <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
      <h3 className="text-sm font-semibold text-zinc-200 mb-4">ROS2 Topics</h3>
      <div className="space-y-2">
        <div className="grid grid-cols-4 gap-2 text-xs text-zinc-400 border-b border-zinc-800 pb-2">
          <div>Topic</div>
          <div>Rate</div>
          <div>Size</div>
          <div>Status</div>
        </div>
        {topics.map((topic, idx) => (
          <div
            key={topic.name || idx}
            className="grid grid-cols-4 gap-2 text-xs py-1 border-b border-zinc-800/50"
          >
            <div className="text-zinc-200 font-mono">{topic.name}</div>
            <div className="text-zinc-400">{topic.rate} Hz</div>
            <div className="text-zinc-400">{topic.size}</div>
            <div className={getStatusTextColor(topic.rate > 0 ? 'connected' : 'disconnected')}>
              {topic.rate > 0 ? 'Active' : 'Inactive'}
            </div>
          </div>
        ))}
      </div>
    </div>
  );
});
