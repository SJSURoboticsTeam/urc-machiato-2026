import React, { memo } from 'react';
import { FixedSizeList as List } from 'react-window';
import { getLogLevelColor } from '../../utils/statusUtils';

/**
 * Virtualized list for log entries. Keeps 60fps with large log buffers.
 * @param {Object} props
 * @param {Array<{ time: string, level: string, component: string, message: string }>} props.logs
 * @param {number} [props.height=384]
 * @param {number} [props.itemHeight=28]
 * @param {string} [props.className]
 */
export const VirtualizedLogList = memo(function VirtualizedLogList({
  logs,
  height = 384,
  itemHeight = 28,
  className = ''
}) {
  const Row = ({ index, style }) => {
    const log = logs[index];
    if (!log) return null;
    const levelColor = getLogLevelColor(log.level);
    return (
      <div style={style} className="flex items-start gap-3 py-1 font-mono text-xs">
        <span className="text-zinc-500 shrink-0">{log.time}</span>
        <span className={`font-semibold shrink-0 ${levelColor}`}>[{log.level}]</span>
        <span className="text-zinc-400 shrink-0">{log.component}:</span>
        <span className="text-zinc-200 flex-1 truncate">{log.message}</span>
      </div>
    );
  };

  if (!logs || logs.length === 0) {
    return (
      <div className={`text-xs text-zinc-500 ${className}`} style={{ height }}>
        No logs
      </div>
    );
  }

  return (
    <List
      height={height}
      itemCount={logs.length}
      itemSize={itemHeight}
      width="100%"
      className={className}
      overscanCount={5}
    >
      {Row}
    </List>
  );
});
