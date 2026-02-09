import React, { memo } from 'react';
import { FixedSizeList } from 'react-window';

/**
 * Generic virtualized list. Use for long lists (messages, logs, etc.) to keep 60fps.
 * @param {Object} props
 * @param {Array} props.items - List data
 * @param {number} [props.height=192]
 * @param {number} [props.itemHeight=40]
 * @param {function(number, *): import('react').ReactNode} props.children - Render prop: (index, item) => node
 * @param {string} [props.emptyMessage='No items']
 * @param {string} [props.className]
 */
export const VirtualizedList = memo(function VirtualizedList({
  items,
  height = 192,
  itemHeight = 40,
  children: renderRow,
  emptyMessage = 'No items',
  className = ''
}) {
  if (!items || items.length === 0) {
    return (
      <div
        className={`text-xs text-zinc-500 flex items-center justify-center ${className}`}
        style={{ height }}
      >
        {emptyMessage}
      </div>
    );
  }

  function Row({ index, style }) {
    return <div style={style}>{renderRow(index, items[index])}</div>;
  }

  return (
    <FixedSizeList
      height={height}
      itemCount={items.length}
      itemSize={itemHeight}
      width="100%"
      className={className}
      overscanCount={5}
    >
      {Row}
    </FixedSizeList>
  );
});
