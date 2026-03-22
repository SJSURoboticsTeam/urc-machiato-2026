import React from 'react';
import { BarChart, Bar, XAxis, YAxis, CartesianGrid, Tooltip, ResponsiveContainer } from 'recharts';

/**
 * Bar chart of execution duration per BT node (bottleneck view).
 */
export function BTPerformanceChart({ nodeTimings = [] }) {
  const data = nodeTimings
    .map((n) => ({ name: (n.name || n.id || '').slice(0, 12), ms: n.durationMs ?? 0, count: n.count ?? 1 }))
    .sort((a, b) => b.ms - a.ms)
    .slice(0, 15);

  return (
    <div className="h-64 min-h-[200px] w-full">
      <ResponsiveContainer width="100%" height="100%" minHeight={200}>
        <BarChart data={data} layout="vertical" margin={{ top: 5, right: 20, left: 60, bottom: 5 }}>
          <CartesianGrid strokeDasharray="3 3" stroke="#3f3f46" />
          <XAxis type="number" unit=" ms" tick={{ fontSize: 10 }} stroke="#71717a" />
          <YAxis type="category" dataKey="name" width={55} tick={{ fontSize: 10 }} stroke="#71717a" />
          <Tooltip
            contentStyle={{ backgroundColor: '#27272a', border: '1px solid #3f3f46' }}
            formatter={(value) => [`${value} ms`, 'Duration']}
          />
          <Bar dataKey="ms" fill="#3b82f6" radius={[0, 2, 2, 0]} isAnimationActive={false} />
        </BarChart>
      </ResponsiveContainer>
    </div>
  );
}
