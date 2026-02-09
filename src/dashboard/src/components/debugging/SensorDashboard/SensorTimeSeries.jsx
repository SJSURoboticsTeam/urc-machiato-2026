import React, { useMemo } from 'react';
import {
  LineChart,
  Line,
  XAxis,
  YAxis,
  CartesianGrid,
  Tooltip,
  ResponsiveContainer,
  Legend,
  ReferenceLine
} from 'recharts';
import { SENSOR_IDS, SENSOR_LABELS, TIME_WINDOWS } from '../constants';

/**
 * Time-series chart of sensor data (e.g. confidence or value) over configurable window.
 * dataBySensor: { [sensorId]: [{ t: number, value: number }, ...] }
 * or timeSeriesData: [{ time: string, imu: number, gps: number, ... }] for multi-line.
 */
export function SensorTimeSeries({
  timeSeriesData = [],
  dataBySensor = {},
  timeWindowSec = 60,
  onTimeWindowChange,
  valueLabel = 'Confidence',
  selectedSensors = null
}) {
  const sensorsToShow = selectedSensors ?? SENSOR_IDS;
  const windowOptions = TIME_WINDOWS;

  const chartData = useMemo(() => {
    if (timeSeriesData.length > 0) {
      return timeSeriesData.slice(-Math.ceil((timeWindowSec / 5) || 1));
    }
    const byTime = {};
    sensorsToShow.forEach((id) => {
      (dataBySensor[id] || []).forEach(({ t, value }) => {
        const key = new Date(t).toISOString().slice(11, 19);
        if (!byTime[key]) byTime[key] = { time: key };
        byTime[key][id] = value;
      });
    });
    return Object.values(byTime).sort((a, b) => a.time.localeCompare(b.time)).slice(-50);
  }, [timeSeriesData, dataBySensor, timeWindowSec, sensorsToShow]);

  const colors = [
    '#22c55e',
    '#3b82f6',
    '#f59e0b',
    '#ef4444',
    '#8b5cf6',
    '#ec4899',
    '#06b6d4',
    '#84cc16',
    '#f97316'
  ];

  return (
    <div className="space-y-2">
      <div className="flex items-center justify-between">
        <span className="text-sm font-medium text-zinc-300">Sensor trends</span>
        <select
          value={timeWindowSec}
          onChange={(e) => onTimeWindowChange?.(Number(e.target.value))}
          className="rounded border border-zinc-600 bg-zinc-800 px-2 py-1 text-xs text-zinc-200"
        >
          {windowOptions.map((sec) => (
            <option key={sec} value={sec}>
              {sec === 30 ? '30s' : sec === 60 ? '1m' : `${sec / 60}m`}
            </option>
          ))}
        </select>
      </div>
      <div className="h-64 min-h-[200px] w-full">
        <ResponsiveContainer width="100%" height="100%" minHeight={200}>
          <LineChart data={chartData} margin={{ top: 5, right: 10, left: 0, bottom: 5 }}>
            <CartesianGrid strokeDasharray="3 3" stroke="#3f3f46" />
            <XAxis dataKey="time" tick={{ fontSize: 10 }} stroke="#71717a" />
            <YAxis domain={[0, 1]} tick={{ fontSize: 10 }} stroke="#71717a" />
            <Tooltip
              contentStyle={{ backgroundColor: '#27272a', border: '1px solid #3f3f46' }}
              labelStyle={{ color: '#a1a1aa' }}
            />
            <Legend />
            <ReferenceLine y={0.8} stroke="#22c55e" strokeDasharray="2 2" />
            <ReferenceLine y={0.5} stroke="#f59e0b" strokeDasharray="2 2" />
            {sensorsToShow.map((id, i) => (
              <Line
                key={id}
                type="monotone"
                dataKey={id}
                name={SENSOR_LABELS[id] || id}
                stroke={colors[i % colors.length]}
                strokeWidth={1.5}
                dot={false}
                isAnimationActive={false}
              />
            ))}
          </LineChart>
        </ResponsiveContainer>
      </div>
      <p className="text-xs text-zinc-500">{valueLabel} over last {timeWindowSec}s</p>
    </div>
  );
}
