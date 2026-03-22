import React, { useState, useCallback, useEffect, useMemo } from 'react';
import { Activity, LayoutGrid, GitBranch, BarChart3 } from 'lucide-react';
import { SensorDashboardContainer } from '../SensorDashboard/SensorDashboardContainer';
import { BlackboardRealtime, BlackboardHistory, KeyAnalytics } from '../BlackboardViewer';
import { BTExecutionView, BTPerformanceChart, BTBlackboardLink } from '../BehaviorTreeAnalytics';
import { IncidentReplayer } from './IncidentReplayer';
import { PerformanceAnalyzer } from './PerformanceAnalyzer';
import { CriticalStateStrip } from './CriticalStateStrip';
import { SectionErrorBoundary } from './SectionErrorBoundary';
import { useROS } from '../../../hooks/useROS';
import { useBlackboardState } from '../../../hooks/useBlackboardState';

const TAB_ID = {
  SENSORS: 'sensors',
  BLACKBOARD: 'blackboard',
  BEHAVIOR_TREE: 'bt',
  SYSTEM: 'system'
};

/** Mock BT data for demo (tree). */
const MOCK_BT_NODES = [
  { id: 'root', name: 'Root', status: 'success', durationMs: 120, children: [
    { id: 'nav', name: 'Navigate', status: 'success', durationMs: 85, children: [] },
    { id: 'sense', name: 'Sense', status: 'success', durationMs: 25, children: [] }
  ] }
];

const MOCK_NODE_TIMINGS = [
  { id: 'Navigate', name: 'Navigate', durationMs: 85, count: 10 },
  { id: 'Sense', name: 'Sense', durationMs: 25, count: 10 },
  { id: 'CheckBattery', name: 'CheckBattery', durationMs: 2, count: 10 }
];

/** Derive sensor data and alert count for critical strip (from SensorDashboard or mock). */
function useCriticalStripState(activeTab, isConnected, sensorDataFromChild) {
  return useMemo(() => {
    const sensorData = sensorDataFromChild ?? {};
    const nowMs = Date.now();
    let alertCount = 0;
    Object.entries(sensorData).forEach(([, data]) => {
      const lastUpdate = data?.lastUpdateMs;
      const confidence = data?.confidence ?? 0;
      if (lastUpdate != null && (nowMs - lastUpdate) / 1000 > 2) alertCount += 1;
      if (confidence < 0.5 && confidence > 0) alertCount += 1;
    });
    return { sensorData, alertCount };
  }, [activeTab, isConnected, sensorDataFromChild]);
}

export function DebuggingDashboard() {
  const [activeTab, setActiveTab] = useState(TAB_ID.SENSORS);
  const [syncTime, setSyncTime] = useState(Date.now());
  const [sensorDataForStrip, setSensorDataForStrip] = useState({});
  const { isConnected, ros } = useROS();
  const [blackboardState, , refreshBlackboard] = useBlackboardState(ros, activeTab === TAB_ID.BLACKBOARD);
  const { sensorData: stripSensorData, alertCount } = useCriticalStripState(activeTab, isConnected, sensorDataForStrip);

  const tabs = [
    { id: TAB_ID.SENSORS, label: 'Sensor Health', icon: Activity },
    { id: TAB_ID.BLACKBOARD, label: 'Blackboard', icon: LayoutGrid },
    { id: TAB_ID.BEHAVIOR_TREE, label: 'Behavior Tree', icon: GitBranch },
    { id: TAB_ID.SYSTEM, label: 'System Analytics', icon: BarChart3 }
  ];

  return (
    <div className="flex h-full min-h-0 flex-col bg-zinc-950" role="application" aria-label="Debugging dashboard">
      {/* Glanceable critical state: connection, sensors, alerts */}
      <CriticalStateStrip
        isConnected={isConnected}
        sensorData={stripSensorData}
        alertCount={alertCount}
      />

      <div className="flex items-center gap-2 border-b border-zinc-800 px-4 py-2 shrink-0">
        {tabs.map(({ id, label, icon: Icon }) => (
          <button
            key={id}
            type="button"
            onClick={() => setActiveTab(id)}
            aria-pressed={activeTab === id}
            aria-label={`${label} tab`}
            className={`flex items-center gap-2 rounded px-3 py-1.5 text-sm font-medium transition-colors ${
              activeTab === id ? 'bg-zinc-800 text-zinc-100' : 'text-zinc-400 hover:text-zinc-200'
            }`}
          >
            <Icon className="h-4 w-4" aria-hidden />
            {label}
          </button>
        ))}
        <span className="ml-auto text-xs text-zinc-500 tabular-nums" aria-hidden>
          {new Date(syncTime).toISOString().slice(11, 19)}
        </span>
      </div>

      <div className="flex-1 min-h-0 overflow-y-auto">
        {activeTab === TAB_ID.SENSORS && (
          <SectionErrorBoundary>
            <SensorDashboardContainer onSensorDataChange={setSensorDataForStrip} />
          </SectionErrorBoundary>
        )}

        {activeTab === TAB_ID.BLACKBOARD && (
          <SectionErrorBoundary>
          <div className="max-w-6xl mx-auto space-y-6 p-4">
            <BlackboardRealtime
              blackboardState={blackboardState}
              onRefresh={refreshBlackboard}
              pollIntervalMs={2000}
            />
            <div className="grid gap-4 md:grid-cols-2">
              <section className="rounded-lg border border-zinc-700 bg-zinc-900/50 p-4" aria-labelledby="bb-history-heading">
                <h3 id="bb-history-heading" className="mb-3 text-sm font-medium text-zinc-300">History</h3>
                <BlackboardHistory
                  historySnapshots={[]}
                  selectedTimeIndex={0}
                  onExport={(data) => {
                    try {
                      const blob = new Blob([data]);
                      const url = URL.createObjectURL(blob);
                      const a = document.createElement('a');
                      a.href = url;
                      a.download = 'blackboard_export.json';
                      a.click();
                      URL.revokeObjectURL(url);
                    } catch (_) {}
                  }}
                />
              </section>
              <KeyAnalytics keyStats={{}} anomalies={[]} />
            </div>
          </div>
          </SectionErrorBoundary>
        )}

        {activeTab === TAB_ID.BEHAVIOR_TREE && (
          <SectionErrorBoundary>
          <div className="max-w-6xl mx-auto space-y-6 p-4">
            <div className="grid gap-4 md:grid-cols-2">
              <BTExecutionView nodes={MOCK_BT_NODES} />
              <section aria-labelledby="bt-perf-heading">
                <h3 id="bt-perf-heading" className="mb-2 text-sm font-medium text-zinc-300">Node duration (bottlenecks)</h3>
                <BTPerformanceChart nodeTimings={MOCK_NODE_TIMINGS} />
              </section>
            </div>
            <BTBlackboardLink nodeKeys={[{ nodeId: 'Navigate', reads: ['robot_x', 'robot_y'], writes: ['navigation_status'] }]} />
          </div>
          </SectionErrorBoundary>
        )}

        {activeTab === TAB_ID.SYSTEM && (
          <SectionErrorBoundary>
          <div className="max-w-6xl mx-auto space-y-6 p-4">
            <IncidentReplayer
              timeRange={{ start: syncTime - 60000, end: syncTime }}
              currentTime={syncTime}
              onSeek={setSyncTime}
            />
            <PerformanceAnalyzer
              bottlenecks={['BT node Navigate > 80ms in 10% of ticks']}
              resourceUsage={{ cpu: 0.45, memory: 0.6, network: 0.2 }}
              recommendations={['Reduce sensor_analytics publish rate to 5 Hz', 'Increase blackboard cache TTL to 200ms']}
            />
          </div>
          </SectionErrorBoundary>
        )}
      </div>
    </div>
  );
}
