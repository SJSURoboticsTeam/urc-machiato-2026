import React, { useState, useCallback } from 'react';
import { useStateMachineContext } from '../../context/StateMachineContext';
import { useUIContext } from '../../context/UIContext';
import {
  DebugLogsPanel,
  DebugStatePanel,
  DebugTopicsPanel,
  DebugPerformancePanel,
  DebugNetworkPanel
} from './debug';

const PANELS = [
  { id: 'logs', label: 'Logs' },
  { id: 'state', label: 'State' },
  { id: 'topics', label: 'Topics' },
  { id: 'performance', label: 'Performance' },
  { id: 'network', label: 'Network' }
];

const INITIAL_LOGS = [
  { time: '23:14:35', level: 'INFO', component: 'StateMachine', message: 'BOOT → IDLE' },
  { time: '23:14:36', level: 'WARN', component: 'Navigation', message: 'GPS HDOP: 2.5' },
  { time: '23:14:37', level: 'ERROR', component: 'Vision', message: 'Camera timeout' },
  { time: '23:14:38', level: 'DEBUG', component: 'CAN', message: 'Motor cmd sent' },
  { time: '23:14:39', level: 'INFO', component: 'Safety', message: 'All checks passed' }
];

const TOPICS = [
  { name: '/mission/cmds', rate: 0.5, size: '256B' },
  { name: '/odom', rate: 10, size: '1.2K' },
  { name: '/can/sensor', rate: 20, size: '512B' },
  { name: '/safety/status', rate: 1, size: '128B' },
  { name: '/vision/camera', rate: 0, size: '0B' }
];

/**
 * DebugTab: multi-panel debug view (logs, state, topics, performance, network).
 * Uses centralized status colors and virtualized log list for 60fps.
 */
export const DebugTab = () => {
  const { currentState, requestStateTransition, isTransitioning } = useStateMachineContext();
  const { alerts, errorCount } = useUIContext();

  const [activePanel, setActivePanel] = useState('logs');
  const [logLevel, setLogLevel] = useState('all');
  const [logComponent, setLogComponent] = useState('all');
  const [logs, setLogs] = useState(INITIAL_LOGS);

  const filteredLogs = logs.filter((log) => {
    if (logLevel !== 'all' && log.level !== logLevel) return false;
    if (logComponent !== 'all' && log.component !== logComponent) return false;
    return true;
  });

  const handleClearLogs = useCallback(() => setLogs([]), []);

  return (
    <div className="p-4 space-y-4">
      <div className="flex items-center gap-2 border-b border-zinc-800 pb-2">
        {PANELS.map((panel) => (
          <button
            key={panel.id}
            type="button"
            onClick={() => setActivePanel(panel.id)}
            className={`px-3 py-1 text-xs rounded transition-colors ${
              activePanel === panel.id
                ? 'bg-zinc-800 text-zinc-200'
                : 'text-zinc-400 hover:text-zinc-200'
            }`}
          >
            {panel.label}
          </button>
        ))}
      </div>

      {activePanel === 'logs' && (
        <DebugLogsPanel
          currentState={currentState}
          logs={logs}
          filteredLogs={filteredLogs}
          logLevel={logLevel}
          setLogLevel={setLogLevel}
          logComponent={logComponent}
          setLogComponent={setLogComponent}
          onClear={handleClearLogs}
        />
      )}

      {activePanel === 'state' && (
        <DebugStatePanel
          currentState={currentState}
          requestStateTransition={requestStateTransition}
          isTransitioning={isTransitioning}
          topics={TOPICS}
        />
      )}

      {activePanel === 'topics' && <DebugTopicsPanel topics={TOPICS} />}

      {activePanel === 'performance' && <DebugPerformancePanel alerts={alerts} />}

      {activePanel === 'network' && <DebugNetworkPanel errorCount={errorCount} />}
    </div>
  );
};
