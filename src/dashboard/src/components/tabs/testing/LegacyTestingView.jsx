import React, { useState, useEffect } from 'react';
import { useStateMachineContext } from '../../../context/StateMachineContext';
import { useTelemetryContext } from '../../../context/TelemetryContext';
import { useUIContext } from '../../../context/UIContext';
import { AlertTriangle } from 'lucide-react';
import { SystemState } from '../../../config/stateDefinitions';
import { getComponentDefinitions } from './componentDefinitions';
import { ComponentFilterTabs } from './ComponentFilterTabs';
import { ComponentTestCard } from './ComponentTestCard';

export function LegacyTestingView() {
  const { currentState } = useStateMachineContext();
  const { systemStatus } = useTelemetryContext();
  const { setRunningTests } = useUIContext();
  const [activeFilter, setActiveFilter] = useState('all');

  const components = getComponentDefinitions(systemStatus);
  const filteredComponents =
    activeFilter === 'all' ? components : components.filter((c) => c.id === activeFilter);

  useEffect(() => {
    const running = components.reduce((sum, comp) => sum + comp.running, 0);
    setRunningTests(running);
  }, [components, setRunningTests]);

  const showMissionWarning = currentState === SystemState.AUTONOMOUS;

  return (
    <div className="p-4 space-y-4">
      <ComponentFilterTabs
        components={components}
        activeFilter={activeFilter}
        setActiveFilter={setActiveFilter}
      />

      {showMissionWarning && (
        <div className="bg-yellow-900/20 border border-yellow-800 rounded p-3">
          <div className="flex items-center gap-2 text-sm text-yellow-400">
            <AlertTriangle className="w-4 h-4" />
            <span>Mission active - some tests unavailable</span>
          </div>
        </div>
      )}

      <div className="space-y-4">
        {filteredComponents.map((component) => (
          <ComponentTestCard key={component.id} component={component} />
        ))}
      </div>
    </div>
  );
}
