import React from 'react';
import { useStateMachineContext } from '../../context/StateMachineContext';
import { useTelemetry } from '../../hooks/useTelemetry';
import { useAlerts } from '../../hooks/useAlerts';
import { useMissionControl } from '../../hooks/useMissionControl';
import { SystemStatusCard } from '../cards/SystemStatusCard';
import { TelemetryCard } from '../cards/TelemetryCard';
import { SystemState } from '../../config/stateDefinitions';
import { getContextualSystems } from './overview';
import { OverviewIdleView, OverviewAutonomousView, OverviewSafetyView } from './overview';

/**
 * OverviewTab – context-aware dashboard that adapts to system state.
 * Delegates to OverviewIdleView, OverviewAutonomousView, or OverviewSafetyView.
 */
export const OverviewTab = () => {
  const { currentState, requestStateTransition } = useStateMachineContext();
  const { telemetry, systemStatus } = useTelemetry();
  const { alerts } = useAlerts();
  const { activeMission } = useMissionControl();

  if (currentState === SystemState.IDLE || currentState === SystemState.BOOT) {
    return (
      <OverviewIdleView
        currentState={currentState}
        systemStatus={systemStatus}
        telemetry={telemetry}
        requestStateTransition={requestStateTransition}
      />
    );
  }

  if (currentState === SystemState.AUTONOMOUS) {
    return (
      <OverviewAutonomousView
        currentState={currentState}
        systemStatus={systemStatus}
        telemetry={telemetry}
        activeMission={activeMission}
        alerts={alerts}
      />
    );
  }

  if (currentState === SystemState.SAFETY) {
    return (
      <OverviewSafetyView
        currentState={currentState}
        systemStatus={systemStatus}
        requestStateTransition={requestStateTransition}
      />
    );
  }

  const systems = getContextualSystems(currentState, systemStatus);
  return (
    <div className="p-4 space-y-4">
      <div className="grid grid-cols-3 gap-4">
        <SystemStatusCard systems={systems} />
        <TelemetryCard telemetry={telemetry} />
        <div className="bg-zinc-900 border border-zinc-800 rounded p-4">
          <h3 className="text-sm font-semibold text-zinc-200 mb-3">Current State</h3>
          <div className="text-lg font-bold text-zinc-200">{currentState}</div>
        </div>
      </div>
    </div>
  );
};
